/**
  ******************************************************************************
  * @file    app_reset_ble.c
  * @brief   应用层-复位/运行状态的文本查询协议 (KT6368A 串口)
  *          命令: RESET,READ 复位记录；STATE,READ RTC/校时/天气快照；HEART,READ 心率故障；
  *          RESET,HISTORY 电池日志里的历次复位记录。均以 CR/LF 结束。
  *          数据归属: 复位记录读自 app_reset（上一启动快照，一次请求只取一份）；RTC状态
  *          与时间会读RTC硬件，天气/心率/校时取自各自模块RAM快照；RESET,HISTORY的历次
  *          复位条目由 app_battery_log 从内置Flash sector7读出。本文件不碰I2C，只读被测
  *          模块，不清除其解析状态，也不修改其业务状态。
  *          发送节奏: 一拍最多一行，除首行外每行之间至少隔100ms，避免小缓冲模块溢出；
  *          每次发送都是异步启动，必须由本模块自己收回结果后再准备下一行。
  ******************************************************************************
***/
#include "app_reset_ble.h"
#include "app_reset.h"
#include "app_battery_log.h"
#include "app_clock.h"
#include "app_weather.h"
#include "app_heart.h"
#include "watch_rtc.h"
#include "kt6368a.h"
#include <stdio.h>
#include <string.h>

enum
{
    APP_REPORT_NONE,
    APP_REPORT_RESET,
    APP_REPORT_STATE,
    APP_REPORT_HISTORY,
    APP_REPORT_HEART
};

static struct
{
    app_reset_info_struct info;
    watch_rtc_status_struct rtc;
    app_weather_struct weather;
    app_clock_result clock_result;
    uint32_t snapshot_ms, clock_frames;
    uint32_t slot, limit, session, exported;
    char line[16];
    char tx[112];
    uint32_t rx_ms, tx_ms;
    uint16_t length, tx_length;
    uint8_t overflow, report, index, pending, history_stage;
} s_ble;

/* 清解析器与报告状态，让下一次查询从第一行重新开始；在途异步TX的长度与缓冲保留，
 * 等service收回结果后再统一清空。 */
void app_reset_ble_reset(void)
{
    s_ble.length = 0; /* 丢弃未成行的接收残帧。 */
    s_ble.overflow = 0; /* 清掉超长/非打印字符标记。 */
    s_ble.report = 0; /* 撤销尚未完成的报告类型(APP_REPORT_NONE)。 */
    s_ble.index = 0; /* 报告从第一行重新开始。 */
    s_ble.history_stage = 0; /* HISTORY流程回到取状态阶段。 */
    /* UART不复制异步缓冲；复位解析器或蓝牙断电均不能改写在途数据。 */
    if (s_ble.pending == 0)
    {
        s_ble.tx_length = 0; /* 无在途发送时才清待发文本长度。 */
    }
}

/* 按行解析命令：行内只接受可打印ASCII，超长或非打印字节整行作废。
 * 距上一字节达到1秒视为新的一行，丢弃半行；只有当前无报告、无在途TX时才接收新命令，
 * 避免一次查询被后续命令打断。 */
void app_reset_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    uint16_t i;
    /* 距上次收到字节已满1秒，视为新行：丢掉半行和溢出标记。 */
    if ((uint32_t)(now_ms - s_ble.rx_ms) >= 1000)
    {
        s_ble.length = 0;
        s_ble.overflow = 0;
    }
    s_ble.rx_ms = now_ms; /* 记录本批字节的接收时刻。 */
    /* 逐字节处理本次收到的数据。 */
    for (i = 0; i < size; ++i)
    {
        /* CR或LF表示一行结束。 */
        if (data[i] == '\r' || data[i] == '\n')
        {
            s_ble.line[s_ble.length] = '\0'; /* 补字符串结束符，形成完整行。 */
            /* 整行无溢出、当前无报告、且没有在途发送时才接受新命令。 */
            if (s_ble.overflow == 0 && s_ble.report == APP_REPORT_NONE && s_ble.pending == 0)
            {
                /* RESET,READ：报告上一启动的复位记录，从第0行开始。 */
                if (strcmp(s_ble.line, "RESET,READ") == 0)
                {
                    s_ble.report = APP_REPORT_RESET;
                    s_ble.index = 0;
                }
                /* STATE,READ：报告RTC/校时/天气快照，从第0行开始。 */
                else if (strcmp(s_ble.line, "STATE,READ") == 0)
                {
                    s_ble.report = APP_REPORT_STATE;
                    s_ble.index = 0;
                }
                /* HEART,READ：报告心率模块状态，单行即完成。 */
                else if (strcmp(s_ble.line, "HEART,READ") == 0)
                {
                    s_ble.report = APP_REPORT_HEART;
                    s_ble.index = 0;
                }
                /* RESET,HISTORY：从电池日志导出历次复位，初始化全部扫描游标。 */
                else if (strcmp(s_ble.line, "RESET,HISTORY") == 0)
                {
                    s_ble.report = APP_REPORT_HISTORY;
                    s_ble.history_stage = 0; /* 先取日志状态。 */
                    s_ble.index = 0;
                    s_ble.slot = 0; /* 从第0槽开始扫描。 */
                    s_ble.session = 0; /* 尚未遇到session标记。 */
                    s_ble.exported = 0; /* 已导出的现场组数清零。 */
                }
            }
            s_ble.length = 0; /* 行已处理，长度归零准备下一行。 */
            s_ble.overflow = 0; /* 行结束清掉本行溢出标记。 */
        }
        /* 非可打印ASCII字符使整行作废。 */
        else if (data[i] < 32 || data[i] > 126)
        {
            s_ble.overflow = 1;
        }
        /* 行缓冲还能放下时追加字符并推进长度。 */
        else if (s_ble.length < sizeof(s_ble.line) - 1)
        {
            s_ble.line[s_ble.length++] = (char)data[i];
        }
        /* 缓冲已满仍来字符，标记溢出，整行不参与命令匹配。 */
        else
        {
            s_ble.overflow = 1;
        }
    }
}

static int app_state_ble_format(uint32_t now_ms)
{
    uint32_t age_ms = 0;
    /* index选择本行内容：0=RTC、1=校时、2=天气、其余=结束行。 */
    switch (s_ble.index)
    {
    case 0:
        /* 与校时、天气接收同属LVGL；整份报告只采样一次，后续发送不改变快照。
         * RTC状态与时间来自硬件读取，不是纯RAM缓存。 */
        watch_rtc_get_status(&s_ble.rtc); /* 读RTC硬件状态与当前时分秒。 */
        app_weather_get_snapshot(&s_ble.weather); /* 取天气模块RAM快照。 */
        s_ble.snapshot_ms = now_ms; /* 记下整份报告的采样时刻。 */
        s_ble.clock_frames = app_clock_get_received_count(); /* 已收到的校时帧数。 */
        s_ble.clock_result = app_clock_get_result(); /* 最近一次校时结果。 */
        /* 格式化RTC行：就绪/LSE/标记/同步/读取状态与时间。 */
        return snprintf(s_ble.tx, sizeof(s_ble.tx),
                        "STATE,RTC,READY,%u,LSE,%u,MARKER,0x%08lX,SYNCED,%u,READ,%u,TIME,%02u:%02u:%02u\r\n",
                        (unsigned)s_ble.rtc.ready, (unsigned)s_ble.rtc.lse_ready, (unsigned long)s_ble.rtc.marker,
                        (unsigned)s_ble.rtc.synced, (unsigned)s_ble.rtc.read_status,
                        (unsigned)s_ble.rtc.time.hours, (unsigned)s_ble.rtc.time.minutes, (unsigned)s_ble.rtc.time.seconds);
    case 1:
        /* 校时行：采样时刻(ms)、收到的帧数和结果码。 */
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "STATE,CLOCK,UP_MS,%lu,RX,%lu,RESULT,%u\r\n",
                        (unsigned long)s_ble.snapshot_ms, (unsigned long)s_ble.clock_frames, (unsigned)s_ble.clock_result);
    case 2:
        /* 天气快照有效时才计算数据年龄。 */
        if (s_ble.weather.valid != 0)
        {
            age_ms = s_ble.snapshot_ms - s_ble.weather.received_ms;
            /* 来自断电前缓存的数据无法得知经过时间，用UINT32_MAX表示未知。 */
            if (s_ble.weather.cached != 0)
            {
                age_ms = UINT32_MAX; /* 完全断电的经过时间未知。 */
            }
        }
        /* 天气行：有效位、状态码、温度×10、湿度、接收时刻、年龄和缓存标记。 */
        return snprintf(s_ble.tx, sizeof(s_ble.tx),
                        "STATE,WEATHER,VALID,%u,CODE,%u,TEMP_X10,%d,RH,%u,RX_MS,%lu,AGE_MS,%lu,CACHED,%u\r\n",
                        (unsigned)s_ble.weather.valid, (unsigned)s_ble.weather.code, (int)s_ble.weather.temperature_x10,
                        (unsigned)s_ble.weather.humidity_rh, (unsigned long)s_ble.weather.received_ms, (unsigned long)age_ms,
                        (unsigned)s_ble.weather.cached);
    default:
        /* 三行发完，输出结束行。 */
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "STATE,END\r\n");
    }
}

static int app_reset_history_format(void)
{
    app_battery_log_status_struct status;
    app_battery_log_record_struct record;
    uint32_t budget = 32; /* 每轮最多尝试32槽，避免长时间占住LVGL任务。 */
    uint8_t result, complete;
    /* stage 0：先取日志状态并生成BEGIN头行。 */
    if (s_ble.history_stage == 0)
    {
        /* 日志锁忙则本轮不生成文本，调用方下一轮重试。 */
        if (app_battery_log_get_status(&status) != 0)
        {
            return 0;
        }
        s_ble.limit = status.slots; /* 后续只扫描已占用槽。 */
        /* 日志还在启动扫描或需要清空时，直接回ERROR行并跳到结束阶段。 */
        if (status.state == APP_BATTERY_LOG_STARTING || status.state == APP_BATTERY_LOG_NEED_CLEAR)
        {
            s_ble.history_stage = 3;
            return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,ERROR,%u\r\n", (unsigned)status.state);
        }
        /* BEGIN行：完整现场组数、已占槽数和当前日志状态。 */
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,BEGIN,%lu,%lu,%u\r\n",
                        (unsigned long)status.resets, (unsigned long)status.slots, (unsigned)status.state);
    }
    /* stage 1：逐槽扫描，每轮最多budget槽，找到一组完整现场就输出一行。 */
    while (s_ble.slot < s_ble.limit && budget != 0)
    {
        --budget; /* 每尝试一槽消耗一次本轮预算。 */
        result = app_battery_log_read(s_ble.slot, s_ble.session, &record);
        /* 日志锁忙：保持游标，下一轮从同一槽继续。 */
        if (result == 2)
        {
            return 0;
        }
        /* 遇到session标记就更新当前开机编号，后续CRC按它校验。 */
        if (result == 0 && record.voltage_mv == UINT16_MAX)
        {
            s_ble.session = record.seconds;
        }
        /* 电压字段为0xFF00且剩余槽够一整组17槽时，尝试解码完整现场。 */
        else if (result == 0 && record.voltage_mv == APP_BATTERY_LOG_RESET_TAG &&
                 s_ble.limit - s_ble.slot >= APP_BATTERY_LOG_RESET_SLOTS)
        {
            /* 锁忙则保持游标，下一轮重试。 */
            if (app_battery_log_read_reset(s_ble.slot, s_ble.session, &s_ble.info, &complete) != 0)
            {
                return 0;
            }
            /* 整组校验通过：计数加一，游标跳过17槽，输出该组入口行。 */
            if (complete != 0)
            {
                ++s_ble.exported;
                s_ble.slot += APP_BATTERY_LOG_RESET_SLOTS;
                return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,ENTRY,SLOT,%lu,BOOT,%lu\r\n",
                                (unsigned long)(s_ble.slot - APP_BATTERY_LOG_RESET_SLOTS), (unsigned long)s_ble.session);
            }
        }
        ++s_ble.slot; /* 普通记录/坏槽/不完整现场都只前进一槽。 */
    }
    /* 已扫到上限：进入结束阶段并输出导出的现场组数。 */
    if (s_ble.slot == s_ble.limit)
    {
        s_ble.history_stage = 3;
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,END,%lu\r\n", (unsigned long)s_ble.exported);
    }
    return 0; /* 预算耗尽但未扫完，下一轮继续。 */
}

static uint8_t app_reset_ble_format(uint32_t now_ms)
{
    static const char *const fault_names[] = {"NONE", "READ", "WRITE", "PID", "CONFIG", "DATA_CHANGED"};
    app_heart_struct heart;
    const char *fault_name = "UNKNOWN";
    int length;
    app_reset_info_struct *info = &s_ble.info;
    /* STATE报告交给 app_state_ble_format 按 index 选行。 */
    if (s_ble.report == APP_REPORT_STATE)
    {
        length = app_state_ble_format(now_ms);
    }
    /* HEART报告：取心率快照，把故障枚举映射成名字后格式化成一行。 */
    else if (s_ble.report == APP_REPORT_HEART)
    {
        app_heart_get_snapshot(&heart);
        /* 故障码在名字表范围内才用表内名字，否则保持"UNKNOWN"。 */
        if ((unsigned)heart.fault.kind < sizeof(fault_names) / sizeof(fault_names[0]))
        {
            fault_name = fault_names[heart.fault.kind];
        }
        /* 心率行：状态、使能、故障码/名字、故障寄存器和值。 */
        length = snprintf(s_ble.tx, sizeof(s_ble.tx),
                          "HEART,STATE,%u,EN,%u,FAULT,%u,%s,REG,0x%02X,VALUE,0x%02X\r\n",
                          (unsigned)heart.state, (unsigned)heart.enabled, (unsigned)heart.fault.kind, fault_name,
                          (unsigned)heart.fault.reg, (unsigned)heart.fault.value);
    }
    /* HISTORY报告在stage 2(逐行输出RESET,READ)之外，由历史扫描函数生成下一行。 */
    else if (s_ble.report == APP_REPORT_HISTORY && s_ble.history_stage != 2)
    {
        length = app_reset_history_format();
        /* 返回0说明锁忙或本轮预算耗尽，保持请求，下一轮继续。 */
        if (length == 0)
        {
            return 1; /* 日志锁忙或本轮扫描预算耗尽，保持请求，下一轮继续。 */
        }
    }
    /* 其余为RESET报告的固定4行，按index选择内容。 */
    else
    {
        switch (s_ble.index)
        {
        case 0:
            /* RESET报告第一次进入时取一份上一启动快照，后续行复用同一份。 */
            if (s_ble.report == APP_REPORT_RESET)
            {
                app_reset_get_info(info); /* 一次请求只取一份上一启动快照。 */
            }
            /* 原因行：记录是否有效、RCC复位标志和停机类型。 */
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,CAUSE,VALID,%lu,FLAGS,0x%08lX,KIND,%lu\r\n",
                              (unsigned long)info->valid, (unsigned long)info->reset_flags, (unsigned long)info->kind);
            break;
        case 1:
            /* 时间行：运行时长、上次喂狗时刻、最大喂狗间隔和任务标记(均为毫秒或原值)。 */
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,TIME,UP_MS,%lu,FEED_MS,%lu,GAP_MS,%lu,TASK,%lu\r\n",
                              (unsigned long)info->uptime_ms, (unsigned long)info->last_feed_ms,
                              (unsigned long)info->max_feed_gap_ms, (unsigned long)info->task_tag);
            break;
        case 2:
            /* 异常帧行：栈指针、EXC_RETURN、PC、LR、xPSR。 */
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,FRAME,SP,0x%08lX,EXC,0x%08lX,PC,0x%08lX,LR,0x%08lX,XPSR,0x%08lX\r\n",
                              (unsigned long)info->sp, (unsigned long)info->exc_return, (unsigned long)info->pc,
                              (unsigned long)info->lr, (unsigned long)info->xpsr);
            break;
        case 3:
            /* 故障状态行：CFSR、HFSR、MMFAR、BFAR寄存器原值。 */
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,FAULT,CFSR,0x%08lX,HFSR,0x%08lX,MMFAR,0x%08lX,BFAR,0x%08lX\r\n",
                              (unsigned long)info->cfsr, (unsigned long)info->hfsr, (unsigned long)info->mmfar, (unsigned long)info->bfar);
            break;
        default:
            /* 4行已发完，输出结束行。 */
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,END\r\n");
            break;
        }
    }
    /* 格式化失败或超出tx缓冲时清报告状态并返回1，避免发送截断内容。 */
    if (length <= 0 || (unsigned)length >= sizeof(s_ble.tx))
    {
        app_reset_ble_reset();
        return 1;
    }
    s_ble.tx_length = (uint16_t)length; /* 记下本行长度，供发送使用。 */
    return 0;
}

/* 每拍推进一次报告：先收上一笔异步TX结果，再格式化下一行，最后在UART空闲时启动发送。
 * 行间至少100ms(首行除外)；ready=0时丢弃未完成报告但仍回收在途结果。
 * 返回值1表示报告未完成或TX仍占用发送服务，不表示发送成功。 */
uint8_t app_reset_ble_service(uint8_t ready, uint32_t now_ms)
{
    HAL_StatusTypeDef result;
    /* 蓝牙未就绪：丢弃未完成报告，但下面仍会回收在途结果。 */
    if (ready == 0)
    {
        app_reset_ble_reset();
    }
    /* 有在途发送时先处理它，取不到结果就继续等待。 */
    if (s_ble.pending != 0)
    {
        /* 结果尚未产生则保持pending，本轮直接返回。 */
        if (kt6368a_take_tx_result(&result) == 0)
        {
            return 1;
        }
        s_ble.pending = 0; /* 已取到结果，清在途标记。 */
        s_ble.tx_length = 0; /* 本笔发送结果已回收，清空文本长度。 */
        /* 发送失败：清报告状态，等手机重新查询。 */
        if (result != HAL_OK)
        {
            app_reset_ble_reset();
            return 0; /* 失败后等待手机重新查询，不自动重发报告。 */
        }
        /* 报告仍有效时推进行号/阶段，为下一行做准备。 */
        if (s_ble.report != 0)
        {
            s_ble.tx_ms = now_ms; /* 记下本行完成时刻，作为100ms间隔起点。 */
            /* HISTORY有独立的三阶段推进。 */
            if (s_ble.report == APP_REPORT_HISTORY)
            {
                /* stage 0：BEGIN已发完，进入逐槽扫描。 */
                if (s_ble.history_stage == 0)
                {
                    s_ble.history_stage = 1;
                }
                /* stage 1：ENTRY已发完，进入该组现场的4行详细输出。 */
                else if (s_ble.history_stage == 1)
                {
                    s_ble.history_stage = 2;
                    s_ble.index = 0;
                }
                /* stage 2：现场详情每发完一行就推进index；4行发完回到stage 1，继续扫描下一组或生成END。 */
                else if (s_ble.history_stage == 2)
                {
                    ++s_ble.index;
                    if (s_ble.index == 4)
                    {
                        s_ble.history_stage = 1;
                    }
                }
                /* stage 3：ERROR/END之后结束整份报告。 */
                else
                {
                    app_reset_ble_reset();
                }
            }
            /* 其它报告按行号推进。 */
            else
            {
                ++s_ble.index;
                /* RESET共5行(4行+END)、STATE共4行、HEART单行，达到行数即结束报告。 */
                if ((s_ble.report == APP_REPORT_RESET && s_ble.index == 5) ||
                    (s_ble.report == APP_REPORT_STATE && s_ble.index == 4) || s_ble.report == APP_REPORT_HEART)
                {
                    app_reset_ble_reset();
                }
            }
        }
    }
    /* 未就绪或无报告时本轮不准备发送。 */
    if (ready == 0 || s_ble.report == 0)
    {
        return 0;
    }
    /* 无待发文本时格式化下一行；格式化未产出文本则按报告是否仍在返回。 */
    if (s_ble.tx_length == 0 && app_reset_ble_format(now_ms) != 0)
    {
        return s_ble.report != 0;
    }
    /* 首行立即发，后续行距上一笔完成至少100ms；且UART总线空闲才启动。 */
    if (((s_ble.report != APP_REPORT_HISTORY && s_ble.index == 0) ||
         (uint32_t)(now_ms - s_ble.tx_ms) >= 100) &&
        kt6368a_tx_is_busy() == 0)
    {
        result = kt6368a_write_async((const uint8_t *)s_ble.tx, s_ble.tx_length, 100); /* 异步启动本行发送。 */
        /* 启动成功则记在途，等下一拍回收结果。 */
        if (result == HAL_OK)
        {
            s_ble.pending = 1;
        }
        /* 非BUSY的失败说明通道异常，清报告状态；BUSY则保持文本下轮再试。 */
        else if (result != HAL_BUSY)
        {
            app_reset_ble_reset();
        }
    }
    return s_ble.report != 0 || s_ble.pending != 0;
}
