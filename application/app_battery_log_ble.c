/**
 * @file app_battery_log_ble.c
 * @brief BATLOG串口协议处理：接收命令、格式化状态/记录、经KT6368A异步发送。
 *        发送是单发在途(pending)的节流状态机，每轮service最多推进有限条记录(budget)，
 *        与其它蓝牙发送者分时共用UART；CLEAR先回执，等300ms后擦除并重启。
 */
#include "app_battery_log_ble.h"
#include "app_battery_log.h"
#include "int_battery_log.h"
#include "kt6368a.h"
#include <stdio.h>
#include <string.h>

/* 待处理命令；回执文本固定为 BATLOG,<字段>…\r\n，READ 走 BEGIN/DATA/END 三段流式输出。 */
enum
{
    BATLOG_NONE,
    BATLOG_STATUS,
    BATLOG_READ,
    BATLOG_CLEAR,
    BATLOG_STOP
};

/* 状态只在 LVGL 任务读写(sensor 不触碰)；跨轮保留的是发送在途与流式游标。 */
static struct
{
    char line[24];
    char tx[112];
    uint32_t rx_ms, tx_ms, index, limit, samples, session, next_index;
    uint16_t length, tx_length;
    uint8_t overflow, command, streaming, pending, after_send, clear_wait;
} s_ble;

/* 丢弃接收残帧与未执行的命令；发送在途(pending)不在此撤销，由service自行回收结果。 */
void app_battery_log_ble_reset(void)
{
    s_ble.length = 0; /* 丢弃未成行的接收残帧。 */
    s_ble.overflow = 0; /* 清掉上一帧的超长/非法字符标记。 */
    s_ble.command = BATLOG_NONE; /* 撤销尚未执行的命令。 */
    s_ble.streaming = 0; /* 结束READ流式导出状态。 */
    s_ble.clear_wait = 0; /* 取消CLEAR回执后的等待重启。 */
    /* 异步TX缓冲必须保留到结果回收；pending=0时清长度，避免旧文本被当成本轮待发。 */
    if (s_ble.pending == 0)
    {
        s_ble.tx_length = 0;
    }
    s_ble.after_send = 0; /* 清除"本笔发送完成后要执行的动作"。 */
}

/* 分包到达的字节在此拼行：CR/LF成形，控制字符或超过23字符使整帧作废(overflow)，
 * 超过1秒没有字节则先丢掉残帧，避免把断连前的半行接到新会话上。 */
void app_battery_log_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    uint16_t i;
    /* 距上次收到字节已满1秒，视为新会话：丢掉半行和溢出标记。 */
    if ((uint32_t)(now_ms - s_ble.rx_ms) >= 1000)
    {
        s_ble.length = 0;
        s_ble.overflow = 0;
    }
    s_ble.rx_ms = now_ms; /* 记录本批字节的接收时刻，供下一次超时判断。 */
    /* 逐字节处理本次收到的数据。 */
    for (i = 0; i < size; ++i)
    {
        /* CR或LF表示一行结束。 */
        if (data[i] == '\r' || data[i] == '\n')
        {
            s_ble.line[s_ble.length] = '\0'; /* 在已收字符后补字符串结束符，形成完整行。 */
            /* 只接受一条未执行命令；CLEAR等待重启期间不再收新命令。 */
            if (s_ble.overflow == 0 && s_ble.command == BATLOG_NONE && s_ble.clear_wait == 0)
            {
                /* 整行精确匹配四条命令之一，命中则记下待执行命令。 */
                if (strcmp(s_ble.line, "BATLOG,STATUS") == 0)
                {
                    s_ble.command = BATLOG_STATUS;
                }
                else if (strcmp(s_ble.line, "BATLOG,READ") == 0)
                {
                    s_ble.command = BATLOG_READ;
                }
                else if (strcmp(s_ble.line, "BATLOG,CLEAR") == 0)
                {
                    s_ble.command = BATLOG_CLEAR;
                }
                else if (strcmp(s_ble.line, "BATLOG,STOP") == 0)
                {
                    s_ble.command = BATLOG_STOP;
                }
            }
            s_ble.length = 0; /* 行已处理，长度归零准备下一行。 */
            s_ble.overflow = 0; /* 行结束同时清掉本行溢出标记。 */
        }
        /* 非可打印ASCII字符使整行作废，不再往缓冲里追加。 */
        else if (data[i] < 32 || data[i] > 126)
        {
            s_ble.overflow = 1;
        }
        /* 行缓冲还能放下时追加当前字符，并推进长度。 */
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

/* 每轮推进：先回收自己的在途发送，再处理命令/流式READ；ready=0时丢弃残帧。
 * 每轮最多尝试32槽，发送间隔≥100ms且总线空闲才发下一条。返回1表示仍需占用蓝牙发送。 */
uint8_t app_battery_log_ble_service(uint8_t ready, uint32_t now_ms)
{
    app_battery_log_status_struct status;
    app_battery_log_record_struct record;
    HAL_StatusTypeDef result;
    uint32_t budget = 32; /* 每轮最多尝试读取32槽，跳过坏槽/旧开机时也有上限，不长时间占住LVGL任务 */
    uint8_t read_result;
    /* 有在途发送时先处理它，本轮到结果回收之前不准备新内容。 */
    if (s_ble.pending != 0)
    {
        /* 在途发送未回收前不推进任何状态，也不替别的发送者取结果。 */
        if (kt6368a_take_tx_result(&result) == 0)
        {
            return 1;
        }
        s_ble.pending = 0; /* 已取到结果，清在途标记。 */
        s_ble.tx_length = 0; /* 本笔发送结果已回收，清空文本长度。 */
        /* 发送失败：清空协议状态，等手机重新请求。 */
        if (result != HAL_OK)
        {
            app_battery_log_ble_reset();
            return 0; /* 发送失败后等手机重新请求，不无限重发。 */
        }
        s_ble.index = s_ble.next_index; /* 发送成功才把读游标推进到下一槽。 */
        s_ble.tx_ms = now_ms; /* 记下本笔完成时刻，作为100ms间隔的起点。 */
        /* CLEAR回执已发完，转入等待300ms后擦除重启。 */
        if (s_ble.after_send == BATLOG_CLEAR)
        {
            s_ble.clear_wait = 1;
        }
        s_ble.after_send = 0; /* 本次"发送后动作"已执行或无需执行，清空。 */
    }
    /* 蓝牙未就绪时丢弃解析/命令状态，返回0表示本轮不占用发送。 */
    if (ready == 0)
    {
        app_battery_log_ble_reset();
        return 0;
    }
    /* CLEAR 回执发出后等300ms再擦扇区重启，留出SPP把文本送完的时间。 */
    if (s_ble.clear_wait != 0)
    {
        /* 距回执完成满300ms才真正擦除并重启。 */
        if ((uint32_t)(now_ms - s_ble.tx_ms) >= 300)
        {
            app_battery_log_clear_and_reset();
        }
        return 1; /* 等待期间继续占用蓝牙发送。 */
    }
    /* 无待发文本且有命令待处理时，先取日志状态并生成首行文本。 */
    if (s_ble.tx_length == 0 && s_ble.command != BATLOG_NONE)
    {
        /* 日志锁忙则保持命令，下一轮再取状态。 */
        if (app_battery_log_get_status(&status) != 0)
        {
            return 1;
        }
        s_ble.streaming = 0; /* 新命令默认不是流式导出，READ分支再单独置位。 */
        /* STATUS：输出有效样本数、已占槽数、容量和日志状态枚举值。 */
        if (s_ble.command == BATLOG_STATUS)
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,STATUS,%lu,%lu,%lu,%u\r\n",
                                                 (unsigned long)status.samples, (unsigned long)status.slots,
                                                 (unsigned long)BATTERY_LOG_CAPACITY, (unsigned)status.state);
        }
        /* READ在日志区需要清空时直接回错误行，不进入导出。 */
        else if (s_ble.command == BATLOG_READ && status.state == APP_BATTERY_LOG_NEED_CLEAR)
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,ERROR,NEED_CLEAR\r\n");
        }
        /* READ正常路径：初始化游标/上限/session，并生成BEGIN头行。 */
        else if (s_ble.command == BATLOG_READ)
        {
            s_ble.index = 0;
            s_ble.next_index = 0;
            s_ble.limit = status.slots; /* 只扫描已占用槽。 */
            s_ble.samples = status.samples; /* BEGIN/END中回报的样本总数。 */
            s_ble.session = 0; /* 初始未知开机编号，遇到session标记后更新。 */
            s_ble.streaming = 1; /* 标记后续轮次继续逐条导出。 */
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,BEGIN,%lu,%lu\r\n",
                                                 (unsigned long)s_ble.samples, (unsigned long)s_ble.limit);
        }
        /* CLEAR：先回执，并记下"发送完成后等待重启"的动作。 */
        else if (s_ble.command == BATLOG_CLEAR)
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,CLEARING_REBOOT\r\n");
            s_ble.after_send = BATLOG_CLEAR;
        }
        /* 其余命令(如STOP)统一回STOPPED。 */
        else
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,STOPPED\r\n");
        }
        s_ble.command = BATLOG_NONE; /* 命令已转成待发文本，清命令避免重复处理。 */
    }
    /* 流式READ：没有待发文本时继续向后找下一条可导出的记录。 */
    if (s_ble.tx_length == 0 && s_ble.streaming != 0)
    {
        /* 在剩余槽内扫描，最多尝试budget次。 */
        while (s_ble.index < s_ble.limit && budget != 0)
        {
            --budget; /* 每尝试一槽消耗一次本轮预算。 */
            read_result = app_battery_log_read(s_ble.index, s_ble.session, &record);
            /* 日志锁忙：保持游标，下一轮从同一槽继续。 */
            if (read_result == 2)
            {
                return 1;
            }
            /* 读取成功才解释记录内容。 */
            if (read_result == 0)
            {
                /* 遇到开机标记就换用新编号，后续记录才能按对应CRC解出来。 */
                if (record.voltage_mv == UINT16_MAX)
                {
                    s_ble.session = record.seconds;
                }
                /* 普通电压记录：格式化成DATA行，并把下一槽位置记为当前槽+1。 */
                else if (s_ble.session != 0 && record.voltage_mv < APP_BATTERY_LOG_RESET_TAG)
                {
                    s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,DATA,%lu,%lu,%lu,%u\r\n",
                                                         (unsigned long)s_ble.index, (unsigned long)s_ble.session,
                                                         (unsigned long)record.seconds, (unsigned)record.voltage_mv);
                    s_ble.next_index = s_ble.index + 1;
                    break; /* 本轮只准备一条DATA，等发送完成后再继续。 */
                }
            }
            ++s_ble.index; /* 坏槽/复位现场槽跳过，游标前进一槽。 */
        }
        /* 已到上限仍无待发文本，说明本轮扫描结束：生成END行并退出流式状态。 */
        if (s_ble.tx_length == 0 && s_ble.index == s_ble.limit)
        {
            s_ble.streaming = 0;
            s_ble.next_index = s_ble.index;
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,END,%lu\r\n", (unsigned long)s_ble.samples);
        }
    }
    /* 与其它蓝牙发送者分时共用UART：距上一笔完成至少100ms且总线空闲才发下一条。 */
    if (s_ble.tx_length != 0 && (uint32_t)(now_ms - s_ble.tx_ms) >= 100 && kt6368a_tx_is_busy() == 0)
    {
        result = kt6368a_write_async((const uint8_t *)s_ble.tx, s_ble.tx_length, 100); /* 异步启动本行发送。 */
        /* 启动成功则记在途，等下一轮回收结果。 */
        if (result == HAL_OK)
        {
            s_ble.pending = 1;
        }
        /* 非BUSY的失败说明通道异常，清空协议状态；BUSY则保持文本下轮再试。 */
        else if (result != HAL_BUSY)
        {
            app_battery_log_ble_reset();
        }
    }
    /* 还有在途发送、待发文本或未导完的记录时返回1，通知调用方本轮别再排队心率波形。 */
    return s_ble.pending != 0 || s_ble.tx_length != 0 || s_ble.streaming != 0;
}
