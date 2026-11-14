/**
 * @file app_ble_service.c
 * @brief 显示任务串行执行的蓝牙业务协调；不依赖LVGL控件或页面状态。
 *
 * AT初始化仍由app_task推进；正常业务是唯一UART接收消费者。各解析器保留原协议，
 * 丢字节仅废弃残行，真实空闲边界和到达时间由驱动提供。ECO等待1000ms安静期，
 * SPORT强制断电；各发送者的异步缓冲及完成结果仍由原发起者保有。
 */
#include "app_ble_service.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_weather.h"
#include "app_uptime.h"
#include "app_diag.h"
#include "app_battery_log_ble.h"
#include "app_reset_ble.h"
#include "app_ota.h"
#include "kt6368a.h"
#include "watch_rtc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define APP_BLE_LOG_SIZE       512
#define APP_BLE_READ_SIZE      64
#define APP_BLE_RX_BUDGET      256
#define APP_BLE_TX_TIMEOUT_MS  20
#define APP_BLE_ECO_QUIET_MS   1000

/* LVGL当前直接使用1kHz FreeRTOS tick，与天气/校时的业务毫秒时基相同。 */
static struct
{
    char log[APP_BLE_LOG_SIZE];
    uint16_t log_length;
    uint32_t log_revision;
    uint32_t rx_bytes, rx_dropped, rx_errors;
    uint32_t quiet_tick, clock_frames;
    uint8_t rx_resync, last_cr;
    uint8_t phone_ready, uptime_sent, management_busy;
    uint8_t eco, enabled;
    app_ble_tx_state_enum tx_state;
#if APP_DEBUG_TELEMETRY
    uint8_t heart_frames[72];
    uint16_t heart_pending_count;
#endif
} s_ble = {.eco = 1, .enabled = 1, .tx_state = APP_BLE_TX_READY};

/* 错误位置由到达队列保留，只废弃残帧，不清掉错误后的完整消息。 */
static void app_ble_rx_reset(void)
{
    /* 让时钟/天气/诊断/电池日志/复位统计/OTA各自的接收状态机丢弃残帧，回到等待新消息。 */
    app_clock_reset_receive();
    app_weather_reset_receive();
    app_diag_reset_receive();
    app_battery_log_ble_reset();
    app_reset_ble_reset();
    app_ota_reset_receive();
    /* 置重同步标志，下一块数据先跳到CR/LF之后才交给解析器。 */
    s_ble.rx_resync = 1;
    /* 手机就绪标志清零，等新的完整时钟帧重新确认通信。 */
    s_ble.phone_ready = 0;
    /* 记录当前已收到的时钟帧计数，作为之后判断新增帧的基线。 */
    s_ble.clock_frames = app_clock_get_received_count();
}

static void app_ble_receive_frames(uint8_t sport_mode)
{
    /* 十六进制半字节到ASCII字符的查表，用于把非可打印字节显示成\xHH。 */
    static const char hex[] = "0123456789ABCDEF";
    /* 本轮从UART驱动取出的原始字节块。 */
    uint8_t data[APP_BLE_READ_SIZE];
    /* 转成可显示文本的临时缓冲，最坏每字节展开成4个字符。 */
    char text[APP_BLE_READ_SIZE * 4];
    /* 本次实际读到的字节数、转出的文本长度、需要丢弃的旧文本数、下标、起点和剩余预算。 */
    uint16_t received, length, remove, i, start, budget = APP_BLE_RX_BUDGET;
    /* 本轮业务毫秒时刻、HAL/RTOS零点偏移、驱动丢字节计数和UART错误计数。 */
    uint32_t now_ms, tick_offset, dropped, errors;
    /* 队列已读空的标志。 */
    uint8_t drained = 0;
    /* 驱动返回的本块元数据：到达时刻、采样时刻和空闲/丢字节标志。 */
    kt6368a_rx_info_struct info;

    /* AT响应留给初始化；RTC未就绪时先保留手机校时等业务字节。 */
    if (s_ble.enabled == 0 || sport_mode != 0 || kt6368a_init_is_pending() != 0 || watch_rtc_init_is_pending() != 0)
    {
        return;
    }
    /* HAL与RTOS的启动零点不同；临界区采样偏移，把UART到达tick转换为业务时间。 */
    taskENTER_CRITICAL();
    tick_offset = (uint32_t)xTaskGetTickCount() - HAL_GetTick();
    taskEXIT_CRITICAL();

    /* 本轮最多处理APP_BLE_RX_BUDGET字节，budget减到0即退出循环。 */
    while (budget != 0)
    {
        /* 剩余预算不足一整块时只读到预算大小。 */
        if (budget < sizeof(data))
        {
            received = kt6368a_read_timed(data, budget, &info);
        }
        else
        {
            /* 否则按data的整块容量读取。 */
            received = kt6368a_read_timed(data, sizeof(data), &info);
        }
        /* 驱动队列已空，置drained供循环后做空闲结算并退出。 */
        if (received == 0)
        {
            drained = 1;
            break;
        }
        /* 扣减本轮预算，记录本次到达时刻作为蓝牙安静期起点。 */
        budget -= received;
        s_ble.quiet_tick = (uint32_t)xTaskGetTickCount();
        /* 把驱动给的HAL到达时刻换算到业务时基，供各解析器打时间戳。 */
        now_ms = info.received_ms + tick_offset;
        /* 驱动报告丢字节：各解析器复位残帧，并进入重同步。 */
        if ((info.flags & KT6368A_RX_LOSS) != 0)
        {
            app_ble_rx_reset();
        }
        /* 驱动报告本块前有真实空闲间隔，说明行边界完整，解除重同步。 */
        if ((info.flags & KT6368A_RX_GAP) != 0)
        {
            s_ble.rx_resync = 0;
        }
        /* start为本块中第一个可交给业务解析的字节下标。 */
        start = 0;
        /* 处于重同步状态时，逐字节跳过残行直到CR/LF之后。 */
        while (s_ble.rx_resync != 0 && start < received)
        {
            /* 遇到行尾即认为残行结束。 */
            if (data[start] == '\r' || data[start] == '\n')
            {
                s_ble.rx_resync = 0;
            }
            /* 无论是否是行尾都前进，行尾字节本身也丢弃。 */
            ++start;
        }
        /* 本块还有有效字节时，把同一段数据按各业务协议分发给全部解析器。 */
        if (start < received)
        {
            /* 电池日志、复位统计、OTA、诊断依次接收。 */
            app_battery_log_ble_receive(data + start, received - start, now_ms);
            app_reset_ble_receive(data + start, received - start, now_ms);
            app_ota_receive(data + start, received - start, now_ms);
            app_diag_receive(data + start, received - start);
            /* 时钟和天气解析也使用同一个到达时刻。 */
            app_clock_receive(data + start, received - start, now_ms);
            app_weather_receive(data + start, received - start, now_ms);
        }
        /* 累计原始接收字节数，供界面显示RX计数。 */
        s_ble.rx_bytes += received;
        /* 下面把本块原始字节转成可显示文本，length从0开始。 */
        length = 0;
        /* 逐字节转换，保留行结构并转义不可打印字符。 */
        for (i = 0; i < received; ++i)
        {
            if (data[i] == '\r')
            {
                /* 回车统一显示成换行，并记住上一个字符是CR。 */
                text[length++] = '\n';
                s_ble.last_cr = 1;
            }
            else if (data[i] == '\n')
            {
                /* CRLF中的LF不再重复换行；单独的LF仍转成换行。 */
                if (!s_ble.last_cr)
                {
                    text[length++] = '\n';
                }
                /* 处理完LF后清除CR标记。 */
                s_ble.last_cr = 0;
            }
            else
            {
                /* 其他字符都清掉CR标记。 */
                s_ble.last_cr = 0;
                if (data[i] == '\t')
                {
                    /* 制表符显示成一个空格。 */
                    text[length++] = ' ';
                }
                else if (data[i] >= 32 && data[i] <= 126)
                {
                    /* 可打印ASCII原样保留。 */
                    text[length++] = (char)data[i];
                }
                else
                {
                    /* 不可打印字节展开成\xHH，高/低半字节分别查表。 */
                    text[length++] = '\\';
                    text[length++] = 'x';
                    text[length++] = hex[data[i] >> 4];
                    text[length++] = hex[data[i] & 0x0F];
                }
            }
        }
        /* 新文本加旧文本会超过缓冲时，算出需要从最旧端移除的字符数。 */
        if (s_ble.log_length + length >= sizeof(s_ble.log))
        {
            remove = s_ble.log_length + length - (sizeof(s_ble.log) - 1);
            /* 把保留部分前移，丢掉最旧的remove个字符。 */
            memmove(s_ble.log, s_ble.log + remove, s_ble.log_length - remove);
            s_ble.log_length -= remove;
        }
        /* 把本轮文本追加到历史末尾，更新长度并补字符串结束符。 */
        for (i = 0; i < length; ++i)
        {
            s_ble.log[s_ble.log_length + i] = text[i];
        }
        s_ble.log_length += length;
        s_ble.log[s_ble.log_length] = '\0';
        /* 页面按日志版本刷新，不消费或修改服务状态。 */
        ++s_ble.log_revision;
    }
    /* 只有队列确实读空时才做丢字节统计与空闲结算，避免把"预算用完"当成空闲。 */
    if (drained != 0)
    {
        /* 取驱动累计的丢字节数和UART错误数。 */
        dropped = kt6368a_get_rx_dropped();
        errors = kt6368a_get_rx_errors();
        /* 本块报告丢字节且计数确有变化时，再复位一次各解析器。 */
        if ((info.flags & KT6368A_RX_LOSS) != 0 && (dropped != s_ble.rx_dropped || errors != s_ble.rx_errors))
        {
            app_ble_rx_reset(); /* 错误后尚无下一字节时，也不能提交残缺的无换行帧。 */
        }
        /* 保存本轮计数，作为下次比较的基线。 */
        s_ble.rx_dropped = dropped;
        s_ble.rx_errors = errors;
        /* 采样时刻与最后到达时刻相差达到空闲阈值，说明总线已静默，推进时钟/天气超时等待。 */
        if ((uint32_t)(info.sampled_ms - info.received_ms) >= KT6368A_RX_IDLE_MS)
        {
            app_clock_service(info.sampled_ms + tick_offset);
            app_weather_service(info.sampled_ms + tick_offset);
        }
    }
    /* 时钟帧计数增加说明收到新的完整校时消息，标记手机通信已就绪。 */
    if (s_ble.clock_frames != app_clock_get_received_count())
    {
        s_ble.clock_frames = app_clock_get_received_count();
        s_ble.phone_ready = 1;
    }
}

/* 亮屏/熄屏都维护一小时温度有效期，ECO或SPORT暂停接收时同样使旧温度到期。 */
static void app_ble_receive(uint8_t sport_mode)
{
    app_weather_struct weather;
    app_ble_receive_frames(sport_mode);
    app_weather_get_snapshot(&weather);
    app_baro_set_reference(weather.sea_level_pa, weather.temperature_x10,
                           weather.valid != 0 && weather.cached == 0 && (uint32_t)((uint32_t)xTaskGetTickCount() - weather.received_ms) < 3600000);
}

/* 合法手机校时确认通信就绪；旧运行记录每次开机只回传一次。 */
static uint8_t app_ble_send_uptime(uint8_t sport_mode)
{
    /* 上次运行累计秒数。 */
    uint32_t seconds;
    /* 格式化后的报文长度。 */
    uint16_t size;
    /* 存放UPTIME_PREV报文的栈缓冲。 */
    char message[64];
    /* 同步发送返回状态。 */
    HAL_StatusTypeDef status;
    /* 已发送过、手机未就绪、运动中、初始化未完、TX忙或没有旧记录时都返回0不发送。 */
    if (s_ble.uptime_sent != 0 || s_ble.phone_ready == 0 || sport_mode != 0 ||
        kt6368a_init_is_pending() != 0 || kt6368a_tx_is_busy() != 0 || app_uptime_get_previous(&seconds) == 0)
    {
        return 0;
    }
    /* 把总秒数拆成总秒、时、分、秒四段，格式化成UPTIME_PREV行。 */
    size = (uint16_t)snprintf(message, sizeof(message), "UPTIME_PREV,%lu,%lu:%02lu:%02lu\r\n",
                                 (unsigned long)seconds, (unsigned long)(seconds / 3600),
                                 (unsigned long)(seconds / 60 % 60), (unsigned long)(seconds % 60));
    /* 同步写出报文，超时用APP_BLE_TX_TIMEOUT_MS。 */
    status = kt6368a_write((const uint8_t *)message, size, APP_BLE_TX_TIMEOUT_MS);
    /* 串口忙时本轮不算发送过，返回0让下次再试。 */
    if (status == HAL_BUSY)
    {
        return 0;
    }
    if (status == HAL_OK)
    {
        /* UART成功才记录已发送，不因重连而重复报告。 */
        s_ble.uptime_sent = 1;
        s_ble.tx_state = APP_BLE_TX_UPTIME_OK;
    }
    else
    {
        /* 其他失败等下一次合法手机消息再试。 */
        s_ble.phone_ready = 0; /* 等下一次合法手机消息再试，不连续发送故障串口。 */
        s_ble.tx_state = APP_BLE_TX_UPTIME_ERROR;
    }
    /* 返回1表示本轮已处理UPTIME发送(成功或失败)，调用方据此结束本轮。 */
    return 1;
}

static uint8_t app_ble_send(uint8_t screen_on, uint8_t sport_mode)
{
    /* 三个命令服务是否仍占用发送服务的汇总标志。 */
    uint8_t management_busy = 0;
#if APP_DEBUG_TELEMETRY
    /* 本轮取出的心率样本，最多2个。 */
    app_heart_sample_struct samples[2];
    /* 单个样本的8个float通道。 */
    float channels[8];
    /* 样本数、样本下标、通道下标和帧内字节偏移。 */
    uint16_t count, i, channel, offset;
    /* float按位拆出的32位中间量。 */
    uint32_t bits;
    /* 异步发送启动返回状态。 */
    HAL_StatusTypeDef status;
    /* 上一次异步发送还未回收结果时，先尝试取回。 */
    if (s_ble.heart_pending_count != 0)
    {
        /* 结果尚未就绪：置管理忙标志并结束本轮，不发起新发送。 */
        if (kt6368a_take_tx_result(&status) == 0)
        {
            s_ble.management_busy = 1;
            return 0;
        }
        /* 结果已到：把本次发送的样本数和成功与否记入诊断统计。 */
        app_diag_heart_tx(s_ble.heart_pending_count, status == HAL_OK);
        /* 清待回收计数，避免重复取同一结果。 */
        s_ble.heart_pending_count = 0;
        /* 发送活动刷新蓝牙安静期起点。 */
        s_ble.quiet_tick = (uint32_t)xTaskGetTickCount();
        if (status == HAL_OK)
        {
            /* 成功显示绿色TX HEART OK。 */
            s_ble.tx_state = APP_BLE_TX_HEART_OK;
        }
        else
        {
            /* 失败显示红色TX HEART ERROR。 */
            s_ble.tx_state = APP_BLE_TX_HEART_ERROR;
        }
    }
#endif
    (void)screen_on;
    /* 三个命令服务各自回收自己的异步TX结果：返回1表示报告或在途TX仍占用发送服务。
     * 本文件不把该返回值当作发送成功，只据此推迟UPTIME与波形，避免抢别的发送者。 */
    if (app_battery_log_ble_service(s_ble.enabled != 0 && sport_mode == 0 && kt6368a_init_is_pending() == 0, (uint32_t)xTaskGetTickCount()) != 0)
    {
        /* 电池日志服务仍忙，汇总置1。 */
        management_busy = 1;
    }
    if (app_reset_ble_service(s_ble.enabled != 0 && sport_mode == 0 && kt6368a_init_is_pending() == 0, (uint32_t)xTaskGetTickCount()) != 0)
    {
        /* 复位统计服务仍忙，汇总置1。 */
        management_busy = 1;
    }
    if (app_ota_service(s_ble.enabled != 0 && sport_mode == 0 && kt6368a_init_is_pending() == 0, (uint32_t)xTaskGetTickCount()) != 0)
    {
        /* OTA服务仍忙，汇总置1。 */
        management_busy = 1;
    }
    /* 本轮或上一轮仍有服务忙时，刷新安静期起点，防止在发送期间断电。 */
    if (s_ble.management_busy != 0 || management_busy != 0)
    {
        s_ble.quiet_tick = (uint32_t)xTaskGetTickCount();
    }
    /* 保存本轮忙状态，供下一轮判断忙到闲的切换。 */
    s_ble.management_busy = management_busy;
    if (management_busy != 0)
    {
        /* 三个命令服务都须收回各自的TX结果，避免另一个待发命令挡住结果回收。 */
        return 1;
    }
    /* 命令服务空闲后先尝试补发一次启动时长记录，返回1表示本轮已处理。 */
    if (app_ble_send_uptime(sport_mode) != 0)
    {
        s_ble.quiet_tick = (uint32_t)xTaskGetTickCount();
        return 0;
    }
#if APP_DEBUG_TELEMETRY
    /* 熄屏、运动中、初始化未完或TX仍忙时不发波形。 */
    if (screen_on == 0 || sport_mode != 0 || kt6368a_init_is_pending() != 0 || kt6368a_tx_is_busy() != 0)
    {
        return 0;
    }
    /* 取最多2个待发心率样本。 */
    count = app_heart_read_samples(samples, sizeof(samples) / sizeof(samples[0]));
    /* 没有新样本就结束。 */
    if (count == 0)
    {
        return 0;
    }
    /* 逐个样本把8个通道打包进帧缓冲。 */
    for (i = 0; i < count; ++i)
    {
        /* 通道0为原始光强ADC计数。 */
        channels[0] = (float)samples[i].raw;
        /* 通道1为滤波后的有符号信号(ADC计数)。 */
        channels[1] = samples[i].filtered;
        /* 通道2为背景基线(ADC计数)。 */
        channels[2] = samples[i].dc;
        /* 通道3为BPM。 */
        channels[3] = (float)samples[i].bpm;
        /* 通道4为波峰间隔毫秒。 */
        channels[4] = (float)samples[i].interval_ms;
        /* 通道5为START后经过时间，由毫秒换算成秒。 */
        channels[5] = (float)samples[i].elapsed_ms / 1000.0f;
        /* 通道6为样本序号。 */
        channels[6] = (float)samples[i].sequence;
        /* 通道7为样本状态枚举值。 */
        channels[7] = (float)samples[i].state;
        /* 8个float依次写到该样本36字节段的0..31偏移。 */
        for (channel = 0; channel < 8; ++channel)
        {
            /* 每个通道占4字节，样本间步长36字节。 */
            offset = i * 36 + channel * 4;
            /* 按float的内存位模式取出32位，再按小端拆成4字节。 */
            /* 复制float位模式，避免类型别名访问改变波形数据。 */
            memcpy(&bits, &channels[channel], sizeof(bits));
            s_ble.heart_frames[offset] = (uint8_t)bits;
            s_ble.heart_frames[offset + 1] = (uint8_t)(bits >> 8);
            s_ble.heart_frames[offset + 2] = (uint8_t)(bits >> 16);
            s_ble.heart_frames[offset + 3] = (uint8_t)(bits >> 24);
        }
        /* 该样本尾部32..35字节写入固定标记00 00 80 7F。 */
        s_ble.heart_frames[i * 36 + 32] = 0;
        s_ble.heart_frames[i * 36 + 33] = 0;
        s_ble.heart_frames[i * 36 + 34] = 0x80;
        s_ble.heart_frames[i * 36 + 35] = 0x7F;
    }
    /* 启动异步发送count个36字节帧，缓冲在结果回收前保持有效。 */
    status = kt6368a_write_async(s_ble.heart_frames, count * 36, APP_BLE_TX_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        /* 启动成功只记录待回收样本数，结果留到下一轮take_tx_result取回。 */
        s_ble.heart_pending_count = count;
    }
    else
    {
        /* 启动失败直接记诊断失败并显示红色错误，不进入待回收状态。 */
        app_diag_heart_tx(count, 0);
        s_ble.tx_state = APP_BLE_TX_HEART_ERROR;
    }
#endif
    return 0;
}

void app_ble_update_power(uint8_t screen_on, uint8_t sport_mode, uint8_t restart_quiet)
{
    /* 先按sport_mode给出期望使能初值：非运动模式为1(开启)，运动模式为0(关闭)。 */
    uint8_t enabled = sport_mode == 0;
    /* 取一次当前毫秒tick，作为安静期判断和重置的统一时间基准。 */
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    if (restart_quiet != 0)
    {
        s_ble.quiet_tick = now_ms;
    }
    /* 期望保持蓝牙开启时，才进入ECO熄屏断电的延迟判断。 */
    if (enabled != 0)
    {
        /* 尚未进入ECO，或屏幕仍亮：把安静计时起点刷新为当前时刻，本轮不断电。 */
        if (s_ble.eco == 0 || screen_on != 0)
        {
            s_ble.quiet_tick = now_ms;
        }
        else
        {
            /* 初始化未完成、命令服务忙、发送在途或接收有待处理数据时，都算活动：重置计时并结束本轮。 */
            if (kt6368a_init_is_pending() != 0 || s_ble.management_busy != 0 ||
                kt6368a_tx_is_busy() != 0 || kt6368a_rx_pending() != 0)
            {
                s_ble.quiet_tick = now_ms;
                return;
            }
            /* 空闲时间还不足APP_BLE_ECO_QUIET_MS，本轮继续等待，不改电源状态。 */
            if ((uint32_t)(now_ms - s_ble.quiet_tick) < APP_BLE_ECO_QUIET_MS)
            {
                return;
            }
            /* 空闲期满，把本轮期望改为断电。 */
            enabled = 0;
        }
    }
    /* 期望值与当前已发布状态一致，无需重复执行电源切换和接收复位。 */
    if (enabled == s_ble.enabled)
    {
        return;
    }
    /* 先记录新的使能状态，再清掉手机就绪标志，等新的完整时钟帧重新置位。 */
    s_ble.enabled = enabled;
    s_ble.phone_ready = 0;
    /* 记录切换前的时钟帧计数作为基线，之后计数增加才说明收到新的完整校时帧。 */
    s_ble.clock_frames = app_clock_get_received_count();
    /* 按enabled启动模块分步初始化，或直接断电并取消未完成的启动。 */
    kt6368a_set_enabled(enabled);
    /* 切换后逐个复位各接收模块的解析状态，丢弃可能残留的半帧。 */
    app_clock_reset_receive();
    app_weather_reset_receive();
    app_diag_reset_receive();
    app_battery_log_ble_reset();
    app_reset_ble_reset();
    app_ota_reset_receive();
    /* 清除接收重同步标志，并把驱动累计的丢弃字节数和UART错误数取回本模块保存。 */
    s_ble.rx_resync = 0;
    s_ble.rx_dropped = kt6368a_get_rx_dropped();
    s_ble.rx_errors = kt6368a_get_rx_errors();
}

uint8_t app_ble_service(uint8_t screen_on, uint8_t sport_mode)
{
    /* 断电或SPORT时也维护天气温度年龄，并让管理模块回收自己的在途结果。 */
    app_ble_receive(sport_mode);
    return app_ble_send(screen_on, sport_mode);
}

void app_ble_set_eco(uint8_t enabled)
{
    s_ble.eco = enabled != 0;
    s_ble.quiet_tick = (uint32_t)xTaskGetTickCount();
}

void app_ble_clear_log(void)
{
    s_ble.log[0] = '\0';
    s_ble.log_length = 0;
    s_ble.rx_bytes = 0;
    s_ble.last_cr = 0;
    ++s_ble.log_revision;
}

void app_ble_send_test(void)
{
#if APP_DEBUG_TELEMETRY
    static const uint8_t message[] = "hello from watch\r\n";
    HAL_StatusTypeDef status;
    status = kt6368a_write(message, sizeof(message) - 1, APP_BLE_TX_TIMEOUT_MS);
    switch (status)
    {
    case HAL_OK:
        s_ble.tx_state = APP_BLE_TX_UART_OK;
        break;
    case HAL_BUSY:
        s_ble.tx_state = APP_BLE_TX_BUSY;
        break;
    case HAL_TIMEOUT:
        s_ble.tx_state = APP_BLE_TX_TIMEOUT;
        break;
    default:
        s_ble.tx_state = APP_BLE_TX_ERROR;
        break;
    }
#endif
}

void app_ble_get_snapshot(app_ble_snapshot_struct *snapshot)
{
    snapshot->device_name = KT6368A_BLE_NAME;
    snapshot->log = s_ble.log;
    snapshot->log_length = s_ble.log_length;
    snapshot->log_revision = s_ble.log_revision;
    snapshot->rx_bytes = s_ble.rx_bytes;
    snapshot->enabled = s_ble.enabled;
    snapshot->eco = s_ble.eco;
    snapshot->tx_state = s_ble.tx_state;
    snapshot->clock_result = app_clock_get_result();
    switch (kt6368a_get_init_status())
    {
    case HAL_OK:
        snapshot->init_result = APP_BLE_INIT_OK;
        break;
    case HAL_BUSY:
        snapshot->init_result = APP_BLE_INIT_BUSY;
        break;
    case HAL_TIMEOUT:
        snapshot->init_result = APP_BLE_INIT_TIMEOUT;
        break;
    default:
        snapshot->init_result = APP_BLE_INIT_ERROR;
        break;
    }
}
