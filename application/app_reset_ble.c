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

void app_reset_ble_reset(void)
{
    s_ble.length = 0;
    s_ble.overflow = 0;
    s_ble.report = 0;
    s_ble.index = 0;
    s_ble.history_stage = 0;
    /* UART不复制异步缓冲；复位解析器或蓝牙断电均不能改写在途数据。 */
    if (s_ble.pending == 0)
    {
        s_ble.tx_length = 0;
    }
}

void app_reset_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    uint16_t i;
    if ((uint32_t)(now_ms - s_ble.rx_ms) >= 1000)
    {
        s_ble.length = 0;
        s_ble.overflow = 0;
    }
    s_ble.rx_ms = now_ms;
    for (i = 0; i < size; ++i)
    {
        if (data[i] == '\r' || data[i] == '\n')
        {
            s_ble.line[s_ble.length] = '\0';
            if (s_ble.overflow == 0 && s_ble.report == APP_REPORT_NONE && s_ble.pending == 0)
            {
                if (strcmp(s_ble.line, "RESET,READ") == 0)
                {
                    s_ble.report = APP_REPORT_RESET;
                    s_ble.index = 0;
                }
                else if (strcmp(s_ble.line, "STATE,READ") == 0)
                {
                    s_ble.report = APP_REPORT_STATE;
                    s_ble.index = 0;
                }
                else if (strcmp(s_ble.line, "HEART,READ") == 0)
                {
                    s_ble.report = APP_REPORT_HEART;
                    s_ble.index = 0;
                }
                else if (strcmp(s_ble.line, "RESET,HISTORY") == 0)
                {
                    s_ble.report = APP_REPORT_HISTORY;
                    s_ble.history_stage = 0;
                    s_ble.index = 0;
                    s_ble.slot = 0;
                    s_ble.session = 0;
                    s_ble.exported = 0;
                }
            }
            s_ble.length = 0;
            s_ble.overflow = 0;
        }
        else if (data[i] < 32 || data[i] > 126)
        {
            s_ble.overflow = 1;
        }
        else if (s_ble.length < sizeof(s_ble.line) - 1)
        {
            s_ble.line[s_ble.length++] = (char)data[i];
        }
        else
        {
            s_ble.overflow = 1;
        }
    }
}

static int app_state_ble_format(uint32_t now_ms)
{
    uint32_t age_ms = 0;
    switch (s_ble.index)
    {
    case 0:
        /* 与校时、天气接收同属LVGL；整份报告只采样一次，后续发送不改变快照。 */
        watch_rtc_get_status(&s_ble.rtc);
        app_weather_get_snapshot(&s_ble.weather);
        s_ble.snapshot_ms = now_ms;
        s_ble.clock_frames = app_clock_get_received_count();
        s_ble.clock_result = app_clock_get_result();
        return snprintf(s_ble.tx, sizeof(s_ble.tx),
                        "STATE,RTC,READY,%u,LSE,%u,MARKER,0x%08lX,SYNCED,%u,READ,%u,TIME,%02u:%02u:%02u\r\n",
                        (unsigned)s_ble.rtc.ready, (unsigned)s_ble.rtc.lse_ready, (unsigned long)s_ble.rtc.marker,
                        (unsigned)s_ble.rtc.synced, (unsigned)s_ble.rtc.read_status,
                        (unsigned)s_ble.rtc.time.hours, (unsigned)s_ble.rtc.time.minutes, (unsigned)s_ble.rtc.time.seconds);
    case 1:
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "STATE,CLOCK,UP_MS,%lu,RX,%lu,RESULT,%u\r\n",
                        (unsigned long)s_ble.snapshot_ms, (unsigned long)s_ble.clock_frames, (unsigned)s_ble.clock_result);
    case 2:
        if (s_ble.weather.valid != 0)
        {
            age_ms = s_ble.snapshot_ms - s_ble.weather.received_ms;
            if (s_ble.weather.cached != 0)
            {
                age_ms = UINT32_MAX; /* 完全断电的经过时间未知。 */
            }
        }
        return snprintf(s_ble.tx, sizeof(s_ble.tx),
                        "STATE,WEATHER,VALID,%u,CODE,%u,TEMP_X10,%d,RH,%u,RX_MS,%lu,AGE_MS,%lu,CACHED,%u\r\n",
                        (unsigned)s_ble.weather.valid, (unsigned)s_ble.weather.code, (int)s_ble.weather.temperature_x10,
                        (unsigned)s_ble.weather.humidity_rh, (unsigned long)s_ble.weather.received_ms, (unsigned long)age_ms,
                        (unsigned)s_ble.weather.cached);
    default:
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "STATE,END\r\n");
    }
}

static int app_reset_history_format(void)
{
    app_battery_log_status_struct status;
    app_battery_log_record_struct record;
    uint32_t budget = 32;
    uint8_t result, complete;
    if (s_ble.history_stage == 0)
    {
        if (app_battery_log_get_status(&status) != 0)
        {
            return 0;
        }
        s_ble.limit = status.slots;
        if (status.state == APP_BATTERY_LOG_STARTING || status.state == APP_BATTERY_LOG_NEED_CLEAR)
        {
            s_ble.history_stage = 3;
            return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,ERROR,%u\r\n", (unsigned)status.state);
        }
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,BEGIN,%lu,%lu,%u\r\n",
                        (unsigned long)status.resets, (unsigned long)status.slots, (unsigned)status.state);
    }
    while (s_ble.slot < s_ble.limit && budget != 0)
    {
        --budget;
        result = app_battery_log_read(s_ble.slot, s_ble.session, &record);
        if (result == 2)
        {
            return 0;
        }
        if (result == 0 && record.voltage_mv == UINT16_MAX)
        {
            s_ble.session = record.seconds;
        }
        else if (result == 0 && record.voltage_mv == APP_BATTERY_LOG_RESET_TAG &&
                 s_ble.limit - s_ble.slot >= APP_BATTERY_LOG_RESET_SLOTS)
        {
            if (app_battery_log_read_reset(s_ble.slot, s_ble.session, &s_ble.info, &complete) != 0)
            {
                return 0;
            }
            if (complete != 0)
            {
                ++s_ble.exported;
                s_ble.slot += APP_BATTERY_LOG_RESET_SLOTS;
                return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,ENTRY,SLOT,%lu,BOOT,%lu\r\n",
                                (unsigned long)(s_ble.slot - APP_BATTERY_LOG_RESET_SLOTS), (unsigned long)s_ble.session);
            }
        }
        ++s_ble.slot;
    }
    if (s_ble.slot == s_ble.limit)
    {
        s_ble.history_stage = 3;
        return snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,HISTORY,END,%lu\r\n", (unsigned long)s_ble.exported);
    }
    return 0;
}

static uint8_t app_reset_ble_format(uint32_t now_ms)
{
    static const char *const fault_names[] = {"NONE", "READ", "WRITE", "PID", "CONFIG", "DATA_CHANGED"};
    app_heart_struct heart;
    const char *fault_name = "UNKNOWN";
    int length;
    app_reset_info_struct *info = &s_ble.info;
    if (s_ble.report == APP_REPORT_STATE)
    {
        length = app_state_ble_format(now_ms);
    }
    else if (s_ble.report == APP_REPORT_HEART)
    {
        app_heart_get_snapshot(&heart);
        if ((unsigned)heart.fault.kind < sizeof(fault_names) / sizeof(fault_names[0]))
        {
            fault_name = fault_names[heart.fault.kind];
        }
        length = snprintf(s_ble.tx, sizeof(s_ble.tx),
                          "HEART,STATE,%u,EN,%u,FAULT,%u,%s,REG,0x%02X,VALUE,0x%02X\r\n",
                          (unsigned)heart.state, (unsigned)heart.enabled, (unsigned)heart.fault.kind, fault_name,
                          (unsigned)heart.fault.reg, (unsigned)heart.fault.value);
    }
    else if (s_ble.report == APP_REPORT_HISTORY && s_ble.history_stage != 2)
    {
        length = app_reset_history_format();
        if (length == 0)
        {
            return 1; /* 日志锁忙或本轮扫描预算耗尽，保持请求，下一轮继续。 */
        }
    }
    else
    {
        switch (s_ble.index)
        {
        case 0:
            if (s_ble.report == APP_REPORT_RESET)
            {
                app_reset_get_info(info); /* 一次请求只取一份上一启动快照。 */
            }
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,CAUSE,VALID,%lu,FLAGS,0x%08lX,KIND,%lu\r\n",
                              (unsigned long)info->valid, (unsigned long)info->reset_flags, (unsigned long)info->kind);
            break;
        case 1:
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,TIME,UP_MS,%lu,FEED_MS,%lu,GAP_MS,%lu,TASK,%lu\r\n",
                              (unsigned long)info->uptime_ms, (unsigned long)info->last_feed_ms,
                              (unsigned long)info->max_feed_gap_ms, (unsigned long)info->task_tag);
            break;
        case 2:
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,FRAME,SP,0x%08lX,EXC,0x%08lX,PC,0x%08lX,LR,0x%08lX,XPSR,0x%08lX\r\n",
                              (unsigned long)info->sp, (unsigned long)info->exc_return, (unsigned long)info->pc,
                              (unsigned long)info->lr, (unsigned long)info->xpsr);
            break;
        case 3:
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,FAULT,CFSR,0x%08lX,HFSR,0x%08lX,MMFAR,0x%08lX,BFAR,0x%08lX\r\n",
                              (unsigned long)info->cfsr, (unsigned long)info->hfsr, (unsigned long)info->mmfar, (unsigned long)info->bfar);
            break;
        default:
            length = snprintf(s_ble.tx, sizeof(s_ble.tx), "RESET,END\r\n");
            break;
        }
    }
    if (length <= 0 || (unsigned)length >= sizeof(s_ble.tx))
    {
        app_reset_ble_reset();
        return 1;
    }
    s_ble.tx_length = (uint16_t)length;
    return 0;
}

uint8_t app_reset_ble_service(uint8_t ready, uint32_t now_ms)
{
    HAL_StatusTypeDef result;
    if (ready == 0)
    {
        app_reset_ble_reset();
    }
    if (s_ble.pending != 0)
    {
        if (kt6368a_take_tx_result(&result) == 0)
        {
            return 1;
        }
        s_ble.pending = 0;
        s_ble.tx_length = 0;
        if (result != HAL_OK)
        {
            app_reset_ble_reset();
            return 0; /* 失败后等待手机重新查询，不自动重发报告。 */
        }
        if (s_ble.report != 0)
        {
            s_ble.tx_ms = now_ms;
            if (s_ble.report == APP_REPORT_HISTORY)
            {
                if (s_ble.history_stage == 0)
                {
                    s_ble.history_stage = 1;
                }
                else if (s_ble.history_stage == 1)
                {
                    s_ble.history_stage = 2;
                    s_ble.index = 0;
                }
                else if (s_ble.history_stage == 2)
                {
                    ++s_ble.index;
                    if (s_ble.index == 4)
                    {
                        s_ble.history_stage = 1;
                    }
                }
                else
                {
                    app_reset_ble_reset();
                }
            }
            else
            {
                ++s_ble.index;
                if ((s_ble.report == APP_REPORT_RESET && s_ble.index == 5) ||
                    (s_ble.report == APP_REPORT_STATE && s_ble.index == 4) || s_ble.report == APP_REPORT_HEART)
                {
                    app_reset_ble_reset();
                }
            }
        }
    }
    if (ready == 0 || s_ble.report == 0)
    {
        return 0;
    }
    if (s_ble.tx_length == 0 && app_reset_ble_format(now_ms) != 0)
    {
        return s_ble.report != 0;
    }
    if (((s_ble.report != APP_REPORT_HISTORY && s_ble.index == 0) ||
         (uint32_t)(now_ms - s_ble.tx_ms) >= 100) &&
        kt6368a_tx_is_busy() == 0)
    {
        result = kt6368a_write_async((const uint8_t *)s_ble.tx, s_ble.tx_length, 100);
        if (result == HAL_OK)
        {
            s_ble.pending = 1;
        }
        else if (result != HAL_BUSY)
        {
            app_reset_ble_reset();
        }
    }
    return s_ble.report != 0 || s_ble.pending != 0;
}
