#include "app_battery_log_ble.h"
#include "app_battery_log.h"
#include "int_battery_log.h"
#include "kt6368a.h"
#include <stdio.h>
#include <string.h>

enum
{
    BATLOG_NONE,
    BATLOG_STATUS,
    BATLOG_READ,
    BATLOG_CLEAR,
    BATLOG_STOP
};

static struct
{
    char line[24];
    char tx[112];
    uint32_t rx_ms, tx_ms, index, limit, samples, session, next_index;
    uint16_t length, tx_length;
    uint8_t overflow, command, streaming, pending, after_send, clear_wait;
} s_ble;

void app_battery_log_ble_reset(void)
{
    s_ble.length = 0;
    s_ble.overflow = 0;
    s_ble.command = BATLOG_NONE;
    s_ble.streaming = 0;
    s_ble.clear_wait = 0;
    /* 异步TX缓冲必须保留到结果回收；断电也不能让其他发送者取走结果。 */
    if (s_ble.pending == 0)
    {
        s_ble.tx_length = 0;
    }
    s_ble.after_send = 0;
}

void app_battery_log_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
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
            if (s_ble.overflow == 0 && s_ble.command == BATLOG_NONE && s_ble.clear_wait == 0)
            {
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

uint8_t app_battery_log_ble_service(uint8_t ready, uint32_t now_ms)
{
    app_battery_log_status_struct status;
    app_battery_log_record_struct record;
    HAL_StatusTypeDef result;
    uint32_t budget = 32;
    uint8_t read_result;
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
            app_battery_log_ble_reset();
            return 0; /* 发送失败后等手机重新请求，不无限重发。 */
        }
        s_ble.index = s_ble.next_index;
        s_ble.tx_ms = now_ms;
        if (s_ble.after_send == BATLOG_CLEAR)
        {
            s_ble.clear_wait = 1;
        }
        s_ble.after_send = 0;
    }
    if (ready == 0)
    {
        app_battery_log_ble_reset();
        return 0;
    }
    if (s_ble.clear_wait != 0)
    {
        if ((uint32_t)(now_ms - s_ble.tx_ms) >= 300)
        {
            app_battery_log_clear_and_reset();
        }
        return 1;
    }
    if (s_ble.tx_length == 0 && s_ble.command != BATLOG_NONE)
    {
        if (app_battery_log_get_status(&status) != 0)
        {
            return 1;
        }
        s_ble.streaming = 0;
        if (s_ble.command == BATLOG_STATUS)
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,STATUS,%lu,%lu,%lu,%u\r\n",
                                                 (unsigned long)status.samples, (unsigned long)status.slots,
                                                 (unsigned long)BATTERY_LOG_CAPACITY, (unsigned)status.state);
        }
        else if (s_ble.command == BATLOG_READ && status.state == APP_BATTERY_LOG_NEED_CLEAR)
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,ERROR,NEED_CLEAR\r\n");
        }
        else if (s_ble.command == BATLOG_READ)
        {
            s_ble.index = 0;
            s_ble.next_index = 0;
            s_ble.limit = status.slots;
            s_ble.samples = status.samples;
            s_ble.session = 0;
            s_ble.streaming = 1;
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,BEGIN,%lu,%lu\r\n",
                                                 (unsigned long)s_ble.samples, (unsigned long)s_ble.limit);
        }
        else if (s_ble.command == BATLOG_CLEAR)
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,CLEARING_REBOOT\r\n");
            s_ble.after_send = BATLOG_CLEAR;
        }
        else
        {
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,STOPPED\r\n");
        }
        s_ble.command = BATLOG_NONE;
    }
    if (s_ble.tx_length == 0 && s_ble.streaming != 0)
    {
        while (s_ble.index < s_ble.limit && budget != 0)
        {
            --budget;
            read_result = app_battery_log_read(s_ble.index, s_ble.session, &record);
            if (read_result == 2)
            {
                return 1;
            }
            if (read_result == 0)
            {
                if (record.voltage_mv == UINT16_MAX)
                {
                    s_ble.session = record.seconds;
                }
                else if (s_ble.session != 0 && record.voltage_mv < APP_BATTERY_LOG_RESET_TAG)
                {
                    s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,DATA,%lu,%lu,%lu,%u\r\n",
                                                         (unsigned long)s_ble.index, (unsigned long)s_ble.session,
                                                         (unsigned long)record.seconds, (unsigned)record.voltage_mv);
                    s_ble.next_index = s_ble.index + 1;
                    break;
                }
            }
            ++s_ble.index;
        }
        if (s_ble.tx_length == 0 && s_ble.index == s_ble.limit)
        {
            s_ble.streaming = 0;
            s_ble.next_index = s_ble.index;
            s_ble.tx_length = (uint16_t)snprintf(s_ble.tx, sizeof(s_ble.tx), "BATLOG,END,%lu\r\n", (unsigned long)s_ble.samples);
        }
    }
    if (s_ble.tx_length != 0 && (uint32_t)(now_ms - s_ble.tx_ms) >= 100 && kt6368a_tx_is_busy() == 0)
    {
        result = kt6368a_write_async((const uint8_t *)s_ble.tx, s_ble.tx_length, 100);
        if (result == HAL_OK)
        {
            s_ble.pending = 1;
        }
        else if (result != HAL_BUSY)
        {
            app_battery_log_ble_reset();
        }
    }
    return s_ble.pending != 0 || s_ble.tx_length != 0 || s_ble.streaming != 0;
}
