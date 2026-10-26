#include "app_battery_log.h"
#include "app_battery.h"
#include "int_battery_log.h"
#include "tps3823.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

static SemaphoreHandle_t s_log_mutex;
static struct
{
    app_battery_log_status_struct status;
    uint32_t session;
    uint32_t last_minute;
    uint8_t session_written;
} s_log;

/* CRC16-CCITT多项式0x1021；格式及session参与初值，坏开机标记不会串接到旧曲线。 */
static uint16_t battery_log_crc(const uint8_t *data, uint32_t session)
{
    uint16_t crc = 0xB471;
    uint8_t i, bit;
    if (data[4] != 0xFF || data[5] != 0xFF)
    {
        crc ^= (uint16_t)session ^ (uint16_t)(session >> 16);
    }
    for (i = 0; i < 6; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000) != 0)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint8_t battery_log_decode(uint32_t slot, uint32_t session, app_battery_log_record_struct *record)
{
    uint8_t data[BATTERY_LOG_RECORD_BYTES];
    uint16_t crc;
    int_battery_log_read(slot, data);
    crc = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
    if (crc != battery_log_crc(data, session))
    {
        return 1;
    }
    record->seconds = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    record->voltage_mv = (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    return 0;
}

void app_battery_log_init(void)
{
    uint32_t slot, i;
    uint8_t data[BATTERY_LOG_RECORD_BYTES], occupied;
    app_battery_log_record_struct record;
    if (s_log_mutex == NULL)
    {
        s_log_mutex = xSemaphoreCreateMutex();
    }
    if (s_log_mutex == NULL)
    {
        s_log.status.state = APP_BATTERY_LOG_ERROR;
        return;
    }
    if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == 0)
    {
        return;
    }
    memset(&s_log, 0, sizeof(s_log));
    s_log.status.state = APP_BATTERY_LOG_READY;
    for (slot = 0; slot < BATTERY_LOG_CAPACITY; ++slot)
    {
        int_battery_log_read(slot, data);
        occupied = 0;
        for (i = 0; i < sizeof(data); ++i)
        {
            occupied |= (uint8_t)(data[i] != 0xFF);
        }
        if (occupied == 0)
        {
            continue;
        }
        s_log.status.slots = slot + 1; /* 掉电留下的坏槽也不重复编程。 */
        if (battery_log_decode(slot, s_log.session, &record) == 0)
        {
            if (record.voltage_mv == UINT16_MAX)
            {
                s_log.session = record.seconds;
            }
            else if (s_log.session != 0)
            {
                ++s_log.status.samples;
            }
        }
    }
    /* 旧固件/未知格式区域不自动擦除；首条必须是有效session记录。 */
    if (s_log.status.slots != 0 && (battery_log_decode(0, 0, &record) != 0 || record.voltage_mv != UINT16_MAX || record.seconds == 0))
    {
        s_log.status.state = APP_BATTERY_LOG_NEED_CLEAR;
    }
    else if (s_log.status.slots == BATTERY_LOG_CAPACITY)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
    }
    else if (s_log.session == UINT32_MAX)
    {
        s_log.status.state = APP_BATTERY_LOG_ERROR;
    }
    ++s_log.session;
    xSemaphoreGive(s_log_mutex);
}

static uint8_t battery_log_append(uint32_t seconds, uint16_t voltage_mv)
{
    uint8_t data[BATTERY_LOG_RECORD_BYTES];
    uint32_t i;
    uint16_t crc;
    if (s_log.status.slots >= BATTERY_LOG_CAPACITY)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
        return 1;
    }
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(seconds >> (8 * i));
    }
    data[4] = (uint8_t)voltage_mv;
    data[5] = (uint8_t)(voltage_mv >> 8);
    crc = battery_log_crc(data, s_log.session);
    data[6] = (uint8_t)crc;
    data[7] = (uint8_t)(crc >> 8);
    if (int_battery_log_write(s_log.status.slots, data) != 0)
    {
        s_log.status.state = APP_BATTERY_LOG_ERROR;
        return 1;
    }
    ++s_log.status.slots;
    if (s_log.status.slots == BATTERY_LOG_CAPACITY)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
    }
    return 0;
}

void app_battery_log_service(uint32_t seconds)
{
    app_battery_struct battery;
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return;
    }
    if (s_log.status.state == APP_BATTERY_LOG_READY && seconds / 60 != s_log.last_minute)
    {
        app_battery_get_snapshot(&battery);
        if (battery.valid != 0 && battery.result == HAL_OK && battery.voltage_mv < UINT16_MAX)
        {
            s_log.last_minute = seconds / 60;
            if (s_log.session_written == 0 && battery_log_append(s_log.session, UINT16_MAX) == 0)
            {
                s_log.session_written = 1;
            }
            if (s_log.session_written != 0 && battery_log_append(seconds, (uint16_t)battery.voltage_mv) == 0)
            {
                ++s_log.status.samples;
            }
        }
    }
    xSemaphoreGive(s_log_mutex);
}

uint8_t app_battery_log_get_status(app_battery_log_status_struct *status)
{
    if (s_log_mutex == NULL)
    {
        *status = s_log.status;
        return 0;
    }
    if (xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return 1;
    }
    *status = s_log.status;
    xSemaphoreGive(s_log_mutex);
    return 0;
}

uint8_t app_battery_log_read(uint32_t slot, uint32_t session, app_battery_log_record_struct *record)
{
    uint8_t result = 1;
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return 2;
    }
    if (slot < s_log.status.slots)
    {
        result = battery_log_decode(slot, session, record);
    }
    xSemaphoreGive(s_log_mutex);
    return result;
}

void app_battery_log_clear_and_reset(void)
{
    /* 清空是显式维护操作；擦除后立即重启，避免暂停造成HAL/RTOS时基失真。
     * 擦除期间屏蔽外部看门狗；禁止升级程序擦除此扇区。 */
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return;
    }
    tps3823_disable();
    __disable_irq();
    int_battery_log_erase();
    NVIC_SystemReset();
}
