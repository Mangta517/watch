/**
 * @file app_uptime.c
 * @brief 每10分钟保存本次开机秒数；0xA0/0xB0各16字节，不覆盖磁/IMU校准。
 *        UPT1、小端序号、秒数、CRC32；轮换写入并读回验证，保留另一份记录。
 */
#include "app_uptime.h"
#include "bl24c02.h"
#include "main.h"
#include <string.h>

#define APP_UPTIME_STORE_BASE 0xA0U
#define APP_UPTIME_RECORD_BYTES 16U
#define APP_UPTIME_MAGIC 0x31545055U
#define APP_UPTIME_SAVE_SECONDS 600U

static struct
{
    uint32_t last_ms, seconds, remainder_ms, attempted_seconds;
    uint32_t sequence, saved_seconds;
    uint8_t valid, loaded, target, phase;
    uint8_t record[APP_UPTIME_RECORD_BYTES], verify[APP_UPTIME_RECORD_BYTES];
    bl24c02_transfer_struct io;
} s_uptime;

/* sensor写一次后发布有效位；LVGL只读，避免把本次运行的新记录当成上次记录。 */
static uint32_t s_previous_seconds;
static volatile uint32_t s_previous_valid;

static uint32_t uptime_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void uptime_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint32_t uptime_crc(const uint8_t *data)
{
    uint32_t crc = UINT32_MAX;
    uint8_t i, bit;
    for (i = 0; i < 12; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            if ((crc & 1U) != 0)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc ^ UINT32_MAX;
}

static uint8_t uptime_load(void)
{
    uint8_t record[2][APP_UPTIME_RECORD_BYTES];
    uint8_t valid[2], i, latest = 0;
    uint32_t difference;
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(APP_UPTIME_STORE_BASE + i * APP_UPTIME_RECORD_BYTES, record[i], APP_UPTIME_RECORD_BYTES) != 0)
        {
            return 0; /* 未读清旧副本时不覆盖，下一记录周期再探测。 */
        }
        valid[i] = uptime_get_u32(record[i]) == APP_UPTIME_MAGIC && uptime_get_u32(record[i] + 12) == uptime_crc(record[i]);
    }
    difference = uptime_get_u32(record[1] + 4) - uptime_get_u32(record[0] + 4);
    if (valid[1] != 0 && (valid[0] == 0 || (difference != 0 && difference < 0x80000000U)))
    {
        latest = 1;
    }
    s_uptime.valid = valid[latest];
    if (s_uptime.valid != 0)
    {
        s_uptime.sequence = uptime_get_u32(record[latest] + 4);
        s_uptime.saved_seconds = uptime_get_u32(record[latest] + 8);
        s_uptime.target = latest ^ 1U;
        if (s_previous_valid == 0)
        {
            s_previous_seconds = s_uptime.saved_seconds;
            __DMB();
            s_previous_valid = 1;
        }
    }
    return 1;
}

void app_uptime_init(void)
{
    memset(&s_uptime, 0, sizeof(s_uptime));
    s_previous_valid = 0;
    s_uptime.last_ms = HAL_GetTick();
    s_uptime.seconds = s_uptime.last_ms / 1000U;
    s_uptime.remainder_ms = s_uptime.last_ms % 1000U;
    s_uptime.loaded = uptime_load();
}

uint8_t app_uptime_service(uint8_t storage_available)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - s_uptime.last_ms;
    uint8_t result;
    /* Sleep仍保留HAL节拍；逐拍累计差值，跨49.7天tick回绕仍连续计时。 */
    s_uptime.last_ms = now_ms;
    s_uptime.seconds += elapsed_ms / 1000U;
    s_uptime.remainder_ms += elapsed_ms % 1000U;
    s_uptime.seconds += s_uptime.remainder_ms / 1000U;
    s_uptime.remainder_ms %= 1000U;
    if (s_uptime.phase == 0)
    {
        if (storage_available == 0 || (uint32_t)(s_uptime.seconds - s_uptime.attempted_seconds) < APP_UPTIME_SAVE_SECONDS)
        {
            return 0;
        }
        s_uptime.attempted_seconds = s_uptime.seconds - s_uptime.seconds % APP_UPTIME_SAVE_SECONDS;
        if (s_uptime.loaded == 0)
        {
            s_uptime.loaded = uptime_load();
            if (s_uptime.loaded == 0)
            {
                return 0;
            }
        }
        uptime_put_u32(s_uptime.record, APP_UPTIME_MAGIC);
        uptime_put_u32(s_uptime.record + 4, s_uptime.sequence + 1U);
        uptime_put_u32(s_uptime.record + 8, s_uptime.seconds);
        uptime_put_u32(s_uptime.record + 12, uptime_crc(s_uptime.record));
        bl24c02_transfer_begin(&s_uptime.io, APP_UPTIME_STORE_BASE + s_uptime.target * APP_UPTIME_RECORD_BYTES,
                              s_uptime.record, APP_UPTIME_RECORD_BYTES, 1);
        s_uptime.phase = 1;
        return 1;
    }
    result = bl24c02_transfer_step(&s_uptime.io);
    if (result == 2)
    {
        return 1;
    }
    if (result != 0)
    {
        s_uptime.phase = 0; /* 不连续重试故障器件，下一10分钟周期再写。 */
        return 0;
    }
    if (s_uptime.phase == 1)
    {
        bl24c02_transfer_begin(&s_uptime.io, APP_UPTIME_STORE_BASE + s_uptime.target * APP_UPTIME_RECORD_BYTES,
                              s_uptime.verify, APP_UPTIME_RECORD_BYTES, 0);
        s_uptime.phase = 2;
        return 1;
    }
    if (memcmp(s_uptime.record, s_uptime.verify, APP_UPTIME_RECORD_BYTES) == 0)
    {
        s_uptime.valid = 1;
        s_uptime.saved_seconds = uptime_get_u32(s_uptime.record + 8);
        ++s_uptime.sequence;
        s_uptime.target ^= 1U;
    }
    s_uptime.phase = 0;
    return 0;
}

uint8_t app_uptime_get_saved(uint32_t *seconds)
{
    if (seconds == NULL || s_uptime.valid == 0)
    {
        return 0;
    }
    *seconds = s_uptime.saved_seconds;
    return 1;
}

uint8_t app_uptime_get_previous(uint32_t *seconds)
{
    if (seconds == NULL || s_previous_valid == 0)
    {
        return 0;
    }
    __DMB();
    *seconds = s_previous_seconds;
    return 1;
}
