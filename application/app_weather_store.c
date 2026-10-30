#include "app_weather_store.h"
#include "app_weather.h"
#include "bl24c02.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define WEATHER_STORE_BASE         0xC0
#define WEATHER_STORE_RECORD_BYTES 16
#define WEATHER_STORE_MAGIC        0x31485457 /* 小端WTH1；偏移4序号、8温度、10湿度、11天气码、12 CRC32。 */

static struct
{
    uint32_t sequence;
    uint8_t loaded, target, phase;
    uint8_t record[WEATHER_STORE_RECORD_BYTES], verify[WEATHER_STORE_RECORD_BYTES];
    app_weather_struct saved, writing;
    bl24c02_transfer_struct io;
} s_store;

/* 双任务只在短临界区交换完整快照，EEPROM操作全部留在sensor。 */
static app_weather_struct s_restored, s_pending;
static uint8_t s_pending_valid;
static app_weather_struct s_published; /* 仅LVGL读写。 */

static uint32_t weather_store_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void weather_store_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (8 * i));
    }
}

static uint32_t weather_store_crc(const uint8_t *data)
{
    uint32_t crc = UINT32_MAX;
    uint8_t i, bit;
    for (i = 0; i < 12; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            if ((crc & 1) != 0)
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

static uint8_t weather_store_same(const app_weather_struct *a, const app_weather_struct *b)
{
    return a->valid != 0 && b->valid != 0 && a->code == b->code && a->temperature_x10 == b->temperature_x10 &&
           a->humidity_rh == b->humidity_rh;
}

static uint8_t weather_store_load(void)
{
    uint8_t record[2][WEATHER_STORE_RECORD_BYTES];
    app_weather_struct weather[2] = {0};
    uint8_t i, latest = 0, result = 0;
    uint32_t difference;
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(WEATHER_STORE_BASE + i * WEATHER_STORE_RECORD_BYTES, record[i], WEATHER_STORE_RECORD_BYTES) != 0)
        {
            result = 1;
            continue;
        }
        weather[i].temperature_x10 = (int16_t)((uint16_t)record[i][8] | ((uint16_t)record[i][9] << 8));
        weather[i].humidity_rh = record[i][10];
        weather[i].code = record[i][11];
        weather[i].valid = weather_store_get_u32(record[i]) == WEATHER_STORE_MAGIC &&
                           weather_store_get_u32(record[i] + 12) == weather_store_crc(record[i]) &&
                           weather[i].temperature_x10 >= -1000 && weather[i].temperature_x10 <= 1000 &&
                           weather[i].humidity_rh <= 100 && weather[i].code <= 99;
    }
    if (weather[1].valid != 0)
    {
        if (weather[0].valid == 0)
        {
            latest = 1;
        }
        else
        {
            difference = weather_store_get_u32(record[1] + 4) - weather_store_get_u32(record[0] + 4);
            if (difference != 0 && difference < 0x80000000U)
            {
                latest = 1;
            }
        }
    }
    if (weather[latest].valid != 0)
    {
        s_store.saved = weather[latest];
        s_store.sequence = weather_store_get_u32(record[latest] + 4);
        s_store.target = latest ^ 1;
        taskENTER_CRITICAL();
        s_restored = s_store.saved;
        taskEXIT_CRITICAL();
    }
    return result;
}

void app_weather_store_init(void)
{
    memset(&s_store, 0, sizeof(s_store));
    /* 不清发布邮箱：手机帧可能在sensor初始化完成前到达。 */
    s_store.loaded = weather_store_load() == 0;
}

void app_weather_store_sync(void)
{
    app_weather_struct weather;
    taskENTER_CRITICAL();
    weather = s_restored;
    taskEXIT_CRITICAL();
    app_weather_restore(&weather);
    app_weather_get_snapshot(&weather);
    if (weather.valid != 0 && weather.cached == 0 &&
        (weather_store_same(&weather, &s_published) == 0 || weather.received_ms != s_published.received_ms))
    {
        taskENTER_CRITICAL();
        s_pending = weather;
        s_pending_valid = 1;
        taskEXIT_CRITICAL();
        s_published = weather;
    }
}

uint8_t app_weather_store_service(uint8_t storage_available)
{
    app_weather_struct weather;
    uint8_t pending, result;
    if (s_store.phase == 0)
    {
        if (storage_available == 0)
        {
            return 0;
        }
        taskENTER_CRITICAL();
        pending = s_pending_valid;
        weather = s_pending;
        s_pending_valid = 0;
        taskEXIT_CRITICAL();
        if (pending == 0)
        {
            return 0;
        }
        if (s_store.loaded == 0)
        {
            s_store.loaded = weather_store_load() == 0;
            if (s_store.loaded == 0)
            {
                return 0; /* 未读清两个副本时不写；下一手机帧再尝试。 */
            }
        }
        if (weather_store_same(&weather, &s_store.saved) != 0)
        {
            return 0;
        }
        s_store.writing = weather;
        weather_store_put_u32(s_store.record, WEATHER_STORE_MAGIC);
        weather_store_put_u32(s_store.record + 4, s_store.sequence + 1);
        s_store.record[8] = (uint8_t)weather.temperature_x10;
        s_store.record[9] = (uint8_t)((uint16_t)weather.temperature_x10 >> 8);
        s_store.record[10] = weather.humidity_rh;
        s_store.record[11] = weather.code;
        weather_store_put_u32(s_store.record + 12, weather_store_crc(s_store.record));
        bl24c02_transfer_begin(&s_store.io, WEATHER_STORE_BASE + s_store.target * WEATHER_STORE_RECORD_BYTES,
                               s_store.record, WEATHER_STORE_RECORD_BYTES, 1);
        s_store.phase = 1;
        return 1;
    }
    result = bl24c02_transfer_step(&s_store.io);
    if (result == 2)
    {
        return 1;
    }
    if (result != 0)
    {
        s_store.phase = 0; /* 不在每轮重试；新手机帧可重新请求。 */
        return 0;
    }
    if (s_store.phase == 1)
    {
        bl24c02_transfer_begin(&s_store.io, WEATHER_STORE_BASE + s_store.target * WEATHER_STORE_RECORD_BYTES,
                               s_store.verify, WEATHER_STORE_RECORD_BYTES, 0);
        s_store.phase = 2;
        return 1;
    }
    if (memcmp(s_store.record, s_store.verify, WEATHER_STORE_RECORD_BYTES) == 0)
    {
        s_store.saved = s_store.writing;
        ++s_store.sequence;
        s_store.target ^= 1;
    }
    s_store.phase = 0;
    return 0;
}
