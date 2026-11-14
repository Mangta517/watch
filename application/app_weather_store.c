#include "app_weather_store.h"
#include "app_weather.h"
#include "bl24c02.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define WEATHER_STORE_BASE         0xD0 /* 0xC0..0xC3保留给Bootloader有效标志。 */
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

/* 小端读32位值：data[0]为最低字节，依次左移8/16/24位后按位或。 */
static uint32_t weather_store_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

/* 小端写32位值：第i字节取value从8*i位起的低8位，共写4字节。 */
static void weather_store_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    /* 从最低字节开始逐字节写入。 */
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (8 * i));
    }
}

/* 对记录前12字节计算CRC32，初值与最终异或均为0xFFFFFFFF。 */
static uint32_t weather_store_crc(const uint8_t *data)
{
    uint32_t crc = UINT32_MAX;
    uint8_t i, bit;
    /* 逐字节异或进CRC低8位。 */
    for (i = 0; i < 12; ++i)
    {
        crc ^= data[i];
        /* 每字节处理8位：最低位为1时右移并异或多项式0xEDB88320，为0时只右移。 */
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
    /* 结果按位取反后返回。 */
    return crc ^ UINT32_MAX;
}

/* 两条有效天气的码/温度/湿度全同返回1，否则0；任一无效则返回0。 */
static uint8_t weather_store_same(const app_weather_struct *a, const app_weather_struct *b)
{
    /* 两份都须有效，且天气码、温度x10、湿度三者全部相同才返回1。 */
    return a->valid != 0 && b->valid != 0 && a->code == b->code && a->temperature_x10 == b->temperature_x10 &&
           a->humidity_rh == b->humidity_rh;
}

/* 扫描0xC0/0xD0/0xE0三槽(0xC0仅读兼容旧记录)，逐条MAGIC+CRC+量程校验，按无符号序号取最新；
 * target选下一个轮换写入槽(与最新所在的0xD0/0xE0错开)，返回1表示至少一槽读失败(未读全时不写新记录)。 */
static uint8_t weather_store_load(void)
{
    uint8_t record[3][WEATHER_STORE_RECORD_BYTES];
    app_weather_struct weather[3] = {0};
    uint8_t i, latest = 0, result = 0;
    uint32_t difference;
    /* 逐槽读取16字节记录：0xC0、0xD0、0xE0。 */
    for (i = 0; i < 3; ++i)
    {
        /* 只读兼容旧0xC0记录；新记录仅写0xD0/0xE0。 */
        if (bl24c02_read(0xC0 + i * WEATHER_STORE_RECORD_BYTES, record[i], WEATHER_STORE_RECORD_BYTES) != 0)
        {
            /* 本槽读失败：置读错误标记并跳过本槽校验，继续读下一槽。 */
            result = 1;
            continue;
        }
        /* 偏移8..9小端拼出温度x10，偏移10为湿度百分比，偏移11为WMO天气码。 */
        weather[i].temperature_x10 = (int16_t)((uint16_t)record[i][8] | ((uint16_t)record[i][9] << 8));
        weather[i].humidity_rh = record[i][10];
        weather[i].code = record[i][11];
        /* 同时满足格式字、CRC、温度-1000..1000、湿度0..100、天气码0..99时本槽有效。 */
        weather[i].valid = weather_store_get_u32(record[i]) == WEATHER_STORE_MAGIC &&
                           weather_store_get_u32(record[i] + 12) == weather_store_crc(record[i]) &&
                           weather[i].temperature_x10 >= -1000 && weather[i].temperature_x10 <= 1000 &&
                           weather[i].humidity_rh <= 100 && weather[i].code <= 99;
    }
    /* 以槽0(0xC0旧记录)为初始候选，再与槽1、槽2(0xD0/0xE0)按序号比较取最新。 */
    for (i = 1; i < 3; ++i)
    {
        if (weather[i].valid == 0)
        {
            continue;
        }
        /* 当前最新槽无效时，直接用本槽替换。 */
        if (weather[latest].valid == 0)
        {
            latest = i;
        }
        else
        {
            /* 两槽都有效：本槽序号减最新槽序号，差值非0且小于0x80000000表示本槽更新。 */
            difference = weather_store_get_u32(record[i] + 4) - weather_store_get_u32(record[latest] + 4);
            if (difference != 0 && difference < 0x80000000U)
            {
                latest = i;
            }
        }
    }
    /* 找到有效记录时恢复保存值、序号和下一写入槽。 */
    if (weather[latest].valid != 0)
    {
        s_store.saved = weather[latest];
        s_store.sequence = weather_store_get_u32(record[latest] + 4);
        /* 最新记录在0xD0时下次写0xE0，在0xE0时下次写0xD0；仅0xC0旧记录有效时下次写0xD0。 */
        s_store.target = 0;
        if (latest == 1)
        {
            s_store.target = 1;
        }
        /* 在短临界区内把恢复快照复制到显示侧读取的s_restored。 */
        taskENTER_CRITICAL();
        s_restored = s_store.saved;
        taskEXIT_CRITICAL();
    }
    return result;
}

/* sensor启动时清本地状态并尝试读三槽；不清发布邮箱。读不全时loaded=0，
 * 后续取得待存快照且存储可用时再重试读取。 */
void app_weather_store_init(void)
{
    /* 清零保存状态、序号、槽位和事务结构。 */
    memset(&s_store, 0, sizeof(s_store));
    /* 不清发布邮箱：手机帧可能在sensor初始化完成前到达。 */
    s_store.loaded = weather_store_load() == 0;
}

/* LVGL每轮调用：先把恢复记录发布给显示，再取当前手机天气；内容或时间戳变化时
 * 经短临界区交给sensor保存，不直接访问EEPROM。 */
void app_weather_store_sync(void)
{
    app_weather_struct weather;
    /* 在短临界区内复制sensor恢复的天气快照。 */
    taskENTER_CRITICAL();
    weather = s_restored;
    taskEXIT_CRITICAL();
    /* 把恢复记录交给天气模块作为当前快照。 */
    app_weather_restore(&weather);
    /* 再取显示任务当前持有的最新天气快照。 */
    app_weather_get_snapshot(&weather);
    /* 仅当快照有效、不是EEPROM恢复的旧标记，且内容或接收tick与上次发布不同才发布。 */
    if (weather.valid != 0 && weather.cached == 0 &&
        (weather_store_same(&weather, &s_published) == 0 || weather.received_ms != s_published.received_ms))
    {
        /* 在短临界区内把完整快照写入待存邮箱并置有效位。 */
        taskENTER_CRITICAL();
        s_pending = weather;
        s_pending_valid = 1;
        taskEXIT_CRITICAL();
        /* 记录本次发布内容，供下轮判断变化。 */
        s_published = weather;
    }
}

/* sensor分步推进：phase=0取待存快照并准备记录，phase=1写EEPROM，phase=2读回校验。
 * 返回1表示仍占用EEPROM(调用方须暂停其他保存)，0表示本轮无事务或已结束。 */
uint8_t app_weather_store_service(uint8_t storage_available)
{
    app_weather_struct weather;
    uint8_t pending, result;
    /* phase=0表示当前没有进行中的EEPROM事务，先看是否有待存快照。 */
    if (s_store.phase == 0)
    {
        /* 存储不可用时推迟新事务，返回0表示不占用EEPROM。 */
        if (storage_available == 0)
        {
            return 0;
        }
        /* 在短临界区内取出待存快照和有效位，并清有效位表示已取走。 */
        taskENTER_CRITICAL();
        pending = s_pending_valid;
        weather = s_pending;
        s_pending_valid = 0;
        taskEXIT_CRITICAL();
        /* 没有待存快照时直接返回。 */
        if (pending == 0)
        {
            return 0;
        }
        /* 启动时未读清记录的话，这里补读三槽；仍失败就放弃本轮写入。 */
        if (s_store.loaded == 0)
        {
            s_store.loaded = weather_store_load() == 0;
            if (s_store.loaded == 0)
            {
                return 0; /* 未读清三槽记录时不写；下一手机帧再尝试。 */
            }
        }
        /* 与已保存内容完全相同时不重复写EEPROM。 */
        if (weather_store_same(&weather, &s_store.saved) != 0)
        {
            return 0;
        }
        /* 保存本次待写快照，供读回校验成功后更新saved。 */
        s_store.writing = weather;
        /* 组装16字节记录：0..3为WTH1，4..7为下一序号，8..9温度x10小端，10湿度，11天气码，12..15 CRC32。 */
        weather_store_put_u32(s_store.record, WEATHER_STORE_MAGIC);
        weather_store_put_u32(s_store.record + 4, s_store.sequence + 1);
        s_store.record[8] = (uint8_t)weather.temperature_x10;
        s_store.record[9] = (uint8_t)((uint16_t)weather.temperature_x10 >> 8);
        s_store.record[10] = weather.humidity_rh;
        s_store.record[11] = weather.code;
        weather_store_put_u32(s_store.record + 12, weather_store_crc(s_store.record));
        /* 在target指向的槽(0xD0/0xE0)启动写事务，写入刚组好的16字节记录。 */
        bl24c02_transfer_begin(&s_store.io, WEATHER_STORE_BASE + s_store.target * WEATHER_STORE_RECORD_BYTES,
                               s_store.record, WEATHER_STORE_RECORD_BYTES, 1);
        /* 标记进入写阶段，并告知调用方EEPROM被占用。 */
        s_store.phase = 1;
        return 1;
    }
    /* 已有事务：推进至多一笔EEPROM操作。 */
    result = bl24c02_transfer_step(&s_store.io);
    /* 返回2表示事务仍在进行，继续占用EEPROM。 */
    if (result == 2)
    {
        return 1;
    }
    /* 非0非2表示本笔失败：回到空闲，等新的手机帧再请求。 */
    if (result != 0)
    {
        s_store.phase = 0; /* 不在每轮重试；新手机帧可重新请求。 */
        return 0;
    }
    /* 写阶段完成：从同一槽启动读回事务，准备逐字节比对。 */
    if (s_store.phase == 1)
    {
        bl24c02_transfer_begin(&s_store.io, WEATHER_STORE_BASE + s_store.target * WEATHER_STORE_RECORD_BYTES,
                               s_store.verify, WEATHER_STORE_RECORD_BYTES, 0);
        /* 切到读回校验阶段。 */
        s_store.phase = 2;
        return 1;
    }
    /* 读回内容与写入内容一致时，确认本槽记录有效。 */
    if (memcmp(s_store.record, s_store.verify, WEATHER_STORE_RECORD_BYTES) == 0)
    {
        /* 已保存快照更新为本次写入的天气。 */
        s_store.saved = s_store.writing;
        /* 序号递增，供下次写入。 */
        ++s_store.sequence;
        /* 下次写另一槽，保留本次记录。 */
        s_store.target ^= 1;
    }
    /* 无论校验是否一致，事务结束回到空闲。 */
    s_store.phase = 0;
    return 0;
}
