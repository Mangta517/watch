/* 真实模块配合可断电/写保护的EEPROM模型，重启只清BSS，保留器件内容。 */
#include <assert.h>
#include <stdio.h>
#include "../../application/app_weather.c"
#include "../../application/app_weather_store.c"

static uint8_t eeprom[256], protected_data[0xC0];
static uint8_t io_active, write_failed, verify_failed, read_failed;
static unsigned writes, critical_depth;

void vPortEnterCritical(void)
{
    ++critical_depth;
}

void vPortExitCritical(void)
{
    assert(critical_depth != 0);
    --critical_depth;
}

uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length)
{
    assert(critical_depth == 0 && io_active == 0);
    assert((address == 0xC0 || address == 0xD0) && length == 16);
    if (read_failed != 0)
    {
        return 1;
    }
    memcpy(data, eeprom + address, length);
    return 0;
}

void bl24c02_transfer_begin(bl24c02_transfer_struct *io, uint16_t address, uint8_t *data, uint16_t length, uint8_t write)
{
    assert(critical_depth == 0 && io_active == 0);
    assert((address == 0xC0 || address == 0xD0) && length == 16);
    io_active = 1;
    io->address = address;
    io->data = data;
    io->remaining = length;
    io->write = write;
    io->phase = 0;
}

uint8_t bl24c02_transfer_step(bl24c02_transfer_struct *io)
{
    assert(critical_depth == 0 && io_active != 0);
    if (io->phase++ == 0)
    {
        return 2;
    }
    io_active = 0;
    if (io->write != 0)
    {
        if (write_failed != 0)
        {
            return 1;
        }
        memcpy(eeprom + io->address, io->data, io->remaining);
        ++writes;
    }
    else
    {
        memcpy(io->data, eeprom + io->address, io->remaining);
        if (verify_failed != 0)
        {
            io->data[0] ^= 1;
        }
    }
    return 0;
}

static void cold_boot(void)
{
    assert(critical_depth == 0);
    io_active = 0;
    memset(&s_weather, 0, sizeof(s_weather));
    memset(&s_store, 0, sizeof(s_store));
    memset(&s_restored, 0, sizeof(s_restored));
    memset(&s_pending, 0, sizeof(s_pending));
    memset(&s_published, 0, sizeof(s_published));
    s_pending_valid = 0;
    app_weather_reset_receive();
    app_weather_store_init();
    app_weather_store_sync();
}

static void receive_weather(const char *frame, uint32_t now_ms)
{
    app_weather_receive((const uint8_t *)frame, (uint16_t)strlen(frame), now_ms);
    app_weather_store_sync();
}

static void finish_save(void)
{
    unsigned step;
    for (step = 0; step < 32; ++step)
    {
        if (app_weather_store_service(1) == 0 && io_active == 0 && s_pending_valid == 0)
        {
            break;
        }
    }
    assert(step < 32 && critical_depth == 0);
    assert(memcmp(eeprom, protected_data, sizeof(protected_data)) == 0);
    for (step = 0xE0; step < sizeof(eeprom); ++step)
    {
        assert(eeprom[step] == 0xFF);
    }
}

static void expect_weather(uint8_t code, int16_t temperature_x10, uint8_t humidity, uint8_t cached)
{
    app_weather_struct weather;
    app_weather_get_snapshot(&weather);
    assert(weather.valid && weather.code == code && weather.temperature_x10 == temperature_x10);
    assert(weather.humidity_rh == humidity && weather.cached == cached);
    if (cached != 0)
    {
        assert(weather.received_ms == 0);
    }
}

int main(void)
{
    uint8_t baseline[256], next_record[16];
    unsigned before, cut;
    memset(eeprom, 0xFF, sizeof(eeprom));
    for (cut = 0; cut < sizeof(protected_data); ++cut)
    {
        protected_data[cut] = (uint8_t)cut;
    }
    memcpy(eeprom, protected_data, sizeof(protected_data));
    cold_boot();
    assert(s_weather.valid == 0 && writes == 0);
    receive_weather("WEATHER,3,-53,81\n", 100);
    assert(app_weather_store_service(0) == 0 && writes == 0);
    assert(app_weather_store_service(1) != 0);
    while (app_weather_store_service(0) != 0)
    {
        /* 在途保存即使资源不再空闲也须完成，调度器据busy位阻止其他方访问。 */
    }
    assert(writes == 1 && memcmp(eeprom + 0xC0, "WTH1", 4) == 0);
    cold_boot();
    expect_weather(3, -53, 81, 1);
    finish_save();
    assert(writes == 1);
    receive_weather("WEATHER,3,-53,81\n", 1);
    expect_weather(3, -53, 81, 0);
    finish_save();
    assert(writes == 1); /* 同内容只更新RAM的新鲜度，不磨损EEPROM。 */
    receive_weather("WEATHER,99,-1000,100\n", 2);
    finish_save();
    assert(writes == 2);
    cold_boot();
    expect_weather(99, -1000, 100, 1);
    puts("PASS: cold-boot restore, negative temperature, deduplication, busy completion and EEPROM partition isolation.");

    eeprom[0xD8] ^= 1;
    cold_boot();
    expect_weather(3, -53, 81, 1);
    memcpy(baseline, eeprom, sizeof(eeprom));
    receive_weather("WEATHER,1,200,70\n", 3);
    finish_save();
    memcpy(next_record, eeprom + 0xD0, sizeof(next_record));
    for (cut = 0; cut <= sizeof(next_record); ++cut)
    {
        memcpy(eeprom, baseline, sizeof(eeprom));
        memcpy(eeprom + 0xD0, next_record, cut);
        cold_boot();
        if (cut == sizeof(next_record))
        {
            expect_weather(1, 200, 70, 1);
        }
        else
        {
            expect_weather(3, -53, 81, 1);
        }
    }
    puts("PASS: CRC fallback and power cut after every byte of the replacement record preserve a complete copy.");

    /* 新帧在旧帧写入期间到达；在途缓冲不变，结束后保存最新一份。 */
    before = writes;
    receive_weather("WEATHER,2,250,50\n", 10);
    assert(app_weather_store_service(1) != 0);
    receive_weather("WEATHER,61,260,60\n", 11);
    receive_weather("WEATHER,95,270,65\n", 12);
    finish_save();
    assert(writes == before + 2);
    cold_boot();
    expect_weather(95, 270, 65, 1);
    receive_weather("WEATHER,100,0,0\n", 13);
    receive_weather("WEATHER,1,1001,0\n", 14);
    receive_weather("WEATHER,1,0,101\n", 15);
    before = writes;
    finish_save();
    assert(writes == before);
    expect_weather(95, 270, 65, 1);
    puts("PASS: latest pending update survives an in-flight write; invalid frames never replace the cached record.");

    receive_weather("WEATHER,0,1000,0\n", 20);
    write_failed = 1;
    finish_save();
    before = writes;
    finish_save();
    assert(writes == before);
    write_failed = 0;
    cold_boot();
    expect_weather(95, 270, 65, 1);
    receive_weather("WEATHER,0,1000,0\n", 21);
    finish_save();
    cold_boot();
    expect_weather(0, 1000, 0, 1);

    receive_weather("WEATHER,1,150,25\n", 22);
    verify_failed = 1;
    finish_save();
    verify_failed = 0;
    receive_weather("WEATHER,2,160,30\n", 23);
    finish_save();
    cold_boot();
    expect_weather(2, 160, 30, 1);
    puts("PASS: write protection and readback mismatch retain the committed state and recover on later phone frames.");

    read_failed = 1;
    cold_boot();
    assert(s_weather.valid == 0);
    receive_weather("WEATHER,3,170,35\n", 30);
    before = writes;
    finish_save();
    assert(writes == before);
    read_failed = 0;
    receive_weather("WEATHER,3,170,35\n", 31);
    finish_save();
    cold_boot();
    expect_weather(3, 170, 35, 1);

    /* 手机新帧先于sensor初始化；后到的EEPROM缓存不能覆盖它。 */
    receive_weather("WEATHER,45,-20,90\n", 40);
    app_weather_store_init();
    app_weather_store_sync();
    expect_weather(45, -20, 90, 0);
    finish_save();
    cold_boot();
    expect_weather(45, -20, 90, 1);

    /* 序号回绕时，0比UINT32_MAX更新。 */
    weather_store_put_u32(eeprom + 0xC4, UINT32_MAX);
    weather_store_put_u32(eeprom + 0xCC, weather_store_crc(eeprom + 0xC0));
    weather_store_put_u32(eeprom + 0xD4, 0);
    weather_store_put_u32(eeprom + 0xDC, weather_store_crc(eeprom + 0xD0));
    cold_boot();
    assert(s_store.sequence == 0 && s_store.target == 0);
    finish_save();
    puts("PASS: read failures never overwrite unread copies; early phone frames and sequence wrap recover correctly.");
    return 0;
}
