#include "app_battery_log.h"
#include "app_battery_log_ble.h"
#include "app_reset_ble.h"
#include "app_clock.h"
#include "app_weather.h"
#include "watch_rtc.h"
#include "app_battery.h"
#include "int_battery_log.h"
#include "semphr.h"
#include "kt6368a.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <windows.h>

FLASH_TypeDef host_flash;
static uint8_t mutex_locked, allocation_failed, flash_locked = 1, irq_mask;
static uint8_t tps_disabled, erase_failed, unlock_failed, corrupt_write;
static int program_remaining = -1;
static unsigned programs, erases, cache_resets;
static app_battery_struct battery = {3900, 0, HAL_OK, 1, 70};
static jmp_buf reset_target;
static uint8_t reset_expected;
static uint8_t tx_active, tx_done, tx_busy;
static HAL_StatusTypeDef tx_start = HAL_OK, tx_result = HAL_OK;
static const uint8_t *tx_buffer;
static uint16_t tx_size;
static char tx_copy[128], output[32768];
static uint32_t tick;
static unsigned tx_count;
static app_reset_info_struct reset_info;

void app_reset_get_info(app_reset_info_struct *info)
{
    *info = reset_info;
}
void watch_rtc_get_status(watch_rtc_status_struct *value)
{
    memset(value, 0, sizeof(*value));
}
void app_weather_get_snapshot(app_weather_struct *value)
{
    memset(value, 0, sizeof(*value));
}
app_clock_result app_clock_get_result(void)
{
    return APP_CLOCK_NONE;
}
uint32_t app_clock_get_received_count(void)
{
    return 0;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    if (allocation_failed != 0)
    {
        return NULL;
    }
    return &mutex_locked;
}
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    (void)wait;
    assert(mutex == &mutex_locked);
    if (mutex_locked != 0)
    {
        return 0;
    }
    mutex_locked = 1;
    return 1;
}
int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex == &mutex_locked && mutex_locked != 0);
    mutex_locked = 0;
    return 1;
}
void app_battery_get_snapshot(app_battery_struct *snapshot)
{
    *snapshot = battery;
}
uint32_t __get_PRIMASK(void)
{
    return irq_mask;
}
void __disable_irq(void)
{
    irq_mask = 1;
}
void __set_PRIMASK(uint32_t mask)
{
    irq_mask = (uint8_t)mask;
}
void host_cache_reset(void)
{
    assert((FLASH->ACR & FLASH_ACR_DCEN) == 0);
    ++cache_resets;
}
void tps3823_disable(void)
{
    tps_disabled = 1;
}
void NVIC_SystemReset(void)
{
    assert(reset_expected && tps_disabled && irq_mask && flash_locked);
    longjmp(reset_target, 1);
}
HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
    if (unlock_failed != 0)
    {
        return HAL_ERROR;
    }
    flash_locked = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
    flash_locked = 1;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value)
{
    uint16_t *dest = (uint16_t *)(uintptr_t)address;
    assert(!irq_mask && mutex_locked && !flash_locked && type == FLASH_TYPEPROGRAM_HALFWORD);
    assert(address >= BATTERY_LOG_START_ADDR && address + 2U <= BATTERY_LOG_END_ADDR && (address & 1U) == 0);
    if (program_remaining == 0)
    {
        return HAL_ERROR;
    }
    if (program_remaining > 0)
    {
        --program_remaining;
    }
    assert((*dest & value) == value);
    *dest = (uint16_t)value;
    if (corrupt_write != 0)
    {
        *dest ^= 1U;
    }
    ++programs;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error)
{
    assert(tps_disabled && irq_mask && !flash_locked);
    assert(erase->Sector == 7 && erase->NbSectors == 1 && erase->VoltageRange == 3);
    ++erases;
    *error = UINT32_MAX;
    if (erase_failed != 0)
    {
        return HAL_ERROR;
    }
    memset((void *)(uintptr_t)BATTERY_LOG_START_ADDR, 0xFF, BATTERY_LOG_END_ADDR - BATTERY_LOG_START_ADDR);
    return HAL_OK;
}
uint8_t kt6368a_tx_is_busy(void)
{
    return tx_busy || tx_active;
}
HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    assert(!tx_active && size < sizeof(tx_copy) && timeout_ms == 100);
    if (tx_start != HAL_OK)
    {
        return tx_start;
    }
    tx_buffer = data;
    tx_size = size;
    memcpy(tx_copy, data, size);
    tx_copy[size] = '\0';
    tx_active = 1;
    tx_done = 0;
    ++tx_count;
    return HAL_OK;
}
uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    assert(tx_active);
    assert(memcmp(tx_copy, tx_buffer, tx_size) == 0);
    if (tx_done == 0)
    {
        return 0;
    }
    *result = tx_result;
    tx_active = tx_done = 0;
    if (tx_result == HAL_OK)
    {
        assert(strlen(output) + tx_size < sizeof(output));
        strcat(output, tx_copy);
    }
    return 1;
}

static app_battery_log_status_struct status(void)
{
    app_battery_log_status_struct value;
    assert(app_battery_log_get_status(&value) == 0);
    return value;
}
static void command(const char *text)
{
    app_battery_log_ble_receive((const uint8_t *)text, (uint16_t)strlen(text), tick);
}
static void pump(unsigned steps)
{
    for (unsigned i = 0; i < steps; ++i)
    {
        tick += 100;
        if (tx_active != 0)
        {
            tx_done = 1;
        }
        app_battery_log_ble_service(1, tick);
    }
}
static void record_tests(void)
{
    app_battery_log_record_struct record;
    app_battery_log_init();
    assert(status().state == APP_BATTERY_LOG_READY && programs == 0 && erases == 0);
    app_battery_log_service(59);
    assert(programs == 0);
    battery.valid = 0;
    app_battery_log_service(60);
    assert(programs == 0);
    battery.valid = 1;
    app_battery_log_service(61);
    assert(status().samples == 1 && status().slots == 2 && programs == 8);
    assert(app_battery_log_read(0, 0, &record) == 0 && record.seconds == 1 && record.voltage_mv == UINT16_MAX);
    assert(app_battery_log_read(1, 1, &record) == 0 && record.seconds == 61 && record.voltage_mv == 3900);
    assert(app_battery_log_read(1, 2, &record) == 1);
    app_battery_log_service(119);
    assert(status().samples == 1);
    battery.voltage_mv = 3850;
    app_battery_log_service(240);
    assert(status().samples == 2 && status().slots == 3); /* 不补造缺失分钟。 */
    app_battery_log_init();
    assert(status().samples == 2 && programs == 12);
    app_battery_log_service(60);
    assert(app_battery_log_read(3, 0, &record) == 0 && record.seconds == 2 && record.voltage_mv == UINT16_MAX);
    assert(app_battery_log_read(4, 2, &record) == 0 && record.seconds == 60);
    assert(status().samples == 3 && erases == 0 && irq_mask == 0 && flash_locked);
    assert((FLASH->ACR & FLASH_ACR_DCEN) != 0 && cache_resets == 5);
    mutex_locked = 1;
    assert(app_battery_log_read(1, 1, &record) == 2);
    app_battery_log_service(120);
    mutex_locked = 0;
    assert(status().samples == 3);
    puts("PASS: minute cadence, filtered mV, invalid samples, no catch-up, reboot sessions, CRC context, mutex and cache.");
}
static void fault_tests(void)
{
    app_battery_log_record_struct record;
    app_battery_log_init();
    app_battery_log_service(60);
    for (int halfwords = 0; halfwords < 4; ++halfwords)
    {
        program_remaining = halfwords;
        app_battery_log_service(120 + (uint32_t)halfwords * 60);
        assert(status().state == APP_BATTERY_LOG_ERROR && flash_locked && !irq_mask);
        program_remaining = -1;
        app_battery_log_init();
        app_battery_log_service(60);
        assert(status().state == APP_BATTERY_LOG_READY);
    }
    *(uint8_t *)(uintptr_t)(BATTERY_LOG_START_ADDR + 8U + 4U) ^= 1U;
    app_battery_log_init();
    assert(app_battery_log_read(1, 1, &record) == 1);
    app_battery_log_service(60);
    assert(status().samples >= 1 && erases == 0);
    unlock_failed = 1;
    app_battery_log_service(120);
    assert(status().state == APP_BATTERY_LOG_ERROR && flash_locked && !irq_mask);
    unlock_failed = 0;
    app_battery_log_init();
    corrupt_write = 1;
    app_battery_log_service(60);
    assert(status().state == APP_BATTERY_LOG_ERROR && flash_locked);
    puts("PASS: torn writes at every halfword, CRC rejection, no overwrite, unlock/readback errors and reboot recovery.");
}
static void full_tests(void)
{
    app_battery_log_init();
    for (uint32_t minute = 1; minute < BATTERY_LOG_CAPACITY; ++minute)
    {
        app_battery_log_service(minute * 60);
    }
    assert(status().slots == BATTERY_LOG_CAPACITY && status().samples == BATTERY_LOG_CAPACITY - 1);
    assert(status().state == APP_BATTERY_LOG_FULL && erases == 0);
    unsigned before = programs;
    app_battery_log_service(BATTERY_LOG_CAPACITY * 60);
    app_battery_log_init();
    app_battery_log_service(60);
    assert(programs == before && status().state == APP_BATTERY_LOG_FULL);
    uint8_t data[8] = {0};
    assert(int_battery_log_write(BATTERY_LOG_CAPACITY, data) == 1);
    assert(int_battery_log_write(0, data) == 1);
    puts("PASS: full 128 KiB capacity, stop without erase, reboot stays full, boundary and occupied-slot rejection.");
}
static void ble_tests(void)
{
    app_battery_log_init();
    app_battery_log_service(60);
    app_battery_log_service(120);
    command("BATLOG,RE");
    pump(1);
    assert(tx_count == 0);
    command("AD\r\n");
    tx_busy = 1;
    pump(2);
    assert(tx_count == 0);
    tx_busy = 0;
    pump(1);
    assert(tx_active);
    command("BATLOG,STATUS\n"); /* 在途缓冲保持不变；下一行优先处理状态。 */
    pump(4);
    assert(strstr(output, "BATLOG,STATUS,2,3,16384,1\r\n"));
    output[0] = '\0';
    command("BATLOG,READ\n");
    pump(1);
    app_battery_log_service(180); /* BEGIN后的新点留给下一次READ。 */
    tx_done = 1;
    mutex_locked = 1;
    app_battery_log_ble_service(1, tick + 100);
    mutex_locked = 0;
    assert(!strstr(output, "BATLOG,DATA"));
    pump(15);
    assert(strstr(output, "BATLOG,BEGIN,2,3\r\n"));
    assert(strstr(output, "BATLOG,DATA,1,1,60,3900\r\n"));
    assert(strstr(output, "BATLOG,DATA,2,1,120,3900\r\n"));
    assert(strstr(output, "BATLOG,END,2\r\n"));
    assert(!strstr(output, "BATLOG,DATA,3,"));
    output[0] = '\0';
    command("BATLOG,READ\n");
    pump(1);
    app_battery_log_ble_reset();
    tx_done = 1;
    assert(app_battery_log_ble_service(0, tick + 100) == 0);
    assert(!tx_active);
    unsigned before = tx_count;
    command("BATLOG,CLE");
    tick += 1001;
    command("AR\n");
    command("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXBATLOG,CLEAR\n");
    const uint8_t binary[] = "BATLOG,CLEAR\0junk\n";
    app_battery_log_ble_receive(binary, sizeof(binary) - 1, tick);
    pump(3);
    assert(tx_count == before && erases == 0);
    tx_start = HAL_BUSY;
    command("BATLOG,STATUS\n");
    pump(2);
    assert(tx_count == before);
    tx_start = HAL_OK;
    pump(1);
    tx_result = HAL_TIMEOUT;
    pump(3);
    assert(!tx_active && tx_count == before + 1);
    tx_result = HAL_OK;
    command("BATLOG,STOP\n");
    pump(3);
    assert(strstr(output, "BATLOG,STOPPED\r\n"));
    puts("PASS: fragmented commands, framing/overflow/binary rejection, BUSY/timeout, stable async buffers, lock contention and complete export.");
}
static void clear_tests(void)
{
    app_battery_log_init();
    app_battery_log_service(60);
    command("BATLOG,CLEAR\n");
    pump(1);
    assert(tx_active && erases == 0 && !tps_disabled);
    tx_done = 1;
    tick += 100;
    app_battery_log_ble_service(1, tick);
    assert(strstr(output, "BATLOG,CLEARING_REBOOT") && erases == 0);
    mutex_locked = 1;
    pump(4);
    assert(erases == 0);
    mutex_locked = 0;
    reset_expected = 1;
    if (setjmp(reset_target) == 0)
    {
        pump(1);
        assert(0);
    }
    assert(erases == 1);
    for (uint32_t address = 0x08000000; address < BATTERY_LOG_START_ADDR; ++address)
    {
        assert(*(const uint8_t *)(uintptr_t)address == 0x5A);
    }
    mutex_locked = irq_mask = 0;
    app_battery_log_init();
    if (erase_failed != 0)
    {
        assert(status().samples == 1);
    }
    else
    {
        assert(status().samples == 0 && status().slots == 0);
    }
    puts("PASS: clear waits for UART completion and mutex, masks TPS3823, erases only sector 7, resets even after erase failure.");
}
static void history_command(const char *text)
{
    app_reset_ble_receive((const uint8_t *)text, (uint16_t)strlen(text), tick);
}

static void history_pump(void)
{
    unsigned budget = 1024;
    while (budget != 0)
    {
        --budget;
        tick += 100;
        if (tx_active != 0)
        {
            tx_done = 1;
        }
        if (app_reset_ble_service(1, tick) == 0)
        {
            break;
        }
    }
    assert(budget != 0 && !tx_active);
}

static void set_reset_info(void)
{
    reset_info.reset_flags = 0x0E000000;
    reset_info.valid = 1;
    reset_info.kind = APP_RESET_HARDFAULT;
    reset_info.pc = 0x08012345;
    reset_info.lr = 0x08022223;
    reset_info.sp = 0x2001FFE0;
    reset_info.uptime_ms = 1234567;
    reset_info.last_feed_ms = 1234500;
    reset_info.max_feed_gap_ms = 221;
    reset_info.cfsr = 0x8200;
    reset_info.hfsr = 0x40000000;
}

static void reset_tests(void)
{
    app_reset_info_struct saved;
    app_battery_log_record_struct record;
    uint8_t original[16];
    uint8_t complete;
    app_battery_log_init();
    app_battery_log_service(60); /* 模拟旧版本留下的正常电压格式。 */
    memcpy(original, (const void *)(uintptr_t)BATTERY_LOG_START_ADDR, sizeof(original));
    set_reset_info();
    app_battery_log_init();
    app_battery_log_service(9);
    assert(status().slots == 2);
    battery.valid = 0;
    app_battery_log_service(10);
    assert(status().slots == 2);
    battery.valid = 1;
    battery.result = HAL_ERROR;
    app_battery_log_service(11);
    assert(status().slots == 2);
    battery.result = HAL_OK;
    mutex_locked = 1;
    app_battery_log_service(12);
    mutex_locked = 0;
    app_battery_log_service(13);
    assert(status().slots == 20 && status().samples == 1 && status().resets == 1);
    assert(memcmp(original, (const void *)(uintptr_t)BATTERY_LOG_START_ADDR, sizeof(original)) == 0);
    assert(app_battery_log_read_reset(3, 2, &saved, &complete) == 0 && complete && memcmp(&saved, &reset_info, sizeof(saved)) == 0);
    assert(app_battery_log_read_reset(3, 1, &saved, &complete) == 0 && !complete);
    assert(app_battery_log_read_reset(UINT32_MAX, 2, &saved, &complete) == 0 && !complete);
    mutex_locked = 1;
    assert(app_battery_log_read_reset(3, 2, &saved, &complete) == 1);
    mutex_locked = 0;
    unsigned before = programs;
    app_battery_log_service(14);
    assert(programs == before); /* 本次开机不重复保存。 */
    for (uint32_t i = 0; i < APP_BATTERY_LOG_RESET_SLOTS; ++i)
    {
        uint8_t *byte = (uint8_t *)(uintptr_t)(BATTERY_LOG_START_ADDR + (3 + i) * 8);
        *byte ^= 1;
        assert(app_battery_log_read_reset(3, 2, &saved, &complete) == 0 && !complete);
        *byte ^= 1;
    }
    app_battery_log_service(60);
    assert(status().samples == 2 && status().slots == 21);
    assert(app_battery_log_read(20, 2, &record) == 0 && record.voltage_mv == 3900);
    memset(&reset_info, 0, sizeof(reset_info)); /* 模拟备份域完全丢失。 */
    reset_info.reset_flags = 0x08000000;
    app_battery_log_init();
    assert(status().resets == 1 && status().samples == 2);
    app_battery_log_service(10);
    assert(status().resets == 2 && status().slots == 39);
    assert(app_battery_log_read_reset(22, 3, &saved, &complete) == 0 && complete && saved.valid == 0 && saved.reset_flags == 0x08000000);
    assert(app_battery_log_read_reset(3, 2, &saved, &complete) == 0 && complete && saved.pc == 0x08012345);

    command("BATLOG,READ\n");
    pump(10);
    assert(strstr(output, "BATLOG,BEGIN,2,39\r\n") && strstr(output, "BATLOG,END,2\r\n"));
    assert(strstr(output, "65280") == NULL);
    output[0] = '\0';
    history_command("RESET,HIS");
    assert(app_reset_ble_service(1, tick) == 0);
    history_command("TORY\r\n");
    mutex_locked = 1;
    assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
    mutex_locked = 0;
    tx_start = HAL_BUSY;
    assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
    tx_start = HAL_OK;
    history_pump();
    assert(strstr(output, "RESET,HISTORY,BEGIN,2,39,1\r\n"));
    assert(strstr(output, "RESET,HISTORY,ENTRY,SLOT,3,BOOT,2\r\n"));
    assert(strstr(output, "RESET,HISTORY,ENTRY,SLOT,22,BOOT,3\r\n"));
    assert(strstr(output, "PC,0x08012345") && strstr(output, "VALID,0,FLAGS,0x08000000,KIND,0"));
    assert(strstr(output, "RESET,HISTORY,END,2\r\n"));
    assert(erases == 0 && flash_locked && !irq_mask);
    puts("PASS: reset history preserves old voltage data, waits for valid startup voltage, saves once, survives backup loss, checks all CRCs and exports full scenes over BLE.");
}

static void reset_torn_tests(void)
{
    for (int halfwords = 0; halfwords < 72; ++halfwords)
    {
        memset((void *)(uintptr_t)BATTERY_LOG_START_ADDR, 0xFF, 0x20000);
        memset(&reset_info, 0, sizeof(reset_info));
        app_battery_log_init();
        app_battery_log_service(60);
        uint8_t original[16];
        memcpy(original, (const void *)(uintptr_t)BATTERY_LOG_START_ADDR, sizeof(original));
        set_reset_info();
        app_battery_log_init();
        program_remaining = halfwords;
        app_battery_log_service(10);
        assert(status().state == APP_BATTERY_LOG_ERROR && status().resets == 0);
        assert(memcmp(original, (const void *)(uintptr_t)BATTERY_LOG_START_ADDR, sizeof(original)) == 0);
        program_remaining = -1;
        app_battery_log_init();
        assert(status().resets == 0 && status().samples == 1);
        app_battery_log_service(10);
        assert(status().resets == 1 && status().state == APP_BATTERY_LOG_READY);
        app_battery_log_init();
        assert(status().resets == 1 && status().samples == 1 && erases == 0);
    }
    puts("PASS: power interruption at all 72 halfword boundaries rejects partial reset entries, preserves old data and resumes after reboot without erasing.");
}

static void reset_full_tests(void)
{
    app_battery_log_init();
    for (uint32_t minute = 1; minute < BATTERY_LOG_CAPACITY - 17; ++minute)
    {
        app_battery_log_service(minute * 60);
    }
    assert(status().slots == BATTERY_LOG_CAPACITY - 17);
    set_reset_info();
    app_battery_log_init();
    unsigned before = programs;
    app_battery_log_service(10);
    assert(status().state == APP_BATTERY_LOG_FULL && status().resets == 0 && programs == before && erases == 0);
    puts("PASS: insufficient space for a complete reset entry stops before programming and never erases existing records.");
}

static void reset_history_tests(void)
{
    app_battery_log_init();
    history_command("RESET,HISTORY\n");
    history_pump();
    assert(strcmp(output, "RESET,HISTORY,BEGIN,0,0,1\r\nRESET,HISTORY,END,0\r\n") == 0);
    for (uint32_t minute = 1; minute <= 70; ++minute)
    {
        app_battery_log_service(minute * 60);
    }
    output[0] = '\0';
    history_command("RESET,HISTORY,EXTRA\n");
    assert(app_reset_ble_service(1, tick) == 0);
    history_command("RESET,HISTORY\n");
    tick += 100;
    assert(app_reset_ble_service(1, tick) == 1 && tx_active);
    tx_done = 1;
    tick += 100;
    mutex_locked = 1;
    assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
    mutex_locked = 0;
    app_battery_log_service(71 * 60); /* BEGIN后的新槽留到下一次请求。 */
    assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
    assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
    history_pump();
    assert(strcmp(output, "RESET,HISTORY,BEGIN,0,71,1\r\nRESET,HISTORY,END,0\r\n") == 0);

    set_reset_info();
    app_battery_log_init();
    app_battery_log_service(10);
    for (unsigned mode = 0; mode < 3; ++mode)
    {
        history_command("RESET,HISTORY\n");
        tick += 100;
        assert(app_reset_ble_service(1, tick) == 1 && tx_active);
        if (mode == 0)
        {
            app_reset_ble_reset();
            assert(app_reset_ble_service(0, tick) == 1);
        }
        tx_result = HAL_OK;
        if (mode == 1)
        {
            tx_result = HAL_ERROR;
        }
        tx_done = 1;
        tick += 100;
        if (mode < 2)
        {
            assert(app_reset_ble_service(1, tick) == 0);
            unsigned before = tx_count;
            assert(app_reset_ble_service(1, tick + 10000) == 0 && tx_count == before);
        }
        else
        {
            assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
            unsigned budget = 4;
            while (tx_active == 0 && budget != 0)
            {
                --budget;
                tick += 100;
                assert(app_reset_ble_service(1, tick) == 1);
            }
            assert(tx_active);
            tx_done = 1;
            tick += 100;
            assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
            tick += 99;
            assert(app_reset_ble_service(1, tick) == 1 && !tx_active);
            tick += 1;
            assert(app_reset_ble_service(1, tick) == 1 && tx_active);
            history_pump();
        }
    }
    tx_result = HAL_OK;
    memset((void *)(uintptr_t)BATTERY_LOG_START_ADDR, 0x12, 8);
    app_battery_log_init();
    output[0] = '\0';
    history_command("RESET,HISTORY\n");
    history_pump();
    assert(strcmp(output, "RESET,HISTORY,ERROR,3\r\n") == 0);
    puts("PASS: history handles empty/unknown logs, bounded scans, snapshot limits, lock contention, malformed commands, 100ms pacing, UART errors and power abort without retries.");
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    void *mapped = VirtualAlloc((void *)(uintptr_t)0x08000000, 0x80000,
                               MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    assert(mapped == (void *)(uintptr_t)0x08000000);
    memset(mapped, 0x5A, 0x60000);
    memset((void *)(uintptr_t)BATTERY_LOG_START_ADDR, 0xFF, 0x20000);
    FLASH->ACR = FLASH_ACR_DCEN;
    if (strcmp(argv[1], "reset_history") == 0)
    {
        reset_history_tests();
    }
    else if (strcmp(argv[1], "reset") == 0)
    {
        reset_tests();
    }
    else if (strcmp(argv[1], "reset_torn") == 0)
    {
        reset_torn_tests();
    }
    else if (strcmp(argv[1], "reset_full") == 0)
    {
        reset_full_tests();
    }
    else if (strcmp(argv[1], "record") == 0)
    {
        record_tests();
    }
    else if (strcmp(argv[1], "full") == 0)
    {
        full_tests();
    }
    else if (strcmp(argv[1], "faults") == 0)
    {
        fault_tests();
    }
    else if (strcmp(argv[1], "unknown") == 0)
    {
        memset((void *)(uintptr_t)BATTERY_LOG_START_ADDR, 0x12, 8);
        app_battery_log_init();
        app_battery_log_service(600);
        assert(status().state == APP_BATTERY_LOG_NEED_CLEAR && programs == 0 && erases == 0);
        command("BATLOG,READ\n");
        pump(4);
        assert(strstr(output, "BATLOG,ERROR,NEED_CLEAR"));
        puts("PASS: unknown occupied region requires explicit clear and is never auto-erased.");
    }
    else if (strcmp(argv[1], "allocation") == 0)
    {
        allocation_failed = 1;
        app_battery_log_init();
        app_battery_log_service(60);
        assert(status().state == APP_BATTERY_LOG_ERROR && programs == 0);
        command("BATLOG,STATUS\n");
        pump(3);
        assert(strstr(output, "BATLOG,STATUS,0,0,16384,4"));
        puts("PASS: mutex allocation error is visible and does not write Flash.");
    }
    else if (strcmp(argv[1], "ble") == 0)
    {
        ble_tests();
    }
    else
    {
        if (strcmp(argv[1], "erase_error") == 0)
        {
            erase_failed = 1;
        }
        clear_tests();
    }
    VirtualFree(mapped, 0, MEM_RELEASE);
    return 0;
}
