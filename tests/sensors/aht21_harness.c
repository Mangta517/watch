/* 编译真实驱动和服务；检查线上协议、CRC、位拼接、采样等待及故障恢复。 */
#include "aht21.h"
#include "app_aht21.h"
#include "my_i2c.h"
#include "semphr.h"
#include "task.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
    uint8_t command;
    uint8_t data[7];
    uint8_t size;
    uint8_t nack;
} expected_transfer;

static expected_transfer transfers[256];
static unsigned transfer_count, transfer_pos, read_pos, phase, delay_calls;
static uint32_t now_ms;
static uint32_t transfer_ms;
static uint8_t locked;
static int scheduler_state = taskSCHEDULER_NOT_STARTED;
static unsigned task_delay_calls;

static void expect_read(const uint8_t *data, uint8_t size, uint8_t nack)
{
    expected_transfer *transfer = &transfers[transfer_count++];
    assert(transfer_count <= sizeof(transfers) / sizeof(transfers[0]));
    transfer->command = 0;
    transfer->size = size;
    transfer->nack = nack;
    memcpy(transfer->data, data, size);
}

static void expect_command(uint8_t command, uint8_t first, uint8_t nack)
{
    const uint8_t data[2] = {first, 0};
    expect_read(data, sizeof(data), nack);
    transfers[transfer_count - 1].command = command;
}

static void expect_status(uint8_t status, uint8_t nack)
{
    expect_read(&status, 1, nack);
}

static uint8_t frame_crc(const uint8_t *data)
{
    uint16_t remainder = 0xFF;
    unsigned i, bit;
    for (i = 0; i < 6; ++i)
    {
        remainder ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            remainder <<= 1;
            if ((remainder & 0x100U) != 0)
            {
                remainder ^= 0x131U;
            }
        }
    }
    return (uint8_t)remainder;
}

static void expect_frame(uint8_t status, uint32_t humidity, uint32_t temperature, uint8_t corrupt)
{
    uint8_t data[7];
    data[0] = status;
    data[1] = (uint8_t)(humidity >> 12);
    data[2] = (uint8_t)(humidity >> 4);
    data[3] = (uint8_t)((humidity << 4) | (temperature >> 16));
    data[4] = (uint8_t)(temperature >> 8);
    data[5] = (uint8_t)temperature;
    data[6] = frame_crc(data) ^ corrupt;
    expect_read(data, sizeof(data), 0);
}

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

void HAL_Delay(uint32_t ms)
{
    assert(phase == 0 && locked == 0 && scheduler_state != taskSCHEDULER_RUNNING);
    now_ms += ms;
    ++delay_calls;
}

int xTaskGetSchedulerState(void)
{
    return scheduler_state;
}

void vTaskDelay(TickType_t ticks)
{
    assert(phase == 0 && locked == 0 && scheduler_state == taskSCHEDULER_RUNNING && ticks > 0);
    now_ms += ticks;
    ++task_delay_calls;
}

void Error_Handler(void)
{
    abort();
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return &locked;
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    assert(mutex == &locked && locked == 0 && wait == portMAX_DELAY);
    locked = 1;
    return 1;
}

int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex == &locked && locked != 0);
    locked = 0;
    return 1;
}

void my_i2c_start(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase == 0 && locked == 0 && transfer_pos < transfer_count);
    assert(transfers[transfer_pos].command == 0);
    read_pos = 0;
    phase = 1;
}

void my_i2c_write_byte(my_i2c_bus_t bus, uint8_t data)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase == 1 && data == 0x71);
    phase = 2;
}

uint8_t my_i2c_wait_ack(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase == 2);
    phase = 3;
    return transfers[transfer_pos].nack;
}

uint8_t my_i2c_read_byte(my_i2c_bus_t bus, uint8_t ack)
{
    expected_transfer *transfer = &transfers[transfer_pos];
    assert(bus == MY_I2C_BUS_SENSOR && phase == 3 && transfer->nack == 0 && read_pos < transfer->size);
    if (read_pos == transfer->size - 1U)
    {
        assert(ack == 1);
    }
    else
    {
        assert(ack == 0);
    }
    return transfer->data[read_pos++];
}

void my_i2c_stop(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase == 3);
    assert(transfers[transfer_pos].nack != 0 || read_pos == transfers[transfer_pos].size);
    ++transfer_pos;
    phase = 0;
    now_ms += transfer_ms;
}

uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t address, uint8_t command, const uint8_t *data, uint16_t size)
{
    expected_transfer *transfer;
    assert(bus == MY_I2C_BUS_SENSOR && address == 0x38 && phase == 0 && locked == 0 && transfer_pos < transfer_count);
    transfer = &transfers[transfer_pos++];
    assert(command == transfer->command && size == transfer->size && memcmp(data, transfer->data, size) == 0);
    now_ms += transfer_ms;
    return transfer->nack;
}

static void check_init(void)
{
    now_ms = 0;
    expect_status(0x08, 0);
    assert(aht21_init() == AHT21_OK && now_ms == 40);
    expect_status(0x00, 0);
    expect_command(0xBE, 0x08, 0);
    expect_status(0x08, 0);
    assert(aht21_init() == AHT21_OK && now_ms == 90);
    expect_status(0x00, 0);
    expect_command(0xBE, 0x08, 0);
    expect_status(0x00, 0);
    assert(aht21_init() == AHT21_CALIBRATION_ERROR);
    expect_status(0x08, 1);
    assert(aht21_init() == AHT21_I2C_ERROR);
    expect_status(0x88, 0);
    assert(aht21_init() == AHT21_BUSY);
    expect_status(0x00, 0);
    expect_command(0xBE, 0x08, 1);
    assert(aht21_init() == AHT21_I2C_ERROR);
}

static void check_init_delays(void)
{
    const int states[] = {taskSCHEDULER_NOT_STARTED, taskSCHEDULER_RUNNING, taskSCHEDULER_SUSPENDED};
    unsigned i, hal_before, task_before;
    uint32_t started;

    for (i = 0; i < sizeof(states) / sizeof(states[0]); ++i)
    {
        scheduler_state = states[i];
        hal_before = delay_calls;
        task_before = task_delay_calls;
        started = now_ms;
        expect_status(0x00, 0);
        expect_command(0xBE, 0x08, 0);
        expect_status(0x08, 0);
        assert(aht21_init() == AHT21_OK && now_ms - started >= 50);
        if (scheduler_state == taskSCHEDULER_RUNNING)
        {
            assert(task_delay_calls - task_before == 2 && delay_calls == hal_before);
        }
        else
        {
            assert(delay_calls - hal_before == 2 && task_delay_calls == task_before);
        }
    }
    scheduler_state = taskSCHEDULER_NOT_STARTED;
    puts("PASS: AHT21 initialization preserves minimum waits and yields only with a running scheduler");
}

static void check_frames(void)
{
    static const uint8_t fixture[7] = {0x08, 0x80, 0x00, 0x06, 0x00, 0x00, 0xA6};
    aht21_struct sample, previous;
    memset(&sample, 0, sizeof(sample));
    assert(frame_crc(fixture) == 0xA6);
    expect_command(0xAC, 0x33, 0);
    assert(aht21_trigger() == AHT21_OK);
    expect_read(fixture, sizeof(fixture), 0);
    assert(aht21_read(&sample) == AHT21_OK);
    assert(sample.humidity_raw == 0x80000 && sample.temperature_raw == 0x60000 && sample.status == 0x08);
    assert(sample.humidity_rh == 50.0f && sample.temperature_c == 25.0f);
    expect_frame(0x08, 0xABCDE, 0x23456, 0);
    assert(aht21_read(&sample) == AHT21_OK);
    assert(sample.humidity_raw == 0xABCDE && sample.temperature_raw == 0x23456 && sample.temperature_c < 0.0f);
    expect_frame(0x08, 0, 0, 0);
    assert(aht21_read(&sample) == AHT21_OK && sample.humidity_rh == 0.0f && sample.temperature_c == -50.0f);
    expect_frame(0x08, 0xFFFFF, 0xFFFFF, 0);
    assert(aht21_read(&sample) == AHT21_OK && sample.humidity_raw == 0xFFFFF && sample.temperature_raw == 0xFFFFF);
    previous = sample;
    expect_frame(0x08, 0, 0, 1);
    assert(aht21_read(&sample) == AHT21_CRC_ERROR && memcmp(&sample, &previous, sizeof(sample)) == 0);
    expect_frame(0x88, 0, 0, 1);
    assert(aht21_read(&sample) == AHT21_BUSY);
    expect_frame(0x00, 0, 0, 0);
    assert(aht21_read(&sample) == AHT21_CALIBRATION_ERROR);
    expect_status(0x08, 1);
    assert(aht21_read(&sample) == AHT21_I2C_ERROR);
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    assert(aht21_read(NULL) == AHT21_INVALID_ARGUMENT);
}

static void check_service(void)
{
    app_aht21_struct snapshot;
    unsigned initial_delays;
    now_ms = 0;
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid == 0);
    expect_status(0x08, 0);
    app_aht21_init();
    initial_delays = delay_calls;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.measuring != 0 && snapshot.valid == 0);
    now_ms = 119;
    app_aht21_service(); /* 79ms 时队列为空，不能提前访问总线。 */
    now_ms = 120;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid != 0 && snapshot.measuring == 0 && snapshot.sample_count == 1 && snapshot.sample_ms == 120);
    now_ms = APP_AHT21_PERIOD_MS + 39;
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS + 40;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS + 120;
    expect_frame(0x88, 0, 0, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.measuring != 0 && snapshot.valid != 0 && snapshot.sample_count == 1);
    now_ms = APP_AHT21_PERIOD_MS + 240;
    expect_frame(0x88, 0, 0, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.result == AHT21_TIMEOUT && snapshot.valid == 0 && snapshot.measuring == 0);
    now_ms = APP_AHT21_PERIOD_MS * 2 + 40;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS * 2 + 120;
    expect_frame(0x08, 0x80000, 0x60000, 1);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.result == AHT21_CRC_ERROR && snapshot.valid == 0 && snapshot.sample_count == 1);
    now_ms = APP_AHT21_PERIOD_MS * 3 + 40;
    expect_command(0xAC, 0x33, 1);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.result == AHT21_I2C_ERROR && snapshot.valid == 0 && snapshot.measuring == 0);
    now_ms = APP_AHT21_PERIOD_MS * 4 + 40;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS * 4 + 120;
    expect_frame(0x08, 0x40000, 0x20000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.result == AHT21_OK && snapshot.valid != 0 && snapshot.sample_count == 2);
    assert(snapshot.sample.humidity_rh == 25.0f && snapshot.sample.temperature_c == -25.0f);
    assert(delay_calls == initial_delays); /* 周期服务不调用任何毫秒等待。 */
    app_aht21_get_snapshot(NULL);
}

static void check_tick_wrap(void)
{
    app_aht21_struct snapshot;
    now_ms = UINT32_MAX - 60;
    expect_status(0x08, 0);
    app_aht21_init();
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms += 79;
    app_aht21_service();
    ++now_ms;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid != 0 && snapshot.sample_count == 1);
}

static aht21_result checked_recover_step(void)
{
    unsigned before = transfer_pos, hal_waits = delay_calls, task_waits = task_delay_calls;
    uint32_t before_ms = now_ms;
    aht21_result result = aht21_recover_step(now_ms);
    assert(transfer_pos - before <= 1 && now_ms == before_ms);
    assert(delay_calls == hal_waits && task_delay_calls == task_waits);
    return result;
}

static void check_recovery(void)
{
    now_ms = UINT32_MAX - 20;
    aht21_recover_begin(now_ms);
    now_ms += 39;
    assert(checked_recover_step() == AHT21_BUSY);
    ++now_ms;
    assert(checked_recover_step() == AHT21_BUSY);
    expect_status(0, 0);
    assert(checked_recover_step() == AHT21_BUSY);
    expect_command(0xBE, 0x08, 0);
    assert(checked_recover_step() == AHT21_BUSY);
    now_ms += 9;
    assert(checked_recover_step() == AHT21_BUSY);
    ++now_ms;
    assert(checked_recover_step() == AHT21_BUSY);
    expect_status(0x08, 0);
    assert(checked_recover_step() == AHT21_OK);
    aht21_recover_begin(now_ms);
    now_ms += 40;
    assert(checked_recover_step() == AHT21_BUSY);
    expect_status(0x88, 0);
    assert(checked_recover_step() == AHT21_BUSY);
    now_ms += 959;
    expect_status(0x88, 0);
    assert(checked_recover_step() == AHT21_BUSY);
    ++now_ms;
    assert(checked_recover_step() == AHT21_TIMEOUT);
    aht21_recover_begin(now_ms);
    now_ms += 40;
    assert(checked_recover_step() == AHT21_BUSY);
    expect_status(0, 0);
    assert(checked_recover_step() == AHT21_BUSY);
    expect_command(0xBE, 0x08, 1);
    assert(checked_recover_step() == AHT21_I2C_ERROR);
    puts("PASS: AHT21 recovery has minimum waits, one transaction per step, bounded BUSY and tick-wrap handling.");
}

static void check_service_recovery(void)
{
    app_aht21_struct snapshot;
    unsigned waits;
    now_ms = 0;
    expect_status(0x88, 0);
    app_aht21_init();
    waits = delay_calls;
    now_ms = APP_AHT21_PERIOD_MS + 39;
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS + 40;
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS + 80;
    app_aht21_service();
    expect_status(0, 0);
    app_aht21_service();
    expect_command(0xBE, 0x08, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS + 90;
    app_aht21_service();
    expect_status(0x08, 0);
    app_aht21_service();
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS + 170;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.sample_count == 1);
    now_ms = APP_AHT21_PERIOD_MS * 2 + 90;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS * 2 + 170;
    expect_frame(0, 0, 0, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == AHT21_CALIBRATION_ERROR);
    now_ms = APP_AHT21_PERIOD_MS * 3 + 169;
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS * 3 + 170;
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS * 3 + 210;
    app_aht21_service();
    expect_status(0x08, 0);
    app_aht21_service();
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms = APP_AHT21_PERIOD_MS * 3 + 290;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.sample_count == 2 && delay_calls == waits);
    puts("PASS: failed boot and lost calibration recover through the service after cooldown without blocking other sensors.");
    app_aht21_set_period(APP_AHT21_SLEEP_PERIOD_MS);
    now_ms += APP_AHT21_SLEEP_PERIOD_MS - 1;
    app_aht21_service();
    ++now_ms;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms += 80;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.sample_count == 3);
    app_aht21_set_period(APP_AHT21_PERIOD_MS);
    now_ms += APP_AHT21_PERIOD_MS;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms += 80;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.sample_count == 4);
    puts("PASS: AHT21 switches to 1min sleep sampling and resumes the normal 1s trigger/read cycle.");
}

static void check_slow_service_recovery(void)
{
    static const uint32_t periods[] = {20, 40, 100};
    app_aht21_struct snapshot;
    uint32_t i, wrap, started, before_ms;
    unsigned before, waits;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        for (i = 0; i < sizeof(periods) / sizeof(periods[0]); ++i)
        {
            now_ms = 0;
            if (wrap != 0)
            {
                now_ms = UINT32_MAX - APP_AHT21_SLEEP_PERIOD_MS - 100;
            }
            expect_status(0x88, 0);
            app_aht21_init();
            app_aht21_set_period(APP_AHT21_SLEEP_PERIOD_MS);
            now_ms += APP_AHT21_SLEEP_PERIOD_MS;
            started = now_ms;
            waits = delay_calls + task_delay_calls;
            expect_status(0, 0);
            expect_command(0xBE, 0x08, 0);
            expect_status(0x08, 0);
            expect_command(0xAC, 0x33, 0);
            expect_frame(0x08, 0x80000, 0x60000, 0);
            do
            {
                before = transfer_pos;
                before_ms = now_ms;
                app_aht21_service();
                assert(transfer_pos - before <= 1 && now_ms == before_ms);
                app_aht21_get_snapshot(&snapshot);
                assert(snapshot.result != AHT21_TIMEOUT);
                now_ms += periods[i];
            } while (snapshot.valid == 0 && (uint32_t)(now_ms - started) < 1600);
            assert(snapshot.valid != 0 && snapshot.sample_count == 1);
            assert(transfer_pos == transfer_count && delay_calls + task_delay_calls == waits);
        }
    }
    puts("PASS: 20/40/100ms recovery cadence completes calibration and one sample inside the sleep window without blocking.");
}

static void check_sleep_windows(void)
{
    app_aht21_struct snapshot;
    uint32_t wrap, cycle, offset, started, window;
    unsigned before;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        assert(transfer_pos == transfer_count);
        transfer_count = transfer_pos = 0;
        now_ms = 0;
        if (wrap != 0)
        {
            now_ms = UINT32_MAX - APP_AHT21_SLEEP_PERIOD_MS - 100;
        }
        expect_status(0x08, 0);
        app_aht21_init();
        app_aht21_set_period(APP_AHT21_SLEEP_PERIOD_MS);
        started = now_ms;
        transfer_ms = 1;
        for (cycle = 1; cycle <= 24; ++cycle)
        {
            window = started + cycle * APP_AHT21_SLEEP_PERIOD_MS;
            expect_command(0xAC, 0x33, 0);
            expect_frame(0x08, 0x80000 + cycle, 0x60000 + cycle, 0);
            before = transfer_pos;
            for (offset = 0; offset <= 1600; offset += 100)
            {
                now_ms = window + offset;
                app_aht21_service();
            }
            app_aht21_get_snapshot(&snapshot);
            assert(snapshot.valid != 0 && snapshot.measuring == 0);
            assert(snapshot.sample_count == cycle && snapshot.sample.humidity_raw == 0x80000 + cycle);
            assert(snapshot.sample_ms - window <= 200);
            assert(transfer_pos - before == 2 && transfer_pos == transfer_count);
        }
        transfer_ms = 0;
    }
    puts("PASS: 24 fixed 1min sleep windows each publish a fresh sample despite I2C tick cost, including tick wrap.");
}

static void check_expired_conversion(void)
{
    app_aht21_struct snapshot;
    uint32_t wrap, started;
    unsigned before;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        now_ms = 0;
        if (wrap != 0)
        {
            now_ms = UINT32_MAX - 100;
        }
        expect_status(0x08, 0);
        app_aht21_init();
        expect_command(0xAC, 0x33, 0);
        app_aht21_service();
        started = now_ms;
        before = transfer_pos;
        now_ms += 201;
        app_aht21_service();
        app_aht21_get_snapshot(&snapshot);
        assert(transfer_pos == before && snapshot.result == AHT21_TIMEOUT);
        assert(snapshot.valid == 0 && snapshot.measuring == 0 && snapshot.sample_count == 0);
        now_ms = started + APP_AHT21_PERIOD_MS;
        expect_command(0xAC, 0x33, 0);
        app_aht21_service();
        now_ms += 80;
        expect_frame(0x08, 0x80000, 0x60000, 0);
        app_aht21_service();
        app_aht21_get_snapshot(&snapshot);
        assert(snapshot.valid != 0 && snapshot.sample_count == 1);
        now_ms = started + APP_AHT21_PERIOD_MS * 2;
        expect_command(0xAC, 0x33, 0);
        app_aht21_service();
        before = transfer_pos;
        now_ms += APP_AHT21_SLEEP_PERIOD_MS;
        app_aht21_service();
        app_aht21_get_snapshot(&snapshot);
        assert(transfer_pos == before && snapshot.result == AHT21_TIMEOUT);
        assert(snapshot.valid == 0 && snapshot.sample_count == 1 && snapshot.sample_ms == started + APP_AHT21_PERIOD_MS + 80);
        ++now_ms;
        expect_command(0xAC, 0x33, 0);
        app_aht21_service();
        now_ms += 80;
        expect_frame(0x08, 0x40000, 0x20000, 0);
        app_aht21_service();
        app_aht21_get_snapshot(&snapshot);
        assert(snapshot.valid != 0 && snapshot.sample_count == 2);
        before = transfer_pos;
        app_aht21_service();
        assert(transfer_pos == before); /* 跳过多个周期后不连续补跑旧测量。 */
    }
    puts("PASS: expired conversions are rejected without reading or refreshing sample time; the next period recovers across tick wrap.");
}

static void check_sleep_recovery_phase(void)
{
    app_aht21_struct snapshot;
    uint32_t started, cycle, window, offset;
    expect_status(0x88, 0);
    app_aht21_init();
    app_aht21_set_period(APP_AHT21_SLEEP_PERIOD_MS);
    started = now_ms;
    transfer_ms = 1;
    for (cycle = 1; cycle <= 3; ++cycle)
    {
        window = started + cycle * APP_AHT21_SLEEP_PERIOD_MS;
        if (cycle == 1)
        {
            expect_status(0x08, 0);
        }
        expect_command(0xAC, 0x33, 0);
        expect_frame(0x08, 0x80000, 0x60000, 0);
        for (offset = 0; offset <= 1600; offset += 100)
        {
            now_ms = window + offset;
            app_aht21_service();
        }
        app_aht21_get_snapshot(&snapshot);
        assert(snapshot.valid != 0 && snapshot.sample_count == cycle);
        assert(snapshot.sample_ms - window <= 500);
        if (cycle > 1)
        {
            assert(snapshot.sample_ms - window <= 200);
        }
        assert(transfer_pos == transfer_count);
    }
    transfer_ms = 0;
    puts("PASS: recovery finishes inside one sleep window and preserves the following fixed 1min deadlines.");
}

static void check_mode_period(void)
{
    app_aht21_struct snapshot;
    unsigned before;
    expect_status(0x08, 0);
    app_aht21_init();
    app_aht21_set_period(APP_AHT21_SLEEP_PERIOD_MS);
    before = transfer_pos;
    now_ms += APP_AHT21_SLEEP_PERIOD_MS - 1;
    app_aht21_service();
    assert(transfer_pos == before);
    ++now_ms;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms += 80;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_set_period(APP_AHT21_PERIOD_MS);
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms += 80;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.sample_count == 2);
    app_aht21_set_period(APP_AHT21_SLEEP_PERIOD_MS);
    before = transfer_pos;
    now_ms += APP_AHT21_PERIOD_MS;
    app_aht21_service();
    assert(transfer_pos == before);
    now_ms += APP_AHT21_SLEEP_PERIOD_MS - APP_AHT21_PERIOD_MS;
    expect_command(0xAC, 0x33, 0);
    app_aht21_service();
    now_ms += 20;
    app_aht21_set_period(APP_AHT21_PERIOD_MS);
    before = transfer_pos;
    app_aht21_service();
    assert(transfer_pos == before);
    now_ms += 60;
    expect_frame(0x08, 0x80000, 0x60000, 0);
    app_aht21_service();
    app_aht21_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.sample_count == 3);
    puts("PASS: 1min sleep cadence requests a fresh conversion immediately on 1s awake entry and preserves an in-flight conversion.");
}

int main(void)
{
    assert(APP_AHT21_PERIOD_MS == 1000 && APP_AHT21_SLEEP_PERIOD_MS == 60000);
    check_init();
    check_init_delays();
    check_frames();
    check_service();
    check_tick_wrap();
    check_recovery();
    check_service_recovery();
    check_slow_service_recovery();
    check_sleep_windows();
    check_expired_conversion();
    check_sleep_recovery_phase();
    check_mode_period();
    assert(transfer_pos == transfer_count && phase == 0 && locked == 0);
    puts("PASS: AHT21 direct reads/ACK/STOP, initialization, CRC, 20-bit conversion, 80ms wait, 1s sampling, timeout/recovery and tick "
         "wrap.");
    return 0;
}

uint8_t my_i2c_prepare(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && phase == 0);
    return 0;
}
