/* 编译真实电池服务，检查ADC生命周期、分压换算、故障重试与快照同步。 */
#include "app_battery.h"
#include "adc.h"
#include "semphr.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
static uint32_t now_ms;
static uint32_t adc_value;
static uint32_t starts, polls, reads, stops;
static uint8_t locked, allocation_failed;
static unsigned fatal_errors;
static HAL_StatusTypeDef start_result = HAL_OK;
static HAL_StatusTypeDef poll_result = HAL_OK;
static HAL_StatusTypeDef stop_result = HAL_OK;

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

void Error_Handler(void)
{
    ++fatal_errors;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return allocation_failed ? NULL : &locked;
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    assert(mutex == &locked && !locked && wait == portMAX_DELAY);
    locked = 1;
    return 1;
}

int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex == &locked && locked);
    locked = 0;
    return 1;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *adc)
{
    assert(adc == &hadc1 && !locked);
    ++starts;
    return start_result;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout_ms)
{
    assert(adc == &hadc1 && !locked && timeout_ms > 0 && timeout_ms <= 2U);
    ++polls;
    return poll_result;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc)
{
    assert(adc == &hadc1 && !locked && poll_result == HAL_OK);
    ++reads;
    return adc_value;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *adc)
{
    assert(adc == &hadc1 && !locked);
    ++stops;
    return stop_result;
}

static void next_sample(void)
{
    now_ms += APP_BATTERY_PERIOD_MS;
    app_battery_service();
}

static void check_failure(HAL_StatusTypeDef result)
{
    app_battery_struct snapshot;
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == result);
    assert(snapshot.voltage_mv == 0 && snapshot.adc_raw == 0 && !locked);
    assert(snapshot.percent == 0);
}

static void check_percent(void)
{
    app_battery_struct snapshot;
    uint8_t previous_percent;
    unsigned i;
    app_battery_init();
    /* 复现开机3.8V升到3.9V：前7次不显示过渡值，第8次直接显示后4次均值。 */
    adc_value = (3800U * 4095U + 3300U) / 6600U;
    app_battery_service();
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    adc_value = (3900U * 4095U + 3300U) / 6600U;
    for (i = 1; i < 7; ++i)
    {
        next_sample();
        app_battery_get_snapshot(&snapshot);
        assert(!snapshot.valid && snapshot.result == HAL_OK);
    }
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.voltage_mv == 3900 && snapshot.percent == 60);
    /* 放电时电量下降；之后即使电压回到3.9V，也不能回涨。 */
    adc_value = (3750U * 4095U + 3300U) / 6600U;
    for (i = 0; i < 120; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv == 3750 && snapshot.percent == 30);
    adc_value = (3900U * 4095U + 3300U) / 6600U;
    for (i = 0; i < 120; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv == 3900 && snapshot.percent == 30);
    /* ADC故障不丢失电量历史，恢复后也不能回涨。 */
    poll_result = HAL_TIMEOUT;
    next_sample();
    check_failure(HAL_TIMEOUT);
    poll_result = HAL_OK;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.percent == 30);
    previous_percent = snapshot.percent;
    adc_value = 0;
    for (i = 0; i < 120; ++i)
    {
        next_sample();
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.percent <= previous_percent);
        previous_percent = snapshot.percent;
    }
    assert(snapshot.voltage_mv == 0 && snapshot.percent == 0);
    adc_value = 4095;
    for (i = 0; i < 120; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv == 6600 && snapshot.percent == 0);
    puts("PASS: startup 3.8V-to-3.9V settling, discharge-only percentage, voltage rebound and failure recovery.");
}

static void check_full_voltage(void)
{
    static const uint16_t voltages_mv[] = {3300, 3850, 3900, 3950, 3990, 4000, 4100};
    static const uint8_t percentages[] = {0, 50, 60, 80, 96, 100, 100};
    app_battery_struct snapshot;
    unsigned i, sample;
    for (i = 0; i < sizeof(voltages_mv) / sizeof(voltages_mv[0]); ++i)
    {
        app_battery_init();
        adc_value = ((uint32_t)voltages_mv[i] * 4095U + 3300U) / 6600U;
        for (sample = 0; sample < 8; ++sample)
        {
            next_sample();
        }
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.result == HAL_OK);
        assert(snapshot.voltage_mv + 1U >= voltages_mv[i] && snapshot.voltage_mv <= voltages_mv[i] + 1U);
        assert(snapshot.percent == percentages[i]);
    }
    puts("PASS: 4.0V full-charge estimate, upper clamp, 3.9-4.0V interpolation and unchanged low-voltage points.");
}

int main(int argc, char **argv)
{
    app_battery_struct snapshot;
    unsigned i;
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    (void)argv;
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    app_battery_service();
    assert(starts == 0);
    if (argc > 1 && strcmp(argv[1], "percentage") == 0)
    {
        check_full_voltage();
        check_percent();
        return 0;
    }
    if (argc > 1)
    {
        allocation_failed = 1;
        app_battery_init();
        assert(fatal_errors == 1);
        app_battery_service();
        assert(starts == 0);
        puts("PASS: battery mutex allocation failure follows Error_Handler.");
        return 0;
    }
    app_battery_init();
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    adc_value = 2389;
    app_battery_service();
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    for (i = 1; i < 8; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.adc_raw == 2389 && snapshot.voltage_mv == 3850);
    assert(snapshot.percent == 50);
    assert(starts == 8 && polls == 8 && reads == 8 && stops == 8);
    now_ms += APP_BATTERY_PERIOD_MS - 1;
    app_battery_service();
    assert(starts == 8);
    ++now_ms;
    app_battery_service();
    assert(starts == 9);
    adc_value = 0;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.adc_raw == 0 && snapshot.voltage_mv == 3369);
    adc_value = 4095;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.adc_raw == 4095 && snapshot.voltage_mv == 3773);
    for (i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i)
    {
        uint32_t previous_polls = polls;
        uint32_t previous_reads = reads;
        uint32_t previous_stops = stops;
        uint32_t previous_voltage_mv = snapshot.voltage_mv;
        start_result = failures[i];
        next_sample();
        check_failure(failures[i]);
        assert(polls == previous_polls && reads == previous_reads && stops == previous_stops);
        start_result = HAL_OK;
        poll_result = failures[i];
        next_sample();
        check_failure(failures[i]);
        assert(polls == previous_polls + 1 && reads == previous_reads && stops == previous_stops + 1);
        poll_result = HAL_OK;
        stop_result = failures[i];
        next_sample();
        check_failure(failures[i]);
        stop_result = HAL_OK;
        adc_value = 2048;
        next_sample();
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.result == HAL_OK && snapshot.adc_raw == 2048);
        assert(snapshot.voltage_mv > 3301 && snapshot.voltage_mv < previous_voltage_mv);
    }
    /* 即使HAL毫秒计数回绕，仍按一秒间隔采样。 */
    now_ms = UINT32_MAX - 500U;
    app_battery_service();
    {
        uint32_t previous_starts = starts;
        now_ms += 999U;
        app_battery_service();
        assert(starts == previous_starts);
        ++now_ms;
        app_battery_service();
        assert(starts == previous_starts + 1);
    }
    app_battery_get_snapshot(NULL);
    assert(!locked && fatal_errors == 0);
    puts("PASS: battery startup sampling, filtered voltage, ADC cleanup, failures/recovery and tick wrap.");
    return 0;
}
