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
    assert(snapshot.valid && snapshot.adc_raw == 2389 && snapshot.voltage_mv == 3850);
    assert(starts == 1 && polls == 1 && reads == 1 && stops == 1);
    now_ms = APP_BATTERY_PERIOD_MS - 1;
    app_battery_service();
    assert(starts == 1);
    ++now_ms;
    app_battery_service();
    assert(starts == 2);
    adc_value = 0;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.voltage_mv == 0);
    adc_value = 4095;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.voltage_mv == 6600);
    for (i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i)
    {
        uint32_t previous_polls = polls;
        uint32_t previous_reads = reads;
        uint32_t previous_stops = stops;
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
        assert(snapshot.valid && snapshot.result == HAL_OK && snapshot.voltage_mv == 3301);
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
    puts("PASS: battery initial sample, one-second cadence, ADC cleanup, millivolt conversion, failures/recovery and tick wrap.");
    return 0;
}
