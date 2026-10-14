/**
 * @file app_battery.c
 * @brief 每秒进行一次 PA1/ADC1_IN1 单次转换，不在 UI 中等待 ADC。
 */
#include "app_battery.h"
#include "app_diag.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

#define APP_BATTERY_ADC_TIMEOUT_MS 2U

static SemaphoreHandle_t s_battery_mux;
static app_battery_struct s_battery_data;
static uint32_t s_attempt_ms;

void app_battery_init(void)
{
    s_battery_mux = xSemaphoreCreateMutex();
    if (s_battery_mux == NULL)
    {
        Error_Handler();
        return;
    }
    xSemaphoreTake(s_battery_mux, portMAX_DELAY);
    memset(&s_battery_data, 0, sizeof(s_battery_data));
    xSemaphoreGive(s_battery_mux);
    s_attempt_ms = HAL_GetTick() - APP_BATTERY_PERIOD_MS;
    app_diag_sensor_init(APP_DIAG_BATTERY, 0);
}

void app_battery_service(void)
{
    app_battery_struct sample = {0};
    HAL_StatusTypeDef stop_result;
    uint32_t now = HAL_GetTick();
    uint32_t raw = 0;
    uint32_t read_started;
    if (s_battery_mux == NULL || now - s_attempt_ms < APP_BATTERY_PERIOD_MS)
    {
        return;
    }
    s_attempt_ms = now;
    read_started = app_diag_clock_count();
    sample.result = HAL_ADC_Start(&hadc1);
    if (sample.result == HAL_OK)
    {
        /* 480周期/12.5MHz 的采样约38.4us；超时有限，不让故障卡住sensor任务。 */
        sample.result = HAL_ADC_PollForConversion(&hadc1, APP_BATTERY_ADC_TIMEOUT_MS);
        if (sample.result == HAL_OK)
        {
            raw = HAL_ADC_GetValue(&hadc1);
        }
        stop_result = HAL_ADC_Stop(&hadc1);
        if (sample.result == HAL_OK)
        {
            sample.result = stop_result;
        }
    }
    if (sample.result == HAL_OK)
    {
        sample.adc_raw = (uint16_t)raw;
        sample.voltage_mv = (raw * APP_BATTERY_VREF_MV * 2U + 2047U) / 4095U;
        sample.valid = 1;
    }
    if (sample.result == HAL_OK)
    {
        app_diag_sensor_record(APP_DIAG_BATTERY, APP_DIAG_OK, (uint32_t)sample.result, read_started);
    }
    else
    {
        app_diag_sensor_record(APP_DIAG_BATTERY, APP_DIAG_ERROR, (uint32_t)sample.result, read_started);
    }
    xSemaphoreTake(s_battery_mux, portMAX_DELAY);
    s_battery_data = sample;
    xSemaphoreGive(s_battery_mux);
}

void app_battery_get_snapshot(app_battery_struct *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    if (s_battery_mux == NULL)
    {
        memset(snapshot, 0, sizeof(*snapshot));
        return;
    }
    xSemaphoreTake(s_battery_mux, portMAX_DELAY);
    *snapshot = s_battery_data;
    xSemaphoreGive(s_battery_mux);
}
