/**
 * @file app_aht21.c
 * @brief 复用现有 sensor 任务；80ms 后读结果，忙状态最多等到 200ms。
 */
#include "app_aht21.h"
#include "app_diag.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

#define APP_AHT21_TIMEOUT_MS 200U

static SemaphoreHandle_t s_aht21_mux;
static app_aht21_struct s_aht21_data;
static uint32_t s_attempt_ms;
static uint8_t s_pending;

void app_aht21_init(void)
{
    aht21_result result;
    s_aht21_mux = xSemaphoreCreateMutex();
    if (s_aht21_mux == NULL)
    {
        Error_Handler();
        return;
    }
    result = aht21_init();
    app_diag_sensor_init(APP_DIAG_AHT, (uint32_t)result);
    xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
    memset(&s_aht21_data, 0, sizeof(s_aht21_data));
    s_aht21_data.result = result;
    xSemaphoreGive(s_aht21_mux);
    s_pending = 0;
    s_attempt_ms = HAL_GetTick() - APP_AHT21_PERIOD_MS;
}

void app_aht21_service(void)
{
    aht21_struct sample;
    aht21_result result;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_attempt_ms;
    uint32_t read_started;
    if (s_aht21_mux == NULL)
    {
        return;
    }
    if (s_pending != 0)
    {
        if (elapsed < AHT21_CONVERSION_MS)
        {
            return;
        }
        read_started = app_diag_clock_count();
        result = aht21_read(&sample);
        if (result == AHT21_BUSY && elapsed < APP_AHT21_TIMEOUT_MS)
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_NOT_READY, (uint32_t)result, read_started);
            return;
        }
        if (result == AHT21_BUSY)
        {
            result = AHT21_TIMEOUT;
        }
        if (result == AHT21_OK)
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_OK, (uint32_t)result, read_started);
        }
        else
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_ERROR, (uint32_t)result, read_started);
        }
        s_pending = 0;
        xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
        s_aht21_data.measuring = 0;
        s_aht21_data.result = result;
        s_aht21_data.valid = result == AHT21_OK;
        if (result == AHT21_OK)
        {
            s_aht21_data.sample = sample;
            s_aht21_data.sample_ms = HAL_GetTick();
            ++s_aht21_data.sample_count;
        }
        xSemaphoreGive(s_aht21_mux);
    }
    else if (elapsed >= APP_AHT21_PERIOD_MS)
    {
        read_started = app_diag_clock_count();
        result = aht21_trigger();
        if (result != AHT21_OK)
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_ERROR, (uint32_t)result, read_started);
        }
        s_attempt_ms = HAL_GetTick();
        s_pending = result == AHT21_OK;
        xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
        s_aht21_data.measuring = s_pending;
        s_aht21_data.result = result;
        if (result != AHT21_OK)
        {
            s_aht21_data.valid = 0;
        }
        xSemaphoreGive(s_aht21_mux);
    }
}

void app_aht21_get_snapshot(app_aht21_struct *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    if (s_aht21_mux == NULL)
    {
        memset(snapshot, 0, sizeof(*snapshot));
        return;
    }
    xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
    *snapshot = s_aht21_data;
    xSemaphoreGive(s_aht21_mux);
}
