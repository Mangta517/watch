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

#define APP_BATTERY_ADC_TIMEOUT_MS          2
#define APP_BATTERY_STARTUP_SAMPLES         8
#define APP_BATTERY_STARTUP_AVERAGE_SAMPLES 4

static SemaphoreHandle_t s_battery_mux;
static app_battery_struct s_battery_data;
static uint32_t s_attempt_ms;
static int32_t s_filtered_mv_q8;
static uint32_t s_startup_sum_mv;
static uint8_t s_startup_samples;
static uint8_t s_last_percent;

static uint8_t battery_percent(uint32_t voltage_mv)
{
    /* 2026-10-27按1815点连续未充电日志重标3.55～4.0V；末6h趋势暂估到达3.5V的时间。 */
    /* 3.5V/5%及3.3V/0%仍是沿用假设，未测到耗尽；计算依据见docs/battery-log.md。 */
    static const uint16_t voltage_points_mv[] = {3300, 3500, 3550, 3600, 3650, 3700, 3750, 3800, 3850, 3900, 3950, 4000};
    static const uint8_t percent_points[] = {0, 5, 18, 31, 41, 54, 64, 70, 79, 88, 94, 100};
    uint32_t i;
    if (voltage_mv <= voltage_points_mv[0])
    {
        return 0;
    }
    for (i = 1; i < sizeof(voltage_points_mv) / sizeof(voltage_points_mv[0]); ++i)
    {
        if (voltage_mv < voltage_points_mv[i])
        {
            uint32_t span_mv = voltage_points_mv[i] - voltage_points_mv[i - 1];
            uint32_t rise = (voltage_mv - voltage_points_mv[i - 1]) * (percent_points[i] - percent_points[i - 1]);
            return (uint8_t)(percent_points[i - 1] + (rise + span_mv / 2) / span_mv);
        }
    }
    return 100;
}

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
    s_startup_samples = 0;
    s_startup_sum_mv = 0;
    s_last_percent = 0;
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
        /* 48MHz PCLK2/8=6MHz，480周期采样约80us；超时有限，不让故障卡住sensor任务。 */
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
        sample.voltage_mv = (raw * APP_BATTERY_VREF_MV * 2 + 2047) / 4095;
        /* 启动前4次不入均值，后4次均值作为初值；沿用每秒调度，不阻塞sensor任务。 */
        if (s_startup_samples < APP_BATTERY_STARTUP_SAMPLES)
        {
            ++s_startup_samples;
            if (s_startup_samples > APP_BATTERY_STARTUP_SAMPLES - APP_BATTERY_STARTUP_AVERAGE_SAMPLES)
            {
                s_startup_sum_mv += sample.voltage_mv;
            }
            if (s_startup_samples == APP_BATTERY_STARTUP_SAMPLES)
            {
                s_filtered_mv_q8 = (int32_t)(s_startup_sum_mv * 256 / APP_BATTERY_STARTUP_AVERAGE_SAMPLES);
                s_last_percent = battery_percent((uint32_t)(s_filtered_mv_q8 + 128) / 256);
            }
        }
        else
        {
            /* Q8保留小数，避免1/8低通因整数取整停在旧值。 */
            s_filtered_mv_q8 += ((int32_t)sample.voltage_mv * 256 - s_filtered_mv_q8) / 8;
        }
        if (s_startup_samples == APP_BATTERY_STARTUP_SAMPLES)
        {
            sample.voltage_mv = (uint32_t)(s_filtered_mv_q8 + 128) / 256;
            sample.percent = battery_percent(sample.voltage_mv);
            /* 尚无充电状态输入：本次开机电量只降不升，电压回弹不代表补充了电荷。 */
            if (sample.percent < s_last_percent)
            {
                s_last_percent = sample.percent;
            }
            sample.percent = s_last_percent;
            sample.valid = 1;
        }
        else
        {
            sample.voltage_mv = 0;
        }
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
