/**
 * @file app_battery.h
 * @brief PA1 电池电压采样；sensor 任务更新，UI 只读取互斥锁保护的快照。
 */
#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

#include "main.h"

#define APP_BATTERY_PERIOD_MS 1000
#define APP_BATTERY_SLEEP_PERIOD_MS 10000
/* PA1 的 1M/1M 分压为1:2；参考电压使用3V3标称值，上板后可按实测VDDA校准。 */
#define APP_BATTERY_VREF_MV 3300

typedef struct
{
    uint32_t voltage_mv; /* 1/8低通后的电池电压，单位mV。 */
    uint16_t adc_raw;    /* 本次转换的原始ADC值，供诊断使用。 */
    HAL_StatusTypeDef result;
    uint8_t valid;
    uint8_t percent; /* 稳定采样后估算0～100%；本次开机只降不升，仅valid时可用。 */
} app_battery_struct;

/* ADC1 已由 MX_ADC1_Init 配置；以下初始化和采样只在 sensor 任务调用。 */
void app_battery_init(void);
/* sensor任务设置屏幕状态；启动8次有效采样仍保持1s，完成后熄屏改为10s。 */
void app_battery_set_sleep_mode(uint8_t sleeping);
void app_battery_service(void);
void app_battery_get_snapshot(app_battery_struct *snapshot);

#endif
