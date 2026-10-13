/**
 * @file app_battery.h
 * @brief PA1 电池电压采样；sensor 任务更新，UI 只读取互斥锁保护的快照。
 */
#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

#include "main.h"

#define APP_BATTERY_PERIOD_MS 1000U
/* PA1 的 1M/1M 分压为1:2；参考电压使用3V3标称值，上板后可按实测VDDA校准。 */
#define APP_BATTERY_VREF_MV 3300U

typedef struct
{
    uint32_t voltage_mv;
    uint16_t adc_raw;
    HAL_StatusTypeDef result;
    uint8_t valid;
} app_battery_struct;

/* ADC1 已由 MX_ADC1_Init 配置；以下初始化和采样只在 sensor 任务调用。 */
void app_battery_init(void);
void app_battery_service(void);
void app_battery_get_snapshot(app_battery_struct *snapshot);

#endif
