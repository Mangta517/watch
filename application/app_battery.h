/**
 * @file app_battery.h
 * @brief PA1 电池电压采样；sensor 任务更新，UI 只读取互斥锁保护的快照。
 */
#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

#include "main.h"

#define APP_BATTERY_PERIOD_MS 1000 /* 亮屏及启动阶段采样周期，毫秒 */
#define APP_BATTERY_SLEEP_PERIOD_MS 10000 /* 熄屏且启动平均完成后的采样周期，毫秒 */
/* PA1的1M/1M分压为1:2；每轮以VREFINT及出厂校准值反算实际VDDA。 */

typedef struct
{
    uint32_t voltage_mv; /* 1/8低通后的电池电压，单位mV。 */
    uint16_t adc_raw;    /* 本次转换的原始ADC值，供诊断使用。 */
    HAL_StatusTypeDef result; /* 本轮ADC采样流程结果，不区分具体失败阶段。 */
    uint8_t valid;            /* 1=启动平均已完成且本次转换成功，UI才可展示电压。 */
    uint8_t percent; /* 启动均值后以约256s慢滤波查表估算0～100%；本次开机只降不升，仅valid时可用。 */
} app_battery_struct;

typedef struct
{
    uint16_t voltage_mv;
    uint8_t percent;
} app_battery_curve_point_struct;

/* 返回Flash中的标定节点，按电压升序排列；count接收节点数。 */
const app_battery_curve_point_struct *app_battery_get_curve(uint8_t *count);
/* 按标定节点插值并四舍五入，电压超出标定范围时取对应端点。 */
uint8_t app_battery_voltage_to_percent(uint32_t voltage_mv);

/* ADC1 已由 MX_ADC1_Init 配置；以下初始化和采样只在 sensor 任务调用。 */
void app_battery_init(void);
/* sensor任务设置屏幕状态；常态熄屏10s，启动和低压确认保持1s。 */
void app_battery_set_sleep_mode(uint8_t sleeping);
/* 到期采内部参考和电池并更新滤波快照；连续3次有效电压≤3250mV时释放电源锁存。 */
void app_battery_service(void);
/* UI等消费方取一致快照(加锁整体复制)；NULL不处理，锁未建时输出全零。 */
void app_battery_get_snapshot(app_battery_struct *snapshot);

#endif
