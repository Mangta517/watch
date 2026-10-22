/**
 * @file app_aht21.h
 * @brief AHT21 两秒采样服务；sensor 任务访问硬件，UI 只读取一致快照。
 */
#ifndef __APP_AHT21_H__
#define __APP_AHT21_H__

#include "aht21.h"

#define APP_AHT21_PERIOD_MS 2000U

typedef struct
{
    aht21_struct sample;
    uint32_t sample_ms; /* 最近成功读取的 HAL tick，毫秒 */
    uint32_t sample_count;
    aht21_result result;
    uint8_t valid;
    uint8_t measuring;
} app_aht21_struct;

/* sensor 任务启动阶段调用；创建快照锁并执行一次传感器初始化。 */
void app_aht21_init(void);
/* sensor 每轮调用；触发与读取分离，转换等待期间不占总线、不延时。 */
void app_aht21_service(void);
/* sensor任务切换周期；取消在途结果，下一次到期再触发，AHT21转换后自动待机。 */
void app_aht21_set_period(uint32_t period_ms);
void app_aht21_get_snapshot(app_aht21_struct *snapshot);

#endif
