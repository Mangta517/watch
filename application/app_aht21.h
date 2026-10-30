/**
 * @file app_aht21.h
 * @brief AHT21亮屏1秒、熄屏1分钟采样；sensor任务访问硬件，UI只读取一致快照。
 */
#ifndef __APP_AHT21_H__
#define __APP_AHT21_H__

#include "aht21.h"

#define APP_AHT21_PERIOD_MS 1000
#define APP_AHT21_SLEEP_PERIOD_MS 60000

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
/* sensor任务切换周期；缩短时沿用在途转换或立即申请新采样，拉长时取消在途结果。 */
void app_aht21_set_period(uint32_t period_ms);
void app_aht21_get_snapshot(app_aht21_struct *snapshot);

#endif
