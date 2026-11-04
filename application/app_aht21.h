/**
 * @file app_aht21.h
 * @brief AHT21亮屏1秒、熄屏10分钟采样；sensor任务访问硬件，UI只读取一致快照。
 */
#ifndef __APP_AHT21_H__
#define __APP_AHT21_H__

#include "aht21.h"

#define APP_AHT21_PERIOD_MS 1000 /* 亮屏采样周期，毫秒 */
#define APP_AHT21_SLEEP_PERIOD_MS 600000 /* 熄屏采样周期，毫秒(10分钟) */

typedef struct
{
    aht21_struct sample;   /* 最近一次成功读回的原始值、换算温湿度及芯片状态。 */
    uint32_t sample_ms; /* 最近成功读取的 HAL tick，毫秒 */
    uint32_t sample_count; /* 开机以来成功读取的次数。 */
    aht21_result result;   /* 最近一次触发/读回/恢复的驱动返回值。 */
    uint8_t valid;         /* 1=sample 为当前有效快照，0=不可展示。 */
    uint8_t measuring;     /* 1=已触发、正等待读回该次转换。 */
} app_aht21_struct;

/* sensor 任务启动阶段调用；创建快照锁并执行一次传感器初始化。 */
void app_aht21_init(void);
/* sensor 每轮调用；触发与读取分离，转换等待期间不占总线、不延时。 */
void app_aht21_service(void);
/* sensor任务切换周期；缩短时沿用在途转换或立即申请新采样，拉长时取消在途结果。 */
void app_aht21_set_period(uint32_t period_ms);
/* UI等消费方取一致快照(加锁整体复制)；NULL不处理，锁未建时输出全零。 */
void app_aht21_get_snapshot(app_aht21_struct *snapshot);
/* 有在途采样/恢复/待读回时返回1；仅未到期(含未到期恢复)返回0，供环境窗口提前收尾。 */
uint8_t app_aht21_is_busy(void);

#endif
