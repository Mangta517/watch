/**
 * @file app_heart.h
 * @brief EM7028 采样与心率估算；sensor 任务访问硬件，UI 只读快照及发送启停请求。
 */
#ifndef __APP_HEART_H__
#define __APP_HEART_H__

#include <stdint.h>
#include "em7028.h"

typedef enum
{
    APP_HEART_OFF,
    APP_HEART_MEASURING,
    APP_HEART_VALID,
    APP_HEART_NO_SIGNAL,
    APP_HEART_ERROR
} app_heart_state;

typedef struct
{
    uint16_t bpm;         /* 仅 VALID 时有效，40～200 BPM；工程估算值，需要实板校准 */
    uint16_t raw;         /* 最近一次 EM7028 光强采样 */
    uint16_t interval_ms; /* 最近两个已确认波峰的间隔；0 表示尚未检测到周期 */
    uint32_t signal_x10;  /* 滤波后变化幅度，单位0.1个ADC计数；用于无信号排查 */
    uint32_t sample_ms;   /* HAL tick，毫秒 */
    app_heart_state state;
    em7028_fault fault; /* 故障时由 sensor 任务复制；UI 不查询驱动私有状态 */
    uint8_t enabled;    /* UI 请求状态，不直接操作 I2C */
} app_heart_struct;

typedef struct
{
    uint32_t elapsed_ms; /* 本次START后的板端读取时刻，与心率算法使用同一HAL tick；不是器件内部转换时间 */
    float filtered;     /* 去背景/低通及可用运动参考消除后的有符号信号，ADC计数 */
    float dc;           /* 背景基线，ADC计数 */
    uint16_t raw;
    uint16_t bpm;
    uint16_t interval_ms;
    uint16_t sequence;  /* 成功读取的样本序号，0～65535循环，重新START归零 */
    app_heart_state state;
} app_heart_sample_struct;

/* 调度器启动后在 sensor 任务创建快照锁；EM7028 已由开机流程配置为待机。 */
void app_heart_init(void);
/* 现有 sensor 任务每轮调用；内部限速为 30ms，不创建新任务。 */
void app_heart_service(void);
/* 仅sensor任务在取得新MPU帧后调用；加速度单位为当前MPU配置的LSB。
 * 不读I2C、不跨任务访问；超过50ms未更新时退回原PPG处理。 */
void app_heart_motion_update(int16_t accel_x, int16_t accel_y, int16_t accel_z, uint32_t sample_ms);
/* UI 请求及一致快照；不访问硬件，NULL 输出不处理。 */
void app_heart_set_enabled(uint8_t enabled);
void app_heart_get_snapshot(app_heart_struct *heart);
/* LVGL任务取出同一次成功读取的原始值/算法状态并发送；最多缓存32个，满时丢弃最旧样本。
 * 不访问I2C，不改变心率估算；STOP/重新START清空旧样本。 */
uint16_t app_heart_read_samples(app_heart_sample_struct *samples, uint16_t size);

#endif
