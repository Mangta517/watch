#ifndef APP_DIAG_H
#define APP_DIAG_H

#include <stdint.h>

/* 运行诊断接口：各任务/驱动调用点把周期、耗时、结果和启动阶段写入本模块的统计区，
 * 由LVGL任务按需快照并经UART发"WDG1"二进制帧。本模块配置并读取TIM2(100us/格)，
 * 不采集任何传感器；默认关闭，手机发DIAG,1才开启。 */

/* 默认关闭自动诊断和心率波形；DIAG,1可独立开启任务/传感器诊断。 */
#ifndef APP_DEBUG_TELEMETRY
#define APP_DEBUG_TELEMETRY 0
#endif

enum
{
    APP_DIAG_LVGL,
    APP_DIAG_SENSOR_TASK,
    APP_DIAG_IDLE, /* 保留WDG1原ID2；IDLE没有业务循环计时。 */
    APP_DIAG_TOUCH,
    APP_DIAG_TASK_COUNT
};

enum
{
    APP_DIAG_MPU,
    APP_DIAG_MAG,
    APP_DIAG_HEART,
    APP_DIAG_BARO,
    APP_DIAG_AHT,
    APP_DIAG_BATTERY,
    APP_DIAG_SENSOR_COUNT
};

enum
{
    APP_DIAG_OK,
    APP_DIAG_NOT_READY,
    APP_DIAG_ERROR
};

enum
{
    APP_BOOT_FIRST_FRAME_MS,   /* 首帧提交完成的HAL毫秒时刻 */
    APP_BOOT_RTC_BEGIN_MS,     /* RTC初始化发起时刻 */
    APP_BOOT_RTC_END_MS,       /* RTC初始化结束时刻 */
    APP_BOOT_RTC_RESULT,       /* RTC初始化最后一次HAL状态 */
    APP_BOOT_BT_BEGIN_MS,      /* 蓝牙初始化发起时刻 */
    APP_BOOT_BT_END_MS,        /* 蓝牙初始化结束时刻 */
    APP_BOOT_BT_RESULT,        /* 蓝牙初始化最后一次HAL状态 */
    APP_BOOT_SENSOR_BEGIN_MS,  /* sensor任务启动时刻 */
    APP_BOOT_SENSOR_READY_MS,  /* 各传感器与存储初始化完成时刻 */
    APP_BOOT_SAMPLING_BEGIN_MS,/* 进入采样主循环的时刻 */
    APP_BOOT_CAL_BEGIN_MS,     /* 本轮零偏校准开始时刻 */
    APP_BOOT_CAL_END_MS,       /* 本轮零偏校准结束时刻 */
    APP_BOOT_CAL_RESULT,       /* 0=恢复有效零偏或校准并保存成功，1=校准失败，2=RAM成功但保存失败 */
    APP_BOOT_CAL_STATE,        /* mpu6050_cal_state当前状态 */
    APP_BOOT_CAL_FRAMES,       /* 已累计的校准帧数 */
    APP_BOOT_CAL_STABLE_FRAMES,/* 满足静止条件的帧数 */
    APP_BOOT_FIELD_COUNT
};

/* 写入一个启动阶段字段；field 取 APP_BOOT_* 枚举。每次调用只保护单字段更新，
 * 同一阶段多次调用之间不是原子事务。value 为毫秒时间戳、状态码或计数，含义随字段而定。 */
void app_diag_boot_set(uint8_t field, uint32_t value);
/* 记录IMU校准保存进度：stage/error/address/elapsed_ms 原样上报，UINT32_MAX表示未知。 */
void app_diag_imu_save_record(uint32_t stage, uint32_t error, uint32_t address, uint32_t elapsed_ms);
/* 整份拷贝16个MPU校准诊断字；只在sensor任务调用。 */
void app_diag_mpu_cal_record(const uint32_t *values);

/* TIM2专用于诊断，100us一计数；FreeRTOS在调度器启动时调用初始化。 */
void app_diag_clock_init(void);
/* 返回TIM2当前计数值，单位100us；32位计数器约5天回绕，调用方只做无符号差值。 */
uint32_t app_diag_clock_count(void);
/* 任务循环首尾配对调用，单位为100us计数换算出的微秒；target_ms为期望周期。 */
void app_diag_task_begin(uint8_t id);
void app_diag_task_end(uint8_t id, uint32_t target_ms);
/* 记录驱动初始化返回值；result由调用方解释，UINT32_MAX表示尚未初始化。 */
void app_diag_sensor_init(uint8_t id, uint32_t result);
/* 0=就绪，1=失败冷却，2=恢复中；状态/尝试次数保存在s_sensors供调试器查看。 */
void app_diag_sensor_recovery(uint8_t id, uint8_t state);
/* 每笔传感器事务上报一次：outcome取APP_DIAG_*，result为驱动原始返回值，
 * started为事务开始时的app_diag_clock_count()，用于统计事务耗时。 */
void app_diag_sensor_record(uint8_t id, uint8_t outcome, uint32_t result, uint32_t started);
void app_diag_heart_queue(uint8_t dropped);
void app_diag_heart_tx(uint16_t count, uint8_t success);
/* HEART last_result低字节为ERROR=2，其余字节依次为fault kind、寄存器、读回值。 */
void app_diag_heart_fault(uint8_t kind, uint8_t reg, uint8_t value);
void app_diag_receive(const uint8_t *data, uint16_t size);
void app_diag_reset_receive(void);
/* IMU,CAL请求仅传递标志；sensor任务消费后执行校准/EEPROM保存。 */
uint8_t app_diag_take_imu_calibration(void);
/* 仅LVGL任务发送，每100ms最多一帧；不增加任务，不在传感器/ISR里发串口。 */
void app_diag_service(void);

#endif
