/**
  ******************************************************************************
  * @file    app_imu.h
  * @brief   应用层-MPU6050 + LSM303DLHC (加速度活动检测 + 独立指南针 + 一致快照)
  *          分层: application(app_task 建 sensor 任务 → 本层提供任务体调用的服务)。
  *          注意: 任务的创建与调度器启动不在本层 (见 app_task.h)。
  *          解算由公共层处理，指南针只使用LSM303自身的加速度和磁场。
  *          总线约定: SENSOR 总线任务级唯一 owner 是 sensor 任务, 其他模块
  *          要取 IMU 数据一律走 App_IMU_GetSnapshot(), 不得直调 my_mpu6050_get_*
  *          EXTI ISR 只记事件，sensor 任务读 0x3A 释放器件锁存。
  ******************************************************************************
***/
#ifndef __APP_IMU_H
#define __APP_IMU_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "imu_types.h"

    typedef enum
    {
        APP_MAG_CAL_NONE,
        APP_MAG_CAL_RUNNING,
        APP_MAG_CAL_DONE,
        APP_MAG_CAL_FAILED
    } app_mag_cal_state;

    typedef enum
    {
        APP_COMPASS_NO_MAG_DATA,
        APP_COMPASS_CALIBRATING,
        APP_COMPASS_FIELD_RANGE,
        APP_COMPASS_ACCEL_INVALID,
        APP_COMPASS_HORIZONTAL_WEAK,
        APP_COMPASS_READY
    } app_compass_state;

    typedef enum
    {
        APP_MAG_PERSIST_NONE,
        APP_MAG_PERSIST_RESTORED,
        APP_MAG_PERSIST_SAVING,
        APP_MAG_PERSIST_SAVED,
        APP_MAG_PERSIST_SAVE_FAILED,
        APP_MAG_PERSIST_LOAD_FAILED
    } app_mag_persist_state;

    /* 加速度、活动与独立指南针快照；MPU与LSM303的在线状态分别报告。 */
    typedef struct
    {
        uint32_t steps;                /* 当日累计步数；片内Flash节流保存。 */
        uint32_t wrist_raise_sequence; /* 仅在放下后抬起时递增，供UI消费。 */
        uint32_t wrist_raise_ms;       /* 最近抬腕事件的HAL时间，避免陈旧事件亮屏。 */
        Accel_Gyro_struct raw;     /* MPU加速度±4g；Gyro字段固定为0。 */
        float compass_deg;         /* OV-Watch直接磁方位角[0,360)°，LSM303加速度倾斜补偿。 */
        app_compass_state compass_state;
        uint8_t compass_valid;     /* 当前加速度与磁场可用于直接指南针。 */
        lsm303dlhc_mag_struct mag; /* 芯片坐标下磁场，未做安装校准 */
        float mag_body_uT[3];      /* 安装校准后的表盘坐标磁场。 */
        uint8_t mag_online;        /* 有当前档位新鲜度窗口内的有效磁场帧。 */
        uint8_t mag_calibrated;    /* 已校准，或已从EEPROM恢复有效校准 */
        app_mag_cal_state mag_cal_state;
        uint16_t mag_cal_samples;                /* 校准采集帧数；点击完成不等于校准成功 */
        float mag_cal_span_uT[3];                /* 当前校准窗口各轴max-min，验收要求均≥20μT */
        app_mag_persist_state mag_persist_state; /* RAM校准与掉电保存分别报告 */
    } app_imu_struct;

    /* 仅sensor任务操作硬件；保持MPU、气压、磁、心率原有开机顺序。 */
    void app_imu_init_hw(void);
    void app_imu_init_mag_hw(void);
    uint8_t app_imu_recover_step(uint32_t now_ms);
    void app_imu_mag_recover_step(uint32_t now_ms);
    /* 旧维护入口；当前启动和日常服务均不调用六轴校准。 */
    void app_imu_calibration_begin(uint32_t now_ms);
    mpu6050_cal_state app_imu_calibration_progress(uint16_t *stable, uint16_t *average);
    void app_imu_restore_offset(const mpu6050_offset_struct *offset);
    void app_imu_get_offset(mpu6050_offset_struct *offset);

    /**
 * @brief 创建数据锁 (须在任务上下文调用, 调度器已启动; 见 app_display 同理)
 */
    void App_IMU_Init(void);

    /**
 * @brief 仅sensor调用，MPU突发读后更新活动与PPG参考，返回驱动读帧结果。
 *        指南针由独立服务调度；read_mag参数保留旧调用签名。
 */
    uint8_t App_IMU_Service(uint8_t read_mag);
    /* 每轮sensor独立调用，active=指南针页或磁校准；返回1表示电源/恢复尚未完成。 */
    uint8_t app_imu_compass_service(uint8_t active, uint8_t read_sensors);
    /* LVGL发布RTC日历，sensor执行跨天清零及Flash保存。day=0表示未校时。 */
    void app_imu_set_day(uint16_t day, uint8_t calendar_restored);
    void app_imu_activity_service(void);
    /* 仅sensor任务：后台25ms低功耗，心率/SPORT 10ms连续加速度。 */
    uint32_t app_imu_set_sample_period(uint32_t period_ms);
    /* 仅sensor任务读取自有活动状态，用于新抬腕事件立即通知LVGL。 */
    uint32_t app_imu_wrist_sequence(void);
    /* 仅sensor任务：先设置请求，再调用set_sample_period；心率参考不启动姿态/磁融合。 */
    void app_imu_set_accel_reference(uint8_t enabled);
    /* 兼容MPU完全休眠接口，唤醒后等待新的加速度帧。 */
    void app_imu_set_sleep(uint8_t sleeping);
    /* 校准命令、采集或校准记录保存尚未完成时返回1。 */
    uint8_t app_imu_is_calibrating(void);
    /* 仅sensor任务调用；与其他EEPROM使用者串行，返回1表示磁记录仍在保存。 */
    uint8_t App_IMU_SaveService(void);

    /**
 * @brief 取最新数据的一致快照 (加锁, 防多字段撕裂)
 * @param imu 输出快照, 不可为 NULL
 * @return 1=MPU加速度在线；指南针独立读取compass_valid和mag_online。
 */
    uint8_t App_IMU_GetSnapshot(app_imu_struct *imu);

    /* UI/任务发出请求即可；采集与系数更新由 sensor 任务执行，不在 UI 中读 I2C。
 * start=1 开始，start=0 完成；旋转各轴30~60s，失败保留上次系数。
 * shortcut: 只做硬铁偏置和对角比例校准，存在轴间软铁耦合时需椭球标定。 */
    void App_IMU_MagCalibrate(uint8_t start);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IMU_H */
