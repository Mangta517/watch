/**
  ******************************************************************************
  * @file    app_imu.h
  * @brief   应用层-MPU6050 + LSM303DLHC (周期采样 + 姿态解算 + 最新数据快照)
  *          分层: application(app_task 建 sensor 任务 → 本层提供任务体调用的服务)。
  *          注意: 任务的创建与调度器启动不在本层 (见 app_task.h)。
  *          解算: 姿态角由 common_imu 四元数互补融合得出 (纯数学在公共层)。
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

#include "mpu6050.h" /* Accel_Gyro_struct */
#include "lsm303dlhc.h"
#include <stdint.h>

    typedef enum
    {
        APP_MAG_CAL_NONE,
        APP_MAG_CAL_RUNNING,
        APP_MAG_CAL_DONE,
        APP_MAG_CAL_FAILED
    } app_mag_cal_state;

    /* 本轮磁参考使用状态；区分读到数据与实际参与航向修正。 */
    typedef enum
    {
        APP_MAG_FUSION_NO_DATA,
        APP_MAG_FUSION_CALIBRATING,
        APP_MAG_FUSION_FIELD_RANGE,
        APP_MAG_FUSION_HORIZONTAL_WEAK,
        APP_MAG_FUSION_IMU_NO_DATA,
        APP_MAG_FUSION_ACTIVE
    } app_mag_fusion_state;

    typedef enum
    {
        APP_MAG_PERSIST_NONE,
        APP_MAG_PERSIST_RESTORED,
        APP_MAG_PERSIST_SAVING,
        APP_MAG_PERSIST_SAVED,
        APP_MAG_PERSIST_SAVE_FAILED,
        APP_MAG_PERSIST_LOAD_FAILED
    } app_mag_persist_state;

    /* IMU 快照: 原始 LSB + 融合姿态角 + 同一轮服务的磁参考状态 */
    typedef struct
    {
        Accel_Gyro_struct raw;     /* 原始读数 (已减零偏; 加速度 ±4g, 陀螺仪 ±1000°/s) */
        float pitch_deg;           /* 俯仰角 (°), 四元数融合 */
        float roll_deg;            /* 横滚角 (°), 四元数融合 */
        float yaw_deg;             /* 偏航角 (-180..180°)，磁北参考；失效时六轴相对递推 */
        lsm303dlhc_mag_struct mag; /* 芯片坐标下磁场，未做安装校准 */
        float mag_body_uT[3];      /* 校准并映射到 MPU6050 机体坐标后的磁场 */
        uint8_t mag_online;        /* 有当前档位新鲜度窗口内的有效磁场帧。 */
        uint8_t mag_fused;         /* 本轮磁场参与航向修正 */
        uint8_t mag_calibrated;    /* 已校准，或已从EEPROM恢复有效校准 */
        app_mag_cal_state mag_cal_state;
        app_mag_fusion_state mag_fusion_state;
        uint16_t mag_cal_samples;                /* 校准采集帧数；点击完成不等于校准成功 */
        float mag_cal_span_uT[3];                /* 当前校准窗口各轴max-min，验收要求均≥20μT */
        app_mag_persist_state mag_persist_state; /* RAM校准与掉电保存分别报告 */
    } app_imu_struct;

    /**
 * @brief 创建数据锁 (须在任务上下文调用, 调度器已启动; 见 app_display 同理)
 */
    void App_IMU_Init(void);

    /**
 * @brief 周期服务一轮: 磁场轮询 + MPU突发读 → 姿态融合 → 一致快照
 *        仅由 app_task 的 sensor 任务体按周期调用, 其他上下文勿调
 */
    void App_IMU_Service(void);
    /* 仅sensor任务：选择20/40ms档位；磁校准请求/进行中强制20ms，返回本轮调度周期。 */
    uint32_t app_imu_set_sample_period(uint32_t period_ms);
    /* 仅sensor任务调用；与MPU零偏保存串行，返回1表示磁记录仍在保存。 */
    uint8_t App_IMU_SaveService(void);

    /**
 * @brief 取最新数据的一致快照 (加锁, 防多字段撕裂)
 * @param imu 输出快照, 不可为 NULL
 * @return 1=快照有效; 0=器件离线或首帧未完成
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
