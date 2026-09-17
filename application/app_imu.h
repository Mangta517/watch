/**
  ******************************************************************************
  * @file    app_imu.h
  * @brief   应用层-MPU6050 数据服务 (周期采样 + 姿态解算 + 最新数据快照)
  *          分层: application(app_task 建 sensor 任务 → 本层提供任务体调用的服务)。
  *          注意: 任务的创建与调度器启动不在本层 (见 app_task.h)。
  *          解算: 姿态角由 common_imu 四元数互补融合得出 (纯数学在公共层)。
  *          总线约定: SENSOR 总线任务级唯一 owner 是 sensor 任务, 其他模块
  *          要取 IMU 数据一律走 App_IMU_GetSnapshot(), 不得直调 my_mpu6050_get_*
  *          (EXTI ISR 读 0x3A 的互斥由驱动内部 PRIMASK 临界区解决)。
  ******************************************************************************
***/
#ifndef __APP_IMU_H
#define __APP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6050.h"      /* Accel_Gyro_struct */
#include <stdint.h>

/* IMU 快照: 原始 LSB + 融合姿态角 */
typedef struct
{
    Accel_Gyro_struct raw;   /* 原始读数 (已减零偏; 加速度 ±2g, 陀螺仪 ±2000dps) */
    float pitch_deg;         /* 俯仰角 (°), 四元数融合 */
    float roll_deg;          /* 横滚角 (°), 四元数融合 */
    float yaw_deg;           /* 偏航角 (°), 纯陀螺仪积分不设界, 无磁参考会慢漂 */
} app_imu_struct;

/**
 * @brief 创建数据锁 (须在任务上下文调用, 调度器已启动; 见 app_display 同理)
 */
void App_IMU_Init(void);

/**
 * @brief 周期服务一轮: 突发读加速度/陀螺仪 → 姿态解算 → 更新快照, 维护在线状态
 *        仅由 app_task 的 sensor 任务体按周期调用, 其他上下文勿调
 */
void App_IMU_Service(void);

/**
 * @brief 取最新数据的一致快照 (加锁, 防多字段撕裂)
 * @param imu 输出快照, 不可为 NULL
 * @return 1=快照有效; 0=器件离线或首帧未完成
 */
uint8_t App_IMU_GetSnapshot(app_imu_struct *imu);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IMU_H */
