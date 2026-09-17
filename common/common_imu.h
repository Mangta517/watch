/**
 ******************************************************************************
 * @file    common_imu.h
 * @brief   姿态解算 (公共层) —— 陀螺仪+加速度计互补 quaternion 融合
 *          移植自 flight_hal/common/common_IMU.c|h, 算法与调参常数保持一致;
 *          旧端 normAccz/GetNormAccZ (定高用) 本工程暂无需求, 未随迁。
 *          平台无关纯数学: 输入 interface/mpu6050.h 的原始 LSB 结构,
 *          依赖前提 —— 陀螺仪量程必须为 ±2000dps (Gyro_G=4000/65536 折算系数)。
 ******************************************************************************
 */
#ifndef __COMMON_IMU_H__
#define __COMMON_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6050.h"      /* Accel_Gyro_struct (原始 LSB 输入) */

/* 欧拉角 (单位: 度) —— 与 flight 端 common_config.h 中定义一致 */
typedef struct
{
    float yaw;            /* 偏航: 纯陀螺仪积分, 无磁参考会缓慢漂移 */
    float pitch;          /* 俯仰: 四元数融合解算 */
    float roll;           /* 横滚: 四元数融合解算 */
} Euler_struct;

/* 四元数 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

/* 折算常数 (定义在 .c, 只读) */
extern float RtA;       /* 弧度 -> 度 */
extern float Gyro_G;    /* 陀螺仪 LSB -> 度/s  (±2000dps) */
extern float Gyro_Gr;   /* 陀螺仪 LSB -> 弧度/s */

/**
 * @brief 由 6 轴原始数据递推姿态欧拉角 (内部 static 四元数, 持续调用即持续融合)
 * @param gyroAccel   加速度/陀螺仪原始 LSB (已减零偏)
 * @param eulerAngle  输出姿态角 (度); yaw 为积分累加, 调用方保持其初值
 * @param dt          距上次调用的实际间隔 (秒), 调用方负责测量+限幅
 */
void Common_IMU_GetEulerAngle(Accel_Gyro_struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_IMU_H__ */
