/**
 ******************************************************************************
 * @file    common_imu.c
 * @brief   姿态解算实现 —— 逐行移植自 flight_hal/common/common_IMU.c
 *          (Mahony/Madgwick 类互补滤波: 加权重构重力向量与实测加速度做向量积,
 *           P+I 修正陀螺仪角速度, 一阶龙格库塔更新四元数, 反解 pitch/roll)
 *          常数 KpDef=0.8 / KiDef=0.0003 为 flight 端实飞调定值, 直接沿用;
 *          改动本文件后需在旧仓库同步评估。
 ******************************************************************************
 */
#include "common_imu.h"

#include <math.h>       /* asin / atan2f */

/* 计算欧拉角用到的3个参数 */
float RtA = 57.2957795f;  // 弧度->度
// 陀螺仪初始化量程+-2000度/秒于 1/(65536 / 4000) = 0.03051756*2
float Gyro_G = 4000.0 / 65536;  // 度/s
// 度每秒,转换弧度每秒则 0.06105 * 0.0174533 = 0.0010653
float Gyro_Gr = 4000.0 / 65536 / 180 * 3.1415926; // 弧度/s
#define squa(Sq) (((float)Sq) * ((float)Sq)) /* 计算平方 */

/**
* @description: 快速计算 1/sqrt(num) (quake-iii 近似 + 一次牛顿迭代)
* @param {float} number
*/
static float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5f;

    x2 = number * 0.5f;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));  // 1st iteration （第一次牛顿迭代）
    return y;
}

/**
* @description: 根据mpu的6轴数据, 获取表征姿态的欧拉角
* @param {Accel_Gyro_Struct} *gyroAccel mpu的6轴数据
* @param {Euler_Struct} *eulerAngle 计算后得到的欧拉角
* @param {float} dt 采样周期 (单位s)
* @return {*}
*/
void Common_IMU_GetEulerAngle(Accel_Gyro_struct *gyroAccel,
                              Euler_struct *eulerAngle,
                              float dt)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};
    static float KpDef   = 0.8f;
    static float KiDef   = 0.0003f;
    static Quaternion_Struct NumQ = {1, 0, 0, 0};
    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // 加速度归一化
    NormQuat = Q_rsqrt(squa(gyroAccel->Accel.Accel_x) +
                       squa(gyroAccel->Accel.Accel_y) +
                       squa(gyroAccel->Accel.Accel_z));

    Acc.x = gyroAccel->Accel.Accel_x * NormQuat;
    Acc.y = gyroAccel->Accel.Accel_y * NormQuat;
    Acc.z = gyroAccel->Accel.Accel_z * NormQuat;
    // 向量差乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // 再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // 角速度融合加速度积分补偿值
    Gyro.x = gyroAccel->Gyro.Gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;  // 弧度制
    Gyro.y = gyroAccel->Gyro.Gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->Gyro.Gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    // 一阶龙格库塔法, 更新四元数
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /*机体坐标系下的Z方向向量*/
    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;   /*矩阵(3,1)项*/
    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;   /*矩阵(3,2)项*/
    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*矩阵(3,3)项*/

    float yaw_G = gyroAccel->Gyro.Gyro_z * Gyro_G;  // 将Z轴角速度陀螺仪值 转换为Z角度/秒
    if ((yaw_G > 0.5f) || (yaw_G < -0.5f))          // 数据太小可以认为是干扰，不是偏航动作
    {
        eulerAngle->yaw += yaw_G * dt;              // 角速度积分成偏航角
    }

    eulerAngle->pitch = asin(vecxZ) * RtA;          // 俯仰角
    eulerAngle->roll  = atan2f(vecyZ, veczZ) * RtA; // 横滚角
}
