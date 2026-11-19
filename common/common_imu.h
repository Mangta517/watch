/**
 ******************************************************************************
 * @file    common_imu.h
 * @brief   姿态解算 (公共层) —— 六轴 quaternion 融合 + 磁参考航向修正
 *          仅做数学处理，不访问HAL或总线；输入类型与灵敏度来自imu_types.h，
 *          对应当前驱动运动档配置：±1000°/s、±4g。
 ******************************************************************************
 */
#ifndef __COMMON_IMU_H__
#define __COMMON_IMU_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "imu_types.h" /* Accel_Gyro_struct (原始LSB输入) */

    /* 欧拉角 (单位: 度) —— 与 flight 端 common_config.h 中定义一致 */
    typedef struct
    {
        float yaw;   /* 偏航 (-180..180°): 有磁参考时朝磁北，无磁参考时相对递推 */
        float pitch; /* 俯仰: 四元数融合解算 */
        float roll;  /* 横滚: 四元数融合解算 */
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
    extern const float RtA;    /* 弧度 -> 度 */
    extern const float Gyro_G; /* 陀螺仪 LSB -> 度/s，当前1/32.8 */

    /**
 * @brief 由 6 轴原始数据递推姿态欧拉角 (内部 static 四元数, 持续调用即持续融合)
 * @param gyroAccel   加速度/陀螺仪原始 LSB (已减零偏)
 * @param eulerAngle  输出姿态角 (度)，yaw 无磁参考时会漂移
 * @param dt          距上次调用的实际间隔 (秒), 调用方负责测量+限幅
 */
    void Common_IMU_GetEulerAngle(Accel_Gyro_struct *gyroAccel, Euler_struct *eulerAngle, float dt);

    /* 单 sensor 任务使用；磁场须映射到 MPU6050 机体坐标，校准后才有准确磁参考。
 * mag_uT=NULL 时六轴递推；euler=NULL只递推，不反解欧拉角。
 * 返回 1=本次用了有效水平磁场修正航向。 */
    uint8_t Common_IMU_GetEulerAngleWithMag(const Accel_Gyro_struct *raw, const float mag_uT[3], Euler_struct *euler, float dt);
    /* 重置递推状态；下一份0.75～1.25g样本初始化俯仰/横滚，首份有效水平磁场直接对齐磁北。 */
    void Common_IMU_Reset(void);
    /* 请求下个有效水平磁场重对齐航向；reseed_gravity=1还要求首份有效加速度重建俯仰/横滚，
     * 清除旧重力积分但保留已有航向；=0保留俯仰/横滚与积分，供连续采样的磁校准结束使用。 */
    void Common_IMU_ReseedYaw(uint8_t reseed_gravity);

    /* OV-Watch式加速度倾斜补偿后求atan2(Hy,Hx)，按本工程同轴坐标修正投影，输出[0,360)°。
     * 加速度以8192LSB/g输入，和磁场须同轴同向；结果枚举区分倾角、场强与水平分量，失败保留输出。 */
    typedef enum
    {
        COMMON_COMPASS_OK,
        COMMON_COMPASS_ACCEL_INVALID,
        COMMON_COMPASS_FIELD_RANGE,
        COMMON_COMPASS_HORIZONTAL_WEAK
    } common_compass_result;
    common_compass_result common_imu_compass_heading(const Accel_struct *accel, const float mag_uT[3], float *heading_deg);
    /* 允许调用方指定合磁场上限（如未校准阶段放宽硬铁偏置上限），下限仍固定为5μT。 */
    common_compass_result common_imu_compass_heading_with_limit(const Accel_struct *accel, const float mag_uT[3],
                                                                float max_field_uT, float *heading_deg);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_IMU_H__ */
