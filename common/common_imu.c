/**
 * @file common_imu.c
 * @brief MPU6050 四元数互补融合 + 磁场水平投影航向修正。
 *        磁场只沿重力轴修正航向，避免磁倾角直接拉偏 pitch/roll。
 */
#include "common_imu.h"
#include <math.h>
#include <string.h>

float RtA = 57.2957795f;
float Gyro_G = 1.0f / MPU6050_GYRO_LSB_PER_DPS;
float Gyro_Gr = 1.0f / MPU6050_GYRO_LSB_PER_DPS / 180.0f * 3.1415926f;

static Quaternion_Struct s_q = {1, 0, 0, 0};
static float s_error_i[3];

void Common_IMU_Reset(void)
{
    s_q.q0 = 1.0f;
    s_q.q1 = s_q.q2 = s_q.q3 = 0.0f;
    memset(s_error_i, 0, sizeof(s_error_i));
}

uint8_t Common_IMU_GetEulerAngleWithMag(const Accel_Gyro_struct *raw, const float mag_uT[3], Euler_struct *euler, float dt)
{
    /* 三轴数组统一按机体X/Y/Z排列；四元数q0为标量，q1/q2/q3为向量。 */
    float q0 = s_q.q0, q1 = s_q.q1, q2 = s_q.q2, q3 = s_q.q3;
    float gravity[3];     /* 四元数预测的单位重力向量 */
    float accel[3];       /* 加速度，先为LSB，再归一化 */
    float gyro[3];        /* 修正后的机体角速度，rad/s */
    float error[3] = {0}; /* 加速度与预测重力的方向误差 */
    float norm, dot;
    float north[3];      /* 四元数预测的单位磁北向量 */
    float horizontal[3]; /* 实测磁场的水平投影 */
    float cross[3];      /* 水平磁场与预测磁北的叉积 */
    float heading_error; /* 航向误差，rad */
    float sine, cosine, pitch_sine;
    uint8_t i, mag_used = 0;

    if (raw == NULL || euler == NULL || !isfinite(dt) || dt <= 0.0f)
    {
        return 0;
    }

    /* 1. 提取机体坐标下的重力方向，作为俯仰/横滚参考。 */
    gravity[0] = 2.0f * (q1 * q3 - q0 * q2);
    gravity[1] = 2.0f * (q0 * q1 + q2 * q3);
    gravity[2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    accel[0] = raw->Accel.Accel_x;
    accel[1] = raw->Accel.Accel_y;
    accel[2] = raw->Accel.Accel_z;
    norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    /* 2. 仅在0.75～1.25g内用加速度修正重力方向，排除明显跑步冲击。 */
    if (norm > 0.75f * MPU6050_ACCEL_LSB_PER_G && norm < 1.25f * MPU6050_ACCEL_LSB_PER_G)
    {
        for (i = 0; i < 3; ++i)
        {
            accel[i] /= norm;
        }
        error[0] = accel[1] * gravity[2] - accel[2] * gravity[1];
        error[1] = accel[2] * gravity[0] - accel[0] * gravity[2];
        error[2] = accel[0] * gravity[1] - accel[1] * gravity[0];
        for (i = 0; i < 3; ++i)
        {
            s_error_i[i] += error[i] * 0.03f * dt; /* 积分反馈按实际dt累计，dt单位为s */
        }
    }
    gyro[0] = raw->Gyro.Gyro_x * Gyro_Gr;
    gyro[1] = raw->Gyro.Gyro_y * Gyro_Gr;
    gyro[2] = raw->Gyro.Gyro_z * Gyro_Gr;
    for (i = 0; i < 3; ++i)
    {
        gyro[i] += 0.8f * error[i] + s_error_i[i];
    }

    /* 3. 磁场投影到水平面，仅沿重力轴修正航向，不直接拉动俯仰/横滚。 */
    if (mag_uT != NULL && isfinite(mag_uT[0]) && isfinite(mag_uT[1]) && isfinite(mag_uT[2]))
    {
        dot = mag_uT[0] * gravity[0] + mag_uT[1] * gravity[1] + mag_uT[2] * gravity[2];
        for (i = 0; i < 3; ++i)
        {
            horizontal[i] = mag_uT[i] - dot * gravity[i];
        }
        norm = sqrtf(horizontal[0] * horizontal[0] + horizontal[1] * horizontal[1] + horizontal[2] * horizontal[2]);
        if (norm > 5.0f) /* 水平磁场大于5μT时才修正航向 */
        {
            for (i = 0; i < 3; ++i)
            {
                horizontal[i] /= norm;
            }
            /* R^T*[1,0,0]：当前四元数预测的磁北，已与重力正交。 */
            north[0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
            north[1] = 2.0f * (q1 * q2 - q0 * q3);
            north[2] = 2.0f * (q1 * q3 + q0 * q2);
            cross[0] = horizontal[1] * north[2] - horizontal[2] * north[1];
            cross[1] = horizontal[2] * north[0] - horizontal[0] * north[2];
            cross[2] = horizontal[0] * north[1] - horizontal[1] * north[0];
            sine = cross[0] * gravity[0] + cross[1] * gravity[1] + cross[2] * gravity[2];
            cosine = horizontal[0] * north[0] + horizontal[1] * north[1] + horizontal[2] * north[2];
            heading_error = atan2f(sine, cosine); /* 跨 ±180° 取最短路径 */
            for (i = 0; i < 3; ++i)
            {
                gyro[i] += 1.5f * heading_error * gravity[i]; /* 航向比例增益1.5/s */
            }
            mag_used = 1;
        }
    }

    /* 4. 用修正后的三轴角速度递推四元数，再归一化。 */
    dt *= 0.5f;
    s_q.q0 += (-q1 * gyro[0] - q2 * gyro[1] - q3 * gyro[2]) * dt;
    s_q.q1 += (q0 * gyro[0] - q3 * gyro[1] + q2 * gyro[2]) * dt;
    s_q.q2 += (q3 * gyro[0] + q0 * gyro[1] - q1 * gyro[2]) * dt;
    s_q.q3 += (-q2 * gyro[0] + q1 * gyro[1] + q0 * gyro[2]) * dt;
    norm = sqrtf(s_q.q0 * s_q.q0 + s_q.q1 * s_q.q1 + s_q.q2 * s_q.q2 + s_q.q3 * s_q.q3);
    s_q.q0 /= norm;
    s_q.q1 /= norm;
    s_q.q2 /= norm;
    s_q.q3 /= norm;

    /* 5. 反解欧拉角，输出单位为度；限制asin输入，避免浮点舍入越界。 */
    pitch_sine = 2.0f * (s_q.q0 * s_q.q2 - s_q.q1 * s_q.q3);
    if (pitch_sine > 1.0f)
    {
        pitch_sine = 1.0f;
    }
    if (pitch_sine < -1.0f)
    {
        pitch_sine = -1.0f;
    }
    euler->pitch = asinf(pitch_sine) * RtA;
    euler->roll = atan2f(2.0f * (s_q.q0 * s_q.q1 + s_q.q2 * s_q.q3), 1.0f - 2.0f * (s_q.q1 * s_q.q1 + s_q.q2 * s_q.q2)) * RtA;
    /* yaw 同样由四元数反解，倾斜时不再把机体 Gyro_z 当地面航向角速度。 */
    euler->yaw = atan2f(2.0f * (s_q.q0 * s_q.q3 + s_q.q1 * s_q.q2), 1.0f - 2.0f * (s_q.q2 * s_q.q2 + s_q.q3 * s_q.q3)) * RtA;
    return mag_used;
}

void Common_IMU_GetEulerAngle(Accel_Gyro_struct *raw, Euler_struct *euler, float dt)
{
    (void)Common_IMU_GetEulerAngleWithMag(raw, NULL, euler, dt);
}
