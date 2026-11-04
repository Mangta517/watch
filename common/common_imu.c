/**
 * @file common_imu.c
 * @brief MPU6050 四元数互补融合 + 磁场水平投影航向修正。
 *        磁场只沿重力轴修正航向，避免磁倾角直接拉偏 pitch/roll。
 */
#include "common_imu.h"
#include <math.h>
#include <string.h>

const float RtA = 57.2957795f;
const float Gyro_G = 1.0f / MPU6050_GYRO_LSB_PER_DPS;
static const float Gyro_Gr = 1.0f / MPU6050_GYRO_LSB_PER_DPS / 180.0f * 3.1415926f;

static Quaternion_Struct s_q = {1, 0, 0, 0};
static float s_error_i[3];
static uint8_t s_gravity_pending = 1;

void Common_IMU_Reset(void)
{
    /* 回到单位四元数: q0=1 表示无旋转, 向量部清零。 */
    s_q.q0 = 1.0f;
    s_q.q1 = s_q.q2 = s_q.q3 = 0.0f;
    /* 清空加速度修正的积分项, 避免旧姿态累积的修正带入新姿态。 */
    memset(s_error_i, 0, sizeof(s_error_i));
    /* 标记需要由首个有效加速度样本重新建立俯仰/横滚。 */
    s_gravity_pending = 1;
}

uint8_t Common_IMU_GetEulerAngleWithMag(const Accel_Gyro_struct *raw, const float mag_uT[3], Euler_struct *euler, float dt)
{
    /* 三轴数组统一按机体X/Y/Z排列；四元数q0为标量，q1/q2/q3为向量。 */
    float q0 = s_q.q0, q1 = s_q.q1, q2 = s_q.q2, q3 = s_q.q3;
    float gravity[3];     /* 四元数预测的单位重力向量 */
    float accel[3];       /* 加速度，先为LSB，再归一化 */
    float gyro[3];        /* 修正后的机体角速度，rad/s */
    float error[3] = {0}; /* 加速度与预测重力的方向误差 */
    /* norm: 向量模长(先用于加速度归一化, 后用于水平磁场归一化与四元数归一化);
     * dot: 磁场在重力方向上的投影; half_roll/half_pitch: 初始姿态的半角。 */
    float norm, dot, half_roll, half_pitch;
    float north[3];      /* 四元数预测的单位磁北向量 */
    float horizontal[3]; /* 实测磁场的水平投影 */
    float cross[3];      /* 水平磁场与预测磁北的叉积 */
    float heading_error; /* 航向误差，rad */
    /* sine/cosine: 航向误差在重力轴上的正弦/余弦分量; pitch_sine: 俯仰角正弦值(限幅前)。 */
    float sine, cosine, pitch_sine;
    /* i: 三轴循环下标; mag_used: 本拍是否实际使用了磁航向修正, 作为返回值。 */
    uint8_t i, mag_used = 0;

    if (raw == NULL || !isfinite(dt) || dt <= 0.0f)
    {
        /* 无数据或积分间隔非法(非有限值/非正): 不递推四元数, 返回未用磁。 */
        return 0;
    }

    /* 取三轴原始LSB；用对应1g的LSB阈值判幅值，再直接除模长归一化。 */
    accel[0] = raw->Accel.Accel_x;
    accel[1] = raw->Accel.Accel_y;
    accel[2] = raw->Accel.Accel_z;
    norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    /* 首个有效重力样本建立俯仰/横滚；倒置时叉积为零，不能从正Z初值自行收敛。
     * 航向从0开始，仍由后续磁参考修正；仅启动/显式Reset后执行一次。 */
    if (s_gravity_pending != 0 && norm > 0.75f * MPU6050_ACCEL_LSB_PER_G && norm < 1.25f * MPU6050_ACCEL_LSB_PER_G)
    {
        /* 由加速度方向直接算出横滚/俯仰半角, 再用半角三角值组合出初始四元数。 */
        half_roll = 0.5f * atan2f(accel[1], accel[2]);
        half_pitch = 0.5f * atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
        s_q.q0 = q0 = cosf(half_roll) * cosf(half_pitch);
        s_q.q1 = q1 = sinf(half_roll) * cosf(half_pitch);
        s_q.q2 = q2 = cosf(half_roll) * sinf(half_pitch);
        s_q.q3 = q3 = -sinf(half_roll) * sinf(half_pitch);
        /* 初始姿态已建立, 后续不再重复此步骤。 */
        s_gravity_pending = 0;
    }

    /* 1. 提取机体坐标下的重力方向，作为俯仰/横滚参考。 */
    /* 由四元数计算世界Z轴在机体坐标中的分量。 */
    gravity[0] = 2.0f * (q1 * q3 - q0 * q2);
    gravity[1] = 2.0f * (q0 * q1 + q2 * q3);
    gravity[2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    /* 2. 仅在0.75～1.25g内用加速度修正重力方向，排除明显跑步冲击。 */
    if (norm > 0.75f * MPU6050_ACCEL_LSB_PER_G && norm < 1.25f * MPU6050_ACCEL_LSB_PER_G)
    {
        /* 归一化加速度, 只保留方向, 使误差与加速度幅值无关。 */
        for (i = 0; i < 3; ++i)
        {
            accel[i] /= norm;
        }
        /* 实测重力方向与四元数预测重力方向叉乘, 得到三轴方向误差。 */
        error[0] = accel[1] * gravity[2] - accel[2] * gravity[1];
        error[1] = accel[2] * gravity[0] - accel[0] * gravity[2];
        error[2] = accel[0] * gravity[1] - accel[1] * gravity[0];
        for (i = 0; i < 3; ++i)
        {
            s_error_i[i] += error[i] * 0.03f * dt; /* 积分反馈按实际dt累计，dt单位为s */
        }
    }
    /* 陀螺仪原始LSB换算为rad/s: Gyro_Gr = 1/LSB每dps ÷ 180 × π。 */
    gyro[0] = raw->Gyro.Gyro_x * Gyro_Gr;
    gyro[1] = raw->Gyro.Gyro_y * Gyro_Gr;
    gyro[2] = raw->Gyro.Gyro_z * Gyro_Gr;
    for (i = 0; i < 3; ++i)
    {
        /* 比例项(0.8)加积分项修正角速度, 让重力方向向实测加速度收敛。 */
        gyro[i] += 0.8f * error[i] + s_error_i[i];
    }

    /* 3. 磁场投影到水平面，仅沿重力轴修正航向，不直接拉动俯仰/横滚。 */
    if (mag_uT != NULL && isfinite(mag_uT[0]) && isfinite(mag_uT[1]) && isfinite(mag_uT[2]))
    {
        /* 磁场与重力单位向量点乘, 得到磁场沿重力轴的分量。 */
        dot = mag_uT[0] * gravity[0] + mag_uT[1] * gravity[1] + mag_uT[2] * gravity[2];
        for (i = 0; i < 3; ++i)
        {
            /* 从磁场中减去该分量, 得到与重力正交的水平投影。 */
            horizontal[i] = mag_uT[i] - dot * gravity[i];
        }
        norm = sqrtf(horizontal[0] * horizontal[0] + horizontal[1] * horizontal[1] + horizontal[2] * horizontal[2]);
        if (norm > 5.0f) /* 水平磁场大于5μT时才修正航向 */
        {
            /* 水平投影归一化, 使其只表示方向。 */
            for (i = 0; i < 3; ++i)
            {
                horizontal[i] /= norm;
            }
            /* R^T*[1,0,0]：当前四元数预测的磁北，已与重力正交。 */
            north[0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
            north[1] = 2.0f * (q1 * q2 - q0 * q3);
            north[2] = 2.0f * (q1 * q3 + q0 * q2);
            /* 实测水平磁场与预测磁北叉乘, 得到绕重力轴的误差方向。 */
            cross[0] = horizontal[1] * north[2] - horizontal[2] * north[1];
            cross[1] = horizontal[2] * north[0] - horizontal[0] * north[2];
            cross[2] = horizontal[0] * north[1] - horizontal[1] * north[0];
            /* 叉积投影到重力轴得误差正弦, 两向量点乘得误差余弦。 */
            sine = cross[0] * gravity[0] + cross[1] * gravity[1] + cross[2] * gravity[2];
            cosine = horizontal[0] * north[0] + horizontal[1] * north[1] + horizontal[2] * north[2];
            heading_error = atan2f(sine, cosine); /* 跨 ±180° 取最短路径 */
            for (i = 0; i < 3; ++i)
            {
                /* 航向误差只沿重力轴注入角速度, 不改变俯仰/横滚。 */
                gyro[i] += 1.5f * heading_error * gravity[i]; /* 航向比例增益1.5/s */
            }
            mag_used = 1; /* 标记本拍已用磁参考。 */
        }
    }

    /* 4. 用修正后的三轴角速度递推四元数，再归一化。 */
    dt *= 0.5f; /* 四元数微分方程的1/2系数并入步长。 */
    /* dt已含1/2，下面以本拍起始q计算q⊗ω*dt的增量。 */
    s_q.q0 += (-q1 * gyro[0] - q2 * gyro[1] - q3 * gyro[2]) * dt;
    s_q.q1 += (q0 * gyro[0] - q3 * gyro[1] + q2 * gyro[2]) * dt;
    s_q.q2 += (q3 * gyro[0] + q0 * gyro[1] - q1 * gyro[2]) * dt;
    s_q.q3 += (-q2 * gyro[0] + q1 * gyro[1] + q0 * gyro[2]) * dt;
    /* 数值积分会积累模长误差, 除以模长恢复单位四元数。 */
    norm = sqrtf(s_q.q0 * s_q.q0 + s_q.q1 * s_q.q1 + s_q.q2 * s_q.q2 + s_q.q3 * s_q.q3);
    s_q.q0 /= norm;
    s_q.q1 /= norm;
    s_q.q2 /= norm;
    s_q.q3 /= norm;

    /* 5. 反解欧拉角，输出单位为度；限制asin输入，避免浮点舍入越界。 */
    if (euler == NULL)
    {
        return mag_used; /* 只递推四元数，按需省略三角函数输出。 */
    }
    pitch_sine = 2.0f * (s_q.q0 * s_q.q2 - s_q.q1 * s_q.q3);
    if (pitch_sine > 1.0f)
    {
        pitch_sine = 1.0f; /* 上限幅到1, 防止asin定义域越界。 */
    }
    if (pitch_sine < -1.0f)
    {
        pitch_sine = -1.0f; /* 下限幅到-1。 */
    }
    /* 三轴欧拉角由四元数分量反解, 再乘RtA由弧度换算为度。 */
    euler->pitch = asinf(pitch_sine) * RtA;
    euler->roll = atan2f(2.0f * (s_q.q0 * s_q.q1 + s_q.q2 * s_q.q3), 1.0f - 2.0f * (s_q.q1 * s_q.q1 + s_q.q2 * s_q.q2)) * RtA;
    /* yaw 同样由四元数反解，倾斜时不再把机体 Gyro_z 当地面航向角速度。 */
    euler->yaw = atan2f(2.0f * (s_q.q0 * s_q.q3 + s_q.q1 * s_q.q2), 1.0f - 2.0f * (s_q.q2 * s_q.q2 + s_q.q3 * s_q.q3)) * RtA;
    return mag_used;
}

void Common_IMU_GetEulerAngle(Accel_Gyro_struct *raw, Euler_struct *euler, float dt)
{
    /* 不传磁参考, 仅用加速度与陀螺仪做姿态解算。 */
    (void)Common_IMU_GetEulerAngleWithMag(raw, NULL, euler, dt);
}
