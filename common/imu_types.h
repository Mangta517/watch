#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdint.h>

/* 驱动、快照和算法共享的数据契约；保留既有类型名、枚举值及EEPROM布局，不暴露硬件操作。 */
#define MPU6050_GYRO_LSB_PER_DPS 32.8f
#define MPU6050_ACCEL_LSB_PER_G  8192

typedef struct
{
    int16_t Gyro_x; /* 原始或减零偏后的计数，单位LSB。 */
    int16_t Gyro_y;
    int16_t Gyro_z;
} Gyro_struct;

typedef struct
{
    int16_t Accel_x; /* 计数，单位LSB；静止时包含重力投影。 */
    int16_t Accel_y;
    int16_t Accel_z;
} Accel_struct;

typedef struct
{
    Gyro_struct Gyro;
    Accel_struct Accel;
} Accel_Gyro_struct;

typedef enum
{
    MPU6050_CAL_IDLE,
    MPU6050_CAL_WAIT_STILL,
    MPU6050_CAL_AVERAGING,
    MPU6050_CAL_DONE,
    MPU6050_CAL_FAILED,
    MPU6050_CAL_RESTORED
} mpu6050_cal_state;

typedef struct
{
    int16_t accel[3]; /* X/Y/Z加速度LSB零偏，Z轴不包含重力1g。 */
    int16_t gyro[3];  /* X/Y/Z陀螺仪LSB零偏。 */
} mpu6050_offset_struct;

typedef struct
{
    int16_t raw[3];    /* 芯片X/Y/Z顺序，不是寄存器的X/Z/Y顺序。 */
    float field_uT[3]; /* 标称灵敏度换算，尚未做安装校准。 */
} lsm303dlhc_mag_struct;

#endif
