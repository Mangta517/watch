/** MPU六轴零偏掉电保存；EEPROM 0x60..0x9F，与原磁校准记录分开。 */
#ifndef APP_IMU_STORE_H
#define APP_IMU_STORE_H
#include "mpu6050.h"

typedef enum
{
    APP_IMU_STORE_OK,
    APP_IMU_STORE_EMPTY,
    APP_IMU_STORE_ERROR
} app_imu_store_result;

/* EEPROM诊断type=5的阶段码；原保存返回值和记录格式保持不变。 */
typedef enum
{
    APP_IMU_SAVE_NONE,
    APP_IMU_SAVE_ARGUMENT,
    APP_IMU_SAVE_READ_SLOT0,
    APP_IMU_SAVE_READ_SLOT1,
    APP_IMU_SAVE_WRITE,
    APP_IMU_SAVE_VERIFY_READ,
    APP_IMU_SAVE_VERIFY_DATA,
    APP_IMU_SAVE_DONE
} app_imu_save_stage;

/* 仅sensor任务调用；load检查格式、量程和CRC，不进行上电静止校准。 */
app_imu_store_result app_imu_store_load(mpu6050_offset_struct *offset);
app_imu_store_result app_imu_store_save(const mpu6050_offset_struct *offset);
#endif
