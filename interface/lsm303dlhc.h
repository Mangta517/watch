/**
 * @file lsm303dlhc.h
 * @brief LSM303DLHC 磁力计；SENSOR 软件 I2C，7 位地址 0x1E。
 *        仅 sensor 任务可调用；加速度计保持上电默认 power-down。
 */
#ifndef __LSM303DLHC_H
#define __LSM303DLHC_H

#include "imu_types.h"

#define LSM303DLHC_MAG_ADDR 0x1E

typedef enum
{
    LSM303DLHC_MAG_OK,
    LSM303DLHC_MAG_NOT_READY,
    LSM303DLHC_MAG_ERROR
} lsm303dlhc_mag_result;

/* 总线初始化后由sensor任务调用；0=成功，1=通信/识别/配置失败。 */
uint8_t lsm303dlhc_mag_init(void);
/* 仅sensor任务：750/300/75对应75/30/7.5Hz；单位0.1Hz，失败进入原恢复流程。 */
uint8_t lsm303dlhc_mag_set_rate(uint16_t rate_hz_x10);
uint8_t lsm303dlhc_mag_set_sleep(uint8_t sleeping);
/* sensor每拍调用：0=就绪，1=失败冷却，2=推进中；每次至多一笔I2C事务。 */
void lsm303dlhc_mag_recover_begin(void);
uint8_t lsm303dlhc_mag_recover_step(uint32_t now_ms);
/* 状态轮询 + 完整六字节突发读；失败或无新数据时保留调用方输出。 */
lsm303dlhc_mag_result lsm303dlhc_mag_read(lsm303dlhc_mag_struct *mag);

#endif
