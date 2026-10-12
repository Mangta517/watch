/**
 * @file lsm303dlhc.h
 * @brief LSM303DLHC 磁力计；SENSOR 软件 I2C，7 位地址 0x1E。
 *        仅 sensor 任务可调用；加速度计保持上电默认 power-down。
 */
#ifndef __LSM303DLHC_H
#define __LSM303DLHC_H

#include <stdint.h>

#define LSM303DLHC_MAG_ADDR 0x1E

typedef struct
{
    int16_t raw[3];    /* 芯片 X/Y/Z 顺序，不是寄存器的 X/Z/Y 顺序 */
    float field_uT[3]; /* 标称灵敏度换算，尚未做安装校准 */
} lsm303dlhc_mag_struct;

typedef enum
{
    LSM303DLHC_MAG_OK,
    LSM303DLHC_MAG_NOT_READY,
    LSM303DLHC_MAG_ERROR
} lsm303dlhc_mag_result;

/* 总线初始化后、调度器启动前调用；0=成功，1=通信/识别/配置失败。 */
uint8_t lsm303dlhc_mag_init(void);
/* 状态轮询 + 完整六字节突发读；失败或无新数据时保留调用方输出。 */
lsm303dlhc_mag_result lsm303dlhc_mag_read(lsm303dlhc_mag_struct *mag);

#endif
