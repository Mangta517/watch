/**
 ******************************************************************************
 * @file    spl006.h
 * @brief   SPL006 气压计驱动 —— 走软件 I2C SENSOR 总线 (PB14=SCL, PB13=SDA)
 *          移植自旧工程 flight_hal/interface/spl006.c (硬件 I2C1 版)，
 *          寄存器配置/校准系数解析/补偿公式保持一致，仅替换总线访问层。
 *
 * 地址说明：本板 SDO 接地 → 7 位地址 0x76 (SDO 接 VDD 则为 0x77)。
 *          软件 I2C 层内部自动拼接读写位，不再用旧工程的 0xEE/0xEF 移位地址。
 * 总线共享：与 MPU6050 同挂 SENSOR 总线，所有器件事务归口 sensor 任务。
 *          PB12 EXTI ISR 只记事件，任务读取 MPU6050 的 0x3A 释放锁存。
 *          不对整次 I2C 事务关中断；my_i2c 只保护 GPIO 方向切换的短操作。
 ******************************************************************************
 */
#ifndef __SPL006_H__
#define __SPL006_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

/* ------------------------------------------------------------ 器件 I2C 地址 */
#define SPL006_ADDR 0x76 /* 7 位地址: 本板 SDO 接地 */

/* ------------------------------------------------------------ 寄存器映射 */
#define SPL06_PRS_B2   0x00 /* 压力数据高字节 (23:16) */
#define SPL06_PRS_B1   0x01
#define SPL06_PRS_B0   0x02
#define SPL06_TMP_B2   0x03 /* 温度数据高字节 (23:16) */
#define SPL06_TMP_B1   0x04
#define SPL06_TMP_B0   0x05
#define SPL06_PRS_CFG  0x06 /* 压力配置 (速率+过采样) */
#define SPL06_TMP_CFG  0x07 /* 温度配置 */
#define SPL06_MEAS_CFG 0x08 /* 测量配置与状态 */
#define SPL06_CFG_REG  0x09 /* 中断与 FIFO 配置 (含 P/T_SHIFT) */
#define SPL06_RESET    0x0C /* 软复位 / FIFO 刷新 */
#define SPL06_ID       0x0D /* 产品 ID 与版本 */

/* PRS_CFG (0x06) 位定义 */
#define SPL06_PM_RATE_MASK 0x70 /* bit6:4 测量速率 */
#define SPL06_PM_RATE_1PS  0x00 /* 000: 1 次/秒 (手表显示够用, 省功耗) */
#define SPL06_PM_RATE_64PS 0x60 /* 110: 64 次/秒 (旧飞控定高用, 备用) */
#define SPL06_PM_PRC_MASK  0x0F /* bit3:0 过采样 */
#define SPL06_PM_PRC_16X   0x04 /* 0100: 16倍 (标准) */

/* TMP_CFG (0x07) 位定义 */
#define SPL06_TMP_EXT      0x80 /* bit7: 温度源 (1=MEMS 内部传感器) */
#define SPL06_TMP_PRC_MASK 0x07 /* bit2:0 过采样 (速率位同 PRS_CFG) */
#define SPL06_TMP_PRC_16X  0x04

/* MEAS_CFG (0x08) 位定义 */
#define SPL06_COEF_RDY       0x80 /* bit7: 校准系数就绪 */
#define SPL06_SENSOR_RDY     0x40 /* bit6: 传感器就绪 */
#define SPL06_TMP_RDY        0x20 /* bit5: 温度新数据 */
#define SPL06_PRS_RDY        0x10 /* bit4: 压力新数据 */
#define SPL06_MEAS_BG_PRSTMP 0x07 /* bit2:0 背景模式 (压力+温度) */

/* CFG_REG (0x09) 位定义 */
#define SPL06_T_SHIFT 0x08 /* bit3: 温度右移 (过采样>8倍时置1) */
#define SPL06_P_SHIFT 0x04 /* bit2: 压力右移 (过采样>8倍时置1) */

/* RESET (0x0C) 命令 */
#define SPL06_SOFT_RESET 0x09 /* 写 0x09 软复位 */

/* 校准系数寄存器 (手册表10; 与旧工程一致, 单字节逐个读) */
#define SPL06_C00_BYTE2 0x13
#define SPL06_C00_BYTE1 0x14
#define SPL06_C00_BYTE0 0x15 /* 高4位=c00[3:0], 低4位=c10[19:16] */
#define SPL06_C10_BYTE1 0x16
#define SPL06_C10_BYTE0 0x17
#define SPL06_C01_BYTE1 0x18
#define SPL06_C01_BYTE0 0x19
#define SPL06_C11_BYTE1 0x1A
#define SPL06_C11_BYTE0 0x1B
#define SPL06_C20_BYTE1 0x1C
#define SPL06_C20_BYTE0 0x1D
#define SPL06_C21_BYTE1 0x1E
#define SPL06_C21_BYTE0 0x1F
#define SPL06_C30_BYTE1 0x20
#define SPL06_C30_BYTE0 0x21

/* 补偿缩放因子: 16 倍过采样对应 253952 (init 固定配 16X+P_SHIFT, 只用这一档;
 * 改过采样率必须同步改此值, 查表见旧工程 spl006.h) */
#define SPL06_SCALE_FACTOR_16X 253952.0f

    /* ------------------------------------------------------------ 接口函数 */
    /* 返回值约定：0=成功，非 0=失败 (总线 NACK / 就绪超时 / 系数未就绪)          */

    /* 初始化: 软复位→等就绪→读校准系数→配 16X 过采样/1Hz 背景模式
 * (调用前需先 my_i2c_init(MY_I2C_BUS_SENSOR); 内含 HAL_Delay, 只可在
 *  调度器启动前的上下文调用一次 —— 与 my_mpu6050_init 同位置) */
    uint8_t spl006_init(void);

    /* 读原始压力/温度计数 (24bit 补码符号扩展): 0=成功, 1=数据未就绪, 2=总线失败 */
    uint8_t spl006_read_pressure(int32_t *pressure_raw);
    uint8_t spl006_read_temperature(int32_t *temperature_raw);

    /* 用出厂系数把原始计数补偿为气压 (Pa)。0=成功, 1=spl006_init 未完成(系数无效)
 * 公式: P = c00 + psc*(c10 + psc*(c20 + psc*c30)) + tsc*c01 + tsc*psc*(c11 + psc*c21)
 *       psc = praw/253952, tsc = traw/253952  */
    uint8_t spl006_process_pressure(int32_t pressure_raw, int32_t temperature_raw, int32_t *compensated_pressure);

#ifdef __cplusplus
}
#endif

#endif /* __SPL006_H__ */
