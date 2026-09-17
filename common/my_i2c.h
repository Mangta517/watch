/**
 ******************************************************************************
 * @file    my_i2c.h
 * @brief   软件 I2C（GPIO 模拟）驱动 —— STM32F411CEU6，双总线
 *
 * 总线分配（引脚定义见 Core/Inc/main.h）：
 *   MY_I2C_BUS_SENSOR : SCL = PB14 (SENSOR_SCL)  SDA = PB13 (SENSOR_SDA)  → MPU6050 等传感器
 *   MY_I2C_BUS_TP     : SCL = PB6  (TP_SCL)      SDA = PB4  (TP_SDA)      → 触摸屏
 * 模式：SCL 推挽输出；SDA 推挽输出(发送相) / 输入+上拉(读 ACK/数据相) 逐相切换
 *       —— 与 touch.c 实测可用方案一致。开漏+内部上拉的上升沿太缓, 已弃用。
 * 速率：约 100~150 kHz（由 MY_I2C_DELAY_LOOPS 决定）
 * 并发：SDA 方向切换走 GPIOB 非原子 RMW, 已在内部用 PRIMASK 临界区保护,
 *       与 touch.c 及 EXTI ISR 互斥（touch.c 的方向切换同样加了保护）。
 *
 * 注意：位操作时序要求总线在单次传输期间不被其他任务打断。
 *       与 FreeRTOS/LVGL 多任务共存时，跨任务共用同一总线需自行加互斥保护。
 ******************************************************************************
 */
#ifndef __MY_I2C_H
#define __MY_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 可选的两条软件 I2C 总线 */
typedef enum
{
    MY_I2C_BUS_SENSOR = 0,   /* PB14=SCL, PB13=SDA, MPU6050 */
    MY_I2C_BUS_TP     = 1,   /* PB6 =SCL, PB4 =SDA, 触摸屏   */
    MY_I2C_BUS_COUNT
} my_i2c_bus_t;

/* ---------------------------------------------------------------- 底层时序 */
void    my_i2c_init(my_i2c_bus_t bus);              /* 初始化该总线 GPIO 并释放总线 */
void    my_i2c_start(my_i2c_bus_t bus);             /* 起始信号：SCL 高期间 SDA 下降沿 */
void    my_i2c_stop(my_i2c_bus_t bus);              /* 停止信号：SCL 高期间 SDA 上升沿 */
uint8_t my_i2c_wait_ack(my_i2c_bus_t bus);          /* 等 slave ACK：0=ACK, 1=NACK */
void    my_i2c_send_ack(my_i2c_bus_t bus, uint8_t ack); /* 主机发应答：0=ACK, 1=NACK */
void    my_i2c_write_byte(my_i2c_bus_t bus, uint8_t dat);           /* MSB first */
uint8_t my_i2c_read_byte(my_i2c_bus_t bus, uint8_t ack);            /* MSB first */

/* ---------------------------------------------------------- 成块读写封装 */
/* dev_addr 为 7 位地址（内部自动移位），reg_addr 为 8 位寄存器地址。
 * 返回：0=成功，1=NACK/参数错误                                    */
uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr,
                           const uint8_t *dat, uint16_t len);
uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr,
                          uint8_t *dat, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MY_I2C_H */
