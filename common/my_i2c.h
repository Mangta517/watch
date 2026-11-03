/**
 ******************************************************************************
 * @file    my_i2c.h
 * @brief   软件 I2C（GPIO 模拟）驱动 —— STM32F411CEU6，三条独立总线
 *
 * 总线分配（引脚定义见 Core/Inc/main.h）：
 *   MY_I2C_BUS_SENSOR : SCL = PB14 (SENSOR_SCL)  SDA = PB13 (SENSOR_SDA)  → MPU6050 等传感器
 *   MY_I2C_BUS_TP     : SCL = PB6  (TP_SCL)      SDA = PB4  (TP_SDA)      → 触摸屏
 *   MY_I2C_BUS_EEPROM : SCL = PA12 (EEPROM_SCL) SDA = PA11 (EEPROM_SDA) → BL24C02F
 * 模式：SENSOR/EEPROM 的 SCL/SDA 开漏输出，需外部上拉到 3.3V。
 *       TP 保留原推挽发送方式；读 ACK/数据、发送 NACK 时 SDA 切换为输入释放。
 *       SENSOR 外部上拉参考 2～4.7kΩ；内部上拉不能替代外部上拉。
 * 节拍：SENSOR 位延时为原值两倍，TP/EEPROM 保持原值；实际频率需实测。
 * 并发：SDA 方向切换走 GPIOB 非原子 RMW, 已在内部用 PRIMASK 临界区保护,
 *       与 touch.c 的方向切换互斥；EXTI ISR 不操作 I2C。
 *
 * SENSOR 总线在调度器启动后仅由 sensor 任务串行访问；UI 只读快照。
 * 触摸任务使用独立 TP 总线，优先级低于 sensor；不访问 SENSOR 引脚。
 * 若新增同总线使用者，须先改所有调用方的任务级互斥约定，禁止 ISR 直接访问。
 ******************************************************************************
 */
#ifndef __MY_I2C_H
#define __MY_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

    /* EEPROM与传感器在调度器启动后均由sensor任务独占；UI只读快照。 */
    typedef enum
    {
        MY_I2C_BUS_SENSOR = 0, /* PB14=SCL, PB13=SDA, MPU6050 */
        MY_I2C_BUS_TP = 1,     /* PB6 =SCL, PB4 =SDA, 触摸屏   */
        MY_I2C_BUS_EEPROM = 2, /* PA12=SCL, PA11=SDA, EEPROM */
        MY_I2C_BUS_COUNT
    } my_i2c_bus_t;

    /* ---------------------------------------------------------------- 底层时序 */
    void my_i2c_init(my_i2c_bus_t bus); /* 初始化该总线 GPIO 并释放总线 */
    /* owner完成器件休眠后调用：释放SCL/SDA并设为模拟无上下拉；使用前重新init。 */
    void my_i2c_suspend(my_i2c_bus_t bus);
    /* 仅一笔事务首次START前调用：线路非空闲时最多恢复一次；0=可开始，1=总线仍故障。 */
    uint8_t my_i2c_prepare(my_i2c_bus_t bus);
    /* 释放SDA，最多9个时钟+STOP；每次等待SCL拉高均有界。TP调用前后失效方向缓存。 */
    uint8_t my_i2c_recover(my_i2c_bus_t bus);
    void my_i2c_start(my_i2c_bus_t bus);                     /* 起始信号：SCL 高期间 SDA 下降沿 */
    void my_i2c_stop(my_i2c_bus_t bus);                      /* 停止信号：SCL 高期间 SDA 上升沿 */
    uint8_t my_i2c_wait_ack(my_i2c_bus_t bus);               /* 等 slave ACK：0=ACK, 1=NACK；返回时 SDA 保持输入释放 */
    void my_i2c_send_ack(my_i2c_bus_t bus, uint8_t ack);     /* 主机发应答：0=拉低ACK, 1=释放SDA发送NACK */
    void my_i2c_write_byte(my_i2c_bus_t bus, uint8_t dat);   /* MSB first */
    uint8_t my_i2c_read_byte(my_i2c_bus_t bus, uint8_t ack); /* MSB first */

    /* ---------------------------------------------------------- 成块读写封装 */
    /* dev_addr 为 7 位地址（内部自动移位），reg_addr 为 8 位寄存器地址。
 * 返回：0=成功，1=NACK/参数错误                                    */
    uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *dat, uint16_t len);
    uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *dat, uint16_t len);

    /* 只发送器件写地址并检查ACK，不写寄存器；用于EEPROM内部写周期轮询。 */
    uint8_t my_i2c_probe(my_i2c_bus_t bus, uint8_t dev_addr);

#ifdef __cplusplus
}
#endif

#endif /* __MY_I2C_H */
