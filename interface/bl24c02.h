/**
 * @file bl24c02.h
 * @brief BL24C02F EEPROM：256字节、16字节页，PA12=SCL、PA11=SDA独立软件I2C。
 *        仅sensor任务调用；WP接低电平允许写入，SCL/SDA需外部上拉。
 */
#ifndef __BL24C02_H
#define __BL24C02_H

#include <stdint.h>

#define BL24C02_ADDR           0x50
#define BL24C02_CAPACITY_BYTES 256U
#define BL24C02_PAGE_BYTES     16U

typedef enum
{
    BL24C02_ERROR_NONE,
    BL24C02_ERROR_ARGUMENT,
    BL24C02_ERROR_READY_TIMEOUT,
    BL24C02_ERROR_READ_NACK,
    BL24C02_ERROR_WRITE_NACK
} bl24c02_error;

void bl24c02_init(void);
/* 返回0=成功、1=参数/通信/写周期超时；read可跨页，write自动按页边界拆分。 */
uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length);
uint8_t bl24c02_write(uint16_t address, const uint8_t *data, uint16_t length);
/* 最近一次读写的错误与页地址；仅sensor任务调用，不增加总线访问。 */
bl24c02_error bl24c02_get_error(uint16_t *address);

#endif
