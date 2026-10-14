/**
 * @file bl24c02.c
 * @brief BL24C02F读写；STOP后内部写入最长3ms，用带超时的ACK轮询等待。
 *        等待时让出CPU，不持有快照锁，不关闭整次事务的中断。
 */
#include "bl24c02.h"
#include "my_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

#define BL24C02_READY_TIMEOUT_MS 10U

static bl24c02_error s_error;
static uint16_t s_address;

bl24c02_error bl24c02_get_error(uint16_t *address)
{
    if (address != NULL)
    {
        *address = s_address;
    }
    return s_error;
}

static uint8_t bl24c02_wait_ready(void)
{
    TickType_t start = xTaskGetTickCount();
    while (my_i2c_probe(MY_I2C_BUS_EEPROM, BL24C02_ADDR) != 0)
    {
        if ((TickType_t)(xTaskGetTickCount() - start) >= pdMS_TO_TICKS(BL24C02_READY_TIMEOUT_MS))
        {
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return 0;
}

void bl24c02_init(void)
{
    my_i2c_init(MY_I2C_BUS_EEPROM);
}

uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length)
{
    s_error = BL24C02_ERROR_NONE;
    s_address = address;
    if (address > BL24C02_CAPACITY_BYTES || length > BL24C02_CAPACITY_BYTES - address || (data == NULL && length != 0))
    {
        s_error = BL24C02_ERROR_ARGUMENT;
        return 1;
    }
    if (length == 0)
    {
        return 0;
    }
    if (bl24c02_wait_ready() != 0)
    {
        s_error = BL24C02_ERROR_READY_TIMEOUT;
        return 1;
    }
    if (my_i2c_read_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)address, data, length) != 0)
    {
        s_error = BL24C02_ERROR_READ_NACK;
        return 1;
    }
    return 0;
}

uint8_t bl24c02_write(uint16_t address, const uint8_t *data, uint16_t length)
{
    uint16_t chunk;
    s_error = BL24C02_ERROR_NONE;
    s_address = address;
    if (address > BL24C02_CAPACITY_BYTES || length > BL24C02_CAPACITY_BYTES - address || (data == NULL && length != 0))
    {
        s_error = BL24C02_ERROR_ARGUMENT;
        return 1;
    }
    if (length != 0 && bl24c02_wait_ready() != 0)
    {
        s_error = BL24C02_ERROR_READY_TIMEOUT;
        return 1;
    }
    while (length != 0)
    {
        /* 页写只递增低4位，跨页会覆盖页头；每段必须在16字节边界内。 */
        chunk = BL24C02_PAGE_BYTES - (address % BL24C02_PAGE_BYTES);
        if (chunk > length)
        {
            chunk = length;
        }
        if (my_i2c_write_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)address, data, chunk) != 0)
        {
            s_error = BL24C02_ERROR_WRITE_NACK;
            return 1;
        }
        if (bl24c02_wait_ready() != 0)
        {
            s_error = BL24C02_ERROR_READY_TIMEOUT;
            return 1;
        }
        address += chunk;
        data += chunk;
        length -= chunk;
        if (length != 0)
        {
            s_address = address;
        }
    }
    return 0;
}
