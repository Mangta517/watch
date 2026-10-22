/**
 * @file bl24c02.c
 * @brief BL24C02F读写；STOP后内部写入最长3ms，用带超时的ACK轮询等待。
 *        等待时让出CPU，不持有快照锁，不关闭整次事务的中断。
 */
#include "bl24c02.h"
#include "my_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stddef.h>

#define BL24C02_READY_TIMEOUT_MS 10U

static bl24c02_error s_error;
static uint16_t s_address;

void bl24c02_transfer_begin(bl24c02_transfer_struct *transfer, uint16_t address, uint8_t *data, uint16_t length, uint8_t write)
{
    transfer->data = data;
    transfer->address = address;
    transfer->page_address = address;
    transfer->remaining = length;
    transfer->write = write;
    transfer->started_ms = HAL_GetTick();
    transfer->phase = 0; /* 0等就绪、1传输、2完成、3失败。 */
    s_error = BL24C02_ERROR_NONE;
    s_address = address;
    if (address > BL24C02_CAPACITY_BYTES || length > BL24C02_CAPACITY_BYTES - address || (data == NULL && length != 0))
    {
        s_error = BL24C02_ERROR_ARGUMENT;
        transfer->phase = 3;
    }
}

uint8_t bl24c02_transfer_step(bl24c02_transfer_struct *transfer)
{
    uint16_t chunk;
    uint8_t result;
    s_address = transfer->address;
    if (transfer->phase == 0 && transfer->write != 0)
    {
        s_address = transfer->page_address;
    }
    if (transfer->phase == 3)
    {
        return 1;
    }
    if (transfer->phase == 2)
    {
        return 0;
    }
    if (transfer->phase == 0)
    {
        /* 先探测再判超时，正常页写在下一拍(10ms)已完成，不因调度间隔误报。 */
        if (my_i2c_probe(MY_I2C_BUS_EEPROM, BL24C02_ADDR) != 0)
        {
            if ((uint32_t)(HAL_GetTick() - transfer->started_ms) >= BL24C02_READY_TIMEOUT_MS)
            {
                s_error = BL24C02_ERROR_READY_TIMEOUT;
                transfer->phase = 3;
                return 1;
            }
            return 2;
        }
        if (transfer->remaining == 0)
        {
            transfer->phase = 2;
            return 0;
        }
        transfer->phase = 1;
        return 2;
    }
    chunk = BL24C02_PAGE_BYTES - transfer->address % BL24C02_PAGE_BYTES;
    if (chunk > transfer->remaining)
    {
        chunk = transfer->remaining;
    }
    if (transfer->write != 0)
    {
        transfer->page_address = transfer->address;
        result = my_i2c_write_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)transfer->address, transfer->data, chunk);
        s_error = BL24C02_ERROR_WRITE_NACK;
    }
    else
    {
        result = my_i2c_read_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)transfer->address, transfer->data, chunk);
        s_error = BL24C02_ERROR_READ_NACK;
    }
    if (result != 0)
    {
        transfer->phase = 3;
        return 1;
    }
    s_error = BL24C02_ERROR_NONE;
    transfer->address += chunk;
    transfer->data += chunk;
    transfer->remaining -= chunk;
    if (transfer->write != 0)
    {
        transfer->phase = 0;
        transfer->started_ms = HAL_GetTick();
    }
    else if (transfer->remaining == 0)
    {
        transfer->phase = 2;
        return 0;
    }
    return 2;
}

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
