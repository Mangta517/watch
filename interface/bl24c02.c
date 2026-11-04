/**
 * @file bl24c02.c
 * @brief BL24C02F读写；STOP后内部写入最长3ms，用带超时的ACK轮询等待。
 *        等待时让出CPU，不持有快照锁，不关闭整次事务的中断。
 */
#include "bl24c02.h"
#include "my_i2c.h"
#ifndef F411_BOOTLOADER
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "main.h"
#include <stddef.h>

#define BL24C02_READY_TIMEOUT_MS 10

static bl24c02_error s_error;
static uint16_t s_address;

void bl24c02_transfer_begin(bl24c02_transfer_struct *transfer, uint16_t address, uint8_t *data, uint16_t length, uint8_t write)
{
    /* 记录调用方缓冲区首址，后续分页推进时随已处理字节数一起后移。 */
    transfer->data = data;
    /* 保存本次事务的起始地址，作为当前读写游标使用。 */
    transfer->address = address;
    /* 写段起始地址，可在页内；初值同事务地址，提交写段前更新。 */
    transfer->page_address = address;
    /* 剩余未处理字节数，传输阶段每完成一段就减去该段长度。 */
    transfer->remaining = length;
    /* 非0表示写方向，0表示读方向。 */
    transfer->write = write;
    /* 记录起始毫秒时刻，供就绪轮询的超时判断使用。 */
    transfer->started_ms = HAL_GetTick();
    transfer->phase = 0; /* 0等就绪、1传输、2完成、3失败。 */
    /* 清除上一次事务遗留的错误码，本次结果只反映本次执行。 */
    s_error = BL24C02_ERROR_NONE;
    /* 对外错误地址先指向本次起始地址，出错时再按需更新。 */
    s_address = address;
    /* 地址越界、长度超出容量尾部，或长度为0以外的空指针都视为参数错误。 */
    if (address > BL24C02_CAPACITY_BYTES || length > BL24C02_CAPACITY_BYTES - address || (data == NULL && length != 0))
    {
        /* 记录参数错误，便于调用方通过bl24c02_get_error查询。 */
        s_error = BL24C02_ERROR_ARGUMENT;
        /* 直接进入失败态，后续step不再发起总线传输。 */
        transfer->phase = 3;
    }
}

static uint8_t bl24c02_transfer_step_active(bl24c02_transfer_struct *transfer)
{
    /* 本拍要处理的字节数：不超过当前页剩余空间，也不超过剩余长度。 */
    uint16_t chunk;
    /* my_i2c成块读写的结果，0=成功、1=NACK或参数错误。 */
    uint8_t result;
    /* 默认把对外错误地址指向当前读写游标。 */
    s_address = transfer->address;
    /* 写方向的就绪阶段以刚提交写入段的起始地址作为出错地址。 */
    if (transfer->phase == 0 && transfer->write != 0)
    {
        s_address = transfer->page_address;
    }
    /* 已失败：保持失败结果，不再访问总线。 */
    if (transfer->phase == 3)
    {
        return 1;
    }
    /* 已完成：直接返回完成结果。 */
    if (transfer->phase == 2)
    {
        return 0;
    }
    /* 就绪阶段：等待EEPROM内部写周期结束，或确认总线可用。 */
    if (transfer->phase == 0)
    {
        /* 先探测再判超时，正常页写在下一拍(10ms)已完成，不因调度间隔误报。 */
        if (my_i2c_probe(MY_I2C_BUS_EEPROM, BL24C02_ADDR) != 0)
        {
            /* 器件仍未ACK，累计等待达到超时上限就判为就绪超时。 */
            if ((uint32_t)(HAL_GetTick() - transfer->started_ms) >= BL24C02_READY_TIMEOUT_MS)
            {
                s_error = BL24C02_ERROR_READY_TIMEOUT;
                transfer->phase = 3;
                return 1;
            }
            /* 尚未超时：本拍不传数据，返回推进中，由调用方下次再调用。 */
            return 2;
        }
        /* 器件已就绪且没有待处理字节，本次事务直接完成。 */
        if (transfer->remaining == 0)
        {
            transfer->phase = 2;
            return 0;
        }
        /* 器件就绪且有数据待传：切换到传输阶段，本拍不传，下一拍开始。 */
        transfer->phase = 1;
        return 2;
    }
    /* 传输阶段：计算当前地址到本页末尾的剩余空间，页写不能跨页。 */
    chunk = BL24C02_PAGE_BYTES - transfer->address % BL24C02_PAGE_BYTES;
    /* 剩余长度更小时，本段只处理剩余部分。 */
    if (chunk > transfer->remaining)
    {
        chunk = transfer->remaining;
    }
    /* 写方向：从当前游标开始写一段，并保存本次写入段的起始地址。 */
    if (transfer->write != 0)
    {
        transfer->page_address = transfer->address;
        result = my_i2c_write_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)transfer->address, transfer->data, chunk);
        s_error = BL24C02_ERROR_WRITE_NACK;
    }
    /* 读方向：从当前游标开始读一段，读操作可以跨页。 */
    else
    {
        result = my_i2c_read_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)transfer->address, transfer->data, chunk);
        s_error = BL24C02_ERROR_READ_NACK;
    }
    /* 总线返回失败：保留刚记录的错误码并进入失败态。 */
    if (result != 0)
    {
        transfer->phase = 3;
        return 1;
    }
    /* 本段传输成功，清除可能残留的错误码。 */
    s_error = BL24C02_ERROR_NONE;
    /* 游标、缓冲区指针和剩余长度同步前进本段字节数。 */
    transfer->address += chunk;
    transfer->data += chunk;
    transfer->remaining -= chunk;
    /* 写方向：本段只是页写命令完成，还需等内部写周期，回到就绪阶段并重置超时起点。 */
    if (transfer->write != 0)
    {
        transfer->phase = 0;
        transfer->started_ms = HAL_GetTick();
    }
    /* 读方向：剩余长度为0说明整次读取完成。 */
    else if (transfer->remaining == 0)
    {
        transfer->phase = 2;
        return 0;
    }
    /* 其余情况：还有数据待处理，返回推进中。 */
    return 2;
}

uint8_t bl24c02_transfer_step(bl24c02_transfer_struct *transfer)
{
    /* 保存内部单步执行的结果，返回前先释放总线引脚。 */
    uint8_t result;
    /* 每步使用前重新初始化EEPROM总线，把SCL/SDA恢复为开漏输出并释放总线。 */
    my_i2c_init(MY_I2C_BUS_EEPROM);
    /* 执行一拍状态机：探测就绪或传输一段数据。 */
    result = bl24c02_transfer_step_active(transfer);
    /* 本步结束即释放PA11/PA12，避免长期占用引脚。 */
    my_i2c_suspend(MY_I2C_BUS_EEPROM);
    /* 原样返回0完成、1失败、2推进中。 */
    return result;
}

bl24c02_error bl24c02_get_error(uint16_t *address)
{
    /* 调用方给出地址指针时，回传最近一次读写的出错或当前页地址。 */
    if (address != NULL)
    {
        *address = s_address;
    }
    /* 返回最近一次读写记录的错误码。 */
    return s_error;
}

static uint8_t bl24c02_wait_ready(void)
{
#ifdef F411_BOOTLOADER
    /* Bootloader无RTOS：以HAL毫秒节拍记录等待起点。 */
    uint32_t start = HAL_GetTick();
    /* 未ACK不一定表示内部写周期在忙：未探测到ACK时继续轮询，直到ACK或超时。 */
    while (my_i2c_probe(MY_I2C_BUS_EEPROM, BL24C02_ADDR) != 0)
    {
        /* 等待时间达到10ms上限仍未就绪，返回失败。 */
        if ((uint32_t)(HAL_GetTick() - start) >= BL24C02_READY_TIMEOUT_MS)
        {
            return 1;
        }
        /* 忙等期间阻塞1ms，给EEPROM内部写周期留出时间。 */
        HAL_Delay(1);
    }
#else
    /* RTOS环境：以系统tick计数记录等待起点。 */
    TickType_t start = xTaskGetTickCount();
    /* 同样轮询器件ACK，未就绪则继续等待。 */
    while (my_i2c_probe(MY_I2C_BUS_EEPROM, BL24C02_ADDR) != 0)
    {
        /* 把10ms超时门限换算成tick，与已等待tick数比较，达到即返回失败。 */
        if ((TickType_t)(xTaskGetTickCount() - start) >= pdMS_TO_TICKS(BL24C02_READY_TIMEOUT_MS))
        {
            return 1;
        }
        /* 让出CPU一个tick，其他任务可在此期间运行。 */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
#endif
    /* 探测到ACK或循环未进入，说明器件已就绪。 */
    return 0;
}

void bl24c02_init(void)
{
    /* 初始化EEPROM专用软件I2C的SCL/SDA引脚。 */
    my_i2c_init(MY_I2C_BUS_EEPROM);
    /* 初始化后立即释放总线，改为模拟无上下拉状态。 */
    my_i2c_suspend(MY_I2C_BUS_EEPROM);
}

static uint8_t bl24c02_read_active(uint16_t address, uint8_t *data, uint16_t length)
{
    /* 每次调用先清除旧错误码，结果只反映本次读取。 */
    s_error = BL24C02_ERROR_NONE;
    /* 记录本次读取起始地址，供错误查询使用。 */
    s_address = address;
    /* 起始地址越界、长度越过256字节容量末尾、或非零长度配空指针都判为参数错误。 */
    if (address > BL24C02_CAPACITY_BYTES || length > BL24C02_CAPACITY_BYTES - address || (data == NULL && length != 0))
    {
        s_error = BL24C02_ERROR_ARGUMENT;
        return 1;
    }
    /* 长度为0无需访问总线，直接视为成功。 */
    if (length == 0)
    {
        return 0;
    }
    /* 读之前先确认器件已完成上一次内部写周期。 */
    if (bl24c02_wait_ready() != 0)
    {
        s_error = BL24C02_ERROR_READY_TIMEOUT;
        return 1;
    }
    /* 一次成块读；EEPROM顺序读可跨页，因此无需分段。 */
    if (my_i2c_read_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)address, data, length) != 0)
    {
        s_error = BL24C02_ERROR_READ_NACK;
        return 1;
    }
    /* 读到全部字节且无NACK，返回成功。 */
    return 0;
}

static uint8_t bl24c02_write_active(uint16_t address, const uint8_t *data, uint16_t length)
{
    /* 本次页写段长度，按页边界和剩余长度取较小值。 */
    uint16_t chunk;
    /* 清除上一次错误码。 */
    s_error = BL24C02_ERROR_NONE;
    /* 记录本次写入起始地址。 */
    s_address = address;
    /* 地址或长度越界、非零长度配空指针均直接判为参数错误。 */
    if (address > BL24C02_CAPACITY_BYTES || length > BL24C02_CAPACITY_BYTES - address || (data == NULL && length != 0))
    {
        s_error = BL24C02_ERROR_ARGUMENT;
        return 1;
    }
    /* 有数据要写时，先等待EEPROM上一轮内部写周期结束。 */
    if (length != 0 && bl24c02_wait_ready() != 0)
    {
        s_error = BL24C02_ERROR_READY_TIMEOUT;
        return 1;
    }
    /* 按页分段写入，直到剩余长度减为0。 */
    while (length != 0)
    {
        /* 页写只递增低4位，跨页会覆盖页头；每段必须在16字节边界内。 */
        chunk = BL24C02_PAGE_BYTES - (address % BL24C02_PAGE_BYTES);
        /* 剩余长度不足一整页时只写剩余部分。 */
        if (chunk > length)
        {
            chunk = length;
        }
        /* 发起本页写命令，NACK即判为写失败。 */
        if (my_i2c_write_bytes(MY_I2C_BUS_EEPROM, BL24C02_ADDR, (uint8_t)address, data, chunk) != 0)
        {
            s_error = BL24C02_ERROR_WRITE_NACK;
            return 1;
        }
        /* 页写命令完成后等待器件内部写周期结束，未就绪判为超时。 */
        if (bl24c02_wait_ready() != 0)
        {
            s_error = BL24C02_ERROR_READY_TIMEOUT;
            return 1;
        }
        /* 地址、数据指针前进本段长度，剩余长度相应减少。 */
        address += chunk;
        data += chunk;
        length -= chunk;
        /* 还有后续页时更新对外错误地址为下一页起始地址。 */
        if (length != 0)
        {
            s_address = address;
        }
    }
    /* 全部页写完且每页写周期均已确认，返回成功。 */
    return 0;
}

uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length)
{
    /* 保存内部读取结果，返回前先释放总线。 */
    uint8_t result;
    /* 使用前重新初始化EEPROM总线引脚。 */
    my_i2c_init(MY_I2C_BUS_EEPROM);
    /* 在总线可用状态下执行实际读取。 */
    result = bl24c02_read_active(address, data, length);
    /* 读取结束释放PA11/PA12。 */
    my_i2c_suspend(MY_I2C_BUS_EEPROM);
    /* 返回0成功、1失败。 */
    return result;
}

uint8_t bl24c02_write(uint16_t address, const uint8_t *data, uint16_t length)
{
    /* 保存内部写入结果，返回前先释放总线。 */
    uint8_t result;
    /* 使用前重新初始化EEPROM总线引脚。 */
    my_i2c_init(MY_I2C_BUS_EEPROM);
    /* 在总线可用状态下执行按页写入。 */
    result = bl24c02_write_active(address, data, length);
    /* 写入结束释放PA11/PA12。 */
    my_i2c_suspend(MY_I2C_BUS_EEPROM);
    /* 返回0成功、1失败。 */
    return result;
}
