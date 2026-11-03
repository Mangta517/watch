#include "int_battery_log.h"
#include "kt6368a.h"
#include <string.h>

/* 测试替换寄存器的副作用，固件直接写半字、读取BSY，不调用Flash中的HAL。 */
#ifndef BATTERY_LOG_PROGRAM_HALFWORD
#define BATTERY_LOG_PROGRAM_HALFWORD(address, value) (*(volatile uint16_t *)(address) = (value))
#endif
#ifndef BATTERY_LOG_POLL_STATUS
#define BATTERY_LOG_POLL_STATUS() (FLASH->SR)
#endif

/* 两个APP的scatter把此段复制到SRAM；循环及常量均不能依赖Flash取指。 */
#if defined(__CC_ARM)
__attribute__((section("battery_log_ram"), noinline))
#endif
static uint8_t battery_log_program_halfword(uint32_t address, uint16_t value, uint16_t received[8], uint8_t *count, uint8_t uart_enabled)
{
    uint32_t status, uart_status;
    uint16_t byte;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE) | FLASH_PSIZE_HALF_WORD | FLASH_CR_PG;
    BATTERY_LOG_PROGRAM_HALFWORD(address, value);
    do
    {
        status = BATTERY_LOG_POLL_STATUS();
        if (uart_enabled != 0)
        {
            uart_status = USART1->SR;
            if ((uart_status & (USART_SR_RXNE | 0x0F)) != 0)
            {
                byte = (uint16_t)((USART1->DR & 0xFF) | ((uart_status & 0x0F) << 8));
                if (*count < 8)
                {
                    received[(*count)++] = byte;
                }
                else
                {
                    received[7] |= 0x0800; /* 不把异常长等待导致的溢出伪装成完整消息。 */
                }
            }
        }
    }
    while ((status & FLASH_SR_BSY) != 0);
    FLASH->CR &= ~FLASH_CR_PG;
    return (status & (FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != 0;
}

static void battery_log_refresh_cache(void)
{
    uint32_t enabled = FLASH->ACR & FLASH_ACR_DCEN;
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    if (enabled != 0)
    {
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

void int_battery_log_read(uint32_t slot, uint8_t record[BATTERY_LOG_RECORD_BYTES])
{
    uint32_t i;
    if (slot >= BATTERY_LOG_CAPACITY)
    {
        memset(record, 0xFF, BATTERY_LOG_RECORD_BYTES);
        return;
    }
    for (i = 0; i < BATTERY_LOG_RECORD_BYTES; ++i)
    {
        record[i] = *(volatile const uint8_t *)(BATTERY_LOG_START_ADDR + slot * BATTERY_LOG_RECORD_BYTES + i);
    }
}

uint8_t int_battery_log_write(uint32_t slot, const uint8_t record[BATTERY_LOG_RECORD_BYTES])
{
    uint8_t old[BATTERY_LOG_RECORD_BYTES];
    uint32_t i;
    uint16_t value;
    uint16_t received[8];
    uint8_t count, byte, uart_enabled;
    uint32_t mask;
    HAL_StatusTypeDef result;

    if (slot >= BATTERY_LOG_CAPACITY)
    {
        return 1;
    }
    int_battery_log_read(slot, old);
    for (i = 0; i < sizeof(old); ++i)
    {
        if (old[i] != 0xFF)
        {
            return 1;
        }
    }
    /* sensor持有日志锁；单次编程在SRAM轮询UART，半字之间恢复中断，最后写CRC。 */
    result = HAL_FLASH_Unlock();
    if (result == HAL_OK)
    {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        for (i = 0; i < BATTERY_LOG_RECORD_BYTES; i += 2)
        {
            value = (uint16_t)((uint16_t)record[i] | ((uint16_t)record[i + 1] << 8));
            mask = __get_PRIMASK();
            __disable_irq();
            count = 0;
            uart_enabled = kt6368a_rx_is_enabled();
            result = HAL_OK;
            if (battery_log_program_halfword(BATTERY_LOG_START_ADDR + slot * BATTERY_LOG_RECORD_BYTES + i,
                                            value, received, &count, uart_enabled) != 0)
            {
                result = HAL_ERROR;
            }
            for (byte = 0; byte < count; ++byte)
            {
                kt6368a_receive_polled((uint8_t)received[byte], (uint8_t)(received[byte] >> 8));
            }
            __set_PRIMASK(mask);
            if (result != HAL_OK)
            {
                break;
            }
        }
        if (HAL_FLASH_Lock() != HAL_OK)
        {
            result = HAL_ERROR;
        }
        battery_log_refresh_cache();
    }
    int_battery_log_read(slot, old);
    if (memcmp(record, old, sizeof(old)) != 0)
    {
        result = HAL_ERROR;
    }
    return result != HAL_OK;
}

void int_battery_log_erase(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error;
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return;
    }
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    (void)HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();
    battery_log_refresh_cache();
}
