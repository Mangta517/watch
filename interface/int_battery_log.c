#include "int_battery_log.h"
#include <string.h>

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
    /* sensor持有日志互斥锁；最后写CRC。半字之间允许UART中断，不整条屏蔽接收。 */
    result = HAL_FLASH_Unlock();
    if (result == HAL_OK)
    {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        for (i = 0; i < BATTERY_LOG_RECORD_BYTES; i += 2)
        {
            value = (uint16_t)((uint16_t)record[i] | ((uint16_t)record[i + 1] << 8));
            result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                       BATTERY_LOG_START_ADDR + slot * BATTERY_LOG_RECORD_BYTES + i, value);
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
