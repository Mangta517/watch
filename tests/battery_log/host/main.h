#ifndef BATTERY_LOG_HOST_MAIN_H
#define BATTERY_LOG_HOST_MAIN_H
#include <stdint.h>

typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t ACR, CR, SR;
} FLASH_TypeDef;
extern FLASH_TypeDef host_flash;
#define FLASH (&host_flash)
#define FLASH_ACR_DCEN 0x400U
#define __HAL_FLASH_DATA_CACHE_DISABLE() (FLASH->ACR &= ~FLASH_ACR_DCEN)
#define __HAL_FLASH_DATA_CACHE_ENABLE() (FLASH->ACR |= FLASH_ACR_DCEN)
void host_cache_reset(void);
#define __HAL_FLASH_DATA_CACHE_RESET() host_cache_reset()

typedef struct
{
    uint32_t TypeErase, Sector, NbSectors, VoltageRange;
} FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_SECTOR_7 7U
#define FLASH_VOLTAGE_RANGE_3 3U
#define FLASH_TYPEPROGRAM_HALFWORD 1U
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define __HAL_FLASH_CLEAR_FLAG(flags) (FLASH->SR &= ~(flags))
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void NVIC_SystemReset(void);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error);
typedef struct
{
    uint32_t SR, DR;
} USART_TypeDef;
extern USART_TypeDef host_log_uart;
#define USART1 (&host_log_uart)
#define USART_SR_RXNE 0x20
#define FLASH_CR_PSIZE 0x300
#define FLASH_PSIZE_HALF_WORD 0x100
#define FLASH_CR_PG 1
#define FLASH_SR_BSY 0x10000
void host_log_program(uint32_t address, uint16_t value);
uint32_t host_log_poll(void);
#define BATTERY_LOG_PROGRAM_HALFWORD(address, value) host_log_program(address, value)
#define BATTERY_LOG_POLL_STATUS() host_log_poll()
#endif
