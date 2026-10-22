#ifndef BOOTLOADER_HOST_FLASH_H
#define BOOTLOADER_HOST_FLASH_H

typedef struct
{
    uint32_t TypeErase, Sector, NbSectors, VoltageRange;
} FLASH_EraseInitTypeDef;

extern uint32_t test_dma_remaining;
extern uint32_t test_dma_tc;
#define FLASH_BASE 0x08000000U
#define FLASH_SECTOR_4 4U
#define FLASH_SECTOR_5 5U
#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_VOLTAGE_RANGE_3 3U
#define FLASH_TYPEPROGRAM_HALFWORD 1U
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define RESET 0U
#define __HAL_FLASH_CLEAR_FLAG(flags) ((void)(flags))
#define __HAL_DMA_GET_COUNTER(dma) ((void)(dma), test_dma_remaining)
#define __HAL_DMA_GET_TC_FLAG_INDEX(dma) ((void)(dma), 1U)
#define __HAL_DMA_GET_FLAG(dma, flag) ((void)(dma), (void)(flag), test_dma_tc)
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart);
void NVIC_SystemReset(void);

#endif
