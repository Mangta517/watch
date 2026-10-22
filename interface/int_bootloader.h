#ifndef INT_BOOTLOADER_H
#define INT_BOOTLOADER_H

#include "main.h"

#define APP_START_ADDR                 0x08008000U
#define APP_FLASH_END_ADDR            0x08080000U
#define APP_MAX_SIZE                  (APP_FLASH_END_ADDR - APP_START_ADDR)
#define BOOTLOADER_DMA_SIZE            8192U
#define BOOTLOADER_DMA_HALF_SIZE       (BOOTLOADER_DMA_SIZE / 2U)
#define BOOTLOADER_RX_IDLE_MS          10U
#define BOOTLOADER_COMPLETE_IDLE_MS    100U
#define BOOTLOADER_TRANSFER_TIMEOUT_MS 30000U

/* int层主循环更新统计；ISR只发布DMA半区完成/错误标记。 */
extern uint32_t uart1_rx_full_len;
extern uint32_t flash_write_offset;
extern uint32_t uart1_rx_complete_tick;
extern uint8_t uart1_rx_idle;
extern volatile uint32_t uart1_rx_error_count;
extern uint32_t uart1_rx_overflow_count;

HAL_StatusTypeDef int_bootloader_erase(uint32_t length);
HAL_StatusTypeDef bootloader_start_dma(uint32_t length);
void int_bootloader_expect_crc(uint32_t crc);
HAL_StatusTypeDef int_bootloader_process(void);
HAL_StatusTypeDef int_bootloader_finish(void);
HAL_StatusTypeDef int_bootloader_stop(void);
uint8_t int_bootloader_rx_dma_active(void);
void int_bootloader_rx_complete(void);
void int_bootloader_rx_error(void);
uint8_t int_bootloader_app_is_valid(void);
void int_bootloader_jump_to_app(void);

#endif
