#ifndef INT_BOOTLOADER_H
#define INT_BOOTLOADER_H

#include "main.h"
#include "int_battery_log.h"

#define APP_START_ADDR                 0x08008000
#define APP_FLASH_END_ADDR             BATTERY_LOG_START_ADDR
#define APP_MAX_SIZE                   (APP_FLASH_END_ADDR - APP_START_ADDR)
#define BOOTLOADER_DMA_SIZE            8192
#define BOOTLOADER_DMA_HALF_SIZE       (BOOTLOADER_DMA_SIZE / 2)
#define BOOTLOADER_RX_IDLE_MS          10
#define BOOTLOADER_COMPLETE_IDLE_MS    100
#define BOOTLOADER_TRANSFER_TIMEOUT_MS 30000
#define BOOTLOADER_EEPROM_FLAG_ADDR    0xC0

/* int层主循环更新统计；ISR只发布DMA半区完成/错误标记。 */
extern uint32_t uart1_rx_full_len;
extern uint32_t flash_write_offset;
extern uint32_t uart1_rx_complete_tick;
extern uint8_t uart1_rx_idle;
extern volatile uint32_t uart1_rx_error_count;
extern uint32_t uart1_rx_overflow_count;

/* 操作结果：0成功、1失败；有效性/活动状态查询仍返回0否、1是。 */
uint8_t int_bootloader_erase(uint32_t length);
uint8_t bootloader_start_dma(uint32_t length);
void int_bootloader_expect_crc(uint32_t crc);
uint8_t int_bootloader_process(void);
uint8_t int_bootloader_finish(void);
uint8_t int_bootloader_stop(void);
uint8_t int_bootloader_rx_dma_active(void);
void int_bootloader_rx_complete(void);
void int_bootloader_rx_error(void);
uint8_t int_bootloader_app_is_valid(void);
void int_bootloader_jump_to_app(void);

#endif
