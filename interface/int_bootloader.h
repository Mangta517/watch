#ifndef INT_BOOTLOADER_H
#define INT_BOOTLOADER_H

/**
 * @file int_bootloader.h
 * @brief 独立保底(Recovery)固件接收底层：USART1 循环 DMA 按双半区收 APP 镜像，
 *        逐半字写入 APP 区 Flash；前 8 字节向量暂存，长度、接收状态、向量及启用时的
 *        整幅 CRC-32/ISO-HDLC (与主机 zlib.crc32 一致) 检查通过后，提交向量并置 VALID 标志。
 *        擦除前持久化 UPDATING 标志，掉电后下次上电仍不跳入半幅程序。
 *        应用层帧协议与调度在 application/app_bootloader.c，本模块不解析命令。
 */

#include "main.h"
#include "boot_layout.h"

#define BOOTLOADER_DMA_SIZE            8192
#define BOOTLOADER_DMA_HALF_SIZE       (BOOTLOADER_DMA_SIZE / 2)
#define BOOTLOADER_RX_IDLE_MS          10
#define BOOTLOADER_COMPLETE_IDLE_MS    100
#define BOOTLOADER_TRANSFER_TIMEOUT_MS 30000

/* 接收长度/写入进度由主循环更新；ISR发布DMA半区完成位置并累加接收错误计数。 */
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
uint8_t int_bootloader_app_is_valid(void);
void int_bootloader_jump_to_app(void);

#endif
