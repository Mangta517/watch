/**
 * @file app_bootloader.h
 * @brief 独立保底(recovery)程序的应用层协议状态机；由保底裸机主循环(不启用RTOS)每轮调用work()。
 *        收固件只发生在这里(保底)，主引导仅选择跳转；协议经KT6368A的UART收发。
 */
#ifndef APP_BOOTLOADER_H
#define APP_BOOTLOADER_H

#include "int_bootloader.h"

/* 文本命令的静默收尾阈值(毫秒)：升级链路走BLE，默认MTU 23下手机单次写入的有效载荷为20字节；
 * 例如'start:224792:12345678\n'为22字节，必然拆成20+2到达(并非所有start命令都是22字节)。
 * 分片之间可能有20~50ms延迟，因此命令结束只能按本阈值判定，
 * 与DMA数据的 BOOTLOADER_RX_IDLE_MS(10ms) 相互独立，不能复用后者。
 * 命令自带CR/LF时仍按旧协议立即结算，本阈值只兜住不带换行的旧主机。 */
#define BOOTLOADER_COMMAND_IDLE_MS 1000

/* work()按此状态推进：INIT解析start命令，RUN擦除并开循环DMA，REC_DATA边收BIN边分块写Flash，
 * CHECK_DATA完成收尾，启用CRC协议时才校验整幅CRC；JUMP_TO_APP留发送窗口后跳转。
 * start:len为旧协议，允许不带CRC；任一步失败走fail()复位，重新选择启动目标。 */
typedef enum
{
    BOOTLOADER_STATE_DEFAULT,
    BOOTLOADER_STATE_INIT,
    BOOTLOADER_STATE_RUN,
    BOOTLOADER_STATE_REC_DATA,
    BOOTLOADER_STATE_CHECK_DATA,
    BOOTLOADER_STATE_JUMP_TO_APP
} Bootloader_state;

extern Bootloader_state boot_state; /* 本程序唯一状态机变量，主循环单写单读 */
/* 发一次欢迎/用法提示并把状态切到INIT，等待上位机 'start:len[:crc32]'。 */
void app_bootloader_init(void);
/* 保底裸机主循环每轮调用；依据boot_state解析命令、驱动擦除/接收/校验并回执或复位。
 * 单执行流，回执用带1000ms超时的阻塞发送即可，不需要APP里的异步分时发送。 */
void app_bootloader_work(void);

#endif
