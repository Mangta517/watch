#ifndef APP_RESET_H
#define APP_RESET_H

#include <stdint.h>

/* 复位原因与异常现场记录：使用RTC备份域BKP1..BKP19，掉电或备份域被清后记录失效。
 * 开机读取上一份记录并立即落一份"本次运行中"的骨架，之后每次实际喂狗推进检查点；
 * 硬件异常/LVGL断言/栈溢出时提交现场并停机，由外部看门狗复位。
 * 记录字段单位：uptime_ms/last_feed_ms/max_feed_gap_ms为毫秒，其余为寄存器或标志原值。 */

enum
{
    APP_RESET_RUNNING = 0,
    APP_RESET_HARDFAULT,
    APP_RESET_MEMMANAGE,
    APP_RESET_BUSFAULT,
    APP_RESET_USAGEFAULT,
    APP_RESET_STACK_OVERFLOW,
    APP_RESET_ERROR_HANDLER,
    APP_RESET_LVGL_ASSERT
};

typedef struct
{
    uint32_t reset_flags;
    uint32_t valid;
    uint32_t kind;
    uint32_t uptime_ms;
    uint32_t last_feed_ms;
    uint32_t max_feed_gap_ms;
    uint32_t sp;
    uint32_t exc_return;
    uint32_t pc;
    uint32_t lr;
    uint32_t xpsr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t task_tag;
} app_reset_info_struct;

/* main在HAL/时钟初始化前调用；保留本次RCC_CSR和上一启动记录，不改RTC时钟源。 */
void app_reset_init(void);
/* 只在实际完成喂狗后调用；首拍建立起点，后续统计两次喂狗间隔。 */
void app_reset_watchdog_feed(uint32_t now_ms);
/* stack为异常入口的原始MSP/PSP；记录后由调用方停机，等待原有看门狗。
   只有非栈类故障且压栈帧完整落在SRAM内时才采信帧内PC/LR/xPSR。 */
void app_reset_capture_fault(uint32_t kind, const uint32_t *stack, uint32_t exc_return);
/* 无硬件帧的停机记录：pc可为0，task_tag为栈溢出钩子压入的前4个任务名字节。 */
void app_reset_capture_stop(uint32_t kind, uint32_t pc, uint32_t task_tag);
/* LVGL断言记录调用者地址后停机，等待已有看门狗复位。 */
void app_reset_assert_stop(void);
/* 固定返回启动时读到的上一记录；无有效记录时仅reset_flags有效。 */
void app_reset_get_info(app_reset_info_struct *info);

#endif
