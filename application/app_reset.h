#ifndef APP_RESET_H
#define APP_RESET_H

#include <stdint.h>

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
/* stack为异常入口的原始MSP/PSP；记录后由调用方停机，等待原有看门狗。 */
void app_reset_capture_fault(uint32_t kind, const uint32_t *stack, uint32_t exc_return);
void app_reset_capture_stop(uint32_t kind, uint32_t pc, uint32_t task_tag);
/* LVGL断言记录调用者地址后停机，等待已有看门狗复位。 */
void app_reset_assert_stop(void);
/* 固定返回启动时读到的上一记录；无有效记录时仅reset_flags有效。 */
void app_reset_get_info(app_reset_info_struct *info);

#endif
