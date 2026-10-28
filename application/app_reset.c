#include "app_reset.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define APP_RESET_MAGIC        0x52535431
#define APP_RESET_VERSION      1
#define APP_RESET_RECORD_WORDS 19
#define APP_RESET_SRAM_START   0x20000000
#define APP_RESET_SRAM_END     0x20020000
#define APP_RESET_STACK_ERRORS (SCB_CFSR_MSTKERR_Msk | SCB_CFSR_MUNSTKERR_Msk | SCB_CFSR_MLSPERR_Msk | \
                                SCB_CFSR_STKERR_Msk | SCB_CFSR_UNSTKERR_Msk | SCB_CFSR_LSPERR_Msk)

static app_reset_info_struct s_previous;
static app_reset_info_struct s_current;
static uint8_t s_have_feed;
static volatile uint8_t s_stopped;

static void reset_backup_enable(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;
    (void)PWR->CR; /* APB1分频下DBP生效有延迟，按手册先读回。 */
    RCC->BDCR |= RCC_BDCR_RTCEN;
}

static volatile uint32_t *reset_backup_word(uint32_t index)
{
    /* BKP0保留给RTC校时标记；本记录仅占BKP1..19。 */
    return (volatile uint32_t *)((uintptr_t)&RTC->BKP0R + (index + 1) * sizeof(uint32_t));
}

static uint32_t reset_checksum(const uint32_t *words)
{
    uint32_t crc = UINT32_MAX;
    uint32_t i, bit;
    for (i = 1; i < APP_RESET_RECORD_WORDS - 1; ++i)
    {
        crc ^= words[i];
        for (bit = 0; bit < 32; ++bit)
        {
            if ((crc & 1) != 0)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc ^ UINT32_MAX;
}

static void reset_commit(const uint32_t *words)
{
    uint32_t i;
    reset_backup_enable();
    *reset_backup_word(0) = 0; /* 先撤销提交标记；中途复位不能把半条记录当成有效。 */
    __DMB();
    for (i = 1; i < APP_RESET_RECORD_WORDS; ++i)
    {
        *reset_backup_word(i) = words[i];
    }
    __DMB();
    *reset_backup_word(0) = APP_RESET_MAGIC;
    __DSB();
}

static void reset_store(void)
{
    uint32_t words[APP_RESET_RECORD_WORDS];
    words[0] = APP_RESET_MAGIC;
    words[1] = APP_RESET_VERSION;
    memcpy(words + 2, &s_current, sizeof(s_current));
    words[APP_RESET_RECORD_WORDS - 1] = reset_checksum(words);
    reset_commit(words);
}

void app_reset_init(void)
{
    uint32_t words[APP_RESET_RECORD_WORDS];
    uint32_t flags = RCC->CSR;
    uint32_t mask = __get_PRIMASK();
    uint32_t i;
    __disable_irq();
    reset_backup_enable();
    for (i = 0; i < APP_RESET_RECORD_WORDS; ++i)
    {
        words[i] = *reset_backup_word(i);
    }
    memset(&s_previous, 0, sizeof(s_previous));
    if (words[0] == APP_RESET_MAGIC && words[1] == APP_RESET_VERSION &&
        words[APP_RESET_RECORD_WORDS - 1] == reset_checksum(words))
    {
        memcpy(&s_previous, words + 2, sizeof(s_previous));
        s_previous.valid = 1;
    }
    s_previous.reset_flags = flags;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    memset(&s_current, 0, sizeof(s_current));
    s_current.valid = 1;
    s_have_feed = 0;
    s_stopped = 0;
    reset_store();
    __set_PRIMASK(mask);
}

void app_reset_watchdog_feed(uint32_t now_ms)
{
    uint32_t words[APP_RESET_RECORD_WORDS];
    uint32_t mask = __get_PRIMASK();
    uint32_t gap_ms;
    __disable_irq();
    if (s_stopped != 0)
    {
        __set_PRIMASK(mask);
        return;
    }
    if (s_have_feed != 0)
    {
        gap_ms = now_ms - s_current.last_feed_ms;
        if (gap_ms > s_current.max_feed_gap_ms)
        {
            s_current.max_feed_gap_ms = gap_ms;
        }
    }
    s_current.uptime_ms = now_ms;
    s_current.last_feed_ms = now_ms;
    s_have_feed = 1;
    words[0] = APP_RESET_MAGIC;
    words[1] = APP_RESET_VERSION;
    memcpy(words + 2, &s_current, sizeof(s_current));
    __set_PRIMASK(mask);

    /* 正常喂狗的CRC不屏蔽UART；异常若在此期间接管，旧检查点不能覆盖异常现场。 */
    words[APP_RESET_RECORD_WORDS - 1] = reset_checksum(words);
    __disable_irq();
    if (s_stopped == 0)
    {
        reset_commit(words); /* RTC清备份域后，下一拍仍重新提交完整记录。 */
    }
    __set_PRIMASK(mask);
}

static void reset_capture_registers(uint32_t kind)
{
    s_stopped = 1;
    s_current.kind = kind;
    s_current.uptime_ms = HAL_GetTick();
    s_current.cfsr = SCB->CFSR;
    s_current.hfsr = SCB->HFSR;
    s_current.mmfar = SCB->MMFAR;
    s_current.bfar = SCB->BFAR;
}

void app_reset_capture_fault(uint32_t kind, const uint32_t *stack, uint32_t exc_return)
{
    uintptr_t address = (uintptr_t)stack;
    uint32_t frame_bytes = 8 * sizeof(uint32_t);
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (s_stopped == 0)
    {
        reset_capture_registers(kind);
        s_current.sp = (uint32_t)address;
        s_current.exc_return = exc_return;
        if ((exc_return & (1U << 4)) == 0)
        {
            frame_bytes = 26 * sizeof(uint32_t);
        }
        if ((s_current.cfsr & APP_RESET_STACK_ERRORS) == 0 && (address & 3) == 0 &&
            address >= APP_RESET_SRAM_START && address <= APP_RESET_SRAM_END - frame_bytes)
        {
            /* Cortex-M4F扩展帧的R0..xPSR仍在前8字，S0..S15/FPSCR在其后。 */
            s_current.lr = stack[5];
            s_current.pc = stack[6];
            s_current.xpsr = stack[7];
        }
        reset_store();
    }
    __set_PRIMASK(mask);
}

void app_reset_capture_stop(uint32_t kind, uint32_t pc, uint32_t task_tag)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (s_stopped == 0)
    {
        reset_capture_registers(kind);
        s_current.pc = pc;
        s_current.task_tag = task_tag;
        reset_store();
    }
    __set_PRIMASK(mask);
}

void app_reset_assert_stop(void)
{
    __disable_irq();
#if defined(__CC_ARM)
    app_reset_capture_stop(APP_RESET_LVGL_ASSERT, (uint32_t)__return_address(), 0);
#else
    app_reset_capture_stop(APP_RESET_LVGL_ASSERT, (uint32_t)(uintptr_t)__builtin_return_address(0), 0);
#endif
    for (;;)
    {
    }
}

void app_reset_get_info(app_reset_info_struct *info)
{
    if (info != NULL)
    {
        *info = s_previous;
    }
}
