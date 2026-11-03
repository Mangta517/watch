#include "app_reset.h"
#include "main.h"
#include "../../Middlewares/LVGL/lvgl/src/misc/lv_assert.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif

#define SRAM_START 0x20000000U
#define SRAM_BYTES 0x20000U
#define RESET_FLAGS_MASK 0xFE000000U
#define RTC_MARKER 0x52544331U

RCC_TypeDef host_rcc;
PWR_TypeDef host_pwr;
RTC_TypeDef host_rtc;
SCB_Type host_scb;
static uint32_t host_tick;
static uint32_t host_primask;
static uint32_t clear_count;
static uint32_t commit_count;
static jmp_buf assert_halt;
static uint8_t stop_at_commit;
static uint8_t track_checksum;
static uint32_t checksum_bits;
static uint32_t inject_at_bit;
static uint32_t inject_kind;

void host_reset_checksum_bit(void)
{
    if (track_checksum != 0)
    {
        assert(host_primask == 0);
        ++checksum_bits;
        if (checksum_bits == inject_at_bit)
        {
            track_checksum = 0;
            host_tick = 975;
            if (inject_kind == APP_RESET_HARDFAULT)
            {
                app_reset_capture_fault(APP_RESET_HARDFAULT, NULL, 0xFFFFFFFD);
            }
            else
            {
                app_reset_capture_stop(APP_RESET_ERROR_HANDLER, 0x08005678, 4);
            }
            track_checksum = 1;
        }
    }
}

uint32_t __get_PRIMASK(void)
{
    return host_primask;
}

void __disable_irq(void)
{
    host_primask = 1;
}

void __set_PRIMASK(uint32_t mask)
{
    host_primask = mask;
}

void __DMB(void)
{
    assert(host_primask == 1);
    assert(host_rtc.backup[1] == 0);
}

void __DSB(void)
{
    assert(host_primask == 1);
    assert(host_rtc.backup[1] != 0);
    ++commit_count;
    if (stop_at_commit != 0)
    {
        stop_at_commit = 0;
        longjmp(assert_halt, 1);
    }
}

uint32_t HAL_GetTick(void)
{
    return host_tick;
}

void host_clear_reset_flags(void)
{
    host_rcc.CSR &= ~RESET_FLAGS_MASK;
    ++clear_count;
}

static void cold_boot(void)
{
    memset(&host_rcc, 0, sizeof(host_rcc));
    memset(&host_pwr, 0, sizeof(host_pwr));
    memset(&host_rtc, 0, sizeof(host_rtc));
    memset(&host_scb, 0, sizeof(host_scb));
    host_tick = 0;
    host_primask = 0;
    host_rtc.backup[0] = RTC_MARKER;
    host_rcc.CSR = (1U << 27) | 3U;
    app_reset_init();
}

static app_reset_info_struct reboot(uint32_t flags)
{
    app_reset_info_struct info;
    host_rcc.CSR = flags;
    host_tick = 0;
    app_reset_init();
    app_reset_get_info(&info);
    assert(info.reset_flags == flags);
    assert((host_rcc.CSR & RESET_FLAGS_MASK) == 0);
    assert(host_rtc.backup[0] == RTC_MARKER);
    return info;
}

static void test_checkpoint(void)
{
    app_reset_info_struct info, unchanged;
    uint32_t flags = (1U << 27) | 3U;
    cold_boot();
    app_reset_get_info(&info);
    assert(info.valid == 0 && info.reset_flags == flags);
    info.reset_flags = 0;
    memset(&unchanged, 0, sizeof(unchanged));
    assert(memcmp(&info, &unchanged, sizeof(info)) == 0);
    assert((host_rcc.APB1ENR & RCC_APB1ENR_PWREN) != 0);
    assert((host_pwr.CR & PWR_CR_DBP) != 0);
    assert(host_rcc.BDCR == RCC_BDCR_RTCEN);
    assert(host_primask == 0 && clear_count != 0);

    app_reset_watchdog_feed(1700U);
    info = reboot(1U << 29);
    assert(info.valid == 1 && info.kind == APP_RESET_RUNNING);
    assert(info.last_feed_ms == 1700U && info.uptime_ms == 1700U);
    assert(info.max_feed_gap_ms == 0);
    unchanged = info;
    app_reset_watchdog_feed(0);
    app_reset_watchdog_feed(125U);
    app_reset_watchdog_feed(225U);
    app_reset_watchdog_feed(525U);
    app_reset_get_info(&info);
    assert(memcmp(&info, &unchanged, sizeof(info)) == 0);
    info = reboot(1U << 26);
    assert(info.max_feed_gap_ms == 300U && info.last_feed_ms == 525U);

    app_reset_watchdog_feed(UINT32_MAX - 49U);
    app_reset_watchdog_feed(50U);
    info = reboot(1U << 29);
    assert(info.last_feed_ms == 50U && info.max_feed_gap_ms == 100U);
    app_reset_get_info(NULL);
}

static void test_backup_reset_and_crc(void)
{
    uint32_t saved[20];
    uint32_t i, prefix;
    app_reset_info_struct info;
    cold_boot();
    app_reset_watchdog_feed(100U);
    for (i = 0; i < 20U; ++i)
    {
        saved[i] = host_rtc.backup[i];
    }
    for (i = 1; i < 20U; ++i)
    {
        memcpy((void *)host_rtc.backup, saved, sizeof(saved));
        host_rtc.backup[i] ^= 1U;
        info = reboot(1U << 26);
        assert(info.valid == 0 && info.kind == 0 && info.last_feed_ms == 0);
    }
    /* 模拟每个写入前缀后掉电：magic先清零，最后一次提交前都必须无效。 */
    for (prefix = 0; prefix <= 18U; ++prefix)
    {
        memset((void *)host_rtc.backup, 0xA5, sizeof(host_rtc.backup));
        host_rtc.backup[0] = RTC_MARKER;
        host_rtc.backup[1] = 0;
        for (i = 0; i < prefix; ++i)
        {
            host_rtc.backup[i + 2U] = saved[i + 2U];
        }
        info = reboot(1U << 27);
        assert(info.valid == 0);
    }
    app_reset_watchdog_feed(200U);
    /* RTC首次选择时钟源会清备份域；下一次喂狗须恢复完整记录和原有最大gap。 */
    memset((void *)host_rtc.backup, 0, sizeof(host_rtc.backup));
    host_rtc.backup[0] = RTC_MARKER;
    app_reset_watchdog_feed(600U);
    info = reboot(1U << 29);
    assert(info.valid == 1 && info.last_feed_ms == 600U && info.max_feed_gap_ms == 400U);
}

static void test_fault_frames(uint32_t *sram)
{
    app_reset_info_struct info;
    uint32_t *stack;
    uint32_t extended, i;
    uint32_t stack_errors[] = {SCB_CFSR_MSTKERR_Msk, SCB_CFSR_MUNSTKERR_Msk, SCB_CFSR_MLSPERR_Msk,
                               SCB_CFSR_STKERR_Msk, SCB_CFSR_UNSTKERR_Msk, SCB_CFSR_LSPERR_Msk};
    for (extended = 0; extended < 2U; ++extended)
    {
        cold_boot();
        app_reset_watchdog_feed(100U);
        app_reset_watchdog_feed(250U);
        host_tick = 875U;
        host_scb.CFSR = 1U << 25;
        host_scb.HFSR = 1U << 30;
        host_scb.MMFAR = 0x20004567U;
        host_scb.BFAR = 0x40000004U;
        stack = sram + 16;
        for (i = 0; i < 26U; ++i)
        {
            stack[i] = 0xAAAAAAAAU;
        }
        stack[5] = 0x08005555U;
        stack[6] = 0x08001234U;
        stack[7] = 0x21000000U;
        if (extended == 0)
        {
            app_reset_capture_fault(APP_RESET_HARDFAULT, stack, 0xFFFFFFFDU);
        }
        else
        {
            app_reset_capture_fault(APP_RESET_USAGEFAULT, stack, 0xFFFFFFEDU);
        }
        assert(host_primask == 0);
        app_reset_watchdog_feed(999U);
        app_reset_capture_stop(APP_RESET_ERROR_HANDLER, 0xBADU, 99U);
        info = reboot(1U << 26);
        assert(info.valid == 1 && info.pc == 0x08001234U && info.lr == 0x08005555U && info.xpsr == 0x21000000U);
        assert(info.sp == (uint32_t)(uintptr_t)stack);
        assert(info.uptime_ms == 875U && info.last_feed_ms == 250U && info.max_feed_gap_ms == 150U);
        assert(info.uptime_ms - info.last_feed_ms == 625U);
        assert(info.cfsr == (1U << 25) && info.hfsr == (1U << 30));
        assert(info.mmfar == 0x20004567U && info.bfar == 0x40000004U && info.task_tag == 0);
        if (extended == 0)
        {
            assert(info.kind == APP_RESET_HARDFAULT && info.exc_return == 0xFFFFFFFDU);
        }
        else
        {
            assert(info.kind == APP_RESET_USAGEFAULT && info.exc_return == 0xFFFFFFEDU);
        }
    }
    for (i = 0; i < sizeof(stack_errors) / sizeof(stack_errors[0]); ++i)
    {
        cold_boot();
        host_scb.CFSR = stack_errors[i];
        sram[5] = sram[6] = sram[7] = 0xBBBBBBBBU;
        app_reset_capture_fault(APP_RESET_BUSFAULT, sram, 0xFFFFFFF9U);
        info = reboot(1U << 26);
        assert(info.valid == 1 && info.kind == APP_RESET_BUSFAULT);
        assert(info.pc == 0 && info.lr == 0 && info.xpsr == 0 && info.cfsr == stack_errors[i]);
    }
}

static void test_checkpoint_interrupts(void)
{
    app_reset_info_struct info;
    uint32_t kind, bit, before;

    cold_boot();
    checksum_bits = 0;
    inject_at_bit = 0;
    track_checksum = 1;
    app_reset_watchdog_feed(100);
    track_checksum = 0;
    assert(checksum_bits == 17 * 32 && host_primask == 0);
    info = reboot(1U << 26);
    assert(info.valid == 1 && info.kind == APP_RESET_RUNNING && info.last_feed_ms == 100);

    /* 每个CRC位都可接收中断；期间异常提交的现场必须优先于旧检查点。 */
    for (kind = 0; kind < 2; ++kind)
    {
        for (bit = 1; bit <= 17 * 32; ++bit)
        {
            cold_boot();
            app_reset_watchdog_feed(100);
            before = commit_count;
            checksum_bits = 0;
            inject_at_bit = bit;
            if (kind == 0)
            {
                inject_kind = APP_RESET_HARDFAULT;
            }
            else
            {
                inject_kind = APP_RESET_ERROR_HANDLER;
            }
            track_checksum = 1;
            app_reset_watchdog_feed(250);
            track_checksum = 0;
            assert(checksum_bits == 17 * 32 && host_primask == 0);
            assert(commit_count == before + 1);
            app_reset_watchdog_feed(500);
            assert(commit_count == before + 1);
            info = reboot(1U << 26);
            assert(info.valid == 1 && info.kind == inject_kind);
            assert(info.uptime_ms == 975 && info.last_feed_ms == 250 && info.max_feed_gap_ms == 150);
            if (kind != 0)
            {
                assert(info.pc == 0x08005678 && info.task_tag == 4);
            }
        }
    }
    inject_at_bit = 0;
}

static void test_bad_stack(uint32_t *sram)
{
    app_reset_info_struct info;
    uintptr_t invalid[] = {0, SRAM_START - 4U, SRAM_START + 1U, SRAM_START + SRAM_BYTES - 28U,
                           SRAM_START + SRAM_BYTES, UINTPTR_MAX - 3U};
    uint32_t i;
    uint32_t *stack;
    for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        cold_boot();
        app_reset_capture_fault(APP_RESET_MEMMANAGE, (const uint32_t *)invalid[i], 0xFFFFFFFDU);
        info = reboot(1U << 26);
        assert(info.valid == 1 && info.pc == 0 && info.lr == 0 && info.xpsr == 0);
        assert(info.sp == (uint32_t)invalid[i] && info.kind == APP_RESET_MEMMANAGE);
    }
    stack = sram + SRAM_BYTES / sizeof(uint32_t) - 8U;
    stack[5] = 0x08000011U;
    stack[6] = 0x08000022U;
    stack[7] = 0x01000000U;
    cold_boot();
    app_reset_capture_fault(APP_RESET_HARDFAULT, stack, 0xFFFFFFF9U);
    info = reboot(1U << 26);
    assert(info.pc == 0x08000022U);
    /* 同一SP只有8字可读时，扩展帧不完整，不能作为有效26字帧使用。 */
    app_reset_capture_fault(APP_RESET_HARDFAULT, stack, 0xFFFFFFE9U);
    info = reboot(1U << 26);
    assert(info.pc == 0 && info.sp == (uint32_t)(uintptr_t)stack);
    stack = sram + SRAM_BYTES / sizeof(uint32_t) - 26U;
    stack[5] = 0x08000033U;
    stack[6] = 0x08000044U;
    stack[7] = 0x01000000U;
    app_reset_capture_fault(APP_RESET_HARDFAULT, stack, 0xFFFFFFE9U);
    info = reboot(1U << 26);
    assert(info.pc == 0x08000044U);
}

static void test_stops(void)
{
    app_reset_info_struct info;
    uint32_t kind;
    for (kind = APP_RESET_STACK_OVERFLOW; kind <= APP_RESET_ERROR_HANDLER; ++kind)
    {
        cold_boot();
        host_primask = 1;
        app_reset_watchdog_feed(50U);
        app_reset_watchdog_feed(150U);
        host_tick = 310U;
        app_reset_capture_stop(kind, 0x08004567U, 2U);
        assert(host_primask == 1);
        app_reset_capture_fault(APP_RESET_HARDFAULT, NULL, 0);
        app_reset_watchdog_feed(500U);
        info = reboot(1U << 26);
        assert(info.valid == 1 && info.kind == kind && info.pc == 0x08004567U && info.task_tag == 2U);
        assert(info.uptime_ms == 310U && info.last_feed_ms == 150U && info.max_feed_gap_ms == 100U);
        assert(host_primask == 1);
    }
}

static void test_lvgl_assert(void)
{
    app_reset_info_struct info;
    cold_boot();
    app_reset_watchdog_feed(100);
    host_tick = 375;
    if (setjmp(assert_halt) == 0)
    {
        stop_at_commit = 1;
        LV_ASSERT(0);
        assert(0);
    }
    assert(host_primask == 1);
    info = reboot(1U << 26);
    assert(info.valid == 1 && info.kind == APP_RESET_LVGL_ASSERT);
    assert(info.pc != 0 && info.lr == 0 && info.xpsr == 0 && info.task_tag == 0);
    assert(info.uptime_ms == 375 && info.last_feed_ms == 100 && info.max_feed_gap_ms == 0);
}

int main(void)
{
    uint32_t *sram;
#ifdef _WIN32
    sram = (uint32_t *)VirtualAlloc((void *)(uintptr_t)SRAM_START, SRAM_BYTES, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
#else
    sram = mmap((void *)(uintptr_t)SRAM_START, SRAM_BYTES, PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
#endif
    assert((uintptr_t)sram == SRAM_START);
    test_checkpoint();
    test_backup_reset_and_crc();
    test_checkpoint_interrupts();
    test_fault_frames(sram);
    test_bad_stack(sram);
    test_stops();
    test_lvgl_assert();
    assert(commit_count != 0);
#ifdef _WIN32
    assert(VirtualFree(sram, 0, MEM_RELEASE) != 0);
#else
    assert(munmap(sram, SRAM_BYTES) == 0);
#endif
    puts("reset: interruptible checkpoint CRC, fault priority, retained/corrupt/partial backup, RTC reset, wrap, core/FPU frames, bad SP, stop, LVGL assert PASS");
    return 0;
}
