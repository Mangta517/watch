/* 执行真实休眠适配层；只替换SysTick/RTC寄存器、WFI及FreeRTOS内核接口。 */
#include "main.h"
#include "task.h"
#include "watch_rtc.h"
#include "app_tickless.h"
#include <assert.h>
#include <stdio.h>

SysTick_Type host_systick;
SCB_Type host_scb;
Host_Cycle_Type host_cycles;
static uint32_t rtc_read_cycles;
static uint64_t rtc_cpu_fraction;
volatile uint32_t uwTick;
static uint32_t kernel_tick, rtc_count, sleep_rtc_counts, step_limit;
static uint32_t requested_load, sleeps, read_calls;
static uint8_t irq_masked, abort_sleep, rtc_ready, lose_lse, normal_tick, deadline_tick, pending_on_read;

void host_disable_irq(void)
{
    assert(!irq_masked);
    irq_masked = 1;
}

void host_enable_irq(void)
{
    assert(irq_masked && kernel_tick == uwTick);
    irq_masked = 0;
    /* 模拟pending的普通tick；UART/DMA回调只能在补时完成后观察时间。 */
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
    {
        ++kernel_tick;
        ++uwTick;
        SCB->ICSR = 0;
    }
}

void host_barrier(void)
{
    /* 写VAL清零后，下个核心时钟装载LOAD。 */
    if ((SysTick->CTRL & 1U) != 0 && SysTick->VAL == 0)
    {
        SysTick->VAL = SysTick->LOAD;
    }
}

uint32_t watch_rtc_get_subsecond(void)
{
    uint32_t result;
    assert(irq_masked);
    ++read_calls;
    if (pending_on_read != 0 && read_calls == 1U)
    {
        SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    }
    if (!rtc_ready || (lose_lse != 0 && read_calls >= 3U))
    {
        return UINT32_MAX;
    }
    result = rtc_count & (WATCH_RTC_SUBSECOND_HZ - 1U);
    /* RTC读取返回前仍在运行CPU；该开销不能因SysTick暂停而丢掉。 */
    if (read_calls >= 2U)
    {
        DWT->CYCCNT += rtc_read_cycles;
        rtc_cpu_fraction += (uint64_t)rtc_read_cycles * WATCH_RTC_SUBSECOND_HZ;
        rtc_count += (uint32_t)(rtc_cpu_fraction / configCPU_CLOCK_HZ);
        rtc_cpu_fraction %= configCPU_CLOCK_HZ;
    }
    return result;
}

eSleepModeStatus eTaskConfirmSleepModeStatus(void)
{
    assert(irq_masked);
    if (abort_sleep != 0)
    {
        return eAbortSleep;
    }
    return eStandardSleep;
}

void vTaskStepTick(TickType_t ticks)
{
    assert(irq_masked && ticks < step_limit);
    host_barrier(); /* 硬件在函数执行期间已装载补齐当前tick的短LOAD。 */
    kernel_tick += ticks;
    assert(kernel_tick == uwTick);
}

void host_sleep(void)
{
    uint32_t elapsed_cycles;
    assert(irq_masked && (SysTick->CTRL & 7U) == 7U);
    assert((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) == 0);
    ++sleeps;
    requested_load = SysTick->LOAD;
    assert(requested_load <= 0xFFFFFFU);
    if (normal_tick != 0)
    {
        SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
        return;
    }
    rtc_count += sleep_rtc_counts;
    elapsed_cycles = (uint32_t)((uint64_t)sleep_rtc_counts * configCPU_CLOCK_HZ / WATCH_RTC_SUBSECOND_HZ);
    if (elapsed_cycles >= requested_load + 1U)
    {
        SysTick->VAL = 2U * (requested_load + 1U) - elapsed_cycles - 1U;
        SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    }
    else
    {
        SysTick->VAL = requested_load - elapsed_cycles;
    }
    if (deadline_tick != 0)
    {
        assert(elapsed_cycles >= requested_load + 1U);
    }
}

static void reset(uint32_t phase_cycles)
{
    assert(!irq_masked);
    SysTick->CTRL = 7U;
    SysTick->LOAD = 83999U;
    SysTick->VAL = 84000U - phase_cycles;
    SCB->ICSR = 0;
    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
    kernel_tick = 0;
    uwTick = 0;
    read_calls = 0;
    sleeps = 0;
    step_limit = 20U;
    rtc_count = 0;
    rtc_ready = 1;
    rtc_read_cycles = 0;
    rtc_cpu_fraction = 0;
    abort_sleep = lose_lse = normal_tick = deadline_tick = pending_on_read = 0;
}

int main(void)
{
    uint32_t i, previous_tick, elapsed_total;
    reset(0);
    sleep_rtc_counts = 328U; /* UART在约10ms时提前唤醒原定20ms休眠。 */
    app_tickless_sleep(20U);
    assert(sleeps == 1U && kernel_tick == 10U && SysTick->LOAD == 83999U && !irq_masked);
    assert(requested_load == 20U * 84000U - 1U);

    reset(42000U);
    sleep_rtc_counts = 16U; /* 0.5ms原tick相位 + 0.488ms睡眠，不凭空补整tick。 */
    app_tickless_sleep(20U);
    assert(kernel_tick == 0U);

    reset(0);
    rtc_count = 32760U;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 10U); /* 秒/午夜回绕不会回退单调时间。 */

    reset(0);
    sleep_rtc_counts = 623U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 19U && SCB->ICSR == SCB_ICSR_PENDSTCLR_Msk);

    reset(0);
    sleep_rtc_counts = 656U;
    deadline_tick = 1;
    app_tickless_sleep(20U);
    assert(kernel_tick == 20U); /* 最后到期tick交ISR，step不跨过任务解阻塞点。 */

    reset(0);
    abort_sleep = 1;
    app_tickless_sleep(20U);
    assert(!sleeps && !kernel_tick && SysTick->VAL == 84000U);

    reset(0);
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    app_tickless_sleep(20U);
    assert(!sleeps && kernel_tick == 1U);

    reset(0);
    pending_on_read = 1;
    app_tickless_sleep(20U);
    assert(!sleeps && kernel_tick == 1U && SysTick->LOAD == 83999U);

    reset(0);
    rtc_ready = 0;
    normal_tick = 1;
    app_tickless_sleep(20U);
    assert(sleeps == 1U && kernel_tick == 1U && requested_load == 83999U);

    reset(0);
    lose_lse = 1;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 10U);

    reset(0);
    lose_lse = 1;
    sleep_rtc_counts = 656U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 20U);

    reset(0);
    sleep_rtc_counts = 3000U;
    step_limit = 100U;
    app_tickless_sleep(UINT32_MAX);
    assert(kernel_tick == 91U && requested_load == 100U * 84000U - 1U);

    reset(0);
    kernel_tick = uwTick = UINT32_MAX - 5U;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 4U);

    reset(0);
    sleep_rtc_counts = 330U;
    rtc_read_cycles = 84000U;
    DWT->CYCCNT = UINT32_MAX - 42000U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 12U); /* 约10ms睡眠+两次RTC读取各1ms，包含DWT回绕。 */

    reset(0);
    elapsed_total = 0;
    sleep_rtc_counts = 33U;
    for (i = 0; i < 10000U; ++i)
    {
        previous_tick = kernel_tick;
        app_tickless_sleep(20U);
        assert(kernel_tick >= previous_tick);
        elapsed_total += sleep_rtc_counts;
        /* 连续短睡眠保留不足1ms的相位及RTC换算余数。 */
        assert(kernel_tick >= elapsed_total * 1000U / 32768U);
        assert(kernel_tick <= elapsed_total * 1000U / 32768U + 1U);
    }
    {
        const uint32_t periods[] = {5U, 10U, 20U};
        uint32_t deadlines[] = {5U, 13U, 29U};
        uint32_t runs[] = {0, 0, 0};
        uint32_t now = 0, next, expected, task;
        /* 模拟三个不同相位的阻塞任务；内核提供最早截止时间，休眠不能跨过任何一项。 */
        while (now < 100U)
        {
            next = deadlines[0];
            for (task = 1; task < 3U; ++task)
            {
                if (deadlines[task] < next)
                {
                    next = deadlines[task];
                }
            }
            expected = next - now;
            reset(0);
            kernel_tick = uwTick = now;
            step_limit = expected;
            sleep_rtc_counts = (expected * 32768U + 999U) / 1000U;
            if (expected < 2U)
            {
                normal_tick = 1;
            }
            app_tickless_sleep(expected);
            assert(kernel_tick == next && uwTick == next);
            now = next;
            for (task = 0; task < 3U; ++task)
            {
                assert(deadlines[task] >= now);
                if (deadlines[task] == now)
                {
                    ++runs[task];
                    deadlines[task] += periods[task];
                }
            }
        }
        assert(runs[0] == 20U && runs[1] == 9U && runs[2] == 4U);
    }
    puts("PASS: RTC early/deadline wake, partial ticks, fractional accumulation, second/midnight and tick wrap, pending/abort, LSE fallback and 100ms limit.");
    puts("PASS: mixed 5/10/20ms blocked-task deadlines retain their phases; the sleep adapter never steps over the earliest unblock time.");
    puts("PASS: CPU time during RTC reads is compensated, including cycle-counter wrap.");
    return 0;
}
