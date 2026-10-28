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
Host_Peripheral_Type host_tim2, host_tim3, host_rcc, host_spi, host_dma, host_adc, host_gpio;
#define HOST_COUNTS_PER_TICK (configCPU_CLOCK_HZ / configTICK_RATE_HZ)
static uint32_t rtc_read_cycles;
static uint64_t rtc_cpu_fraction;
volatile uint32_t uwTick;
static uint32_t kernel_tick, rtc_count, sleep_rtc_counts, step_limit;
static uint32_t requested_load, sleeps, read_calls;
static uint8_t irq_masked, abort_sleep, rtc_ready, rtc_fail_read, normal_tick, deadline_tick, pending_on_read;
static uint8_t screen_off, wakeup_armed, wakeup_failed, sync_failed;
static uint32_t stops, wakeup_counts, restore_counts;

uint8_t app_task_is_screen_off(void)
{
    return screen_off;
}

uint8_t watch_rtc_start_wakeup(uint32_t subsecond_counts)
{
    assert(irq_masked && !wakeup_armed);
    if (wakeup_failed != 0)
    {
        return 1;
    }
    wakeup_counts = subsecond_counts;
    wakeup_armed = 1;
    return 0;
}

void watch_rtc_stop_wakeup(void)
{
    assert(irq_masked && wakeup_armed);
    wakeup_armed = 0;
}

uint8_t watch_rtc_sync_after_stop(void)
{
    assert(irq_masked && (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL);
    if (sync_failed != 0)
    {
        rtc_ready = 0;
        return 1;
    }
    rtc_count += restore_counts;
    return 0;
}

void host_modify_reg(uint32_t *reg, uint32_t mask, uint32_t value)
{
    assert(irq_masked && reg == &RCC->CFGR && (RCC->CR & RCC_CR_PLLON) != 0);
    *reg = (*reg & ~mask) | value | RCC_CFGR_SWS_PLL;
}

void Error_Handler(void)
{
    assert(0 && "clock must recover before interrupts are unmasked");
}

void HAL_PWR_EnterSTOPMode(uint32_t regulator, uint8_t entry)
{
    assert(irq_masked && wakeup_armed && screen_off);
    assert(regulator == PWR_LOWPOWERREGULATOR_ON && entry == PWR_STOPENTRY_WFI);
    assert((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0);
    assert((BLE_EN_GPIO_Port->ODR & BLE_EN_Pin) == 0 && (RCC->APB2ENR & RCC_APB2ENR_USART1EN) == 0);
    assert(TIM3->CCR3 == 0 && (SPI1->SR & SPI_SR_BSY) == 0 && (DMA2_Stream2->CR & DMA_SxCR_EN) == 0);
    ++stops;
    rtc_count += sleep_rtc_counts;
    RCC->CFGR = 0; /* STOP唤醒先使用HSI，硬件随后允许PLL重新锁定。 */
    RCC->CR = RCC_CR_PLLRDY;
}

void host_disable_irq(void)
{
    assert(!irq_masked);
    irq_masked = 1;
}

void host_enable_irq(void)
{
    assert(irq_masked && kernel_tick == uwTick);
    assert(!wakeup_armed && (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL);
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
    if (!rtc_ready || (rtc_fail_read != 0 && read_calls >= rtc_fail_read))
    {
        return UINT32_MAX;
    }
    result = rtc_count & (WATCH_RTC_SUBSECOND_HZ - 1U);
    /* RTC读取返回前仍在运行CPU；该开销不能因SysTick暂停而丢掉。 */
    if (read_calls >= 2U)
    {
        DWT->CYCCNT += rtc_read_cycles;
        TIM2->CNT += (uint32_t)((uint64_t)rtc_read_cycles * 10000 / configCPU_CLOCK_HZ);
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
    SysTick->LOAD = (HOST_COUNTS_PER_TICK - 1U);
    SysTick->VAL = HOST_COUNTS_PER_TICK - phase_cycles;
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
    screen_off = wakeup_armed = wakeup_failed = sync_failed = 0;
    stops = wakeup_counts = restore_counts = 0;
    host_tim2 = host_tim3 = host_rcc = host_spi = host_dma = host_adc = host_gpio = (Host_Peripheral_Type){0};
    RCC->CFGR = RCC_CFGR_SWS_PLL | RCC_CFGR_SW_PLL;
    RCC->CR = RCC_CR_PLLRDY | RCC_CR_PLLON;
    abort_sleep = rtc_fail_read = normal_tick = deadline_tick = pending_on_read = 0;
}

int main(void)
{
    uint32_t i, previous_tick, elapsed_total;
    reset(0);
    sleep_rtc_counts = 328U; /* UART在约10ms时提前唤醒原定20ms休眠。 */
    app_tickless_sleep(20U);
    assert(sleeps == 1U && kernel_tick == 10U && SysTick->LOAD == (HOST_COUNTS_PER_TICK - 1U) && !irq_masked);
    assert(requested_load == 20U * HOST_COUNTS_PER_TICK - 1U);

    reset((HOST_COUNTS_PER_TICK / 2U));
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
    assert(!sleeps && !kernel_tick && SysTick->VAL == HOST_COUNTS_PER_TICK);

    reset(0);
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    app_tickless_sleep(20U);
    assert(!sleeps && kernel_tick == 1U);

    reset(0);
    pending_on_read = 1;
    app_tickless_sleep(20U);
    assert(!sleeps && kernel_tick == 1U && SysTick->LOAD == (HOST_COUNTS_PER_TICK - 1U));

    reset(0);
    rtc_ready = 0;
    normal_tick = 1;
    app_tickless_sleep(20U);
    assert(sleeps == 1U && kernel_tick == 1U && requested_load == (HOST_COUNTS_PER_TICK - 1U));

    reset(0);
    rtc_fail_read = 3;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 10U);

    reset(0);
    rtc_fail_read = 3;
    sleep_rtc_counts = 656U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 20U);

    reset(0);
    sleep_rtc_counts = 3000U;
    step_limit = 100U;
    app_tickless_sleep(UINT32_MAX);
    assert(kernel_tick == 91U && requested_load == 100U * HOST_COUNTS_PER_TICK - 1U);

    reset(0);
    kernel_tick = uwTick = UINT32_MAX - 5U;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(kernel_tick == 4U);

    reset(0);
    sleep_rtc_counts = 330U;
    rtc_read_cycles = HOST_COUNTS_PER_TICK;
    DWT->CYCCNT = UINT32_MAX - (HOST_COUNTS_PER_TICK / 2U);
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

    reset(0);
    screen_off = 1;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 10 && TIM2->CNT == 100);
    assert(wakeup_counts == 19U * WATCH_RTC_SUBSECOND_HZ / 1000U);

    reset(0);
    screen_off = 1;
    sleep_rtc_counts = 330U;
    rtc_read_cycles = HOST_COUNTS_PER_TICK;
    app_tickless_sleep(20U);
    assert(stops == 1 && kernel_tick == 12 && TIM2->CNT == 120); /* 保留结束RTC读取期间的CPU运行时间。 */

    reset(0);
    screen_off = 1;
    rtc_count = 32760U;
    sleep_rtc_counts = 328U;
    restore_counts = 66U; /* PLL恢复/RTC影子同步也占用真实时间，纳入补时。 */
    TIM2->CNT = UINT32_MAX - 20U;
    app_tickless_sleep(20U);
    assert(stops == 1 && kernel_tick == 12 && TIM2->CNT == 99U);

    reset(0);
    screen_off = 1;
    sleep_rtc_counts = 4000U;
    app_tickless_sleep(20U);
    assert(stops == 1 && kernel_tick == 20); /* 延迟唤醒也不能step越过最近解阻塞点。 */

    for (i = 0; i < 7U; ++i)
    {
        reset(0);
        screen_off = 1;
        sleep_rtc_counts = 33U;
        switch (i)
        {
            case 0:
                BLE_EN_GPIO_Port->ODR = BLE_EN_Pin;
                break;
            case 1:
                RCC->APB2ENR = RCC_APB2ENR_USART1EN;
                break;
            case 2:
                TIM3->CCR3 = 1;
                break;
            case 3:
                SPI1->SR = SPI_SR_BSY;
                break;
            case 4:
                DMA2_Stream2->CR = DMA_SxCR_EN;
                break;
            case 5:
                ADC1->CR2 = ADC_CR2_ADON;
                break;
            default:
                wakeup_failed = 1;
                break;
        }
        app_tickless_sleep(20U);
        assert(stops == 0 && sleeps == 1 && kernel_tick == 1);
    }

    reset(0);
    screen_off = 1;
    pending_on_read = 1;
    app_tickless_sleep(20U);
    assert(!stops && !sleeps && !wakeup_armed && kernel_tick == 1);

    reset(0);
    screen_off = 1;
    rtc_fail_read = 2;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(stops == 0 && sleeps == 1 && !wakeup_armed && kernel_tick == 10);

    reset(0);
    screen_off = 1;
    sleep_rtc_counts = 33U;
    app_tickless_sleep(4U);
    assert(stops == 0 && sleeps == 1 && kernel_tick == 1);

    reset(0);
    screen_off = 1;
    sync_failed = 1;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(stops == 1 && !wakeup_armed && !rtc_ready);
    normal_tick = 1;
    app_tickless_sleep(20U);
    assert(stops == 1 && sleeps == 1 && kernel_tick == 1);
    puts("PASS: STOP wake/clock-before-ISR, RTC and diagnostic time, deadline clamp, BLE/PWM/SPI/DMA/ADC veto, WUT and shadow-sync fallback.");
    return 0;
}
