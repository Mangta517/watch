#include "app_tickless.h"
#include "watch_rtc.h"
#include "FreeRTOS.h"
#include "task.h"

#if configTICK_RATE_HZ != 1000
#error Tickless HAL compensation requires a 1ms RTOS tick.
#endif

/* Sleep保留SysTick时钟，用一次长计数唤醒；RTC负责补时。最多100ms，不越过喂狗周期。 */
#define APP_TICKLESS_MAX_TICKS 100U

static uint32_t s_rtc_fraction;
extern __IO uint32_t uwTick;

void app_tickless_sleep(uint32_t expected_ticks)
{
    const uint32_t counts_per_tick = configCPU_CLOCK_HZ / configTICK_RATE_HZ;
    uint32_t control, remaining, rtc_start, rtc_end, elapsed_counts, phase;
    uint32_t complete_ticks, pend_tick = 0;
    uint32_t stopped_cycle, resumed_cycle, rtc_start_cycle, rtc_end_cycle;
    uint64_t scaled_counts;

    __disable_irq();
    __DSB();
    __ISB();
    if (eTaskConfirmSleepModeStatus() == eAbortSleep || (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
    {
        __enable_irq();
        return;
    }
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; /* 与原HAL Sleep入口一致，保留外设时钟。 */
    rtc_start = watch_rtc_get_subsecond();
    if (rtc_start == UINT32_MAX || expected_ticks < 2U)
    {
        /* LSE启动/故障时保留1ms心跳，HAL超时和启动看门狗仍正常推进。 */
        __DSB();
        __WFI();
        __ISB();
        __enable_irq();
        return;
    }
    if (expected_ticks > APP_TICKLESS_MAX_TICKS)
    {
        expected_ticks = APP_TICKLESS_MAX_TICKS;
    }

    /* DWT只测量RTC区间之外的运行开销，不依赖它在Sleep中的计数行为。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    control = SysTick->CTRL;
    stopped_cycle = DWT->CYCCNT;
    SysTick->CTRL = control & ~SysTick_CTRL_ENABLE_Msk;
    remaining = SysTick->VAL;
    /* 停表前刚到期的普通tick由原ISR处理，不能清掉或再次补偿。 */
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
    {
        SysTick->CTRL = control;
        __enable_irq();
        return;
    }
    if (remaining == 0U)
    {
        remaining = SysTick->LOAD + 1U;
    }
    phase = counts_per_tick - remaining;
    rtc_start_cycle = DWT->CYCCNT;
    rtc_start = watch_rtc_get_subsecond();

    /* 在最近任务截止时间唤醒；到期的最后一个tick交ISR处理，step不越过解阻塞点。 */
    SysTick->LOAD = remaining + counts_per_tick * (expected_ticks - 1U) - 1U;
    SysTick->VAL = 0U;
    resumed_cycle = DWT->CYCCNT;
    SysTick->CTRL = control;
    __DSB();
    __WFI();
    __ISB();
    SysTick->CTRL = control & ~SysTick_CTRL_ENABLE_Msk;
    rtc_end_cycle = DWT->CYCCNT;
    rtc_end = watch_rtc_get_subsecond();

    if (rtc_start != UINT32_MAX && rtc_end != UINT32_MAX)
    {
        /* 单次小于1秒，模减同时覆盖跨秒/午夜；日历校时只能在任务恢复后发生。 */
        scaled_counts = (uint64_t)((rtc_end - rtc_start) & (WATCH_RTC_SUBSECOND_HZ - 1U));
        scaled_counts = scaled_counts * configCPU_CLOCK_HZ + s_rtc_fraction;
        elapsed_counts = (uint32_t)(scaled_counts / WATCH_RTC_SUBSECOND_HZ);
        s_rtc_fraction = (uint32_t)(scaled_counts % WATCH_RTC_SUBSECOND_HZ);
    }
    else
    {
        /* Sleep中计数器仍运行；LSE运行期故障时用本次长SysTick保住经过时间。 */
        elapsed_counts = SysTick->LOAD - SysTick->VAL;
        if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
        {
            elapsed_counts += SysTick->LOAD + 1U;
        }
        rtc_start_cycle = resumed_cycle;
    }
    /* 两次RTC调用前取同一位置的周期戳，读寄存器的固定延迟相消。
     * 补上停表到起始采样、结束采样到恢复计算的实际CPU开销，周期差允许回绕。 */
    __DSB();
    elapsed_counts += rtc_start_cycle - stopped_cycle;
    elapsed_counts += DWT->CYCCNT - rtc_end_cycle;
    elapsed_counts += phase;
    complete_ticks = elapsed_counts / counts_per_tick;
    remaining = counts_per_tick - elapsed_counts % counts_per_tick;
    if (complete_ticks >= expected_ticks)
    {
        complete_ticks = expected_ticks - 1U;
        pend_tick = 1U;
    }

    /* 长计数产生的pending不是普通1ms tick；先清除，再按实测值一次补齐。 */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    SysTick->LOAD = remaining - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = control;
    uwTick += complete_ticks;
    vTaskStepTick((TickType_t)complete_ticks);
    SysTick->LOAD = counts_per_tick - 1U;
    if (pend_tick != 0U)
    {
        SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    }
    /* UART/DMA/EXTI中断第一次执行时，HAL和RTOS时间已经补齐。 */
    __DSB();
    __ISB();
    __enable_irq();
}
