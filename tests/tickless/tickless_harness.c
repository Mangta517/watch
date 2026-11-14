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
Host_Peripheral_Type host_tim2, host_tim3, host_rcc, host_spi, host_dma, host_adc, host_gpio, host_exti;
#define HOST_COUNTS_PER_TICK (configCPU_CLOCK_HZ / configTICK_RATE_HZ)
static uint32_t rtc_read_cycles;
static uint64_t rtc_cpu_fraction;
volatile uint32_t uwTick;
static uint32_t kernel_tick, rtc_count, sleep_rtc_counts, step_limit;
static uint32_t requested_load, sleeps, read_calls;
static uint8_t irq_masked, abort_sleep, rtc_ready, rtc_fail_read, normal_tick, deadline_tick, pending_on_read;
static uint8_t screen_off, wakeup_armed, wakeup_failed, sync_failed;
static uint32_t stops, wakeup_counts, restore_counts;
static uint32_t stop_exti_imr; /* 记录进入STOP瞬间的EXTI->IMR，验证只屏蔽PB12。 */
static uint8_t stop_exti_seen;
static uint8_t stop_exti_early; /* PB12未屏蔽时，STOP期间新到的DATA_RDY让WFI立即返回。 */
static uint8_t stop_mpu_event; /* STOP期间器件新产生一次DATA_RDY事件，置EXTI->PR。 */

/* sensor进度授权状态：由测试显式控制，不硬编码为真。 */
static uint8_t wd_auth; /* 1 表示 app_task_watchdog_stop_allowed 返回授权。 */
/* 暂停/恢复状态，验证配对与PLL前恢复。 */
static uint8_t wd_paused;       /* 1 表示当前处于TPS3823暂停状态。 */
static uint8_t wd_in_stop;      /* HAL_PWR_EnterSTOPMode置1，tps3823_stop_resume清零。 */
static uint32_t wd_pause_count; /* 本轮累计暂停次数。 */
static uint32_t wd_resume_count; /* 本轮累计恢复次数。 */
static uint32_t wd_resume_rtc; /* 每次硬件恢复瞬间的一分钟内计数快照，用于度量硬件侧间隔。 */

/* 每次构建从驱动源码提取定义；只模拟寄存器，不维护查询函数副本。 */
#include "stop_queries.inc"

uint8_t app_task_is_screen_off(void)
{
    return screen_off;
}

uint8_t app_task_watchdog_stop_allowed(void)
{
    assert(irq_masked);
    return wd_auth;
}

void tps3823_stop_pause(void)
{
    assert(irq_masked && !wd_paused);
    wd_paused = 1;
    ++wd_pause_count;
}

void tps3823_stop_resume(void)
{
    assert(irq_masked && wd_paused);
    /* STOP路径：resume必须在PLL恢复之前，此时CFGR SWS仍为HSI（0），不是PLL。 */
    if (wd_in_stop != 0)
    {
        assert((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
        wd_in_stop = 0;
    }
    wd_paused = 0;
    ++wd_resume_count;
    /* 记录恢复发生的硬件时基，供用例度量"最后一次恢复之后"的真实间隔。 */
    wd_resume_rtc = rtc_count;
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
    stop_exti_imr = EXTI->IMR;
    stop_exti_seen = 1;
    ++stops;
    /* STOP路径置标志，供tps3823_stop_resume断言PLL尚未恢复。 */
    wd_in_stop = wd_paused;
    if (stop_mpu_event != 0)
    {
        EXTI->PR |= 1U << 12; /* 进入STOP之后才出现的DATA_RDY事件。 */
    }
    if ((EXTI->IMR & (1U << 12)) != 0 && (EXTI->PR & (1U << 12)) != 0)
    {
        /* PB12未屏蔽且请求已置位：EXTI仍可唤醒，WFI立即返回，只经过一小段CPU时间。 */
        stop_exti_early = 1;
        sleep_rtc_counts = 30;
    }
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
    /* 返回前必须已恢复TPS3823超时检测，不能有残留的暂停。 */
    assert(!wd_paused);
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

uint32_t watch_rtc_get_counter(void)
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
    result = rtc_count % WATCH_RTC_COUNTER_MOD;
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
    stop_exti_imr = 0;
    stop_exti_seen = 0;
    stop_exti_early = 0;
    stop_mpu_event = 0;
    wd_auth = 0;
    wd_paused = 0;
    wd_in_stop = 0;
    wd_pause_count = 0;
    wd_resume_count = 0;
    wd_resume_rtc = 0;
    host_tim2 = host_tim3 = host_rcc = host_spi = host_dma = host_adc = host_gpio = host_exti = (Host_Peripheral_Type){0};
    RCC->CFGR = RCC_CFGR_SWS_PLL | RCC_CFGR_SW_PLL;
    RCC->CR = RCC_CR_PLLRDY | RCC_CR_PLLON;
    abort_sleep = rtc_fail_read = normal_tick = deadline_tick = pending_on_read = 0;
}

int main(void)
{
    uint32_t i, previous_tick, elapsed_total, authorized_resume_rtc;
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
    sleep_rtc_counts = 6000;
    step_limit = 200;
    app_tickless_sleep(UINT32_MAX);
    /* 无授权时UINT32_MAX请求依次受1000/400ms上限，Sleep路径再受24位SysTick约束至262ms；
     * RTC实测约183ms，内核据RTC差值补时。 */
    assert(kernel_tick == 183 && requested_load == 262 * HOST_COUNTS_PER_TICK - 1);
    assert(wd_pause_count == 0); /* 无授权，不暂停。 */

    reset(0);
    rtc_count = 32000;
    sleep_rtc_counts = 6554;
    step_limit = 200;
    deadline_tick = 1;
    app_tickless_sleep(200);
    assert(kernel_tick == 200 && requested_load == 200 * HOST_COUNTS_PER_TICK - 1);

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
    puts("PASS: RTC early/deadline wake, partial ticks, fractional accumulation, second/midnight and tick wrap, pending/abort, LSE fallback and 200ms limit.");
    puts("PASS: mixed 5/10/20ms blocked-task deadlines retain their phases; the sleep adapter never steps over the earliest unblock time.");
    puts("PASS: CPU time during RTC reads is compensated, including cycle-counter wrap.");

    /* 有sensor进度授权但BLE保持连接：STOP资格被否决，长等待仍走Sleep；
     * 24位LOAD上限依旧把单次等待收紧到262ms，暂停与恢复必须成对包住这次实际WFI。
     * 该用例不依赖STOP是否编译进来，因此放在配置分支之前。 */
    reset(0);
    wd_auth = 1;
    screen_off = 1;
    BLE_EN_GPIO_Port->ODR = BLE_EN_Pin;
    sleep_rtc_counts = 6000;
    step_limit = 400;
    app_tickless_sleep(UINT32_MAX);
    assert(stops == 0 && sleeps == 1 && kernel_tick == 183);
    assert(requested_load == 262 * HOST_COUNTS_PER_TICK - 1);
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused);

    /* RTC参考不可用时只做1ms心跳WFI：有授权同样必须在WFI之前暂停、返回之后立即恢复。 */
    reset(0);
    wd_auth = 1;
    rtc_ready = 0;
    normal_tick = 1;
    app_tickless_sleep(20U);
    assert(sleeps == 1U && kernel_tick == 1U && requested_load == (HOST_COUNTS_PER_TICK - 1U));
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused);

    /* 等待不足2拍也走1ms心跳WFI：RTC计数有效但不补时，配对仍须成立且不推进RTC快照。 */
    reset(0);
    wd_auth = 1;
    normal_tick = 1;
    sleep_rtc_counts = 33U;
    app_tickless_sleep(1U);
    assert(sleeps == 1U && kernel_tick == 1U && rtc_count == 0U);
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused);

    /* 内核要求放弃本轮睡眠、或入口已有到期tick：授权在两次检查之后才查询，
     * 因此这两条直接返回的路径即便拿到授权也必须完全没有暂停动作。 */
    reset(0);
    wd_auth = 1;
    abort_sleep = 1;
    app_tickless_sleep(20U);
    assert(!sleeps && !kernel_tick);
    assert(wd_pause_count == 0 && wd_resume_count == 0);

    reset(0);
    wd_auth = 1;
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    app_tickless_sleep(20U);
    assert(!sleeps && kernel_tick == 1U);
    assert(wd_pause_count == 0 && wd_resume_count == 0);

    puts("PASS: authorization pairs pause/resume around every real idle wait (BLE-vetoed Sleep at the 262ms cap, "
         "RTC-unavailable and sub-two-tick 1ms WFI), while abort/pending entries return before any pause.");

    if (APP_TICKLESS_ENABLE_STOP == 0)
    {
        reset(0);
        screen_off = 1;
        sleep_rtc_counts = 328;
        app_tickless_sleep(20);
        assert(stops == 0 && sleeps == 1 && wakeup_counts == 0 && kernel_tick == 10);
        assert((RCC->CR & RCC_CR_PLLON) != 0 && (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL);
        puts("PASS: screen-off with all peripherals idle still uses Sleep and retains the 64MHz PLL configuration.");
        return 0;
    }

    reset(0);
    screen_off = 1;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 10 && TIM2->CNT == 100);
    assert(wakeup_counts == 19U * WATCH_RTC_SUBSECOND_HZ / 1000U);

    reset(0);
    screen_off = 1;
    rtc_count = 32000;
    sleep_rtc_counts = 6000;
    step_limit = 200;
    app_tickless_sleep(UINT32_MAX);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 183 && TIM2->CNT == 1831);
    /* STOP用RTC唤醒可达400ms上限；预留1ms给同步/PLL恢复，故装载399ms的亚秒计数。 */
    assert(wakeup_counts == 399 * WATCH_RTC_SUBSECOND_HZ / 1000);

    reset(0);
    screen_off = 1;
    sleep_rtc_counts = 1213; /* PA4等外部中断仍可在200ms期限之前唤醒。 */
    step_limit = 200;
    app_tickless_sleep(200);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 37 && !wakeup_armed);

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

    /* 长等待在蓝牙保连或RTC唤醒启动失败时回退Sleep，不能溢出24位LOAD。 */
    for (i = 0; i < 2; ++i)
    {
        reset(0);
        screen_off = 1;
        host_exti.IMR = (1U << 4) | (1U << 12);
        sleep_rtc_counts = 6000;
        step_limit = 400;
        if (i == 0)
        {
            BLE_EN_GPIO_Port->ODR = BLE_EN_Pin;
        }
        else
        {
            wakeup_failed = 1;
        }
        app_tickless_sleep(400);
        assert(stops == 0 && sleeps == 1 && kernel_tick == 183);
        assert(requested_load == 262 * HOST_COUNTS_PER_TICK - 1);
        assert(host_exti.IMR == ((1U << 4) | (1U << 12)));
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
    host_exti.IMR = (1U << 4) | (1U << 12);
    stop_mpu_event = 1;
    sleep_rtc_counts = 328U;
    app_tickless_sleep(20U);
    assert(stops == 1 && !wakeup_armed && !rtc_ready);
    assert(host_exti.IMR == ((1U << 4) | (1U << 12)) && host_exti.PR == (1U << 12));
    normal_tick = 1;
    app_tickless_sleep(20U);
    assert(stops == 1 && sleeps == 1 && kernel_tick == 1);

    /* 静止慢待机400ms STOP：进入STOP时屏蔽PB12、保留PA4及其他线，退出后精确恢复原mask。 */
    reset(0);
    screen_off = 1;
    host_exti.IMR = (1U << 4) | (1U << 12) | (1U << 0) | 0x80000000U;
    sleep_rtc_counts = 13107U; /* 约400ms，RTC按装载期限唤醒。 */
    step_limit = 401;
    app_tickless_sleep(400U);
    assert(stops == 1 && stop_exti_seen != 0 && !wakeup_armed);
    assert((stop_exti_imr & (1U << 12)) == 0);                              /* STOP期间PB12被屏蔽。 */
    assert((stop_exti_imr & (1U << 4)) != 0);                              /* PA4按键仍可唤醒。 */
    assert((stop_exti_imr & (1U << 0)) != 0 && (stop_exti_imr & 0x80000000U) != 0); /* 其他线不受影响。 */
    assert(host_exti.IMR == ((1U << 4) | (1U << 12) | (1U << 0) | 0x80000000U)); /* 退出后精确恢复。 */

    /* 所有真实STOP都屏蔽PB12：5/10/100/200/400ms下STOP期间新到一次DATA_RDY事件，
     * STOP仍须等到RTC期限，不会被该事件立即唤醒。 */
    {
        const uint32_t periods[] = {5, 10, 100, 200, 400};
        /* RTC按(期限-1ms)装载亚秒计数，提前的1ms留给同步与PLL恢复。 */
        const uint32_t armed_counts[] = {131, 294, 3244, 6520, 13074};
        for (i = 0; i < 5; ++i)
        {
            reset(0);
            screen_off = 1;
            host_exti.IMR = (1U << 4) | (1U << 12) | (1U << 0) | 0x80000000U;
            stop_mpu_event = 1; /* 进入STOP之后器件新产生一次DATA_RDY事件。 */
            sleep_rtc_counts = armed_counts[i];
            step_limit = 401;
            app_tickless_sleep(periods[i]);
            assert(stops == 1 && stop_exti_seen != 0 && stop_exti_early == 0); /* 新事件未能结束STOP。 */
            assert(wakeup_counts == armed_counts[i]);
            /* 屏蔽前已置位的NVIC pending不受影响；此处验证STOP期间的新事件被挡住。 */
            assert(kernel_tick + 2 >= periods[i] && kernel_tick <= periods[i]);
            assert((stop_exti_imr & (1U << 12)) == 0);                              /* PB12在所有STOP内屏蔽。 */
            assert((stop_exti_imr & (1U << 4)) != 0);                              /* PA4按键仍可唤醒。 */
            assert((stop_exti_imr & (1U << 0)) != 0 && (stop_exti_imr & 0x80000000U) != 0); /* 其他线不受影响。 */
            assert(host_exti.IMR == ((1U << 4) | (1U << 12) | (1U << 0) | 0x80000000U)); /* 退出后精确恢复。 */
            assert(host_exti.PR == (1U << 12)); /* 恢复mask不清掉STOP期间锁存的EXTI请求。 */
        }
    }

    /* 屏蔽期间PA4等外部中断仍可提前唤醒，且提前退出同样精确恢复原mask。 */
    reset(0);
    screen_off = 1;
    host_exti.IMR = (1U << 4) | (1U << 12) | (1U << 1);
    sleep_rtc_counts = 6554U; /* 约200ms，早于400ms期限。 */
    step_limit = 401;
    app_tickless_sleep(400U);
    assert(stops == 1 && !wakeup_armed);
    assert((stop_exti_imr & (1U << 12)) == 0);                    /* 屏蔽只在STOP期间生效。 */
    assert((stop_exti_imr & (1U << 4)) != 0);                    /* PA4全程armed。 */
    assert(host_exti.IMR == ((1U << 4) | (1U << 12) | (1U << 1))); /* 提前唤醒也精确恢复。 */
    reset(0);
    screen_off = 1;
    host_exti.IMR = 1U << 4; /* 进入STOP前PB12已被其他逻辑屏蔽，退出不能擅自打开。 */
    sleep_rtc_counts = 6554;
    step_limit = 401;
    app_tickless_sleep(400);
    assert(stops == 1 && stop_exti_imr == (1U << 4) && host_exti.IMR == (1U << 4));

    /* 有sensor进度授权的1秒STOP：整段等待由暂停/恢复保护，不受400ms收紧；
     * RTC一分钟内计数差恰为32768，补满1000个tick与TIM2的10000个100us诊断计数。 */
    reset(0);
    screen_off = 1;
    wd_auth = 1;
    sleep_rtc_counts = 32768U;
    step_limit = 1000;
    app_tickless_sleep(1000U);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 1000 && uwTick == 1000);
    assert(TIM2->CNT == 10000);
    assert(wakeup_counts == 999U * WATCH_RTC_SUBSECOND_HZ / 1000U);
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);

    /* 起点距一分钟模数边界仅200计数：STOP跨分（午夜同分钟也是同一模数回绕）
     * 之后取模差仍是完整1秒，绝不能因回绕补成0。 */
    reset(0);
    screen_off = 1;
    wd_auth = 1;
    rtc_count = WATCH_RTC_COUNTER_MOD - 200U;
    sleep_rtc_counts = 32768U;
    step_limit = 1000;
    app_tickless_sleep(1000U);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 1000 && uwTick == 1000);
    assert(TIM2->CNT == 10000);
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);

    /* 从普通秒边界前768计数起步的完整1秒：跨秒取模差同样为32768。 */
    reset(0);
    screen_off = 1;
    wd_auth = 1;
    rtc_count = 32000U;
    sleep_rtc_counts = 32768U;
    step_limit = 1000;
    app_tickless_sleep(1000U);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 1000 && uwTick == 1000);
    assert(TIM2->CNT == 10000);
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);

    /* 有授权时外设提前唤醒（约30ms）：只按实测RTC差补时，暂停/恢复仍各一次，
     * 剩余等待由后续IDLE轮次重新决策。 */
    reset(0);
    screen_off = 1;
    wd_auth = 1;
    sleep_rtc_counts = 1000U;
    step_limit = 1000;
    app_tickless_sleep(1000U);
    assert(stops == 1 && sleeps == 0 && kernel_tick == 30 && uwTick == 30);
    assert(TIM2->CNT == 305);
    assert(wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);

    /* 授权300ms短STOP一次、随后授权失效的模型（覆盖touch重试300ms跨过200ms授权窗口的反例）：
     * 暂停只在第一次实际等待之前发生；失去授权后700ms请求收紧到400ms，
     * 且自始至终不再出现新的暂停，硬件超时检测连续有效。 */
    reset(0);
    screen_off = 1;
    wd_auth = 1;
    sleep_rtc_counts = 9830U; /* 约300ms。 */
    step_limit = 1001;
    app_tickless_sleep(300U);
    assert(stops == 1 && wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);
    assert(wakeup_counts == 299U * WATCH_RTC_SUBSECOND_HZ / 1000U);
    authorized_resume_rtc = wd_resume_rtc;
    /* 硬件恢复时刻即这次STOP结束，约300ms，不能提前也不能滞后。 */
    assert(authorized_resume_rtc * 1000U / WATCH_RTC_SUBSECOND_HZ >= 299U);
    assert(authorized_resume_rtc * 1000U / WATCH_RTC_SUBSECOND_HZ <= 301U);

    wd_auth = 0;
    sleep_rtc_counts = 13107U; /* 约400ms：无授权的700ms请求收紧至此上限。 */
    app_tickless_sleep(700U);
    assert(stops == 2 && wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);
    assert(wakeup_counts == 399U * WATCH_RTC_SUBSECOND_HZ / 1000U);

    sleep_rtc_counts = 9830U; /* 剩余300ms。 */
    app_tickless_sleep(300U);
    assert(stops == 3 && wd_pause_count == 1 && wd_resume_count == 1 && !wd_paused && wd_in_stop == 0);
    assert(wakeup_counts == 299U * WATCH_RTC_SUBSECOND_HZ / 1000U);
    assert(kernel_tick >= 990U && kernel_tick <= 1000U);
    {
        uint32_t gap_counts;
        /* 自最后一次硬件恢复计数到累计等待结束的间隔：软件授权失效后由400/300ms上限
         * 保证该间隔仍小于TPS3823最短900ms超时。 */
        gap_counts = (rtc_count + WATCH_RTC_COUNTER_MOD - authorized_resume_rtc) % WATCH_RTC_COUNTER_MOD;
        assert(gap_counts * 1000ULL / WATCH_RTC_SUBSECOND_HZ >= 600U);
        assert(gap_counts * 1000ULL / WATCH_RTC_SUBSECOND_HZ <= 900U);
    }

    puts("PASS: an authorized 1s STOP compensates a full second across minute/midnight-modulo and second boundaries (never 0 on wrap), "
         "pairs one pause/resume around the STOP, and after the authorization expires later waits cap at 400ms with no new pause "
         "while the hardware gap stays below the 900ms timeout.");
    puts("PASS: STOP wake/clock-before-ISR, RTC and diagnostic time, deadline clamp, BLE/PWM/SPI/DMA/ADC veto, WUT and shadow-sync fallback.");
    puts("PASS: every real STOP masks the MPU PB12 EXTI (keeps PA4 and other lines) and restores the exact original IMR on normal and early wake.");
    puts("PASS: 5/10/100/200/400ms STOP blocks a new DATA_RDY event during STOP, keeps its EXTI PR pending, and still ends at the RTC deadline; preexisting NVIC pending is not blocked.");
    return 0;
}
