/**
  ******************************************************************************
  * @file    app_tickless.c
  * @brief   应用层-空闲合并时钟 (tickless idle)
  *          IDLE 每轮被调用一次：停 SysTick → 用 RTC(LSE) 亚秒计数作为主参考，再补上
  *          RTC 采样区间之外的 CPU 运行开销 → 一次性补回 uwTick 与 RTOS tick，再恢复
  *          1ms 节拍。DWT 只用于补这段区间外的运行开销，不承担睡眠计时。
  *          两段深度不同：Sleep 保留外设时钟、能继续响应串口/EXTI，单次最长约 199ms
  *          （SysTick LOAD 只有 24 位），更长的等待由内核多次进入；STOP 由 RTC 唤醒
  *          定时器(WUT)、PA4或已武装的MPU MOT/PB12唤醒，醒来后必须
  *          先恢复 84MHz PLL 再走后面的补时计算。若 RTC 参考不可用，Sleep 改用本次
  *          加长的 SysTick 回退测量经过时间。并非每个 IDLE 轮次都会睡满一个长周期，
  *          实际时长受最近任务截止时间、400ms 上限和 STOP 资格共同收紧；
  *          WDI 始终保持输出，RTC/LSE 停振时外部看门狗仍能超时复位。
  *          计时归属：LSE/RTC 走时是秒级以下的参考源，TIM2 由 app_diag 用作 100us 诊断
  *          计数并在 STOP 期间停走，本文件负责把停走的这段时间也补回去。
  ******************************************************************************
***/
#include "app_tickless.h"
#include "app_task.h"
#include "app_reset.h"
#include "watch_rtc.h"
#include "lcd.h"
#include "kt6368a.h"
#include "battery_adc.h"
#include "FreeRTOS.h"
#include "task.h"

#if configTICK_RATE_HZ != 1000
#error Tickless HAL compensation requires a 1ms RTOS tick.
#endif

/* 保持TPS3823连续计时；400ms上限给最短900ms超时留出运行裕量，普通熄屏还受25ms传感器期限限制。 */
#define APP_TICKLESS_MAX_TICKS 400
#define APP_TICKLESS_STOP_MIN_TICKS 5
/* PB12只在MOT已武装时作为STOP唤醒源。 */
#define APP_TICKLESS_MPU_EXTI_LINE 12

/* STOP故障用TASK记录分支码，PC记录寄存器快照或未截断的RTC差值。 */
typedef enum
{
    APP_TICKLESS_STOP_OK = 0,
    APP_TICKLESS_STOP_PVDO = 1,
    APP_TICKLESS_STOP_RTC_SYNC = 3,
    APP_TICKLESS_STOP_PLL_TIMEOUT = 21,
    APP_TICKLESS_STOP_VOS_TIMEOUT = 22,
    APP_TICKLESS_STOP_SWS_TIMEOUT = 23,
    APP_TICKLESS_STOP_RTC_READ = 41,
    APP_TICKLESS_STOP_RTC_DELTA = 42
} app_tickless_stop_tag;

static uint32_t s_rtc_fraction;
extern __IO uint32_t uwTick;

/* STOP 资格判定（返回1才允许深睡）：STOP 会停掉外设时钟，串口收不到数据、ADC 采样会中断，
 * 因此要求屏幕已熄、背光 PWM 占空为 0、蓝牙使能脚拉低、USART1 时钟关闭，且 SPI1/DMA2/ADC
 * 没有在途操作；任一条不满足就退回 Sleep，只合并 SysTick 而不切外设。
 * 这是软件侧的前提检查，不构成"STOP期间任何中断都能收到"的硬件保证。 */
static uint8_t app_tickless_stop_allowed(void)
{
    return APP_TICKLESS_ENABLE_STOP != 0 && app_task_is_screen_off() != 0 &&
           lcd_is_stop_idle() != 0 && kt6368a_is_stop_idle() != 0 && battery_adc_is_stop_idle() != 0;
}

/* STOP醒来后把系统时钟切回84MHz PLL：STOP唤醒后硬件先以HSI运行，RCC配置保留。
 * 全程在关中断上下文执行，用寄存器轮询而不是HAL延时；PLL或切换超时进Error_Handler。 */
static app_tickless_stop_tag app_tickless_restore_clock(void)
{
    /* 轮询计数：按 HSI_VALUE/100 设定重试上限，超时则进入错误处理。 */
    uint32_t retries = HSI_VALUE / 100;

    /* STOP醒来使用HSI；寄存器配置保留。中断屏蔽期间不用依赖HAL tick的时钟接口。 */
    /* 重新打开 PLL（STOP 唤醒后该使能位不保证保持，此处显式置位）。 */
    RCC->CR |= RCC_CR_PLLON;
    /* 轮询等待 PLL 锁定；每次未就绪先递减重试计数，减到 0 视为超时。 */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
        if (--retries == 0)
        {
            return APP_TICKLESS_STOP_PLL_TIMEOUT;
        }
    }
    /* PLL锁定后还需等待电压调节器达到84MHz所需档位。 */
    retries = HSI_VALUE / 100;
    while ((PWR->CSR & PWR_CSR_VOSRDY) == 0)
    {
        if (--retries == 0)
        {
            return APP_TICKLESS_STOP_VOS_TIMEOUT;
        }
    }
    /* 把系统时钟切换源设为 PLL。 */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    /* 重置重试计数，供下面的切换完成等待使用。 */
    retries = HSI_VALUE / 100;
    /* 轮询 SWS 状态位，直到系统时钟实际已切到 PLL。 */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        if (--retries == 0)
        {
            return APP_TICKLESS_STOP_SWS_TIMEOUT;
        }
    }
    /* 数据同步屏障，确保时钟切换在后续指令前生效。 */
    __DSB();
    /* 指令同步屏障，清空流水线，避免取到切换前的旧状态。 */
    __ISB();
    return APP_TICKLESS_STOP_OK;
}

/* 关中断合并空闲，按RTC实测时间补回HAL/RTOS；最近任务期限与400ms上限共同限制等待。
 * Sleep、STOP和1ms回退均不暂停或刷新看门狗，喂狗仍只由双任务进度决定。 */
void app_tickless_sleep(uint32_t expected_ticks)
{
    /* 每 tick 对应的 CPU 周期数：84MHz 主频 / 1000Hz 节拍 = 84000。 */
    const uint32_t counts_per_tick = configCPU_CLOCK_HZ / configTICK_RATE_HZ;
    /* control 保存 SysTick 原 CTRL 配置；remaining 为停表时剩余计数；rtc_start/rtc_end 为一分钟内计数；
     * elapsed_counts 为折算出的总 CPU 周期；phase 为停表时已走过的当前 tick 相位。 */
    uint32_t control, remaining, rtc_start, rtc_end, elapsed_counts, phase;
    /* complete_ticks 为实测完整tick，step_ticks 为期限前可直接推进的部分。 */
    uint32_t complete_ticks, step_ticks;
    /* DWT周期戳：停表时刻、进入等待前时刻，以及两次RTC读取前的采样戳。 */
    uint32_t stopped_cycle, resumed_cycle, rtc_start_cycle, rtc_end_cycle;
    /* STOP 相关：唤醒计数、TIM2 起止计数值与停钟期间应补的计数。 */
    uint32_t stop_counts = 0, timer_start = 0, timer_end, timer_elapsed;
    /* STOP 前保存的 EXTI 中断屏蔽寄存器原值，醒来后精确恢复。 */
    uint32_t exti_imr_saved = 0;
    /* 本轮是否选择STOP路径：RTC唤醒配置成功时为1。 */
    uint8_t use_stop = 0;
    /* 64 位中间量：亚秒计数换算成 CPU 周期时避免乘法溢出。 */
    uint64_t scaled_counts;

    /* 屏蔽可屏蔽中断，后续在该屏蔽区内停SysTick、切时钟并补时。 */
    __disable_irq();
    /* 数据同步屏障，确保前面的关中断与后续寄存器操作按序完成。 */
    __DSB();
    /* 指令同步屏障，清空流水线。 */
    __ISB();
    /* 内核判定为 eAbortSleep（有任务需在切换前运行等）或已有到期 tick 时，放弃本轮睡眠。 */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep || (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
    {
        /* 恢复中断后返回，交给正常调度/节拍处理。 */
        __enable_irq();
        return;
    }
    /* 清 SLEEPDEEP 位，默认走 Sleep 而非 STOP；与原HAL Sleep入口一致，保留外设时钟。 */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; /* 与原HAL Sleep入口一致，保留外设时钟。 */
    /* 读取 RTC 一分钟内递增计数作为睡眠起点的参考时基。 */
    rtc_start = watch_rtc_get_counter();
    /* RTC 参考不可用（UINT32_MAX）或等待短于 2 tick 时不做补偿，只保留 1ms 心跳 WFI。 */
    if (rtc_start == UINT32_MAX || expected_ticks < 2)
    {
        /* LSE启动/故障或等待过短时，保持看门狗计时并由1ms心跳唤醒。 */
        /* 屏障保证 WFI 前的状态已写入。 */
        __DSB();
        /* 进入一次普通 WFI，由下一个中断（通常是 1ms 节拍）唤醒。 */
        __WFI();
        /* 唤醒后清流水线，再开中断返回。 */
        __ISB();
        __enable_irq();
        return;
    }
    /* 把期望睡眠时长收紧到 400 tick 上限，避免单轮睡得过久。 */
    if (expected_ticks > APP_TICKLESS_MAX_TICKS)
    {
        expected_ticks = APP_TICKLESS_MAX_TICKS;
    }
    /* DWT只测量RTC区间之外的运行开销，不依赖它在Sleep中的计数行为。 */
    /* 使能调试跟踪单元与 DWT 周期计数器。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* 保存 SysTick 当前控制配置，供停表与收尾恢复。 */
    control = SysTick->CTRL;
    /* 记录停表前的 DWT 周期戳。 */
    stopped_cycle = DWT->CYCCNT;
    /* 关闭 SysTick 计数，停止 1ms 节拍。 */
    SysTick->CTRL = control & ~SysTick_CTRL_ENABLE_Msk;
    /* 读取当前计数值（向下计数），用于计算已走过的相位。 */
    remaining = SysTick->VAL;
    /* 停表前刚到期的普通tick由原ISR处理，不能清掉或再次补偿。 */
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
    {
        /* 原样恢复 SysTick 控制，重新开中断后返回。 */
        SysTick->CTRL = control;
        __enable_irq();
        return;
    }
    /* 计数值读到 0 时按一整周期处理，避免相位算出满量程。 */
    if (remaining == 0)
    {
        remaining = SysTick->LOAD + 1;
    }
    /* 已走过的相位 = 一个 tick 的周期数 - 剩余计数。 */
    phase = counts_per_tick - remaining;
    if (expected_ticks >= APP_TICKLESS_STOP_MIN_TICKS && app_tickless_stop_allowed() != 0)
    {
        /* 从任务期限扣除当前相位，并给唤醒后的时钟恢复预留1ms。 */
        stop_counts = (uint32_t)((uint64_t)((expected_ticks - 1) * counts_per_tick - phase) *
                                 WATCH_RTC_SUBSECOND_HZ / configCPU_CLOCK_HZ);
        use_stop = watch_rtc_start_wakeup(stop_counts) == 0;
    }
    /* 记录 RTC 起始读取前的 DWT 周期戳，供后面扣除读取开销。 */
    rtc_start_cycle = DWT->CYCCNT;
    /* 记录 TIM2 起始计数值，STOP 后据此补回停钟时间。 */
    timer_start = TIM2->CNT;
    /* 再次读取 RTC 一分钟内计数，作为实际睡眠区间起点。 */
    rtc_start = watch_rtc_get_counter();
    /* 若这次读数不可用，撤销 STOP 唤醒并退回 Sleep 路径。 */
    if (use_stop != 0 && rtc_start == UINT32_MAX)
    {
        watch_rtc_stop_wakeup();
        use_stop = 0;
    }

    /* 记录进入等待前的 DWT 周期戳（仅回退路径会用到）。 */
    resumed_cycle = DWT->CYCCNT;
    /* STOP期间保留已武装的MOT中断，醒来先恢复PLL再开放中断。 */
    if (use_stop != 0)
    {
        uint8_t mpu_exti_masked;
        exti_imr_saved = EXTI->IMR;
        mpu_exti_masked = (exti_imr_saved & (1U << APP_TICKLESS_MPU_EXTI_LINE)) != 0 && app_task_mpu_mot_armed() == 0;
        if (mpu_exti_masked != 0)
        {
            EXTI->IMR = exti_imr_saved & ~(1U << APP_TICKLESS_MPU_EXTI_LINE);
        }
        uint8_t debug_enabled = (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0;
        if (debug_enabled == 0)
        {
            /* RTC覆盖整个STOP区间，期间无需DWT；清除可能残留的低功耗调试位。 */
            DBGMCU->CR &= ~(DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
            DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
            CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
        }
        __DSB();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        if (debug_enabled == 0)
        {
            /* 恢复周期计数，供RTC同步超时和唤醒后的补时使用。 */
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }
        /* 仍处于HSI时查询供电；欠压不再切回84MHz，走已有故障记录/复位路径。 */
        uint32_t pwr_csr = PWR->CSR;
        if ((pwr_csr & PWR_CSR_PVDO) != 0)
        {
            if (mpu_exti_masked != 0)
            {
                EXTI->IMR = exti_imr_saved;
            }
            watch_rtc_stop_wakeup();
            app_reset_capture_stop(APP_RESET_ERROR_HANDLER, pwr_csr, APP_TICKLESS_STOP_PVDO);
            Error_Handler();
            return;
        }
        /* 醒来后把系统时钟从 HSI 切回 84MHz PLL。 */
        app_tickless_stop_tag clock_fault = app_tickless_restore_clock();
        if (clock_fault != APP_TICKLESS_STOP_OK)
        {
            uint32_t detail = RCC->CR & 0xFFFF0000U;
            if (clock_fault == APP_TICKLESS_STOP_VOS_TIMEOUT)
            {
                detail |= PWR->CSR & 0xFFFF;
            }
            else
            {
                detail |= RCC->CFGR & 0xFFFF;
            }
            if (mpu_exti_masked != 0)
            {
                EXTI->IMR = exti_imr_saved;
            }
            watch_rtc_stop_wakeup();
            app_reset_capture_stop(APP_RESET_ERROR_HANDLER, detail, clock_fault);
            Error_Handler();
            return;
        }
        if (mpu_exti_masked != 0)
        {
            EXTI->IMR = exti_imr_saved;
        }
        /* STOP没有独立的运行时钟可补偿；同步失败时记录并复位，不能把未知时长当作0继续运行。 */
        if (watch_rtc_sync_after_stop() != 0)
        {
            /* 在关闭WUT清除WUTF前保存现场。 */
            uint32_t detail = ((RCC->BDCR & 0xFFFF) << 16) | (RTC->ISR & 0xFFFF);
            watch_rtc_stop_wakeup();
            app_reset_capture_stop(APP_RESET_ERROR_HANDLER, detail, APP_TICKLESS_STOP_RTC_SYNC);
            Error_Handler();
            return;
        }
        /* 关闭本轮 RTC 唤醒定时器，避免影响后续睡眠。 */
        watch_rtc_stop_wakeup();
    }
    else
    {
        /* Sleep保留外设时钟由SysTick定时；LOAD仅24位，单次最长约199ms。超过该上限的等待
         * 分多次Sleep完成，内核每次返回后再按最近截止时间重排，不跨过解阻塞点。
         * 长SysTick同时充当RTC参考失效时的回退时基。 */
        /* 本轮 Sleep 的 tick 数，先按期望值，再受 24 位 LOAD 上限收紧。 */
        uint32_t sleep_ticks = expected_ticks;
        /* 24 位重装值能表示的最大 tick 数：0x1000000 个周期 / 84000。 */
        uint32_t sleep_max = (0xFFFFFFU + 1U) / counts_per_tick;
        if (sleep_ticks > sleep_max)
        {
            sleep_ticks = sleep_max;
        }
        /* 在剩余计数基础上加长重装值，覆盖本次睡眠的 tick 数。 */
        SysTick->LOAD = remaining + counts_per_tick * (sleep_ticks - 1) - 1;
        /* 清零当前计数值，从新重装值开始计数。 */
        SysTick->VAL = 0;
        /* 按原配置重新使能 SysTick。 */
        SysTick->CTRL = control;
        /* 屏障保证重装与使能在 WFI 前完成。 */
        __DSB();
        /* 进入 Sleep，等待长 SysTick 或外设中断唤醒。 */
        __WFI();
        /* 唤醒后清流水线。 */
        __ISB();
        /* 再次停表，进入统一的补时计算。 */
        SysTick->CTRL = control & ~SysTick_CTRL_ENABLE_Msk;
    }
    /* 记录睡眠结束时的 DWT 周期戳。 */
    rtc_end_cycle = DWT->CYCCNT;
    /* 读取 TIM2 结束计数值。 */
    timer_end = TIM2->CNT;
    /* 读取 RTC 一分钟内计数作为睡眠终点参考。 */
    rtc_end = watch_rtc_get_counter();
    uint32_t rtc_delta = UINT32_MAX;
    if (rtc_start != UINT32_MAX && rtc_end != UINT32_MAX)
    {
        rtc_delta = (rtc_end + WATCH_RTC_COUNTER_MOD - rtc_start) % WATCH_RTC_COUNTER_MOD;
    }
    /* 给时钟恢复留余量，同时拒绝无效读数和整秒或整分钟跳变。 */
    if (rtc_delta > (APP_TICKLESS_MAX_TICKS + 10) * WATCH_RTC_SUBSECOND_HZ / configTICK_RATE_HZ)
    {
        if (use_stop != 0)
        {
            if (rtc_delta == UINT32_MAX)
            {
                uint32_t ssr = RTC->SSR;
                uint32_t tr = RTC->TR;
                (void)RTC->DR; /* 读日期释放日历影子寄存器锁存。 */
                uint32_t detail = ((tr & 0xFFFF) << 16) | (ssr & 0xFFFF);
                app_reset_capture_stop(APP_RESET_ERROR_HANDLER, detail, APP_TICKLESS_STOP_RTC_READ);
            }
            else
            {
                app_reset_capture_stop(APP_RESET_ERROR_HANDLER, rtc_delta, APP_TICKLESS_STOP_RTC_DELTA);
            }
            Error_Handler();
            return;
        }
        rtc_delta = UINT32_MAX;
    }

    /* RTC可用时以一分钟内计数为准：两次读数之差按 84MHz / 32768Hz 折算成CPU周期，
     * 除不尽的余数留在 s_rtc_fraction 里下一轮继续折算，避免每轮都向下取整把空闲时间越睡越少。 */
    if (rtc_delta != UINT32_MAX)
    {
        scaled_counts = (uint64_t)rtc_delta * configCPU_CLOCK_HZ + s_rtc_fraction;
        /* 取出完整 CPU 周期数作为经过时间。 */
        elapsed_counts = (uint32_t)(scaled_counts / WATCH_RTC_SUBSECOND_HZ);
        /* 除不尽的余数留到下一轮，避免反复向下取整累积误差。 */
        s_rtc_fraction = (uint32_t)(scaled_counts % WATCH_RTC_SUBSECOND_HZ);
        /* 只有 STOP 会让 TIM2 停走，因此仅该路径需要补诊断计数。 */
        if (use_stop != 0)
        {
            /* TIM2在STOP中停走；补上停钟时间，保留诊断周期和CPU占比的时间基准。 */
            /* 把经过的 CPU 周期换算成 TIM2 计数：每 100us 计一次，即每秒 10000 个计数。 */
            timer_elapsed = (uint32_t)((uint64_t)elapsed_counts * 10000 / configCPU_CLOCK_HZ);
            /* 若停钟期间应补的计数多于 TIM2 实际走过的量，把差额直接加进计数器。 */
            if (timer_elapsed > (uint32_t)(timer_end - timer_start))
            {
                TIM2->CNT += timer_elapsed - (uint32_t)(timer_end - timer_start);
            }
        }
    }
    else
    {
        /* Sleep中计数器仍运行；LSE运行期故障时用本次长SysTick保住经过时间。 */
        /* 先清零，仅在 Sleep 路径用 SysTick 差值回退计算。 */
        elapsed_counts = 0;
        if (use_stop == 0)
        {
            remaining = SysTick->VAL;
            elapsed_counts = SysTick->LOAD - remaining;
            if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
            {
                /* VAL仍为0时刚到期，下一周期还未装载。 */
                if (remaining == 0)
                {
                    elapsed_counts = SysTick->LOAD + 1;
                }
                else
                {
                    elapsed_counts += SysTick->LOAD + 1;
                }
            }
        }
        /* 回退路径没有有效RTC起始采样，使用进入等待前的周期戳作为CPU开销区间分界。 */
        rtc_start_cycle = resumed_cycle;
    }
    /* 两次RTC调用前取同一位置的周期戳，读寄存器的固定延迟相消。
     * 补上停表到起始采样、结束采样到恢复计算的实际CPU开销，周期差允许回绕。 */
    /* 补上停表到 RTC 起始采样之间的运行周期。 */
    __DSB();
    elapsed_counts += rtc_start_cycle - stopped_cycle;
    /* 补上 RTC 结束采样到当前计算之间的运行周期。 */
    elapsed_counts += DWT->CYCCNT - rtc_end_cycle;
    /* 加上停表前已走过的当前 tick 相位。 */
    elapsed_counts += phase;
    /* 总周期数除以每 tick 周期数，得到可补的完整 tick 数。 */
    complete_ticks = elapsed_counts / counts_per_tick;
    /* 余数换算成下一个 tick 还差的周期数，用于收尾装载。 */
    remaining = counts_per_tick - elapsed_counts % counts_per_tick;
    /* 不足约3us的尾段直接进位，给下面装载当前拍和恢复常规LOAD留出时间。 */
    if (remaining <= 256)
    {
        ++complete_ticks;
        remaining = counts_per_tick;
    }
    /* 直接跳过的节拍必须早于任务解阻塞点。 */
    step_ticks = complete_ticks;
    if (step_ticks >= expected_ticks)
    {
        step_ticks = expected_ticks - 1;
    }

    /* 清除长 SysTick 期间产生的到期标志，避免被当成普通节拍重复计数。 */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    /* 先按剩余周期数装载 SysTick，使下一拍在正确相位到期。 */
    SysTick->LOAD = remaining - 1;
    /* 清零当前计数值，从新装载值开始计数。 */
    SysTick->VAL = 0;
    /* 按原配置重新使能 SysTick。 */
    SysTick->CTRL = control;
    __DSB();
    /* 确认当前拍已从LOAD取值，在调用内核前恢复下一拍的1ms重装值。 */
    while (SysTick->VAL == 0)
    {
    }
    SysTick->LOAD = counts_per_tick - 1;
    /* HAL 毫秒计数一次性补上折算出的完整 tick 数。 */
    uwTick += complete_ticks;
    vTaskStepTick((TickType_t)step_ticks);
    /* IDLE此时已挂起调度器。到期及超期的节拍记入内核，恢复调度时逐拍处理任务期限。 */
    for (; step_ticks < complete_ticks; ++step_ticks)
    {
        (void)xTaskIncrementTick();
    }
    /* 屏障保证补时与装载对后续中断可见。 */
    __DSB();
    /* 清流水线，避免执行切换前的旧指令。 */
    __ISB();
    /* 重新开中断，恢复正常运行。 */
    __enable_irq();
}
