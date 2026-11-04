/**
  ******************************************************************************
  * @file    app_tickless.c
  * @brief   应用层-空闲合并时钟 (tickless idle)
  *          IDLE 每轮被调用一次：停 SysTick → 用 RTC(LSE) 亚秒计数作为主参考，再补上
  *          RTC 采样区间之外的 CPU 运行开销 → 一次性补回 uwTick 与 RTOS tick，再恢复
  *          1ms 节拍。DWT 只用于补这段区间外的运行开销，不承担睡眠计时。
  *          两段深度不同：Sleep 保留外设时钟、能继续响应串口/EXTI，单次最长约 262ms
  *          （SysTick LOAD 只有 24 位），更长的等待由内核多次进入；STOP 由 RTC 唤醒
  *          定时器(WUT)唤醒（MPU 的 PB12 临时屏蔽，PA4 等其余 EXTI 保留），醒来后必须
  *          先恢复 64MHz PLL 再走后面的补时计算。若 RTC 参考不可用，Sleep 改用本次
  *          加长的 SysTick 回退测量经过时间。并非每个 IDLE 轮次都会睡满一个长周期，
  *          实际时长受最近任务截止时间、1000ms 上限和 STOP 资格共同收紧；
  *          无 sensor 进度授权时超过 400ms 的请求再收紧至 400ms 以保持硬件超时。
  *          计时归属：LSE/RTC 走时是秒级以下的参考源，TIM2 由 app_diag 用作 100us 诊断
  *          计数并在 STOP 期间停走，本文件负责把停走的这段时间也补回去。
  ******************************************************************************
***/
#include "app_tickless.h"
#include "app_task.h"
#include "watch_rtc.h"
#include "tps3823.h"
#include "FreeRTOS.h"
#include "task.h"

#if configTICK_RATE_HZ != 1000
#error Tickless HAL compensation requires a 1ms RTOS tick.
#endif

/* Sleep保留串口接收，合适的熄屏间隙用RTC唤醒STOP；最长1000ms覆盖静止熄屏1秒唤醒，仍受最近任务截止时间约束。 */
#define APP_TICKLESS_MAX_TICKS 1000
/* 无sensor进度授权时，超过该时长的等待收紧至此值；TPS3823最短900ms超时能在此范围内触发，
 * 保持硬件计时可在任务停滞后复位。 */
#define APP_TICKLESS_WATCHDOG_PAUSE_MIN_TICKS 400
#define APP_TICKLESS_STOP_MIN_TICKS 5
/* MPU DATA_RDY接PB12；sensor按任务期限读数据，STOP临时屏蔽PB12无效唤醒。 */
#define APP_TICKLESS_MPU_EXTI_LINE 12

static uint32_t s_rtc_fraction;
extern __IO uint32_t uwTick;

/* STOP 资格判定（返回1才允许深睡）：STOP 会停掉外设时钟，串口收不到数据、ADC 采样会中断，
 * 因此要求屏幕已熄、背光 PWM 占空为 0、蓝牙使能脚拉低、USART1 时钟关闭，且 SPI1/DMA2/ADC
 * 没有在途操作；任一条不满足就退回 Sleep，只合并 SysTick 而不切外设。
 * 这是软件侧的前提检查，不构成"STOP期间任何中断都能收到"的硬件保证。 */
static uint8_t app_tickless_stop_allowed(void)
{
    /* F411 USART无法在STOP接收；屏灭后仍可能有LCD DMA安全收尾，必须等它完成。 */
    /* 逐项与运算：全部条件为真才返回 1（允许 STOP），任一条不满足返回 0 退回 Sleep。 */
    /* 条件依次为：总开关打开、屏幕已熄、背光 PWM 占空为 0、蓝牙使能脚为低、USART1 时钟关闭、
     * SPI1 不忙、DMA2 Stream2 未使能、ADC1 未开启。 */
    return APP_TICKLESS_ENABLE_STOP != 0 && app_task_is_screen_off() != 0 && TIM3->CCR3 == 0 &&
           (BLE_EN_GPIO_Port->ODR & BLE_EN_Pin) == 0 && (RCC->APB2ENR & RCC_APB2ENR_USART1EN) == 0 &&
           (SPI1->SR & SPI_SR_BSY) == 0 && (DMA2_Stream2->CR & DMA_SxCR_EN) == 0 &&
           (ADC1->CR2 & ADC_CR2_ADON) == 0;
}

/* STOP醒来后把系统时钟切回64MHz PLL：STOP唤醒后硬件先以HSI运行，RCC配置保留。
 * 全程在关中断上下文执行，用寄存器轮询而不是HAL延时；PLL或切换超时进Error_Handler。 */
static void app_tickless_restore_clock(void)
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
            Error_Handler();
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
            Error_Handler();
        }
    }
    /* 数据同步屏障，确保时钟切换在后续指令前生效。 */
    __DSB();
    /* 指令同步屏障，清空流水线，避免取到切换前的旧状态。 */
    __ISB();
}

/* 在IDLE上下文、关中断状态下合并一次空闲：先查询sensor进度授权，再判断能否STOP，
 * 停SysTick、取RTC起点，然后STOP(或加长SysTick的Sleep)等待，醒来按RTC差值(不可用时按SysTick)
 * 折算周期，最后一次性补uwTick与RTOS tick并恢复1ms节拍。expected_ticks单位是tick(1ms)，
 * 会被APP_TICKLESS_MAX_TICKS和最近任务截止时间收紧；无授权时超过
 * APP_TICKLESS_WATCHDOG_PAUSE_MIN_TICKS的请求还会进一步收紧至该值。 */
void app_tickless_sleep(uint32_t expected_ticks)
{
    /* 每 tick 对应的 CPU 周期数：64MHz 主频 / 1000Hz 节拍 = 64000。 */
    const uint32_t counts_per_tick = configCPU_CLOCK_HZ / configTICK_RATE_HZ;
    /* control 保存 SysTick 原 CTRL 配置；remaining 为停表时剩余计数；rtc_start/rtc_end 为一分钟内计数；
     * elapsed_counts 为折算出的总 CPU 周期；phase 为停表时已走过的当前 tick 相位。 */
    uint32_t control, remaining, rtc_start, rtc_end, elapsed_counts, phase;
    /* complete_ticks 为折算出的完整 tick 数；pend_tick 标记收尾是否需要补置 pending。 */
    uint32_t complete_ticks, pend_tick = 0;
    /* DWT周期戳：停表时刻、进入等待前时刻，以及两次RTC读取前的采样戳。 */
    uint32_t stopped_cycle, resumed_cycle, rtc_start_cycle, rtc_end_cycle;
    /* STOP 相关：唤醒计数、TIM2 起止计数值与停钟期间应补的计数。 */
    uint32_t stop_counts = 0, timer_start = 0, timer_end, timer_elapsed;
    /* STOP 前保存的 EXTI 中断屏蔽寄存器原值，醒来后精确恢复。 */
    uint32_t exti_imr_saved = 0;
    /* 本轮是否选择STOP路径：RTC唤醒配置成功时为1。 */
    uint8_t use_stop = 0;
    /* 本轮是否已获sensor进度授权：在实际WFI/STOP前暂停TPS3823超时检测，醒来后立即恢复；
     * 不依赖use_stop，RTC无效、1ms回退和Sleep路径同样可暂停。 */
    uint8_t pause_watchdog = 0;
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
    /* 查询一次sensor进度授权：sensor已启动且距最近喂狗不超过200ms时为1。
     * 该结果决定本轮所有实际等待路径（含1ms回退、Sleep、STOP）是否暂停TPS3823超时检测；
     * 授权不依赖use_stop，不在tps3823_stop_resume路径更新last_feed_ms，
     * 因此200ms内无新sensor进度即失去授权，不会无限掩盖任务停滞。 */
    pause_watchdog = app_task_watchdog_stop_allowed();
    /* 读取 RTC 一分钟内递增计数作为睡眠起点的参考时基。 */
    rtc_start = watch_rtc_get_counter();
    /* RTC 参考不可用（UINT32_MAX）或等待短于 2 tick 时不做补偿，只保留 1ms 心跳 WFI。 */
    if (rtc_start == UINT32_MAX || expected_ticks < 2)
    {
        /* LSE启动/故障或等待过短时不做补偿，只保留1ms心跳WFI；
         * 已获sensor进度授权时在实际WFI前暂停超时检测，醒来立即恢复并补喂。 */
        if (pause_watchdog != 0)
        {
            tps3823_stop_pause();
        }
        /* 屏障保证 WFI 前的状态已写入。 */
        __DSB();
        /* 进入一次普通 WFI，由下一个中断（通常是 1ms 节拍）唤醒。 */
        __WFI();
        /* WFI返回后立即恢复超时检测，早于后面的任何补时计算。 */
        if (pause_watchdog != 0)
        {
            tps3823_stop_resume();
        }
        /* 唤醒后清流水线，再开中断返回。 */
        __ISB();
        __enable_irq();
        return;
    }
    /* 把期望睡眠时长收紧到 1000 tick 上限，避免单轮睡得过久。 */
    if (expected_ticks > APP_TICKLESS_MAX_TICKS)
    {
        expected_ticks = APP_TICKLESS_MAX_TICKS;
    }
    /* 无sensor进度授权时超过400ms的等待收紧至该值，使TPS3823硬件超时（最短900ms）
     * 能在任务停滞后及时触发复位。授权时不收紧，全段等待由pause/resume保护。 */
    if (pause_watchdog == 0 && expected_ticks > APP_TICKLESS_WATCHDOG_PAUSE_MIN_TICKS)
    {
        expected_ticks = APP_TICKLESS_WATCHDOG_PAUSE_MIN_TICKS;
    }

    /* 时长达到 STOP 门槛且软件前提全部满足时，尝试用 RTC 唤醒定时器安排 STOP。 */
    if (expected_ticks >= APP_TICKLESS_STOP_MIN_TICKS && app_tickless_stop_allowed() != 0)
    {
        /* 预留1ms给RTC同步/PLL恢复，余下不足一拍的时间由恢复后的SysTick完成。 */
        /* 把 tick 数换算成 RTC 亚秒计数（32768Hz 参考）；减 1 即预留的那 1ms。 */
        stop_counts = (expected_ticks - 1) * WATCH_RTC_SUBSECOND_HZ / configTICK_RATE_HZ;
        /* 启动唤醒定时器；返回 0 表示配置成功，才把 use_stop 置 1。 */
        use_stop = watch_rtc_start_wakeup(stop_counts) == 0;
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
        /* 若已安排 STOP 唤醒，先撤销，避免残留定时器。 */
        if (use_stop != 0)
        {
            watch_rtc_stop_wakeup();
        }
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

    /* 在最近任务截止时间唤醒；到期的最后一个tick交ISR处理，step不越过解阻塞点。 */
    /* 记录进入等待前的 DWT 周期戳（仅回退路径会用到）。 */
    resumed_cycle = DWT->CYCCNT;
    /* STOP 路径：屏蔽 MPU 数据就绪中断后深睡，醒来先恢复 PLL。 */
    if (use_stop != 0)
    {
        /* 所有真实STOP都临时屏蔽PB12，PA4/RTC及其它线保留，NVIC使能不动。
         * 屏蔽前已置位的pending不清，WFI仍可能立即返回。 */
        /* 保存 EXTI 中断屏蔽寄存器原值，便于醒来精确恢复。 */
        exti_imr_saved = EXTI->IMR;
        /* 只清 PB12 对应的屏蔽位，其余中断线保持原状。 */
        EXTI->IMR = exti_imr_saved & ~(1U << APP_TICKLESS_MPU_EXTI_LINE);
        /* 已获sensor进度授权时在实际STOP前暂停超时检测，PB1/WDG_EN保持接通欠压复位。 */
        if (pause_watchdog != 0)
        {
            tps3823_stop_pause();
        }
        /* 屏障保证屏蔽设置与授权暂停在进入 STOP 前生效。 */
        __DSB();
        /* 进入 STOP 模式（低功耗稳压器、WFI 进入），由 RTC 唤醒定时器等中断唤醒。 */
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        /* 醒来立即恢复超时检测，必须早于PLL恢复；PLL失败时欠压/超时复位通路已恢复。 */
        if (pause_watchdog != 0)
        {
            tps3823_stop_resume();
        }
        /* 醒来后把系统时钟从 HSI 切回 64MHz PLL。 */
        app_tickless_restore_clock();
        /* 精确恢复原IMR；保留的pending请求在开中断后处理。 */
        EXTI->IMR = exti_imr_saved;
        /* STOP后重新同步RTC；失败则将RTC参考标为不可用，本轮无法通过SysTick恢复STOP停钟期间的时间。 */
        if (watch_rtc_sync_after_stop() != 0)
        {
            rtc_start = UINT32_MAX;
        }
        /* 关闭本轮 RTC 唤醒定时器，避免影响后续睡眠。 */
        watch_rtc_stop_wakeup();
    }
    else
    {
        /* Sleep保留外设时钟由SysTick定时；LOAD仅24位，单次最长约262ms。超过该上限的等待
         * 分多次Sleep完成，内核每次返回后再按最近截止时间重排，不跨过解阻塞点。
         * 长SysTick同时充当RTC参考失效时的回退时基。 */
        /* 本轮 Sleep 的 tick 数，先按期望值，再受 24 位 LOAD 上限收紧。 */
        uint32_t sleep_ticks = expected_ticks;
        /* 24 位重装值能表示的最大 tick 数：0x1000000 个周期 / 64000。 */
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
        /* 已获sensor进度授权时在实际WFI前暂停超时检测，Sleep期间TPS3823不计数。 */
        if (pause_watchdog != 0)
        {
            tps3823_stop_pause();
        }
        /* 屏障保证重装与使能在 WFI 前完成。 */
        __DSB();
        /* 进入 Sleep，等待长 SysTick 或外设中断唤醒。 */
        __WFI();
        /* WFI返回后立即恢复超时检测并补喂一次，早于后面的补时计算。 */
        if (pause_watchdog != 0)
        {
            tps3823_stop_resume();
        }
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

    /* RTC可用时以一分钟内计数为准：两次读数之差按 64MHz / 32768Hz 折算成CPU周期，
     * 除不尽的余数留在 s_rtc_fraction 里下一轮继续折算，避免每轮都向下取整把空闲时间越睡越少。 */
    if (rtc_start != UINT32_MAX && rtc_end != UINT32_MAX)
    {
        /* 单次最长1000ms，远小于一分钟模数；先加模数再取模，覆盖跨秒、跨分和午夜。
         * 日历校时只能在任务恢复后发生，本区间内计数单调递增。 */
        scaled_counts = (uint64_t)((rtc_end + WATCH_RTC_COUNTER_MOD - rtc_start) % WATCH_RTC_COUNTER_MOD);
        /* 换算成 CPU 周期，并加上上一轮遗留的余数。 */
        scaled_counts = scaled_counts * configCPU_CLOCK_HZ + s_rtc_fraction;
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
            /* 长 SysTick 走过的周期数 = 重装值 - 当前值。 */
            elapsed_counts = SysTick->LOAD - SysTick->VAL;
            /* 若期间已产生到期 pending，说明还跨过了一个完整重装周期，补加一整周期。 */
            if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0)
            {
                elapsed_counts += SysTick->LOAD + 1;
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
    /* 实测达到或超过期望时长时，只补到最后一拍之前，把最后一拍留给正常 ISR。 */
    if (complete_ticks >= expected_ticks)
    {
        complete_ticks = expected_ticks - 1;
        /* 标记收尾需要补置 pending，让 ISR 处理这最后一拍。 */
        pend_tick = 1;
    }

    /* 收尾一次性补齐时间：长SysTick自己产生的pending不是普通的1ms节拍，先清掉；
     * 按实测周期数装回SysTick，uwTick与RTOS tick同步加上，LOAD随后恢复成1ms；
     * 若实测已达到expected_ticks，只补到最后一拍之前，留下的pending交给正常ISR解除阻塞。 */
    /* 清除长 SysTick 期间产生的到期标志，避免被当成普通节拍重复计数。 */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    /* 先按剩余周期数装载 SysTick，使下一拍在正确相位到期。 */
    SysTick->LOAD = remaining - 1;
    /* 清零当前计数值，从新装载值开始计数。 */
    SysTick->VAL = 0;
    /* 按原配置重新使能 SysTick。 */
    SysTick->CTRL = control;
    /* HAL 毫秒计数一次性补上折算出的完整 tick 数。 */
    uwTick += complete_ticks;
    /* RTOS tick 同步推进同样的 tick 数。 */
    vTaskStepTick((TickType_t)complete_ticks);
    /* 恢复 1ms 的标准重装值，回到正常节拍。 */
    SysTick->LOAD = counts_per_tick - 1;
    /* 若有一拍留给 ISR，补置 pending 让正常节拍中断随后处理。 */
    if (pend_tick != 0)
    {
        SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
    }
    /* UART/DMA/EXTI中断第一次执行时，HAL和RTOS时间已经补齐。 */
    /* 屏障保证补时与装载对后续中断可见。 */
    __DSB();
    /* 清流水线，避免执行切换前的旧指令。 */
    __ISB();
    /* 重新开中断，恢复正常运行。 */
    __enable_irq();
}
