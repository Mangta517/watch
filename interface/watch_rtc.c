#include "watch_rtc.h"
#include <stddef.h>

#define WATCH_RTC_INITIALIZED 0x52544330
#define WATCH_RTC_SYNCED      0x52544331

static RTC_HandleTypeDef watch_rtc;
static uint8_t rtc_ready;
static uint8_t rtc_configured;
static uint32_t rtc_start_ms;
static uint8_t rtc_start_pending;
static HAL_StatusTypeDef rtc_start_status = HAL_ERROR;

/* RTC异步标志按DWT限1ms并用次数兜底；IDLE屏蔽中断时不依赖HAL tick。 */
static uint8_t watch_rtc_wait_flag(uint32_t flag)
{
    /* started 记录开始等待时的 DWT 周期计数；remaining 为兜底循环次数 */
    uint32_t started, remaining = SystemCoreClock / 1000;

    /* 使能 DWT 跟踪单元，否则 CYCCNT 不会计数 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* 启动周期计数器，用它在屏蔽中断时也能计量约 1ms 的等待上限 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* 记下起点周期数，后面用差值判断是否已过 1ms */
    started = DWT->CYCCNT;
    /* 轮询等待 RTC 的指定标志置位 */
    while ((RTC->ISR & flag) == 0)
    {
        /* LSE 停振、已等满约 1ms 周期或兜底次数耗尽，都立即返回失败 */
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0 ||
            (uint32_t)(DWT->CYCCNT - started) >= SystemCoreClock / 1000 || remaining == 0)
        {
            /* 返回 1 表示标志未在限时内等到 */
            return 1;
        }
        /* 递减兜底计数，防止 DWT 异常时死等 */
        --remaining;
        /* 空操作占一个周期，避免循环过紧 */
        __NOP();
    }
    /* 标志已置位；若 LSE 已停振仍返回失败，否则返回 0 表示成功 */
    return __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0;
}

void watch_rtc_init_begin(void)
{
    /* 清除就绪标志，初始化完成前不对外提供时间 */
    rtc_ready = 0;
    /* 清除已配置标志，等待 watch_rtc_init 成功后重新置位 */
    rtc_configured = 0;
    /* 打开 PWR 时钟，备份域访问需要它 */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* 解除备份域写保护，才能配置 RTC 和备份寄存器 */
    HAL_PWR_EnableBkUpAccess();
    /* 启动 LSE 振荡器，RTC 以 32768Hz 作为时钟源 */
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
    /* 记录启动时刻，供 watch_rtc_init_service 计算 5s 超时 */
    rtc_start_ms = HAL_GetTick();
    /* 标记启动流程尚未完成，后续由 service 逐拍推进 */
    rtc_start_pending = 1;
    /* 初始状态设为 BUSY，表示正在等待 LSE 起振 */
    rtc_start_status = HAL_BUSY;
}

HAL_StatusTypeDef watch_rtc_init_service(void)
{
    /* 没有待完成的启动流程时，直接返回上一次记录的启动结果 */
    if (rtc_start_pending == 0)
    {
        return rtc_start_status;
    }
    /* LSE 已就绪：执行完整 RTC 配置并结束等待 */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0)
    {
        /* 记录配置结果，供后续调用直接查询 */
        rtc_start_status = watch_rtc_init();
        /* 启动流程结束，不再重复配置 */
        rtc_start_pending = 0;
    }
    /* LSE 尚未就绪，检查是否已超过 5s 启动超时 */
    else if ((uint32_t)(HAL_GetTick() - rtc_start_ms) >= LSE_STARTUP_TIMEOUT)
    {
        /* 超时后记录 TIMEOUT，调用方据此显示 RTC 不可用 */
        rtc_start_status = HAL_TIMEOUT;
        /* 结束等待，避免每拍都继续轮询 */
        rtc_start_pending = 0;
    }
    /* 返回当前启动状态：BUSY 表示仍在等 LSE */
    return rtc_start_status;
}

uint8_t watch_rtc_init_is_pending(void)
{
    /* 返回 1 表示启动流程仍在等待 LSE 或尚未配置完成 */
    return rtc_start_pending;
}

HAL_StatusTypeDef watch_rtc_init(void)
{
    /* oscillator 描述本次要应用的 LSE 振荡器配置 */
    RCC_OscInitTypeDef oscillator = {0};
    /* clock 指定 RTC 使用 LSE 作为内核时钟 */
    RCC_PeriphCLKInitTypeDef clock = {0};
    /* time 保存首次初始化时要写入的时间值（字段先清零） */
    RTC_TimeTypeDef time = {0};
    /* date 保存首次初始化时要写入的占位日期 */
    RTC_DateTypeDef date = {0};
    /* status 接收各 HAL 调用返回的执行状态 */
    HAL_StatusTypeDef status;
    /* marker 保存从备份寄存器读出的初始化/校时标记 */
    uint32_t marker;

    /* 配置未完成前先撤销就绪标志 */
    rtc_ready = 0;
    /* 同时撤销已配置标志，失败时调用方不会误用 RTC */
    rtc_configured = 0;
    /* 打开 PWR 时钟，备份域写访问的前提 */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* 解除备份域写保护，允许配置 RTC 和备份寄存器 */
    HAL_PWR_EnableBkUpAccess();
    /* 本次只配置 LSE，不涉及 HSE/HSI/PLL */
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    /* 请求打开 LSE 振荡器 */
    oscillator.LSEState = RCC_LSE_ON;
    /* 不修改 PLL，避免影响系统主频配置 */
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    /* 应用振荡器配置；若 LSE 尚未起振，HAL 内部会等待或超时 */
    status = HAL_RCC_OscConfig(&oscillator);
    /* LSE 未能起振时直接返回 HAL 状态，RTC 保持未配置 */
    if (status != HAL_OK)
    {
        return status;
    }
    /* 只配置 RTC 这一路外设时钟 */
    clock.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    /* RTC 内核时钟选择 LSE（32768Hz） */
    clock.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    /* 应用RTC时钟源选择；同源LSE复用，若原先已选择其他RTC时钟源，切换至LSE时HAL会复位备份域。 */
    status = HAL_RCCEx_PeriphCLKConfig(&clock);
    /* 时钟源配置失败则返回，避免后续按错误时钟配置分频 */
    if (status != HAL_OK)
    {
        return status;
    }
    /* 打开 RTC 外设时钟，寄存器访问才有效 */
    __HAL_RCC_RTC_ENABLE();
    /* 指定使用芯片的 RTC 实例 */
    watch_rtc.Instance = RTC;
    /* 24 小时制，与上层显示和校时参数一致 */
    watch_rtc.Init.HourFormat = RTC_HOURFORMAT_24;
    /* 异步分频为 0：32768Hz 全部由同步分频承担 */
    watch_rtc.Init.AsynchPrediv = 0;
    /* 同步分频 = 32767，使亚秒计数范围为 0~32767 */
    watch_rtc.Init.SynchPrediv = WATCH_RTC_SUBSECOND_HZ - 1;
    /* 关闭 RTC 输出引脚功能，本板不需要输出时钟 */
    watch_rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    /* 输出极性配置为高（输出已关闭，此项仅保持结构体完整） */
    watch_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    /* 输出类型配置为开漏（输出已关闭，此项仅保持结构体完整） */
    watch_rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    /* 初始化 RTC 外设；若备份域已初始化过，HAL 会跳过日历重配 */
    status = HAL_RTC_Init(&watch_rtc);
    /* 初始化失败直接返回，不继续改分频或写时间 */
    if (status != HAL_OK)
    {
        return status;
    }
    /* HAL_RTC_Init会跳过已初始化的备份域；升级旧固件时只迁移分频，保留日期/时间/备份标记。 */
    if (RTC->PRER != WATCH_RTC_SUBSECOND_HZ - 1)
    {
        /* 改 PRER 前先关闭 RTC 寄存器写保护 */
        __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
        /* 进入初始化模式，日历计数暂停后才能安全改分频 */
        status = RTC_EnterInitMode(&watch_rtc);
        /* 只有成功进入初始化模式才写分频值 */
        if (status == HAL_OK)
        {
            /* 同步分频写为 32767，得到 32768Hz 的亚秒计数 */
            RTC->PRER = WATCH_RTC_SUBSECOND_HZ - 1;
            /* 退出初始化模式，日历恢复计数 */
            status = RTC_ExitInitMode(&watch_rtc);
        }
        /* 无论是否进入过初始化模式，都恢复写保护 */
        __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
        /* 进入/退出初始化模式失败时返回错误，避免带着旧分频继续运行 */
        if (status != HAL_OK)
        {
            return status;
        }
    }
    /* 读备份寄存器 DR0，判断这是首次上电还是已有走时/已校时 */
    marker = HAL_RTCEx_BKUPRead(&watch_rtc, RTC_BKP_DR0);
    /* 备份标记为“已初始化”或“已校时”时跳过写时间，保留备份域里已有的走时 */
    if (marker != WATCH_RTC_INITIALIZED && marker != WATCH_RTC_SYNCED)
    {
        /* 手机发送时间，日期为内部日历占位。首次初始化，后续复位保留走时。 */
        date.Year = 0;
        /* 月份用 1 月占位，真正的日期由上层忽略 */
        date.Month = RTC_MONTH_JANUARY;
        /* 日设为 1 号，保证日期字段合法 */
        date.Date = 1;
        /* 星期设为周六占位，仅需满足 HAL 参数范围 */
        date.WeekDay = RTC_WEEKDAY_SATURDAY;
        /* 不使用夏令时补偿 */
        time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        /* 复位 BKP 位（HAL 的 StoreOperation 语义），不保留旧夏令时状态 */
        time.StoreOperation = RTC_STOREOPERATION_RESET;
        /* 写入时间字段（全 0），后续由手机校时覆盖 */
        status = HAL_RTC_SetTime(&watch_rtc, &time, RTC_FORMAT_BIN);
        /* 写时间失败直接返回，避免留下半配置状态 */
        if (status != HAL_OK)
        {
            return status;
        }
        /* 写入占位日期，使日历寄存器组合法 */
        status = HAL_RTC_SetDate(&watch_rtc, &date, RTC_FORMAT_BIN);
        /* 写日期失败同样返回错误 */
        if (status != HAL_OK)
        {
            return status;
        }
        /* 记录“已初始化”标记，下次复位不再覆盖日期和时间 */
        HAL_RTCEx_BKUPWrite(&watch_rtc, RTC_BKP_DR0, WATCH_RTC_INITIALIZED);
    }
    /* 标记配置完成，允许上层校时 */
    rtc_configured = 1;
    /* 标记 RTC 就绪，允许读取时间和亚秒计数 */
    rtc_ready = 1;
    /* 返回 HAL_OK 表示 RTC 初始化成功 */
    return HAL_OK;
}

uint8_t watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    /* time_bcd 按 RTC_TR 的 BCD 字段排布拼出的 32bit 时间寄存器值 */
    uint32_t time_bcd;
    /* result 记录各等待步骤的结果：0=成功，1=超时或失败 */
    uint8_t result;
    /* RTC 未配置、LSE 停振或任一时间字段越界时，拒绝校时并返回 1 */
    if (rtc_configured == 0 || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0 || hours > 23 || minutes > 59 || seconds > 59)
    {
        return 1;
    }
    /* 十位放到对应 BCD 字段高位，个位放到低位：小时<<20、分<<12、秒<<4 */
    time_bcd = ((uint32_t)(hours / 10) << 20) | ((uint32_t)(hours % 10) << 16) |
               ((uint32_t)(minutes / 10) << 12) | ((uint32_t)(minutes % 10) << 8) |
               ((uint32_t)(seconds / 10) << 4) | (uint32_t)(seconds % 10);
    /* 将HAL句柄状态记为BUSY；校时由LVGL任务独占。 */
    watch_rtc.State = HAL_RTC_STATE_BUSY;
    /* 撤销就绪标志，校时完成前不对外提供时间 */
    rtc_ready = 0;
    /* 改 RTC 寄存器前关闭写保护 */
    __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
    /* 校时由LVGL独占；避免HAL的1s INITF/RSF等待跨过外部看门狗期限。 */
    RTC->ISR |= RTC_ISR_INIT;
    /* 等待 INITF：置位表示已进入初始化模式，日历计数暂停 */
    result = watch_rtc_wait_flag(RTC_ISR_INITF);
    /* 进入初始化模式成功才写入新的时间值 */
    if (result == 0)
    {
        /* 直接写 TR 寄存器，time_bcd 已按 BCD 字段排好 */
        RTC->TR = time_bcd;
        /* 清 BKP 位，与 HAL 的 StoreOperation=RESET 语义一致 */
        RTC->CR &= ~RTC_CR_BKP;
    }
    /* 无论写入是否成功都退出初始化模式，让日历恢复计数 */
    RTC->ISR &= ~RTC_ISR_INIT;
    /* 非旁路影子模式时，需清 RSF 再等其置位，确保读到新时间 */
    if (result == 0 && (RTC->CR & RTC_CR_BYPSHAD) == 0)
    {
        /* 先清 RSF，触发一次影子寄存器重新装载 */
        RTC->ISR = RTC_RSF_MASK & RTC_ISR_RESERVED_MASK;
        /* 等待 RSF 重新置位，表示日历已同步到影子寄存器 */
        result = watch_rtc_wait_flag(RTC_ISR_RSF);
    }
    /* 写入和同步都成功且 LSE 仍在振荡，才恢复就绪状态 */
    if (result == 0 && __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0)
    {
        /* HAL 句柄恢复 READY，允许后续 HAL RTC 调用 */
        watch_rtc.State = HAL_RTC_STATE_READY;
        /* 重新发布就绪标志，上层可以读取时间 */
        rtc_ready = 1;
        /* 记录“已校时”标记，重启后据此判断时间是否可信 */
        HAL_RTCEx_BKUPWrite(&watch_rtc, RTC_BKP_DR0, WATCH_RTC_SYNCED);
    }
    else
    {
        /* 任一步失败或 LSE 停振：句柄置错误态，返回 1 表示校时失败 */
        watch_rtc.State = HAL_RTC_STATE_ERROR;
        /* 统一返回 1，调用方无需区分具体失败原因 */
        result = 1;
    }
    /* 校时流程结束，恢复 RTC 寄存器写保护 */
    __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
    /* 返回 0 表示校时成功，1 表示失败 */
    return result;
}

HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time)
{
    /* rtc_time 接收 HAL 读出的时/分/秒及亚秒字段 */
    RTC_TimeTypeDef rtc_time = {0};
    /* date 接收日期，读取它才能释放影子寄存器锁存 */
    RTC_DateTypeDef date = {0};
    /* status 接收 HAL 调用的返回状态 */
    HAL_StatusTypeDef status;
    /* HAL读取寄存器不会检查LSE，停振时不得把冻结的时间当作有效值。 */
    if (!rtc_ready || time == NULL || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0)
    {
        return HAL_ERROR;
    }
    /* 读当前时间；Format=BIN 时 HAL 把 TR 中的 BCD 字段转成二进制填入 rtc_time */
    status = HAL_RTC_GetTime(&watch_rtc, &rtc_time, RTC_FORMAT_BIN);
    /* 读时间失败直接返回 HAL 状态 */
    if (status != HAL_OK)
    {
        return status;
    }
    /* 读取日期才能释放RTC影子寄存器锁存，否则时间可能停在上一份快照。 */
    status = HAL_RTC_GetDate(&watch_rtc, &date, RTC_FORMAT_BIN);
    /* 读日期失败同样返回，避免把未同步的数据交给上层 */
    if (status != HAL_OK)
    {
        return status;
    }
    /* 两次读取之间 LSE 若停振，读到的值不可信，返回错误 */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0)
    {
        return HAL_ERROR;
    }
    /* 把 HAL 转好的小时值（二进制，0~23）复制到调用方结构体 */
    time->hours = rtc_time.Hours;
    /* 复制分钟字段 */
    time->minutes = rtc_time.Minutes;
    /* 复制秒字段 */
    time->seconds = rtc_time.Seconds;
    /* 返回 HAL_OK 表示本次时间快照有效 */
    return HAL_OK;
}

uint8_t watch_rtc_is_synced(void)
{
    /* RTC 就绪且备份标记为“已校时”才返回 1，否则返回 0 */
    return rtc_ready && HAL_RTCEx_BKUPRead(&watch_rtc, RTC_BKP_DR0) == WATCH_RTC_SYNCED;
}

void watch_rtc_get_status(watch_rtc_status_struct *status)
{
    /* 时间字段先清零，读取失败时保持 0 而不是残留随机值 */
    status->time.hours = 0;
    /* 分钟同样先清零 */
    status->time.minutes = 0;
    /* 秒同样先清零 */
    status->time.seconds = 0;
    /* 直接读备份寄存器 DR0，得到当前初始化/校时标记 */
    status->marker = RTC->BKP0R;
    /* 复制软件就绪标志，供上层判断能否取时间 */
    status->ready = rtc_ready;
    /* 查询 LSE 就绪位，1 表示 32768Hz 时钟在振荡 */
    status->lse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0;
    /* 就绪且标记为“已校时”才算时间可信 */
    status->synced = rtc_ready && status->marker == WATCH_RTC_SYNCED;
    /* 最后尝试读一次时间，结果状态一并写入，失败时 time 字段保持 0 */
    status->read_status = watch_rtc_get_time(&status->time);
}

/* 一次HAL时间/日期快照合成一分钟内的递增32768Hz计数：秒×32768 + 本秒已过亚秒。
 * 单次HAL_RTC_GetTime/GetDate在同一影子锁存内取秒与亚秒，避免两次独立读取跨秒后拼出不存在的相位。
 * 未就绪、LSE停振或读取失败返回UINT32_MAX。 */
static uint32_t watch_rtc_read_counter(void)
{
    /* time 接收 HAL 读出的时间与亚秒计数 */
    RTC_TimeTypeDef time = {0};
    /* date 接收日期，仅用于释放影子寄存器锁存 */
    RTC_DateTypeDef date = {0};
    /* 返回一分钟内的递增计数；SubSeconds 是递减值，需换算成本秒已过计数。 */
    uint32_t counter;
    /* RTC 未就绪或 LSE 停振时无法给出有效计数，返回 UINT32_MAX */
    if (rtc_ready == 0 || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0)
    {
        return UINT32_MAX;
    }
    /* 读取时间寄存器；失败时同样返回 UINT32_MAX 表示无效 */
    if (HAL_RTC_GetTime(&watch_rtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    {
        return UINT32_MAX;
    }
    /* 与表盘读取一样，必须读DR释放影子寄存器锁存；此处不等待硬件或调用HAL_Delay。 */
    if (HAL_RTC_GetDate(&watch_rtc, &date, RTC_FORMAT_BIN) != HAL_OK || time.SubSeconds >= WATCH_RTC_SUBSECOND_HZ)
    {
        return UINT32_MAX;
    }
    /* 秒字段超出日历范围说明快照不可信，不能参与跨秒/跨分差值。 */
    if (time.Seconds >= 60)
    {
        return UINT32_MAX;
    }
    /* 秒×32768 得到整秒计数，再加本秒已经过的亚秒计数，得到0~一分钟模数内的递增相位。 */
    counter = (uint32_t)time.Seconds * WATCH_RTC_SUBSECOND_HZ + (WATCH_RTC_SUBSECOND_HZ - 1 - time.SubSeconds);
    /* 返回值始终小于WATCH_RTC_COUNTER_MOD，tickless按该模数做无符号差值。 */
    return counter;
}

uint32_t watch_rtc_get_counter(void)
{
    /* 一分钟内递增计数：跨秒、跨分和午夜由调用方按WATCH_RTC_COUNTER_MOD取模。 */
    return watch_rtc_read_counter();
}

uint32_t watch_rtc_get_subsecond(void)
{
    /* counter 保存一分钟内的递增计数；无效值原样返回。 */
    uint32_t counter = watch_rtc_read_counter();

    /* 未就绪或读取失败保持原UINT32_MAX无效值语义。 */
    if (counter == UINT32_MAX)
    {
        return UINT32_MAX;
    }
    /* 只保留本秒内的相位，维持原接口0~32767语义。 */
    return counter % WATCH_RTC_SUBSECOND_HZ;
}

uint8_t watch_rtc_start_wakeup(uint32_t subsecond_counts)
{
    /* WUT 计数以 16 个 RTC 时钟为一档，先把亚秒计数换算成档数 */
    uint32_t wakeup_cycles = subsecond_counts / 16;

    /* RTC 未就绪、LSE 停振或档数不在本函数允许的 2~65536 范围时返回 1 */
    if (rtc_ready == 0 || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0 || wakeup_cycles < 2 || wakeup_cycles > 65536)
    {
        return 1;
    }
    /* 先停掉可能仍在运行的唤醒定时器并清标志，避免旧配置干扰 */
    watch_rtc_stop_wakeup();
    /* 等 WUTWF 置位，表示唤醒定时器寄存器可写 */
    if (watch_rtc_wait_flag(RTC_ISR_WUTWF) != 0)
    {
        return 1;
    }
    /* 设置 RTC 唤醒中断优先级为最低（15），不抢占其他中断 */
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 15, 0);
    /* 使能 RTC 唤醒中断，允许定时器到期后进入 ISR */
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    /* WUT周期为(WUTR+1)*16/LSE；再提前一档，留出同步和唤醒开销。 */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&watch_rtc, wakeup_cycles - 2, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
    {
        /* 启动失败时撤销刚配置的定时器和中断，避免留下半配置状态 */
        watch_rtc_stop_wakeup();
        /* 返回 1 表示唤醒定时器未启动成功 */
        return 1;
    }
    /* 返回 0 表示唤醒定时器已按目标时间启动 */
    return 0;
}

void watch_rtc_stop_wakeup(void)
{
    /* 修改 CR 前关闭 RTC 寄存器写保护 */
    __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
    /* 同时关掉唤醒定时器使能位和中断使能位：停止计数且不再产生 WUT 中断 */
    RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
    /* 清唤醒定时器标志，防止旧的到期标志残留 */
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&watch_rtc, RTC_FLAG_WUTF);
    /* 恢复 RTC 寄存器写保护 */
    __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
    /* 关闭 RTC 唤醒的 EXTI 中断线，避免从 EXTI 侧再次触发 */
    __HAL_RTC_WAKEUPTIMER_EXTI_DISABLE_IT();
    /* 清 EXTI 挂起标志 */
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
    /* 清 NVIC 中可能已挂起的 RTC 唤醒中断 */
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
}

uint8_t watch_rtc_sync_after_stop(void)
{
    /* result 保存等待影子寄存器同步的结果：0=成功，1=超时 */
    uint8_t result;

    /* 清 RSF 前需要能写 RTC 寄存器，先关写保护 */
    __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
    /* STOP期间影子寄存器冻结，醒后必须重新同步才能用于补时。 */
    RTC->ISR = RTC_RSF_MASK & RTC_ISR_RESERVED_MASK;
    /* 等待 RSF 重新置位，表示日历已重新装载到影子寄存器 */
    result = watch_rtc_wait_flag(RTC_ISR_RSF);
    /* 同步操作结束，恢复写保护 */
    __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
    /* 同步失败说明时间快照不可信，撤销就绪标志 */
    if (result != 0)
    {
        rtc_ready = 0;
    }
    /* 返回 0 表示同步成功，1 表示超时或 LSE 异常 */
    return result;
}
