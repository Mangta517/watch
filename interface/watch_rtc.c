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
    uint32_t started, remaining = SystemCoreClock / 1000;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    started = DWT->CYCCNT;
    while ((RTC->ISR & flag) == 0)
    {
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0 ||
            (uint32_t)(DWT->CYCCNT - started) >= SystemCoreClock / 1000 || remaining == 0)
        {
            return 1;
        }
        --remaining;
        __NOP();
    }
    return __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0;
}

void watch_rtc_init_begin(void)
{
    rtc_ready = 0;
    rtc_configured = 0;
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
    rtc_start_ms = HAL_GetTick();
    rtc_start_pending = 1;
    rtc_start_status = HAL_BUSY;
}

HAL_StatusTypeDef watch_rtc_init_service(void)
{
    if (rtc_start_pending == 0)
    {
        return rtc_start_status;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0)
    {
        rtc_start_status = watch_rtc_init();
        rtc_start_pending = 0;
    }
    else if ((uint32_t)(HAL_GetTick() - rtc_start_ms) >= LSE_STARTUP_TIMEOUT)
    {
        rtc_start_status = HAL_TIMEOUT;
        rtc_start_pending = 0;
    }
    return rtc_start_status;
}

uint8_t watch_rtc_init_is_pending(void)
{
    return rtc_start_pending;
}

HAL_StatusTypeDef watch_rtc_init(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    HAL_StatusTypeDef status;
    uint32_t marker;

    rtc_ready = 0;
    rtc_configured = 0;
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    oscillator.LSEState = RCC_LSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    status = HAL_RCC_OscConfig(&oscillator);
    if (status != HAL_OK)
    {
        return status;
    }
    clock.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    clock.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    status = HAL_RCCEx_PeriphCLKConfig(&clock);
    if (status != HAL_OK)
    {
        return status;
    }
    __HAL_RCC_RTC_ENABLE();
    watch_rtc.Instance = RTC;
    watch_rtc.Init.HourFormat = RTC_HOURFORMAT_24;
    watch_rtc.Init.AsynchPrediv = 0;
    watch_rtc.Init.SynchPrediv = WATCH_RTC_SUBSECOND_HZ - 1;
    watch_rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    watch_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    watch_rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    status = HAL_RTC_Init(&watch_rtc);
    if (status != HAL_OK)
    {
        return status;
    }
    /* HAL_RTC_Init会跳过已初始化的备份域；升级旧固件时只迁移分频，保留日期/时间/备份标记。 */
    if (RTC->PRER != WATCH_RTC_SUBSECOND_HZ - 1)
    {
        __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
        status = RTC_EnterInitMode(&watch_rtc);
        if (status == HAL_OK)
        {
            RTC->PRER = WATCH_RTC_SUBSECOND_HZ - 1;
            status = RTC_ExitInitMode(&watch_rtc);
        }
        __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    marker = HAL_RTCEx_BKUPRead(&watch_rtc, RTC_BKP_DR0);
    if (marker != WATCH_RTC_INITIALIZED && marker != WATCH_RTC_SYNCED)
    {
        /* 手机发送时间，日期为内部日历占位。首次初始化，后续复位保留走时。 */
        date.Year = 0;
        date.Month = RTC_MONTH_JANUARY;
        date.Date = 1;
        date.WeekDay = RTC_WEEKDAY_SATURDAY;
        time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        time.StoreOperation = RTC_STOREOPERATION_RESET;
        status = HAL_RTC_SetTime(&watch_rtc, &time, RTC_FORMAT_BIN);
        if (status != HAL_OK)
        {
            return status;
        }
        status = HAL_RTC_SetDate(&watch_rtc, &date, RTC_FORMAT_BIN);
        if (status != HAL_OK)
        {
            return status;
        }
        HAL_RTCEx_BKUPWrite(&watch_rtc, RTC_BKP_DR0, WATCH_RTC_INITIALIZED);
    }
    rtc_configured = 1;
    rtc_ready = 1;
    return HAL_OK;
}

uint8_t watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    uint32_t time_bcd;
    uint8_t result;
    if (rtc_configured == 0 || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0 || hours > 23 || minutes > 59 || seconds > 59)
    {
        return 1;
    }
    time_bcd = ((uint32_t)(hours / 10) << 20) | ((uint32_t)(hours % 10) << 16) |
               ((uint32_t)(minutes / 10) << 12) | ((uint32_t)(minutes % 10) << 8) |
               ((uint32_t)(seconds / 10) << 4) | (uint32_t)(seconds % 10);
    watch_rtc.State = HAL_RTC_STATE_BUSY;
    rtc_ready = 0;
    __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
    /* 校时由LVGL独占；避免HAL的1s INITF/RSF等待跨过外部看门狗期限。 */
    RTC->ISR |= RTC_ISR_INIT;
    result = watch_rtc_wait_flag(RTC_ISR_INITF);
    if (result == 0)
    {
        RTC->TR = time_bcd;
        RTC->CR &= ~RTC_CR_BKP;
    }
    RTC->ISR &= ~RTC_ISR_INIT;
    if (result == 0 && (RTC->CR & RTC_CR_BYPSHAD) == 0)
    {
        RTC->ISR = RTC_RSF_MASK & RTC_ISR_RESERVED_MASK;
        result = watch_rtc_wait_flag(RTC_ISR_RSF);
    }
    if (result == 0 && __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0)
    {
        watch_rtc.State = HAL_RTC_STATE_READY;
        rtc_ready = 1;
        HAL_RTCEx_BKUPWrite(&watch_rtc, RTC_BKP_DR0, WATCH_RTC_SYNCED);
    }
    else
    {
        watch_rtc.State = HAL_RTC_STATE_ERROR;
        result = 1;
    }
    __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
    return result;
}

HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time)
{
    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef date = {0};
    HAL_StatusTypeDef status;
    /* HAL读取寄存器不会检查LSE，停振时不得把冻结的时间当作有效值。 */
    if (!rtc_ready || time == NULL || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0)
    {
        return HAL_ERROR;
    }
    status = HAL_RTC_GetTime(&watch_rtc, &rtc_time, RTC_FORMAT_BIN);
    if (status != HAL_OK)
    {
        return status;
    }
    /* 读取日期才能释放RTC影子寄存器锁存，否则时间可能停在上一份快照。 */
    status = HAL_RTC_GetDate(&watch_rtc, &date, RTC_FORMAT_BIN);
    if (status != HAL_OK)
    {
        return status;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0)
    {
        return HAL_ERROR;
    }
    time->hours = rtc_time.Hours;
    time->minutes = rtc_time.Minutes;
    time->seconds = rtc_time.Seconds;
    return HAL_OK;
}

uint8_t watch_rtc_is_synced(void)
{
    return rtc_ready && HAL_RTCEx_BKUPRead(&watch_rtc, RTC_BKP_DR0) == WATCH_RTC_SYNCED;
}

void watch_rtc_get_status(watch_rtc_status_struct *status)
{
    status->time.hours = 0;
    status->time.minutes = 0;
    status->time.seconds = 0;
    status->marker = RTC->BKP0R;
    status->ready = rtc_ready;
    status->lse_ready = __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0;
    status->synced = rtc_ready && status->marker == WATCH_RTC_SYNCED;
    status->read_status = watch_rtc_get_time(&status->time);
}

uint32_t watch_rtc_get_subsecond(void)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    if (rtc_ready == 0 || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0)
    {
        return UINT32_MAX;
    }
    if (HAL_RTC_GetTime(&watch_rtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    {
        return UINT32_MAX;
    }
    /* 与表盘读取一样，必须读DR释放影子寄存器锁存；此处不等待硬件或调用HAL_Delay。 */
    if (HAL_RTC_GetDate(&watch_rtc, &date, RTC_FORMAT_BIN) != HAL_OK || time.SubSeconds >= WATCH_RTC_SUBSECOND_HZ)
    {
        return UINT32_MAX;
    }
    return WATCH_RTC_SUBSECOND_HZ - 1 - time.SubSeconds;
}

uint8_t watch_rtc_start_wakeup(uint32_t subsecond_counts)
{
    uint32_t wakeup_cycles = subsecond_counts / 16;

    if (rtc_ready == 0 || __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0 || wakeup_cycles < 2 || wakeup_cycles > 65536)
    {
        return 1;
    }
    watch_rtc_stop_wakeup();
    if (watch_rtc_wait_flag(RTC_ISR_WUTWF) != 0)
    {
        return 1;
    }
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    /* WUT周期为(WUTR+1)*16/LSE；再提前一档，留出同步和唤醒开销。 */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&watch_rtc, wakeup_cycles - 2, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
    {
        watch_rtc_stop_wakeup();
        return 1;
    }
    return 0;
}

void watch_rtc_stop_wakeup(void)
{
    __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
    RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&watch_rtc, RTC_FLAG_WUTF);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
    __HAL_RTC_WAKEUPTIMER_EXTI_DISABLE_IT();
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
    HAL_NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
}

uint8_t watch_rtc_sync_after_stop(void)
{
    uint8_t result;

    __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
    /* STOP期间影子寄存器冻结，醒后必须重新同步才能用于补时。 */
    RTC->ISR = RTC_RSF_MASK & RTC_ISR_RESERVED_MASK;
    result = watch_rtc_wait_flag(RTC_ISR_RSF);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&watch_rtc);
    if (result != 0)
    {
        rtc_ready = 0;
    }
    return result;
}
