#include "watch_rtc.h"
#include <stddef.h>

#define WATCH_RTC_INITIALIZED 0x52544330U
#define WATCH_RTC_SYNCED      0x52544331U

static RTC_HandleTypeDef watch_rtc;
static uint8_t rtc_ready;
static uint32_t rtc_start_ms;
static uint8_t rtc_start_pending;
static HAL_StatusTypeDef rtc_start_status = HAL_ERROR;

void watch_rtc_init_begin(void)
{
    rtc_ready = 0;
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
    watch_rtc.Init.SynchPrediv = WATCH_RTC_SUBSECOND_HZ - 1U;
    watch_rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    watch_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    watch_rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    status = HAL_RTC_Init(&watch_rtc);
    if (status != HAL_OK)
    {
        return status;
    }
    /* HAL_RTC_Init会跳过已初始化的备份域；升级旧固件时只迁移分频，保留日期/时间/备份标记。 */
    if (RTC->PRER != WATCH_RTC_SUBSECOND_HZ - 1U)
    {
        __HAL_RTC_WRITEPROTECTION_DISABLE(&watch_rtc);
        status = RTC_EnterInitMode(&watch_rtc);
        if (status == HAL_OK)
        {
            RTC->PRER = WATCH_RTC_SUBSECOND_HZ - 1U;
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
    rtc_ready = 1;
    return HAL_OK;
}

HAL_StatusTypeDef watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    RTC_TimeTypeDef time = {0};
    HAL_StatusTypeDef status;
    if (!rtc_ready || hours > 23 || minutes > 59 || seconds > 59)
    {
        return HAL_ERROR;
    }
    time.Hours = hours;
    time.Minutes = minutes;
    time.Seconds = seconds;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;
    status = HAL_RTC_SetTime(&watch_rtc, &time, RTC_FORMAT_BIN);
    if (status == HAL_OK)
    {
        HAL_RTCEx_BKUPWrite(&watch_rtc, RTC_BKP_DR0, WATCH_RTC_SYNCED);
    }
    return status;
}

HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time)
{
    RTC_TimeTypeDef rtc_time = {0};
    RTC_DateTypeDef date = {0};
    HAL_StatusTypeDef status;
    if (!rtc_ready || time == NULL)
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
    time->hours = rtc_time.Hours;
    time->minutes = rtc_time.Minutes;
    time->seconds = rtc_time.Seconds;
    return HAL_OK;
}

uint8_t watch_rtc_is_synced(void)
{
    return rtc_ready && HAL_RTCEx_BKUPRead(&watch_rtc, RTC_BKP_DR0) == WATCH_RTC_SYNCED;
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
    return WATCH_RTC_SUBSECOND_HZ - 1U - time.SubSeconds;
}
