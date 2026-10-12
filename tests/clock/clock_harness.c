#include "watch_rtc.h"
#include "app_clock.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t rtc_seconds, backup;
static uint8_t power_enabled, backup_enabled, rtc_enabled, shadow_locked;
static uint32_t set_calls, date_calls;
static HAL_StatusTypeDef oscillator_status = HAL_OK;
static HAL_StatusTypeDef set_status = HAL_OK;

void mock_power_enable(void)
{
    power_enabled = 1;
}
void HAL_PWR_EnableBkUpAccess(void)
{
    assert(power_enabled);
    backup_enabled = 1;
}
void mock_rtc_enable(void)
{
    rtc_enabled = 1;
}
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *oscillator)
{
    assert(backup_enabled && oscillator->OscillatorType == RCC_OSCILLATORTYPE_LSE);
    assert(oscillator->LSEState == RCC_LSE_ON && oscillator->PLL.PLLState == RCC_PLL_NONE);
    return oscillator_status;
}
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *clock)
{
    assert(clock->PeriphClockSelection == RCC_PERIPHCLK_RTC && clock->RTCClockSelection == RCC_RTCCLKSOURCE_LSE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *rtc)
{
    assert(rtc_enabled && rtc->Instance == RTC && rtc->Init.HourFormat == RTC_HOURFORMAT_24);
    assert(rtc->Init.AsynchPrediv == 127 && rtc->Init.SynchPrediv == 255);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *rtc, RTC_TimeTypeDef *time, uint32_t format)
{
    assert(rtc->Instance == RTC && format == RTC_FORMAT_BIN);
    if (set_status != HAL_OK)
    {
        return set_status;
    }
    assert(time->Hours < 24 && time->Minutes < 60 && time->Seconds < 60);
    rtc_seconds = time->Hours * 3600U + time->Minutes * 60U + time->Seconds;
    ++set_calls;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *rtc, RTC_DateTypeDef *date, uint32_t format)
{
    assert(rtc->Instance == RTC && format == RTC_FORMAT_BIN);
    assert(date->Year == 0 && date->Month == 1 && date->Date == 1 && date->WeekDay == RTC_WEEKDAY_SATURDAY);
    ++date_calls;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *rtc, RTC_TimeTypeDef *time, uint32_t format)
{
    assert(rtc->Instance == RTC && format == RTC_FORMAT_BIN && !shadow_locked);
    shadow_locked = 1;
    time->Hours = rtc_seconds / 3600;
    time->Minutes = rtc_seconds / 60 % 60;
    time->Seconds = rtc_seconds % 60;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *rtc, RTC_DateTypeDef *date, uint32_t format)
{
    assert(rtc->Instance == RTC && format == RTC_FORMAT_BIN && shadow_locked);
    shadow_locked = 0;
    memset(date, 0, sizeof(*date));
    return HAL_OK;
}
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *rtc, uint32_t reg)
{
    assert(rtc->Instance == RTC && reg == RTC_BKP_DR0 && backup_enabled && rtc_enabled);
    return backup;
}
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *rtc, uint32_t reg, uint32_t value)
{
    assert(rtc->Instance == RTC && reg == RTC_BKP_DR0 && backup_enabled);
    backup = value;
}
static void feed(const char *text, uint32_t ms)
{
    app_clock_receive((const uint8_t *)text, (uint16_t)strlen(text), ms);
}
int main(void)
{
    watch_time_struct time;
    uint32_t calls, seconds;
    static const uint8_t nul_frame[] = {'1', '2', ':', '3', '0', 0, '\n'};
    assert(!watch_rtc_is_synced());
    assert(watch_rtc_get_time(&time) == HAL_ERROR);
    assert(watch_rtc_init() == HAL_OK && !watch_rtc_is_synced());
    assert(set_calls == 1 && date_calls == 1);
    calls = set_calls;
    feed("14:", 100);
    app_clock_service(250);
    assert(set_calls == calls);
    feed("35", 260);
    app_clock_service(459);
    assert(set_calls == calls);
    app_clock_service(460);
    assert(app_clock_get_result() == APP_CLOCK_SYNCED && watch_rtc_is_synced());
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 14 && time.minutes == 35 && time.seconds == 0);
    rtc_seconds += 61;
    assert(watch_rtc_get_time(&time) == HAL_OK && time.minutes == 36 && time.seconds == 1);
    assert(watch_rtc_get_time(&time) == HAL_OK); /* 连续读取验证GetDate已释放影子锁存 */
    calls = set_calls;
    seconds = rtc_seconds;
    assert(watch_rtc_init() == HAL_OK && rtc_seconds == seconds && set_calls == calls && date_calls == 1);
    assert(watch_rtc_is_synced());
    feed("24:00\n", 500);
    assert(app_clock_get_result() == APP_CLOCK_INVALID && rtc_seconds == seconds);
    feed("12:60\r\n", 600);
    assert(app_clock_get_result() == APP_CLOCK_INVALID && set_calls == calls);
    feed("12:34garbage\n", 700);
    assert(app_clock_get_result() == APP_CLOCK_INVALID && set_calls == calls);
    feed("1::2\n", 800);
    assert(app_clock_get_result() == APP_CLOCK_INVALID && set_calls == calls);
    app_clock_receive(nul_frame, sizeof(nul_frame), 900);
    assert(app_clock_get_result() == APP_CLOCK_INVALID && set_calls == calls);
    feed("hello from phone\n", 1000);
    assert(set_calls == calls);
    feed("9:5\r\n", 1100);
    assert(rtc_seconds == 9U * 3600 + 5U * 60);
    feed("23:59\n", 1200);
    rtc_seconds = (rtc_seconds + 60) % 86400;
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 0 && time.minutes == 0);
    feed("00:00\n12:30\n", 1300);
    assert(rtc_seconds == 12U * 3600 + 30U * 60);
    calls = set_calls;
    seconds = rtc_seconds;
    set_status = HAL_ERROR;
    feed("18:20\n", 1400);
    assert(app_clock_get_result() == APP_CLOCK_RTC_ERROR && set_calls == calls && rtc_seconds == seconds);
    set_status = HAL_OK;
    feed("01:02", UINT32_MAX - 100U);
    app_clock_service(99U);
    assert(app_clock_get_result() == APP_CLOCK_SYNCED && rtc_seconds == 3720);
    oscillator_status = HAL_TIMEOUT;
    assert(watch_rtc_init() == HAL_TIMEOUT && watch_rtc_get_time(&time) == HAL_ERROR);
    assert(!watch_rtc_is_synced());
    oscillator_status = HAL_OK;
    assert(watch_rtc_init() == HAL_OK && watch_rtc_is_synced() && rtc_seconds == 3720);
    puts("PASS: LSE/RTC configuration, backup retention, shadow unlock, fragmented HH:MM, no-newline idle, ranges/noise/failures, midnight "
         "and tick wrap.");
    return 0;
}
