#include "watch_rtc.h"
#include "app_clock.h"
#include "app_weather.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t rtc_seconds, backup;
RTC_TypeDef host_rtc;
static uint32_t rtc_subsecond, prescaler_changes;
static uint32_t tick, lse_ready;
uint32_t HAL_GetTick(void)
{
    return tick;
}
void mock_lse_config(uint32_t value)
{
    assert(value == RCC_LSE_ON);
}
uint32_t mock_lse_flag(uint32_t flag)
{
    assert(flag == RCC_FLAG_LSERDY);
    return lse_ready;
}
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
    assert(rtc->Init.AsynchPrediv == 0 && rtc->Init.SynchPrediv == 32767);
    if (host_rtc.PRER == 0)
    {
        host_rtc.PRER = rtc->Init.SynchPrediv;
    }
    return HAL_OK;
}
HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *rtc)
{
    assert(rtc->Instance == RTC);
    ++prescaler_changes;
    return HAL_OK;
}
HAL_StatusTypeDef RTC_ExitInitMode(RTC_HandleTypeDef *rtc)
{
    assert(rtc->Instance == RTC && host_rtc.PRER == 32767U);
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
    time->SubSeconds = 32767U - rtc_subsecond;
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
int main(int argc, char **argv)
{
    watch_time_struct time;
    uint32_t calls, seconds;
    static const uint8_t nul_frame[] = {'1', '2', ':', '3', '0', 0, '\n'};
    assert(!watch_rtc_is_synced());
    assert(watch_rtc_get_time(&time) == HAL_ERROR);
    assert(watch_rtc_get_subsecond() == UINT32_MAX);
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
    host_rtc.PRER = (127U << 16) | 255U; /* 模拟旧固件已经初始化过的备份域。 */
    assert(watch_rtc_init() == HAL_OK && rtc_seconds == seconds && set_calls == calls && date_calls == 1);
    assert(prescaler_changes == 1U && host_rtc.PRER == 32767U);
    lse_ready = 1;
    rtc_subsecond = 12345U;
    assert(watch_rtc_get_subsecond() == 12345U && !shadow_locked);
    rtc_subsecond = 32767U;
    assert(watch_rtc_get_subsecond() == 32767U && !shadow_locked);
    rtc_subsecond = 0;
    assert(watch_rtc_get_subsecond() == 0 && !shadow_locked);
    lse_ready = 0;
    assert(watch_rtc_get_subsecond() == UINT32_MAX);
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
    /* 含秒消息在每个UART分包位置均能完整解析，不在消息结束前写RTC。 */
    for (uint16_t split = 0; split <= 8; ++split)
    {
        static const uint8_t frame[] = "14:35:47\n";
        calls = set_calls;
        app_clock_receive(frame, split, 2000);
        assert(set_calls == calls);
        app_clock_receive(frame + split, sizeof(frame) - 1 - split, 2010);
        assert(set_calls == calls + 1 && rtc_seconds == 14U * 3600U + 35U * 60U + 47U);
    }
    calls = set_calls;
    seconds = rtc_seconds;
    {
        static const char *invalid[] = {"12:34:60\n", "24:00:00\n", "12:34:\n", "12::34\n",
                                        "12:34:56:7\n", "12:34:005\n", "123:1:2\n"};
        for (uint8_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
        {
            feed(invalid[i], 2100);
            assert(app_clock_get_result() == APP_CLOCK_INVALID && set_calls == calls && rtc_seconds == seconds);
        }
    }
    assert(watch_rtc_set_time(12, 34, 60) == HAL_ERROR && set_calls == calls);
    set_status = HAL_ERROR;
    feed("18:20:59\n", 2200);
    assert(app_clock_get_result() == APP_CLOCK_RTC_ERROR && set_calls == calls && rtc_seconds == seconds);
    set_status = HAL_OK;
    feed("9:5:7\r\n", 2300);
    assert(rtc_seconds == 9U * 3600U + 5U * 60U + 7U);
    feed("23:59:59\n", 2400);
    rtc_seconds = (rtc_seconds + 1) % 86400;
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 0 && time.minutes == 0 && time.seconds == 0);
    feed("01:02:03", UINT32_MAX - 100U);
    app_clock_service(99U);
    assert(app_clock_get_result() == APP_CLOCK_SYNCED && rtc_seconds == 3723);
    oscillator_status = HAL_TIMEOUT;
    assert(watch_rtc_init() == HAL_TIMEOUT && watch_rtc_get_time(&time) == HAL_ERROR);
    assert(!watch_rtc_is_synced());
    oscillator_status = HAL_OK;
    assert(watch_rtc_init() == HAL_OK && watch_rtc_is_synced() && rtc_seconds == 3723);
    puts("PASS: LSE/RTC configuration, backup retention, shadow unlock, fragmented HH:MM:SS and legacy HH:MM, no-newline idle, ranges/noise/failures, midnight "
         "and tick wrap.");
    calls = set_calls;
    tick = UINT32_MAX - 1000U;
    watch_rtc_init_begin();
    assert(watch_rtc_init_is_pending());
    tick += 4999U;
    assert(watch_rtc_init_service() == HAL_BUSY && watch_rtc_init_is_pending());
    ++tick;
    assert(watch_rtc_init_service() == HAL_TIMEOUT && !watch_rtc_init_is_pending());
    assert(set_calls == calls);
    lse_ready = 1;
    watch_rtc_init_begin();
    assert(watch_rtc_init_service() == HAL_OK && !watch_rtc_init_is_pending());
    assert(watch_rtc_is_synced() && rtc_seconds == 3723 && set_calls == calls);
    oscillator_status = HAL_ERROR;
    watch_rtc_init_begin();
    assert(watch_rtc_init_service() == HAL_ERROR && !watch_rtc_init_is_pending());
    oscillator_status = HAL_OK;
    watch_rtc_init_begin();
    assert(watch_rtc_init_service() == HAL_OK);
    puts("PASS: staged LSE wait, timeout/tick wrap, retained time and final initialization failures.");
    if (argc > 1)
    {
        char frames[128];
        app_weather_struct weather;
        FILE *file = fopen(argv[1], "rb");
        size_t length;
        assert(file != NULL);
        length = fread(frames, 1, sizeof(frames), file);
        fclose(file);
        app_clock_receive((const uint8_t *)frames, (uint16_t)length, 1000);
        app_weather_receive((const uint8_t *)frames, (uint16_t)length, 1000);
        app_weather_get_snapshot(&weather);
        assert(rtc_seconds == 14U * 3600U + 35U * 60U + 47U && app_clock_get_result() == APP_CLOCK_SYNCED);
        assert(weather.valid && weather.code == 3 && weather.temperature_x10 == -53 && weather.humidity_rh == 81);
        puts("PASS: real Java-generated phone frames -> C clock/weather parsers -> RTC and weather snapshot.");
    }
    return 0;
}
