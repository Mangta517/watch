#include "watch_rtc.h"
#include "app_clock.h"
#include "app_weather.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t rtc_seconds;
RTC_TypeDef host_rtc;
Host_Cycle_Type host_cycles;
uint32_t SystemCoreClock = 64000000;
#define backup host_rtc.BKP0R
static uint32_t rtc_subsecond, prescaler_changes;
static uint32_t tick, lse_ready, tick_reads;
static uint32_t wakeup_calls, wakeup_counter, nop_calls, async_flag, async_after;
static uint8_t wakeup_irq_enabled, wakeup_exti_enabled, wakeup_exti_pending, wakeup_irq_pending, cycles_frozen;
static uint8_t runtime_set_active, rtc_write_protected = 1, set_init_stuck, set_sync_stuck;
static uint32_t set_phase_nops, set_elapsed_nops, set_drop_lse_after;
static HAL_StatusTypeDef wakeup_status = HAL_OK;
uint32_t HAL_GetTick(void)
{
    ++tick_reads;
    return tick;
}
void host_nop(void)
{
    ++nop_calls;
    if (cycles_frozen == 0)
    {
        DWT->CYCCNT += 160;
    }
    if (runtime_set_active != 0)
    {
        ++set_elapsed_nops;
        if (set_drop_lse_after != 0 && set_elapsed_nops >= set_drop_lse_after)
        {
            lse_ready = 0;
        }
        if ((RTC->ISR & RTC_ISR_INIT) != 0)
        {
            if (lse_ready != 0 && set_init_stuck == 0 && ++set_phase_nops >= 2)
            {
                RTC->ISR |= RTC_ISR_INITF;
                set_phase_nops = 0;
            }
        }
        else
        {
            RTC->ISR &= ~RTC_ISR_INITF;
            if (lse_ready != 0 && set_sync_stuck == 0 && ++set_phase_nops >= 3)
            {
                RTC->ISR |= RTC_ISR_RSF;
            }
        }
        return;
    }
    if (async_after != 0 && nop_calls >= async_after)
    {
        RTC->ISR |= async_flag;
    }
}
void host_wakeup_exti_disable(void)
{
    wakeup_exti_enabled = 0;
}
void host_wakeup_exti_clear(void)
{
    wakeup_exti_pending = 0;
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq == RTC_WKUP_IRQn && priority == 15 && subpriority == 0);
}
void HAL_NVIC_EnableIRQ(int irq)
{
    assert(irq == RTC_WKUP_IRQn);
    wakeup_irq_enabled = 1;
}
void HAL_NVIC_ClearPendingIRQ(int irq)
{
    assert(irq == RTC_WKUP_IRQn);
    wakeup_irq_pending = 0;
}
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *rtc, uint32_t count, uint32_t clock)
{
    assert(rtc->Instance == RTC && clock == RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    assert((RTC->CR & RTC_CR_WUTE) == 0 && (RTC->ISR & RTC_ISR_WUTWF) != 0);
    assert(wakeup_irq_enabled && wakeup_irq_pending == 0 && wakeup_exti_pending == 0);
    ++wakeup_calls;
    wakeup_counter = count;
    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
    wakeup_exti_enabled = 1;
    return wakeup_status;
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
static uint32_t set_calls, date_calls, get_time_calls;
static uint32_t hal_set_calls;
static uint8_t drop_lse_on_read;
static HAL_StatusTypeDef oscillator_status = HAL_OK;
static HAL_StatusTypeDef set_status = HAL_OK;
static HAL_StatusTypeDef get_status = HAL_OK;

void host_write_protection(RTC_HandleTypeDef *rtc, uint8_t enabled)
{
    assert(rtc->Instance == RTC && enabled != rtc_write_protected);
    rtc_write_protected = enabled;
    if (enabled == 0)
    {
        runtime_set_active = rtc->State == HAL_RTC_STATE_BUSY;
        set_phase_nops = set_elapsed_nops = 0;
    }
    else if (runtime_set_active != 0)
    {
        assert((RTC->ISR & RTC_ISR_INIT) == 0);
        RTC->ISR &= ~RTC_ISR_INITF;
        if (rtc->State == HAL_RTC_STATE_READY)
        {
            uint32_t hours = ((RTC->TR >> 20) & 3) * 10 + ((RTC->TR >> 16) & 15);
            uint32_t minutes = ((RTC->TR >> 12) & 7) * 10 + ((RTC->TR >> 8) & 15);
            uint32_t seconds = ((RTC->TR >> 4) & 7) * 10 + (RTC->TR & 15);

            assert(hours < 24 && minutes < 60 && seconds < 60);
            rtc_seconds = hours * 3600 + minutes * 60 + seconds;
            rtc_subsecond = 0;
            ++set_calls;
        }
        runtime_set_active = 0;
    }
}

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
    if (oscillator_status == HAL_OK)
    {
        lse_ready = 1;
    }
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
    rtc->State = HAL_RTC_STATE_READY;
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
    ++hal_set_calls;
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
    ++get_time_calls;
    if (get_status != HAL_OK)
    {
        return get_status;
    }
    shadow_locked = 1;
    time->Hours = rtc_seconds / 3600;
    time->Minutes = rtc_seconds / 60 % 60;
    time->Seconds = rtc_seconds % 60;
    time->SubSeconds = 32767U - rtc_subsecond;
    if (drop_lse_on_read != 0)
    {
        lse_ready = 0;
    }
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

static void bounded_set_time_tests(void)
{
    watch_time_struct time;
    watch_rtc_status_struct status;
    uint32_t tick_reads_before, hal_calls_before, time_before, calls_before;
    uint32_t date_before = 0x0026A328;

    watch_rtc_init_begin();
    assert(watch_rtc_set_time(12, 34, 56) == 1);
    assert(watch_rtc_init_service() == HAL_OK);
    RTC->DR = date_before;
    RTC->ISR = RTC_ISR_WUTWF | RTC_FLAG_WUTF;
    RTC->CR = RTC_CR_WUTIE | RTC_CR_BKP;
    tick_reads_before = tick_reads;
    hal_calls_before = hal_set_calls;

    assert(watch_rtc_set_time(23, 59, 58) == 0);
    assert(RTC->TR == 0x00235958 && RTC->DR == date_before);
    assert(RTC->CR == RTC_CR_WUTIE && (RTC->ISR & RTC_FLAG_WUTF) != 0);
    assert(rtc_write_protected && set_elapsed_nops == 5);
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 23 && time.minutes == 59 && time.seconds == 58);

    backup = 0x52544330;
    time_before = RTC->TR;
    calls_before = set_calls;
    lse_ready = 0;
    nop_calls = 0;
    assert(watch_rtc_set_time(9, 8, 7) == 1);
    assert(nop_calls == 0 && RTC->TR == time_before && backup == 0x52544330 && set_calls == calls_before);
    assert(rtc_write_protected && (RTC->ISR & RTC_ISR_INIT) == 0);
    lse_ready = 1;
    assert(watch_rtc_set_time(9, 8, 7) == 0 && RTC->TR == 0x00090807 && backup == 0x52544331);

    backup = 0x52544330;
    time_before = RTC->TR;
    set_init_stuck = 1;
    nop_calls = 0;
    DWT->CYCCNT = UINT32_MAX - 160;
    assert(watch_rtc_set_time(12, 34, 56) == 1 && nop_calls <= SystemCoreClock / 1000 / 160 + 1);
    assert(RTC->TR == time_before && backup == 0x52544330 && rtc_write_protected);
    assert((RTC->ISR & RTC_ISR_INIT) == 0 && watch_rtc_get_time(&time) == HAL_ERROR);
    cycles_frozen = 1;
    nop_calls = 0;
    assert(watch_rtc_set_time(12, 34, 56) == 1 && nop_calls == SystemCoreClock / 1000);
    assert(RTC->TR == time_before && (RTC->ISR & RTC_ISR_INIT) == 0 && rtc_write_protected);
    cycles_frozen = 0;
    set_init_stuck = 0;
    assert(watch_rtc_set_time(12, 34, 56) == 0 && RTC->TR == 0x00123456);

    backup = 0x52544330;
    set_sync_stuck = 1;
    nop_calls = 0;
    assert(watch_rtc_set_time(13, 24, 35) == 1 && nop_calls <= SystemCoreClock / 1000 / 160 + 3);
    assert(RTC->TR == 0x00132435 && RTC->DR == date_before && backup == 0x52544330);
    assert((RTC->ISR & RTC_ISR_INIT) == 0 && rtc_write_protected);
    watch_rtc_get_status(&status);
    assert(status.ready == 0 && status.read_status == HAL_ERROR);
    set_sync_stuck = 0;
    assert(watch_rtc_set_time(13, 24, 35) == 0 && watch_rtc_is_synced());
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 13 && time.minutes == 24 && time.seconds == 35);

    for (uint32_t drop_after = 1; drop_after <= 3; drop_after += 2)
    {
        backup = 0x52544330;
        set_drop_lse_after = drop_after;
        nop_calls = 0;
        assert(watch_rtc_set_time(14, 25, 36) == 1 && nop_calls == drop_after);
        assert((RTC->ISR & RTC_ISR_INIT) == 0 && rtc_write_protected && backup == 0x52544330);
        assert(watch_rtc_get_time(&time) == HAL_ERROR);
        set_drop_lse_after = 0;
        lse_ready = 1;
        assert(watch_rtc_set_time(14, 25, 36) == 0);
        assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 14 && time.minutes == 25 && time.seconds == 36);
    }

    async_after = 0;
    assert(watch_rtc_sync_after_stop() == 1 && watch_rtc_get_time(&time) == HAL_ERROR);
    assert(watch_rtc_set_time(1, 2, 3) == 0 && RTC->TR == 0x00010203);
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 1 && time.minutes == 2 && time.seconds == 3);
    RTC->CR |= RTC_CR_BYPSHAD;
    nop_calls = 0;
    assert(watch_rtc_set_time(4, 5, 6) == 0 && nop_calls == 2 && RTC->TR == 0x00040506);
    assert((RTC->CR & RTC_CR_BYPSHAD) != 0 && RTC->DR == date_before && rtc_write_protected);
    RTC->CR &= ~RTC_CR_BYPSHAD;
    assert(tick_reads == tick_reads_before && hal_set_calls == hal_calls_before);
    puts("PASS: runtime time writes use bounded INITF/RSF waits without HAL setters/ticks; BCD/date/flags/protection, LSE loss, "
         "timeout/cycle wrap and frozen counter recover on the next sync, including after STOP sync failure.");
}

int main(int argc, char **argv)
{
    watch_time_struct time;
    watch_rtc_status_struct status;
    uint32_t calls, seconds, read_calls;
    static const uint8_t nul_frame[] = {'1', '2', ':', '3', '0', 0, '\n'};
    assert(!watch_rtc_is_synced());
    assert(watch_rtc_set_time(12, 34, 56) == 1 && hal_set_calls == 0 && set_calls == 0);
    assert(watch_rtc_get_time(&time) == HAL_ERROR);
    assert(watch_rtc_get_subsecond() == UINT32_MAX);
    assert(watch_rtc_start_wakeup(3276) == 1 && wakeup_calls == 0);
    backup = 0x12345678;
    watch_rtc_get_status(&status);
    assert(status.ready == 0 && status.lse_ready == 0 && status.synced == 0 && status.read_status == HAL_ERROR);
    assert(status.marker == backup && status.time.hours == 0 && set_calls == 0 && date_calls == 0);
    assert(watch_rtc_init() == HAL_OK && !watch_rtc_is_synced());
    assert(set_calls == 1 && date_calls == 1);
    {
        uint32_t tick_reads_before = tick_reads;

        RTC->ISR = RTC_ISR_WUTWF | RTC_FLAG_WUTF;
        wakeup_irq_pending = wakeup_exti_pending = 1;
        assert(watch_rtc_start_wakeup(3276) == 0 && wakeup_counter == 202);
        assert(wakeup_exti_enabled && (RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) == (RTC_CR_WUTE | RTC_CR_WUTIE));
        assert((RTC->ISR & RTC_FLAG_WUTF) == 0);
        RTC->ISR |= RTC_FLAG_WUTF;
        wakeup_irq_pending = wakeup_exti_pending = 1;
        watch_rtc_stop_wakeup();
        assert((RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) == 0 && (RTC->ISR & RTC_FLAG_WUTF) == 0);
        assert(!wakeup_exti_enabled && !wakeup_exti_pending && !wakeup_irq_pending);

        RTC->ISR = 0;
        nop_calls = 0;
        async_after = 4;
        async_flag = RTC_ISR_WUTWF;
        assert(watch_rtc_start_wakeup(164) == 0 && wakeup_counter == 8 && nop_calls == 4);
        watch_rtc_stop_wakeup();
        async_after = 0;
        wakeup_status = HAL_ERROR;
        assert(watch_rtc_start_wakeup(164) == 1);
        assert((RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) == 0 && !wakeup_exti_enabled);
        wakeup_status = HAL_OK;

        calls = wakeup_calls;
        RTC->ISR = 0;
        nop_calls = 0;
        assert(watch_rtc_start_wakeup(164) == 1 && wakeup_calls == calls && nop_calls <= SystemCoreClock / 1000 / 160 + 1);
        cycles_frozen = 1;
        nop_calls = 0;
        assert(watch_rtc_start_wakeup(164) == 1 && wakeup_calls == calls && nop_calls == SystemCoreClock / 1000);
        cycles_frozen = 0;
        assert(watch_rtc_start_wakeup(31) == 1 && watch_rtc_start_wakeup(1048592) == 1 && wakeup_calls == calls);
        lse_ready = 0;
        assert(watch_rtc_start_wakeup(164) == 1 && wakeup_calls == calls);
        lse_ready = 1;

        RTC->ISR = RTC_ISR_RSF;
        nop_calls = 0;
        async_after = 3;
        async_flag = RTC_ISR_RSF;
        DWT->CYCCNT = UINT32_MAX - 160;
        assert(watch_rtc_sync_after_stop() == 0 && nop_calls == 3);
        async_after = 0;
        nop_calls = 0;
        assert(watch_rtc_sync_after_stop() == 1 && nop_calls <= SystemCoreClock / 1000 / 160 + 1);
        assert(watch_rtc_get_subsecond() == UINT32_MAX);
        assert(watch_rtc_init() == HAL_OK);
        assert(tick_reads == tick_reads_before && set_calls == 1 && date_calls == 1);
        puts("PASS: RTC WUT arms early, clears stale/pending wakeups, rejects unavailable clocks, bounds flag waits and resynchronizes shadows without HAL ticks.");
    }
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
    watch_rtc_get_status(&status);
    assert(status.ready && status.lse_ready && status.synced && status.read_status == HAL_OK);
    assert(status.marker == 0x52544331 && status.time.hours == 14 && status.time.minutes == 36 && status.time.seconds == 1);
    assert(set_calls == calls && date_calls == 1 && !shadow_locked);
    backup = 0x52544330;
    watch_rtc_get_status(&status);
    assert(status.synced == 0 && status.read_status == HAL_OK && backup == 0x52544330);
    assert(rtc_seconds == seconds && set_calls == calls && date_calls == 1);
    backup = 0x52544331;
    get_status = HAL_ERROR;
    watch_rtc_get_status(&status);
    assert(status.synced && status.read_status == HAL_ERROR && status.time.hours == 0);
    assert(set_calls == calls && backup == 0x52544331 && !shadow_locked);
    get_status = HAL_OK;
    puts("PASS: read-only RTC status exposes raw marker, LSE, readiness and read errors without changing time or backup.");
    rtc_subsecond = 12345U;
    assert(watch_rtc_get_subsecond() == 12345U && !shadow_locked);
    rtc_subsecond = 32767U;
    assert(watch_rtc_get_subsecond() == 32767U && !shadow_locked);
    rtc_subsecond = 0;
    assert(watch_rtc_get_subsecond() == 0 && !shadow_locked);
    lse_ready = 0;
    assert(watch_rtc_get_subsecond() == UINT32_MAX);
    assert(watch_rtc_is_synced());
    read_calls = get_time_calls;
    time.hours = 1;
    time.minutes = 2;
    time.seconds = 3;
    assert(watch_rtc_get_time(&time) == HAL_ERROR);
    assert(time.hours == 1 && time.minutes == 2 && time.seconds == 3);
    watch_rtc_get_status(&status);
    assert(status.ready && !status.lse_ready && status.synced && status.read_status == HAL_ERROR);
    assert(status.time.hours == 0 && status.time.minutes == 0 && status.time.seconds == 0);
    assert(status.marker == 0x52544331 && backup == 0x52544331);
    assert(get_time_calls == read_calls && set_calls == calls && date_calls == 1 && rtc_seconds == seconds && !shadow_locked);
    lse_ready = 1;
    assert(watch_rtc_get_time(&time) == HAL_OK && time.hours == 14 && time.minutes == 36 && time.seconds == 1);
    drop_lse_on_read = 1;
    time.hours = 1;
    time.minutes = 2;
    time.seconds = 3;
    assert(watch_rtc_get_time(&time) == HAL_ERROR && !shadow_locked);
    assert(time.hours == 1 && time.minutes == 2 && time.seconds == 3);
    drop_lse_on_read = 0;
    lse_ready = 1;
    watch_rtc_get_status(&status);
    assert(status.ready && status.lse_ready && status.synced && status.read_status == HAL_OK);
    assert(status.time.hours == 14 && status.time.minutes == 36 && status.time.seconds == 1);
    assert(status.marker == 0x52544331 && backup == 0x52544331);
    assert(set_calls == calls && date_calls == 1 && rtc_seconds == seconds && !shadow_locked);
    puts("PASS: runtime LSE loss reports read error without HAL reads or output changes; loss during read unlocks shadows; "
         "recovery retains time and marker.");
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
    set_init_stuck = 1;
    feed("18:20\n", 1400);
    assert(app_clock_get_result() == APP_CLOCK_RTC_ERROR && set_calls == calls && rtc_seconds == seconds);
    set_init_stuck = 0;
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
    assert(watch_rtc_set_time(12, 34, 60) == 1 && set_calls == calls);
    set_init_stuck = 1;
    feed("18:20:59\n", 2200);
    assert(app_clock_get_result() == APP_CLOCK_RTC_ERROR && set_calls == calls && rtc_seconds == seconds);
    set_init_stuck = 0;
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
    lse_ready = 0;
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
    bounded_set_time_tests();
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
