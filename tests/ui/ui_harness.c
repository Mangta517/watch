/* Uses the project's real LVGL renderer and input engine, with hardware stubs.
 * The UI source is included so assertions can inspect page/scroll state. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../application/app_ui.c"
#include "lv_port_indev_template.h"
#include "app_reset.h"
#include "app_battery_log.h"
#include "touch.h"

void app_reset_assert_stop(void)
{
    abort();
}

void app_reset_get_info(app_reset_info_struct *info)
{
    memset(info, 0, sizeof(*info));
    info->valid = 1;
    info->reset_flags = 0x24000000;
    info->kind = APP_RESET_HARDFAULT;
    info->pc = 0x08012344;
}

static unsigned host_log_rx, host_log_reset, host_log_tx;
uint8_t app_battery_log_get_status(app_battery_log_status_struct *status)
{
    memset(status, 0, sizeof(*status));
    status->state = APP_BATTERY_LOG_READY;
    return 0;
}
uint8_t app_battery_log_read(uint32_t slot, uint32_t session, app_battery_log_record_struct *record)
{
    (void)slot;
    (void)session;
    (void)record;
    return 1;
}
uint8_t app_battery_log_read_reset(uint32_t slot, uint32_t session, app_reset_info_struct *info, uint8_t *complete)
{
    (void)slot;
    (void)session;
    (void)info;
    *complete = 0;
    return 0;
}
static uint8_t host_log_busy;
void app_battery_log_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    (void)data;
    (void)now_ms;
    host_log_rx += size;
}
void app_battery_log_ble_reset(void)
{
    ++host_log_reset;
}
uint8_t app_battery_log_ble_service(uint8_t ready, uint32_t now_ms)
{
    (void)ready;
    (void)now_ms;
    ++host_log_tx;
    return host_log_busy;
}

static uint32_t host_tick = 9 * 3600000 + 30 * 60000 + 42000;
uint32_t HAL_GetTick(void)
{
    return host_tick;
}

static GPIO_PinState host_key = GPIO_PIN_SET;
static GPIO_PinState host_wake = GPIO_PIN_RESET;
static lv_indev_state_t host_touch = LV_INDEV_STATE_RELEASED;
static uint8_t host_touch_error;
static lv_point_t host_point;
static unsigned int host_touch_critical, host_touch_reads;
static uint8_t host_online = 1;
static uint32_t host_steps, host_wrist_sequence, host_wrist_ms;
static uint8_t host_mag_online = 1;
static uint8_t host_mag_calibrated;
static float host_yaw = 725.2f;
static float host_altitude = -0.5f;
static app_mag_cal_state host_mag_cal_state;
static app_mag_fusion_state host_mag_fusion_state = APP_MAG_FUSION_ACTIVE;
static app_mag_persist_state host_mag_persist_state;
static uint8_t host_light;
static uint8_t host_lcd_sleeping, host_tp_sleeping;
static unsigned int host_lcd_sleeps, host_tp_sleeps, host_tp_wakes;
static uint8_t host_touch_run_during_sleep;
static unsigned int host_screen_off_publications, host_screen_on_publications;

void lv_port_disp_set_sleep(uint8_t sleeping)
{
    host_lcd_sleeping = sleeping;
    if (sleeping != 0)
    {
        assert(host_light == 0);
        ++host_lcd_sleeps;
        if (host_touch_run_during_sleep != 0)
        {
            host_tick += 101;
            app_touch_service(); /* LCD等待期间，低优先级触摸任务已休眠并准备阻塞。 */
        }
    }
}
static uint8_t host_idle, host_motion, host_sport;
void app_task_set_power_state(uint8_t idle, uint8_t motion, uint8_t screen_off, uint8_t sport)
{
    host_idle = idle;
    host_motion = motion;
    host_sport = sport;
    assert(screen_off == (s_screen_on == 0));
    if (host_touch_run_during_sleep != 0)
    {
        if (screen_off != 0)
        {
            ++host_screen_off_publications;
        }
        else if (host_screen_off_publications != 0)
        {
            ++host_screen_on_publications;
        }
    }
}
static lv_color_t framebuffer[240 * 280];
static lv_color_t draw_buffer[240 * 12];
static uint32_t host_flush_calls;
static uint32_t host_flush_pixels;
static uint32_t host_battery_reads;
static uint32_t host_heart_reads;
static uint32_t host_baro_reads;
static unsigned min_free = 40960;
static uint8_t host_bt_pending, host_rtc_pending;
static uint8_t host_bt_powered = 1;
static unsigned host_bt_enable_calls, host_bt_disable_calls;
void kt6368a_set_enabled(uint8_t enabled)
{
    host_bt_powered = enabled;
    if (enabled != 0)
    {
        ++host_bt_enable_calls;
    }
    else
    {
        ++host_bt_disable_calls;
    }
    kt6368a_flush_rx();
}
static HAL_StatusTypeDef host_bt_init_status = HAL_OK;
static uint8_t host_bt_rx[1024];
static uint32_t host_bt_rx_ms[1024];
static uint8_t host_bt_rx_flags[1024];
static uint16_t host_bt_rx_size;
static uint16_t host_bt_rx_pos;
static uint32_t host_bt_dropped, host_bt_errors;
static uint32_t host_bt_activity_ms;
static uint8_t host_bt_activity_valid, host_bt_loss_pending;
static uint8_t host_bt_inject_on_empty;
extern unsigned host_diag_rx_resets;
static HAL_StatusTypeDef host_bt_tx_result = HAL_OK;
static uint8_t host_bt_tx_busy;
static uint8_t host_async_active;
static const uint8_t *host_async_data;
static uint8_t host_bt_tx_data[256];
static uint16_t host_bt_tx_size;
static uint32_t host_bt_tx_calls;
static uint8_t host_previous_valid;
static uint32_t host_previous_seconds;

uint8_t app_uptime_get_previous(uint32_t *seconds)
{
    if (host_previous_valid == 0)
    {
        return 0;
    }
    *seconds = host_previous_seconds;
    return 1;
}
static uint32_t host_rtc_seconds = 10 * 3600 + 10 * 60;
static uint32_t host_rtc_tick = 9 * 3600000 + 30 * 60000 + 42000;
static uint32_t host_rtc_set_calls;
static uint32_t host_rtc_read_calls;
static uint8_t host_rtc_synced = 1;
static HAL_StatusTypeDef host_rtc_status = HAL_OK;
static app_heart_struct host_heart;
static app_heart_sample_struct host_heart_samples[4];
static uint16_t host_heart_samples_count;
static app_aht21_struct host_aht21;
static app_battery_struct host_battery = {.voltage_mv = 3850, .valid = 1, .percent = 50};

void app_battery_get_snapshot(app_battery_struct *snapshot)
{
    ++host_battery_reads;
    *snapshot = host_battery;
}

void app_aht21_get_snapshot(app_aht21_struct *snapshot)
{
    *snapshot = host_aht21;
}

uint16_t app_heart_read_samples(app_heart_sample_struct *samples, uint16_t size)
{
    uint16_t count;
    if (host_heart_samples_count < size)
    {
        count = host_heart_samples_count;
    }
    else
    {
        count = size;
    }
    memcpy(samples, host_heart_samples, count * sizeof(samples[0]));
    host_heart_samples_count = 0;
    return count;
}

void app_heart_get_snapshot(app_heart_struct *heart)
{
    ++host_heart_reads;
    *heart = host_heart;
}

void app_heart_set_enabled(uint8_t enabled)
{
    host_heart.enabled = enabled;
    host_heart.bpm = 0;
    if (enabled)
    {
        host_heart.state = APP_HEART_MEASURING;
    }
    else
    {
        host_heart.state = APP_HEART_OFF;
    }
}

uint8_t watch_rtc_init_is_pending(void)
{
    return host_rtc_pending;
}
HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time)
{
    uint32_t seconds = (host_rtc_seconds + (uint32_t)(host_tick - host_rtc_tick) / 1000) % 86400;
    ++host_rtc_read_calls;
    if (host_rtc_status != HAL_OK)
    {
        return host_rtc_status;
    }
    time->hours = seconds / 3600;
    time->minutes = seconds / 60 % 60;
    time->seconds = seconds % 60;
    return HAL_OK;
}
uint8_t watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    if (host_rtc_status != HAL_OK)
    {
        return 1;
    }
    assert(hours <= 23 && minutes <= 59 && seconds <= 59);
    host_rtc_seconds = hours * 3600 + minutes * 60 + seconds;
    host_rtc_tick = host_tick;
    host_rtc_synced = 1;
    ++host_rtc_set_calls;
    return 0;
}
uint8_t watch_rtc_is_synced(void)
{
    return host_rtc_synced;
}

void watch_rtc_get_status(watch_rtc_status_struct *status)
{
    memset(status, 0, sizeof(*status));
    status->ready = host_rtc_status == HAL_OK;
    status->lse_ready = status->ready;
    status->synced = host_rtc_synced;
    status->marker = 0x52544330;
    if (host_rtc_synced != 0)
    {
        status->marker = 0x52544331;
    }
    status->read_status = watch_rtc_get_time(&status->time);
}

uint8_t kt6368a_tx_is_busy(void)
{
    return host_bt_tx_busy;
}

uint8_t kt6368a_rx_pending(void)
{
    return host_bt_rx_pos != host_bt_rx_size;
}

HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    assert(data != NULL && size < sizeof(host_bt_tx_data));
    assert(timeout_ms == UI_BT_TX_TIMEOUT_MS || timeout_ms == 100);
    ++host_bt_tx_calls;
    if (host_bt_tx_result != HAL_OK)
    {
        return host_bt_tx_result;
    }
    memcpy(host_bt_tx_data, data, size);
    host_bt_tx_data[size] = '\0';
    host_bt_tx_size = size;
    return HAL_OK;
}

uint8_t kt6368a_is_connected(void)
{
    return 0; /* 本板没有连接状态输入 */
}

HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef result = kt6368a_write(data, size, timeout_ms);
    if (result == HAL_OK)
    {
        host_async_active = host_bt_tx_busy = 1;
        host_async_data = data;
    }
    return result;
}

uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    if (host_async_active == 0)
    {
        return 0;
    }
    assert(memcmp(host_async_data, host_bt_tx_data, host_bt_tx_size) == 0);
    host_async_active = host_bt_tx_busy = 0;
    *result = HAL_OK;
    return 1;
}
uint8_t kt6368a_init_is_pending(void)
{
    return host_bt_pending;
}
HAL_StatusTypeDef kt6368a_get_init_status(void)
{
    return host_bt_init_status;
}
uint16_t kt6368a_read(uint8_t *data, uint16_t size)
{
    uint16_t count = host_bt_rx_size - host_bt_rx_pos;
    if (count > size)
    {
        count = size;
    }
    memcpy(data, host_bt_rx + host_bt_rx_pos, count);
    host_bt_rx_pos += count;
    return count;
}
static void host_bluetooth_feed(const uint8_t *data, uint16_t size);
uint16_t kt6368a_read_timed(uint8_t *data, uint16_t size, kt6368a_rx_info_struct *info)
{
    uint16_t count = 0;
    info->received_ms = host_bt_activity_ms;
    info->sampled_ms = HAL_GetTick();
    info->flags = 0;
    if (host_bt_rx_pos == host_bt_rx_size)
    {
        if (host_bt_loss_pending != 0)
        {
            info->flags = KT6368A_RX_LOSS;
        }
        if (host_bt_inject_on_empty != 0)
        {
            host_bt_inject_on_empty = 0;
            host_bluetooth_feed((const uint8_t *)"4:56\n", 5);
            ++host_tick;
        }
        return 0;
    }
    info->received_ms = host_bt_rx_ms[host_bt_rx_pos];
    info->flags = host_bt_rx_flags[host_bt_rx_pos];
    while (count < size && host_bt_rx_pos < host_bt_rx_size)
    {
        if (count != 0 && (host_bt_rx_ms[host_bt_rx_pos] != info->received_ms || host_bt_rx_flags[host_bt_rx_pos] != 0))
        {
            break;
        }
        data[count++] = host_bt_rx[host_bt_rx_pos++];
    }
    return count;
}
uint32_t kt6368a_get_rx_dropped(void)
{
    return host_bt_dropped;
}
uint32_t kt6368a_get_rx_errors(void)
{
    return host_bt_errors;
}
void kt6368a_flush_rx(void)
{
    host_bt_rx_pos = host_bt_rx_size;
    host_bt_activity_valid = 0;
    host_bt_loss_pending = 0;
}
static void host_bluetooth_feed(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    if (host_bt_rx_pos == host_bt_rx_size)
    {
        host_bt_rx_size = host_bt_rx_pos = 0;
    }
    assert(host_bt_rx_size + size <= sizeof(host_bt_rx));
    for (i = 0; i < size; ++i)
    {
        uint16_t index = host_bt_rx_size++;
        host_bt_rx[index] = data[i];
        host_bt_rx_ms[index] = HAL_GetTick();
        host_bt_rx_flags[index] = 0;
        if (host_bt_activity_valid != 0 && (uint32_t)(HAL_GetTick() - host_bt_activity_ms) >= KT6368A_RX_IDLE_MS)
        {
            host_bt_rx_flags[index] |= KT6368A_RX_GAP;
        }
        if (host_bt_loss_pending != 0)
        {
            host_bt_rx_flags[index] |= KT6368A_RX_LOSS;
            host_bt_loss_pending = 0;
        }
        host_bt_activity_ms = HAL_GetTick();
        host_bt_activity_valid = 1;
    }
}
static void host_bluetooth_fault(uint8_t dropped)
{
    if (dropped != 0)
    {
        ++host_bt_dropped;
    }
    else
    {
        ++host_bt_errors;
    }
    host_bt_loss_pending = 1;
    host_bt_activity_ms = HAL_GetTick();
    host_bt_activity_valid = 1;
}

static void bluetooth_clear_frame_tests(void)
{
    uint32_t calls = host_rtc_set_calls;
    uint32_t seconds = host_rtc_seconds;
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    bluetooth_receive();
    host_bluetooth_feed((const uint8_t *)"34:", 3);
    bluetooth_clear_cb(NULL);
    assert(host_bt_rx_pos == 0);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls);
    host_bluetooth_feed((const uint8_t *)"56\n", 3);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1);
    assert(host_rtc_seconds == 12 * 3600 + 34 * 60 + 56);
    host_rtc_seconds = seconds;
    host_rtc_set_calls = calls;
    bluetooth_clear_cb(NULL);
    puts("PASS: CLEAR preserves queued bytes and partial clock frames.");
}

static void bluetooth_rx_error_tests(void)
{
    uint32_t calls = host_rtc_set_calls;
    unsigned resets = host_diag_rx_resets;
    app_weather_struct before, after;
    app_weather_get_snapshot(&before);
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    bluetooth_receive();
    host_bluetooth_fault(1);
    host_bluetooth_feed((const uint8_t *)"34\n", 3);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls && host_diag_rx_resets == resets + 1);
    host_bluetooth_feed((const uint8_t *)"35\n13:14:15\n", 12);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 13 * 3600 + 14 * 60 + 15);

    host_bluetooth_feed((const uint8_t *)"WEATHER,3,2", 11);
    bluetooth_receive();
    host_bluetooth_fault(0);
    bluetooth_receive();
    host_bluetooth_feed((const uint8_t *)"63,80\n", 6);
    bluetooth_receive();
    app_weather_get_snapshot(&after);
    assert(after.received_ms == before.received_ms && after.temperature_x10 == before.temperature_x10);
    host_bluetooth_feed((const uint8_t *)"WEATHER,1,-20,60\n", 17);
    bluetooth_receive();
    app_weather_get_snapshot(&after);
    assert(after.code == 1 && after.temperature_x10 == -20 && after.humidity_rh == 60);

    host_bluetooth_feed((const uint8_t *)"10:", 3);
    bluetooth_receive();
    calls = host_rtc_set_calls;
    host_bluetooth_fault(0);
    host_bluetooth_feed((const uint8_t *)"20\n", 3);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls);
    host_bluetooth_feed((const uint8_t *)"09:08", 5);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 9 * 3600 + 8 * 60);

    resets = host_diag_rx_resets;
    host_bt_dropped = UINT32_MAX - 1;
    host_bluetooth_fault(1);
    bluetooth_receive();
    host_bluetooth_fault(1);
    bluetooth_receive();
    assert(host_bt_dropped == 0 && host_diag_rx_resets == resets + 2);
    host_bluetooth_feed((const uint8_t *)"\n07:06:05\n", 10);
    bluetooth_receive();
    assert(host_rtc_seconds == 7 * 3600 + 6 * 60 + 5);
    puts("PASS: UART loss/error/counter-wrap resets all parsers, discards suffixes and recovers at newline or idle without clearing valid weather.");
}

static void bluetooth_arrival_time_tests(void)
{
    uint32_t calls = host_rtc_set_calls;
    uint32_t saved_seconds = host_rtc_seconds;
    uint8_t budget_frame[UI_BT_RX_BUDGET + 6];
    app_weather_struct weather;
    assert(host_bt_rx_pos == host_bt_rx_size);
    set_screen_on(1);
    create_page(PAGE_WATCH, 0, 0);
    assert(s_time != NULL);
    host_bt_pending = host_rtc_pending = 0;
    host_rtc_status = HAL_OK;
    host_bluetooth_feed((const uint8_t *)"\n", 1);
    bluetooth_receive();

    /* UART只间隔2ms，后半帧排队250ms不能被误当成手机停发。 */
    host_bluetooth_feed((const uint8_t *)"12:3", 4);
    bluetooth_receive();
    host_tick += 2;
    host_bluetooth_feed((const uint8_t *)"4:56\n", 5);
    host_tick += 250;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 12 * 3600 + 34 * 60 + 56);
    refresh_clock();
    assert(strcmp(lv_label_get_text(s_time), "12:34") == 0);

    /* 空队列在199ms取快照后立即来了后缀，任务到200ms才运行也不能提交旧前缀。 */
    calls = host_rtc_set_calls;
    host_bluetooth_feed((const uint8_t *)"12:3", 4);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS - 1;
    host_bt_inject_on_empty = 1;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls && host_bt_rx_pos < host_bt_rx_size);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 12 * 3600 + 34 * 60 + 56);

    host_bluetooth_feed((const uint8_t *)"WEATHER,3,2", 11);
    bluetooth_receive();
    host_tick += 2;
    host_bluetooth_feed((const uint8_t *)"63,80\n", 6);
    host_tick += 250;
    bluetooth_receive();
    app_weather_get_snapshot(&weather);
    assert(weather.valid != 0 && weather.code == 3 && weather.temperature_x10 == 263 && weather.humidity_rh == 80);

    /* 真正200ms空闲仍结束无换行帧，包括已经排在同一队列里的下一帧。 */
    calls = host_rtc_set_calls;
    host_bluetooth_feed((const uint8_t *)"01:02", 5);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS - 1;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls);
    ++host_tick;
    host_bluetooth_feed((const uint8_t *)"03:04", 5);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 3600 + 2 * 60);
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 2 && host_rtc_seconds == 3 * 3600 + 4 * 60);
    host_bluetooth_feed((const uint8_t *)"WEATHER,1,121,60", sizeof("WEATHER,1,121,60") - 1);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    host_bluetooth_feed((const uint8_t *)"WEATHER,3,222,70", sizeof("WEATHER,3,222,70") - 1);
    bluetooth_receive();
    app_weather_get_snapshot(&weather);
    assert(weather.code == 1 && weather.temperature_x10 == 121 && weather.humidity_rh == 60);
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    app_weather_get_snapshot(&weather);
    assert(weather.code == 3 && weather.temperature_x10 == 222 && weather.humidity_rh == 70);

    /* 错误后的干净首帧已经到达时，消费方不能再清掉整条队列。 */
    host_bluetooth_feed((const uint8_t *)"19:", 3);
    bluetooth_receive();
    host_bluetooth_fault(0);
    host_tick += KT6368A_RX_IDLE_MS;
    host_bluetooth_feed((const uint8_t *)"06:07:08\nWEATHER,2,191,55\n", sizeof("06:07:08\nWEATHER,2,191,55\n") - 1);
    host_tick += 300;
    calls = host_rtc_set_calls;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 6 * 3600 + 7 * 60 + 8);
    app_weather_get_snapshot(&weather);
    assert(weather.code == 2 && weather.temperature_x10 == 191 && weather.humidity_rh == 55);

    host_bluetooth_feed((const uint8_t *)"17:", 3);
    bluetooth_receive();
    host_bluetooth_fault(0);
    host_tick += 2;
    host_bluetooth_feed((const uint8_t *)"23\n", 3);
    host_tick += 250;
    calls = host_rtc_set_calls;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls);
    host_bluetooth_feed((const uint8_t *)"08:09:10\n", 9);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 8 * 3600 + 9 * 60 + 10);

    /* 一轮256字节预算耗尽时，队列中的同帧后缀尚未消费，不能跑空闲结帧。 */
    memset(budget_frame, '\n', sizeof(budget_frame));
    memcpy(budget_frame + UI_BT_RX_BUDGET - 3, "12:34:56\n", 9);
    host_bluetooth_feed(budget_frame, sizeof(budget_frame));
    host_tick += 250;
    calls = host_rtc_set_calls;
    bluetooth_receive();
    assert(host_bt_rx_pos == UI_BT_RX_BUDGET && host_bt_rx_pos < host_bt_rx_size && host_rtc_set_calls == calls);
    bluetooth_receive();
    assert(host_bt_rx_pos == host_bt_rx_size && host_rtc_set_calls == calls + 1);
    assert(host_rtc_seconds == 12 * 3600 + 34 * 60 + 56);

    host_tick = UINT32_MAX - 1;
    host_bluetooth_feed((const uint8_t *)"22:", 3);
    bluetooth_receive();
    host_tick += 2;
    host_bluetooth_feed((const uint8_t *)"11:09\n", 6);
    host_tick += 250;
    calls = host_rtc_set_calls;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 22 * 3600 + 11 * 60 + 9);
    host_rtc_seconds = saved_seconds;
    host_rtc_tick = host_tick;
    refresh_clock();
    puts("PASS: real UART arrival times preserve delayed clock/weather fragments, genuine idle frames, clean frames after errors, loss boundaries, RX budgets and tick wrap.");
}

TickType_t xTaskGetTickCount(void)
{
    return host_tick;
}
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    if (pin == WAKE_Pin)
    {
        return host_wake;
    }
    else
    {
        return host_key;
    }
}
void LCD_Set_Light(uint8_t percent)
{
    host_light = percent;
}
uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)
{
    memset(imu, 0, sizeof(*imu));
    imu->steps = host_steps;
    imu->wrist_raise_sequence = host_wrist_sequence;
    imu->wrist_raise_ms = host_wrist_ms;
    imu->pitch_deg = 12.4f;
    imu->roll_deg = -3.6f;
    imu->yaw_deg = host_yaw;
    imu->mag_online = host_mag_online;
    imu->mag_fused = host_mag_online && host_mag_cal_state != APP_MAG_CAL_RUNNING;
    imu->mag_calibrated = host_mag_calibrated;
    imu->mag_cal_state = host_mag_cal_state;
    imu->mag_persist_state = host_mag_persist_state;
    if (!host_mag_online)
    {
        imu->mag_fusion_state = APP_MAG_FUSION_NO_DATA;
    }
    else if (host_mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        imu->mag_fusion_state = APP_MAG_FUSION_CALIBRATING;
    }
    else
    {
        imu->mag_fusion_state = host_mag_fusion_state;
    }
    imu->mag_cal_samples = 120;
    imu->mag_cal_span_uT[0] = 80;
    imu->mag_cal_span_uT[1] = 100;
    imu->mag_cal_span_uT[2] = 120;
    imu->mag_body_uT[0] = 26.4f;
    imu->mag_body_uT[1] = -14.2f;
    imu->mag_body_uT[2] = 37.8f;
    return host_online;
}
void App_IMU_MagCalibrate(uint8_t start)
{
    if (start)
    {
        host_mag_cal_state = APP_MAG_CAL_RUNNING;
    }
    else
    {
        host_mag_cal_state = APP_MAG_CAL_DONE;
    }
    if (!start)
    {
        host_mag_calibrated = 1;
        host_mag_persist_state = APP_MAG_PERSIST_SAVED;
    }
}
uint8_t App_Baro_GetSnapshot(app_baro_struct *baro)
{
    ++host_baro_reads;
    memset(baro, 0, sizeof(*baro));
    baro->pressure_pa = 101325;
    baro->alt_m = host_altitude;
    return host_online;
}
static app_baro_history_struct host_baro_history;
void app_baro_get_history(app_baro_history_struct *history)
{
    *history = host_baro_history;
}
static void host_record_baro(void)
{
    int16_t value = APP_BARO_HISTORY_NONE;
    if (host_online != 0 && isfinite(host_altitude))
    {
        value = (int16_t)roundf(host_altitude);
    }
    memmove(host_baro_history.altitude_m, host_baro_history.altitude_m + 1,
            (APP_BARO_HISTORY_POINTS - 1) * sizeof(int16_t));
    host_baro_history.altitude_m[APP_BARO_HISTORY_POINTS - 1] = value;
    ++host_baro_history.sequence;
}
static void flush(lv_disp_drv_t *driver, const lv_area_t *area, lv_color_t *pixels)
{
    int x, y;
    assert(host_lcd_sleeping == 0);
    ++host_flush_calls;
    host_flush_pixels += lv_area_get_size(area);
    for (y = area->y1; y <= area->y2; ++y)
    {
        for (x = area->x1; x <= area->x2; ++x)
        {
            framebuffer[y * 240 + x] = *pixels++;
        }
    }
    lv_disp_flush_ready(driver);
}
void host_touch_enter_critical(void)
{
    assert(host_touch_critical++ == 0);
}
void host_touch_exit_critical(void)
{
    assert(host_touch_critical-- == 1);
}
uint8_t Touch_Init(void)
{
    assert(host_touch_critical == 0);
    return 0xB5;
}
uint8_t Touch_Sleep(void)
{
    assert(host_touch_critical == 0 && host_tp_sleeping == 0);
    host_tp_sleeping = 1;
    ++host_tp_sleeps;
    return 0;
}
void Touch_Wakeup(void)
{
    assert(host_touch_critical == 0 && host_tp_sleeping != 0);
    host_tp_sleeping = 0;
    ++host_tp_wakes;
}
uint8_t Touch_GetPoint(uint16_t *x, uint16_t *y)
{
    assert(host_touch_critical == 0 && host_tp_sleeping == 0);
    ++host_touch_reads;
    if (host_touch_error != 0)
    {
        return TOUCH_READ_ERROR;
    }
    *x = (uint16_t)host_point.x;
    *y = (uint16_t)host_point.y;
    return host_touch == LV_INDEV_STATE_PRESSED;
}
static unsigned int host_short_clicks;
static void short_click_cb(lv_event_t *event)
{
    (void)event;
    ++host_short_clicks;
}
static void touch_bridge_tests(void)
{
    lv_obj_t *button = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(button, 5, 5);
    lv_obj_set_size(button, 30, 30);
    lv_obj_add_event_cb(button, short_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_update_layout(button);
    host_point.x = host_point.y = 15;
    host_touch = LV_INDEV_STATE_PRESSED;
    host_tick += 5;
    app_touch_service();
    host_touch = LV_INDEV_STATE_RELEASED;
    host_tick += 5;
    app_touch_service();
    lv_timer_handler();
    assert(host_short_clicks == 1); /* 两次采集之间没有LVGL轮询，仍收到完整点击。 */
    host_touch = LV_INDEV_STATE_PRESSED;
    host_tick += 5;
    app_touch_service();
    lv_timer_handler();
    host_tick += 100; /* 模拟touch任务停滞，过期取消不能触发一次假点击。 */
    lv_timer_handler();
    assert(host_short_clicks == 1);
    host_touch = LV_INDEV_STATE_RELEASED;
    host_tick += 5;
    app_touch_service();
    lv_timer_handler();
    /* 读失败取消按下；恢复时仍按住及之后松手都不能触发点击。 */
    host_touch = LV_INDEV_STATE_PRESSED;
    host_tick += 5;
    app_touch_service();
    lv_timer_ready(lv_indev_get_next(NULL)->driver->read_timer);
    lv_timer_handler();
    host_touch_error = 1;
    host_tick += 5;
    app_touch_service();
    lv_timer_ready(lv_indev_get_next(NULL)->driver->read_timer);
    lv_timer_handler();
    assert(host_short_clicks == 1);
    host_touch_error = 0;
    host_tick += 5;
    app_touch_service();
    lv_timer_ready(lv_indev_get_next(NULL)->driver->read_timer);
    lv_timer_handler();
    assert(host_short_clicks == 1);
    host_touch = LV_INDEV_STATE_RELEASED;
    host_tick += 5;
    app_touch_service();
    lv_timer_ready(lv_indev_get_next(NULL)->driver->read_timer);
    lv_timer_handler();
    assert(host_short_clicks == 1);
    /* 真实松手后的新点击正常，不永久禁用输入。 */
    host_touch = LV_INDEV_STATE_PRESSED;
    host_tick += 5;
    app_touch_service();
    host_touch = LV_INDEV_STATE_RELEASED;
    host_tick += 5;
    app_touch_service();
    lv_timer_ready(lv_indev_get_next(NULL)->driver->read_timer);
    lv_timer_handler();
    assert(host_short_clicks == 2);
    lv_obj_del(button);
    puts("PASS: real LVGL receives buffered clicks; stale/read-failed touch cancels without a false click, then recovers after release.");
}
static void advance(unsigned ms)
{
    unsigned elapsed;
    unsigned int touch_reads;
    static uint32_t touch_tick, display_tick;
    uint32_t period_ms;
    lv_mem_monitor_t memory;
    for (elapsed = 0; elapsed < ms; elapsed += 5)
    {
        host_tick += 5; /* Match the firmware LVGL task's5ms scheduling. */
        period_ms = 5;
        if (host_idle != 0)
        {
            period_ms = 20;
        }
        if ((uint32_t)(host_tick - touch_tick) >= period_ms &&
            (app_touch_is_sleeping() == 0 || App_UI_IsScreenOn() != 0))
        {
            touch_tick = host_tick;
            app_touch_service();
        }
        touch_reads = host_touch_reads;
        if ((uint32_t)(host_tick - display_tick) < period_ms)
        {
            continue;
        }
        display_tick = host_tick;
        if (App_UI_IsScreenOn() != 0)
        {
            lv_timer_handler();
        }
        else
        {
            App_UI_ServiceSleeping();
        }
        assert(host_touch_reads == touch_reads); /* LVGL输入回调不能访问I2C。 */
        lv_mem_monitor(&memory);
        if (memory.free_size < min_free)
        {
            min_free = memory.free_size;
        }
    }
}
/* 长时间时钟测试保持有输入活动；无触摸待机由独立用例验证。 */
static void advance_active(unsigned ms)
{
    unsigned step;
    while (ms != 0)
    {
        step = ms;
        if (step > 1000)
        {
            step = 1000;
        }
        lv_disp_trig_activity(NULL);
        advance(step);
        ms -= step;
    }
}

/* 用真实刷新回调检查：输入与显示内容都不变时，不应再次传输像素。 */
static void assert_static_refresh(void)
{
    uint32_t calls;
    unsigned iteration;
    lv_refr_now(NULL);
    calls = host_flush_calls;
    for (iteration = 0; iteration < 3; ++iteration)
    {
        refresh_data();
        lv_refr_now(NULL);
        assert(host_flush_calls == calls);
    }
}

static lv_coord_t trend_latest(const ui_trend_struct *trend)
{
    uint16_t index = (trend->series->start_point + UI_TREND_POINTS - 1) % UI_TREND_POINTS;
    return trend->series->y_points[index];
}

static void compass_direction_tests(void)
{
    static const float yaw[] = {0.0f, -90.0f, -180.0f, 90.0f, -359.8f, 1080.0f};
    static const int16_t heading[] = {0, 90, 180, 270, 0, 0};
    unsigned i;
    for (i = 0; i < sizeof(yaw) / sizeof(yaw[0]); ++i)
    {
        host_yaw = yaw[i];
        s_compass_heading = -1;
        refresh_data();
        assert(s_compass_heading == heading[i]);
    }
    /* 359°→1°及反向都只走北点附近，不绕远路。 */
    host_yaw = -359.0f;
    s_compass_heading = -1;
    refresh_data();
    host_yaw = -1.0f;
    advance_active(100);
    assert(s_compass_heading == 359 || s_compass_heading == 0);
    advance_active(1500);
    assert(s_compass_heading == 1);
    host_yaw = -359.0f;
    advance_active(1600);
    assert(s_compass_heading == 359);

    /* 90°目标跳变后700ms内追到2°以内，防止显示滤波重新产生长拖尾。 */
    host_yaw = 0.0f;
    s_compass_heading = -1;
    refresh_data();
    host_yaw = -90.0f;
    advance_active(700);
    assert(s_compass_heading >= 88 && s_compass_heading <= 90);

    host_yaw = 0.0f;
    s_compass_heading = -1;
    refresh_data();
    host_yaw = -120.0f;
    s_refresh_tick = lv_tick_get(); /* 手动刷新后对齐100ms周期，覆盖一轮完整显示步长。 */
    advance_active(110);
    assert(s_compass_heading > 12 && s_compass_heading <= 36);
    advance_active(900);
    assert(s_compass_heading == 120);
    host_yaw = -90.0f;
    advance_active(3000);
    for (i = 0; i < 10; ++i)
    {
        if (i % 2 == 0)
        {
            host_yaw = -90.4f;
        }
        else
        {
            host_yaw = -89.6f;
        }
        advance_active(100);
        assert(s_compass_heading == 90);
    }
    host_yaw = NAN;
    refresh_data();
    assert(s_compass_heading == -1 && strcmp(lv_label_get_text(s_compass_value), "--") == 0);
    host_yaw = 725.2f;
    host_online = 0;
    refresh_data();
    assert(s_compass_heading == -1);
    host_online = 1;
    refresh_data();
    assert(s_compass_heading == 355);
    assert(strcmp(lv_label_get_text(s_compass_value), "355\xC2\xB0 N") == 0);
}
static void compass_redraw_tests(void)
{
    static const int16_t headings[] = {0, 1, 44, 45, 89, 90, 135, 179, 180, 225, 269, 270, 315, 359, -1, 45, 45, -1, 0};
    static lv_color_t partial_frame[240 * 280];
    app_imu_struct imu;
    lv_area_t card;
    uint32_t pixels;
    float display_deg = s_compass_display_deg;
    uint8_t online;
    unsigned i;

    App_IMU_GetSnapshot(&imu);
    lv_obj_get_coords(s_compass, &card);
    lv_refr_now(NULL);
    for (i = 0; i < sizeof(headings) / sizeof(headings[0]); ++i)
    {
        online = headings[i] >= 0;
        imu.yaw_deg = -(float)headings[i];
        s_compass_display_deg = (float)headings[i];
        /* 第二个45度保持指针位置，只切换未校准/已校准颜色。 */
        if (i == 16)
        {
            imu.mag_calibrated = 1;
        }
        pixels = host_flush_pixels;
        compass_update(&imu, online);
        lv_refr_now(NULL);
        assert(host_flush_pixels > pixels && host_flush_pixels - pixels < lv_area_get_size(&card));
        memcpy(partial_frame, framebuffer, sizeof(partial_frame));
        lv_obj_invalidate(s_compass);
        lv_refr_now(NULL);
        assert(memcmp(partial_frame, framebuffer, sizeof(partial_frame)) == 0);
    }
    App_IMU_GetSnapshot(&imu);
    s_compass_display_deg = display_deg;
    compass_update(&imu, 1);
    lv_refr_now(NULL);
    puts("PASS: compass partial redraw matches full-card pixels across quadrants, validity and color changes, with fewer transferred pixels.");
}

static void tap(int x, int y)
{
    host_point.x = x;
    host_point.y = y;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(40);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(230);
}

static void assert_in_body(lv_obj_t *obj)
{
    lv_area_t body, area;
    lv_obj_update_layout(s_screen);
    lv_obj_get_coords(s_body, &body);
    lv_obj_get_coords(obj, &area);
    assert(area.x1 >= body.x1 && area.x2 <= body.x2);
    assert(area.y1 >= body.y1 && area.y2 <= body.y2);
}

static void tap_object(lv_obj_t *obj)
{
    lv_area_t area;
    lv_obj_get_coords(obj, &area);
    tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
}
static void drag(int x, int y, int dx, int dy)
{
    unsigned step;
    host_point.x = x;
    host_point.y = y;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(20);
    for (step = 1; step <= 12; ++step)
    {
        host_point.x = x + dx * (int)step / 12;
        host_point.y = y + dy * (int)step / 12;
        advance(10);
    }
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(300);
}
static void key_back(void)
{
    host_key = GPIO_PIN_RESET;
    advance(60);
    host_key = GPIO_PIN_SET;
    advance(250);
}
/* 同时检查表盘数字时间、指针坐标和RTC秒值。 */
static const char *clock_text(void)
{
    return lv_label_get_text(s_time);
}

static void assert_rtc_seconds(uint8_t seconds)
{
    watch_time_struct time;
    assert(watch_rtc_get_time(&time) == HAL_OK);
    assert(time.seconds == seconds);
}

/* 通过真实入口控件和触摸引擎导航，避免测试直接调用create_page。 */
static void open_feature(ui_page_t page)
{
    static const ui_page_t destinations[] = {PAGE_HEART, PAGE_MOTION, PAGE_ENVIRONMENT, PAGE_AHT21,
                                              PAGE_WEATHER, PAGE_BLUETOOTH, PAGE_DISPLAY, PAGE_SPORT};
    lv_obj_t *entry = NULL;
    lv_area_t area;
    unsigned index;
    assert(s_page == PAGE_SETTINGS);
    for (index = 0; index < sizeof(destinations) / sizeof(destinations[0]); ++index)
    {
        if (destinations[index] == page)
        {
            entry = lv_obj_get_child(s_body, index);
            break;
        }
    }
    assert(entry != NULL);
    lv_obj_scroll_to_view(entry, LV_ANIM_OFF);
    advance(120);
    lv_obj_get_coords(entry, &area);
    tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
    assert(s_page == page);
    assert(s_main_pages[PAGE_WATCH].screen == NULL && s_main_pages[PAGE_SETTINGS].screen == NULL);
    assert(strcmp(lv_label_get_text(s_battery_label), "~50%") == 0);
    assert(s_battery_bar != NULL && lv_bar_get_value(s_battery_bar) == 50);
    assert(!lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN));
}

/* 检查真实页面文本，防止后续新增状态文字超出精简字库而静默显示缺字框。 */
static void assert_label_glyphs(lv_obj_t *obj)
{
    uint32_t child;
    if (lv_obj_check_type(obj, &lv_label_class))
    {
        const lv_font_t *font = lv_obj_get_style_text_font(obj, LV_PART_MAIN);
        if (font == &lv_font_montserrat_20 || font == &lv_font_montserrat_30)
        {
            const char *text = lv_label_get_text(obj);
            uint32_t index = 0;
            while (text[index] != '\0')
            {
                uint32_t letter = _lv_txt_encoded_next(text, &index);
                if (letter != '\n' && letter != '\r')
                {
                    lv_font_glyph_dsc_t glyph;
                    assert(lv_font_get_glyph_dsc(font, &glyph, letter, 0));
                    assert(!glyph.is_placeholder);
                }
            }
        }
    }
    for (child = 0; child < lv_obj_get_child_cnt(obj); ++child)
    {
        assert_label_glyphs(lv_obj_get_child(obj, child));
    }
}

static void screenshot(const char *name)
{
    char path[256];
    int pixel;
    FILE *file;
    lv_obj_invalidate(s_screen);
    advance(30);
    assert_label_glyphs(s_screen);
    snprintf(path, sizeof(path), "tests/ui/output/%s.ppm", name);
    file = fopen(path, "wb");
    assert(file);
    fprintf(file, "P6\n240 280\n255\n");
    for (pixel = 0; pixel < 240 * 280; ++pixel)
    {
        lv_color32_t color;
        color.full = lv_color_to32(framebuffer[pixel]);
        fputc(color.ch.red, file);
        fputc(color.ch.green, file);
        fputc(color.ch.blue, file);
    }
    fclose(file);
}
static void activity_ui_tests(void)
{
    assert(s_page == PAGE_WATCH && s_watch_steps != NULL);
    host_steps = 12345;
    refresh_battery();
    assert(strcmp(lv_label_get_text(s_watch_steps), "12345") == 0);
    screenshot("watch_activity");
    set_screen_on(0);
    host_wrist_ms = HAL_GetTick();
    ++host_wrist_sequence;
    App_UI_ServiceSleeping();
    assert(s_screen_on != 0 && host_light == s_brightness && host_bt_powered != 0);
    set_screen_on(0);
    App_UI_ServiceSleeping();
    assert(s_screen_on == 0);
    host_wrist_ms = HAL_GetTick() - 1000;
    ++host_wrist_sequence;
    App_UI_ServiceSleeping();
    assert(s_screen_on == 0);
    host_wrist_ms = HAL_GetTick();
    ++host_wrist_sequence;
    App_UI_ServiceSleeping();
    assert(s_screen_on != 0 && strcmp(lv_label_get_text(s_watch_steps), "12345") == 0);
    puts("PASS: homepage steps/icon, fresh wrist wake, one-time consumption, stale-event rejection and restored screen power.");

    /* 同一轮先超时熄屏再抬腕亮屏，两次电源变化都必须发布，供任务层产生唤醒通知。 */
    app_touch_service();
    host_touch_run_during_sleep = 1;
    host_tick += UI_TOUCH_IDLE_MS + 1;
    host_wrist_ms = HAL_GetTick();
    ++host_wrist_sequence;
    ui_timer_cb(NULL);
    assert(s_screen_on != 0 && app_touch_is_sleeping() != 0);
    assert(host_screen_off_publications != 0 && host_screen_on_publications != 0);
    host_touch_run_during_sleep = 0;
    app_touch_service(); /* 消费已发布亮屏变化对应的通知，恢复芯片。 */
    assert(app_touch_is_sleeping() == 0 && host_tp_sleeping == 0);
    puts("PASS: idle sleep and wrist wake in one UI round publish both transitions before the touch task resumes.");
}

static void bluetooth_sleep_tests(void)
{
    unsigned enables = host_bt_enable_calls, disables = host_bt_disable_calls;
    unsigned resets = host_log_reset, calls, i;
    uint32_t frames = app_clock_get_received_count();
    app_weather_struct weather;
    host_bt_pending = host_rtc_pending = 0;
    host_bt_tx_result = HAL_OK;
    host_heart_samples_count = host_previous_valid = 0;
    assert(s_sport_mode == 0 && host_async_active == 0 && s_screen_on != 0);
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    bluetooth_receive();
    set_screen_on(0);
    host_bluetooth_feed((const uint8_t *)"34:56\n", 6);
    App_UI_ServiceSleeping();
    assert(s_screen_on == 0 && host_light == 0 && s_bt_phone_ready != 0);
    assert(app_clock_get_received_count() == frames + 1);
    host_bluetooth_feed((const uint8_t *)"WEATHER,", 8);
    App_UI_ServiceSleeping();
    set_screen_on(1);
    host_bluetooth_feed((const uint8_t *)"3,263,80\n", 9);
    bluetooth_receive();
    app_weather_get_snapshot(&weather);
    assert(weather.valid != 0 && weather.code == 3 && weather.temperature_x10 == 263);
    set_screen_on(0);
    calls = host_bt_tx_calls;
    host_bluetooth_feed((const uint8_t *)"STATE,READ\n", 11);
    App_UI_ServiceSleeping();
    assert(host_async_active != 0 && strstr((const char *)host_bt_tx_data, "STATE,RTC,READY,") != NULL);
    /* 发送进行中切换屏幕不会关UART、取消TX或丢掉命令报告。 */
    set_screen_on(1);
    assert(host_async_active != 0);
    set_screen_on(0);
    App_UI_ServiceSleeping();
    for (i = 0; i < 3; ++i)
    {
        host_tick += 100;
        App_UI_ServiceSleeping();
        assert(host_async_active != 0);
        App_UI_ServiceSleeping();
        assert(host_async_active == 0);
    }
    assert(host_bt_tx_calls == calls + 4 && s_screen_on == 0 && host_bt_powered != 0);
    assert(host_bt_enable_calls == enables && host_bt_disable_calls == disables && host_log_reset == resets);
    set_screen_on(1);
    puts("PASS: BLE stays powered across PA4/idle/wrist screen changes; fragmented clock/weather and sleeping STATE RX/TX survive without UART restart or parser reset.");
}

static void bluetooth_eco_tests(void)
{
    unsigned resets, enables, disables;
    uint32_t frames = app_clock_get_received_count();
    app_weather_struct before, after;
    assert(s_bt_eco == 0 && s_bt_enabled != 0 && host_async_active == 0);
    host_previous_valid = host_heart_samples_count = host_log_busy = 0;
    host_bt_pending = host_rtc_pending = 0;
    host_wake = GPIO_PIN_RESET;
    create_page(PAGE_BLUETOOTH, 0, 0);
    advance(220);
    assert(strcmp(lv_label_get_text(s_bt_power_label), "BLE: KEEP CONNECTED") == 0);
    lv_obj_scroll_to_view(lv_obj_get_parent(s_bt_power_label), LV_ANIM_OFF);
    advance(30);
    tap_object(lv_obj_get_parent(s_bt_power_label));
    assert(s_bt_eco != 0 && host_bt_powered != 0);
    assert(strcmp(lv_label_get_text(s_bt_power_label), "BLE: ECO / SCREEN OFF") == 0);
    screenshot("bluetooth_eco");
    resets = host_log_reset;
    disables = host_bt_disable_calls;
    app_weather_get_snapshot(&before);
    set_screen_on(0);
    host_tick += UI_BT_ECO_QUIET_MS - 1;
    App_UI_ServiceSleeping();
    assert(host_bt_powered != 0);

    /* 接收分包、日志导出、在途TX和初始化都延后ECO断电。 */
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    App_UI_ServiceSleeping();
    host_tick += 100;
    host_bluetooth_feed((const uint8_t *)"34:56\n", 6);
    App_UI_ServiceSleeping();
    assert(app_clock_get_received_count() == frames + 1 && host_log_reset == resets);
    host_log_busy = 1;
    host_tick += 2 * UI_BT_ECO_QUIET_MS;
    App_UI_ServiceSleeping();
    assert(host_bt_powered != 0 && host_log_reset == resets);
    host_log_busy = 0;
    host_bt_tx_busy = 1;
    host_tick += 2 * UI_BT_ECO_QUIET_MS;
    App_UI_ServiceSleeping();
    assert(host_bt_powered != 0);
    host_bt_tx_busy = 0;
    host_bt_pending = 1;
    host_tick += 2 * UI_BT_ECO_QUIET_MS;
    App_UI_ServiceSleeping();
    assert(host_bt_powered != 0);
    host_bt_pending = 0;
    host_tick += UI_BT_ECO_QUIET_MS - 1;
    App_UI_ServiceSleeping();
    assert(host_bt_powered != 0);
    ++host_tick;
    App_UI_ServiceSleeping();
    assert(host_bt_powered == 0 && s_bt_enabled == 0 && host_bt_disable_calls == disables + 1);
    assert(host_log_reset == resets + 1 && s_screen_on == 0);
    app_weather_get_snapshot(&after);
    assert(after.valid == before.valid && after.received_ms == before.received_ms);
    enables = host_bt_enable_calls;
    set_screen_on(1);
    assert(s_bt_enabled != 0 && host_bt_enable_calls == enables + 1);

    /* SPORT始终断电；退出时遵守当前ECO/屏幕选择。 */
    s_sport_mode = 1;
    bluetooth_update_power();
    assert(host_bt_powered == 0);
    set_screen_on(0);
    host_tick += UI_BT_ECO_QUIET_MS;
    s_sport_mode = 0;
    bluetooth_update_power();
    assert(host_bt_powered == 0);
    set_screen_on(1);
    assert(host_bt_powered != 0);

    /* 32位tick回绕后仍完整等待安静期，切回保连立即恢复供电。 */
    host_tick = UINT32_MAX - 500;
    set_screen_on(0);
    host_tick += UI_BT_ECO_QUIET_MS - 1;
    App_UI_ServiceSleeping();
    assert(host_bt_powered != 0);
    ++host_tick;
    App_UI_ServiceSleeping();
    assert(host_bt_powered == 0);
    bluetooth_power_cb(NULL);
    assert(s_bt_eco == 0 && host_bt_powered != 0);
    set_screen_on(1);
    create_page(PAGE_WATCH, 0, 0);
    advance(220);
    puts("PASS: BLE ECO UI, quiet grace, fragmented RX, export/TX/init deferral, SPORT override, cached weather and tick-wrap reconnect.");
}

static void touch_idle_tests(void)
{
    uint8_t brightness = s_brightness;
    uint32_t reads, flushes, touches, second;
    unsigned enables = host_bt_enable_calls, disables = host_bt_disable_calls;
    lv_point_t frozen_hand;
    char frozen_text[16];

    assert(s_page == PAGE_WATCH && s_screen_on);
    host_wake = GPIO_PIN_RESET;
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(60);
    assert(App_UI_IsWakeKeyActive() == 0);
    lv_disp_trig_activity(NULL);
    advance(9980);
    assert(s_screen_on && host_light == brightness);
    advance(40);
    assert(!s_screen_on && host_light == 0 && s_brightness == brightness);
    assert(host_idle != 0 && host_bt_powered != 0);
    assert(host_lcd_sleeping != 0 && host_lcd_sleeps != 0);
    assert(_lv_disp_get_refr_timer(NULL)->paused != 0);
    reads = host_rtc_read_calls;
    flushes = host_flush_calls;
    touches = host_touch_reads;
    frozen_hand = s_second_points[1];
    strcpy(frozen_text, clock_text());
    advance(61000);
    assert(host_tp_sleeping != 0 && host_tp_sleeps != 0);
    assert(host_rtc_read_calls == reads && host_flush_calls == flushes && host_touch_reads == touches);
    assert(strcmp(clock_text(), frozen_text) == 0);
    assert(memcmp(&frozen_hand, &s_second_points[1], sizeof(frozen_hand)) == 0);

    /* 普通熄屏保持蓝牙接收；手机校时不会亮屏，触摸仍被禁止。 */
    host_bluetooth_feed((const uint8_t *)"23:59:58\n", 9);
    advance(3000);
    assert(host_bt_rx_pos == host_bt_rx_size && host_rtc_read_calls == reads && s_screen_on == 0);
    host_point.x = 5;
    host_point.y = 275;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(80);
    assert(!s_screen_on && host_light == 0 && host_touch_reads == touches);
    host_wake = GPIO_PIN_SET;
    poll_wake_key();
    assert(App_UI_IsWakeKeyActive() != 0);
    advance(60);
    assert(s_screen_on && host_light == brightness && host_bt_powered != 0);
    assert(App_UI_IsWakeKeyActive() != 0);
    advance(40); /* 显示和触摸恢复后，给UI定时器一次刷新当前RTC的机会。 */
    assert(host_lcd_sleeping == 0 && host_tp_sleeping == 0 && host_tp_wakes != 0);
    assert(host_rtc_read_calls > reads && host_idle == 0 && host_motion == 0);
    second = (host_rtc_seconds + (uint32_t)(host_tick - host_rtc_tick) / 1000) % 86400;
    assert(s_last_second == second && s_page == PAGE_WATCH);
    host_wake = GPIO_PIN_RESET;
    poll_wake_key();
    assert(App_UI_IsWakeKeyActive() != 0);
    advance(60);
    assert(App_UI_IsWakeKeyActive() == 0);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(60);

    /* 亮屏期间持续触摸算活动，松开后重新计满10秒。 */
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(10200);
    assert(s_screen_on && host_light == brightness);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(60);
    lv_disp_trig_activity(NULL);
    advance(10020);
    assert(!s_screen_on && host_light == 0);
    host_wake = GPIO_PIN_SET;
    advance(60);
    host_wake = GPIO_PIN_RESET;
    advance(60);
    assert(s_screen_on && s_page == PAGE_WATCH && host_light == brightness);

    /* 活动时间跨32位tick回绕，仍按10秒熄屏，触摸不能唤醒。 */
    host_tick = UINT32_MAX - 2500;
    lv_disp_trig_activity(NULL);
    advance(9980);
    assert(s_screen_on && host_light == brightness);
    advance(40);
    assert(!s_screen_on && host_light == 0);
    tap(5, 275);
    assert(!s_screen_on && host_light == 0);
    host_wake = GPIO_PIN_SET;
    advance(60);
    host_wake = GPIO_PIN_RESET;
    advance(60);
    assert(s_screen_on && host_light == brightness && s_page == PAGE_WATCH);
    assert(host_bt_enable_calls == enables && host_bt_disable_calls == disables);
    bluetooth_sleep_tests();
    bluetooth_eco_tests();
    puts("PASS: 10s idle and PA4 sleep keep BLE power/RX without restarting UART; LCD/touch sleep and RTC wake restore survive tick wrap.");
}

static void uptime_report_tests(void)
{
    unsigned calls = host_bt_tx_calls;
    host_heart_samples_count = 0;
    s_bt_phone_ready = s_bt_uptime_sent = 0;
    s_bt_clock_frames = app_clock_get_received_count();
    host_previous_seconds = 90061;
    host_previous_valid = 1;
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls); /* 有旧记录也要等手机。 */
    host_bluetooth_feed((const uint8_t *)"ER+7\r\n25:00\n", 12);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls && !s_bt_phone_ready);
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls);
    host_bluetooth_feed((const uint8_t *)"34:56\n", 6);
    bluetooth_receive();
    assert(host_log_rx != 0 && host_log_tx != 0);
    host_log_busy = 1;
    host_tick += UI_TOUCH_IDLE_MS;
    bluetooth_send_heart();
    poll_touch_idle();
    assert(s_screen_on != 0 && host_bt_tx_calls == calls); /* 导出保持供电并优先于开机报告/波形。 */
    host_log_busy = 0;
    host_bt_tx_busy = 1;
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls && s_bt_phone_ready);
    host_bt_tx_busy = 0;
    host_bt_tx_result = HAL_BUSY;
    bluetooth_send_heart();
    assert(!s_bt_uptime_sent && host_bt_tx_calls == calls + 1);
    host_bt_tx_result = HAL_OK;
    bluetooth_send_heart();
    assert(s_bt_uptime_sent && host_bt_tx_calls == calls + 2);
    assert(strcmp((const char *)host_bt_tx_data, "UPTIME_PREV,90061,25:01:01\r\n") == 0);
    calls = host_bt_tx_calls;
    host_previous_seconds = 600;
    host_bluetooth_feed((const uint8_t *)"12:34:56\n", 9);
    bluetooth_receive();
    bluetooth_send_heart();
    bluetooth_update_power();
    assert(host_log_reset != 0);
    host_bluetooth_feed((const uint8_t *)"12:34:56\n", 9);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls); /* 重连或新存档均不重复报告。 */

    s_bt_uptime_sent = 0;
    host_bt_tx_result = HAL_ERROR;
    bluetooth_send_heart();
    assert(!s_bt_uptime_sent && !s_bt_phone_ready && host_bt_tx_calls == calls + 1);
    host_bt_tx_result = HAL_OK;
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls + 1);
    host_bluetooth_feed((const uint8_t *)"12:34:56", 8); /* 无换行，等待原200ms帧结束。 */
    bluetooth_receive();
    bluetooth_send_heart();
    assert(!s_bt_uptime_sent);
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    s_sport_mode = 1;
    bluetooth_send_heart();
    assert(!s_bt_uptime_sent && host_bt_tx_calls == calls + 1);
    s_sport_mode = 0;
    s_screen_on = 0;
    bluetooth_send_heart();
    assert(s_bt_uptime_sent && host_bt_tx_calls == calls + 2);
    s_screen_on = 1;
    bluetooth_send_heart();
    assert(s_bt_uptime_sent && host_bt_tx_calls == calls + 2);

    s_bt_uptime_sent = 0;
    host_previous_valid = 0;
    bluetooth_send_heart();
    assert(!s_bt_uptime_sent && host_bt_tx_calls == calls + 2);
    if (APP_DEBUG_TELEMETRY != 0)
    {
        host_heart_samples_count = 1;
        bluetooth_send_heart();
        assert(host_async_active);
        host_previous_valid = 1;
        bluetooth_send_heart(); /* 回收心率异步结果后再发送旧运行时间。 */
        assert(!host_async_active && s_bt_uptime_sent);
    }
    host_previous_valid = 0;
    puts("PASS: restored uptime waits for valid phone sync, handles fragments/idle/BUSY/failure, reports once per boot in quiet/debug builds and shares heart TX ownership.");
}

static void state_report_tests(void)
{
    unsigned calls = host_bt_tx_calls;
    uint32_t sets = host_rtc_set_calls;
    uint32_t frames = app_clock_get_received_count();
    app_weather_struct before, after;
    unsigned i;
    app_weather_get_snapshot(&before);
    host_bluetooth_feed((const uint8_t *)"STATE,", 6);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls);
    host_bluetooth_feed((const uint8_t *)"READ\n", 5);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_async_active && strstr((const char *)host_bt_tx_data, "STATE,RTC,READY,") != NULL);
    bluetooth_send_heart();
    for (i = 1; i < 4; ++i)
    {
        host_tick += 100;
        bluetooth_send_heart();
        assert(host_async_active);
        if (i == 1)
        {
            assert(strstr((const char *)host_bt_tx_data, "STATE,CLOCK,UP_MS,") != NULL);
        }
        else if (i == 2)
        {
            assert(strstr((const char *)host_bt_tx_data, "STATE,WEATHER,VALID,") != NULL);
        }
        else
        {
            assert(strcmp((const char *)host_bt_tx_data, "STATE,END\r\n") == 0);
        }
        bluetooth_send_heart();
        assert(host_async_active == 0);
    }
    app_weather_get_snapshot(&after);
    assert(memcmp(&before, &after, sizeof(before)) == 0);
    assert(host_rtc_set_calls == sets && app_clock_get_received_count() == frames);
    assert(host_bt_tx_calls == calls + 4);
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls + 4);
    puts("PASS: STATE query traverses shared UI RX/TX without changing weather, writing RTC or counting a phone time sync.");
}

static void reset_report_tests(void)
{
    unsigned calls = host_bt_tx_calls;
    unsigned i;
    host_bt_pending = 0;
    host_rtc_pending = 0;
    host_bt_tx_result = HAL_OK;
    host_heart_samples_count = 0;
    host_previous_valid = 0;
    assert(host_async_active == 0);
    host_bluetooth_feed((const uint8_t *)"RESET,", 6);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls);
    host_bluetooth_feed((const uint8_t *)"READ\n", 5);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_async_active && host_bt_tx_calls == calls + 1);
    assert(strstr((const char *)host_bt_tx_data, "RESET,CAUSE,VALID,1,FLAGS,0x24000000,KIND,1") != NULL);
    host_log_busy = 1; /* 另一个命令排队也不能挡住本报告收取自己的TX结果。 */
    bluetooth_send_heart();
    assert(host_async_active == 0);
    host_log_busy = 0;
    for (i = 0; i < 4; ++i)
    {
        host_tick += 100;
        bluetooth_send_heart();
        assert(host_async_active);
        bluetooth_send_heart();
        assert(host_async_active == 0);
    }
    assert(host_bt_tx_calls == calls + 5);
    assert(strcmp((const char *)host_bt_tx_data, "RESET,END\r\n") == 0);
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls + 5);
    puts("PASS: RESET query traverses the real UI RX/TX path in quiet/debug builds and collects its TX while a log command waits.");
    calls = host_bt_tx_calls;
    host_bluetooth_feed((const uint8_t *)"RESET,HIS", 9);
    bluetooth_receive();
    bluetooth_send_heart();
    assert(host_bt_tx_calls == calls);
    host_bluetooth_feed((const uint8_t *)"TORY\n", 5);
    bluetooth_receive();
    host_tick += 100;
    bluetooth_send_heart();
    assert(host_async_active && host_bt_tx_calls == calls + 1);
    assert(strcmp((const char *)host_bt_tx_data, "RESET,HISTORY,BEGIN,0,0,1\r\n") == 0);
    bluetooth_send_heart();
    assert(host_async_active == 0);
    host_tick += 100;
    bluetooth_send_heart();
    assert(host_async_active && host_bt_tx_calls == calls + 2);
    assert(strcmp((const char *)host_bt_tx_data, "RESET,HISTORY,END,0\r\n") == 0);
    bluetooth_send_heart();
    assert(host_async_active == 0);
    puts("PASS: fragmented RESET,HISTORY reaches the real UI RX/TX path in quiet/debug builds.");
    state_report_tests();
}

int main(int argc, char **argv)
{
    static lv_disp_draw_buf_t buffer;
    static lv_disp_drv_t display;
    lv_coord_t saved_scroll;
    unsigned iteration;
    lv_mem_monitor_t before, after;
    lv_obj_t *cached_main[2];
    uint8_t cached_test = argc > 1 && strcmp(argv[1], "cached_weather") == 0;
    if (cached_test == 0)
    {
        bluetooth_clear_frame_tests();
    }
    lv_init();
    lv_disp_draw_buf_init(&buffer, draw_buffer, NULL, 240 * 12);
    lv_disp_drv_init(&display);
    display.hor_res = 240;
    display.ver_res = 280;
    display.draw_buf = &buffer;
    display.flush_cb = flush;
    lv_disp_drv_register(&display);
    app_touch_init();
    lv_port_indev_init();
    host_wake = GPIO_PIN_SET; /* 模拟按住 SW2 开机。 */
    App_UI_Create();
    if (cached_test != 0)
    {
        app_weather_struct cached = {0};
        static const char fresh[] = "WEATHER,3,-53,81\n";
        cached.valid = 1;
        cached.code = 3;
        cached.temperature_x10 = -53;
        cached.humidity_rh = 81;
        cached.received_ms = UINT32_MAX - 123;
        app_weather_restore(&cached);
        create_page(PAGE_WEATHER, 0, 0);
        refresh_weather();
        assert(strcmp(lv_label_get_text(s_weather_value), "-5.3 C") == 0);
        assert(strstr(lv_label_get_text(s_weather_label), "CACHED / UPDATE VIA BLE") != NULL);
        assert(strstr(lv_label_get_text(s_weather_label), "min ago") == NULL);
        assert(lv_color_to32(lv_obj_get_style_text_color(s_weather_condition, 0)) == lv_color_to32(lv_color_hex(UI_MUTED)));
        advance_active(220);
        assert(s_transition == 0);
        screenshot("cached_weather");
        app_weather_receive((const uint8_t *)fresh, sizeof(fresh) - 1, lv_tick_get());
        refresh_weather();
        assert(strstr(lv_label_get_text(s_weather_label), "0 min ago") != NULL);
        assert(strstr(lv_label_get_text(s_weather_label), "CACHED") == NULL);
        assert(lv_color_to32(lv_obj_get_style_text_color(s_weather_condition, 0)) == lv_color_to32(lv_color_hex(UI_TEXT)));
        puts("PASS: restored weather displays cached status with unknown age; a matching phone update restores freshness.");
        return 0;
    }
    if (APP_DEBUG_TELEMETRY == 0)
    {
        host_heart_samples_count = 2;
        bluetooth_send_heart();
        bluetooth_send_test_cb(NULL);
        assert(host_heart_samples_count == 2 && host_bt_tx_calls == 0);
        advance(100);
        assert(host_bt_tx_calls == 0);
        assert(strcmp(clock_text(), "10:10") == 0);
        mag_calibrate_cb(NULL);
        assert(host_mag_cal_state == APP_MAG_CAL_RUNNING);
        mag_calibrate_cb(NULL);
        assert(host_mag_cal_state == APP_MAG_CAL_DONE && host_mag_calibrated);
        host_bluetooth_feed((const uint8_t *)"11:22:33\n", 9);
        advance(120);
        assert(host_bt_rx_pos == host_bt_rx_size && host_bt_tx_calls == 0);
        assert(strcmp(clock_text(), "11:22") == 0);
        touch_idle_tests();
        uptime_report_tests();
        reset_report_tests();
        bluetooth_arrival_time_tests();
        puts("PASS: performance UI emits no debug TX; magnetic calibration, incoming clock sync and UI still work.");
        return 0;
    }
    assert(host_flush_calls == 0);
    lv_refr_now(NULL); /* 首帧刷新不依赖后续UI定时器或慢模块初始化。 */
    assert(host_flush_calls > 0);
    printf("PASS: immediate first-frame refresh before the UI service loop.\n");
    advance(30);
    assert(s_page == PAGE_WATCH && host_light == 50);
    assert(strcmp(clock_text(), "10:10") == 0);
    assert_rtc_seconds(0);
    assert(!lv_obj_has_flag(s_body, LV_OBJ_FLAG_SCROLLABLE) && lv_obj_get_scroll_bottom(s_body) <= 0);
    assert(lv_obj_get_style_radius(s_body, 0) == 0 && !lv_obj_get_style_clip_corner(s_body, 0));
    assert(lv_obj_get_width(s_body) == 240 && lv_obj_get_height(s_body) == 280);
    assert(lv_obj_get_x(s_body) == 0 && lv_obj_get_y(s_body) == 0);
    assert(framebuffer[0].full == lv_color_hex(UI_BG).full);
    assert_static_refresh();
    host_wake = GPIO_PIN_RESET;
    drag(110, 216, 0, -100);
    drag(110, 100, 0, 120);
    assert(s_page == PAGE_WATCH && lv_obj_get_scroll_y(s_body) == 0 && s_history_count == 0);
    assert(s_screen_on && host_light == 50); /* 开机按键松开不熄屏。 */
    screenshot("watch");
    /* 三针来自同一RTC快照，秒针在0/15/30秒及分钟进位时跳格。 */
    assert(s_second_points[1].x == 100 && s_second_points[1].y == 18);
    assert(s_hour_points[1].x < 100 && s_hour_points[1].y < 100);
    assert(s_minute_points[1].x > 100 && s_minute_points[1].y < 100);
    assert_rtc_seconds(0);
    advance_active(15000);
    assert_rtc_seconds(15);
    assert(s_second_points[1].x == 182 && s_second_points[1].y == 100);
    screenshot("watch_seconds");
    advance_active(15000);
    assert_rtc_seconds(30);
    assert(s_second_points[1].x == 100 && s_second_points[1].y == 182);
    advance_active(30000);
    assert_rtc_seconds(0);
    assert(strcmp(clock_text(), "10:11") == 0);
    assert_rtc_seconds(0);
    /* 同一RTC秒内不读取摘要、不传输像素；下一秒统一更新指针和电池。 */
    {
        watch_time_struct time;
        lv_point_t previous = s_second_points[1];
        uint32_t reads = host_battery_reads;
        uint32_t heart_reads = host_heart_reads;
        uint32_t calls;
        assert(watch_rtc_get_time(&time) == HAL_OK);
        host_rtc_seconds = time.hours * 3600 + time.minutes * 60 + time.seconds;
        host_rtc_tick = host_tick;
        lv_refr_now(NULL);
        calls = host_flush_calls;
        host_battery.voltage_mv = 3860;
        advance(850);
        assert(host_battery_reads == reads && host_heart_reads == heart_reads);
        assert(host_flush_calls == calls);
        assert(s_second_points[1].x == previous.x && s_second_points[1].y == previous.y);
        advance(250);
        assert(host_battery_reads == reads + 1 && host_heart_reads == heart_reads + 1);
        assert(strcmp(lv_label_get_text(s_battery_label), "~50% 3.86V") == 0);
        assert(s_second_points[1].x != previous.x || s_second_points[1].y != previous.y);
        host_battery.voltage_mv = 3851;
        advance(1000);
    }
    host_battery.valid = 0;
    refresh_battery();
    lv_refr_now(NULL);
    assert(strcmp(lv_label_get_text(s_battery_label), "--%") == 0);
    assert(lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN));
    screenshot("battery_waiting");
    /* 毫伏变化但显示仍相同时不重绘；显示值变化、ADC故障和恢复必须更新。 */
    host_battery.valid = 1;
    host_battery.voltage_mv = 3851;
    refresh_battery();
    lv_refr_now(NULL);
    assert(strcmp(lv_label_get_text(s_battery_label), "~50% 3.85V") == 0);
    assert_static_refresh();
    {
        uint32_t calls = host_flush_calls;
        host_battery.voltage_mv = 3859;
        refresh_battery();
        lv_refr_now(NULL);
        assert(host_flush_calls == calls);
        host_battery.voltage_mv = 3860;
        refresh_battery();
        lv_refr_now(NULL);
        assert(host_flush_calls > calls && strcmp(lv_label_get_text(s_battery_label), "~50% 3.86V") == 0);
    }
    host_battery.valid = 0;
    host_battery.result = HAL_TIMEOUT;
    refresh_battery();
    lv_refr_now(NULL);
    assert(strcmp(lv_label_get_text(s_battery_label), "ADC ERR") == 0);
    assert(lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN));
    assert(lv_obj_get_style_text_color(s_battery_label, LV_PART_MAIN).full == lv_color_hex(UI_RED).full);
    assert_static_refresh();
    screenshot("battery_error");
    host_battery.valid = 1;
    host_battery.result = HAL_OK;
    host_battery.voltage_mv = 3851;
    refresh_battery();
    lv_refr_now(NULL);
    assert(strcmp(lv_label_get_text(s_battery_label), "~50% 3.85V") == 0);
    assert(lv_obj_get_style_text_color(s_battery_label, LV_PART_MAIN).full == lv_color_hex(UI_TEXT).full);
    assert(!lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN));
    assert(lv_bar_get_value(s_battery_bar) == 50);
    {
        lv_area_t area;
        lv_obj_get_coords(s_battery_bar, &area);
        assert(framebuffer[(area.y1 + 6) * 240 + area.x1 + 6].full == lv_color_hex(UI_BATTERY_GREEN).full);
        assert(framebuffer[(area.y1 + 6) * 240 + area.x2 - 5].full == lv_color_hex(UI_CARD).full);
    }
    /* 百分比变化必须刷新；0/100%单行显示，低电量及恢复使用正确颜色。 */
    {
        uint32_t calls = host_flush_calls;
        host_battery.percent = 20;
        refresh_battery();
        lv_refr_now(NULL);
        assert(host_flush_calls > calls);
        assert(strcmp(lv_label_get_text(s_battery_label), "~20% 3.85V") == 0);
        assert(lv_obj_get_style_text_color(s_battery_label, LV_PART_MAIN).full == lv_color_hex(UI_RED).full);
        assert(lv_bar_get_value(s_battery_bar) == 20);
        assert(lv_obj_get_style_bg_color(s_battery_bar, LV_PART_INDICATOR).full == lv_color_hex(UI_RED).full);
        screenshot("battery_low");
        host_battery.percent = 0;
        refresh_battery();
        assert(strcmp(lv_label_get_text(s_battery_label), "~0% 3.85V") == 0);
        assert(lv_bar_get_value(s_battery_bar) == 0);
        screenshot("battery_empty");
        host_battery.percent = 100;
        refresh_battery();
        lv_obj_update_layout(s_screen);
        assert(strcmp(lv_label_get_text(s_battery_label), "~100% 3.85V") == 0);
        assert(lv_obj_get_height(s_battery_label) == lv_font_montserrat_14.line_height);
        assert(lv_obj_get_style_text_color(s_battery_label, LV_PART_MAIN).full == lv_color_hex(UI_TEXT).full);
        assert(lv_bar_get_value(s_battery_bar) == 100);
        assert(!lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN));
        assert(lv_obj_get_style_bg_color(s_battery_bar, LV_PART_INDICATOR).full == lv_color_hex(UI_BATTERY_GREEN).full);
        screenshot("battery_full");
        host_battery.percent = 50;
        refresh_battery();
        lv_refr_now(NULL);
    }
    /* 真实按下及拖动刻度盘：背景保持透明，纵向滑动不误入校时页。 */
    host_point.x = 120;
    host_point.y = 80;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(40);
    lv_refr_now(NULL);
    assert(framebuffer[19 * 240 + 38].full == lv_color_hex(UI_BG).full);
    host_point.y = 160;
    advance(60);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(230);
    assert(s_page == PAGE_WATCH && s_history_count == 0);
    key_back();
    assert(s_page == PAGE_WATCH && s_history_count == 0);
    /* Fixed watch -> feature grid; left swipes stop at the last main page. */
    drag(180, 140, -120, 0);
    assert(s_page == PAGE_SETTINGS && s_history_count == 1);
    assert(lv_obj_get_child_cnt(s_body) == 8);
    assert(!s_brightness_label && !s_angles[0] && !s_pressure && !s_altitude && !s_heart_card && !s_aht21_card);
    open_feature(PAGE_HEART);
    assert(strcmp(lv_label_get_text(s_battery_label), "~50%") == 0);
    host_battery.percent = 100;
    refresh_battery();
    lv_obj_update_layout(s_screen);
    assert(strcmp(lv_label_get_text(s_battery_label), "~100%") == 0);
    assert(lv_obj_get_height(s_battery_label) == lv_font_montserrat_10.line_height);
    host_battery.percent = 50;
    refresh_battery();

    assert_static_refresh();
    /* 真实心率卡片的按钮、有效值和异常状态，不让旧 BPM 留在无效状态。 */
    {
        lv_area_t area;
        assert(s_heart_card != NULL && strcmp(lv_label_get_text(s_heart_value), "--") == 0);
        lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
        advance(120);
        lv_obj_get_coords(s_heart_button, &area);
        tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
        assert(host_heart.enabled && strcmp(lv_label_get_text(s_heart_button), "STOP") == 0);
        assert(strstr(lv_label_get_text(s_heart_status), "MEASURING") != NULL);
        screenshot("heart_measuring");
        host_heart.state = APP_HEART_VALID;
        host_heart.bpm = 72;
        host_heart.raw = 12345;
        host_heart.signal_x10 = 160;
        host_heart.interval_ms = 830;
        host_heart_samples[0].raw = 30000;
        host_heart_samples[0].filtered = -42.5f;
        host_heart_samples[0].dc = 30042.5f;
        host_heart_samples[0].bpm = 72;
        host_heart_samples[0].interval_ms = 830;
        host_heart_samples[0].elapsed_ms = 30;
        host_heart_samples[0].sequence = 65535;
        host_heart_samples[0].state = APP_HEART_VALID;
        host_heart_samples[1] = host_heart_samples[0];
        host_heart_samples[1].raw = 65535;
        host_heart_samples[1].elapsed_ms = 60;
        host_heart_samples[1].sequence = 0;
        host_heart_samples_count = 2;
        host_bt_pending = 1;
        bluetooth_send_heart();
        assert(host_heart_samples_count == 2); /* AT初始化期间不取走心率队列。 */
        host_bt_rx[0] = 'T';
        host_bt_rx_size = 1;
        host_bt_rx_pos = 0;
        bluetooth_receive();
        assert(host_bt_rx_pos == 0); /* AT响应仍由初始化状态机独占。 */
        bluetooth_clear_cb(NULL);
        assert(host_bt_rx_pos == 0); /* 清屏也不清掉AT响应。 */
        host_bt_pending = 0;
        host_bt_tx_busy = 1;
        bluetooth_send_heart();
        assert(host_heart_samples_count == 2); /* 诊断在途时保留队列，稍后发送。 */
        host_bt_tx_busy = 0;
        host_rtc_pending = 1;
        bluetooth_receive();
        assert(host_bt_rx_pos == 0); /* 不向尚未就绪的RTC提交自动校时。 */
        host_rtc_pending = 0;
        host_bt_rx_size = 0;
        advance(120);
        assert(host_bt_tx_size == 72);
        {
            static const float expected[2][8] = {{30000.0f, -42.5f, 30042.5f, 72.0f, 830.0f, 0.03f, 65535.0f, 2.0f},
                                                  {65535.0f, -42.5f, 30042.5f, 72.0f, 830.0f, 0.06f, 0.0f, 2.0f}};
            unsigned frame, channel, offset;
            uint32_t bits;
            float value;
            FILE *stream;
            for (frame = 0; frame < 2; ++frame)
            {
                for (channel = 0; channel < 8; ++channel)
                {
                    offset = frame * 36 + channel * 4;
                    bits = (uint32_t)host_bt_tx_data[offset] | ((uint32_t)host_bt_tx_data[offset + 1] << 8) |
                           ((uint32_t)host_bt_tx_data[offset + 2] << 16) | ((uint32_t)host_bt_tx_data[offset + 3] << 24);
                    memcpy(&value, &bits, sizeof(value));
                    assert(value == expected[frame][channel]);
                }
                assert(memcmp(host_bt_tx_data + frame * 36 + 32, "\x00\x00\x80\x7F", 4) == 0);
            }
            stream = fopen("tests/ui/output/heart-telemetry.bin", "wb");
            assert(stream != NULL && fwrite(host_bt_tx_data, 1, host_bt_tx_size, stream) == host_bt_tx_size);
            assert(fclose(stream) == 0);
        }
        assert(strcmp(lv_label_get_text(s_heart_value), "72") == 0);
        assert(strstr(lv_label_get_text(s_heart_status), "HRS1 12345  AC 16.0") != NULL);
        assert(strstr(lv_label_get_text(s_heart_status), "PERIOD 830 ms") != NULL);
        assert(strcmp(lv_label_get_text(s_heart_summary), "TRACKING") == 0);
        assert(lv_obj_has_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN));
        assert_in_body(s_heart_card);
        assert_in_body(lv_obj_get_parent(s_heart_trend.chart));
        assert_in_body(lv_obj_get_parent(s_heart_details_title));
        screenshot("heart");
        tap_object(lv_obj_get_parent(s_heart_details_title));
        assert(!lv_obj_has_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN));
        assert_in_body(s_heart_details);
        assert(host_heart.enabled && host_heart.bpm == 72);
        screenshot("heart_details");
        tap_object(lv_obj_get_parent(s_heart_details_title));
        assert(lv_obj_has_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN));
        assert(lv_obj_get_scroll_y(s_body) == 0);
        assert_in_body(lv_obj_get_parent(s_heart_trend.chart));
        {
            unsigned i;
            uint16_t start;
            assert(lv_chart_get_type(s_heart_trend.chart) == LV_CHART_TYPE_BAR);
            for (i = 0; i < UI_TREND_POINTS; ++i)
            {
                host_heart.bpm = 60 + i * 4;
                advance_active(1000);
                assert(trend_latest(&s_heart_trend) == host_heart.bpm);
            }
            start = s_heart_trend.series->start_point;
            refresh_data();
            assert(s_heart_trend.series->start_point == start);
            assert(((lv_chart_t *)s_heart_trend.chart)->ymax[0] == 200);
            lv_obj_scroll_to_view(lv_obj_get_parent(s_heart_trend.chart), LV_ANIM_OFF);
            advance(30);
            screenshot("heart_history");
            lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
            host_heart.bpm = 72;
        }
        host_heart.state = APP_HEART_NO_SIGNAL;
        advance(1000);
        assert(trend_latest(&s_heart_trend) == LV_CHART_POINT_NONE);
        assert(strcmp(lv_label_get_text(s_heart_value), "--") == 0);
        assert(strstr(lv_label_get_text(s_heart_status), "NO SIGNAL") != NULL);
        screenshot("heart_no_signal");
        host_heart.state = APP_HEART_ERROR;
        host_heart.enabled = 0;
        host_heart.fault.kind = EM7028_FAULT_READ;
        host_heart.fault.reg = 0x29;
        advance(120);
        assert(strstr(lv_label_get_text(s_heart_status), "UNAVAILABLE") != NULL);
        assert(strstr(lv_label_get_text(s_heart_status), "I2C READ REG 29") != NULL);
        assert(strcmp(lv_label_get_text(s_heart_summary), "UNAVAILABLE") == 0);
        assert(strcmp(lv_label_get_text(s_heart_button), "START") == 0);
        screenshot("heart_unavailable");
        host_heart.fault.kind = EM7028_FAULT_CONFIG;
        host_heart.fault.reg = 0x0D;
        host_heart.fault.value = 0x32;
        advance(120);
        assert(strstr(lv_label_get_text(s_heart_status), "CONFIG CHECK REG 0D") != NULL);
        assert(strstr(lv_label_get_text(s_heart_status), "VALUE 32") != NULL);
        tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
        assert(host_heart.enabled);
        tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
        assert(!host_heart.enabled && strstr(lv_label_get_text(s_heart_status), "STANDBY") != NULL);
        lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
        advance(120);
    }
    key_back();
    drag(170, 100, -110, 0);
    assert(s_page == PAGE_SETTINGS && s_history_count == 1);
    screenshot("settings");
    open_feature(PAGE_DISPLAY);
    screenshot("display");
    /* Slider is isolated from page navigation. */
    drag(77, 129, 85, 0);
    assert(s_page == PAGE_DISPLAY && host_light > 30 && s_history_count == 2);
    /* SW2 消抖、按住不重复、亮度恢复，以及熄屏期间禁止触摸和返回操作。 */
    {
        uint8_t saved_brightness = host_light;
        uint8_t saved_history = s_history_count;
        lv_coord_t screen_scroll = lv_obj_get_scroll_y(s_body);
        host_wake = GPIO_PIN_SET;
        advance(10);
        host_wake = GPIO_PIN_RESET;
        advance(60);
        assert(s_screen_on && host_light == saved_brightness);
        host_wake = GPIO_PIN_SET;
        advance(20);
        assert(s_screen_on && host_light == saved_brightness);
        advance(40);
        assert(!s_screen_on && host_light == 0); /* 松手前已触发，不需要长按。 */
        {
            unsigned int off_flushes = host_flush_calls;
            advance(1000);
            assert(host_flush_calls == off_flushes); /* 熄屏不提交新像素。 */
            host_bluetooth_feed((const uint8_t *)"WEATHER,3,263,80\n", 17);
            advance(60);
            assert(host_bt_powered != 0 && host_bt_rx_pos == host_bt_rx_size && host_flush_calls == off_flushes);
            bluetooth_clear_cb(NULL); /* 不影响后面的原蓝牙日志断言。 */
        }
        assert(!s_screen_on && host_light == 0); /* 持续按住不重复触发。 */
        host_wake = GPIO_PIN_RESET;
        advance(60);
        assert(!s_screen_on && host_light == 0);
        drag(77, 129, 85, 0);
        drag(60, 140, 120, 0);
        key_back();
        assert(!s_screen_on && host_light == 0 && s_brightness == saved_brightness);
        assert(s_page == PAGE_DISPLAY && s_history_count == saved_history);
        assert(lv_obj_get_scroll_y(s_body) == screen_scroll);
        host_point.x = 110;
        host_point.y = 140;
        host_touch = LV_INDEV_STATE_PRESSED;
        host_wake = GPIO_PIN_SET;
        advance(60);
        assert(s_screen_on && host_light == saved_brightness); /* 按下即亮屏，尚未松手。 */
        assert(host_bt_powered != 0 && host_bt_rx_pos == host_bt_rx_size);
        host_wake = GPIO_PIN_RESET;
        advance(60);
        assert(s_screen_on && host_light == saved_brightness);
        host_touch = LV_INDEV_STATE_RELEASED;
        advance(60);
        assert(s_page == PAGE_DISPLAY && s_history_count == saved_history);
        assert(lv_obj_get_scroll_y(s_body) == screen_scroll);
    }
    /* A short electrical glitch must not trigger Back. */
    host_key = GPIO_PIN_RESET;
    advance(10);
    host_key = GPIO_PIN_SET;
    advance(60);
    assert(s_page == PAGE_DISPLAY);
    key_back();
    assert(s_page == PAGE_SETTINGS);
    key_back();
    assert(s_page == PAGE_WATCH);
    drag(180, 140, -120, 0);
    drag(115, 216, 0, -100);
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) > 0);
    screenshot("settings_scrolled");
    drag(115, 100, 0, 120);
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) == 0);
    open_feature(PAGE_MOTION);
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    advance(30);
    assert(strcmp(lv_label_get_text(s_angles[2]), "5.2") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "MAG UNCAL / calibrate first") == 0);
    assert(strcmp(lv_label_get_text(s_mag_values[1]), "-14.2") == 0);
    assert(strcmp(lv_label_get_text(s_compass_status), "UNCALIBRATED") == 0);
    compass_direction_tests();
    compass_redraw_tests();
    assert_static_refresh();
    assert_in_body(s_compass);
    assert_in_body(lv_obj_get_parent(s_compass_calibration));
    screenshot("motion");
    /* 首屏校准入口只定位操作区，不会直接开始采集。 */
    tap_object(lv_obj_get_parent(s_compass_calibration));
    assert(host_mag_cal_state == APP_MAG_CAL_NONE);
    assert(lv_obj_get_scroll_y(s_body) > 0);
    assert_in_body(lv_obj_get_parent(s_mag_cal_label));
    /* Scroll to and tap the real calibration control. */
    lv_obj_scroll_to_view(lv_obj_get_parent(s_mag_cal_label), LV_ANIM_OFF);
    advance(30);
    {
        lv_area_t area;
        lv_obj_get_coords(lv_obj_get_parent(s_mag_cal_label), &area);
        tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
    }
    assert(host_mag_cal_state == APP_MAG_CAL_RUNNING);
    assert(strcmp(lv_label_get_text(s_mag_cal_label), "FINISH CALIBRATION") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / CALIBRATING") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "COLLECTING: 120") != NULL);
    assert(strcmp(lv_label_get_text(s_compass_status), "CALIBRATING") == 0);
    screenshot("motion_calibrating");
    {
        lv_area_t area;
        lv_obj_get_coords(lv_obj_get_parent(s_mag_cal_label), &area);
        tap((area.x1 + area.x2) / 2, (area.y1 + area.y2) / 2);
    }
    assert(host_mag_calibrated);
    assert(strcmp(lv_label_get_text(s_yaw_source), "MAG NORTH / fused yaw") == 0);
    assert(strcmp(lv_label_get_text(s_mag_status), "CAL OK / MAG ONLINE") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "CAL SAVED: 120") != NULL);
    assert(strcmp(lv_label_get_text(s_compass_status), "MAG NORTH") == 0);
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    advance(30);
    screenshot("compass");
    {
        unsigned enables;
        assert(s_sport_button == NULL);
        key_back();
        open_feature(PAGE_SPORT);
        assert(s_sport_button != NULL && strcmp(lv_label_get_text(s_sport_label), "START SPORT") == 0);
        assert_in_body(s_sport_button);
        tap_object(s_sport_button);
        assert(s_sport_mode != 0 && host_sport != 0 && host_bt_powered == 0);
        assert(strcmp(lv_label_get_text(s_sport_label), "STOP SPORT") == 0);
        screenshot("sport_mode");
        key_back();
        open_feature(PAGE_HEART);
        advance(120);
        assert(strcmp(lv_label_get_text(s_heart_button), "SPORT") == 0);
        assert(lv_obj_has_state(s_heart_button, LV_STATE_DISABLED));
        key_back();
        open_feature(PAGE_BLUETOOTH);
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "OFF / SPORT MODE") == 0);
        key_back();
        open_feature(PAGE_ENVIRONMENT);
        advance(120);
        assert(strcmp(lv_label_get_text(s_baro_status), "PAUSED") == 0);
        key_back();
        open_feature(PAGE_SPORT);
        assert(strcmp(lv_label_get_text(s_sport_label), "STOP SPORT") == 0);
        enables = host_bt_enable_calls;
        host_wake = GPIO_PIN_SET;
        advance(60);
        assert(s_screen_on == 0 && host_sport != 0 && host_bt_powered == 0);
        advance(1000);
        host_wake = GPIO_PIN_RESET;
        advance(60);
        host_wake = GPIO_PIN_SET;
        advance(60);
        assert(s_screen_on != 0 && host_sport != 0 && host_bt_powered == 0 && host_bt_enable_calls == enables);
        host_wake = GPIO_PIN_RESET;
        advance(60);
        tap_object(s_sport_button);
        assert(s_sport_mode == 0 && host_sport == 0 && host_bt_powered != 0);
        assert(strcmp(lv_label_get_text(s_sport_label), "START SPORT") == 0);
        key_back();
        open_feature(PAGE_MOTION);
        puts("PASS: sport button touch, page retention, automatic heart UI, paused barometer/BLE, screen-off continuity and exit power restore.");
    }
    lv_obj_scroll_to_view(s_mag_cal_status, LV_ANIM_OFF);
    advance(30);
    screenshot("motion_calibration_status");
    lv_obj_scroll_to_y(s_body, lv_obj_get_y(lv_obj_get_parent(s_mag_values[0])), LV_ANIM_OFF);
    advance(30);
    screenshot("motion_magnetic");
    host_mag_online = 0;
    advance(120);
    assert(strcmp(lv_label_get_text(s_mag_values[0]), "--") == 0);
    assert(strcmp(lv_label_get_text(s_angles[2]), "5.2") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / MAG NO DATA") == 0);
    assert(strcmp(lv_label_get_text(s_compass_status), "GYRO ONLY") == 0);
    assert(s_compass_color == UI_MUTED);
    host_mag_online = 1;
    advance(120);
    host_mag_persist_state = APP_MAG_PERSIST_RESTORED;
    advance(120);
    assert(strcmp(lv_label_get_text(s_mag_status), "EEPROM CAL / MAG ONLINE") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "CAL RESTORED / EEPROM") != NULL);
    lv_obj_scroll_to_view(s_mag_cal_status, LV_ANIM_OFF);
    advance(30);
    screenshot("motion_restored");
    host_mag_persist_state = APP_MAG_PERSIST_SAVE_FAILED;
    advance(120);
    assert(strcmp(lv_label_get_text(s_mag_status), "CAL OK / SAVE FAILED") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "RAM only") != NULL);
    assert(strcmp(lv_label_get_text(s_yaw_source), "MAG NORTH / fused yaw") == 0);
    screenshot("motion_save_failed");
    host_mag_cal_state = APP_MAG_CAL_NONE;
    host_mag_persist_state = APP_MAG_PERSIST_LOAD_FAILED;
    advance(120);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "EEPROM READ FAILED") != NULL);
    host_mag_cal_state = APP_MAG_CAL_DONE;
    host_mag_persist_state = APP_MAG_PERSIST_SAVED;
    advance(120);
    host_mag_fusion_state = APP_MAG_FUSION_FIELD_RANGE;
    advance(120);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / FIELD RANGE") == 0);
    host_mag_fusion_state = APP_MAG_FUSION_HORIZONTAL_WEAK;
    advance(120);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / HORIZ <5uT") == 0);
    host_mag_cal_state = APP_MAG_CAL_FAILED;
    advance(120);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "CAL FAILED: 120") != NULL);
    host_mag_cal_state = APP_MAG_CAL_DONE;
    host_mag_fusion_state = APP_MAG_FUSION_ACTIVE;
    advance(120);
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    advance(30);
    drag(110, 216, 0, -100);
    assert(s_page == PAGE_MOTION && lv_obj_get_scroll_y(s_body) > 0);
    screenshot("motion_scrolled");
    host_online = 0;
    advance(120);
    assert(strcmp(lv_label_get_text(s_angles[0]), "--") == 0);
    assert(strcmp(lv_label_get_text(s_imu_status), "NO DATA") == 0);
    host_online = 1;
    advance(120);
    key_back();
    open_feature(PAGE_ENVIRONMENT);
    lv_obj_scroll_to_y(s_body, 60, LV_ANIM_OFF);
    advance(30);
    saved_scroll = lv_obj_get_scroll_y(s_body);
    assert(s_page == PAGE_ENVIRONMENT && s_history_count == 2);
    assert(strcmp(lv_label_get_text(s_pressure), "1013.25") == 0);
    assert(strcmp(lv_label_get_text(s_altitude), "-0.5") == 0);
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    advance(30);
    assert_in_body(lv_obj_get_parent(s_altitude));
    assert_in_body(lv_obj_get_parent(s_altitude_trend.chart));
    screenshot("environment");
    {
        unsigned i;
        uint32_t reads;
        lv_chart_t *chart = (lv_chart_t *)s_altitude_trend.chart;
        assert(lv_chart_get_type(s_altitude_trend.chart) == LV_CHART_TYPE_BAR);
        /* 页面每5s取数，只在后台十分钟历史有新增记录时更新柱状图。 */
        reads = host_baro_reads;
        advance(1000);
        assert(host_baro_reads == reads);
        for (i = 0; i < UI_TREND_POINTS; ++i)
        {
            host_altitude = -12.0f + i * 2.0f;
            host_record_baro();
            reads = host_baro_reads;
            advance_active(5000);
            assert(host_baro_reads == reads + 1);
            assert(trend_latest(&s_altitude_trend) == (lv_coord_t)host_altitude);
        }
        assert(chart->ymin[0] < -12 && chart->ymax[0] > 34);
        lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
        advance(30);
        screenshot("altitude_history");
        host_online = 0;
        host_record_baro();
        advance_active(5000);
        assert(trend_latest(&s_altitude_trend) == LV_CHART_POINT_NONE);
        host_online = 1;
        host_altitude = NAN;
        host_record_baro();
        advance_active(5000);
        assert(trend_latest(&s_altitude_trend) == LV_CHART_POINT_NONE);
        host_altitude = -0.5f;
        advance_active(125000);
        assert(chart->ymax[0] - chart->ymin[0] >= 20);
        lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    }
    key_back();
    host_altitude = 123.0f;
    host_record_baro(); /* 页面已关闭，模拟后台采样。 */
    open_feature(PAGE_ENVIRONMENT);
    advance(120);
    assert(trend_latest(&s_altitude_trend) == 123);
    key_back();
    open_feature(PAGE_AHT21);
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    advance(120);
    assert(s_aht21_card != NULL && strcmp(lv_label_get_text(s_aht21_status), "WAITING") == 0);
    assert(strcmp(lv_label_get_text(s_humidity), "--") == 0);
    host_aht21.measuring = 1;
    advance(120);
    assert(strcmp(lv_label_get_text(s_aht21_status), "MEASURING") == 0);
    screenshot("aht21_measuring");
    host_aht21.measuring = 0;
    host_aht21.valid = 1;
    host_aht21.sample.humidity_rh = 50.0f;
    host_aht21.sample.temperature_c = 25.0f;
    host_aht21.sample_count = 12;
    advance(120);
    assert(strcmp(lv_label_get_text(s_humidity), "50.0") == 0);
    assert(strcmp(lv_label_get_text(s_air_temperature), "25.0") == 0);
    assert(strcmp(lv_label_get_text(s_aht21_status), "ONLINE / CRC OK") == 0);
    assert(strcmp(lv_label_get_text(s_aht21_samples), "SAMPLES 12 / 1s") == 0);
    assert_static_refresh();
    /* 原始值变化但一位小数相同，不刷新；跨过显示精度后必须刷新。 */
    {
        uint32_t calls = host_flush_calls;
        host_aht21.sample.humidity_rh = 50.09f;
        host_aht21.sample.temperature_c = 25.09f;
        refresh_data();
        lv_refr_now(NULL);
        assert(host_flush_calls == calls);
        host_aht21.sample.temperature_c = 25.2f;
        refresh_data();
        lv_refr_now(NULL);
        assert(host_flush_calls > calls && strcmp(lv_label_get_text(s_air_temperature), "25.2") == 0);
        host_aht21.sample.humidity_rh = 50.0f;
        host_aht21.sample.temperature_c = 25.0f;
        refresh_data();
        lv_refr_now(NULL);
        assert_static_refresh();
    }
    screenshot("aht21");
    host_aht21.sample.temperature_c = -25.0f;
    advance(120);
    assert(strcmp(lv_label_get_text(s_air_temperature), "-25.0") == 0);
    host_aht21.valid = 0;
    host_aht21.result = AHT21_CRC_ERROR;
    advance(120);
    assert(strcmp(lv_label_get_text(s_humidity), "--") == 0);
    assert(strcmp(lv_label_get_text(s_air_temperature), "--") == 0);
    assert(strcmp(lv_label_get_text(s_aht21_status), "CRC ERROR") == 0);
    screenshot("aht21_crc_error");
    host_aht21.result = AHT21_I2C_ERROR;
    advance(120);
    assert(strcmp(lv_label_get_text(s_aht21_status), "I2C ERROR") == 0);
    screenshot("aht21_unavailable");
    host_aht21.result = AHT21_CALIBRATION_ERROR;
    advance(120);
    assert(strcmp(lv_label_get_text(s_aht21_status), "NOT CALIBRATED") == 0);
    host_aht21.result = AHT21_TIMEOUT;
    advance(120);
    assert(strcmp(lv_label_get_text(s_aht21_status), "CONVERSION TIMEOUT") == 0);
    host_aht21.result = AHT21_OK;
    host_aht21.valid = 1;
    host_aht21.sample.temperature_c = 25.0f;
    advance(120);
    assert(strcmp(lv_label_get_text(s_aht21_status), "ONLINE / CRC OK") == 0);
    key_back();
    open_feature(PAGE_ENVIRONMENT);
    lv_obj_scroll_to_y(s_body, saved_scroll, LV_ANIM_OFF);
    advance(30);
    /* 详情页右滑返回入口，并恢复功能网格之前的滚动位置。 */
    saved_scroll = s_history[s_history_count - 1].scroll_y;
    drag(60, 140, 120, 0);
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) == saved_scroll);
    key_back();
    assert(s_page == PAGE_WATCH);
    /* 预热两张主页面，确认只有新页移动，随后用真实手势检查缓存复用和内存稳定。 */
    {
        lv_obj_t *watch = s_screen;
        lv_anim_t *animation;
        create_page(PAGE_SETTINGS, 0, 0);
        animation = lv_anim_get(s_screen, NULL);
        assert(animation != NULL && animation->path_cb == lv_anim_path_ease_in_out);
        animation = lv_anim_get(watch, NULL);
        assert(animation == NULL || animation->exec_cb == NULL);
        advance(200);
        assert(!s_transition);
        create_page(PAGE_WATCH, 0, 1);
        advance(200);
        assert(s_screen == watch && !s_transition);
    }
    cached_main[PAGE_WATCH] = s_main_pages[PAGE_WATCH].screen;
    cached_main[PAGE_SETTINGS] = s_main_pages[PAGE_SETTINGS].screen;
    lv_mem_monitor(&before);
    for (iteration = 0; iteration < 100; ++iteration)
    {
        drag(180, 140, -120, 0);
        assert(s_page == PAGE_SETTINGS && s_screen == cached_main[PAGE_SETTINGS]);
        key_back();
        assert(s_page == PAGE_WATCH && s_screen == cached_main[PAGE_WATCH]);
    }
    lv_mem_monitor(&after);
    assert(after.free_size >= before.free_size - 512);
    fprintf(stderr, "LVGL minimum after navigation: %u bytes\n", min_free);
    assert(min_free > 4096);
    /* Back never repeats while the key is held. */
    drag(180, 140, -120, 0);
    drag(170, 100, -110, 0);
    host_key = GPIO_PIN_RESET;
    advance(1000);
    assert(s_page == PAGE_SETTINGS);
    host_key = GPIO_PIN_SET;
    advance(250);
    assert(s_page == PAGE_WATCH);
    advance(500);
    assert(s_page == PAGE_WATCH);
    /* History is bounded even when forward navigation revisits a page. */
    key_back();
    while (s_history_count)
    {
        key_back();
    }
    assert(s_page == PAGE_WATCH);
    for (iteration = 0; iteration < 10; ++iteration)
    {
        drag(180, 140, -120, 0);
        assert(s_page == PAGE_SETTINGS);
        drag(60, 70, 120, 0);
        assert(s_page == PAGE_WATCH);
    }
    assert(s_history_count == UI_HISTORY);
    for (iteration = 0; iteration < UI_HISTORY; ++iteration)
    {
        key_back();
    }
    assert(s_page == PAGE_WATCH && s_history_count == 0);
    key_back();
    assert(s_page == PAGE_WATCH);
    /* A slow swipe with repeated samples still accumulates enough distance. */
    host_point.x = 180;
    host_point.y = 140;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(20);
    for (iteration = 0; iteration < 60; ++iteration)
    {
        --host_point.x;
        advance(10);
    }
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(250);
    assert(s_page == PAGE_SETTINGS);
    /* 在其他页面收到的数据也保留，通过真实Settings卡片进入接收窗口。 */
    {
        static const uint8_t message[] = "hello from BLE\r\npitch=12.4\n";
        uint8_t split[66], long_data[600];
        static const uint8_t binary[] = {0x00, 0xFF};
        uint32_t received;
        uint32_t tx_before = host_bt_tx_calls;
        host_bluetooth_feed(message, sizeof(message) - 1);
        advance(120);
        assert(host_bt_rx_pos == host_bt_rx_size);
        assert(strcmp(s_bt_text, "hello from BLE\npitch=12.4\n") == 0);
        drag(170, 100, -110, 0);
        assert(s_page == PAGE_SETTINGS);
        open_feature(PAGE_BLUETOOTH);
        assert(s_page == PAGE_BLUETOOTH);
        assert(strcmp(lv_label_get_text(s_battery_label), "~50%") == 0);
        assert(strcmp(lv_label_get_text(s_bt_log), s_bt_text) == 0);
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT OK / BLE ONLY") == 0);
        assert_static_refresh();
        /* 文字不变而颜色变化时，仍须传输新的像素。 */
        {
            uint32_t calls = host_flush_calls;
            uint32_t previous_color = s_bt_tx_color;
            const char *previous_text = s_bt_tx_text;
            s_bt_tx_color = UI_RED;
            refresh_data();
            lv_refr_now(NULL);
            assert(host_flush_calls > calls);
            assert(strcmp(lv_label_get_text(s_bt_tx_status), previous_text) == 0);
            assert(lv_obj_get_style_text_color(s_bt_tx_status, LV_PART_MAIN).full == lv_color_hex(UI_RED).full);
            assert_static_refresh();
            s_bt_tx_color = previous_color;
            refresh_data();
            lv_refr_now(NULL);
        }
        host_bt_tx_result = HAL_BUSY;
        tap(74, 114);
        assert(host_bt_tx_calls == tx_before + 1);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX BUSY") == 0);
        host_bt_tx_result = HAL_OK;
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT OK / BLE ONLY") == 0);
        screenshot("bluetooth");

        /* 点击真实发送按钮，检查UART获得的内容及各失败状态，RX记录不受影响。 */
        received = s_bt_rx_bytes;
        tap(74, 114);
        assert(host_bt_tx_calls == tx_before + 2 && host_bt_tx_size == 18);
        assert(strcmp((const char *)host_bt_tx_data, "hello from watch\r\n") == 0);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX UART OK") == 0);
        assert(s_bt_rx_bytes == received && strcmp(s_bt_text, "hello from BLE\npitch=12.4\n") == 0);
        host_bt_tx_result = HAL_TIMEOUT;
        tap(74, 114);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX TIMEOUT") == 0);
        host_bt_tx_result = HAL_BUSY;
        tap(74, 114);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX BUSY") == 0);
        host_bt_tx_result = HAL_ERROR;
        tap(74, 114);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX ERROR") == 0);
        host_bt_tx_result = HAL_OK;
        tap(74, 114);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX UART OK") == 0);
        screenshot("bluetooth_tx");

        /* Clear按钮清空显示及计数，跨64字节读取边界的CRLF只产生一次换行。 */
        tap(174, 114);
        assert(s_bt_text_length == 0 && s_bt_rx_bytes == 0);
        assert(strcmp(lv_label_get_text(s_bt_log), "Waiting for data...") == 0);
        screenshot("bluetooth_empty");
        memset(split, 'A', 63);
        split[63] = '\r';
        split[64] = '\n';
        split[65] = 'B';
        host_bluetooth_feed(split, sizeof(split));
        advance(120);
        assert(s_bt_text_length == 65 && s_bt_text[63] == '\n' && s_bt_text[64] == 'B');
        assert(s_bt_rx_bytes == sizeof(split));
        host_bluetooth_feed(binary, sizeof(binary));
        advance(120);
        assert(strstr(lv_label_get_text(s_bt_log), "\\x00\\xFF") != NULL);

        /* 大量输入只保留固定容量的最近内容，长日志可以滚动查看。 */
        memset(long_data, 'L', sizeof(long_data));
        host_bluetooth_feed(long_data, sizeof(long_data));
        advance(120);
        assert(s_bt_text_length == UI_BT_LOG_SIZE - 1);
        assert(strlen(lv_label_get_text(s_bt_log)) == UI_BT_LOG_SIZE - 1);
        assert(lv_obj_get_scroll_y(s_bt_log_box) > 0);
        assert(lv_obj_get_scroll_bottom(s_bt_log_box) == 0);
        received = s_bt_rx_bytes;
        key_back();
        assert(s_page == PAGE_SETTINGS);
        host_bluetooth_feed((const uint8_t *)"\nreceived off-page\n", 19);
        advance(120);
        open_feature(PAGE_BLUETOOTH);
        assert(s_page == PAGE_BLUETOOTH);
        assert(s_bt_rx_bytes == received + 19);
        assert(strstr(lv_label_get_text(s_bt_log), "received off-page") != NULL);

        host_bt_init_status = HAL_TIMEOUT;
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT TIMEOUT") == 0);
        host_bt_init_status = HAL_OK;
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT OK / BLE ONLY") == 0);
        lv_mem_monitor(&before);
        for (iteration = 0; iteration < 10; ++iteration)
        {
            key_back();
            assert(s_page == PAGE_SETTINGS);
            open_feature(PAGE_BLUETOOTH);
            assert(s_page == PAGE_BLUETOOTH);
        }
        lv_mem_monitor(&after);
        assert(after.free_size >= before.free_size - 512 && min_free > 4096);

        /* 使用真实校时解析器；HH:MM跨接收片段，空闲结束后才设置RTC。 */
        tap(174, 114);
        host_bluetooth_feed((const uint8_t *)"14:", 3);
        advance(100);
        assert(host_rtc_set_calls == 0);
        host_bluetooth_feed((const uint8_t *)"35", 2);
        advance(120);
        assert(host_rtc_set_calls == 0);
        advance(120);
        assert(host_rtc_set_calls == 1 && app_clock_get_result() == APP_CLOCK_SYNCED);
        advance(100);
        assert(strcmp(lv_label_get_text(s_bt_clock_status), "TIME SYNCED") == 0);
        assert_static_refresh();
        screenshot("bluetooth_time");
        key_back();
        key_back();
        key_back();
        assert(s_page == PAGE_WATCH);
        assert(strcmp(clock_text(), "14:35") == 0);
        screenshot("watch_synced");
        advance_active(60000);
        assert(strcmp(clock_text(), "14:36") == 0);
        host_bluetooth_feed((const uint8_t *)"24:61\n", 6);
        advance(120);
        assert(host_rtc_set_calls == 1 && app_clock_get_result() == APP_CLOCK_INVALID);
        assert(strcmp(clock_text(), "14:36") == 0);
        host_bluetooth_feed((const uint8_t *)"23:59\n", 6);
        advance(120);
        assert(strcmp(clock_text(), "23:59") == 0);
        advance_active(60000);
        assert(strcmp(clock_text(), "00:00") == 0);
        screenshot("watch_midnight");
        host_rtc_status = HAL_ERROR;
        advance(120);
        assert(strcmp(clock_text(), "--:--") == 0);
        assert(strcmp(lv_label_get_text(s_clock_hint), "RTC ERROR") == 0);
        assert_static_refresh();
        host_rtc_status = HAL_OK;
        host_rtc_synced = 0;
        advance(1100);
        assert(strcmp(clock_text(), "--:--") == 0);
        assert(strcmp(lv_label_get_text(s_clock_hint), "SET VIA BLE") == 0);
        assert(lv_obj_has_flag(s_hour_hand, LV_OBJ_FLAG_HIDDEN));
        assert(lv_obj_has_flag(s_minute_hand, LV_OBJ_FLAG_HIDDEN));
        assert(lv_obj_has_flag(s_second_hand, LV_OBJ_FLAG_HIDDEN));
        screenshot("watch_unsynced");
        assert_static_refresh();
        host_rtc_synced = 1;
        advance(120);
        assert(min_free > 4096);
    }
    /* 含秒校时仍从真实UART接收入口进入，RTC秒值推进与表盘分钟进位一致。 */
    host_bluetooth_feed((const uint8_t *)"14:35:47\n", 9);
    advance(120);
    assert(strcmp(clock_text(), "14:35") == 0);
    assert_rtc_seconds(47);
    advance(1000);
    assert(strcmp(clock_text(), "14:35") == 0);
    assert_rtc_seconds(48);
    {
        uint32_t calls = host_rtc_set_calls;
        host_bluetooth_feed((const uint8_t *)"14:35:60\n", 9);
        advance(120);
        assert(host_rtc_set_calls == calls && app_clock_get_result() == APP_CLOCK_INVALID);
        assert(strcmp(clock_text(), "14:35") == 0);
        assert_rtc_seconds(48);
    }
    host_bluetooth_feed((const uint8_t *)"23:59:59\n", 9);
    advance(120);
    assert(strcmp(clock_text(), "23:59") == 0);
    assert_rtc_seconds(59);
    advance(1000);
    assert(strcmp(clock_text(), "00:00") == 0);
    assert_rtc_seconds(0);
    printf("PASS: HH:MM:SS sync, internal RTC second progression, invalid second preservation and midnight carry.\n");
    /* 表盘只用本地温湿度，手机天气经蓝牙接收后仅显示在天气页。 */
    host_aht21.valid = 1;
    host_aht21.sample.temperature_c = 25.0f;
    host_aht21.sample.humidity_rh = 50.0f;
    host_bluetooth_feed((const uint8_t *)"WEATHER,3,263,80\n", 17);
    advance_active(1100);
    assert(strcmp(lv_label_get_text(s_clock_hint), "RTC / LSE") == 0);
    assert(strcmp(lv_label_get_text(s_watch_temperature), "25.0C\n50.0%") == 0);
    assert_static_refresh();
    screenshot("watch_climate");
    activity_ui_tests();
    assert(lv_obj_get_height(s_watch_temperature) <= lv_obj_get_height(lv_obj_get_parent(s_watch_temperature)));
    host_aht21.sample.temperature_c = -5.0f;
    host_aht21.sample.humidity_rh = 100.0f;
    advance_active(1100);
    assert(strcmp(lv_label_get_text(s_watch_temperature), "-5.0C\n100.0%") == 0);
    host_aht21.valid = 0;
    advance_active(1100);
    assert(strcmp(lv_label_get_text(s_watch_temperature), "--C\n--%") == 0);
    host_aht21.valid = 1;
    host_aht21.sample.temperature_c = 25.0f;
    host_aht21.sample.humidity_rh = 50.0f;
    advance_active(1100);
    assert(strcmp(lv_label_get_text(s_watch_temperature), "25.0C\n50.0%") == 0);
    drag(180, 140, -120, 0);
    assert(s_page == PAGE_SETTINGS);
    open_feature(PAGE_WEATHER);
    assert(strstr(lv_label_get_text(s_weather_label), "26.3C / RH 80%") != NULL);
    assert(strcmp(lv_label_get_text(s_weather_value), "26.3 C") == 0);
    assert(strcmp(lv_label_get_text(s_weather_condition), "OVERCAST") == 0);
    {
        static const struct
        {
            const char *frame;
            const char *description;
        } cases[] =
        {
            {"WEATHER,0,263,80\n", "CLEAR"},
            {"WEATHER,1,263,80\n", "MAINLY CLEAR"},
            {"WEATHER,2,263,80\n", "PARTLY CLOUDY"},
            {"WEATHER,45,263,80\n", "FOG"},
            {"WEATHER,51,263,80\n", "DRIZZLE"},
            {"WEATHER,61,263,80\n", "RAIN"},
            {"WEATHER,71,263,80\n", "SNOW"},
            {"WEATHER,95,263,80\n", "THUNDERSTORM"},
            {"WEATHER,4,263,80\n", "UNKNOWN"}
        };
        uint32_t icon_hashes[sizeof(cases) / sizeof(cases[0])];
        unsigned index;
        for (index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index)
        {
            lv_area_t condition_area, card_area, body_area, icon_area, value_area;
            lv_coord_t x, y;
            char preview_name[32];
            host_bluetooth_feed((const uint8_t *)cases[index].frame, (uint16_t)strlen(cases[index].frame));
            advance(120);
            assert(strcmp(lv_label_get_text(s_weather_condition), cases[index].description) == 0);
            assert(strcmp(lv_label_get_text(s_weather_value), "26.3 C") == 0);
            lv_obj_get_coords(s_weather_condition, &condition_area);
            lv_obj_get_coords(lv_obj_get_parent(s_weather_condition), &card_area);
            lv_obj_get_coords(s_body, &body_area);
            assert(condition_area.x1 >= card_area.x1 && condition_area.x2 <= card_area.x2);
            assert(condition_area.y1 >= card_area.y1 && condition_area.y2 <= card_area.y2);
            assert(condition_area.y1 >= body_area.y1 && condition_area.y2 <= body_area.y2);
            lv_obj_get_coords(s_weather_icon, &icon_area);
            lv_obj_get_coords(s_weather_value, &value_area);
            assert(icon_area.x1 >= card_area.x1 && icon_area.x2 < value_area.x1);
            assert(icon_area.y1 >= card_area.y1 && icon_area.y2 < condition_area.y1);
            assert(s_weather_icon_code == (uint8_t)strtoul(cases[index].frame + 8, NULL, 10));
            icon_hashes[index] = 2166136261U;
            for (y = icon_area.y1; y <= icon_area.y2; ++y)
            {
                for (x = icon_area.x1; x <= icon_area.x2; ++x)
                {
                    icon_hashes[index] = (icon_hashes[index] ^ lv_color_to32(framebuffer[y * 240 + x])) * 16777619U;
                }
            }
            snprintf(preview_name, sizeof(preview_name), "weather_icon_%u", index);
            screenshot(preview_name);
        }
        assert(icon_hashes[0] != icon_hashes[2]); /* 太阳与多云实际像素不同。 */
        assert(icon_hashes[2] != icon_hashes[3]);
        assert(icon_hashes[3] != icon_hashes[5]);
        assert(icon_hashes[5] != icon_hashes[6]);
        assert(icon_hashes[6] != icon_hashes[7]);
        assert(icon_hashes[7] != icon_hashes[8]);
    }
    host_bluetooth_feed((const uint8_t *)"WEATHER,2,263,80\n", sizeof("WEATHER,2,263,80\n") - 1);
    advance(120);
    host_bluetooth_feed((const uint8_t *)"WEATHER,100,263,80\n", sizeof("WEATHER,100,263,80\n") - 1);
    advance(120);
    assert(strcmp(lv_label_get_text(s_weather_condition), "PARTLY CLOUDY") == 0);
    assert(s_weather_icon_code == 2);
    assert_static_refresh();
    screenshot("phone_weather");
    host_tick += 3600000;
    lv_disp_trig_activity(NULL); /* 仅推进天气数据年龄，保持页面处于使用状态。 */
    advance(120);
    assert(lv_color_to32(lv_obj_get_style_text_color(s_weather_condition, 0)) == lv_color_to32(lv_color_hex(UI_MUTED)));
    host_bluetooth_feed((const uint8_t *)"WEATHER,0,263,80\n", sizeof("WEATHER,0,263,80\n") - 1);
    advance(120);
    assert(strcmp(lv_label_get_text(s_weather_condition), "CLEAR") == 0);
    assert(lv_color_to32(lv_obj_get_style_text_color(s_weather_condition, 0)) == lv_color_to32(lv_color_hex(UI_TEXT)));
    key_back();
    key_back();
    host_tick += 3600000;
    advance(120);
    assert(!s_screen_on && host_light == 0);
    host_wake = GPIO_PIN_SET;
    advance(60);
    host_wake = GPIO_PIN_RESET;
    advance(60); /* 一小时无操作后，按PA4恢复亮度及当前RTC表盘。 */
    assert(strcmp(lv_label_get_text(s_clock_hint), "RTC / LSE") == 0);
    assert(strcmp(lv_label_get_text(s_watch_temperature), "25.0C\n50.0%") == 0);
    /* 刻度盘和数字时间不再进入蓝牙；短拖动也不能误触心率入口。 */
    tap(120, 100);
    assert(s_page == PAGE_WATCH);
    tap(120, 235);
    assert(s_page == PAGE_WATCH);
    drag(80, 100, 20, 0);
    assert(s_page == PAGE_WATCH);
    drag(80, 100, 120, 0);
    assert(s_page == PAGE_WATCH);
    drag(40, 236, 20, 0);
    assert(s_page == PAGE_WATCH);
    drag(40, 236, 0, 20);
    assert(s_page == PAGE_WATCH);
    /* 移出后又回到按下点，仍按拖动处理。 */
    host_point.x = 40;
    host_point.y = 236;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(40);
    host_point.x = 60;
    advance(40);
    host_point.x = 40;
    advance(40);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(230);
    assert(s_page == PAGE_WATCH);
    /* 表盘快捷入口的详情右滑固定回功能页，实体键仍按历史返回。 */
    tap(174, 230);
    assert(s_page == PAGE_AHT21);
    drag(60, 140, 120, 0);
    assert(s_page == PAGE_SETTINGS);
    key_back();
    assert(s_page == PAGE_WATCH);
    tap(64, 230);
    assert(s_page == PAGE_HEART);
    drag(60, 140, 120, 0);
    assert(s_page == PAGE_SETTINGS);
    key_back();
    assert(s_page == PAGE_WATCH);
    tap(42, 266);
    assert(s_page == PAGE_WATCH);
    drag(180, 140, -120, 0);
    assert(s_page == PAGE_SETTINGS);
    {
        static const ui_page_t pages[] = {PAGE_HEART, PAGE_MOTION, PAGE_ENVIRONMENT, PAGE_AHT21,
                                           PAGE_WEATHER, PAGE_BLUETOOTH, PAGE_DISPLAY, PAGE_SPORT};
        /* 网格内不足翻页门槛的横向拖动，不触发卡片点击。 */
        drag(30, 70, 20, 0);
        assert(s_page == PAGE_SETTINGS);
        lv_mem_monitor(&before);
        for (iteration = 0; iteration < 70; ++iteration)
        {
            open_feature(pages[iteration % 8]);
            assert_static_refresh();
            saved_scroll = s_history[s_history_count - 1].scroll_y;
            drag(60, 22, 120, 0);
            assert(s_page == PAGE_SETTINGS);
            assert(lv_obj_get_scroll_y(s_body) == saved_scroll);
        }
        lv_mem_monitor(&after);
        assert(after.free_size >= before.free_size - 512 && min_free > 4096);
    }
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    screenshot("settings");
    lv_obj_scroll_to_y(s_body, 120, LV_ANIM_OFF);
    screenshot("settings_scrolled");
    key_back();
    /* 最终预览使用与参考图相同的时间；健康和天气数值仍是主机模拟输入。 */
    host_rtc_seconds = 8 * 3600 + 36 * 60 + 50;
    host_rtc_tick = host_tick;
    host_heart.state = APP_HEART_VALID;
    host_heart.bpm = 72;
    host_heart.last_bpm = 72;
    host_bluetooth_feed((const uint8_t *)"WEATHER,3,263,80\n", 17);
    advance(120);
    assert(strcmp(clock_text(), "08:36") == 0);
    assert_rtc_seconds(50);
    assert(strcmp(lv_label_get_text(s_watch_heart), "72") == 0);
    host_heart.state = APP_HEART_OFF;
    host_heart.bpm = 0;
    advance(120);
    assert(strcmp(lv_label_get_text(s_watch_heart), "72") == 0);
    host_heart.state = APP_HEART_ERROR;
    advance(120);
    assert(strcmp(lv_label_get_text(s_watch_heart), "72") == 0);
    assert_static_refresh();
    screenshot("watch");
    bluetooth_rx_error_tests();
    touch_bridge_tests();
    touch_idle_tests();
    /* 切页动画中熄屏：暂停绘图，唤醒后能完成动画并恢复交互。 */
    create_page(PAGE_SETTINGS, 0, 0);
    assert(s_transition);
    host_wake = GPIO_PIN_SET;
    advance(60);
    assert(!s_screen_on && s_transition);
    {
        unsigned int off_flushes = host_flush_calls;
        advance(1000);
        assert(host_flush_calls == off_flushes && s_transition);
    }
    host_wake = GPIO_PIN_RESET;
    advance(60);
    host_wake = GPIO_PIN_SET;
    advance(60);
    assert(s_screen_on);
    advance(200); /* LVGL首次恢复可能才开始计动画时间，保留原160ms过渡时长。 */
    assert(s_screen_on && !s_transition && s_page == PAGE_SETTINGS);
    puts("PASS: PA4 keeps Bluetooth power/RX across screen sleep and resumes the paused display transition.");
    printf("PASS: eight feature entries, watch without BLE shortcuts, drag/click isolation, fixed detail swipe return and grid scroll restoration.\n");
    printf("PASS: phone weather UART parsing, watch/weather display, age and unchanged redraw.\n");
    printf("PASS: battery fill pixels, percentage, low/empty/full states, cached pages, waiting/error/recovery and unchanged redraw.\n");
    printf("PASS: unchanged UI produces no pixel transfers; displayed decimal and color changes still redraw.\n");
    printf("PASS: BLE receive page entry/back, off-page reception, clear, split CRLF, binary bytes, bounded log and initialization states.\n");
    printf("PASS: heart card START/STOP, BPM display, measuring/no-signal/unavailable states and preserved navigation.\n");
    puts("PASS: background 10min barometer/1s BPM history, off-page retention, missing data gaps, negative altitude and compass smoothing.");
    puts("PASS: first-screen measurement/chart layouts, heart DETAILS touch expand/collapse and direct compass calibration entry.");
    printf("PASS: AHT21 values, negative temperature, waiting/measuring, CRC/I2C/calibration/timeout states and recovery.\n");
    printf("PASS: Send Test touch button, exact TX bytes, UART attempt without link GPIO, timeout/busy/error, RX unchanged.\n");
    printf("PASS: BLE HH:MM -> RTC -> analog UI, fragmented input, invalid-time preservation, autonomous ticking, midnight and "
           "unsynced/error states.\n");
    printf("PASS: RTC hands at 0/15/30 seconds, once-per-second summaries, no redraw within a second and minute carry.\n");
    printf("PASS: analog RTC watch, independent feature pages, swipes, slider, calibration, scroll history and 200 transitions.\n");
    printf("PASS: 16-entry history overflow/return, empty history, slow swipes with repeated coordinates.\n");
    printf("PASS: PA4 boot hold, debounce, press-triggered backlight toggle, brightness restore and screen-off input isolation.\n");
    printf("LVGL pool: minimum free %u bytes; free before/after stress %lu/%lu bytes.\n", min_free, (unsigned long)before.free_size,
           (unsigned long)after.free_size);
    uptime_report_tests();
    reset_report_tests();
    bluetooth_arrival_time_tests();
    return 0;
}
