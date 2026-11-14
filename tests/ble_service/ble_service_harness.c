/* 正常多.c链接蓝牙服务，不包含UI或服务实现，不链接LVGL。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_ble_service.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_weather.h"
#include "app_diag.h"
#include "app_battery_log_ble.h"
#include "app_reset_ble.h"
#include "app_ota.h"
#include "watch_rtc.h"
#include "FreeRTOS.h"
#include "task.h"

static uint32_t host_tick = 10000;
uint32_t HAL_GetTick(void)
{
    return host_tick;
}

TickType_t xTaskGetTickCount(void)
{
    return host_tick + 37; /* 保留HAL与业务时间的不同零点。 */
}

void host_touch_enter_critical(void)
{
}

void host_touch_exit_critical(void)
{
}

#include "host_uart.h"

static uint32_t host_rtc_set_calls, host_rtc_seconds;
static uint8_t host_rtc_fail;
static unsigned host_rx[3], host_calls[3];
static uint8_t host_busy[3], host_ready[3];
static uint16_t host_heart_samples_count;
static uint32_t host_heart_tx_ok, host_heart_tx_error;
static uint8_t host_temperature_valid;
static int16_t host_temperature_x10;
unsigned host_diag_rx_resets;

uint8_t watch_rtc_init_is_pending(void)
{
    return host_rtc_pending;
}

uint8_t watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    ++host_rtc_set_calls;
    host_rtc_seconds = hours * 3600 + minutes * 60 + seconds;
    return host_rtc_fail;
}

void app_baro_set_reference(uint32_t pressure_pa, int16_t temperature_x10, uint8_t temperature_valid)
{
    (void)pressure_pa;
    host_temperature_x10 = temperature_x10;
    host_temperature_valid = temperature_valid;
}

uint16_t app_heart_read_samples(app_heart_sample_struct *samples, uint16_t size)
{
    uint16_t i, count = host_heart_samples_count;
    if (count > size)
    {
        count = size;
    }
    for (i = 0; i < count; ++i)
    {
        samples[i] = (app_heart_sample_struct){.raw = 12345, .bpm = 72, .sequence = i};
    }
    host_heart_samples_count -= count;
    return count;
}

void app_diag_heart_tx(uint16_t count, uint8_t success)
{
    if (success != 0)
    {
        host_heart_tx_ok += count;
    }
    else
    {
        host_heart_tx_error += count;
    }
}

void app_diag_receive(const uint8_t *data, uint16_t size)
{
    (void)data;
    (void)size;
}

void app_diag_reset_receive(void)
{
    ++host_diag_rx_resets;
}

void app_battery_log_ble_reset(void)
{
}

void app_reset_ble_reset(void)
{
}

void app_ota_reset_receive(void)
{
}

void app_battery_log_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    (void)data;
    assert((uint32_t)(xTaskGetTickCount() - now_ms) < 1000);
    host_rx[0] += size;
}

void app_reset_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    (void)data;
    (void)now_ms;
    host_rx[1] += size;
}

void app_ota_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    (void)data;
    (void)now_ms;
    host_rx[2] += size;
}

uint8_t app_battery_log_ble_service(uint8_t ready, uint32_t now_ms)
{
    (void)now_ms;
    ++host_calls[0];
    host_ready[0] = ready;
    return host_busy[0];
}

uint8_t app_reset_ble_service(uint8_t ready, uint32_t now_ms)
{
    (void)now_ms;
    ++host_calls[1];
    host_ready[1] = ready;
    return host_busy[1];
}

uint8_t app_ota_service(uint8_t ready, uint32_t now_ms)
{
    (void)now_ms;
    ++host_calls[2];
    host_ready[2] = ready;
    return host_busy[2];
}

static app_ble_snapshot_struct host_snapshot(void)
{
    app_ble_snapshot_struct snapshot;
    app_ble_get_snapshot(&snapshot);
    return snapshot;
}

static void host_service(uint8_t screen_on, uint8_t sport_mode)
{
    uint8_t active = app_ble_service(screen_on, sport_mode);
    if (host_busy[0] != 0 || host_busy[1] != 0 || host_busy[2] != 0)
    {
        assert(active != 0);
    }
}

static void host_feed(const char *message)
{
    host_bluetooth_feed((const uint8_t *)message, (uint16_t)strlen(message));
}

/* 既有CLEAR与错误恢复回归通过公共接口调用，原用例同时保留UI集成覆盖。 */
static void bluetooth_clear_frame_tests(void)
{
    uint32_t calls = host_rtc_set_calls;
    uint32_t seconds = host_rtc_seconds;
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    host_service(1, 0);
    host_bluetooth_feed((const uint8_t *)"34:", 3);
    app_ble_clear_log();
    assert(host_bt_rx_pos == 0);
    host_service(1, 0);
    assert(host_rtc_set_calls == calls);
    host_bluetooth_feed((const uint8_t *)"56\n", 3);
    host_service(1, 0);
    assert(host_rtc_set_calls == calls + 1);
    assert(host_rtc_seconds == 12 * 3600 + 34 * 60 + 56);
    host_rtc_seconds = seconds;
    host_rtc_set_calls = calls;
    app_ble_clear_log();
    puts("PASS: CLEAR preserves queued bytes and partial clock frames.");
}

static void bluetooth_rx_error_tests(void)
{
    uint32_t calls = host_rtc_set_calls;
    unsigned resets = host_diag_rx_resets;
    app_weather_struct before, after;
    app_weather_get_snapshot(&before);
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    host_service(1, 0);
    host_bluetooth_fault(1);
    host_bluetooth_feed((const uint8_t *)"34\n", 3);
    host_service(1, 0);
    assert(host_rtc_set_calls == calls && host_diag_rx_resets == resets + 1);
    host_bluetooth_feed((const uint8_t *)"35\n13:14:15\n", 12);
    host_service(1, 0);
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 13 * 3600 + 14 * 60 + 15);

    host_bluetooth_feed((const uint8_t *)"WEATHER,3,2", 11);
    host_service(1, 0);
    host_bluetooth_fault(0);
    host_service(1, 0);
    host_bluetooth_feed((const uint8_t *)"63,80\n", 6);
    host_service(1, 0);
    app_weather_get_snapshot(&after);
    assert(after.received_ms == before.received_ms && after.temperature_x10 == before.temperature_x10);
    host_bluetooth_feed((const uint8_t *)"WEATHER,1,-20,60\n", 17);
    host_service(1, 0);
    app_weather_get_snapshot(&after);
    assert(after.code == 1 && after.temperature_x10 == -20 && after.humidity_rh == 60);

    host_bluetooth_feed((const uint8_t *)"10:", 3);
    host_service(1, 0);
    calls = host_rtc_set_calls;
    host_bluetooth_fault(0);
    host_bluetooth_feed((const uint8_t *)"20\n", 3);
    host_service(1, 0);
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    host_service(1, 0);
    assert(host_rtc_set_calls == calls);
    host_bluetooth_feed((const uint8_t *)"09:08", 5);
    host_service(1, 0);
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    host_service(1, 0);
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 9 * 3600 + 8 * 60);

    resets = host_diag_rx_resets;
    host_bt_dropped = UINT32_MAX - 1;
    host_bluetooth_fault(1);
    host_service(1, 0);
    host_bluetooth_fault(1);
    host_service(1, 0);
    assert(host_bt_dropped == 0 && host_diag_rx_resets == resets + 2);
    host_bluetooth_feed((const uint8_t *)"\n07:06:05\n", 10);
    host_service(1, 0);
    assert(host_rtc_seconds == 7 * 3600 + 6 * 60 + 5);
    puts("PASS: UART loss/error/counter-wrap resets all parsers, discards suffixes and recovers at newline or idle without clearing valid weather.");
}


static void uptime_tests(void)
{
    unsigned i;
    uint32_t calls;
    host_previous_valid = 1;
    host_previous_seconds = 90061;
    host_service(1, 0);
    assert(host_bt_tx_calls == 0);
    host_feed("ER+7\r\n25:00\n12:");
    host_service(1, 0);
    assert(host_bt_tx_calls == 0);
    host_busy[0] = 1;
    host_feed("34:56\n");
    host_service(1, 0);
    assert(host_bt_tx_calls == 0);
    for (i = 0; i < 3; ++i)
    {
        assert(host_rx[i] == host_rx[0] && host_calls[i] == host_calls[0]);
    }
    host_busy[0] = 0;
    host_bt_tx_busy = 1;
    host_service(1, 0);
    assert(host_bt_tx_calls == 0);
    host_bt_tx_busy = 0;
    host_bt_tx_result = HAL_BUSY;
    host_service(1, 0);
    assert(host_bt_tx_calls == 1 && host_snapshot().tx_state == APP_BLE_TX_READY);
    host_bt_tx_result = HAL_OK;
    host_service(0, 1);
    assert(host_bt_tx_calls == 1);
    host_service(0, 0);
    assert(host_bt_tx_calls == 2 && host_snapshot().tx_state == APP_BLE_TX_UPTIME_OK);
    assert(strcmp((const char *)host_bt_tx_data, "UPTIME_PREV,90061,25:01:01\r\n") == 0);
    app_ble_update_power(1, 1, 0);
    app_ble_update_power(1, 0, 0);
    host_feed("12:34:56\n");
    host_service(1, 0);
    assert(host_bt_tx_calls == 2); /* 真正供电切换也不重复本次开机的报告。 */
    calls = host_bt_tx_calls;
    host_previous_seconds = 600;
    host_service(1, 0);
    assert(host_bt_tx_calls == calls);
    puts("PASS: UPTIME waits for valid phone input, yields to every management service/TX, retries BUSY, works sleeping and reports once across reconnect.");
}

static void uptime_error_tests(HAL_StatusTypeDef error)
{
    host_previous_valid = 1;
    host_previous_seconds = 90061;
    host_rtc_fail = 1; /* 合法校时帧即确认手机通信，不要求RTC写入成功。 */
    host_bt_tx_result = error;
    host_feed("12:34:56\n");
    host_service(1, 0);
    assert(host_snapshot().clock_result == APP_CLOCK_RTC_ERROR);
    assert(host_bt_tx_calls == 1 && host_snapshot().tx_state == APP_BLE_TX_UPTIME_ERROR);
    host_bt_tx_result = HAL_OK;
    host_service(1, 0);
    assert(host_bt_tx_calls == 1);
    host_feed("12:34:56");
    host_service(1, 0);
    assert(host_bt_tx_calls == 1);
    host_tick += 200;
    host_service(0, 0);
    assert(host_bt_tx_calls == 2 && host_snapshot().tx_state == APP_BLE_TX_UPTIME_OK);
    host_service(1, 0);
    assert(host_bt_tx_calls == 2);
    puts("PASS: failed UPTIME waits for a new valid clock frame; true idle and RTC-write failure retain phone-ready semantics.");
}

static void uptime_heart_tests(void)
{
    host_feed("12:34:56\n");
    host_heart_samples_count = 1;
    host_service(1, 0); /* 没有旧记录时不报告，调试构建仍可发送心率。 */
    host_previous_valid = 1;
    host_previous_seconds = 600;
#if APP_DEBUG_TELEMETRY
    assert(host_async_active != 0 && host_bt_tx_calls == 1);
    host_async_wait = 1;
    host_service(1, 0);
    assert(host_async_active != 0 && host_bt_tx_calls == 1);
    host_async_wait = 0;
    host_service(1, 0);
    assert(host_async_active == 0 && host_heart_tx_ok == 1 && host_bt_tx_calls == 2);
#else
    assert(host_bt_tx_calls == 0 && host_heart_samples_count == 1);
    host_service(1, 0);
    assert(host_bt_tx_calls == 1 && host_heart_samples_count == 1);
#endif
    assert(strcmp((const char *)host_bt_tx_data, "UPTIME_PREV,600,0:10:00\r\n") == 0);
    assert(host_snapshot().tx_state == APP_BLE_TX_UPTIME_OK);
    puts("PASS: no old record means no UPTIME; heart TX is collected by its owner before the synchronous UPTIME report.");
}

static void ownership_tests(void)
{
    static const uint8_t external[] = "external";
    unsigned i;
    assert(kt6368a_write_async(external, sizeof(external), 20) == HAL_OK);
    host_service(1, 0);
    assert(host_async_active != 0); /* 未发起的TX结果不能由服务取走。 */
    assert(kt6368a_take_tx_result(&host_bt_tx_result) != 0);
    host_heart_samples_count = 1;
    host_service(1, 0);
#if APP_DEBUG_TELEMETRY
    assert(host_async_active != 0 && host_heart_samples_count == 0);
    host_async_wait = 1;
    host_service(0, 0);
    host_tick += 2000;
    app_ble_update_power(0, 0, 0);
    assert(host_snapshot().enabled != 0);
    app_ble_update_power(0, 1, 0);
    host_async_wait = 0;
    host_async_result = HAL_ERROR; /* 驱动断电后将取消结果留给原发起者。 */
    host_service(0, 1);
    assert(host_async_active == 0 && host_heart_tx_error == 1);
    assert(host_snapshot().tx_state == APP_BLE_TX_HEART_ERROR);
#else
    assert(host_async_active == 0 && host_heart_samples_count == 1);
    app_ble_update_power(0, 1, 0);
    host_service(0, 1);
#endif
    for (i = 0; i < 3; ++i)
    {
        assert(host_ready[i] == 0 && host_calls[i] != 0);
    }
    app_ble_send_test();
#if APP_DEBUG_TELEMETRY == 0
    assert(host_bt_tx_calls == 1);
#endif
    puts("PASS: external TX ownership, debug/quiet gates and management cleanup after forced power-off.");
}

static void receive_power_tests(void)
{
    app_ble_snapshot_struct before;
    app_weather_struct weather;
    unsigned i;
    bluetooth_clear_frame_tests();
    bluetooth_rx_error_tests();
    app_ble_clear_log();
    before = host_snapshot();
    host_bt_pending = 1;
    host_feed("WEATHER,3,263,80\n");
    host_service(1, 0);
    assert(host_snapshot().rx_bytes == 0);
    host_bt_pending = 0;
    host_rtc_pending = 1;
    host_service(1, 0);
    assert(host_snapshot().rx_bytes == 0);
    host_rtc_pending = 0;
    host_service(1, 0);
    app_weather_get_snapshot(&weather);
    assert(weather.received_ms == xTaskGetTickCount());
    assert(host_temperature_valid != 0 && host_temperature_x10 == 263);
    assert(host_snapshot().log_revision != before.log_revision);
    app_ble_update_power(0, 0, 1);
    host_tick += 999;
    app_ble_update_power(0, 0, 0);
    assert(host_snapshot().enabled != 0);
    ++host_tick;
    app_ble_update_power(0, 0, 0);
    assert(host_snapshot().enabled == 0);
    host_tick += 3600000 - 1000;
    host_service(0, 0);
    assert(host_temperature_valid == 0);
    for (i = 0; i < 3; ++i)
    {
        assert(host_ready[i] == 0);
    }
    app_ble_set_eco(0);
    app_ble_update_power(0, 0, 0);
    assert(host_snapshot().enabled != 0 && host_snapshot().eco == 0);
    host_tick += 2000;
    app_ble_update_power(0, 0, 0);
    assert(host_snapshot().enabled != 0);
    puts("PASS: independent RX/CLEAR/loss recovery, timebase offset, AT/RTC guards, ECO grace, KEEP CONNECTED and weather expiry while powered off.");
}

int main(int argc, char **argv)
{
    const char *mode = "receive_power";
    if (argc > 1)
    {
        mode = argv[1];
    }
    if (strcmp(mode, "uptime") == 0)
    {
        uptime_tests();
    }
    else if (strcmp(mode, "uptime_error") == 0)
    {
        uptime_error_tests(HAL_ERROR);
    }
    else if (strcmp(mode, "uptime_timeout") == 0)
    {
        uptime_error_tests(HAL_TIMEOUT);
    }
    else if (strcmp(mode, "ownership") == 0)
    {
        ownership_tests();
    }
    else if (strcmp(mode, "uptime_heart") == 0)
    {
        uptime_heart_tests();
    }
    else
    {
        receive_power_tests();
    }
    return 0;
}
