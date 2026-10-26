#include "app_reset_ble.h"
#include "app_reset.h"
#include "app_battery_log.h"
#include "app_clock.h"
#include "app_weather.h"
#include "watch_rtc.h"
#include "kt6368a.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

enum
{
    HOST_TX_NONE,
    HOST_TX_RESET,
    HOST_TX_OTHER
};

static app_reset_info_struct host_info;
static watch_rtc_status_struct host_rtc;
static app_weather_struct host_weather;
static app_clock_result host_clock_result;
static uint32_t host_clock_frames;
static unsigned host_state_snapshots;
static const uint8_t *host_tx;
static uint16_t host_size;
static uint8_t host_owner, host_result_ready;
static HAL_StatusTypeDef host_start_result, host_result;
static uint32_t host_now;
static unsigned host_writes, host_takes, host_snapshots, host_lines;
static char host_pending[112];
static char host_output[4096];

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

void app_reset_get_info(app_reset_info_struct *info)
{
    ++host_snapshots;
    *info = host_info;
}

void watch_rtc_get_status(watch_rtc_status_struct *status)
{
    ++host_state_snapshots;
    *status = host_rtc;
}

void app_weather_get_snapshot(app_weather_struct *snapshot)
{
    *snapshot = host_weather;
}

app_clock_result app_clock_get_result(void)
{
    return host_clock_result;
}

uint32_t app_clock_get_received_count(void)
{
    return host_clock_frames;
}

uint8_t kt6368a_tx_is_busy(void)
{
    return host_owner != HOST_TX_NONE;
}

HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    assert(host_owner == HOST_TX_NONE);
    assert(timeout_ms == 100U && size != 0 && size < sizeof(host_pending));
    ++host_writes;
    if (host_start_result != HAL_OK)
    {
        return host_start_result;
    }
    host_tx = data;
    host_size = size;
    memcpy(host_pending, data, size);
    host_pending[size] = '\0';
    host_owner = HOST_TX_RESET;
    host_result_ready = 0;
    return HAL_OK;
}

uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    ++host_takes;
    assert(host_owner == HOST_TX_RESET); /* 不能消费其它发送者尚未回收的结果。 */
    assert(memcmp(host_tx, host_pending, host_size) == 0);
    if (host_result_ready == 0)
    {
        return 0;
    }
    *result = host_result;
    if (host_result == HAL_OK)
    {
        assert(strlen(host_output) + host_size < sizeof(host_output));
        strncat(host_output, host_pending, host_size);
        ++host_lines;
    }
    host_owner = HOST_TX_NONE;
    host_result_ready = 0;
    return 1;
}

static void host_reset(void)
{
    assert(host_owner == HOST_TX_NONE);
    app_reset_ble_reset();
    host_now = 2000U;
    host_writes = 0;
    host_takes = 0;
    host_snapshots = 0;
    host_state_snapshots = 0;
    memset(&host_rtc, 0, sizeof(host_rtc));
    memset(&host_weather, 0, sizeof(host_weather));
    host_clock_result = APP_CLOCK_NONE;
    host_clock_frames = 0;
    host_lines = 0;
    host_start_result = HAL_OK;
    host_output[0] = '\0';
    memset(&host_info, 0, sizeof(host_info));
    host_info.reset_flags = 0xA6000003U;
    host_info.valid = 1;
    host_info.kind = 3;
    host_info.uptime_ms = UINT32_MAX;
    host_info.last_feed_ms = UINT32_MAX - 1U;
    host_info.max_feed_gap_ms = 805U;
    host_info.task_tag = 2;
    host_info.sp = 0x2001FFE0U;
    host_info.exc_return = 0xFFFFFFFDU;
    host_info.pc = 0x08012345U;
    host_info.lr = 0x08022223U;
    host_info.xpsr = 0x21000000U;
    host_info.cfsr = 0x00008200U;
    host_info.hfsr = 0x40000000U;
    host_info.mmfar = 0x20000000U;
    host_info.bfar = 0xDEADBEEFU;
}

static void host_receive(const char *text)
{
    app_reset_ble_receive((const uint8_t *)text, (uint16_t)strlen(text), host_now);
}

static void host_complete(HAL_StatusTypeDef result)
{
    assert(host_owner == HOST_TX_RESET && host_result_ready == 0);
    assert(memcmp(host_tx, host_pending, host_size) == 0);
    host_result = result;
    host_result_ready = 1;
}

static void host_finish_report(void)
{
    unsigned budget = 20;
    unsigned snapshots = host_snapshots;
    while (budget != 0)
    {
        --budget;
        if (host_owner == HOST_TX_RESET)
        {
            host_complete(HAL_OK);
        }
        host_now += 100U;
        if (app_reset_ble_service(1, host_now) == 0)
        {
            break;
        }
    }
    assert(budget != 0 && host_owner == HOST_TX_NONE);
    assert(host_lines == 5 && snapshots != 0 && host_snapshots == snapshots);
    assert(strstr(host_output, "RESET,END\r\n") != NULL);
    assert(app_reset_ble_service(1, host_now + 10000U) == 0);
}

static void host_assert_silent(void)
{
    assert(app_reset_ble_service(1, host_now) == 0);
    assert(host_writes == 0 && host_takes == 0 && host_snapshots == 0);
}

static void host_query(void)
{
    host_receive("RESET,READ\n");
    assert(app_reset_ble_service(1, host_now) == 1);
}

static void report_tests(void)
{
    host_reset();
    host_assert_silent();
    host_receive("BATLOG,READ\nWEATHER,3,263,80\n");
    host_assert_silent();
    host_receive("RES");
    host_assert_silent();
    host_now += 20U;
    host_receive("ET,RE");
    host_assert_silent();
    host_receive("AD");
    host_assert_silent();
    host_receive("\r\n");
    assert(app_reset_ble_service(1, host_now) == 1);
    assert(host_owner == HOST_TX_RESET && host_writes == 1);
    host_receive("RESET,READ\nRESET,READ\n");
    memset(&host_info, 0, sizeof(host_info)); /* 在途报告必须继续使用原快照。 */
    host_finish_report();
    assert(strcmp(host_output,
                  "RESET,CAUSE,VALID,1,FLAGS,0xA6000003,KIND,3\r\n"
                  "RESET,TIME,UP_MS,4294967295,FEED_MS,4294967294,GAP_MS,805,TASK,2\r\n"
                  "RESET,FRAME,SP,0x2001FFE0,EXC,0xFFFFFFFD,PC,0x08012345,LR,0x08022223,XPSR,0x21000000\r\n"
                  "RESET,FAULT,CFSR,0x00008200,HFSR,0x40000000,MMFAR,0x20000000,BFAR,0xDEADBEEF\r\n"
                  "RESET,END\r\n") == 0);
    assert(host_writes == 5);

    host_reset();
    memset(&host_info, 0, sizeof(host_info));
    host_info.reset_flags = 0x06000003U;
    host_query();
    host_finish_report();
    assert(strstr(host_output, "RESET,CAUSE,VALID,0,FLAGS,0x06000003,KIND,0\r\n") != NULL);
    assert(strstr(host_output, "RESET,TIME,UP_MS,0,FEED_MS,0,GAP_MS,0,TASK,0\r\n") != NULL);
    puts("PASS: explicit fragmented query, fixed snapshot, complete raw fields and valid=0; no unsolicited stream.");
}

static void framing_tests(void)
{
    static const uint8_t binary[] = "RESET,\0READ\n";
    host_reset();
    host_receive("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXRESET,READ\n");
    host_assert_silent();
    app_reset_ble_receive(binary, sizeof(binary) - 1U, host_now);
    host_assert_silent();
    host_receive("RESET,READ,EXTRA\n");
    host_assert_silent();
    host_receive("RESET,");
    host_now += 1000U;
    host_receive("READ\n");
    host_assert_silent();
    host_receive("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    host_now += 1000U;
    host_query();
    host_finish_report();

    host_reset();
    host_now = UINT32_MAX - 500U;
    host_receive("RESET,");
    host_now += 600U;
    host_receive("READ\n");
    assert(app_reset_ble_service(1, host_now) == 1);
    host_finish_report();

    host_reset();
    host_now = UINT32_MAX - 500U;
    host_receive("RESET,");
    host_now += 1000U;
    host_receive("READ\n");
    host_assert_silent();
    host_query();
    host_finish_report();
    puts("PASS: overflow/binary/extra data rejected; newline, timeout and uint32 wrap recover command framing.");
}

static void ownership_tests(void)
{
    unsigned writes;
    host_reset();
    host_owner = HOST_TX_OTHER;
    host_query();
    assert(host_writes == 0 && host_takes == 0);
    assert(app_reset_ble_service(1, host_now + 100U) == 1);
    assert(host_writes == 0 && host_takes == 0);
    host_owner = HOST_TX_NONE; /* 由其它发送者回收自己的结果。 */
    host_start_result = HAL_BUSY;
    assert(app_reset_ble_service(1, host_now) == 1);
    assert(host_writes == 1 && host_takes == 0 && host_snapshots == 1);
    memset(&host_info, 0, sizeof(host_info));
    host_start_result = HAL_OK;
    assert(app_reset_ble_service(1, host_now) == 1);
    assert(host_writes == 2 && host_owner == HOST_TX_RESET);
    writes = host_writes;
    assert(app_reset_ble_service(1, host_now + 1000U) == 1);
    assert(host_writes == writes && host_owner == HOST_TX_RESET);
    host_complete(HAL_OK);
    assert(app_reset_ble_service(1, host_now) == 1);
    assert(host_writes == writes && host_owner == HOST_TX_NONE);
    assert(app_reset_ble_service(1, host_now + 99U) == 1);
    assert(host_writes == writes);
    host_now += 100U;
    assert(app_reset_ble_service(1, host_now) == 1);
    host_finish_report();
    assert(host_writes == 6 && strstr(host_output, "FLAGS,0xA6000003") != NULL);
    puts("PASS: HAL_BUSY retains one snapshot; async buffer/result ownership and 100ms line pacing are preserved.");
}

static void failure_tests(void)
{
    static const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_TIMEOUT};
    unsigned i;
    for (i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i)
    {
        host_reset();
        host_start_result = failures[i];
        host_receive("RESET,READ\n");
        assert(app_reset_ble_service(1, host_now) == 0);
        assert(host_writes == 1 && host_takes == 0);
        assert(app_reset_ble_service(1, host_now + 10000U) == 0 && host_writes == 1);
        host_start_result = HAL_OK;
        host_query();
        host_complete(failures[i]);
        assert(app_reset_ble_service(1, host_now) == 0);
        assert(host_owner == HOST_TX_NONE && host_lines == 0);
        assert(app_reset_ble_service(1, host_now + 10000U) == 0 && host_writes == 2);
        host_query();
        host_finish_report();
    }
    puts("PASS: immediate/asynchronous UART errors stop the report; only a new explicit query restarts it.");
}

static void power_tests(void)
{
    static const HAL_StatusTypeDef completions[] = {HAL_OK, HAL_ERROR};
    unsigned i, writes;
    for (i = 0; i < sizeof(completions) / sizeof(completions[0]); ++i)
    {
        host_reset();
        host_query();
        writes = host_writes;
        app_reset_ble_reset();
        assert(memcmp(host_tx, host_pending, host_size) == 0);
        assert(app_reset_ble_service(0, host_now) == 1);
        host_receive("RESET,READ\n");
        host_complete(completions[i]);
        assert(app_reset_ble_service(0, host_now) == 0);
        assert(host_owner == HOST_TX_NONE && host_writes == writes);
        assert(app_reset_ble_service(1, host_now + 10000U) == 0 && host_writes == writes);
    }
    host_reset();
    host_receive("RESET,");
    app_reset_ble_reset();
    host_receive("READ\n");
    host_assert_silent();
    host_receive("RESET,READ\n");
    assert(app_reset_ble_service(0, host_now) == 0);
    host_assert_silent();
    host_owner = HOST_TX_OTHER;
    assert(app_reset_ble_service(0, host_now) == 0 && host_takes == 0);
    host_owner = HOST_TX_NONE;
    puts("PASS: power/parser reset aborts reports, preserves in-flight bytes and reclaims only the owned result.");
}

static void host_state_query(void)
{
    host_receive("STATE,READ\n");
    assert(app_reset_ble_service(1, host_now) == 1);
}

static void host_finish_state(void)
{
    unsigned budget = 20;
    while (budget != 0)
    {
        --budget;
        if (host_owner == HOST_TX_RESET)
        {
            host_complete(HAL_OK);
        }
        host_now += 100;
        if (app_reset_ble_service(1, host_now) == 0)
        {
            break;
        }
    }
    assert(budget != 0 && host_owner == HOST_TX_NONE);
    assert(host_lines == 4 && host_state_snapshots == 1 && host_snapshots == 0);
    assert(strstr(host_output, "STATE,END\r\n") != NULL);
    assert(app_reset_ble_service(1, host_now + 10000) == 0);
}

static void state_tests(void)
{
    host_reset();
    host_rtc.marker = 0x12345678;
    host_rtc.read_status = HAL_ERROR;
    host_receive("STATE,");
    host_assert_silent();
    host_receive("READ\r\n");
    assert(app_reset_ble_service(1, host_now) == 1);
    host_finish_state();
    assert(strcmp(host_output,
                  "STATE,RTC,READY,0,LSE,0,MARKER,0x12345678,SYNCED,0,READ,1,TIME,00:00:00\r\n"
                  "STATE,CLOCK,UP_MS,2000,RX,0,RESULT,0\r\n"
                  "STATE,WEATHER,VALID,0,CODE,0,TEMP_X10,0,RH,0,RX_MS,0,AGE_MS,0\r\n"
                  "STATE,END\r\n") == 0);

    host_reset();
    host_now = 0;
    host_rtc.ready = 1;
    host_rtc.lse_ready = 1;
    host_rtc.synced = 1;
    host_rtc.marker = 0x52544331;
    host_rtc.time.hours = 23;
    host_rtc.time.minutes = 59;
    host_rtc.time.seconds = 58;
    host_clock_frames = UINT32_MAX;
    host_clock_result = APP_CLOCK_RTC_ERROR;
    host_weather.valid = 1;
    host_weather.code = 99;
    host_weather.temperature_x10 = -1000;
    host_weather.humidity_rh = 100;
    host_weather.received_ms = 1;
    host_owner = HOST_TX_OTHER;
    host_state_query();
    assert(host_writes == 0 && host_takes == 0 && host_state_snapshots == 1);
    host_rtc.marker = 0;
    host_weather.valid = 0;
    host_clock_frames = 0;
    host_clock_result = APP_CLOCK_NONE;
    host_owner = HOST_TX_NONE;
    host_start_result = HAL_BUSY;
    assert(app_reset_ble_service(1, host_now + 100) == 1);
    host_start_result = HAL_OK;
    assert(app_reset_ble_service(1, host_now + 200) == 1);
    host_receive("RESET,READ\nSTATE,READ\n");
    host_finish_state();
    assert(strcmp(host_output,
                  "STATE,RTC,READY,1,LSE,1,MARKER,0x52544331,SYNCED,1,READ,0,TIME,23:59:58\r\n"
                  "STATE,CLOCK,UP_MS,0,RX,4294967295,RESULT,3\r\n"
                  "STATE,WEATHER,VALID,1,CODE,99,TEMP_X10,-1000,RH,100,RX_MS,1,AGE_MS,4294967295\r\n"
                  "STATE,END\r\n") == 0);

    host_reset();
    host_receive("STATE,READ,EXTRA\n");
    host_assert_silent();
    host_receive("STATE,");
    host_now += 1000;
    host_receive("READ\n");
    host_assert_silent();
    host_state_query();
    host_complete(HAL_ERROR);
    assert(app_reset_ble_service(1, host_now) == 0);
    assert(app_reset_ble_service(1, host_now + 1000) == 0);
    host_reset();
    host_state_query();
    app_reset_ble_reset();
    assert(memcmp(host_tx, host_pending, host_size) == 0);
    host_complete(HAL_OK);
    assert(app_reset_ble_service(0, host_now) == 0 && host_writes == 1);
    assert(app_reset_ble_service(1, host_now + 1000) == 0);
    puts("PASS: STATE query returns one frozen RTC/clock/weather snapshot; wrap, BUSY, framing, errors and power abort preserve TX ownership.");
}

int main(void)
{
    report_tests();
    framing_tests();
    ownership_tests();
    failure_tests();
    power_tests();
    state_tests();
    return 0;
}
