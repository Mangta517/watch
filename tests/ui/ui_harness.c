/* Uses the project's real LVGL renderer and input engine, with hardware stubs.
 * The UI source is included so assertions can inspect page/scroll state. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../application/app_ui.c"

static uint32_t host_tick = 9U * 3600000U + 30U * 60000U + 42000U;
static GPIO_PinState host_key = GPIO_PIN_SET;
static GPIO_PinState host_wake = GPIO_PIN_RESET;
static lv_indev_state_t host_touch = LV_INDEV_STATE_RELEASED;
static lv_point_t host_point;
static uint8_t host_online = 1;
static uint8_t host_mag_online = 1;
static uint8_t host_mag_calibrated;
static app_mag_cal_state host_mag_cal_state;
static app_mag_fusion_state host_mag_fusion_state = APP_MAG_FUSION_ACTIVE;
static app_mag_persist_state host_mag_persist_state;
static uint8_t host_light;
static lv_color_t framebuffer[240 * 280];
static lv_color_t draw_buffer[240 * 12];
static uint32_t host_flush_calls;
static uint32_t host_battery_reads;
static uint32_t host_heart_reads;
static unsigned min_free = 40960;
static uint8_t host_bt_pending, host_rtc_pending;
static HAL_StatusTypeDef host_bt_init_status = HAL_OK;
static uint8_t host_bt_rx[1024];
static uint16_t host_bt_rx_size;
static uint16_t host_bt_rx_pos;
static uint32_t host_bt_dropped, host_bt_errors;
static uint8_t host_bt_error_on_read;
extern unsigned host_diag_rx_resets;
static HAL_StatusTypeDef host_bt_tx_result = HAL_OK;
static uint8_t host_bt_tx_busy;
static uint8_t host_async_active;
static const uint8_t *host_async_data;
static uint8_t host_bt_tx_data[256];
static uint16_t host_bt_tx_size;
static uint32_t host_bt_tx_calls;
static uint32_t host_rtc_seconds = 10U * 3600U + 10U * 60U;
static uint32_t host_rtc_tick = 9U * 3600000U + 30U * 60000U + 42000U;
static uint32_t host_rtc_set_calls;
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
    uint16_t count = host_heart_samples_count < size ? host_heart_samples_count : size;
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
    host_heart.state = enabled ? APP_HEART_MEASURING : APP_HEART_OFF;
}

uint8_t watch_rtc_init_is_pending(void)
{
    return host_rtc_pending;
}
HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time)
{
    uint32_t seconds = (host_rtc_seconds + (uint32_t)(host_tick - host_rtc_tick) / 1000U) % 86400U;
    if (host_rtc_status != HAL_OK)
    {
        return host_rtc_status;
    }
    time->hours = seconds / 3600U;
    time->minutes = seconds / 60U % 60U;
    time->seconds = seconds % 60U;
    return HAL_OK;
}
HAL_StatusTypeDef watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    if (host_rtc_status != HAL_OK)
    {
        return host_rtc_status;
    }
    assert(hours <= 23 && minutes <= 59 && seconds <= 59);
    host_rtc_seconds = hours * 3600U + minutes * 60U + seconds;
    host_rtc_tick = host_tick;
    host_rtc_synced = 1;
    ++host_rtc_set_calls;
    return HAL_OK;
}
uint8_t watch_rtc_is_synced(void)
{
    return host_rtc_synced;
}

uint8_t kt6368a_tx_is_busy(void)
{
    return host_bt_tx_busy;
}

HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    assert(data != NULL && size < sizeof(host_bt_tx_data) && timeout_ms == UI_BT_TX_TIMEOUT_MS);
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
    if (host_bt_error_on_read != 0)
    {
        ++host_bt_errors;
        host_bt_error_on_read = 0;
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
}
static void host_bluetooth_feed(const uint8_t *data, uint16_t size)
{
    assert(host_bt_rx_pos == host_bt_rx_size && size <= sizeof(host_bt_rx));
    memcpy(host_bt_rx, data, size);
    host_bt_rx_size = size;
    host_bt_rx_pos = 0;
}

static void bluetooth_rx_error_tests(void)
{
    uint32_t calls = host_rtc_set_calls;
    unsigned resets = host_diag_rx_resets;
    app_weather_struct before, after;
    app_weather_get_snapshot(&before);
    host_bluetooth_feed((const uint8_t *)"12:", 3);
    bluetooth_receive();
    ++host_bt_dropped;
    host_bluetooth_feed((const uint8_t *)"34\n", 3);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls && host_diag_rx_resets == resets + 1);
    host_bluetooth_feed((const uint8_t *)"35\n13:14:15\n", 12);
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 13U * 3600U + 14U * 60U + 15U);

    host_bluetooth_feed((const uint8_t *)"WEATHER,3,2", 11);
    bluetooth_receive();
    ++host_bt_errors;
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
    host_bt_error_on_read = 1;
    host_bluetooth_feed((const uint8_t *)"20\n", 3);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls);
    host_bluetooth_feed((const uint8_t *)"09:08", 5);
    bluetooth_receive();
    host_tick += APP_CLOCK_FRAME_IDLE_MS;
    bluetooth_receive();
    assert(host_rtc_set_calls == calls + 1 && host_rtc_seconds == 9U * 3600U + 8U * 60U);

    host_bt_dropped = UINT32_MAX;
    bluetooth_receive();
    host_bt_dropped = 0;
    bluetooth_receive();
    assert(host_diag_rx_resets == resets + 5);
    host_bluetooth_feed((const uint8_t *)"\n07:06:05\n", 10);
    bluetooth_receive();
    assert(host_rtc_seconds == 7U * 3600U + 6U * 60U + 5U);
    puts("PASS: UART loss/error/counter-wrap resets all parsers, discards suffixes and recovers at newline or idle without clearing valid weather.");
}

TickType_t xTaskGetTickCount(void)
{
    return host_tick;
}
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    return pin == WAKE_Pin ? host_wake : host_key;
}
void LCD_Set_Light(uint8_t percent)
{
    host_light = percent;
}
uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)
{
    memset(imu, 0, sizeof(*imu));
    imu->pitch_deg = 12.4f;
    imu->roll_deg = -3.6f;
    imu->yaw_deg = 725.2f;
    imu->mag_online = host_mag_online;
    imu->mag_fused = host_mag_online && host_mag_cal_state != APP_MAG_CAL_RUNNING;
    imu->mag_calibrated = host_mag_calibrated;
    imu->mag_cal_state = host_mag_cal_state;
    imu->mag_persist_state = host_mag_persist_state;
    imu->mag_fusion_state = !host_mag_online
                                ? APP_MAG_FUSION_NO_DATA
                                : (host_mag_cal_state == APP_MAG_CAL_RUNNING ? APP_MAG_FUSION_CALIBRATING : host_mag_fusion_state);
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
    host_mag_cal_state = start ? APP_MAG_CAL_RUNNING : APP_MAG_CAL_DONE;
    if (!start)
    {
        host_mag_calibrated = 1;
        host_mag_persist_state = APP_MAG_PERSIST_SAVED;
    }
}
uint8_t App_Baro_GetSnapshot(app_baro_struct *baro)
{
    memset(baro, 0, sizeof(*baro));
    baro->pressure_pa = 101325;
    baro->alt_m = -0.5f;
    return host_online;
}
static void flush(lv_disp_drv_t *driver, const lv_area_t *area, lv_color_t *pixels)
{
    int x, y;
    ++host_flush_calls;
    for (y = area->y1; y <= area->y2; ++y)
    {
        for (x = area->x1; x <= area->x2; ++x)
        {
            framebuffer[y * 240 + x] = *pixels++;
        }
    }
    lv_disp_flush_ready(driver);
}
static void read_touch(lv_indev_drv_t *driver, lv_indev_data_t *data)
{
    (void)driver;
    data->state = host_touch;
    data->point = host_point;
}
static void advance(unsigned ms)
{
    unsigned elapsed;
    lv_mem_monitor_t memory;
    for (elapsed = 0; elapsed < ms; elapsed += 5)
    {
        host_tick += 5; /* Match the firmware LVGL task's5ms scheduling. */
        lv_timer_handler();
        lv_mem_monitor(&memory);
        if (memory.free_size < min_free)
        {
            min_free = memory.free_size;
        }
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
static void tap(int x, int y)
{
    host_point.x = x;
    host_point.y = y;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(40);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(230);
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
                                              PAGE_WEATHER, PAGE_BLUETOOTH, PAGE_DISPLAY};
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

static void screenshot(const char *name)
{
    char path[256];
    int pixel;
    FILE *file;
    lv_obj_invalidate(s_screen);
    advance(30);
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
int main(void)
{
    static lv_disp_draw_buf_t buffer;
    static lv_disp_drv_t display;
    static lv_indev_drv_t input;
    lv_coord_t saved_scroll;
    unsigned iteration;
    lv_mem_monitor_t before, after;
    lv_obj_t *cached_main[2];
    lv_init();
    lv_disp_draw_buf_init(&buffer, draw_buffer, NULL, 240 * 12);
    lv_disp_drv_init(&display);
    display.hor_res = 240;
    display.ver_res = 280;
    display.draw_buf = &buffer;
    display.flush_cb = flush;
    lv_disp_drv_register(&display);
    lv_indev_drv_init(&input);
    input.type = LV_INDEV_TYPE_POINTER;
    input.read_cb = read_touch;
    input.gesture_min_velocity = 0;
    input.gesture_limit = 40;
    lv_indev_drv_register(&input);
    host_wake = GPIO_PIN_SET; /* 模拟按住 SW2 开机。 */
    App_UI_Create();
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
    advance(15000);
    assert_rtc_seconds(15);
    assert(s_second_points[1].x == 182 && s_second_points[1].y == 100);
    screenshot("watch_seconds");
    advance(15000);
    assert_rtc_seconds(30);
    assert(s_second_points[1].x == 100 && s_second_points[1].y == 182);
    advance(30000);
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
        host_rtc_seconds = time.hours * 3600U + time.minutes * 60U + time.seconds;
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
    assert(framebuffer[19 * 240 + 21].full == lv_color_hex(UI_BG).full);
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
    assert(lv_obj_get_child_cnt(s_body) == 7);
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
        screenshot("heart");
        host_heart.state = APP_HEART_NO_SIGNAL;
        advance(120);
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
        advance(1000);
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
    assert_static_refresh();
    screenshot("motion");
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
    lv_obj_scroll_to_view(s_mag_cal_status, LV_ANIM_OFF);
    advance(30);
    screenshot("motion_calibration_status");
    lv_obj_scroll_to_y(s_body, 384, LV_ANIM_OFF);
    advance(30);
    screenshot("motion_magnetic");
    host_mag_online = 0;
    advance(120);
    assert(strcmp(lv_label_get_text(s_mag_values[0]), "--") == 0);
    assert(strcmp(lv_label_get_text(s_angles[2]), "5.2") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / MAG NO DATA") == 0);
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
    screenshot("environment");
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
    assert(strcmp(lv_label_get_text(s_aht21_samples), "SAMPLES 12 / 2s") == 0);
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
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT OK / BLE + SPP") == 0);
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
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT OK / BLE + SPP") == 0);
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
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT OK / BLE + SPP") == 0);
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
        advance(60000);
        assert(strcmp(clock_text(), "14:36") == 0);
        host_bluetooth_feed((const uint8_t *)"24:61\n", 6);
        advance(120);
        assert(host_rtc_set_calls == 1 && app_clock_get_result() == APP_CLOCK_INVALID);
        assert(strcmp(clock_text(), "14:36") == 0);
        host_bluetooth_feed((const uint8_t *)"23:59\n", 6);
        advance(120);
        assert(strcmp(clock_text(), "23:59") == 0);
        advance(60000);
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
    /* 手机天气经实际蓝牙接收入口解析，切页保留，并显示最后同步时间。 */
    host_bluetooth_feed((const uint8_t *)"WEATHER,3,263,80\n", 17);
    advance(1100);
    assert(strcmp(lv_label_get_text(s_clock_hint), "OVERCAST") == 0);
    assert(strcmp(lv_label_get_text(s_watch_temperature), "26.3C") == 0);
    assert_static_refresh();
    screenshot("watch_weather");
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
    host_tick += 3600000U;
    advance(120);
    assert(lv_color_to32(lv_obj_get_style_text_color(s_weather_condition, 0)) == lv_color_to32(lv_color_hex(UI_MUTED)));
    host_bluetooth_feed((const uint8_t *)"WEATHER,0,263,80\n", sizeof("WEATHER,0,263,80\n") - 1);
    advance(120);
    assert(strcmp(lv_label_get_text(s_weather_condition), "CLEAR") == 0);
    assert(lv_color_to32(lv_obj_get_style_text_color(s_weather_condition, 0)) == lv_color_to32(lv_color_hex(UI_TEXT)));
    key_back();
    key_back();
    host_tick += 3600000U;
    advance(120);
    assert(strstr(lv_label_get_text(s_clock_hint), "OLD WEATHER") != NULL);
    /* 表盘快捷控件与左滑入口均使用真实触摸；底部非入口区域不触发导航。 */
    tap(174, 230);
    assert(s_page == PAGE_WEATHER);
    key_back();
    tap(64, 230);
    assert(s_page == PAGE_HEART);
    key_back();
    tap(120, 100);
    assert(s_page == PAGE_BLUETOOTH);
    key_back();
    tap(42, 266);
    assert(s_page == PAGE_WATCH);
    drag(180, 140, -120, 0);
    assert(s_page == PAGE_SETTINGS);
    {
        static const ui_page_t pages[] = {PAGE_HEART, PAGE_MOTION, PAGE_ENVIRONMENT, PAGE_AHT21,
                                           PAGE_WEATHER, PAGE_BLUETOOTH, PAGE_DISPLAY};
        lv_mem_monitor(&before);
        for (iteration = 0; iteration < 70; ++iteration)
        {
            open_feature(pages[iteration % 7]);
            assert_static_refresh();
            key_back();
            assert(s_page == PAGE_SETTINGS);
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
    host_rtc_seconds = 8U * 3600U + 36U * 60U + 50U;
    host_rtc_tick = host_tick;
    host_heart.state = APP_HEART_VALID;
    host_heart.bpm = 72;
    host_bluetooth_feed((const uint8_t *)"WEATHER,3,263,80\n", 17);
    advance(120);
    assert(strcmp(clock_text(), "08:36") == 0);
    assert_rtc_seconds(50);
    assert(strcmp(lv_label_get_text(s_watch_heart), "72") == 0);
    assert_static_refresh();
    screenshot("watch");
    bluetooth_rx_error_tests();
    printf("PASS: seven feature entries, full-screen watch, shortcuts, swipe entry, and detail return/scroll restoration.\n");
    printf("PASS: phone weather UART parsing, watch/weather display, age and unchanged redraw.\n");
    printf("PASS: battery fill pixels, percentage, low/empty/full states, cached pages, waiting/error/recovery and unchanged redraw.\n");
    printf("PASS: unchanged UI produces no pixel transfers; displayed decimal and color changes still redraw.\n");
    printf("PASS: BLE receive page entry/back, off-page reception, clear, split CRLF, binary bytes, bounded log and initialization states.\n");
    printf("PASS: heart card START/STOP, BPM display, measuring/no-signal/unavailable states and preserved navigation.\n");
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
    return 0;
}
