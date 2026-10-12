/* Uses the project's real LVGL renderer and input engine, with hardware stubs.
 * The UI source is included so assertions can inspect page/scroll state. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../application/app_ui.c"

static uint32_t host_tick = 9U * 3600000U + 30U * 60000U + 42000U;
static GPIO_PinState host_key = GPIO_PIN_SET;
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
static unsigned min_free = 40960;
static uint8_t host_bt_connected;
static HAL_StatusTypeDef host_bt_init_status = HAL_OK;
static uint8_t host_bt_rx[1024];
static uint16_t host_bt_rx_size;
static uint16_t host_bt_rx_pos;
static HAL_StatusTypeDef host_bt_tx_result = HAL_OK;
static uint8_t host_bt_tx_data[64];
static uint16_t host_bt_tx_size;
static uint32_t host_bt_tx_calls;
static uint32_t host_rtc_seconds = 10U * 3600U + 10U * 60U;
static uint32_t host_rtc_tick = 9U * 3600000U + 30U * 60000U + 42000U;
static uint32_t host_rtc_set_calls;
static uint8_t host_rtc_synced = 1;
static HAL_StatusTypeDef host_rtc_status = HAL_OK;

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
HAL_StatusTypeDef watch_rtc_set_time(uint8_t hours, uint8_t minutes)
{
    if (host_rtc_status != HAL_OK)
    {
        return host_rtc_status;
    }
    assert(hours <= 23 && minutes <= 59);
    host_rtc_seconds = hours * 3600U + minutes * 60U;
    host_rtc_tick = host_tick;
    host_rtc_synced = 1;
    ++host_rtc_set_calls;
    return HAL_OK;
}
uint8_t watch_rtc_is_synced(void)
{
    return host_rtc_synced;
}

HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    assert(data != NULL && size < sizeof(host_bt_tx_data) && timeout_ms == UI_BT_TX_TIMEOUT_MS);
    ++host_bt_tx_calls;
    if (!host_bt_connected)
    {
        return HAL_BUSY;
    }
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
    return host_bt_connected;
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

TickType_t xTaskGetTickCount(void)
{
    return host_tick;
}
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    return host_key;
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
    App_UI_Create();
    advance(30);
    assert(s_page == PAGE_WATCH && host_light == 30);
    assert(strcmp(lv_label_get_text(s_time), "10:10") == 0);
    assert(s_hour_points[1].x < 100 && s_hour_points[1].y < 100);
    assert(s_minute_points[1].x > 100 && s_minute_points[1].y < 100);
    assert(!lv_obj_has_flag(s_body, LV_OBJ_FLAG_SCROLLABLE) && lv_obj_get_scroll_bottom(s_body) == 0);
    drag(110, 216, 0, -100);
    drag(110, 100, 0, 120);
    assert(s_page == PAGE_WATCH && lv_obj_get_scroll_y(s_body) == 0 && s_history_count == 0);
    screenshot("watch");
    /* 秒针读取RTC秒数：15秒指向3、30秒指向6，60秒回到12并推进分钟。 */
    assert(s_second_points[1].x == 100 && s_second_points[1].y == 18);
    advance(15000);
    assert(s_second_points[1].x == 182 && s_second_points[1].y == 100);
    screenshot("watch_seconds");
    advance(15000);
    assert(s_second_points[1].x == 100 && s_second_points[1].y == 182);
    advance(30000);
    assert(s_second_points[1].x == 100 && s_second_points[1].y == 18);
    assert(strcmp(lv_label_get_text(s_time), "10:11") == 0);
    key_back();
    assert(s_page == PAGE_WATCH && s_history_count == 0);
    /* Fixed watch -> combined page; left swipes stop at the last main page. */
    drag(180, 140, -120, 0);
    assert(s_page == PAGE_SETTINGS && s_history_count == 1);
    assert(s_brightness_label && s_angles[0] && s_pressure && s_altitude);
    drag(170, 100, -110, 0);
    assert(s_page == PAGE_SETTINGS && s_history_count == 1);
    screenshot("settings");
    /* Slider is isolated from page navigation. */
    drag(77, 129, 85, 0);
    assert(s_page == PAGE_SETTINGS && host_light > 30 && s_history_count == 1);
    /* A short electrical glitch must not trigger Back. */
    host_key = GPIO_PIN_RESET;
    advance(10);
    host_key = GPIO_PIN_SET;
    advance(60);
    assert(s_page == PAGE_SETTINGS);
    key_back();
    assert(s_page == PAGE_WATCH);
    drag(180, 140, -120, 0);
    drag(115, 216, 0, -100);
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) > 0);
    screenshot("settings_scrolled");
    drag(115, 100, 0, 120);
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) == 0);
    lv_obj_scroll_to_y(s_body, 260, LV_ANIM_OFF);
    advance(30);
    assert(strcmp(lv_label_get_text(s_angles[2]), "5.2") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "MAG UNCAL / calibrate first") == 0);
    assert(strcmp(lv_label_get_text(s_mag_values[1]), "-14.2") == 0);
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
    lv_obj_scroll_to_y(s_body, 340, LV_ANIM_OFF);
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
    lv_obj_scroll_to_y(s_body, 284, LV_ANIM_OFF);
    advance(30);
    drag(110, 216, 0, -100);
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) > 284);
    screenshot("motion_scrolled");
    host_online = 0;
    advance(120);
    assert(strcmp(lv_label_get_text(s_angles[0]), "--") == 0);
    assert(strcmp(lv_label_get_text(s_imu_status), "NO DATA") == 0);
    host_online = 1;
    advance(120);
    /* Environment is another section of the same page, with no detail navigation. */
    lv_obj_scroll_to_y(s_body, 1140, LV_ANIM_OFF);
    advance(30);
    saved_scroll = lv_obj_get_scroll_y(s_body);
    assert(s_page == PAGE_SETTINGS && s_history_count == 1);
    assert(strcmp(lv_label_get_text(s_pressure), "1013.25") == 0);
    assert(strcmp(lv_label_get_text(s_altitude), "-0.5") == 0);
    screenshot("environment");
    /* Leave the long page and return through history to its exact scroll position. */
    drag(60, 140, 120, 0);
    assert(s_page == PAGE_WATCH);
    assert(lv_obj_get_scroll_y(s_body) == 0 && !lv_obj_has_flag(s_body, LV_OBJ_FLAG_SCROLLABLE));
    key_back();
    assert(s_page == PAGE_SETTINGS && lv_obj_get_scroll_y(s_body) == saved_scroll);
    key_back();
    assert(s_page == PAGE_WATCH);
    /* Real gestures, rendering and automatic old-screen deletion are stressed. */
    lv_mem_monitor(&before);
    for (iteration = 0; iteration < 100; ++iteration)
    {
        drag(180, 140, -120, 0);
        assert(s_page == PAGE_SETTINGS);
        key_back();
        assert(s_page == PAGE_WATCH);
    }
    lv_mem_monitor(&after);
    assert(after.free_size >= before.free_size - 512);
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
        host_bluetooth_feed(message, sizeof(message) - 1);
        advance(120);
        assert(host_bt_rx_pos == host_bt_rx_size);
        assert(strcmp(s_bt_text, "hello from BLE\npitch=12.4\n") == 0);
        drag(170, 100, -110, 0);
        assert(s_page == PAGE_SETTINGS);
        tap(110, 220);
        assert(s_page == PAGE_BLUETOOTH);
        assert(strcmp(lv_label_get_text(s_bt_log), s_bt_text) == 0);
        assert(strcmp(lv_label_get_text(s_bt_status), "DISCONNECTED") == 0);
        tap(74, 114);
        assert(host_bt_tx_calls == 0);
        assert(strcmp(lv_label_get_text(s_bt_tx_status), "TX NO LINK") == 0);
        host_bt_connected = 1;
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "CONNECTED / BLE") == 0);
        screenshot("bluetooth");

        /* 点击真实发送按钮，检查UART获得的内容及各失败状态，RX记录不受影响。 */
        received = s_bt_rx_bytes;
        tap(74, 114);
        assert(host_bt_tx_calls == 1 && host_bt_tx_size == 18);
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
        tap(110, 220);
        assert(s_page == PAGE_BLUETOOTH);
        assert(s_bt_rx_bytes == received + 19);
        assert(strstr(lv_label_get_text(s_bt_log), "received off-page") != NULL);

        host_bt_connected = 0;
        host_bt_init_status = HAL_TIMEOUT;
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "INIT TIMEOUT") == 0);
        host_bt_init_status = HAL_OK;
        advance(120);
        assert(strcmp(lv_label_get_text(s_bt_status), "DISCONNECTED") == 0);
        lv_mem_monitor(&before);
        for (iteration = 0; iteration < 10; ++iteration)
        {
            key_back();
            assert(s_page == PAGE_SETTINGS);
            tap(110, 220);
            assert(s_page == PAGE_BLUETOOTH);
        }
        lv_mem_monitor(&after);
        assert(after.free_size >= before.free_size - 512 && min_free > 4096);

        /* 使用真实校时解析器；HH:MM跨接收片段，空闲结束后才设置RTC。 */
        tap(174, 114);
        host_bt_connected = 1;
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
        screenshot("bluetooth_time");
        key_back();
        key_back();
        key_back();
        assert(s_page == PAGE_WATCH);
        assert(strcmp(lv_label_get_text(s_time), "14:35") == 0);
        assert(s_hour_points[1].x > 100 && s_hour_points[1].y < 100);
        assert(s_minute_points[1].x < 100 && s_minute_points[1].y > 100);
        screenshot("watch_synced");
        advance(60000);
        assert(strcmp(lv_label_get_text(s_time), "14:36") == 0);
        host_bluetooth_feed((const uint8_t *)"24:61\n", 6);
        advance(120);
        assert(host_rtc_set_calls == 1 && app_clock_get_result() == APP_CLOCK_INVALID);
        assert(strcmp(lv_label_get_text(s_time), "14:36") == 0);
        host_bluetooth_feed((const uint8_t *)"23:59\n", 6);
        advance(120);
        assert(strcmp(lv_label_get_text(s_time), "23:59") == 0);
        advance(60000);
        assert(strcmp(lv_label_get_text(s_time), "00:00") == 0);
        screenshot("watch_midnight");
        host_rtc_status = HAL_ERROR;
        advance(120);
        assert(strcmp(lv_label_get_text(s_time), "--:--") == 0);
        assert(strcmp(lv_label_get_text(s_clock_hint), "RTC ERROR") == 0);
        assert(lv_obj_has_flag(s_second_hand, LV_OBJ_FLAG_HIDDEN));
        host_rtc_status = HAL_OK;
        host_rtc_synced = 0;
        advance(120);
        assert(strcmp(lv_label_get_text(s_time), "--:--") == 0);
        assert(strcmp(lv_label_get_text(s_clock_hint), "SET VIA BLE") == 0);
        assert(lv_obj_has_flag(s_hour_hand, LV_OBJ_FLAG_HIDDEN));
        assert(lv_obj_has_flag(s_second_hand, LV_OBJ_FLAG_HIDDEN));
        screenshot("watch_unsynced");
        host_rtc_synced = 1;
        advance(120);
        assert(!lv_obj_has_flag(s_second_hand, LV_OBJ_FLAG_HIDDEN));
        assert(min_free > 4096);
    }
    printf("PASS: BLE receive page entry/back, off-page reception, clear, split CRLF, binary bytes, bounded log and link states.\n");
    printf("PASS: Send Test touch button, exact TX bytes, disconnected guard, UART timeout/busy/error, RX unchanged.\n");
    printf("PASS: BLE HH:MM -> RTC -> analog/digital UI, fragmented input, invalid-time preservation, autonomous ticking, midnight and "
           "unsynced/error states.\n");
    printf("PASS: RTC second hand at 0/15/30 seconds, minute wrap and hidden/recovered states.\n");
    printf("PASS: fixed watch, combined settings/motion/environment, swipes, slider, calibration, scroll history and 200 transitions.\n");
    printf("PASS: 16-entry history overflow/return, empty history, slow swipes with repeated coordinates.\n");
    printf("LVGL 40 KiB pool: minimum free %u bytes; free before/after stress %lu/%lu bytes.\n", min_free, (unsigned long)before.free_size,
           (unsigned long)after.free_size);
    return 0;
}
