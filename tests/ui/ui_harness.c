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

TickType_t xTaskGetTickCount(void) { return host_tick; }
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port; (void)pin;
    return host_key;
}
void LCD_Set_Light(uint8_t percent) { host_light = percent; }
uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)
{
    memset(imu, 0, sizeof(*imu));
    imu->pitch_deg = 12.4f; imu->roll_deg = -3.6f; imu->yaw_deg = 725.2f;
    imu->mag_online = host_mag_online;
    imu->mag_fused = host_mag_online && host_mag_cal_state != APP_MAG_CAL_RUNNING;
    imu->mag_calibrated = host_mag_calibrated;
    imu->mag_cal_state = host_mag_cal_state;
    imu->mag_persist_state = host_mag_persist_state;
    imu->mag_fusion_state = !host_mag_online ? APP_MAG_FUSION_NO_DATA :
        (host_mag_cal_state == APP_MAG_CAL_RUNNING ? APP_MAG_FUSION_CALIBRATING : host_mag_fusion_state);
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
    baro->pressure_pa = 101325; baro->alt_m = -0.5f;
    return host_online;
}
static void flush(lv_disp_drv_t *driver, const lv_area_t *area, lv_color_t *pixels)
{
    int x, y;
    for (y = area->y1; y <= area->y2; ++y)
        for (x = area->x1; x <= area->x2; ++x)
            framebuffer[y * 240 + x] = *pixels++;
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
    for (elapsed = 0; elapsed < ms; elapsed += 5) {
        host_tick += 5; /* Match the firmware LVGL task's5ms scheduling. */
        lv_timer_handler();
        lv_mem_monitor(&memory);
        if (memory.free_size < min_free) min_free = memory.free_size;
    }
}
static void tap(int x, int y)
{
    host_point.x = x; host_point.y = y;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(40);
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(230);
}
static void drag(int x, int y, int dx, int dy)
{
    unsigned step;
    host_point.x = x; host_point.y = y;
    host_touch = LV_INDEV_STATE_PRESSED;
    advance(20);
    for (step = 1; step <= 12; ++step) {
        host_point.x = x + dx * (int)step / 12;
        host_point.y = y + dy * (int)step / 12;
        advance(10);
    }
    host_touch = LV_INDEV_STATE_RELEASED;
    advance(300);
}
static void key_back(void)
{
    host_key = GPIO_PIN_RESET; advance(60);
    host_key = GPIO_PIN_SET; advance(250);
}
static void screenshot(const char *name)
{
    char path[256];
    int pixel;
    FILE *file;
    lv_obj_invalidate(s_screen); advance(30);
    snprintf(path, sizeof(path), "tests/ui/output/%s.ppm", name);
    file = fopen(path, "wb"); assert(file);
    fprintf(file, "P6\n240 280\n255\n");
    for (pixel = 0; pixel < 240 * 280; ++pixel) {
        lv_color32_t color;
        color.full = lv_color_to32(framebuffer[pixel]);
        fputc(color.ch.red, file); fputc(color.ch.green, file); fputc(color.ch.blue, file);
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
    display.hor_res = 240; display.ver_res = 280;
    display.draw_buf = &buffer; display.flush_cb = flush;
    lv_disp_drv_register(&display);
    lv_indev_drv_init(&input);
    input.type = LV_INDEV_TYPE_POINTER; input.read_cb = read_touch;
    input.gesture_min_velocity = 0; input.gesture_limit = 40;
    lv_indev_drv_register(&input);
    App_UI_Create(); advance(30);
    assert(s_page == PAGE_WATCH && host_light == 30);
    screenshot("watch");
    key_back(); assert(s_page == PAGE_WATCH && s_history_count == 0);
    /* Horizontal swipe begins on the arc and then on a card. */
    drag(180, 140, -120, 0);
    assert(s_page == PAGE_SENSORS && s_history_count == 1);
    screenshot("sensors");
    drag(170, 100, -110, 0);
    assert(s_page == PAGE_SETTINGS && s_history_count == 2);
    screenshot("settings");
    /* Slider is isolated from page navigation. */
    drag(77, 129, 85, 0);
    assert(s_page == PAGE_SETTINGS && host_light > 30 && s_history_count == 2);
    /* A short electrical glitch must not trigger Back. */
    host_key = GPIO_PIN_RESET; advance(10);
    host_key = GPIO_PIN_SET; advance(60);
    assert(s_page == PAGE_SETTINGS);
    key_back(); assert(s_page == PAGE_SENSORS);
    drag(115, 216, 0, -100);
    assert(s_page == PAGE_SENSORS && lv_obj_get_scroll_y(s_body) > 0);
    screenshot("sensors_scrolled");
    drag(115, 100, 0, 120);
    assert(s_page == PAGE_SENSORS && lv_obj_get_scroll_y(s_body) == 0);
    tap(100, 93); assert(s_page == PAGE_IMU);
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
        tap((area.x1 + area.x2)/2, (area.y1 + area.y2)/2);
    }
    assert(host_mag_cal_state == APP_MAG_CAL_RUNNING);
    assert(strcmp(lv_label_get_text(s_mag_cal_label), "FINISH CALIBRATION") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / CALIBRATING") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "COLLECTING: 120") != NULL);
    screenshot("motion_calibrating");
    {
        lv_area_t area;
        lv_obj_get_coords(lv_obj_get_parent(s_mag_cal_label), &area);
        tap((area.x1 + area.x2)/2, (area.y1 + area.y2)/2);
    }
    assert(host_mag_calibrated);
    assert(strcmp(lv_label_get_text(s_yaw_source), "MAG NORTH / fused yaw") == 0);
    assert(strcmp(lv_label_get_text(s_mag_status), "CAL OK / MAG ONLINE") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "CAL SAVED: 120") != NULL);
    lv_obj_scroll_to_view(s_mag_cal_status, LV_ANIM_OFF); advance(30);
    screenshot("motion_calibration_status");
    lv_obj_scroll_to_y(s_body, 340, LV_ANIM_OFF); advance(30);
    screenshot("motion_magnetic");
    host_mag_online = 0; advance(120);
    assert(strcmp(lv_label_get_text(s_mag_values[0]), "--") == 0);
    assert(strcmp(lv_label_get_text(s_angles[2]), "5.2") == 0);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / MAG NO DATA") == 0);
    host_mag_online = 1; advance(120);
    host_mag_persist_state = APP_MAG_PERSIST_RESTORED; advance(120);
    assert(strcmp(lv_label_get_text(s_mag_status), "EEPROM CAL / MAG ONLINE") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "CAL RESTORED / EEPROM") != NULL);
    lv_obj_scroll_to_view(s_mag_cal_status, LV_ANIM_OFF); advance(30);
    screenshot("motion_restored");
    host_mag_persist_state = APP_MAG_PERSIST_SAVE_FAILED; advance(120);
    assert(strcmp(lv_label_get_text(s_mag_status), "CAL OK / SAVE FAILED") == 0);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "RAM only") != NULL);
    assert(strcmp(lv_label_get_text(s_yaw_source), "MAG NORTH / fused yaw") == 0);
    screenshot("motion_save_failed");
    host_mag_cal_state = APP_MAG_CAL_NONE;
    host_mag_persist_state = APP_MAG_PERSIST_LOAD_FAILED; advance(120);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "EEPROM READ FAILED") != NULL);
    host_mag_cal_state = APP_MAG_CAL_DONE;
    host_mag_persist_state = APP_MAG_PERSIST_SAVED; advance(120);
    host_mag_fusion_state = APP_MAG_FUSION_FIELD_RANGE; advance(120);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / FIELD RANGE") == 0);
    host_mag_fusion_state = APP_MAG_FUSION_HORIZONTAL_WEAK; advance(120);
    assert(strcmp(lv_label_get_text(s_yaw_source), "GYRO ONLY / HORIZ <5uT") == 0);
    host_mag_cal_state = APP_MAG_CAL_FAILED; advance(120);
    assert(strstr(lv_label_get_text(s_mag_cal_status), "CAL FAILED: 120") != NULL);
    host_mag_cal_state = APP_MAG_CAL_DONE;
    host_mag_fusion_state = APP_MAG_FUSION_ACTIVE; advance(120);
    lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF); advance(30);
    drag(110, 216, 0, -100);
    assert(s_page == PAGE_IMU && lv_obj_get_scroll_y(s_body) > 0);
    screenshot("motion_scrolled");
    host_online = 0; advance(120);
    assert(strcmp(lv_label_get_text(s_angles[0]), "--") == 0);
    assert(strcmp(lv_label_get_text(s_imu_status), "NO DATA") == 0);
    host_online = 1; advance(120);
    key_back(); assert(s_page == PAGE_SENSORS);
    /* Navigate from a scrolled parent, then restore its exact position. */
    drag(115, 216, 0, -60);
    saved_scroll = lv_obj_get_scroll_y(s_body);
    tap(100, 182 - saved_scroll);
    assert(s_page == PAGE_BARO);
    assert(strcmp(lv_label_get_text(s_pressure), "1013.25") == 0);
    assert(strcmp(lv_label_get_text(s_altitude), "-0.5") == 0);
    screenshot("environment");
    key_back();
    assert(s_page == PAGE_SENSORS && lv_obj_get_scroll_y(s_body) == saved_scroll);
    drag(60, 140, 120, 0); assert(s_page == PAGE_WATCH);
    /* Real gestures, rendering and automatic old-screen deletion are stressed. */
    lv_mem_monitor(&before);
    for (iteration = 0; iteration < 100; ++iteration) {
        drag(180, 140, -120, 0); assert(s_page == PAGE_SENSORS);
        key_back(); assert(s_page == PAGE_WATCH);
    }
    lv_mem_monitor(&after);
    assert(after.free_size >= before.free_size - 512);
    assert(min_free > 4096);
    /* Back never repeats while the key is held. */
    drag(180, 140, -120, 0);
    drag(170, 100, -110, 0);
    host_key = GPIO_PIN_RESET; advance(1000);
    assert(s_page == PAGE_SETTINGS);
    host_key = GPIO_PIN_SET; advance(250);
    assert(s_page == PAGE_SENSORS);
    advance(500); assert(s_page == PAGE_SENSORS);
    /* History is bounded even when forward navigation revisits a page. */
    key_back();
    while (s_history_count) key_back();
    assert(s_page == PAGE_WATCH);
    for (iteration = 0; iteration < 10; ++iteration) {
        drag(180, 140, -120, 0); assert(s_page == PAGE_SENSORS);
        drag(60, 140, 120, 0); assert(s_page == PAGE_WATCH);
    }
    assert(s_history_count == UI_HISTORY);
    for (iteration = 0; iteration < UI_HISTORY; ++iteration) key_back();
    assert(s_page == PAGE_WATCH && s_history_count == 0);
    key_back(); assert(s_page == PAGE_WATCH);
    /* A slow swipe with repeated samples still accumulates enough distance. */
    host_point.x = 180; host_point.y = 140;
    host_touch = LV_INDEV_STATE_PRESSED; advance(20);
    for (iteration = 0; iteration < 60; ++iteration) {
        --host_point.x; advance(10);
    }
    host_touch = LV_INDEV_STATE_RELEASED; advance(250);
    assert(s_page == PAGE_SENSORS);
    printf("PASS: swipes, vertical scroll, slider, sensor states, key debounce, history restoration, 200 transitions.\n");
    printf("PASS: 16-entry history overflow/return, empty history, slow swipes with repeated coordinates.\n");
    printf("LVGL 40 KiB pool: minimum free %u bytes; free before/after stress %lu/%lu bytes.\n",
           min_free, (unsigned long)before.free_size, (unsigned long)after.free_size);
    return 0;
}
