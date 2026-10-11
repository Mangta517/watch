/**
 * @file app_ui.c
 * @brief 240x280 dark watch UI. All LVGL calls run in the display task.
 *        Horizontal gestures navigate; vertical drags scroll the page body.
 *        KEY_1 (PA5, active low) goes back through page/scroll history.
 */
#include "app_ui.h"
#include "app_imu.h"
#include "app_baro.h"
#include "lcd.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define UI_BG         0x080C12
#define UI_CARD       0x121B28
#define UI_BORDER     0x253348
#define UI_TEXT       0xC5CFDD
#define UI_MUTED      0x7B8BA2
#define UI_BLUE       0x386AC2
#define UI_ORANGE     0xCE7438
#define UI_GREEN      0x398B70
#define UI_RED        0xBA5057
#define UI_POLL_MS    10
#define UI_REFRESH_MS 100
#define UI_ANIM_MS    160
#define UI_KEY_MS     30
#define UI_HISTORY    16

typedef enum
{
    PAGE_WATCH, PAGE_SENSORS, PAGE_SETTINGS, PAGE_IMU, PAGE_BARO, PAGE_COUNT
} ui_page_t;

typedef struct
{
    ui_page_t page;
    lv_coord_t scroll_y;
} ui_history_t;

static lv_obj_t *s_screen;
static lv_obj_t *s_body;
static lv_obj_t *s_time;
static lv_obj_t *s_seconds;
static lv_obj_t *s_ring;
static lv_obj_t *s_imu_status;
static lv_obj_t *s_baro_status;
static lv_obj_t *s_angles[3];
static lv_obj_t *s_mag_values[3];
static lv_obj_t *s_mag_status;
static lv_obj_t *s_yaw_source;
static lv_obj_t *s_mag_cal_label;
static lv_obj_t *s_mag_cal_status;
static lv_obj_t *s_pressure;
static lv_obj_t *s_altitude;
static lv_obj_t *s_brightness_label;
static ui_page_t s_page;
static ui_history_t s_history[UI_HISTORY];
static uint8_t s_history_count;
static uint8_t s_brightness = 30;
static uint8_t s_pending_back;
static ui_page_t s_pending_page = PAGE_COUNT;
static uint8_t s_transition;
static uint32_t s_transition_tick;
static uint32_t s_refresh_tick;
static uint32_t s_last_second = UINT32_MAX;
static uint8_t s_key_raw;
static uint8_t s_key_stable;
static uint8_t s_key_armed;
static uint32_t s_key_tick;

static void ui_timer_cb(lv_timer_t *timer);
static void refresh_data(void);
static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back);

/* Explicit styles keep the built-in light theme out of the entire UI. */
static lv_obj_t *panel(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, 16, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    return obj;
}

static lv_obj_t *label(lv_obj_t *parent, const char *text, int x, int y, const lv_font_t *font, uint32_t color)
{
    lv_obj_t *obj = lv_label_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_style_text_font(obj, font, 0);
    lv_obj_set_style_text_color(obj, lv_color_hex(color), 0);
    lv_obj_set_pos(obj, x, y);
    lv_label_set_text(obj, text);
    return obj;
}

static lv_obj_t *note(lv_obj_t *parent, const char *text, int y)
{
    lv_obj_t *obj = label(parent, text, 4, y, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(obj, 204);
    lv_label_set_long_mode(obj, LV_LABEL_LONG_WRAP);
    return obj;
}

static void navigate_cb(lv_event_t *event)
{
    const ui_page_t *page = (const ui_page_t *)lv_event_get_user_data(event);
    if (!s_transition && !s_pending_back)
    {
        s_pending_page = *page;
    }
}

static void gesture_cb(lv_event_t *event)
{
    lv_indev_t *indev = lv_indev_get_act();
    lv_dir_t dir;
    (void)event;
    if (indev == NULL || s_transition || s_pending_back)
    {
        return;
    }
    dir = lv_indev_get_gesture_dir(indev);
    if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT)
    {
        return;
    }
    if (s_page <= PAGE_SETTINGS)
    {
        if (dir == LV_DIR_LEFT && s_page < PAGE_SETTINGS)
        {
            s_pending_page = (ui_page_t)(s_page + 1);
        }
        else if (dir == LV_DIR_RIGHT && s_page > PAGE_WATCH)
        {
            s_pending_page = (ui_page_t)(s_page - 1);
        }
    }
    else if (dir == LV_DIR_RIGHT)
    {
        s_pending_back = 1;
    }
    /* A swipe must never also click the card under the finger. */
    lv_indev_wait_release(indev);
}

static lv_obj_t *card(const char *icon, const char *title, const char *subtitle, int y, uint32_t accent, const ui_page_t *destination)
{
    lv_obj_t *obj = panel(s_body, 0, y, 212, 76, UI_CARD);
    lv_obj_t *badge = panel(obj, 12, 16, 40, 40, UI_BORDER);
    lv_obj_t *symbol = label(badge, icon, 0, 0, &lv_font_montserrat_20, accent);
    lv_obj_center(symbol);
    label(obj, title, 64, 13, &lv_font_montserrat_14, UI_TEXT);
    label(obj, subtitle, 64, 38, &lv_font_montserrat_10, UI_MUTED);
    label(obj, LV_SYMBOL_RIGHT, 192, 29, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(obj, lv_color_hex(UI_BORDER), LV_STATE_PRESSED);
    lv_obj_add_event_cb(obj, navigate_cb, LV_EVENT_CLICKED, (void *)destination);
    lv_obj_clear_flag(badge, LV_OBJ_FLAG_CLICKABLE);
    return obj;
}

static void watch_page(void)
{
    static const ui_page_t sensors = PAGE_SENSORS;
    static const ui_page_t settings = PAGE_SETTINGS;
    lv_obj_t *caption;
    s_ring = lv_arc_create(s_body);
    lv_obj_remove_style_all(s_ring);
    lv_obj_set_size(s_ring, 180, 180);
    lv_obj_set_pos(s_ring, 16, 2);
    lv_arc_set_bg_angles(s_ring, 0, 360);
    lv_arc_set_rotation(s_ring, 270);
    lv_arc_set_range(s_ring, 0, 60);
    lv_obj_set_style_arc_color(s_ring, lv_color_hex(UI_BORDER), LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_ring, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_ring, lv_color_hex(UI_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(s_ring, 5, LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(s_ring, true, LV_PART_INDICATOR);
    lv_obj_clear_flag(s_ring, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    caption = label(s_body, "RUN TIME", 0, 43, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(caption, 212);
    lv_obj_set_style_text_align(caption, LV_TEXT_ALIGN_CENTER, 0);
    s_time = label(s_body, "00:00", 0, 65, &lv_font_montserrat_30, UI_TEXT);
    lv_obj_set_width(s_time, 212);
    lv_obj_set_style_text_align(s_time, LV_TEXT_ALIGN_CENTER, 0);
    s_seconds = label(s_body, "00 SEC", 0, 107, &lv_font_montserrat_14, UI_ORANGE);
    lv_obj_set_width(s_seconds, 212);
    lv_obj_set_style_text_align(s_seconds, LV_TEXT_ALIGN_CENTER, 0);
    caption = label(s_body, "SYSTEM ACTIVE", 0, 135, &lv_font_montserrat_10, UI_GREEN);
    lv_obj_set_width(caption, 212);
    lv_obj_set_style_text_align(caption, LV_TEXT_ALIGN_CENTER, 0);
    card(LV_SYMBOL_CHARGE, "Sensors", "Motion / pressure", 194, UI_BLUE, &sensors);
    card(LV_SYMBOL_SETTINGS, "Settings", "Display / brightness", 282, UI_ORANGE, &settings);
    note(s_body, "Time elapsed since power-on.", 371);
}

static void sensors_page(void)
{
    static const ui_page_t imu = PAGE_IMU;
    static const ui_page_t baro = PAGE_BARO;
    lv_obj_t *obj;
    label(s_body, "Know your surroundings", 0, 4, &lv_font_montserrat_10, UI_MUTED);
    obj = card(LV_SYMBOL_REFRESH, "Motion", "IMU + MAG / 9 AXES", 26, UI_BLUE, &imu);
    s_imu_status = label(obj, "WAITING", 64, 55, &lv_font_montserrat_10, UI_MUTED);
    obj = card(LV_SYMBOL_UP, "Environment", "SPL006 / BAROMETER", 114, UI_ORANGE, &baro);
    s_baro_status = label(obj, "WAITING", 64, 55, &lv_font_montserrat_10, UI_MUTED);
    obj = panel(s_body, 0, 202, 212, 104, UI_CARD);
    label(obj, "LIVE DATA", 14, 13, &lv_font_montserrat_10, UI_BLUE);
    label(obj, "Open a card to view\nangles, pressure and\nestimated altitude.", 14, 34, &lv_font_montserrat_14, UI_TEXT);
    note(s_body, "Swipe sideways to switch pages.", 322);
}

static lv_obj_t *metric(const char *title, const char *unit, int y, uint32_t color)
{
    lv_obj_t *obj = panel(s_body, 0, y, 212, 88, UI_CARD);
    lv_obj_t *value;
    label(obj, title, 14, 12, &lv_font_montserrat_10, UI_MUTED);
    label(obj, unit, 173, 12, &lv_font_montserrat_10, color);
    value = label(obj, "--", 14, 35, &lv_font_montserrat_30, color);
    lv_obj_set_width(value, 184);
    return value;
}

static void mag_calibrate_cb(lv_event_t *event)
{
    app_imu_struct imu;
    (void)event;
    if (s_transition)
    {
        return;
    }
    (void)App_IMU_GetSnapshot(&imu);
    App_IMU_MagCalibrate(imu.mag_cal_state != APP_MAG_CAL_RUNNING);
}

static void imu_page(void)
{
    lv_obj_t *obj;
    s_imu_status = label(s_body, "WAITING", 2, 4, &lv_font_montserrat_10, UI_MUTED);
    s_angles[0] = metric("PITCH", "DEG", 26, UI_BLUE);
    s_angles[1] = metric("ROLL", "DEG", 126, UI_ORANGE);
    s_angles[2] = metric("YAW", "DEG", 226, UI_GREEN);
    s_yaw_source = note(s_body, "GYRO ONLY / heading may drift", 326);
    s_mag_status = label(s_body, "MAG WAITING", 2, 360, &lv_font_montserrat_10, UI_MUTED);
    s_mag_values[0] = metric("MAG X", "uT", 384, UI_BLUE);
    s_mag_values[1] = metric("MAG Y", "uT", 484, UI_ORANGE);
    s_mag_values[2] = metric("MAG Z", "uT", 584, UI_GREEN);
    obj = panel(s_body, 0, 684, 212, 48, UI_BORDER);
    s_mag_cal_label = label(obj, "CALIBRATE MAG", 14, 16, &lv_font_montserrat_14, UI_TEXT);
    lv_obj_clear_flag(s_mag_cal_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, mag_calibrate_cb, LV_EVENT_CLICKED, NULL);
    s_mag_cal_status = note(s_body, "CAL NOT RUN", 744);
    note(s_body, "Rotate every axis for 30-60s, then tap FINISH. EEPROM keeps successful calibration.", 790);
}

static void baro_page(void)
{
    s_baro_status = label(s_body, "WAITING", 2, 4, &lv_font_montserrat_10, UI_MUTED);
    s_pressure = metric("PRESSURE", "hPa", 26, UI_ORANGE);
    s_altitude = metric("ALTITUDE", "m", 126, UI_BLUE);
    note(s_body, "Altitude uses standard sea-level pressure. Weather can change this estimate.", 232);
    note(s_body, "SPL006 / live sensor snapshot", 286);
}

static void brightness_cb(lv_event_t *event)
{
    s_brightness = (uint8_t)lv_slider_get_value(lv_event_get_target(event));
    LCD_Set_Light(s_brightness);
    lv_label_set_text_fmt(s_brightness_label, "%d%%", (int)s_brightness);
}

static void settings_page(void)
{
    lv_obj_t *obj = panel(s_body, 0, 4, 212, 128, UI_CARD);
    lv_obj_t *slider;
    label(obj, "DISPLAY", 14, 14, &lv_font_montserrat_10, UI_BLUE);
    label(obj, "Brightness", 14, 38, &lv_font_montserrat_14, UI_TEXT);
    s_brightness_label = label(obj, "", 151, 38, &lv_font_montserrat_14, UI_ORANGE);
    lv_label_set_text_fmt(s_brightness_label, "%d%%", (int)s_brightness);
    slider = lv_slider_create(obj);
    lv_obj_remove_style_all(slider);
    lv_obj_set_pos(slider, 24, 82);
    lv_obj_set_size(slider, 164, 10);
    lv_slider_set_range(slider, 10, 100);
    lv_slider_set_value(slider, s_brightness, LV_ANIM_OFF);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(UI_BORDER), LV_PART_MAIN);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(UI_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_bg_color(slider, lv_color_hex(UI_ORANGE), LV_PART_KNOB);
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_pad_all(slider, 7, LV_PART_KNOB);
    lv_obj_set_ext_click_area(slider, 16);
    /* A brightness adjustment must not navigate to another page. */
    lv_obj_clear_flag(slider, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(slider, brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);
    obj = panel(s_body, 0, 144, 212, 136, UI_CARD);
    label(obj, "CONTROLS", 14, 14, &lv_font_montserrat_10, UI_ORANGE);
    label(obj, "Left / right", 14, 38, &lv_font_montserrat_14, UI_TEXT);
    label(obj, "Switch screens", 14, 59, &lv_font_montserrat_10, UI_MUTED);
    label(obj, "Up / down", 14, 81, &lv_font_montserrat_14, UI_TEXT);
    label(obj, "Scroll content", 14, 102, &lv_font_montserrat_10, UI_MUTED);
    obj = panel(s_body, 0, 292, 212, 84, UI_CARD);
    label(obj, LV_SYMBOL_LEFT "  SIDE BUTTON", 14, 16, &lv_font_montserrat_14, UI_TEXT);
    label(obj, "Return to the previous page", 14, 44, &lv_font_montserrat_10, UI_MUTED);
    note(s_body, "Brightness resets after restart.", 392);
}

static void clear_page_refs(void)
{
    s_time = s_seconds = s_ring = NULL;
    s_imu_status = s_baro_status = NULL;
    s_pressure = s_altitude = s_brightness_label = NULL;
    memset(s_angles, 0, sizeof(s_angles));
    memset(s_mag_values, 0, sizeof(s_mag_values));
    s_mag_status = s_yaw_source = s_mag_cal_label = NULL;
    s_mag_cal_status = NULL;
    s_last_second = UINT32_MAX;
}

static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back)
{
    static const char *titles[] = { "WATCH", "SENSORS", "SETTINGS", "MOTION", "ENVIRONMENT" };
    lv_obj_t *dot;
    lv_obj_t *old_screen = s_screen;
    uint8_t i;
    uint8_t tab = (page <= PAGE_SETTINGS) ? (uint8_t)page : PAGE_SENSORS;
    int width = lv_disp_get_hor_res(NULL);
    int height = lv_disp_get_ver_res(NULL);
    clear_page_refs();
    s_page = page;
    s_screen = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_screen);
    lv_obj_set_style_bg_color(s_screen, lv_color_hex(UI_BG), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(s_screen, gesture_cb, LV_EVENT_GESTURE, NULL);
    label(s_screen, titles[page], 14, 13, &lv_font_montserrat_14, UI_TEXT);
    label(s_screen, page == PAGE_WATCH ? "RUN" : (page == PAGE_SETTINGS ? "LCD" : "LIVE"), width - 43, 15,
          &lv_font_montserrat_10, UI_MUTED);
    s_body = panel(s_screen, (width - 212) / 2, 38, 212, height - 62, UI_BG);
    lv_obj_set_style_radius(s_body, 0, 0);
    lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_body, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_body, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_bg_color(s_body, lv_color_hex(UI_BLUE), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(s_body, LV_OPA_60, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(s_body, 2, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(s_body, 0, LV_PART_SCROLLBAR);
    switch (page)
    {
        case PAGE_WATCH:    watch_page(); break;
        case PAGE_SENSORS:  sensors_page(); break;
        case PAGE_SETTINGS: settings_page(); break;
        case PAGE_IMU:      imu_page(); break;
        case PAGE_BARO:     baro_page(); break;
        default: break;
    }
    for (i = 0; i < 3; ++i)
    {
        dot = panel(s_screen, width / 2 - 20 + i * 16, height - 14, i == tab ? 12 : 5, 5, i == tab ? UI_BLUE : UI_BORDER);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
    }
    label(s_screen, s_history_count ? LV_SYMBOL_LEFT " KEY" : "SWIPE", 14, height - 17, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_update_layout(s_screen);
    lv_obj_scroll_to_y(s_body, scroll_y, LV_ANIM_OFF);
    refresh_data();
    if (old_screen == NULL)
    {
        lv_scr_load(s_screen);
    }
    else
    {
        s_transition = 1;
        s_transition_tick = lv_tick_get();
        lv_scr_load_anim(s_screen, back ? LV_SCR_LOAD_ANIM_MOVE_RIGHT : LV_SCR_LOAD_ANIM_MOVE_LEFT, UI_ANIM_MS, 0, true);
    }
}

static void set_status(lv_obj_t *obj, uint8_t online)
{
    if (obj == NULL)
    {
        return;
    }
    lv_label_set_text(obj, online ? "ONLINE" : "NO DATA");
    lv_obj_set_style_text_color(obj, lv_color_hex(online ? UI_GREEN : UI_RED), 0);
}

/* Fixed-point text avoids pulling printf floating-point support into firmware. */
static void set_decimal(lv_obj_t *obj, float value, uint8_t wrap)
{
    int32_t fixed;
    int32_t magnitude;
    if (!isfinite(value) || (!wrap && (value > 99999.0f || value < -99999.0f)))
    {
        lv_label_set_text(obj, "--");
        return;
    }
    if (wrap)
    {
        value = fmodf(value, 360.0f);
        if (value > 180.0f)
        {
            value -= 360.0f;
        }
        if (value <= -180.0f)
        {
            value += 360.0f;
        }
    }
    fixed = (int32_t)(value * 10.0f);
    magnitude = fixed < 0 ? -fixed : fixed;
    lv_label_set_text_fmt(obj, "%s%ld.%ld", value < 0.0f ? "-" : "", (long)(magnitude / 10), (long)(magnitude % 10));
}

/* 航向暂停原因直接来自采样服务，不通过“有磁场数字”猜测融合状态。 */
static const char *mag_yaw_status(const app_imu_struct *imu)
{
    switch (imu->mag_fusion_state)
    {
        case APP_MAG_FUSION_ACTIVE:
            return imu->mag_calibrated ? "MAG NORTH / fused yaw" : "MAG UNCAL / calibrate first";
        case APP_MAG_FUSION_CALIBRATING:
            return "GYRO ONLY / CALIBRATING";
        case APP_MAG_FUSION_FIELD_RANGE:
            return "GYRO ONLY / FIELD RANGE";
        case APP_MAG_FUSION_HORIZONTAL_WEAK:
            return "GYRO ONLY / HORIZ <5uT";
        case APP_MAG_FUSION_IMU_NO_DATA:
            return "GYRO ONLY / NO IMU FRAME";
        default:
            return "GYRO ONLY / MAG NO DATA";
    }
}

static void mag_calibration_status(const app_imu_struct *imu)
{
    const char *result;
    if (imu->mag_cal_state == APP_MAG_CAL_NONE)
    {
        lv_label_set_text(s_mag_cal_status, imu->mag_persist_state == APP_MAG_PERSIST_LOAD_FAILED ?
            "EEPROM READ FAILED\nCalibration required" : "CAL NOT RUN");
        return;
    }
    if (imu->mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        result = "COLLECTING";
    }
    else if (imu->mag_cal_state == APP_MAG_CAL_DONE)
    {
        switch (imu->mag_persist_state)
        {
            case APP_MAG_PERSIST_RESTORED:
                lv_label_set_text(s_mag_cal_status, "CAL RESTORED / EEPROM\nReady for magnetic fusion");
                return;
            case APP_MAG_PERSIST_SAVING:
                lv_label_set_text(s_mag_cal_status, "CAL OK / SAVING EEPROM");
                return;
            case APP_MAG_PERSIST_SAVE_FAILED:
                lv_label_set_text(s_mag_cal_status, "CAL OK / SAVE FAILED\nRAM only: check EEPROM/WP");
                return;
            case APP_MAG_PERSIST_SAVED:
                result = "CAL SAVED";
                break;
            default:
                result = "CAL OK";
                break;
        }
    }
    else
    {
        result = "CAL FAILED";
    }
    /* 显示帧数与三轴覆盖幅度，保留失败证据，方便再次旋转校准。 */
    lv_label_set_text_fmt(s_mag_cal_status, "%s: %u samples\nX/Y/Z span: %ld/%ld/%ld uT", result,
        (unsigned)imu->mag_cal_samples, (long)imu->mag_cal_span_uT[0],
        (long)imu->mag_cal_span_uT[1], (long)imu->mag_cal_span_uT[2]);
}

static void refresh_data(void)
{
    app_imu_struct imu;
    app_baro_struct baro;
    uint8_t online;
    uint32_t seconds;
    if (s_time != NULL)
    {
        seconds = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
        if (seconds != s_last_second)
        {
            s_last_second = seconds;
            lv_label_set_text_fmt(s_time, "%02lu:%02lu", (unsigned long)(seconds / 3600), (unsigned long)((seconds / 60) % 60));
            lv_label_set_text_fmt(s_seconds, "%02lu SEC", (unsigned long)(seconds % 60));
            lv_arc_set_value(s_ring, (int16_t)(seconds % 60));
        }
    }
    if (s_imu_status != NULL)
    {
        online = App_IMU_GetSnapshot(&imu);
        set_status(s_imu_status, online);
        if (s_angles[0] != NULL)
        {
            if (online)
            {
                set_decimal(s_angles[0], imu.pitch_deg, 0);
                set_decimal(s_angles[1], imu.roll_deg, 0);
                set_decimal(s_angles[2], imu.yaw_deg, 1);
            }
            else
            {
                lv_label_set_text(s_angles[0], "--");
                lv_label_set_text(s_angles[1], "--");
                lv_label_set_text(s_angles[2], "--");
            }
            lv_label_set_text(s_yaw_source, online ? mag_yaw_status(&imu) : "GYRO ONLY / NO IMU FRAME");
            if (imu.mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                lv_label_set_text(s_mag_status, "MAG CALIBRATING");
            }
            else if (imu.mag_cal_state == APP_MAG_CAL_FAILED)
            {
                lv_label_set_text(s_mag_status, "CAL FAILED / rotate all axes");
            }
            else if (imu.mag_cal_state == APP_MAG_CAL_DONE)
            {
                if (imu.mag_persist_state == APP_MAG_PERSIST_SAVE_FAILED)
                {
                    lv_label_set_text(s_mag_status, "CAL OK / SAVE FAILED");
                }
                else if (imu.mag_persist_state == APP_MAG_PERSIST_RESTORED)
                {
                    lv_label_set_text(s_mag_status, imu.mag_online ? "EEPROM CAL / MAG ONLINE" : "EEPROM CAL / MAG NO DATA");
                }
                else
                {
                    lv_label_set_text(s_mag_status, imu.mag_online ? "CAL OK / MAG ONLINE" : "CAL OK / MAG NO DATA");
                }
            }
            else
            {
                lv_label_set_text(s_mag_status, imu.mag_online ? "LSM303DLHC / ONLINE" : "LSM303DLHC / NO DATA");
            }
            lv_obj_set_style_text_color(s_mag_status,
                lv_color_hex(imu.mag_online && imu.mag_persist_state != APP_MAG_PERSIST_SAVE_FAILED ? UI_GREEN : UI_RED), 0);
            lv_label_set_text(s_mag_cal_label, imu.mag_cal_state == APP_MAG_CAL_RUNNING ? "FINISH CALIBRATION" : "CALIBRATE MAG");
            mag_calibration_status(&imu);
            {
                uint8_t i;
                for (i = 0; i < 3; ++i)
                {
                    if (imu.mag_online)
                    {
                        set_decimal(s_mag_values[i], imu.mag_body_uT[i], 0);
                    }
                    else
                    {
                        lv_label_set_text(s_mag_values[i], "--");
                    }
                }
            }
        }
    }
    if (s_baro_status != NULL)
    {
        online = App_Baro_GetSnapshot(&baro);
        set_status(s_baro_status, online);
        if (s_pressure != NULL)
        {
            if (online && baro.pressure_pa > 0)
            {
                lv_label_set_text_fmt(s_pressure, "%ld.%02ld", (long)(baro.pressure_pa / 100), (long)(baro.pressure_pa % 100));
                set_decimal(s_altitude, baro.alt_m, 0);
            }
            else
            {
                lv_label_set_text(s_pressure, "--");
                lv_label_set_text(s_altitude, "--");
            }
        }
    }
}

void App_UI_Back(void)
{
    /* Called only in the LVGL task, or while holding App_Display_Lock(). */
    s_pending_page = PAGE_COUNT;
    s_pending_back = 1;
}

static void poll_key(void)
{
    uint8_t pressed = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_RESET;
    if (pressed != s_key_raw)
    {
        s_key_raw = pressed;
        s_key_tick = lv_tick_get();
    }
    if (s_key_raw != s_key_stable && lv_tick_elaps(s_key_tick) >= UI_KEY_MS)
    {
        s_key_stable = s_key_raw;
        if (s_key_stable)
        {
            s_key_armed = 1;
        }
        else if (s_key_armed)
        {
            s_key_armed = 0;
            App_UI_Back();
        }
    }
}

static uint8_t touch_pressed(void)
{
    lv_indev_t *indev = NULL;
    while ((indev = lv_indev_get_next(indev)) != NULL)
    {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER && indev->proc.state == LV_INDEV_STATE_PRESSED)
        {
            return 1;
        }
    }
    return 0;
}

static void ui_timer_cb(lv_timer_t *timer)
{
    ui_history_t previous;
    ui_page_t next;
    (void)timer;
    poll_key();
    if (s_transition && lv_tick_elaps(s_transition_tick) >= UI_ANIM_MS + UI_POLL_MS)
    {
        s_transition = 0;
    }
    /* Rebuild only after release and after the previous animation, so event
     * targets stay valid and a gesture cannot turn into a card click. */
    if (!s_transition && !touch_pressed())
    {
        if (s_pending_back)
        {
            s_pending_back = 0;
            s_pending_page = PAGE_COUNT;
            if (s_history_count != 0)
            {
                previous = s_history[--s_history_count];
                create_page(previous.page, previous.scroll_y, 1);
            }
        }
        else if (s_pending_page != PAGE_COUNT)
        {
            next = s_pending_page;
            s_pending_page = PAGE_COUNT;
            if (next != s_page)
            {
                if (s_history_count == UI_HISTORY)
                {
                    memmove(s_history, s_history + 1, sizeof(s_history[0]) * (UI_HISTORY - 1));
                    --s_history_count;
                }
                s_history[s_history_count].page = s_page;
                s_history[s_history_count].scroll_y = lv_obj_get_scroll_y(s_body);
                ++s_history_count;
                create_page(next, 0, next <= PAGE_SETTINGS && s_page <= PAGE_SETTINGS && next < s_page);
            }
        }
    }
    if (lv_tick_elaps(s_refresh_tick) >= UI_REFRESH_MS)
    {
        s_refresh_tick = lv_tick_get();
        refresh_data();
    }
}

void App_UI_Create(void)
{
    lv_obj_t *boot_screen;
    if (s_screen != NULL)
    {
        return;
    }
    boot_screen = lv_scr_act();
    create_page(PAGE_WATCH, 0, 0);
    lv_obj_del(boot_screen);
    LCD_Set_Light(s_brightness);
    /* A key held during boot must be released before it can trigger Back. */
    s_key_raw = s_key_stable = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_RESET;
    s_key_tick = s_refresh_tick = lv_tick_get();
    lv_timer_create(ui_timer_cb, UI_POLL_MS, NULL);
}
