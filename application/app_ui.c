/**
 * @file app_ui.c
 * @brief 240x280 dark watch UI. All LVGL calls run in the display task.
 *        三针RTC表盘不滚动；每项功能从独立控件进入详情页。
 *        KEY_1 (PA5, active low) goes back through page/scroll history.
 *        WAKE (PA4, active high) toggles the backlight after a debounced press.
 */
#include "app_ui.h"
#include "app_diag.h"
#include "app_imu.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_aht21.h"
#include "app_battery.h"
#include "app_touch.h"
#include "app_task.h"
#include "kt6368a.h"
#include "watch_rtc.h"
#include "app_clock.h"
#include "app_weather.h"
#include "lcd.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

#define UI_BG               0x041426
#define UI_CARD             0x0C2238
#define UI_BORDER           0x173B54
#define UI_TEXT             0xF4FAFF
#define UI_MUTED            0x7EABC1
#define UI_BLUE             0x70C5E8
#define UI_ORANGE           0xCE7438
#define UI_GREEN            0x398B70
#define UI_BATTERY_GREEN    0x45D483
#define UI_RED              0xBA5057
#define UI_POLL_MS          10
#define UI_REFRESH_MS       100
#define UI_ANIM_MS          160
#define UI_KEY_MS           30
#define UI_TOUCH_IDLE_MS    5000U
#define UI_MIN_BRIGHTNESS   2U
#define UI_HISTORY          16
#define UI_BT_LOG_SIZE      512
#define UI_BT_READ_SIZE     64
#define UI_BT_RX_BUDGET     256
#define UI_BT_TX_TIMEOUT_MS 20U
#define UI_TREND_POINTS     24U
#define UI_TREND_SAMPLE_MS  1000U
#define UI_BARO_REFRESH_MS  5000U
#define UI_COMPASS_FILTER_MS 100U
#define UI_COMPASS_MAX_DPS  360.0f

typedef enum
{
    PAGE_WATCH,
    PAGE_SETTINGS,
    PAGE_BLUETOOTH,
    PAGE_DISPLAY,
    PAGE_HEART,
    PAGE_MOTION,
    PAGE_ENVIRONMENT,
    PAGE_AHT21,
    PAGE_WEATHER,
    PAGE_COUNT
} ui_page_t;

typedef struct
{
    ui_page_t page;
    lv_coord_t scroll_y;
} ui_history_t;

/* 两张主页面只在左右切换时驻留；进入详情释放，保持原详情页内存预算。 */
typedef struct
{
    lv_obj_t *screen, *body, *battery, *battery_bar;
    lv_obj_t *time, *hint, *heart, *temperature;
    lv_obj_t *hour_hand, *minute_hand, *second_hand;
} ui_main_page_struct;

static ui_main_page_struct s_main_pages[2];

static lv_obj_t *s_screen;
static lv_obj_t *s_battery_label;
static lv_obj_t *s_battery_bar;
static lv_obj_t *s_weather_label;
static lv_obj_t *s_weather_value;
static lv_obj_t *s_weather_condition;
static lv_obj_t *s_weather_icon;
static uint8_t s_weather_icon_code;
static lv_obj_t *s_body;
static lv_obj_t *s_time;
static lv_obj_t *s_hour_hand;
static lv_obj_t *s_minute_hand;
static lv_obj_t *s_second_hand;
static lv_point_t s_hour_points[2];
static lv_point_t s_minute_points[2];
static lv_point_t s_second_points[2];
static uint32_t s_clock_refresh_tick;
static lv_obj_t *s_watch_heart;
static lv_obj_t *s_watch_temperature;
static lv_obj_t *s_clock_hint;
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
typedef struct
{
    lv_obj_t *chart;
    lv_obj_t *scale;
    lv_obj_t *scale_low;
    lv_chart_series_t *series;
    uint32_t sample_tick;
    uint32_t sample_period_ms;
} ui_trend_struct;

static ui_trend_struct s_altitude_trend;
static ui_trend_struct s_heart_trend;
static lv_obj_t *s_compass;
static lv_obj_t *s_compass_value;
static lv_obj_t *s_compass_status;
static lv_obj_t *s_compass_calibration;
static int16_t s_compass_heading = -1;
static uint32_t s_compass_color;
static float s_compass_display_deg;
static uint32_t s_compass_tick;
static lv_obj_t *s_heart_card;
static lv_obj_t *s_heart_value;
static lv_obj_t *s_heart_status;
static lv_obj_t *s_heart_summary;
static lv_obj_t *s_heart_details;
static lv_obj_t *s_heart_details_title;
static lv_obj_t *s_heart_button;
static lv_obj_t *s_aht21_card;
static lv_obj_t *s_humidity;
static lv_obj_t *s_air_temperature;
static lv_obj_t *s_aht21_status;
static lv_obj_t *s_aht21_samples;
static lv_obj_t *s_brightness_label;
static lv_obj_t *s_bt_status;
static lv_obj_t *s_bt_count;
static lv_obj_t *s_bt_tx_status;
static lv_obj_t *s_bt_clock_status;
static lv_obj_t *s_bt_log;
static lv_obj_t *s_bt_log_box;
/* 接收历史由LVGL任务独占，切页只删除控件，不清除RAM中的内容。 */
static char s_bt_text[UI_BT_LOG_SIZE];
static uint16_t s_bt_text_length;
static uint32_t s_bt_rx_bytes;
static uint32_t s_bt_rx_dropped, s_bt_rx_errors, s_bt_rx_activity_ms;
static uint8_t s_bt_rx_resync;
static uint8_t s_bt_last_cr;
static uint8_t s_bt_dirty;
static const char *s_bt_tx_text = "TX READY";
static uint32_t s_bt_tx_color = UI_MUTED;
static ui_page_t s_page;
static ui_history_t s_history[UI_HISTORY];
static uint8_t s_history_count;
static uint8_t s_brightness = 50;
static uint8_t s_screen_on = 1;
static uint8_t s_idle_dimmed;
static uint8_t s_pending_back;
static ui_page_t s_pending_page = PAGE_COUNT;
static uint8_t s_transition;
static uint32_t s_refresh_tick;
static uint32_t s_last_second = UINT32_MAX;
static uint8_t s_key_raw;
static uint8_t s_key_stable;
static uint8_t s_key_armed;
static uint32_t s_key_tick;
static uint8_t s_wake_raw;
static uint8_t s_wake_stable;
static uint32_t s_wake_tick;

static void ui_timer_cb(lv_timer_t *timer);
static void refresh_data(void);
static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back);
static void refresh_bluetooth(void);
static void refresh_battery(void);

/* 直接比较控件中的最终显示值，切页重建后无需维护额外缓存。 */
static void set_label_text(lv_obj_t *obj, const char *text)
{
    if (strcmp(lv_label_get_text(obj), text) != 0)
    {
        lv_label_set_text(obj, text);
    }
}

static void set_label_text_fmt(lv_obj_t *obj, const char *format, ...)
{
    /* 当前最长的格式化文本为心率诊断和三轴校准状态，均小于128字节。 */
    char text[128];
    va_list args;
    va_start(args, format);
    lv_vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    set_label_text(obj, text);
}

static void set_text_color(lv_obj_t *obj, uint32_t color)
{
    lv_color_t desired = lv_color_hex(color);
    if (lv_obj_get_style_text_color(obj, LV_PART_MAIN).full != desired.full)
    {
        lv_obj_set_style_text_color(obj, desired, LV_PART_MAIN);
    }
}

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

static void create_battery_bar(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *terminal;
    s_battery_bar = lv_bar_create(parent);
    lv_obj_remove_style_all(s_battery_bar);
    lv_obj_set_pos(s_battery_bar, x, y);
    lv_obj_set_size(s_battery_bar, 30, 12);
    lv_obj_set_style_bg_color(s_battery_bar, lv_color_hex(UI_CARD), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_battery_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_battery_bar, lv_color_hex(UI_MUTED), LV_PART_MAIN);
    lv_obj_set_style_border_width(s_battery_bar, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s_battery_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_battery_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_battery_bar, lv_color_hex(UI_BATTERY_GREEN), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_battery_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_battery_bar, 1, LV_PART_INDICATOR);
    lv_obj_clear_flag(s_battery_bar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_battery_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE | LV_OBJ_FLAG_HIDDEN);
    lv_bar_set_range(s_battery_bar, 0, 100);
    terminal = panel(s_battery_bar, 28, 1, 3, 6, UI_MUTED);
    lv_obj_set_style_radius(terminal, 1, 0);
    lv_obj_clear_flag(terminal, LV_OBJ_FLAG_CLICKABLE);
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
        if (s_page == PAGE_WATCH && (dir == LV_DIR_TOP || dir == LV_DIR_BOTTOM))
        {
            lv_indev_wait_release(indev);
        }
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

/* 入口的目标页保存在静态数据中；触摸释放后仍由原定时器切页。 */
static void make_entry(lv_obj_t *obj, const ui_page_t *destination)
{
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(obj, lv_color_hex(UI_BORDER), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_add_event_cb(obj, navigate_cb, LV_EVENT_CLICKED, (void *)destination);
}

/* 连续三次贝塞尔心形按16倍分辨率采样，400字节透明度保存在Flash，平滑连接两瓣与尖端。 */
static void heart_icon_draw_cb(lv_event_t *event)
{
    static const uint8_t opacity[20 * 20] =
    {
        0x00, 0x00, 0x01, 0x04, 0x01, 0x00, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x04, 0x03, 0x00, 0x01, 0x04, 0x01, 0x00, 0x00,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x01, 0x00, 0x0c, 0x7a, 0xc8, 0xd5, 0xaf, 0x55, 0x00, 0x00, 0x00, 0x00, 0x55, 0xaf, 0xd5, 0xc8, 0x7a, 0x0c, 0x00, 0x01,
        0x02, 0x11, 0xcc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xab, 0x05, 0x05, 0xab, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcc, 0x11, 0x02,
        0x00, 0x8f, 0xff, 0xf8, 0xfd, 0xfd, 0xfc, 0xfa, 0xff, 0xa4, 0xa4, 0xff, 0xfa, 0xfc, 0xfd, 0xfd, 0xf8, 0xff, 0x8f, 0x00,
        0x07, 0xdc, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xdc, 0x07,
        0x16, 0xec, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xec, 0x16,
        0x0b, 0xe3, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xff, 0xe3, 0x0b,
        0x00, 0xa8, 0xfe, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xfe, 0xa8, 0x00,
        0x00, 0x3b, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0x3b, 0x00,
        0x03, 0x00, 0x9d, 0xff, 0xfa, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfa, 0xff, 0x9d, 0x00, 0x03,
        0x01, 0x03, 0x0b, 0xce, 0xff, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xff, 0xce, 0x0b, 0x03, 0x01,
        0x00, 0x02, 0x00, 0x1c, 0xe0, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xe0, 0x1c, 0x00, 0x02, 0x00,
        0x00, 0x00, 0x03, 0x00, 0x27, 0xe6, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xe6, 0x27, 0x00, 0x03, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x03, 0x00, 0x26, 0xe2, 0xff, 0xfb, 0xff, 0xff, 0xfb, 0xff, 0xe2, 0x26, 0x00, 0x03, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x26, 0xdc, 0xff, 0xfb, 0xfb, 0xff, 0xdc, 0x26, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x1c, 0xce, 0xff, 0xff, 0xce, 0x1c, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x10, 0xbf, 0xbf, 0x10, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    static const lv_img_dsc_t heart =
    {
        .header = {.cf = LV_IMG_CF_ALPHA_8BIT, .w = 20, .h = 20},
        .data_size = sizeof(opacity),
        .data = opacity
    };
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    lv_draw_img_dsc_t image;
    lv_area_t area;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    lv_draw_img_dsc_init(&image);
    image.recolor = lv_color_hex(UI_BLUE);
    image.recolor_opa = LV_OPA_COVER;
    lv_draw_img(context, &image, &area, &heart);
}

static lv_obj_t *heart_icon(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *obj = panel(parent, x, y, 20, 20, UI_BG);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, heart_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    return obj;
}

static void weather_icon_shape(lv_draw_ctx_t *context, const lv_area_t *origin, int x, int y, int w, int h, uint32_t color)
{
    lv_draw_rect_dsc_t style;
    lv_area_t area = {origin->x1 + x, origin->y1 + y, origin->x1 + x + w - 1, origin->y1 + y + h - 1};
    lv_draw_rect_dsc_init(&style);
    style.radius = LV_RADIUS_CIRCLE;
    style.bg_color = lv_color_hex(color);
    lv_draw_rect(context, &style, &area);
}

static void weather_icon_line(lv_draw_ctx_t *context, const lv_area_t *origin, int x1, int y1, int x2, int y2, uint32_t color)
{
    lv_draw_line_dsc_t style;
    lv_point_t start = {origin->x1 + x1, origin->y1 + y1};
    lv_point_t end = {origin->x1 + x2, origin->y1 + y2};
    lv_draw_line_dsc_init(&style);
    style.color = lv_color_hex(color);
    style.width = 3;
    style.round_start = 1;
    style.round_end = 1;
    lv_draw_line(context, &style, &start, &end);
}

/* 天气码沿用手机协议；矢量图形由显示任务绘制，不额外加载位图或字库。 */
static void weather_icon_draw_cb(lv_event_t *event)
{
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    lv_area_t area;
    uint8_t code = s_weather_icon_code;
    uint8_t i;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    if (code == 255 || strcmp(app_weather_description(code), "UNKNOWN") == 0)
    {
        lv_draw_label_dsc_t style;
        lv_draw_label_dsc_init(&style);
        style.font = &lv_font_montserrat_30;
        style.color = lv_color_hex(UI_MUTED);
        style.align = LV_TEXT_ALIGN_CENTER;
        area.y1 += 16;
        lv_draw_label(context, &style, &area, "?", NULL);
        return;
    }
    if (code <= 2)
    {
        for (i = 0; i < 8; ++i)
        {
            int32_t sine = lv_trigo_sin(i * 45);
            int32_t cosine = lv_trigo_cos(i * 45);
            weather_icon_line(context, &area, 27 + sine * 17 / LV_TRIGO_SIN_MAX, 25 + cosine * 17 / LV_TRIGO_SIN_MAX,
                              27 + sine * 22 / LV_TRIGO_SIN_MAX, 25 + cosine * 22 / LV_TRIGO_SIN_MAX, 0xF5BD58);
        }
        weather_icon_shape(context, &area, 15, 13, 25, 25, 0xF5BD58);
        if (code == 0)
        {
            return;
        }
    }
    if (code == 3)
    {
        weather_icon_shape(context, &area, 9, 17, 24, 24, UI_MUTED);
        weather_icon_shape(context, &area, 22, 12, 29, 29, UI_MUTED);
    }
    weather_icon_shape(context, &area, 7, 29, 25, 25, UI_TEXT);
    weather_icon_shape(context, &area, 22, 21, 30, 30, UI_TEXT);
    weather_icon_shape(context, &area, 40, 32, 20, 20, UI_TEXT);
    weather_icon_shape(context, &area, 17, 37, 36, 17, UI_TEXT);
    if (code == 45 || code == 48)
    {
        weather_icon_line(context, &area, 8, 50, 56, 50, UI_MUTED);
        weather_icon_line(context, &area, 12, 58, 51, 58, UI_MUTED);
    }
    else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82))
    {
        for (i = 0; i < 3; ++i)
        {
            weather_icon_line(context, &area, 18 + i * 15, 56, 14 + i * 15, 62, UI_BLUE);
        }
    }
    else if ((code >= 71 && code <= 77) || code == 85 || code == 86)
    {
        for (i = 0; i < 3; ++i)
        {
            weather_icon_line(context, &area, 14 + i * 16, 58, 20 + i * 16, 58, UI_BLUE);
            weather_icon_line(context, &area, 17 + i * 16, 55, 17 + i * 16, 61, UI_BLUE);
        }
    }
    else if (code >= 95)
    {
        weather_icon_line(context, &area, 35, 46, 27, 54, 0xF5BD58);
        weather_icon_line(context, &area, 27, 54, 36, 54, 0xF5BD58);
        weather_icon_line(context, &area, 36, 54, 28, 62, 0xF5BD58);
    }
}

static void card(const char *icon, const char *title, int index, uint32_t accent, const ui_page_t *destination)
{
    int width;
    if (index == 6)
    {
        width = 212;
    }
    else
    {
        width = 100;
    }
    lv_obj_t *obj = panel(s_body, (index % 2) * 112, 4 + (index / 2) * 80, width, 72, UI_CARD);
    lv_obj_t *symbol;
    lv_obj_t *caption = label(obj, title, 0, 46, &lv_font_montserrat_10, UI_TEXT);
    if (*destination == PAGE_HEART)
    {
        symbol = heart_icon(obj, (width - 20) / 2, 12);
    }
    else
    {
        symbol = label(obj, icon, 0, 12, &lv_font_montserrat_20, accent);
        lv_obj_set_width(symbol, width);
        lv_obj_set_style_text_align(symbol, LV_TEXT_ALIGN_CENTER, 0);
    }
    lv_obj_set_width(caption, width);
    lv_obj_set_style_text_align(caption, LV_TEXT_ALIGN_CENTER, 0);
    make_entry(obj, destination);
}

/* 恢复原60格时钟刻度，使用原生LVGL线条，不占用整屏图片缓冲。 */
static void clock_dial_draw_cb(lv_event_t *event)
{
    lv_obj_t *dial = lv_event_get_target(event);
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    lv_area_t area;
    lv_draw_line_dsc_t style;
    lv_point_t inner, outer;
    int16_t angle, radius, sine, cosine;
    uint8_t i;
    lv_obj_get_coords(dial, &area);
    lv_draw_line_dsc_init(&style);
    for (i = 0; i < 60; ++i)
    {
        angle = i * 6;
        if (i % 5 == 0)
        {
            radius = 82;
            style.color = lv_color_hex(UI_TEXT);
            style.width = 3;
        }
        else
        {
            radius = 88;
            style.color = lv_color_hex(UI_MUTED);
            style.width = 1;
        }
        sine = lv_trigo_sin(angle);
        cosine = lv_trigo_cos(angle);
        inner.x = area.x1 + 100 + (int32_t)sine * radius / LV_TRIGO_SIN_MAX;
        inner.y = area.y1 + 100 - (int32_t)cosine * radius / LV_TRIGO_SIN_MAX;
        outer.x = area.x1 + 100 + (int32_t)sine * 92 / LV_TRIGO_SIN_MAX;
        outer.y = area.y1 + 100 - (int32_t)cosine * 92 / LV_TRIGO_SIN_MAX;
        lv_draw_line(context, &style, &inner, &outer);
    }
}

static lv_obj_t *clock_hand(lv_obj_t *dial, lv_point_t *points, uint8_t width, uint32_t color)
{
    lv_obj_t *hand = lv_line_create(dial);
    lv_obj_remove_style_all(hand);
    lv_obj_set_style_line_color(hand, lv_color_hex(color), 0);
    lv_obj_set_style_line_width(hand, width, 0);
    lv_obj_set_style_line_rounded(hand, true, 0);
    lv_obj_clear_flag(hand, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    points[0].x = points[1].x = 100;
    points[0].y = 100;
    points[1].y = 40;
    lv_line_set_points(hand, points, 2);
    return hand;
}

static void set_clock_hand(lv_obj_t *hand, lv_point_t *points, float degrees, uint8_t radius)
{
    float angle = degrees * 3.14159265f / 180.0f;
    lv_point_t endpoint;
    endpoint.x = (lv_coord_t)(100.0f + sinf(angle) * radius + 0.5f);
    endpoint.y = (lv_coord_t)(100.0f - cosf(angle) * radius + 0.5f);
    if (endpoint.x != points[1].x || endpoint.y != points[1].y)
    {
        points[1] = endpoint;
        lv_line_set_points(hand, points, 2);
    }
}

static void watch_page(void)
{
    static const ui_page_t climate = PAGE_AHT21;
    static const ui_page_t heart = PAGE_HEART;
    static const ui_page_t bluetooth = PAGE_BLUETOOTH;
    lv_obj_t *dial = panel(s_body, 20, 18, 200, 200, UI_BG);
    lv_obj_t *number, *hub, *entry;
    char text[3];
    uint8_t hour;
    lv_obj_set_style_radius(dial, 0, 0);
    lv_obj_set_style_bg_opa(dial, LV_OPA_TRANSP, 0);
    lv_obj_add_event_cb(dial, clock_dial_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    make_entry(dial, &bluetooth);
    /* 刻度盘按下时保持透明，避免入口高亮露出200x200的方形背景。 */
    lv_obj_set_style_bg_opa(dial, LV_OPA_TRANSP, LV_STATE_PRESSED);
    for (hour = 1; hour <= 12; ++hour)
    {
        lv_snprintf(text, sizeof(text), "%u", (unsigned)hour);
        number = label(dial, text, 90 + (int32_t)lv_trigo_sin(hour * 30) * 75 / LV_TRIGO_SIN_MAX,
                       92 - (int32_t)lv_trigo_cos(hour * 30) * 75 / LV_TRIGO_SIN_MAX, &lv_font_montserrat_14, UI_TEXT);
        lv_obj_set_width(number, 20);
        lv_obj_set_style_text_align(number, LV_TEXT_ALIGN_CENTER, 0);
    }
    s_clock_hint = label(dial, "SET VIA BLE", 20, 126, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_clock_hint, 160);
    lv_obj_set_style_text_align(s_clock_hint, LV_TEXT_ALIGN_CENTER, 0);
    s_hour_hand = clock_hand(dial, s_hour_points, 6, UI_TEXT);
    s_minute_hand = clock_hand(dial, s_minute_points, 4, UI_ORANGE);
    s_second_hand = clock_hand(dial, s_second_points, 2, UI_RED);
    hub = panel(dial, 95, 95, 10, 10, UI_BLUE);
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_CLICKABLE);
    s_time = label(s_body, "--:--", 88, 225, &lv_font_montserrat_20, UI_TEXT);
    lv_obj_set_width(s_time, 64);
    lv_obj_set_style_text_align(s_time, LV_TEXT_ALIGN_CENTER, 0);
    make_entry(s_time, &bluetooth);
    entry = panel(s_body, 14, 226, 64, 22, UI_BG);
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    heart_icon(entry, 0, 0);
    s_watch_heart = label(entry, "--", 26, 3, &lv_font_montserrat_14, UI_TEXT);
    make_entry(entry, &heart);
    entry = panel(s_body, 166, 220, 64, 32, UI_BG);
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    s_watch_temperature = label(entry, "--C\n--%", 0, 0, &lv_font_montserrat_14, UI_TEXT);
    lv_obj_set_style_text_line_space(s_watch_temperature, 0, 0);
    make_entry(entry, &climate);
    lv_obj_set_parent(s_battery_label, s_body);
    lv_obj_set_pos(s_battery_label, 78, 254);
    lv_obj_set_width(s_battery_label, 130);
    lv_obj_set_style_text_font(s_battery_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(s_battery_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_parent(s_battery_bar, s_body);
    lv_obj_set_pos(s_battery_bar, 34, 257);
}

/* 表盘按RTC秒值更新；触摸/动画保持原节拍，不给秒针另建动画。 */
static void refresh_clock(void)
{
    watch_time_struct time;
    app_aht21_struct climate;
    app_heart_struct heart;
    HAL_StatusTypeDef status;
    uint32_t now = lv_tick_get();
    uint32_t seconds;
    int32_t magnitude;
    int32_t humidity_x10;
    const char *sign = "";
    if (s_idle_dimmed != 0)
    {
        return; /* 待机不读取RTC、不更新指针；RTC硬件继续独立走时。 */
    }
    status = watch_rtc_get_time(&time);
    if (status == HAL_OK && watch_rtc_is_synced())
    {
        seconds = (uint32_t)time.hours * 3600U + (uint32_t)time.minutes * 60U + time.seconds;
        if (seconds == s_last_second)
        {
            return; /* 只读RTC秒值判断节拍；同一秒不重取摘要、不重绘指针。 */
        }
    }
    else if (s_last_second == UINT32_MAX && lv_tick_elaps(s_clock_refresh_tick) < 1000U)
    {
        return;
    }
    s_clock_refresh_tick = now;
    refresh_battery();
    if (status != HAL_OK || !watch_rtc_is_synced())
    {
        set_label_text(s_time, "--:--");
        if (status == HAL_OK)
        {
            set_label_text(s_clock_hint, "SET VIA BLE");
        }
        else
        {
            set_label_text(s_clock_hint, "RTC ERROR");
        }
        if (!lv_obj_has_flag(s_hour_hand, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_add_flag(s_hour_hand, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_minute_hand, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_second_hand, LV_OBJ_FLAG_HIDDEN);
        }
        s_last_second = UINT32_MAX;
    }
    else
    {
        set_label_text(s_clock_hint, "RTC / LSE");
        if (lv_obj_has_flag(s_hour_hand, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(s_hour_hand, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_minute_hand, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_second_hand, LV_OBJ_FLAG_HIDDEN);
        }
        seconds = (uint32_t)time.hours * 3600U + (uint32_t)time.minutes * 60U + time.seconds;
        if (seconds != s_last_second)
        {
            s_last_second = seconds;
            set_label_text_fmt(s_time, "%02u:%02u", (unsigned)time.hours, (unsigned)time.minutes);
            set_clock_hand(s_hour_hand, s_hour_points, (time.hours % 12) * 30.0f + time.minutes * 0.5f + time.seconds / 120.0f, 44);
            set_clock_hand(s_minute_hand, s_minute_points, time.minutes * 6.0f + time.seconds * 0.1f, 64);
            set_clock_hand(s_second_hand, s_second_points, time.seconds * 6.0f, 82);
        }
    }
    app_heart_get_snapshot(&heart);
    if (heart.state == APP_HEART_VALID)
    {
        set_label_text_fmt(s_watch_heart, "%u", (unsigned)heart.bpm);
    }
    else
    {
        set_label_text(s_watch_heart, "--");
    }
    app_aht21_get_snapshot(&climate);
    if (climate.valid != 0)
    {
        magnitude = (int32_t)(climate.sample.temperature_c * 10.0f);
        humidity_x10 = (int32_t)(climate.sample.humidity_rh * 10.0f);
        if (magnitude < 0)
        {
            magnitude = -magnitude;
            sign = "-";
        }
        set_label_text_fmt(s_watch_temperature, "%s%ld.%ldC\n%ld.%ld%%", sign, (long)(magnitude / 10), (long)(magnitude % 10),
                           (long)(humidity_x10 / 10), (long)(humidity_x10 % 10));
    }
    else
    {
        set_label_text(s_watch_temperature, "--C\n--%");
    }
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

/* 同单位三轴共用一张卡片，减少合并页面同时驻留的控件。 */
static void motion_metrics(const char *const titles[3], const char *unit, int y, lv_obj_t *values[3])
{
    static const uint32_t colors[3] = {UI_BLUE, UI_ORANGE, UI_GREEN};
    lv_obj_t *obj = panel(s_body, 0, y, 212, 288, UI_CARD);
    uint8_t i;
    label(obj, unit, 173, 12, &lv_font_montserrat_10, UI_MUTED);
    for (i = 0; i < 3; ++i)
    {
        label(obj, titles[i], 14, 12 + i * 100, &lv_font_montserrat_10, UI_MUTED);
        values[i] = label(obj, "--", 14, 35 + i * 100, &lv_font_montserrat_30, colors[i]);
        lv_obj_set_width(values[i], 184);
    }
}

/* 心率每秒、海拔每5秒记录，最右为最新值；缺测留空，不把无效值画成0。 */
static void trend_create(ui_trend_struct *trend, const char *title, int y, int height, uint32_t color, uint32_t sample_ms)
{
    lv_obj_t *card = panel(s_body, 0, y, 212, height, UI_CARD);
    lv_obj_t *caption;
    label(card, title, 12, 8, &lv_font_montserrat_10, color);
    trend->scale = label(card, "--", 0, 23, &lv_font_montserrat_10, UI_MUTED);
    trend->scale_low = label(card, "--", 0, height - 31, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(trend->scale, 36);
    lv_obj_set_width(trend->scale_low, 36);
    lv_obj_set_style_text_align(trend->scale, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_align(trend->scale_low, LV_TEXT_ALIGN_RIGHT, 0);
    caption = label(card, "", 42, height - 17, &lv_font_montserrat_10, UI_MUTED);
    set_label_text_fmt(caption, "%lus / bar", (unsigned long)(sample_ms / 1000U));
    label(card, "NOW", 172, height - 17, &lv_font_montserrat_10, UI_MUTED);
    trend->chart = lv_chart_create(card);
    lv_obj_remove_style_all(trend->chart);
    lv_obj_set_pos(trend->chart, 42, 28);
    lv_obj_set_size(trend->chart, 156, height - 50);
    lv_obj_clear_flag(trend->chart, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_line_color(trend->chart, lv_color_hex(UI_BORDER), LV_PART_MAIN);
    lv_obj_set_style_line_width(trend->chart, 1, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(trend->chart, LV_OPA_COVER, LV_PART_ITEMS);
    lv_obj_set_style_pad_column(trend->chart, 2, LV_PART_ITEMS);
    lv_chart_set_type(trend->chart, LV_CHART_TYPE_BAR);
    lv_chart_set_point_count(trend->chart, UI_TREND_POINTS);
    lv_chart_set_div_line_count(trend->chart, 3, 0);
    lv_chart_set_range(trend->chart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
    trend->series = lv_chart_add_series(trend->chart, lv_color_hex(color), LV_CHART_AXIS_PRIMARY_Y);
    trend->sample_period_ms = sample_ms;
    trend->sample_tick = lv_tick_get() - sample_ms;
}

static void trend_update(ui_trend_struct *trend, lv_coord_t value, uint8_t altitude)
{
    lv_coord_t low = 0, high = 200;
    uint16_t i;
    uint32_t now = lv_tick_get();
    if (trend->chart == NULL || (uint32_t)(now - trend->sample_tick) < trend->sample_period_ms)
    {
        return;
    }
    trend->sample_tick = now;
    lv_chart_set_next_value(trend->chart, trend->series, value);
    if (altitude)
    {
        low = LV_CHART_POINT_NONE;
        high = -30000;
        for (i = 0; i < UI_TREND_POINTS; ++i)
        {
            lv_coord_t point = trend->series->y_points[i];
            if (point != LV_CHART_POINT_NONE)
            {
                low = LV_MIN(low, point);
                high = LV_MAX(high, point);
            }
        }
        if (low == LV_CHART_POINT_NONE)
        {
            set_label_text(trend->scale, "--");
            set_label_text(trend->scale_low, "--");
            return;
        }
        /* 海拔允许负值，纵轴跟随窗口并保留至少20m跨度以免放大静止噪声。 */
        low -= 10;
        high += 10;
    }
    set_label_text_fmt(trend->scale, "%d", (int)high);
    set_label_text_fmt(trend->scale_low, "%d", (int)low);
    lv_chart_set_range(trend->chart, LV_CHART_AXIS_PRIMARY_Y, low, high);
}

static void compass_draw_cb(lv_event_t *event)
{
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    lv_area_t area;
    lv_point_t inner, outer, center, tip, left, right, needle[3];
    lv_draw_line_dsc_t style;
    lv_draw_arc_dsc_t ring;
    lv_draw_rect_dsc_t face;
    int16_t angle, radius;
    int32_t sine, cosine;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    center.x = area.x1 + 106;
    center.y = area.y1 + 64;
    weather_icon_shape(context, &area, 46, 4, 120, 120, UI_BG);
    lv_draw_arc_dsc_init(&ring);
    ring.color = lv_color_hex(UI_BORDER);
    ring.width = 2;
    lv_draw_arc(context, &ring, &center, 60, 0, 360);
    ring.width = 1;
    lv_draw_arc(context, &ring, &center, 47, 0, 360);
    lv_draw_line_dsc_init(&style);
    for (angle = 0; angle < 360; angle += 5)
    {
        radius = 54;
        style.width = 1;
        style.color = lv_color_hex(UI_BORDER);
        if (angle % 30 == 0)
        {
            radius = 50;
            style.width = 2;
            style.color = lv_color_hex(UI_MUTED);
        }
        if (angle == 0)
        {
            style.color = lv_color_hex(UI_RED);
        }
        sine = lv_trigo_sin(angle);
        cosine = lv_trigo_cos(angle);
        inner.x = center.x + sine * radius / LV_TRIGO_SIN_MAX;
        inner.y = center.y - cosine * radius / LV_TRIGO_SIN_MAX;
        outer.x = center.x + sine * 57 / LV_TRIGO_SIN_MAX;
        outer.y = center.y - cosine * 57 / LV_TRIGO_SIN_MAX;
        lv_draw_line(context, &style, &inner, &outer);
    }
    if (s_compass_heading < 0)
    {
        return;
    }
    sine = lv_trigo_sin(s_compass_heading);
    cosine = lv_trigo_cos(s_compass_heading);
    left.x = center.x - cosine * 8 / LV_TRIGO_SIN_MAX;
    left.y = center.y - sine * 8 / LV_TRIGO_SIN_MAX;
    right.x = center.x + cosine * 8 / LV_TRIGO_SIN_MAX;
    right.y = center.y + sine * 8 / LV_TRIGO_SIN_MAX;
    lv_draw_rect_dsc_init(&face);
    needle[2] = center;
    /* 四个三角面组成双色菱形指针，不增加位图或常驻控件。 */
    for (angle = 0; angle < 2; ++angle)
    {
        if (angle == 0)
        {
            tip.x = center.x + sine * 30 / LV_TRIGO_SIN_MAX;
            tip.y = center.y - cosine * 30 / LV_TRIGO_SIN_MAX;
            face.bg_color = lv_color_hex(s_compass_color);
        }
        else
        {
            tip.x = center.x - sine * 24 / LV_TRIGO_SIN_MAX;
            tip.y = center.y + cosine * 24 / LV_TRIGO_SIN_MAX;
            face.bg_color = lv_color_hex(UI_BLUE);
        }
        needle[0] = tip;
        needle[1] = left;
        lv_draw_triangle(context, &face, needle);
        face.bg_color = lv_color_darken(face.bg_color, LV_OPA_30);
        needle[1] = right;
        lv_draw_triangle(context, &face, needle);
    }
    weather_icon_shape(context, &area, 102, 60, 8, 8, UI_TEXT);
    weather_icon_shape(context, &area, 104, 62, 4, 4, UI_BG);
}

static void compass_create(int y)
{
    s_compass = panel(s_body, 0, y, 212, 174, UI_CARD);
    label(s_compass, "N", 102, 22, &lv_font_montserrat_10, UI_RED);
    label(s_compass, "E", 142, 59, &lv_font_montserrat_10, UI_TEXT);
    label(s_compass, "S", 102, 102, &lv_font_montserrat_10, UI_TEXT);
    label(s_compass, "W", 61, 59, &lv_font_montserrat_10, UI_TEXT);
    s_compass_value = label(s_compass, "--", 14, 122, &lv_font_montserrat_30, UI_TEXT);
    lv_obj_set_width(s_compass_value, 184);
    lv_obj_set_style_text_align(s_compass_value, LV_TEXT_ALIGN_CENTER, 0);
    s_compass_status = label(s_compass, "NO IMU DATA", 14, 156, &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_width(s_compass_status, 184);
    lv_obj_set_style_text_align(s_compass_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_event_cb(s_compass, compass_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
}

static void compass_update(const app_imu_struct *imu, uint8_t online)
{
    static const char *const directions[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int16_t heading = -1;
    uint32_t color = UI_MUTED;
    uint32_t status_color = UI_MUTED;
    uint32_t now = lv_tick_get();
    uint32_t elapsed = LV_MIN((uint32_t)(now - s_compass_tick), UI_REFRESH_MS);
    float degrees, delta, step, limit;
    if (online && isfinite(imu->yaw_deg))
    {
        /* 世界+Z向上：右手yaw取反得到顺时针磁航向，参考方向为MPU的+X轴。 */
        degrees = fmodf(-imu->yaw_deg, 360.0f);
        if (degrees < 0.0f)
        {
            degrees += 360.0f;
        }
        if (s_compass_heading < 0)
        {
            s_compass_display_deg = degrees;
        }
        else
        {
            /* 跨北点走最短角差；100ms低通并限速360°/s，停屏后也不积攒一次大跳。 */
            delta = fmodf(degrees - s_compass_display_deg + 540.0f, 360.0f) - 180.0f;
            step = delta * (float)elapsed / (UI_COMPASS_FILTER_MS + (float)elapsed);
            limit = UI_COMPASS_MAX_DPS * (float)elapsed / 1000.0f;
            step = LV_CLAMP(-limit, step, limit);
            s_compass_display_deg = fmodf(s_compass_display_deg + step + 360.0f, 360.0f);
        }
        heading = (int16_t)(s_compass_display_deg + 0.5f) % 360;
        set_label_text_fmt(s_compass_value, "%03d\xC2\xB0 %s", (int)heading, directions[((heading + 22) / 45) % 8]);
        if (imu->mag_fused && imu->mag_fusion_state == APP_MAG_FUSION_ACTIVE)
        {
            if (imu->mag_calibrated)
            {
                color = UI_RED;
                status_color = UI_GREEN;
                set_label_text(s_compass_status, "MAG NORTH");
            }
            else
            {
                color = UI_ORANGE;
                status_color = UI_ORANGE;
                set_label_text(s_compass_status, "UNCALIBRATED");
            }
        }
        else
        {
            if (imu->mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                status_color = UI_ORANGE;
                set_label_text(s_compass_status, "CALIBRATING");
            }
            else
            {
                set_label_text(s_compass_status, "GYRO ONLY");
            }
        }
    }
    else
    {
        set_label_text(s_compass_value, "--");
        set_label_text(s_compass_status, "NO IMU DATA");
        status_color = UI_RED;
    }
    set_text_color(s_compass_status, status_color);
    s_compass_tick = now;
    if (heading != s_compass_heading || color != s_compass_color)
    {
        s_compass_heading = heading;
        s_compass_color = color;
        lv_obj_invalidate(s_compass);
    }
}

static void mag_calibration_open_cb(lv_event_t *event)
{
    (void)event;
    if (!s_transition)
    {
        lv_obj_scroll_to_view(lv_obj_get_parent(s_mag_cal_label), LV_ANIM_OFF);
    }
}

static void imu_page(int y)
{
    static const char *const angles[3] = {"PITCH", "ROLL", "YAW"};
    static const char *const magnetic[3] = {"MAG X", "MAG Y", "MAG Z"};
    lv_obj_t *obj;
    compass_create(y);
    obj = panel(s_body, 0, y + 182, 212, 36, UI_BORDER);
    s_compass_calibration = label(obj, "CALIBRATION  " LV_SYMBOL_RIGHT, 14, 11, &lv_font_montserrat_14, UI_TEXT);
    lv_obj_clear_flag(s_compass_calibration, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, mag_calibration_open_cb, LV_EVENT_CLICKED, NULL);
    y += 228;
    s_imu_status = label(s_body, "WAITING", 2, y + 4, &lv_font_montserrat_10, UI_MUTED);
    motion_metrics(angles, "DEG", y + 26, s_angles);
    s_yaw_source = note(s_body, "GYRO ONLY / heading may drift", y + 326);
    s_mag_status = label(s_body, "MAG WAITING", 2, y + 360, &lv_font_montserrat_10, UI_MUTED);
    motion_metrics(magnetic, "uT", y + 384, s_mag_values);
    obj = panel(s_body, 0, y + 684, 212, 48, UI_BORDER);
    s_mag_cal_label = label(obj, "CALIBRATE MAG", 14, 16, &lv_font_montserrat_14, UI_TEXT);
    lv_obj_clear_flag(s_mag_cal_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, mag_calibrate_cb, LV_EVENT_CLICKED, NULL);
    s_mag_cal_status = note(s_body, "CAL NOT RUN", y + 744);
    note(s_body, "Rotate every axis for 30-60s, then tap FINISH. EEPROM keeps successful calibration.", y + 790);
}

static void baro_page(int y)
{
    lv_obj_t *card = panel(s_body, 0, y, 212, 94, UI_CARD);
    label(card, "ALTITUDE", 12, 10, &lv_font_montserrat_10, UI_BLUE);
    s_baro_status = label(card, "WAITING", 140, 10, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_baro_status, 60);
    lv_obj_set_style_text_align(s_baro_status, LV_TEXT_ALIGN_RIGHT, 0);
    s_altitude = label(card, "--", 12, 27, &lv_font_montserrat_30, UI_TEXT);
    lv_obj_set_width(s_altitude, 168);
    label(card, "m", 184, 40, &lv_font_montserrat_14, UI_MUTED);
    panel(card, 12, 64, 188, 1, UI_BORDER);
    label(card, "PRESSURE", 12, 75, &lv_font_montserrat_10, UI_MUTED);
    s_pressure = label(card, "--", 92, 72, &lv_font_montserrat_14, UI_TEXT);
    label(card, "hPa", 182, 75, &lv_font_montserrat_10, UI_MUTED);
    trend_create(&s_altitude_trend, "ALTITUDE TREND / m", y + 102, 116, UI_BLUE, UI_BARO_REFRESH_MS);
    note(s_body, "Altitude uses standard sea-level pressure. Weather can change this estimate.", y + 232);
    note(s_body, "SPL006 / live sensor snapshot", y + 286);
}

static void brightness_cb(lv_event_t *event)
{
    s_brightness = (uint8_t)lv_slider_get_value(lv_event_get_target(event));
    if (s_screen_on && s_idle_dimmed == 0)
    {
        LCD_Set_Light(s_brightness);
    }
    else if (s_screen_on)
    {
        LCD_Set_Light(UI_MIN_BRIGHTNESS);
    }
    else
    {
        LCD_Set_Light(0);
    }
    set_label_text_fmt(s_brightness_label, "%d%%", (int)s_brightness);
}

/* 丢字节位置在环形队列中不可定位，废弃队列和残帧后重新寻找消息边界。 */
static uint8_t bluetooth_rx_check(uint32_t now_ms)
{
    uint32_t dropped = kt6368a_get_rx_dropped();
    uint32_t errors = kt6368a_get_rx_errors();
    if (dropped == s_bt_rx_dropped && errors == s_bt_rx_errors)
    {
        return 0;
    }
    s_bt_rx_dropped = dropped;
    s_bt_rx_errors = errors;
    kt6368a_flush_rx();
    app_clock_reset_receive();
    app_weather_reset_receive();
    app_diag_reset_receive();
    s_bt_rx_resync = 1;
    s_bt_rx_activity_ms = now_ms;
    return 1;
}

/* 按字节接收，保留最近511个显示字符；非ASCII字节显示为\xHH。 */
static void bluetooth_receive(void)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t data[UI_BT_READ_SIZE];
    char text[UI_BT_READ_SIZE * 4];
    uint16_t received, length, remove, i, start, budget = UI_BT_RX_BUDGET;
    uint32_t now_ms;

    /* AT响应留给初始化；RTC未就绪时先保留手机校时等业务字节。 */
    if (kt6368a_init_is_pending() != 0 || watch_rtc_init_is_pending() != 0)
    {
        return;
    }
    if (bluetooth_rx_check(lv_tick_get()) != 0)
    {
        return;
    }

    while (budget != 0)
    {
        if (budget < sizeof(data))
        {
            received = kt6368a_read(data, budget);
        }
        else
        {
            received = kt6368a_read(data, sizeof(data));
        }
        if (received == 0)
        {
            break;
        }
        budget -= received;
        now_ms = lv_tick_get();
        if (bluetooth_rx_check(now_ms) != 0)
        {
            return; /* 读取期间发生的溢出/串口错误也不能交给解析器。 */
        }
        if (s_bt_rx_resync != 0 && (uint32_t)(now_ms - s_bt_rx_activity_ms) >= APP_CLOCK_FRAME_IDLE_MS)
        {
            s_bt_rx_resync = 0;
        }
        start = 0;
        while (s_bt_rx_resync != 0 && start < received)
        {
            if (data[start] == '\r' || data[start] == '\n')
            {
                s_bt_rx_resync = 0;
            }
            ++start;
        }
        s_bt_rx_activity_ms = now_ms;
        if (start < received)
        {
            app_diag_receive(data + start, received - start);
            app_clock_receive(data + start, received - start, now_ms);
            app_weather_receive(data + start, received - start, now_ms);
        }
        s_bt_rx_bytes += received;
        length = 0;
        for (i = 0; i < received; ++i)
        {
            if (data[i] == '\r')
            {
                text[length++] = '\n';
                s_bt_last_cr = 1;
            }
            else if (data[i] == '\n')
            {
                if (!s_bt_last_cr)
                {
                    text[length++] = '\n';
                }
                s_bt_last_cr = 0;
            }
            else
            {
                s_bt_last_cr = 0;
                if (data[i] == '\t')
                {
                    text[length++] = ' ';
                }
                else if (data[i] >= 32 && data[i] <= 126)
                {
                    text[length++] = (char)data[i];
                }
                else
                {
                    text[length++] = '\\';
                    text[length++] = 'x';
                    text[length++] = hex[data[i] >> 4];
                    text[length++] = hex[data[i] & 0x0F];
                }
            }
        }
        if (s_bt_text_length + length >= sizeof(s_bt_text))
        {
            remove = s_bt_text_length + length - (sizeof(s_bt_text) - 1);
            memmove(s_bt_text, s_bt_text + remove, s_bt_text_length - remove);
            s_bt_text_length -= remove;
        }
        memcpy(s_bt_text + s_bt_text_length, text, length);
        s_bt_text_length += length;
        s_bt_text[s_bt_text_length] = '\0';
        s_bt_dirty = 1;
    }
    if (bluetooth_rx_check(lv_tick_get()) == 0)
    {
        app_clock_service(lv_tick_get());
        app_weather_service(lv_tick_get());
    }
}

static void bluetooth_clear_cb(lv_event_t *event)
{
    (void)event;
    if (s_transition)
    {
        return;
    }
    /* CLEAR只清显示历史，保留队列与解析残帧，让业务消息完整接收。 */
    s_bt_text[0] = '\0';
    s_bt_text_length = 0;
    s_bt_rx_bytes = 0;
    s_bt_last_cr = 0;
    s_bt_dirty = 1;
    refresh_bluetooth();
}

static void bluetooth_send_test_cb(lv_event_t *event)
{
    static const uint8_t message[] = "hello from watch\r\n";
    HAL_StatusTypeDef status;
    (void)event;
    if (APP_DEBUG_TELEMETRY == 0 || s_transition)
    {
        return;
    }
    /* 本板未接蓝牙状态脚，直接发送并显示 UART 返回结果。 */
    status = kt6368a_write(message, sizeof(message) - 1, UI_BT_TX_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        s_bt_tx_color = UI_GREEN;
    }
    else
    {
        s_bt_tx_color = UI_ORANGE;
    }
    switch (status)
    {
    case HAL_OK:
        s_bt_tx_text = "TX UART OK";
        break;
    case HAL_BUSY:
        s_bt_tx_text = "TX BUSY";
        break;
    case HAL_TIMEOUT:
        s_bt_tx_text = "TX TIMEOUT";
        break;
    default:
        s_bt_tx_text = "TX ERROR";
        s_bt_tx_color = UI_RED;
        break;
    }
    refresh_bluetooth();
}

/* 心率调试波形使用原LVGL任务异步发送；性能版不取走队列、不打包或发送。 */
static void bluetooth_send_heart(void)
{
#if APP_DEBUG_TELEMETRY
    app_heart_sample_struct samples[2];
    static uint8_t frames[72];
    static uint16_t pending_count;
    float channels[8];
    uint16_t count, i, channel, offset;
    uint32_t bits;
    HAL_StatusTypeDef status;
    if (pending_count != 0)
    {
        if (kt6368a_take_tx_result(&status) == 0)
        {
            return;
        }
        app_diag_heart_tx(pending_count, status == HAL_OK);
        pending_count = 0;
        if (status == HAL_OK)
        {
            s_bt_tx_text = "TX HEART OK";
            s_bt_tx_color = UI_GREEN;
        }
        else
        {
            s_bt_tx_text = "TX HEART ERROR";
            s_bt_tx_color = UI_RED;
        }
    }
    if (kt6368a_init_is_pending() != 0 || kt6368a_tx_is_busy() != 0)
    {
        return;
    }
    count = app_heart_read_samples(samples, sizeof(samples) / sizeof(samples[0]));
    if (count == 0)
    {
        return;
    }
    for (i = 0; i < count; ++i)
    {
        channels[0] = (float)samples[i].raw;
        channels[1] = samples[i].filtered;
        channels[2] = samples[i].dc;
        channels[3] = (float)samples[i].bpm;
        channels[4] = (float)samples[i].interval_ms;
        channels[5] = (float)samples[i].elapsed_ms / 1000.0f;
        channels[6] = (float)samples[i].sequence;
        channels[7] = (float)samples[i].state;
        for (channel = 0; channel < 8; ++channel)
        {
            offset = i * 36 + channel * 4;
            memcpy(&bits, &channels[channel], sizeof(bits));
            frames[offset] = (uint8_t)bits;
            frames[offset + 1] = (uint8_t)(bits >> 8);
            frames[offset + 2] = (uint8_t)(bits >> 16);
            frames[offset + 3] = (uint8_t)(bits >> 24);
        }
        frames[i * 36 + 32] = 0;
        frames[i * 36 + 33] = 0;
        frames[i * 36 + 34] = 0x80;
        frames[i * 36 + 35] = 0x7F;
    }
    status = kt6368a_write_async(frames, count * 36U, UI_BT_TX_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        pending_count = count;
    }
    else
    {
        app_diag_heart_tx(count, 0);
        s_bt_tx_text = "TX HEART ERROR";
        s_bt_tx_color = UI_RED;
    }
#endif
}

static void bluetooth_page(void)
{
    lv_obj_t *button, *caption;
    s_bt_status = label(s_body, "INIT PENDING", 4, 4, &lv_font_montserrat_10, UI_MUTED);
    note(s_body, KT6368A_BLE_NAME, 22);
    s_bt_count = label(s_body, "RX 0 B", 4, 44, &lv_font_montserrat_10, UI_BLUE);
    s_bt_tx_status = label(s_body, s_bt_tx_text, 84, 44, &lv_font_montserrat_10, s_bt_tx_color);
    button = panel(s_body, 0, 62, 124, 28, UI_BLUE);
    caption = label(button, "SEND TEST", 0, 0, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_center(caption);
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button, bluetooth_send_test_cb, LV_EVENT_CLICKED, NULL);
    if (APP_DEBUG_TELEMETRY == 0)
    {
        lv_obj_add_state(button, LV_STATE_DISABLED);
    }
    button = panel(s_body, 136, 62, 76, 28, UI_BORDER);
    caption = label(button, "CLEAR", 0, 0, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_center(caption);
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button, bluetooth_clear_cb, LV_EVENT_CLICKED, NULL);
    s_bt_log_box = panel(s_body, 0, 98, 212, 94, UI_CARD);
    lv_obj_add_flag(s_bt_log_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_bt_log_box, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_bt_log_box, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_bt_log_box, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_bottom(s_bt_log_box, 10, 0);
    lv_obj_set_style_bg_color(s_bt_log_box, lv_color_hex(UI_BLUE), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(s_bt_log_box, LV_OPA_60, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(s_bt_log_box, 2, LV_PART_SCROLLBAR);
    s_bt_log = label(s_bt_log_box, "Waiting for data...", 10, 10, &lv_font_montserrat_14, UI_TEXT);
    lv_obj_set_width(s_bt_log, 188);
    lv_label_set_long_mode(s_bt_log, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(s_bt_log, 3, 0);
    s_bt_clock_status = label(s_body, "Send HH:MM:SS to set time", 4, 200, &lv_font_montserrat_10, UI_MUTED);
    s_bt_dirty = 1;
}

static void refresh_bluetooth(void)
{
    HAL_StatusTypeDef status;
    const char *text = "INIT OK / BLE ONLY";
    uint32_t color = UI_MUTED;
    if (s_bt_status == NULL)
    {
        return;
    }
    status = kt6368a_get_init_status();
    /* 初始化结果不等于手机连接状态，KT_LED 没有接入 STM32。 */
    if (status != HAL_OK)
    {
        if (status == HAL_TIMEOUT)
        {
            text = "INIT TIMEOUT";
        }
        else if (status == HAL_BUSY)
        {
            text = "INIT BUSY";
        }
        else
        {
            text = "INIT ERROR";
        }
        color = UI_ORANGE;
    }
    set_label_text(s_bt_status, text);
    set_text_color(s_bt_status, color);
    set_label_text_fmt(s_bt_count, "RX %lu B", (unsigned long)s_bt_rx_bytes);
    set_label_text(s_bt_tx_status, s_bt_tx_text);
    set_text_color(s_bt_tx_status, s_bt_tx_color);
    switch (app_clock_get_result())
    {
    case APP_CLOCK_SYNCED:
        set_label_text(s_bt_clock_status, "TIME SYNCED");
        set_text_color(s_bt_clock_status, UI_GREEN);
        break;
    case APP_CLOCK_INVALID:
        set_label_text(s_bt_clock_status, "TIME INVALID");
        set_text_color(s_bt_clock_status, UI_RED);
        break;
    case APP_CLOCK_RTC_ERROR:
        set_label_text(s_bt_clock_status, "RTC ERROR");
        set_text_color(s_bt_clock_status, UI_RED);
        break;
    default:
        set_label_text(s_bt_clock_status, "Send HH:MM:SS to set time");
        break;
    }
    if (s_bt_dirty)
    {
        if (s_bt_text_length)
        {
            set_label_text(s_bt_log, s_bt_text);
        }
        else
        {
            set_label_text(s_bt_log, "Waiting for data...");
        }
        lv_obj_update_layout(s_bt_log_box);
        lv_obj_scroll_to_y(s_bt_log_box, lv_obj_get_scroll_y(s_bt_log_box) + lv_obj_get_scroll_bottom(s_bt_log_box), LV_ANIM_OFF);
        s_bt_dirty = 0;
    }
}

static void heart_toggle_cb(lv_event_t *event)
{
    app_heart_struct heart;
    (void)event;
    if (s_transition)
    {
        return;
    }
    app_heart_get_snapshot(&heart);
    app_heart_set_enabled(!heart.enabled);
    refresh_data();
}

static void heart_details_cb(lv_event_t *event)
{
    (void)event;
    if (s_transition)
    {
        return;
    }
    if (lv_obj_has_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN);
        set_label_text(s_heart_details_title, "DETAILS  " LV_SYMBOL_UP);
        lv_obj_scroll_to_view(s_heart_details, LV_ANIM_OFF);
    }
    else
    {
        lv_obj_add_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN);
        set_label_text(s_heart_details_title, "DETAILS  " LV_SYMBOL_RIGHT);
        lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    }
}

static void heart_window(void)
{
    lv_obj_t *toggle;
    s_heart_card = panel(s_body, 0, 0, 212, 76, UI_CARD);
    heart_icon(s_heart_card, 14, 6);
    label(s_heart_card, "HEART RATE", 40, 10, &lv_font_montserrat_10, UI_RED);
    s_heart_value = label(s_heart_card, "--", 14, 25, &lv_font_montserrat_30, UI_RED);
    label(s_heart_card, "BPM", 78, 40, &lv_font_montserrat_10, UI_MUTED);
    s_heart_summary = label(s_heart_card, "STANDBY", 14, 60, &lv_font_montserrat_10, UI_MUTED);
    s_heart_button = label(s_heart_card, "START", 118, 27, &lv_font_montserrat_14, UI_BG);
    lv_obj_set_size(s_heart_button, 82, 36);
    lv_obj_set_style_bg_color(s_heart_button, lv_color_hex(UI_BLUE), 0);
    lv_obj_set_style_bg_opa(s_heart_button, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_heart_button, 12, 0);
    lv_obj_set_style_pad_top(s_heart_button, 10, 0);
    lv_obj_set_style_text_align(s_heart_button, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(s_heart_button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_heart_button, heart_toggle_cb, LV_EVENT_CLICKED, NULL);
    trend_create(&s_heart_trend, "HEART RATE TREND / BPM", 84, 98, UI_RED, UI_TREND_SAMPLE_MS);
    toggle = panel(s_body, 0, 190, 212, 28, UI_BORDER);
    s_heart_details_title = label(toggle, "DETAILS  " LV_SYMBOL_RIGHT, 14, 8, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_clear_flag(s_heart_details_title, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(toggle, heart_details_cb, LV_EVENT_CLICKED, NULL);
    s_heart_details = panel(s_body, 0, 226, 212, 76, UI_CARD);
    s_heart_status = label(s_heart_details, "STANDBY", 14, 12, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_heart_status, 184);
    lv_obj_add_flag(s_heart_details, LV_OBJ_FLAG_HIDDEN);
}

static void refresh_heart(void)
{
    app_heart_struct heart;
    const char *status = "STANDBY";
    if (s_heart_card == NULL)
    {
        return;
    }
    app_heart_get_snapshot(&heart);
    if (heart.enabled && heart.state == APP_HEART_VALID && heart.bpm >= 40U && heart.bpm <= 200U)
    {
        trend_update(&s_heart_trend, (lv_coord_t)heart.bpm, 0);
    }
    else
    {
        trend_update(&s_heart_trend, LV_CHART_POINT_NONE, 0);
    }
    if (heart.state == APP_HEART_VALID)
    {
        set_label_text_fmt(s_heart_value, "%u", (unsigned)heart.bpm);
        status = "TRACKING";
    }
    else
    {
        set_label_text(s_heart_value, "--");
        switch (heart.state)
        {
        case APP_HEART_MEASURING:
            status = "MEASURING";
            break;
        case APP_HEART_NO_SIGNAL:
            status = "NO SIGNAL";
            break;
        case APP_HEART_ERROR:
            status = "UNAVAILABLE";
            break;
        default:
            break;
        }
    }
    set_label_text(s_heart_summary, status);
    if (heart.state == APP_HEART_VALID)
    {
        set_text_color(s_heart_summary, UI_GREEN);
    }
    else if (heart.state == APP_HEART_ERROR)
    {
        set_text_color(s_heart_summary, UI_RED);
    }
    else if (heart.state == APP_HEART_MEASURING || heart.state == APP_HEART_NO_SIGNAL)
    {
        set_text_color(s_heart_summary, UI_ORANGE);
    }
    else
    {
        set_text_color(s_heart_summary, UI_MUTED);
    }
    /* 故障时保留实际失败步骤；读取失败不能只显示算法的 NO SIGNAL。 */
    if (heart.state == APP_HEART_ERROR)
    {
        const char *reason;
        switch (heart.fault.kind)
        {
        case EM7028_FAULT_READ:
            reason = "I2C READ";
            break;
        case EM7028_FAULT_WRITE:
            reason = "I2C WRITE";
            break;
        case EM7028_FAULT_PID:
            reason = "PID CHECK";
            break;
        case EM7028_FAULT_CONFIG:
            reason = "CONFIG CHECK";
            break;
        case EM7028_FAULT_DATA_CHANGED:
            reason = "DATA CHANGED";
            break;
        default:
            reason = "SAMPLE WAIT";
            break;
        }
        if (heart.fault.kind == EM7028_FAULT_PID || heart.fault.kind == EM7028_FAULT_CONFIG)
        {
            set_label_text_fmt(s_heart_status, "%s\n%s REG %02X\nVALUE %02X", status, reason, (unsigned)heart.fault.reg,
                                  (unsigned)heart.fault.value);
        }
        else
        {
            set_label_text_fmt(s_heart_status, "%s\n%s REG %02X\nRetry START", status, reason, (unsigned)heart.fault.reg);
        }
    }
    else
    {
        set_label_text_fmt(s_heart_status, "%s\nHRS1 %u  AC %lu.%lu\nPERIOD %u ms", status, (unsigned)heart.raw,
                              (unsigned long)(heart.signal_x10 / 10U), (unsigned long)(heart.signal_x10 % 10U),
                              (unsigned)heart.interval_ms);
    }
    if (heart.state == APP_HEART_ERROR)
    {
        set_text_color(s_heart_status, UI_RED);
    }
    else
    {
        set_text_color(s_heart_status, UI_MUTED);
    }
    if (heart.enabled)
    {
        set_label_text(s_heart_button, "STOP");
    }
    else
    {
        set_label_text(s_heart_button, "START");
    }
}

static void display_page(void)
{
    lv_obj_t *obj = panel(s_body, 0, 4, 212, 128, UI_CARD);
    lv_obj_t *slider;
    label(obj, "DISPLAY", 14, 14, &lv_font_montserrat_10, UI_BLUE);
    label(obj, "Brightness", 14, 38, &lv_font_montserrat_14, UI_TEXT);
    s_brightness_label = label(obj, "", 151, 38, &lv_font_montserrat_14, UI_ORANGE);
    set_label_text_fmt(s_brightness_label, "%d%%", (int)s_brightness);
    slider = lv_slider_create(obj);
    lv_obj_remove_style_all(slider);
    lv_obj_set_pos(slider, 24, 82);
    lv_obj_set_size(slider, 164, 10);
    lv_slider_set_range(slider, UI_MIN_BRIGHTNESS, 100);
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
    note(s_body, "Brightness resets after restart.", 150);
}

static void climate_page(void)
{
    s_aht21_card = panel(s_body, 0, 4, 212, 212, UI_CARD);
    label(s_aht21_card, "AHT21 / TEMP + HUMIDITY", 14, 12, &lv_font_montserrat_10, UI_BLUE);
    s_aht21_status = label(s_aht21_card, "WAITING", 14, 30, &lv_font_montserrat_10, UI_MUTED);
    label(s_aht21_card, "HUMIDITY", 14, 54, &lv_font_montserrat_10, UI_MUTED);
    label(s_aht21_card, "%RH", 169, 80, &lv_font_montserrat_10, UI_GREEN);
    s_humidity = label(s_aht21_card, "--", 14, 70, &lv_font_montserrat_30, UI_GREEN);
    lv_obj_set_width(s_humidity, 146);
    label(s_aht21_card, "AIR TEMPERATURE", 14, 116, &lv_font_montserrat_10, UI_MUTED);
    label(s_aht21_card, "C", 184, 142, &lv_font_montserrat_10, UI_ORANGE);
    s_air_temperature = label(s_aht21_card, "--", 14, 132, &lv_font_montserrat_30, UI_ORANGE);
    lv_obj_set_width(s_air_temperature, 146);
    s_aht21_samples = label(s_aht21_card, "SAMPLES 0 / 2s", 14, 186, &lv_font_montserrat_10, UI_MUTED);
}

static void settings_page(void)
{
    static const ui_page_t destinations[] = {PAGE_HEART, PAGE_MOTION, PAGE_ENVIRONMENT, PAGE_AHT21,
                                              PAGE_WEATHER, PAGE_BLUETOOTH, PAGE_DISPLAY};
    static const char *const titles[] = {"Heart rate", "Motion", "Barometer", "Climate", "Weather", "Bluetooth", "Display"};
    static const char *const icons[] = {LV_SYMBOL_TINT, LV_SYMBOL_REFRESH, LV_SYMBOL_UP, LV_SYMBOL_TINT,
                                        LV_SYMBOL_IMAGE, LV_SYMBOL_BLUETOOTH, LV_SYMBOL_SETTINGS};
    uint8_t i;
    for (i = 0; i < sizeof(destinations) / sizeof(destinations[0]); ++i)
    {
        card(icons[i], titles[i], i, UI_BLUE, &destinations[i]);
    }
}

static void back_cb(lv_event_t *event)
{
    (void)event;
    if (!s_transition)
    {
        App_UI_Back();
    }
}

static void clear_page_refs(void)
{
    s_battery_label = NULL;
    s_battery_bar = NULL;
    s_weather_label = s_weather_value = s_weather_condition = NULL;
    s_weather_icon = NULL;
    s_weather_icon_code = 255;
    s_time = s_clock_hint = s_watch_heart = s_watch_temperature = NULL;
    s_hour_hand = s_minute_hand = s_second_hand = NULL;
    s_clock_refresh_tick = lv_tick_get() - 1000U;
    s_imu_status = s_baro_status = NULL;
    s_pressure = s_altitude = s_brightness_label = NULL;
    s_heart_card = s_heart_value = s_heart_status = s_heart_button = NULL;
    s_heart_summary = s_heart_details = s_heart_details_title = NULL;
    memset(&s_altitude_trend, 0, sizeof(s_altitude_trend));
    memset(&s_heart_trend, 0, sizeof(s_heart_trend));
    s_compass = s_compass_value = s_compass_status = NULL;
    s_compass_calibration = NULL;
    s_compass_heading = -1;
    s_compass_tick = lv_tick_get();
    s_aht21_card = s_humidity = s_air_temperature = s_aht21_status = s_aht21_samples = NULL;
    memset(s_angles, 0, sizeof(s_angles));
    memset(s_mag_values, 0, sizeof(s_mag_values));
    s_mag_status = s_yaw_source = s_mag_cal_label = NULL;
    s_mag_cal_status = NULL;
    s_bt_status = s_bt_count = s_bt_tx_status = s_bt_log = s_bt_log_box = NULL;
    s_bt_clock_status = NULL;
    s_last_second = UINT32_MAX;
}

static void screen_loaded_cb(lv_event_t *event)
{
    if (lv_event_get_target(event) == s_screen)
    {
        s_transition = 0;
    }
}

static void main_page_save(ui_main_page_struct *page)
{
    page->screen = s_screen;
    page->body = s_body;
    page->battery = s_battery_label;
    page->battery_bar = s_battery_bar;
    page->time = s_time;
    page->hint = s_clock_hint;
    page->heart = s_watch_heart;
    page->temperature = s_watch_temperature;
    page->hour_hand = s_hour_hand;
    page->minute_hand = s_minute_hand;
    page->second_hand = s_second_hand;
}

static void main_page_restore(const ui_main_page_struct *page)
{
    s_screen = page->screen;
    s_body = page->body;
    s_battery_label = page->battery;
    s_battery_bar = page->battery_bar;
    s_time = page->time;
    s_clock_hint = page->hint;
    s_watch_heart = page->heart;
    s_watch_temperature = page->temperature;
    s_hour_hand = page->hour_hand;
    s_minute_hand = page->minute_hand;
    s_second_hand = page->second_hand;
}

static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back)
{
    static const char *titles[] = {"WATCH", "APPS", "BLUETOOTH", "DISPLAY", "HEART RATE", "COMPASS",
                                   "BAROMETER", "CLIMATE", "WEATHER"};
    lv_obj_t *dot, *navigation;
    lv_obj_t *old_screen = s_screen;
    ui_page_t old_page = s_page;
    lv_scr_load_anim_t animation_type = LV_SCR_LOAD_ANIM_OVER_LEFT;
    lv_anim_t *animation;
    uint8_t i, tab;
    lv_coord_t dot_width;
    uint32_t dot_color;
    uint8_t auto_delete = 1;
    int width = lv_disp_get_hor_res(NULL);
    int height = lv_disp_get_ver_res(NULL);
    tab = PAGE_SETTINGS;
    if (page == PAGE_WATCH)
    {
        tab = PAGE_WATCH;
    }
    if (page > PAGE_SETTINGS)
    {
        for (i = 0; i < 2; ++i)
        {
            if (s_main_pages[i].screen != NULL && s_main_pages[i].screen != old_screen)
            {
                lv_obj_del(s_main_pages[i].screen);
            }
            memset(&s_main_pages[i], 0, sizeof(s_main_pages[i]));
        }
    }
    else if (old_page <= PAGE_SETTINGS)
    {
        auto_delete = 0;
    }
    clear_page_refs();
    s_page = page;
    if (page <= PAGE_SETTINGS && s_main_pages[page].screen != NULL)
    {
        main_page_restore(&s_main_pages[page]);
    }
    else
    {
        s_screen = lv_obj_create(NULL);
        lv_obj_remove_style_all(s_screen);
        lv_obj_set_style_bg_color(s_screen, lv_color_hex(UI_BG), 0);
        lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
        lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_add_event_cb(s_screen, gesture_cb, LV_EVENT_GESTURE, NULL);
        lv_obj_add_event_cb(s_screen, screen_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);
        if (page != PAGE_WATCH)
        {
            navigation = label(s_screen, titles[page], 14, 13, &lv_font_montserrat_14, UI_TEXT);
            if (page > PAGE_SETTINGS)
            {
                lv_obj_add_flag(navigation, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_ext_click_area(navigation, 10);
                lv_obj_add_event_cb(navigation, back_cb, LV_EVENT_CLICKED, NULL);
            }
        }
        create_battery_bar(s_screen, width - 106, 15);
        s_battery_label = label(s_screen, "--%", width - 66, 15, &lv_font_montserrat_10, UI_MUTED);
        lv_obj_set_width(s_battery_label, 52);
        lv_obj_set_style_text_align(s_battery_label, LV_TEXT_ALIGN_RIGHT, 0);
        s_body = panel(s_screen, (width - 212) / 2, 38, 212, height - 62, UI_BG);
        lv_obj_set_style_radius(s_body, 0, 0);
        if (page == PAGE_WATCH)
        {
            /* 使用整块矩形屏幕，摘要不再受圆形边缘裁切。 */
            lv_obj_set_pos(s_body, 0, 0);
            lv_obj_set_size(s_body, width, height);
        }
        if (page != PAGE_WATCH)
        {
            lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
        }
        lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
        if (page == PAGE_WATCH)
        {
            lv_obj_set_scroll_dir(s_body, LV_DIR_NONE);
            lv_obj_set_scrollbar_mode(s_body, LV_SCROLLBAR_MODE_OFF);
        }
        else
        {
            lv_obj_set_scroll_dir(s_body, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(s_body, LV_SCROLLBAR_MODE_AUTO);
        }
        lv_obj_set_style_bg_color(s_body, lv_color_hex(UI_BLUE), LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_opa(s_body, LV_OPA_60, LV_PART_SCROLLBAR);
        lv_obj_set_style_width(s_body, 2, LV_PART_SCROLLBAR);
        lv_obj_set_style_pad_right(s_body, 0, LV_PART_SCROLLBAR);
        switch (page)
        {
        case PAGE_WATCH:
            watch_page();
            break;
        case PAGE_SETTINGS:
            settings_page();
            break;
        case PAGE_BLUETOOTH:
            bluetooth_page();
            break;
        case PAGE_DISPLAY:
            display_page();
            break;
        case PAGE_HEART:
            heart_window();
            break;
        case PAGE_MOTION:
            imu_page(0);
            break;
        case PAGE_ENVIRONMENT:
            baro_page(0);
            break;
        case PAGE_AHT21:
            climate_page();
            break;
        case PAGE_WEATHER:
            s_weather_value = metric("TODAY'S WEATHER", "BLE", 4, UI_BLUE);
            lv_obj_set_height(lv_obj_get_parent(s_weather_value), 132);
            lv_obj_set_pos(s_weather_value, 88, 48);
            lv_obj_set_width(s_weather_value, 110);
            lv_obj_set_style_text_font(s_weather_value, &lv_font_montserrat_20, 0);
            s_weather_icon = panel(lv_obj_get_parent(s_weather_value), 14, 27, 64, 64, UI_CARD);
            lv_obj_set_style_bg_opa(s_weather_icon, LV_OPA_TRANSP, 0);
            lv_obj_clear_flag(s_weather_icon, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(s_weather_icon, weather_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
            s_weather_condition = label(lv_obj_get_parent(s_weather_value), "--", 14, 100, &lv_font_montserrat_20, UI_TEXT);
            lv_obj_set_width(s_weather_condition, 184);
            s_weather_label = note(s_body, "WAITING FOR PHONE", 148);
            note(s_body, "Weather comes from your phone. Connect the app to sync.", 190);
            break;
        default:
            break;
        }
        if (page != PAGE_WATCH)
        {
            for (i = 0; i < 2; ++i)
            {
                dot_width = 5;
                dot_color = UI_BORDER;
                if (i == tab)
                {
                    dot_width = 12;
                    dot_color = UI_BLUE;
                }
                dot = panel(s_screen, width / 2 - 12 + i * 16, height - 14, dot_width, 5, dot_color);
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
            }
            navigation = label(s_screen, LV_SYMBOL_LEFT " BACK", 14, height - 17, &lv_font_montserrat_10, UI_BLUE);
            lv_obj_add_flag(navigation, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_ext_click_area(navigation, 8);
            lv_obj_add_event_cb(navigation, back_cb, LV_EVENT_CLICKED, NULL);
        }
        if (page <= PAGE_SETTINGS)
        {
            main_page_save(&s_main_pages[page]);
        }
    }
    refresh_data();
    lv_obj_update_layout(s_screen);
    lv_obj_scroll_to_y(s_body, scroll_y, LV_ANIM_OFF);
    if (old_screen == NULL)
    {
        lv_scr_load(s_screen);
    }
    else if (old_screen != s_screen)
    {
        s_transition = 1;
        if (back != 0)
        {
            animation_type = LV_SCR_LOAD_ANIM_OVER_RIGHT;
        }
        /* 旧页静止，新页滑入；只驱动一张屏幕的位置，减少每帧布局工作。 */
        lv_scr_load_anim(s_screen, animation_type, UI_ANIM_MS, 0, auto_delete != 0);
        animation = lv_anim_get(s_screen, NULL);
        if (animation != NULL)
        {
            lv_anim_set_path_cb(animation, lv_anim_path_ease_in_out);
        }
    }
}

static void set_status(lv_obj_t *obj, uint8_t online)
{
    if (obj == NULL)
    {
        return;
    }
    if (online)
    {
        set_label_text(obj, "ONLINE");
        set_text_color(obj, UI_GREEN);
    }
    else
    {
        set_label_text(obj, "NO DATA");
        set_text_color(obj, UI_RED);
    }
}

/* Fixed-point text avoids pulling printf floating-point support into firmware. */
static void set_decimal(lv_obj_t *obj, float value, uint8_t wrap)
{
    int32_t fixed;
    int32_t magnitude;
    const char *sign;
    if (!isfinite(value) || (!wrap && (value > 99999.0f || value < -99999.0f)))
    {
        set_label_text(obj, "--");
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
    if (fixed < 0)
    {
        magnitude = -fixed;
    }
    else
    {
        magnitude = fixed;
    }
    if (value < 0.0f)
    {
        sign = "-";
    }
    else
    {
        sign = "";
    }
    set_label_text_fmt(obj, "%s%ld.%ld", sign, (long)(magnitude / 10), (long)(magnitude % 10));
}

/* 航向暂停原因直接来自采样服务，不通过“有磁场数字”猜测融合状态。 */
static const char *mag_yaw_status(const app_imu_struct *imu)
{
    switch (imu->mag_fusion_state)
    {
    case APP_MAG_FUSION_ACTIVE:
        if (imu->mag_calibrated)
        {
            return "MAG NORTH / fused yaw";
        }
        else
        {
            return "MAG UNCAL / calibrate first";
        }
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
        if (imu->mag_persist_state == APP_MAG_PERSIST_LOAD_FAILED)
        {
            set_label_text(s_mag_cal_status, "EEPROM READ FAILED\nCalibration required");
        }
        else
        {
            set_label_text(s_mag_cal_status, "CAL NOT RUN");
        }
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
            set_label_text(s_mag_cal_status, "CAL RESTORED / EEPROM\nReady for magnetic fusion");
            return;
        case APP_MAG_PERSIST_SAVING:
            set_label_text(s_mag_cal_status, "CAL OK / SAVING EEPROM");
            return;
        case APP_MAG_PERSIST_SAVE_FAILED:
            set_label_text(s_mag_cal_status, "CAL OK / SAVE FAILED\nRAM only: check EEPROM/WP");
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
    set_label_text_fmt(s_mag_cal_status, "%s: %u samples\nX/Y/Z span: %ld/%ld/%ld uT", result, (unsigned)imu->mag_cal_samples,
                          (long)imu->mag_cal_span_uT[0], (long)imu->mag_cal_span_uT[1], (long)imu->mag_cal_span_uT[2]);
}

static void refresh_aht21(void)
{
    app_aht21_struct snapshot;
    const char *status = "WAITING";
    uint32_t color = UI_MUTED;
    if (s_aht21_card == NULL)
    {
        return;
    }
    app_aht21_get_snapshot(&snapshot);
    if (snapshot.valid != 0)
    {
        set_decimal(s_humidity, snapshot.sample.humidity_rh, 0);
        set_decimal(s_air_temperature, snapshot.sample.temperature_c, 0);
    }
    else
    {
        set_label_text(s_humidity, "--");
        set_label_text(s_air_temperature, "--");
    }
    if (snapshot.result != AHT21_OK)
    {
        color = UI_RED;
        switch (snapshot.result)
        {
        case AHT21_BUSY:
            status = "BUSY";
            break;
        case AHT21_I2C_ERROR:
            status = "I2C ERROR";
            break;
        case AHT21_CALIBRATION_ERROR:
            status = "NOT CALIBRATED";
            break;
        case AHT21_CRC_ERROR:
            status = "CRC ERROR";
            break;
        case AHT21_TIMEOUT:
            status = "CONVERSION TIMEOUT";
            break;
        default:
            status = "READ ERROR";
            break;
        }
    }
    else if (snapshot.measuring != 0)
    {
        status = "MEASURING";
        color = UI_ORANGE;
    }
    else if (snapshot.valid != 0)
    {
        status = "ONLINE / CRC OK";
        color = UI_GREEN;
    }
    set_label_text(s_aht21_status, status);
    set_text_color(s_aht21_status, color);
    set_label_text_fmt(s_aht21_samples, "SAMPLES %lu / 2s", (unsigned long)snapshot.sample_count);
}

static void refresh_weather(void)
{
    app_weather_struct weather;
    if (s_weather_label == NULL)
    {
        return;
    }
    app_weather_get_snapshot(&weather);
    if (weather.valid)
    {
        int32_t magnitude;
        const char *sign;
        if (weather.temperature_x10 < 0)
        {
            magnitude = -weather.temperature_x10;
            sign = "-";
        }
        else
        {
            magnitude = weather.temperature_x10;
            sign = "";
        }
        set_label_text_fmt(s_weather_value, "%s%ld.%ld C", sign, (long)(magnitude / 10), (long)(magnitude % 10));
        set_label_text(s_weather_condition, app_weather_description(weather.code));
        if (s_weather_icon_code != weather.code)
        {
            s_weather_icon_code = weather.code;
            lv_obj_invalidate(s_weather_icon);
        }
        set_label_text_fmt(s_weather_label, "%s%ld.%ldC / RH %u%%\n%lu min ago", sign,
                           (long)(magnitude / 10), (long)(magnitude % 10), (unsigned)weather.humidity_rh,
                           (unsigned long)(lv_tick_elaps(weather.received_ms) / 60000U));
        if (lv_tick_elaps(weather.received_ms) >= 3600000U)
        {
            set_text_color(s_weather_label, UI_MUTED);
            set_text_color(s_weather_condition, UI_MUTED);
        }
        else
        {
            set_text_color(s_weather_label, UI_TEXT);
            set_text_color(s_weather_condition, UI_TEXT);
        }
    }
}

static void refresh_battery(void)
{
    app_battery_struct battery;
    uint32_t color = UI_BATTERY_GREEN;
    if (s_battery_label == NULL)
    {
        return;
    }
    app_battery_get_snapshot(&battery);
    if (battery.valid)
    {
        if (lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN);
        }
        lv_bar_set_value(s_battery_bar, battery.percent, LV_ANIM_OFF);
        if (s_page == PAGE_WATCH)
        {
            set_label_text_fmt(s_battery_label, "~%u%% %lu.%02luV", (unsigned int)battery.percent,
                               (unsigned long)(battery.voltage_mv / 1000U), (unsigned long)(battery.voltage_mv % 1000U / 10U));
        }
        else
        {
            set_label_text_fmt(s_battery_label, "~%u%%", (unsigned int)battery.percent);
        }
        if (battery.percent <= 20U)
        {
            color = UI_RED;
            set_text_color(s_battery_label, UI_RED);
        }
        else
        {
            set_text_color(s_battery_label, UI_TEXT);
        }
        /* 仅颜色变化时设置样式，保持电量不变时不重复刷屏。 */
        if (lv_obj_get_style_bg_color(s_battery_bar, LV_PART_INDICATOR).full != lv_color_hex(color).full)
        {
            lv_obj_set_style_bg_color(s_battery_bar, lv_color_hex(color), LV_PART_INDICATOR);
        }
    }
    else
    {
        if (!lv_obj_has_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_add_flag(s_battery_bar, LV_OBJ_FLAG_HIDDEN);
        }
        if (battery.result == HAL_OK)
        {
            set_label_text(s_battery_label, "--%");
            set_text_color(s_battery_label, UI_MUTED);
        }
        else
        {
            set_label_text(s_battery_label, "ADC ERR");
            set_text_color(s_battery_label, UI_RED);
        }
    }
}

static void refresh_data(void)
{
    app_imu_struct imu;
    app_baro_struct baro;
    uint8_t online;
    if (s_time != NULL)
    {
        refresh_clock();
        return;
    }
    refresh_battery();
    refresh_weather();
    if (s_imu_status != NULL)
    {
        online = App_IMU_GetSnapshot(&imu);
        set_status(s_imu_status, online);
        compass_update(&imu, online);
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
                set_label_text(s_angles[0], "--");
                set_label_text(s_angles[1], "--");
                set_label_text(s_angles[2], "--");
            }
            if (online)
            {
                set_label_text(s_yaw_source, mag_yaw_status(&imu));
            }
            else
            {
                set_label_text(s_yaw_source, "GYRO ONLY / NO IMU FRAME");
            }
            if (imu.mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                set_label_text(s_mag_status, "MAG CALIBRATING");
            }
            else if (imu.mag_cal_state == APP_MAG_CAL_FAILED)
            {
                set_label_text(s_mag_status, "CAL FAILED / rotate all axes");
            }
            else if (imu.mag_cal_state == APP_MAG_CAL_DONE)
            {
                if (imu.mag_persist_state == APP_MAG_PERSIST_SAVE_FAILED)
                {
                    set_label_text(s_mag_status, "CAL OK / SAVE FAILED");
                }
                else if (imu.mag_persist_state == APP_MAG_PERSIST_RESTORED)
                {
                    if (imu.mag_online)
                    {
                        set_label_text(s_mag_status, "EEPROM CAL / MAG ONLINE");
                    }
                    else
                    {
                        set_label_text(s_mag_status, "EEPROM CAL / MAG NO DATA");
                    }
                }
                else
                {
                    if (imu.mag_online)
                    {
                        set_label_text(s_mag_status, "CAL OK / MAG ONLINE");
                    }
                    else
                    {
                        set_label_text(s_mag_status, "CAL OK / MAG NO DATA");
                    }
                }
            }
            else
            {
                if (imu.mag_online)
                {
                    set_label_text(s_mag_status, "LSM303DLHC / ONLINE");
                }
                else
                {
                    set_label_text(s_mag_status, "LSM303DLHC / NO DATA");
                }
            }
            if (imu.mag_online && imu.mag_persist_state != APP_MAG_PERSIST_SAVE_FAILED)
            {
                set_text_color(s_mag_status, UI_GREEN);
            }
            else
            {
                set_text_color(s_mag_status, UI_RED);
            }
            if (imu.mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                set_label_text(s_mag_cal_label, "FINISH CALIBRATION");
            }
            else
            {
                set_label_text(s_mag_cal_label, "CALIBRATE MAG");
            }
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
                        set_label_text(s_mag_values[i], "--");
                    }
                }
            }
        }
    }
    if (s_baro_status != NULL && lv_tick_elaps(s_altitude_trend.sample_tick) >= UI_BARO_REFRESH_MS)
    {
        online = App_Baro_GetSnapshot(&baro);
        set_status(s_baro_status, online);
        if (online && baro.pressure_pa > 0 && isfinite(baro.alt_m) && baro.alt_m >= -30000.0f && baro.alt_m <= 30000.0f)
        {
            trend_update(&s_altitude_trend, (lv_coord_t)roundf(baro.alt_m), 1);
        }
        else
        {
            trend_update(&s_altitude_trend, LV_CHART_POINT_NONE, 1);
        }
        if (s_pressure != NULL)
        {
            if (online && baro.pressure_pa > 0)
            {
                set_label_text_fmt(s_pressure, "%ld.%02ld", (long)(baro.pressure_pa / 100), (long)(baro.pressure_pa % 100));
                set_decimal(s_altitude, baro.alt_m, 0);
            }
            else
            {
                set_label_text(s_pressure, "--");
                set_label_text(s_altitude, "--");
            }
        }
    }
    refresh_bluetooth();
    refresh_heart();
    refresh_aht21();
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
            s_key_armed = s_screen_on;
        }
        else if (s_key_armed)
        {
            s_key_armed = 0;
            if (s_screen_on)
            {
                App_UI_Back();
            }
        }
    }
}

/* PA4 上升沿中断由 HAL 清除；UI 任务扫描电平，按下和松开均消抖，不在 ISR 操作屏幕。 */
static void poll_wake_key(void)
{
    uint8_t pressed = HAL_GPIO_ReadPin(WAKE_GPIO_Port, WAKE_Pin) == GPIO_PIN_SET;
    lv_indev_t *indev;

    if (pressed != s_wake_raw)
    {
        s_wake_raw = pressed;
        s_wake_tick = lv_tick_get();
    }
    if (s_wake_raw != s_wake_stable && lv_tick_elaps(s_wake_tick) >= UI_KEY_MS)
    {
        s_wake_stable = s_wake_raw;
        if (s_wake_stable)
        {
            s_screen_on = !s_screen_on;
            if (s_screen_on)
            {
                s_idle_dimmed = 0;
                lv_disp_trig_activity(NULL);
                lv_timer_resume(_lv_disp_get_refr_timer(NULL));
                lv_timer_ready(_lv_disp_get_refr_timer(NULL));
                s_refresh_tick = lv_tick_get() - UI_REFRESH_MS;
                lv_obj_invalidate(s_screen);
                LCD_Set_Light(s_brightness);
            }
            else
            {
                LCD_Set_Light(0);
                lv_timer_pause(_lv_disp_get_refr_timer(NULL));
            }
            /* 熄屏不接收触摸；亮屏时等待原触点松开，避免误点击。 */
            app_touch_set_enabled(s_screen_on);
            for (indev = lv_indev_get_next(NULL); indev != NULL; indev = lv_indev_get_next(indev))
            {
                if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER)
                {
                    lv_indev_reset(indev, NULL);
                    lv_indev_enable(indev, s_screen_on != 0);
                    if (s_screen_on)
                    {
                        lv_indev_wait_release(indev);
                    }
                }
            }
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

/* 保留LVGL输入轮询，触摸（含持续按住）由输入引擎更新活动时间。 */
static void poll_touch_idle(void)
{
    uint8_t dimmed;
    if (s_screen_on == 0)
    {
        return;
    }
    dimmed = lv_disp_get_inactive_time(NULL) >= UI_TOUCH_IDLE_MS;
    if (dimmed == s_idle_dimmed)
    {
        return;
    }
    s_idle_dimmed = dimmed;
    if (s_idle_dimmed != 0)
    {
        LCD_Set_Light(UI_MIN_BRIGHTNESS);
    }
    else
    {
        LCD_Set_Light(s_brightness);
        s_last_second = UINT32_MAX;
        s_clock_refresh_tick = lv_tick_get() - 1000U;
        s_refresh_tick = lv_tick_get() - UI_REFRESH_MS;
    }
}

static void publish_power_state(void)
{
    uint8_t idle = s_screen_on == 0 || s_idle_dimmed != 0;
    app_task_set_power_state(idle, idle == 0 && s_page == PAGE_MOTION);
}

static void ui_timer_cb(lv_timer_t *timer)
{
    ui_history_t previous;
    ui_page_t next;
    (void)timer;
    bluetooth_receive(); /* 唯一接收消费者；切到其他页面时也继续保存数据。 */
    bluetooth_send_heart();
    poll_wake_key();
    poll_key();
    poll_touch_idle();
    publish_power_state();
    /* Rebuild only after release and after the previous animation, so event
     * targets stay valid and a gesture cannot turn into a card click. */
    if (s_screen_on && !s_transition && !touch_pressed())
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
    if (s_screen_on && !s_transition && lv_tick_elaps(s_refresh_tick) >= UI_REFRESH_MS)
    {
        s_refresh_tick = lv_tick_get();
        refresh_data();
    }
}

uint8_t App_UI_IsScreenOn(void)
{
    return s_screen_on;
}

void App_UI_ServiceSleeping(void)
{
    bluetooth_receive();
    bluetooth_send_heart();
    poll_wake_key();
    poll_key();
    publish_power_state();
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
    /* 开机时按住 SW2 只用于上电，松开不立即熄屏。 */
    s_wake_raw = s_wake_stable = HAL_GPIO_ReadPin(WAKE_GPIO_Port, WAKE_Pin) == GPIO_PIN_SET;
    s_wake_tick = s_key_tick = s_refresh_tick = lv_tick_get();
    lv_disp_trig_activity(NULL);
    lv_timer_create(ui_timer_cb, UI_POLL_MS, NULL);
    publish_power_state();
}
