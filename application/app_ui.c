/**
 * @file app_ui.c
 * @brief 240x280 dark watch UI. All LVGL calls run in the display task.
 *        SYDNEY数字表盘不滚动；每项功能从独立控件进入详情页。
 *        KEY_1 (PA5, active low) goes back through page/scroll history.
 *        WAKE (PA4, active high) toggles the backlight after a debounced press.
 */
#include "app_ui.h"
#include "app_imu.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_aht21.h"
#include "app_battery.h"
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
#define UI_RED              0xBA5057
#define UI_POLL_MS          10
#define UI_REFRESH_MS       100
#define UI_ANIM_MS          160
#define UI_KEY_MS           30
#define UI_HISTORY          16
#define UI_BT_LOG_SIZE      512
#define UI_BT_READ_SIZE     64
#define UI_BT_RX_BUDGET     256
#define UI_BT_TX_TIMEOUT_MS 20U
#define UI_WATCH_DIAMETER   236

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

static lv_obj_t *s_screen;
static lv_obj_t *s_battery_label;
static lv_obj_t *s_weather_label;
static lv_obj_t *s_weather_value;
static lv_obj_t *s_body;
static lv_obj_t *s_time;
static lv_obj_t *s_minutes;
static lv_obj_t *s_watch_heart;
static lv_obj_t *s_watch_temperature;
static lv_obj_t *s_clock_hint;
LV_FONT_DECLARE(ui_sydney_digits);
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
static lv_obj_t *s_heart_card;
static lv_obj_t *s_heart_value;
static lv_obj_t *s_heart_status;
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
static uint8_t s_bt_last_cr;
static uint8_t s_bt_dirty;
static const char *s_bt_tx_text = "TX READY";
static uint32_t s_bt_tx_color = UI_MUTED;
static ui_page_t s_page;
static ui_history_t s_history[UI_HISTORY];
static uint8_t s_history_count;
static uint8_t s_brightness = 50;
static uint8_t s_screen_on = 1;
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
static uint8_t s_wake_raw;
static uint8_t s_wake_stable;
static uint32_t s_wake_tick;

static void ui_timer_cb(lv_timer_t *timer);
static void refresh_data(void);
static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back);
static void refresh_bluetooth(void);

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

static void card(const char *icon, const char *title, int index, uint32_t accent, const ui_page_t *destination)
{
    int width = index == 6 ? 212 : 100;
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

/* 帆顶和桥拱按曲线采样为细分轮廓，保留原生LVGL抗锯齿，无整屏图片缓冲。 */
static void sydney_draw_cb(lv_event_t *event)
{
    static const lv_point_t sails[][12] =
    {
        {{0, 127}, {0, 103}, {7, 102}, {15, 103}, {23, 105}, {31, 110}, {38, 116}, {44, 123}, {46, 127}, {32, 127}, {16, 127}, {0, 127}},
        {{26, 127}, {30, 115}, {35, 104}, {41, 94}, {49, 85}, {58, 78}, {67, 73}, {76, 70}, {85, 69}, {79, 95}, {74, 114}, {71, 127}},
        {{76, 127}, {80, 95}, {88, 93}, {95, 93}, {101, 94}, {107, 96}, {112, 99}, {117, 104}, {121, 111}, {124, 119}, {126, 127}, {105, 127}},
        {{105, 127}, {109, 116}, {115, 107}, {120, 103}, {128, 102}, {135, 103}, {130, 113}, {125, 127}, {120, 127}, {115, 127}, {110, 127}, {105, 127}}
    };
    static const lv_point_t folds[][3] =
    {
        {{0, 127}, {8, 117}, {16, 127}},
        {{16, 127}, {26, 112}, {31, 127}},
        {{26, 127}, {35, 104}, {71, 127}},
        {{76, 127}, {80, 95}, {91, 127}},
        {{105, 127}, {115, 107}, {125, 127}}
    };
    static const lv_point_t bridge[] =
    {
        {100, 128}, {104, 124}, {108, 120}, {112, 117}, {116, 113}, {120, 110},
        {124, 108}, {128, 105}, {132, 103}, {136, 101}, {140, 99}, {144, 98},
        {148, 97}, {152, 96}, {157, 95}, {161, 95}, {165, 94}, {169, 95},
        {173, 95}, {177, 96}, {181, 97}, {185, 98}, {189, 99}, {194, 101},
        {198, 103}, {202, 105}, {206, 108}, {210, 110}, {214, 113}, {218, 117},
        {223, 120}, {227, 124}, {231, 128}
    };
    static const uint32_t water_colors[] = {0x4D8DA7, 0x37758D, 0x2B5D74, 0x20475E};
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    lv_draw_rect_dsc_t fill;
    lv_draw_line_dsc_t line;
    lv_area_t area;
    lv_point_t points[12], first, last;
    int16_t offset;
    uint8_t i, j;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    lv_draw_rect_dsc_init(&fill);
    fill.bg_color = lv_color_hex(UI_TEXT);
    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 12; ++j)
        {
            points[j].x = area.x1 + sails[i][j].x;
            points[j].y = area.y1 + sails[i][j].y;
        }
        lv_draw_polygon(context, &fill, points, 12);
    }
    fill.bg_color = lv_color_hex(UI_BLUE);
    for (i = 0; i < 5; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            points[j].x = area.x1 + folds[i][j].x;
            points[j].y = area.y1 + folds[i][j].y;
        }
        lv_draw_triangle(context, &fill, points);
    }
    lv_draw_line_dsc_init(&line);
    line.color = lv_color_hex(0xC6E6EF);
    line.width = 1;
    for (i = 0; i < 12; ++i)
    {
        first.x = last.x = area.x1 + 109 + i * 10;
        offset = 109 + i * 10 - 166;
        first.y = area.y1 + 95 + offset * offset * 33 / (66 * 66);
        last.y = area.y1 + 127;
        lv_draw_line(context, &line, &first, &last);
    }
    line.color = lv_color_hex(UI_BLUE);
    line.width = 6;
    line.round_start = 1;
    line.round_end = 1;
    for (i = 1; i < sizeof(bridge) / sizeof(bridge[0]); ++i)
    {
        first.x = area.x1 + bridge[i - 1].x;
        first.y = area.y1 + bridge[i - 1].y;
        last.x = area.x1 + bridge[i].x;
        last.y = area.y1 + bridge[i].y;
        lv_draw_line(context, &line, &first, &last);
    }
    line.round_start = line.round_end = 0;
    line.width = 7;
    first.x = area.x1 + 101;
    last.x = area.x1 + 235;
    first.y = last.y = area.y1 + 130;
    lv_draw_line(context, &line, &first, &last);
    line.width = 5;
    first.x = last.x = area.x1 + 228;
    first.y = area.y1 + 114;
    lv_draw_line(context, &line, &first, &last);
    line.color = lv_color_hex(UI_TEXT);
    line.width = 2;
    first.y = area.y1 + 112;
    last.y = area.y1 + 116;
    lv_draw_line(context, &line, &first, &last);
    line.width = 6;
    first.x = area.x1;
    last.x = area.x1 + 235;
    for (i = 0; i < 4; ++i)
    {
        first.y = last.y = area.y1 + 143 + i * 10;
        line.color = lv_color_hex(water_colors[i]);
        lv_draw_line(context, &line, &first, &last);
    }
}

/* 最外层黑色表圈用同心圆描边；DRAW_POST使轮廓盖在城市剪影之上。 */
static void sydney_bezel_draw_cb(lv_event_t *event)
{
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    lv_draw_arc_dsc_t ring;
    lv_area_t area;
    lv_point_t center;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    center.x = area.x1 + UI_WATCH_DIAMETER / 2;
    center.y = area.y1 + UI_WATCH_DIAMETER / 2;
    lv_draw_arc_dsc_init(&ring);
    ring.color = lv_color_hex(0x000000);
    ring.width = 8;
    lv_draw_arc(context, &ring, &center, 118, 0, 360);
    ring.color = lv_color_hex(0xE3EDF3);
    ring.width = 1;
    lv_draw_arc(context, &ring, &center, 117, 0, 360);
    ring.color = lv_color_hex(0x657481);
    lv_draw_arc(context, &ring, &center, 111, 0, 360);
}

static void watch_page(void)
{
    static const ui_page_t weather = PAGE_WEATHER;
    static const ui_page_t heart = PAGE_HEART;
    static const ui_page_t bluetooth = PAGE_BLUETOOTH;
    lv_obj_t *scene, *entry;
    s_time = label(s_body, "--", 105, 28, &ui_sydney_digits, UI_TEXT);
    make_entry(s_time, &bluetooth);
    scene = panel(s_body, 0, 0, UI_WATCH_DIAMETER, 180, UI_BG);
    lv_obj_set_style_bg_opa(scene, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(scene, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scene, sydney_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    s_minutes = label(s_body, "--", 105, 129, &ui_sydney_digits, UI_TEXT);
    make_entry(s_minutes, &bluetooth);
    entry = panel(s_body, 44, 33, 61, 48, UI_BG);
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    s_clock_hint = label(entry, "SET VIA BLE", 0, 0, &lv_font_montserrat_10, UI_BLUE);
    lv_obj_set_width(s_clock_hint, 61);
    lv_label_set_long_mode(s_clock_hint, LV_LABEL_LONG_WRAP);
    s_watch_temperature = label(entry, "--", 0, 17, &lv_font_montserrat_20, UI_TEXT);
    make_entry(entry, &weather);
    entry = panel(s_body, 36, 174, 69, 36, UI_BG);
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    heart_icon(entry, 0, 2);
    s_watch_heart = label(entry, "--", 25, 0, &lv_font_montserrat_20, UI_TEXT);
    label(entry, "BPM", 25, 24, &lv_font_montserrat_10, UI_MUTED);
    make_entry(entry, &heart);
    lv_obj_set_parent(s_battery_label, s_body);
    lv_obj_set_pos(s_battery_label, 70, 210);
    lv_obj_set_width(s_battery_label, 96);
    lv_obj_set_style_text_font(s_battery_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(s_battery_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_event_cb(s_body, sydney_bezel_draw_cb, LV_EVENT_DRAW_POST, NULL);
}

static void refresh_clock(void)
{
    watch_time_struct time;
    app_weather_struct weather;
    app_heart_struct heart;
    HAL_StatusTypeDef status = watch_rtc_get_time(&time);
    uint32_t seconds;
    app_heart_get_snapshot(&heart);
    if (heart.state == APP_HEART_VALID)
    {
        set_label_text_fmt(s_watch_heart, "%u", (unsigned)heart.bpm);
    }
    else
    {
        set_label_text(s_watch_heart, "--");
    }
    if (status != HAL_OK || !watch_rtc_is_synced())
    {
        set_label_text(s_time, "--");
        set_label_text(s_minutes, "--");
        set_label_text(s_clock_hint, status == HAL_OK ? "SET VIA BLE" : "RTC ERROR");
        set_label_text(s_watch_temperature, "--");
        s_last_second = UINT32_MAX;
        return;
    }
    app_weather_get_snapshot(&weather);
    if (weather.valid)
    {
        int32_t magnitude = weather.temperature_x10 < 0 ? -weather.temperature_x10 : weather.temperature_x10;
        set_label_text(s_clock_hint, lv_tick_elaps(weather.received_ms) >= 3600000U
                       ? "OLD WEATHER" : app_weather_description(weather.code));
        set_label_text_fmt(s_watch_temperature, "%s%ld.%ldC", weather.temperature_x10 < 0 ? "-" : "",
                           (long)(magnitude / 10), (long)(magnitude % 10));
    }
    else
    {
        set_label_text(s_clock_hint, "NO WEATHER");
        set_label_text(s_watch_temperature, "--");
    }
    seconds = (uint32_t)time.hours * 3600U + (uint32_t)time.minutes * 60U + time.seconds;
    if (seconds != s_last_second)
    {
        s_last_second = seconds;
        set_label_text_fmt(s_time, "%02u", (unsigned)time.hours);
        set_label_text_fmt(s_minutes, "%02u", (unsigned)time.minutes);
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

static void imu_page(int y)
{
    static const char *const angles[3] = {"PITCH", "ROLL", "YAW"};
    static const char *const magnetic[3] = {"MAG X", "MAG Y", "MAG Z"};
    lv_obj_t *obj;
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
    s_baro_status = label(s_body, "WAITING", 2, y + 4, &lv_font_montserrat_10, UI_MUTED);
    s_pressure = metric("PRESSURE", "hPa", y + 26, UI_ORANGE);
    s_altitude = metric("ALTITUDE", "m", y + 126, UI_BLUE);
    note(s_body, "Altitude uses standard sea-level pressure. Weather can change this estimate.", y + 232);
    note(s_body, "SPL006 / live sensor snapshot", y + 286);
}

static void brightness_cb(lv_event_t *event)
{
    s_brightness = (uint8_t)lv_slider_get_value(lv_event_get_target(event));
    LCD_Set_Light(s_screen_on ? s_brightness : 0);
    set_label_text_fmt(s_brightness_label, "%d%%", (int)s_brightness);
}

/* 按字节接收，不要求发送端的一条信息在一次UART接收中完成。
 * 保留最近511个显示字符；非ASCII/非打印字节显示为\xHH，避免NUL截断文本。 */
static void bluetooth_receive(void)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t data[UI_BT_READ_SIZE];
    char text[UI_BT_READ_SIZE * 4];
    uint16_t received, length, remove, i, budget = UI_BT_RX_BUDGET;

    while (budget != 0)
    {
        received = kt6368a_read(data, budget < sizeof(data) ? budget : sizeof(data));
        if (received == 0)
        {
            break;
        }
        budget -= received;
        app_clock_receive(data, received, lv_tick_get());
        app_weather_receive(data, received, lv_tick_get());
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
    app_clock_service(lv_tick_get());
    app_weather_service(lv_tick_get());
}

static void bluetooth_clear_cb(lv_event_t *event)
{
    (void)event;
    if (s_transition)
    {
        return;
    }
    kt6368a_flush_rx();
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
    if (s_transition)
    {
        return;
    }
    /* 本板未接蓝牙状态脚，直接发送并显示 UART 返回结果。 */
    status = kt6368a_write(message, sizeof(message) - 1, UI_BT_TX_TIMEOUT_MS);
    s_bt_tx_color = status == HAL_OK ? UI_GREEN : UI_ORANGE;
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

/* 与原有蓝牙收发共用LVGL任务；每轮最多2帧/72字节，115200下约6.3ms，sensor任务不等待UART。 */
static void bluetooth_send_heart(void)
{
    app_heart_sample_struct samples[2];
    uint8_t frames[72];
    float channels[8];
    uint16_t count, i, channel, offset;
    uint32_t bits;
    HAL_StatusTypeDef status;
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
    status = kt6368a_write(frames, count * 36U, UI_BT_TX_TIMEOUT_MS);
    s_bt_tx_text = status == HAL_OK ? "TX HEART OK" : "TX HEART ERROR";
    s_bt_tx_color = status == HAL_OK ? UI_GREEN : UI_RED;
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
        text = status == HAL_TIMEOUT ? "INIT TIMEOUT" : (status == HAL_BUSY ? "INIT BUSY" : "INIT ERROR");
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
        set_label_text(s_bt_log, s_bt_text_length ? s_bt_text : "Waiting for data...");
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

static void heart_window(void)
{
    s_heart_card = panel(s_body, 0, 4, 212, 144, UI_CARD);
    label(s_heart_card, "HEART RATE / BPM", 14, 12, &lv_font_montserrat_10, UI_RED);
    heart_icon(s_heart_card, 174, 10);
    s_heart_value = label(s_heart_card, "--", 14, 35, &lv_font_montserrat_30, UI_RED);
    s_heart_status = label(s_heart_card, "STANDBY", 14, 82, &lv_font_montserrat_10, UI_MUTED);
    s_heart_button = label(s_heart_card, "START", 116, 42, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_set_size(s_heart_button, 82, 30);
    lv_obj_set_style_bg_color(s_heart_button, lv_color_hex(UI_BLUE), 0);
    lv_obj_set_style_bg_opa(s_heart_button, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_heart_button, 12, 0);
    lv_obj_set_style_pad_top(s_heart_button, 8, 0);
    lv_obj_set_style_text_align(s_heart_button, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(s_heart_button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_heart_button, heart_toggle_cb, LV_EVENT_CLICKED, NULL);
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
    set_text_color(s_heart_status, heart.state == APP_HEART_ERROR ? UI_RED : UI_MUTED);
    set_label_text(s_heart_button, heart.enabled ? "STOP" : "START");
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
    s_weather_label = s_weather_value = NULL;
    s_time = s_minutes = s_clock_hint = s_watch_heart = s_watch_temperature = NULL;
    s_imu_status = s_baro_status = NULL;
    s_pressure = s_altitude = s_brightness_label = NULL;
    s_heart_card = s_heart_value = s_heart_status = s_heart_button = NULL;
    s_aht21_card = s_humidity = s_air_temperature = s_aht21_status = s_aht21_samples = NULL;
    memset(s_angles, 0, sizeof(s_angles));
    memset(s_mag_values, 0, sizeof(s_mag_values));
    s_mag_status = s_yaw_source = s_mag_cal_label = NULL;
    s_mag_cal_status = NULL;
    s_bt_status = s_bt_count = s_bt_tx_status = s_bt_log = s_bt_log_box = NULL;
    s_bt_clock_status = NULL;
    s_last_second = UINT32_MAX;
}

static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back)
{
    static const char *titles[] = {"SYDNEY", "APPS", "BLUETOOTH", "DISPLAY", "HEART RATE", "MOTION",
                                   "BAROMETER", "CLIMATE", "WEATHER"};
    lv_obj_t *dot, *navigation;
    lv_obj_t *old_screen = s_screen;
    uint8_t i;
    uint8_t tab = page == PAGE_WATCH ? PAGE_WATCH : PAGE_SETTINGS;
    int width = lv_disp_get_hor_res(NULL);
    int height = lv_disp_get_ver_res(NULL);
    clear_page_refs();
    s_page = page;
    s_screen = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_screen);
    lv_obj_set_style_bg_color(s_screen, lv_color_hex(page == PAGE_WATCH ? 0x000000 : UI_BG), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(s_screen, gesture_cb, LV_EVENT_GESTURE, NULL);
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
    s_battery_label = label(s_screen, "BAT --.--V", width - 94, 15, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_battery_label, 80);
    lv_obj_set_style_text_align(s_battery_label, LV_TEXT_ALIGN_RIGHT, 0);
    s_body = panel(s_screen, (width - 212) / 2, 38, 212, height - 62, UI_BG);
    lv_obj_set_style_radius(s_body, 0, 0);
    if (page == PAGE_WATCH)
    {
        /* 表盘外保持纯黑，圆盘在屏幕内居中。 */
        lv_obj_set_pos(s_body, (width - UI_WATCH_DIAMETER) / 2, (height - UI_WATCH_DIAMETER) / 2);
        lv_obj_set_size(s_body, UI_WATCH_DIAMETER, UI_WATCH_DIAMETER);
        lv_obj_set_style_radius(s_body, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_clip_corner(s_body, true, 0);
    }
    if (page != PAGE_WATCH)
    {
        lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_body, page == PAGE_WATCH ? LV_DIR_NONE : LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_body, page == PAGE_WATCH ? LV_SCROLLBAR_MODE_OFF : LV_SCROLLBAR_MODE_AUTO);
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
        s_weather_value = metric("PHONE WEATHER", "BLE", 4, UI_BLUE);
        s_weather_label = note(s_body, "PHONE WEATHER / WAITING FOR PHONE", 104);
        note(s_body, "Weather comes from your phone. Open Bluetooth to receive updates.", 162);
        break;
    default:
        break;
    }
    if (page != PAGE_WATCH)
    {
        for (i = 0; i < 2; ++i)
        {
            dot = panel(s_screen, width / 2 - 12 + i * 16, height - 14, i == tab ? 12 : 5, 5, i == tab ? UI_BLUE : UI_BORDER);
            lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
            lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
        }
        navigation = label(s_screen, LV_SYMBOL_LEFT " BACK", 14, height - 17, &lv_font_montserrat_10, UI_BLUE);
        lv_obj_add_flag(navigation, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(navigation, 8);
        lv_obj_add_event_cb(navigation, back_cb, LV_EVENT_CLICKED, NULL);
    }
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
    set_label_text(obj, online ? "ONLINE" : "NO DATA");
    set_text_color(obj, online ? UI_GREEN : UI_RED);
}

/* Fixed-point text avoids pulling printf floating-point support into firmware. */
static void set_decimal(lv_obj_t *obj, float value, uint8_t wrap)
{
    int32_t fixed;
    int32_t magnitude;
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
    magnitude = fixed < 0 ? -fixed : fixed;
    set_label_text_fmt(obj, "%s%ld.%ld", value < 0.0f ? "-" : "", (long)(magnitude / 10), (long)(magnitude % 10));
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
        set_label_text(s_mag_cal_status, imu->mag_persist_state == APP_MAG_PERSIST_LOAD_FAILED
                                                ? "EEPROM READ FAILED\nCalibration required"
                                                : "CAL NOT RUN");
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
        int32_t magnitude = weather.temperature_x10 < 0 ? -weather.temperature_x10 : weather.temperature_x10;
        set_label_text_fmt(s_weather_value, "%s%ld.%ld C", weather.temperature_x10 < 0 ? "-" : "",
                           (long)(magnitude / 10), (long)(magnitude % 10));
        set_label_text_fmt(s_weather_label, "PHONE WEATHER / %s\n%s%ld.%ldC / RH %u%% / %lu min ago",
                           app_weather_description(weather.code), weather.temperature_x10 < 0 ? "-" : "",
                           (long)(magnitude / 10), (long)(magnitude % 10), (unsigned)weather.humidity_rh,
                           (unsigned long)(lv_tick_elaps(weather.received_ms) / 60000U));
        set_text_color(s_weather_label, lv_tick_elaps(weather.received_ms) >= 3600000U ? UI_MUTED : UI_TEXT);
    }
}

static void refresh_battery(void)
{
    app_battery_struct battery;
    if (s_battery_label == NULL)
    {
        return;
    }
    app_battery_get_snapshot(&battery);
    if (battery.valid)
    {
        set_label_text_fmt(s_battery_label, "BAT %lu.%02luV", (unsigned long)(battery.voltage_mv / 1000U),
                           (unsigned long)(battery.voltage_mv % 1000U / 10U));
        set_text_color(s_battery_label, UI_TEXT);
    }
    else
    {
        set_label_text(s_battery_label, battery.result == HAL_OK ? "BAT --.--V" : "BAT ADC ERR");
        set_text_color(s_battery_label, battery.result == HAL_OK ? UI_MUTED : UI_RED);
    }
}

static void refresh_data(void)
{
    app_imu_struct imu;
    app_baro_struct baro;
    uint8_t online;
    refresh_battery();
    refresh_weather();
    if (s_time != NULL)
    {
        refresh_clock();
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
                set_label_text(s_angles[0], "--");
                set_label_text(s_angles[1], "--");
                set_label_text(s_angles[2], "--");
            }
            set_label_text(s_yaw_source, online ? mag_yaw_status(&imu) : "GYRO ONLY / NO IMU FRAME");
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
                    set_label_text(s_mag_status, imu.mag_online ? "EEPROM CAL / MAG ONLINE" : "EEPROM CAL / MAG NO DATA");
                }
                else
                {
                    set_label_text(s_mag_status, imu.mag_online ? "CAL OK / MAG ONLINE" : "CAL OK / MAG NO DATA");
                }
            }
            else
            {
                set_label_text(s_mag_status, imu.mag_online ? "LSM303DLHC / ONLINE" : "LSM303DLHC / NO DATA");
            }
            set_text_color(s_mag_status, imu.mag_online && imu.mag_persist_state != APP_MAG_PERSIST_SAVE_FAILED ? UI_GREEN : UI_RED);
            set_label_text(s_mag_cal_label, imu.mag_cal_state == APP_MAG_CAL_RUNNING ? "FINISH CALIBRATION" : "CALIBRATE MAG");
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
    if (s_baro_status != NULL)
    {
        online = App_Baro_GetSnapshot(&baro);
        set_status(s_baro_status, online);
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
            LCD_Set_Light(s_screen_on ? s_brightness : 0);
            /* 熄屏不接收触摸；亮屏时等待原触点松开，避免误点击。 */
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

static void ui_timer_cb(lv_timer_t *timer)
{
    ui_history_t previous;
    ui_page_t next;
    (void)timer;
    bluetooth_receive(); /* 唯一接收消费者；切到其他页面时也继续保存数据。 */
    bluetooth_send_heart();
    poll_wake_key();
    poll_key();
    if (s_transition && lv_tick_elaps(s_transition_tick) >= UI_ANIM_MS + UI_POLL_MS)
    {
        s_transition = 0;
    }
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
    /* 开机时按住 SW2 只用于上电，松开不立即熄屏。 */
    s_wake_raw = s_wake_stable = HAL_GPIO_ReadPin(WAKE_GPIO_Port, WAKE_Pin) == GPIO_PIN_SET;
    s_wake_tick = s_key_tick = s_refresh_tick = lv_tick_get();
    lv_timer_create(ui_timer_cb, UI_POLL_MS, NULL);
}
