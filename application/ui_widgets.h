/**
 * @file ui_widgets.h
 * @brief UI内部公共控件、配色与趋势图；仅由LVGL显示任务调用。
 *
 * 调用方持有控件和趋势图上下文。本模块只操作传入的对象，不保存页面指针，
 * 不读取传感器、不处理页面跳转，也不分配额外的页面状态。
 */
#ifndef UI_WIDGETS_H
#define UI_WIDGETS_H

#include "lvgl.h"

#define UI_BG                0x041426
#define UI_CARD              0x0C2238
#define UI_BORDER            0x173B54
#define UI_TEXT              0xF4FAFF
#define UI_MUTED             0x7EABC1
#define UI_BLUE              0x70C5E8
#define UI_ORANGE            0xCE7438
#define UI_GREEN             0x398B70
#define UI_BATTERY_GREEN     0x45D483
#define UI_RED               0xBA5057
#define UI_TREND_POINTS      24

typedef struct
{
    lv_obj_t *chart;
    lv_obj_t *scale;
    lv_obj_t *scale_low;
    lv_chart_series_t *series;
    uint32_t sample_tick;
    uint32_t sample_period_ms;
} ui_trend_struct;

void ui_set_label_text(lv_obj_t *obj, const char *text);
void ui_set_label_text_fmt(lv_obj_t *obj, const char *format, ...);
void ui_set_text_color(lv_obj_t *obj, uint32_t color);
lv_obj_t *ui_panel(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color);
lv_obj_t *ui_label(lv_obj_t *parent, const char *text, int x, int y, const lv_font_t *font, uint32_t color);
lv_obj_t *ui_note(lv_obj_t *parent, const char *text, int y);
void ui_set_status(lv_obj_t *obj, uint8_t online);
/* wrap非0时将角度折到(-180,180]；数值按原规则截断为一位小数。 */
void ui_set_decimal(lv_obj_t *obj, float value, uint8_t wrap);
/* 坐标为像素，sample_ms为通用单点更新间隔；页面自行提供后台历史。 */
void ui_trend_create(ui_trend_struct *trend, lv_obj_t *parent, const char *title, int y, int height,
                     uint32_t color, uint32_t sample_ms);
void ui_trend_update_range(ui_trend_struct *trend, uint8_t altitude);
void ui_trend_update(ui_trend_struct *trend, lv_coord_t value, uint8_t altitude);

#endif /* UI_WIDGETS_H */
