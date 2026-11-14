/**
 * @file app_ui.c
 * @brief 240x280 dark watch UI. All LVGL calls run in the display task.
 *        三针RTC表盘不滚动；每项功能从独立控件进入详情页。
 *        KEY_1 (PA5, active low) goes back through page/scroll history.
 *        WAKE (PA4, active high) toggles the backlight after a debounced press.
 *
 *        模块职责: 本文件组织页面、交互与显示电源；蓝牙业务见app_ble_service.c，公共控件见ui_widgets.c。
 *        UI创建、刷新和业务服务由LVGL显示任务执行；App_UI_Back也允许在持有
 *        App_Display_Lock时提交返回请求。sensor任务维护业务快照，UI通过各模块的
 *        读取接口取数。
 *
 *        页面与缓存: 表盘与设置页互切保留缓存；进入详情时清空主页缓存，非当前主页
 *        立即删除，当前旧屏幕由切页动画结束后删除。返回主页按历史滚动位置重建。
 *
 *        切页时序: 触摸/按键回调只写入s_pending_page或s_pending_back，真正的
 *        create_page在ui_timer_cb里、手指已松开且上一段动画结束(s_transition==0)
 *        后才执行，避免删除仍被LVGL事件链引用的对象，也避免滑动手势被当成点击。
 *        动画完成由LV_EVENT_SCREEN_LOADED回调清s_transition。
 *
 *        蓝牙服务: 亮屏/熄屏均先推进app_ble_service，处理按键/抬腕后再更新BLE供电。
 *        页面只读服务快照并提交清日志、测试发送与ECO选择，不读取UART或管理协议状态。
 *
 *        亮灭屏: set_screen_on统一处理背光、触摸使能、刷新定时器与电源状态发布；
 *        休眠/唤醒命令失败时保留s_screen_requested与pending标志，下一轮UI服务继续重试。
 *        熄灭会先按"已熄屏"发布状态；亮屏则要等唤醒命令成功后才更新s_screen_on。
 */
#include "app_ui.h"
#include "watch_key.h"
#include "app_ble_service.h"
#include "ui_widgets.h"
#include "app_diag.h"
#include "app_imu.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_aht21.h"
#include "app_battery.h"
#include "app_touch.h"
#include "app_task.h"
#include "watch_rtc.h"
#include "app_clock.h"
#include "app_weather.h"
#include "lcd.h"
#include "lv_port_disp_template.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define UI_POLL_MS           10
#define UI_REFRESH_MS        100
#define UI_ANIM_MS           160
#define UI_KEY_MS            30
#define UI_TOUCH_IDLE_MS     10000
#define UI_ENTRY_DRAG_LIMIT  10
#define UI_MIN_BRIGHTNESS    2
#define UI_HISTORY           16
#define UI_TREND_SAMPLE_MS   1000
#define UI_BARO_REFRESH_MS   5000
#define UI_COMPASS_FILTER_MS 100
#define UI_COMPASS_MAX_DPS   360.0f

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
    PAGE_SPORT,
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
    lv_obj_t *time, *hint, *heart, *temperature, *steps;
    lv_obj_t *hour_hand, *minute_hand, *second_hand;
} ui_main_page_struct;

static ui_main_page_struct s_main_pages[2];

/* 页面控件引用按职责归组；通信缓冲、按键和表针点数组保持各自原有生命周期。 */
typedef struct
{
    struct
    {
        lv_obj_t *label;
        lv_obj_t *bar;
    } battery;
    struct
    {
        lv_obj_t *label;
        lv_obj_t *value;
        lv_obj_t *condition;
        lv_obj_t *icon;
        uint8_t icon_code;
    } weather;
    struct
    {
        lv_obj_t *time;
        lv_obj_t *hint;
        lv_obj_t *heart;
        lv_obj_t *temperature;
        lv_obj_t *steps;
        lv_obj_t *hour_hand;
        lv_obj_t *minute_hand;
        lv_obj_t *second_hand;
        uint32_t refresh_tick;
        uint32_t last_second;
    } watch;
    struct
    {
        lv_obj_t *status;
        lv_obj_t *angles[3];
        lv_obj_t *mag_values[3];
        lv_obj_t *mag_status;
        lv_obj_t *yaw_source;
        lv_obj_t *cal_label;
        lv_obj_t *cal_status;
        lv_obj_t *compass;
        lv_obj_t *compass_value;
        lv_obj_t *compass_status;
        lv_obj_t *compass_calibration;
        int16_t compass_heading;
        uint32_t compass_tick;
    } motion;
    struct
    {
        lv_obj_t *status;
        lv_obj_t *altitude_input;
        lv_obj_t *calibration_status;
        uint8_t calibration_waiting;
        lv_obj_t *pressure;
        lv_obj_t *altitude;
        ui_trend_struct trend;
        uint32_t refresh_tick;
        uint32_t history_sequence;
    } baro;
    struct
    {
        lv_obj_t *card;
        lv_obj_t *value;
        lv_obj_t *status;
        lv_obj_t *summary;
        lv_obj_t *details;
        lv_obj_t *details_title;
        lv_obj_t *button;
        lv_obj_t *auto_button;
        lv_obj_t *auto_label;
        ui_trend_struct trend;
    } heart;
    struct
    {
        lv_obj_t *card;
        lv_obj_t *humidity;
        lv_obj_t *temperature;
        lv_obj_t *status;
        lv_obj_t *samples;
    } climate;
    struct
    {
        lv_obj_t *brightness;
    } display;
    struct
    {
        lv_obj_t *status;
        lv_obj_t *count;
        lv_obj_t *tx_status;
        lv_obj_t *clock_status;
        lv_obj_t *log;
        lv_obj_t *log_box;
        lv_obj_t *power_label;
        uint32_t log_revision;
        uint8_t log_loaded;
    } bluetooth;
    struct
    {
        lv_obj_t *button;
        lv_obj_t *label;
    } sport;
} ui_page_refs_struct;

static ui_page_refs_struct s_view;

static lv_obj_t *s_screen;

static lv_obj_t *s_body;

static lv_point_t s_hour_points[2];
static lv_point_t s_minute_points[2];
static lv_point_t s_second_points[2];

static uint32_t s_wrist_sequence;

static uint32_t s_compass_color;
static float s_compass_display_deg;

static ui_page_t s_page;
static ui_history_t s_history[UI_HISTORY];
static uint8_t s_history_count;
static uint8_t s_brightness = LCD_DEFAULT_BRIGHTNESS_PERCENT; /* 开机默认亮度，与LCD_Init共用同一宏 */
static uint8_t s_screen_on = 1;
static uint8_t s_screen_requested = 1;
static uint8_t s_screen_power_pending;
static uint8_t s_sport_mode;

static void publish_power_state(void);
static void set_screen_on(uint8_t screen_on);
static uint8_t s_pending_back;
static ui_page_t s_pending_page = PAGE_COUNT;
static uint8_t s_transition;
static lv_point_t s_entry_press;
static uint8_t s_entry_dragged;
static uint32_t s_refresh_tick;

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

static void create_battery_bar(lv_obj_t *parent, int x, int y)
{
    /* 电池正极小凸块，稍后作为bar的子对象画在右侧。 */
    lv_obj_t *terminal;
    /* 在parent下创建进度条对象，同时保存到全局s_view.battery.bar供刷新电量使用。 */
    s_view.battery.bar = lv_bar_create(parent);
    /* 清除进度条默认样式，背景、边框和指示器全部由下面逐项指定。 */
    lv_obj_remove_style_all(s_view.battery.bar);
    /* 设置进度条左上角坐标(x,y)。 */
    lv_obj_set_pos(s_view.battery.bar, x, y);
    /* 设置外框尺寸30x12像素。 */
    lv_obj_set_size(s_view.battery.bar, 30, 12);
    /* 主体背景色使用卡片色UI_CARD。 */
    lv_obj_set_style_bg_color(s_view.battery.bar, lv_color_hex(UI_CARD), LV_PART_MAIN);
    /* 主体背景不透明，盖住底下的表盘。 */
    lv_obj_set_style_bg_opa(s_view.battery.bar, LV_OPA_COVER, LV_PART_MAIN);
    /* 主体边框用暗蓝色UI_MUTED描出电池轮廓。 */
    lv_obj_set_style_border_color(s_view.battery.bar, lv_color_hex(UI_MUTED), LV_PART_MAIN);
    /* 边框宽度1像素。 */
    lv_obj_set_style_border_width(s_view.battery.bar, 1, LV_PART_MAIN);
    /* 外框圆角2像素，贴近电池外形。 */
    lv_obj_set_style_radius(s_view.battery.bar, 2, LV_PART_MAIN);
    /* 内边距2像素，让绿色指示条与外框留出间隙。 */
    lv_obj_set_style_pad_all(s_view.battery.bar, 2, LV_PART_MAIN);
    /* 指示器(已充电量部分)使用电池绿色UI_BATTERY_GREEN。 */
    lv_obj_set_style_bg_color(s_view.battery.bar, lv_color_hex(UI_BATTERY_GREEN), LV_PART_INDICATOR);
    /* 指示器背景不透明。 */
    lv_obj_set_style_bg_opa(s_view.battery.bar, LV_OPA_COVER, LV_PART_INDICATOR);
    /* 指示器圆角1像素，与外框内缘配合。 */
    lv_obj_set_style_radius(s_view.battery.bar, 1, LV_PART_INDICATOR);
    /* 去掉可点击和可滚动标志，电量条只做显示。 */
    lv_obj_clear_flag(s_view.battery.bar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    /* 允许子对象凸块画出边界，并让电量条初始隐藏，由refresh_battery在电量有效时显示。 */
    lv_obj_add_flag(s_view.battery.bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE | LV_OBJ_FLAG_HIDDEN);
    /* 进度范围设为0..100，与电量百分比一致。 */
    lv_bar_set_range(s_view.battery.bar, 0, 100);
    /* 在电量条右侧外沿画3x6像素小凸块，颜色UI_MUTED。 */
    terminal = ui_panel(s_view.battery.bar, 28, 1, 3, 6, UI_MUTED);
    /* 凸块圆角收成1像素，避免方形棱角。 */
    lv_obj_set_style_radius(terminal, 1, 0);
    /* 凸块不响应点击。 */
    lv_obj_clear_flag(terminal, LV_OBJ_FLAG_CLICKABLE);
}

static void navigate_cb(lv_event_t *event)
{
    /* 事件user_data指向静态保存的目标页枚举，只读取出。 */
    const ui_page_t *page = (const ui_page_t *)lv_event_get_user_data(event);
    /* 取本次事件类型，区分按下、按住移动和点击。 */
    lv_event_code_t code = lv_event_get_code(event);
    /* 临时保存当前触点坐标，用于计算拖动距离。 */
    lv_point_t point;
    if (code == LV_EVENT_PRESSED)
    {
        /* 按下瞬间记录触点，作为拖动判定的起点。 */
        lv_indev_get_point(lv_indev_get_act(), &s_entry_press);
        /* 清除上一轮的拖动标志，本次按下从"未拖动"开始。 */
        s_entry_dragged = 0;
    }
    else if (code == LV_EVENT_PRESSING)
    {
        /* 按住移动时取当前触点。 */
        lv_indev_get_point(lv_indev_get_act(), &point);
        /* 不足翻页距离的拖动也取消点击，移回起点后仍保持取消；横向或纵向位移超过10像素即锁定拖动标志。 */
        if (LV_ABS(point.x - s_entry_press.x) > UI_ENTRY_DRAG_LIMIT || LV_ABS(point.y - s_entry_press.y) > UI_ENTRY_DRAG_LIMIT)
        {
            /* 锁定拖动标志，后续释放不再当成点击。 */
            s_entry_dragged = 1;
        }
    }
    else if (code == LV_EVENT_CLICKED && !s_entry_dragged && !s_transition && !s_pending_back)
    {
        /* 只有未拖动、无切页动画、无待处理返回时，才记录目标页；切页本身推迟到ui_timer_cb执行，这里仅提交请求。 */
        s_pending_page = *page;
    }
}

static void gesture_cb(lv_event_t *event)
{
    /* 当前手势所属输入设备。 */
    lv_indev_t *indev = lv_indev_get_act();
    /* 本次识别出的手势方向。 */
    lv_dir_t dir;
    (void)event;
    /* 没有输入设备、正在切页或已有返回请求时忽略手势。 */
    if (indev == NULL || s_transition || s_pending_back)
    {
        return;
    }
    /* 读取输入设备当前手势方向。 */
    dir = lv_indev_get_gesture_dir(indev);
    /* 只处理左右滑动；上下滑动不属于翻页手势。 */
    if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT)
    {
        /* 表盘页的上下滑动先等待手指抬起，避免被当成点击。 */
        if (s_page == PAGE_WATCH && (dir == LV_DIR_TOP || dir == LV_DIR_BOTTOM))
        {
            lv_indev_wait_release(indev);
        }
        return;
    }
    /* 主页/设置页之间按方向加减页号切换。 */
    if (s_page <= PAGE_SETTINGS)
    {
        /* 左滑且还没到设置页，就前进一页。 */
        if (dir == LV_DIR_LEFT && s_page < PAGE_SETTINGS)
        {
            s_pending_page = (ui_page_t)(s_page + 1);
        }
        /* 右滑且不在表盘页，就退回一页。 */
        else if (dir == LV_DIR_RIGHT && s_page > PAGE_WATCH)
        {
            s_pending_page = (ui_page_t)(s_page - 1);
        }
    }
    /* 详情页右滑统一返回设置页。 */
    else if (dir == LV_DIR_RIGHT)
    {
        s_pending_page = PAGE_SETTINGS;
    }
    /* A swipe must never also click the card under the finger. */
    /* 让输入设备等手指抬起再继续，避免滑动同时触发卡片点击。 */
    lv_indev_wait_release(indev);
}

/* 入口的目标页保存在静态数据中；触摸释放后仍由原定时器切页。 */
static void make_entry(lv_obj_t *obj, const ui_page_t *destination)
{
    /* 给入口对象加可点击标志，使其能接收按下/点击事件。 */
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    /* 按下态背景改为边框色UI_BORDER，给出触摸反馈。 */
    lv_obj_set_style_bg_color(obj, lv_color_hex(UI_BORDER), LV_STATE_PRESSED);
    /* 按下态背景不透明，确保反馈色可见。 */
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_STATE_PRESSED);
    /* 注册navigate_cb并接收全部事件；destination作为user_data随事件传回。 */
    lv_obj_add_event_cb(obj, navigate_cb, LV_EVENT_ALL, (void *)destination);
}

/* 连续三次贝塞尔心形按16倍分辨率采样，400字节透明度保存在Flash，平滑连接两瓣与尖端。 */
static void heart_icon_draw_cb(lv_event_t *event)
{
    /* 20x20像素的8位透明度表，每字节一个像素，00全透明、ff全不透明。 */
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
        /* 图像描述: 8位透明度、宽20、高20。 */
        .header = {.cf = LV_IMG_CF_ALPHA_8BIT, .w = 20, .h = 20},
        /* 数据长度直接取透明度数组的字节数。 */
        .data_size = sizeof(opacity),
        /* 指向上面的透明度表。 */
        .data = opacity
    };
    /* 取得当前绘制上下文，后续手动调用LVGL绘制接口。 */
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    /* 图像绘制参数，描述重着色等选项。 */
    lv_draw_img_dsc_t image;
    /* 目标对象在屏幕上的坐标区域。 */
    lv_area_t area;
    /* 取事件目标控件的绝对坐标，作为图像绘制范围。 */
    lv_obj_get_coords(lv_event_get_target(event), &area);
    /* 把图像参数初始化为LVGL默认值。 */
    lv_draw_img_dsc_init(&image);
    /* 用UI_BLUE对透明度图像重着色，得到蓝色心形。 */
    image.recolor = lv_color_hex(UI_BLUE);
    /* 重着色不透明度完全覆盖。 */
    image.recolor_opa = LV_OPA_COVER;
    /* 在当前上下文把20x20心形画到控件区域。 */
    lv_draw_img(context, &image, &area, &heart);
}

static lv_obj_t *heart_icon(lv_obj_t *parent, int x, int y)
{
    /* 建一个20x20、背景为页面底色UI_BG的容器作为心形画布。 */
    lv_obj_t *obj = ui_panel(parent, x, y, 20, 20, UI_BG);
    /* 把容器背景改成透明，只保留自绘心形。 */
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    /* 画布本身不接收点击，点击交给外层入口。 */
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    /* 在LV_EVENT_DRAW_MAIN阶段调用heart_icon_draw_cb画心形。 */
    lv_obj_add_event_cb(obj, heart_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    /* 返回画布对象。 */
    return obj;
}

static void weather_icon_shape(lv_draw_ctx_t *context, const lv_area_t *origin, int x, int y, int w, int h, uint32_t color)
{
    /* 矩形绘制参数。 */
    lv_draw_rect_dsc_t style;
    /* 以origin左上角为基准，把相对坐标换算成绝对区域；宽高各减1得到闭区间右下角。 */
    lv_area_t area = {origin->x1 + x, origin->y1 + y, origin->x1 + x + w - 1, origin->y1 + y + h - 1};
    /* 初始化矩形样式为默认值。 */
    lv_draw_rect_dsc_init(&style);
    /* 圆角取最大值，把矩形画成圆形/胶囊形。 */
    style.radius = LV_RADIUS_CIRCLE;
    /* 填充色使用传入的24位颜色。 */
    style.bg_color = lv_color_hex(color);
    /* 在上下文中按样式填充该区域。 */
    lv_draw_rect(context, &style, &area);
}

static void weather_icon_line(lv_draw_ctx_t *context, const lv_area_t *origin, int x1, int y1, int x2, int y2, uint32_t color)
{
    /* 线条绘制参数。 */
    lv_draw_line_dsc_t style;
    /* 把相对起点换算为屏幕绝对坐标。 */
    lv_point_t start = {origin->x1 + x1, origin->y1 + y1};
    /* 把相对终点换算为屏幕绝对坐标。 */
    lv_point_t end = {origin->x1 + x2, origin->y1 + y2};
    /* 初始化线条样式为默认值。 */
    lv_draw_line_dsc_init(&style);
    /* 线条颜色使用传入的24位颜色。 */
    style.color = lv_color_hex(color);
    /* 线宽固定3像素。 */
    style.width = 3;
    /* 起点画圆头。 */
    style.round_start = 1;
    /* 终点画圆头。 */
    style.round_end = 1;
    /* 在上下文中绘制这条线段。 */
    lv_draw_line(context, &style, &start, &end);
}

/* 天气码沿用手机协议；矢量图形由显示任务绘制，不额外加载位图或字库。 */
static void weather_icon_draw_cb(lv_event_t *event)
{
    /* 取绘制上下文，后续所有矢量图形都画到这块目标上。 */
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    /* 控件绝对坐标，作为图标内部相对坐标的原点。 */
    lv_area_t area;
    /* 复制当前天气码，绘制过程中保持一致。 */
    uint8_t code = s_view.weather.icon_code;
    /* 循环计数。 */
    uint8_t i;
    /* 取事件目标控件坐标，后面叠加相对偏移。 */
    lv_obj_get_coords(lv_event_get_target(event), &area);
    /* 天气码255或描述为UNKNOWN时没有对应图形。 */
    if (code == 255 || strcmp(app_weather_description(code), "UNKNOWN") == 0)
    {
        /* 文本绘制参数。 */
        lv_draw_label_dsc_t style;
        /* 初始化文本样式。 */
        lv_draw_label_dsc_init(&style);
        /* 问号使用30号字体。 */
        style.font = &lv_font_montserrat_30;
        /* 文本颜色为灰色UI_MUTED。 */
        style.color = lv_color_hex(UI_MUTED);
        /* 文本在区域内居中对齐。 */
        style.align = LV_TEXT_ALIGN_CENTER;
        /* 下移16像素，让问号在图标区域垂直居中。 */
        area.y1 += 16;
        /* 绘制"?"作为未知天气占位；最后参数为lv_draw_label_hint_t *hint，传NULL表示不使用绘制位置缓存提示。 */
        lv_draw_label(context, &style, &area, "?", NULL);
        /* 未知分支结束，不再画云和雨。 */
        return;
    }
    /* 天气码0..2是晴/少云类，先画太阳光芒。 */
    if (code <= 2)
    {
        /* 8条光芒，每45度一条。 */
        for (i = 0; i < 8; ++i)
        {
            /* 计算当前角度的正弦，范围按LV_TRIGO_SIN_MAX归一化。 */
            int32_t sine = lv_trigo_sin(i * 45);
            /* 计算当前角度的余弦。 */
            int32_t cosine = lv_trigo_cos(i * 45);
            /* 以(27,25)为太阳中心，从半径17到22画一段光芒。 */
            weather_icon_line(context, &area, 27 + sine * 17 / LV_TRIGO_SIN_MAX, 25 + cosine * 17 / LV_TRIGO_SIN_MAX,
                              27 + sine * 22 / LV_TRIGO_SIN_MAX, 25 + cosine * 22 / LV_TRIGO_SIN_MAX, 0xF5BD58);
        }
        /* 在(15,13)画25x25的黄色太阳圆盘。 */
        weather_icon_shape(context, &area, 15, 13, 25, 25, 0xF5BD58);
        /* 天气码0是纯晴天，画完太阳就结束。 */
        if (code == 0)
        {
            return;
        }
    }
    /* 天气码3是阴天，用两个灰色圆叠出云团。 */
    if (code == 3)
    {
        /* 左下的云团。 */
        weather_icon_shape(context, &area, 9, 17, 24, 24, UI_MUTED);
        /* 右上更高的云团。 */
        weather_icon_shape(context, &area, 22, 12, 29, 29, UI_MUTED);
    }
    /* 下面四个白色圆/胶囊拼出主云体。 */
    /* 云体左圆。 */
    weather_icon_shape(context, &area, 7, 29, 25, 25, UI_TEXT);
    /* 云体中上圆。 */
    weather_icon_shape(context, &area, 22, 21, 30, 30, UI_TEXT);
    /* 云体右圆。 */
    weather_icon_shape(context, &area, 40, 32, 20, 20, UI_TEXT);
    /* 底部横向胶囊把三个圆连成完整云底。 */
    weather_icon_shape(context, &area, 17, 37, 36, 17, UI_TEXT);
    /* 雾天码45/48在云下画两条灰线。 */
    if (code == 45 || code == 48)
    {
        /* 第一条雾线，y=50。 */
        weather_icon_line(context, &area, 8, 50, 56, 50, UI_MUTED);
        /* 第二条雾线，y=58。 */
        weather_icon_line(context, &area, 12, 58, 51, 58, UI_MUTED);
    }
    /* 毛毛雨/雨类码段在云下画三道蓝色斜线。 */
    else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82))
    {
        /* 三条雨线，每条向右偏移15像素。 */
        for (i = 0; i < 3; ++i)
        {
            /* 从(18+i*15,56)斜向左下到(14+i*15,62)。 */
            weather_icon_line(context, &area, 18 + i * 15, 56, 14 + i * 15, 62, UI_BLUE);
        }
    }
    /* 雪类码段用三道"十字"表示雪花。 */
    else if ((code >= 71 && code <= 77) || code == 85 || code == 86)
    {
        /* 三片雪花，每片向右偏移16像素。 */
        for (i = 0; i < 3; ++i)
        {
            /* 雪花横线。 */
            weather_icon_line(context, &area, 14 + i * 16, 58, 20 + i * 16, 58, UI_BLUE);
            /* 雪花竖线，与横线交叉。 */
            weather_icon_line(context, &area, 17 + i * 16, 55, 17 + i * 16, 61, UI_BLUE);
        }
    }
    /* 雷暴码95以上画折线闪电。 */
    else if (code >= 95)
    {
        /* 闪电第一段，从右上斜向左下。 */
        weather_icon_line(context, &area, 35, 46, 27, 54, 0xF5BD58);
        /* 闪电中间横向折返。 */
        weather_icon_line(context, &area, 27, 54, 36, 54, 0xF5BD58);
        /* 闪电末段，再斜向左下收尖。 */
        weather_icon_line(context, &area, 36, 54, 28, 62, 0xF5BD58);
    }
}

static void card(const char *icon, const char *title, int index, uint32_t accent, const ui_page_t *destination)
{
    /* 卡片宽度: index 6(Display)横跨整行，其余(含index 7 Sport mode)半宽。 */
    int width;
    if (index == 6)
    {
        /* 单独占一行的宽卡片。 */
        width = 212;
    }
    else
    {
        /* 两列布局下的半宽卡片。 */
        width = 100;
    }
    /* 按index算网格坐标: 列=(index%2)*112，行=(index/2)*80再加4像素顶距。 */
    lv_obj_t *obj = ui_panel(s_body, (index % 2) * 112, 4 + (index / 2) * 80, width, 72, UI_CARD);
    /* 卡片中央的图标或符号控件。 */
    lv_obj_t *symbol;
    /* 卡片底部标题，宽度稍后统一设为卡片宽并居中。 */
    lv_obj_t *caption = ui_label(obj, title, 0, 46, &lv_font_montserrat_10, UI_TEXT);
    /* 目标页是心率页时，用自绘心形代替字体图标。 */
    if (*destination == PAGE_HEART)
    {
        /* 心形居中: (卡片宽-20)/2，纵向y=12。 */
        symbol = heart_icon(obj, (width - 20) / 2, 12);
    }
    else
    {
        /* 其他卡片用20号字体显示传入的符号，颜色取强调色。 */
        symbol = ui_label(obj, icon, 0, 12, &lv_font_montserrat_20, accent);
        /* 符号标签宽度撑满卡片，便于居中。 */
        lv_obj_set_width(symbol, width);
        /* 符号文本水平居中。 */
        lv_obj_set_style_text_align(symbol, LV_TEXT_ALIGN_CENTER, 0);
    }
    /* 标题标签宽度撑满卡片。 */
    lv_obj_set_width(caption, width);
    /* 标题文本水平居中。 */
    lv_obj_set_style_text_align(caption, LV_TEXT_ALIGN_CENTER, 0);
    /* 把整张卡片注册为入口，点击后跳到destination页。 */
    make_entry(obj, destination);
}

/* 恢复原60格时钟刻度，使用原生LVGL线条，不占用整屏图片缓冲。 */
static void clock_dial_draw_cb(lv_event_t *event)
{
    /* 事件目标就是表盘容器，取它作为坐标原点。 */
    lv_obj_t *dial = lv_event_get_target(event);
    /* 绘制上下文。 */
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    /* 表盘绝对坐标区域。 */
    lv_area_t area;
    /* 刻度线样式，循环中按主/次刻度改写颜色和线宽。 */
    lv_draw_line_dsc_t style;
    /* 刻度内端和外端点。 */
    lv_point_t inner, outer;
    /* 当前刻度角度(度)、半径(像素)及三角函数值。 */
    int16_t angle, radius, sine, cosine;
    /* 60个刻度计数。 */
    uint8_t i;
    /* 取表盘控件坐标，圆心取area左上角加(100,100)。 */
    lv_obj_get_coords(dial, &area);
    /* 初始化线条样式，循环内只改颜色和宽度。 */
    lv_draw_line_dsc_init(&style);
    /* 一圈60格，每格6度。 */
    for (i = 0; i < 60; ++i)
    {
        /* 第i格对应的角度，0度在正上方。 */
        angle = i * 6;
        /* 每5格(30度)一个主刻度，即整点位置。 */
        if (i % 5 == 0)
        {
            /* 主刻度内端半径82像素。 */
            radius = 82;
            /* 主刻度用亮色UI_TEXT。 */
            style.color = lv_color_hex(UI_TEXT);
            /* 主刻度线宽3像素。 */
            style.width = 3;
        }
        else
        {
            /* 次刻度内端半径88像素，更短。 */
            radius = 88;
            /* 次刻度用暗色UI_MUTED。 */
            style.color = lv_color_hex(UI_MUTED);
            /* 次刻度线宽1像素。 */
            style.width = 1;
        }
        /* 当前角度的正弦。 */
        sine = lv_trigo_sin(angle);
        /* 当前角度的余弦。 */
        cosine = lv_trigo_cos(angle);
        /* 内端: 圆心加半径方向偏移，y轴向下故余弦项取负。 */
        inner.x = area.x1 + 100 + (int32_t)sine * radius / LV_TRIGO_SIN_MAX;
        inner.y = area.y1 + 100 - (int32_t)cosine * radius / LV_TRIGO_SIN_MAX;
        /* 外端固定在半径92像素处。 */
        outer.x = area.x1 + 100 + (int32_t)sine * 92 / LV_TRIGO_SIN_MAX;
        outer.y = area.y1 + 100 - (int32_t)cosine * 92 / LV_TRIGO_SIN_MAX;
        /* 画出一根由内端到外端的刻度线。 */
        lv_draw_line(context, &style, &inner, &outer);
    }
}

static lv_obj_t *clock_hand(lv_obj_t *dial, lv_point_t *points, uint8_t width, uint32_t color)
{
    /* 在表盘容器上创建线条对象作为指针。 */
    lv_obj_t *hand = lv_line_create(dial);
    /* 去掉线条默认样式。 */
    lv_obj_remove_style_all(hand);
    /* 指针颜色使用传入值(时针白、分针橙、秒针红)。 */
    lv_obj_set_style_line_color(hand, lv_color_hex(color), 0);
    /* 指针线宽由调用方指定，单位像素。 */
    lv_obj_set_style_line_width(hand, width, 0);
    /* 线段两端画圆头。 */
    lv_obj_set_style_line_rounded(hand, true, 0);
    /* 指针不接收点击、不参与滚动。 */
    lv_obj_clear_flag(hand, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    /* 两个点的x都先固定在表盘中心x=100。 */
    points[0].x = points[1].x = 100;
    /* 起点在圆心y=100。 */
    points[0].y = 100;
    /* 终点先放在上方y=40，之后由set_clock_hand按角度改写。 */
    points[1].y = 40;
    /* 把这两个点交给线条对象，2表示两个端点。 */
    lv_line_set_points(hand, points, 2);
    /* 返回指针对象。 */
    return hand;
}

static void set_clock_hand(lv_obj_t *hand, lv_point_t *points, float degrees, uint8_t radius)
{
    /* 角度由度转弧度，供sinf/cosf使用。 */
    float angle = degrees * 3.14159265f / 180.0f;
    /* 指针末端坐标。 */
    lv_point_t endpoint;
    /* x=圆心100+sin(角度)*半径，+0.5后截断实现四舍五入。 */
    endpoint.x = (lv_coord_t)(100.0f + sinf(angle) * radius + 0.5f);
    /* y=圆心100-cos(角度)*半径，屏幕y向下所以取负。 */
    endpoint.y = (lv_coord_t)(100.0f - cosf(angle) * radius + 0.5f);
    /* 只有端点确实变化时才更新线条，避免同一位置重复重绘。 */
    if (endpoint.x != points[1].x || endpoint.y != points[1].y)
    {
        /* 保存新端点。 */
        points[1] = endpoint;
        /* 用更新后的两点刷新线条几何。 */
        lv_line_set_points(hand, points, 2);
    }
}

static void steps_icon_draw_cb(lv_event_t *event)
{
    /* 图标所在控件区域。 */
    lv_area_t area;
    /* 绘制上下文。 */
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    /* 取控件绝对坐标作为相对绘制原点。 */
    lv_obj_get_coords(lv_event_get_target(event), &area);
    /* 左脚掌主体: 6x9绿色胶囊。 */
    weather_icon_shape(context, &area, 1, 4, 6, 9, UI_GREEN);
    /* 左脚趾: 4x3绿色小圆。 */
    weather_icon_shape(context, &area, 2, 0, 4, 3, UI_GREEN);
    /* 右脚掌主体: 6x9绿色胶囊。 */
    weather_icon_shape(context, &area, 11, 8, 6, 9, UI_GREEN);
    /* 右脚趾: 4x3绿色小圆。 */
    weather_icon_shape(context, &area, 12, 4, 4, 3, UI_GREEN);
}

static void watch_page(void)
{
    /* 两个静态目标页常量，分别供温湿度入口和心率入口保存指针。 */
    static const ui_page_t climate = PAGE_AHT21;
    static const ui_page_t heart = PAGE_HEART;
    /* 200x200表盘容器，放在body坐标(20,18)。 */
    lv_obj_t *dial = ui_panel(s_body, 20, 18, 200, 200, UI_BG);
    /* 表盘上的小时数字、中心轴帽和临时入口容器。 */
    lv_obj_t *number, *hub, *entry;
    /* 小时数字文本缓冲，两位数字加结束符。 */
    char text[3];
    /* 1..12的小时循环变量。 */
    uint8_t hour;
    /* 顶部摘要留出8像素上边距，避免脚印贴住屏幕顶边；脚印图标容器20x18，背景先设为页面底色。 */
    entry = ui_panel(s_body, 16, 8, 20, 18, UI_BG);
    /* 容器背景改透明，只保留自绘脚印。 */
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    /* 图标容器不接收点击。 */
    lv_obj_clear_flag(entry, LV_OBJ_FLAG_CLICKABLE);
    /* 在DRAW_MAIN阶段用steps_icon_draw_cb画脚印。 */
    lv_obj_add_event_cb(entry, steps_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    /* 步数数值标签，起点(42,9)，14号白字。 */
    s_view.watch.steps = ui_label(s_body, "0", 42, 9, &lv_font_montserrat_14, UI_TEXT);
    /* 数值标签宽度限制92像素。 */
    lv_obj_set_width(s_view.watch.steps, 92);
    /* 数值右侧的"STEPS"固定文字，10号灰色。 */
    ui_label(s_body, "STEPS", 138, 11, &lv_font_montserrat_10, UI_MUTED);
    /* 右上角(208,9)的蓝色向上箭头符号，14号字体。 */
    ui_label(s_body, LV_SYMBOL_UP, 208, 9, &lv_font_montserrat_14, UI_BLUE);
    /* 表盘容器去掉圆角，改为矩形区域；圆形刻度由自绘回调绘制。 */
    lv_obj_set_style_radius(dial, 0, 0);
    /* 表盘背景改透明，露出页面底色。 */
    lv_obj_set_style_bg_opa(dial, LV_OPA_TRANSP, 0);
    /* 在DRAW_MAIN阶段用clock_dial_draw_cb画60格刻度。 */
    lv_obj_add_event_cb(dial, clock_dial_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    /* 表盘本身不接收点击，避免挡住指针区域的事件。 */
    lv_obj_clear_flag(dial, LV_OBJ_FLAG_CLICKABLE);
    /* 逐个摆放1..12点的小时数字。 */
    for (hour = 1; hour <= 12; ++hour)
    {
        /* 把小时数格式化为1..2位十进制字符串。 */
        lv_snprintf(text, sizeof(text), "%u", (unsigned)hour);
        /* 按30度/小时算圆周位置: 圆心(100,100)加半径75，y方向余弦取负。 */
        number = ui_label(dial, text, 90 + (int32_t)lv_trigo_sin(hour * 30) * 75 / LV_TRIGO_SIN_MAX,
                       92 - (int32_t)lv_trigo_cos(hour * 30) * 75 / LV_TRIGO_SIN_MAX, &lv_font_montserrat_14, UI_TEXT);
        /* 数字标签宽度固定20像素。 */
        lv_obj_set_width(number, 20);
        /* 数字在标签内水平居中。 */
        lv_obj_set_style_text_align(number, LV_TEXT_ALIGN_CENTER, 0);
    }
    /* 表盘下方的校时提示文字，10号灰色。 */
    s_view.watch.hint = ui_label(dial, "SET VIA BLE", 20, 126, &lv_font_montserrat_10, UI_MUTED);
    /* 提示宽度160像素。 */
    lv_obj_set_width(s_view.watch.hint, 160);
    /* 提示文本水平居中。 */
    lv_obj_set_style_text_align(s_view.watch.hint, LV_TEXT_ALIGN_CENTER, 0);
    /* 时针: 6像素白线，端点数组s_hour_points。 */
    s_view.watch.hour_hand = clock_hand(dial, s_hour_points, 6, UI_TEXT);
    /* 分针: 4像素橙线。 */
    s_view.watch.minute_hand = clock_hand(dial, s_minute_points, 4, UI_ORANGE);
    /* 秒针: 2像素红线。 */
    s_view.watch.second_hand = clock_hand(dial, s_second_points, 2, UI_RED);
    /* 中心轴帽: 10x10蓝色方块，先放在(95,95)。 */
    hub = ui_panel(dial, 95, 95, 10, 10, UI_BLUE);
    /* 轴帽圆角取最大值，变成正圆。 */
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
    /* 轴帽不接收点击。 */
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_CLICKABLE);
    /* 表盘下方时间文本，初始"--:--"，20号白字。 */
    s_view.watch.time = ui_label(s_body, "--:--", 88, 225, &lv_font_montserrat_20, UI_TEXT);
    /* 时间标签宽度64像素。 */
    lv_obj_set_width(s_view.watch.time, 64);
    /* 时间文本水平居中。 */
    lv_obj_set_style_text_align(s_view.watch.time, LV_TEXT_ALIGN_CENTER, 0);
    /* 左下心率入口容器: 64x22，位置(14,226)。 */
    entry = ui_panel(s_body, 14, 226, 64, 22, UI_BG);
    /* 容器背景透明，只显示心形和数值。 */
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    /* 在容器内(0,0)画20x20心形。 */
    heart_icon(entry, 0, 0);
    /* 心率数值标签，初始"--"，位于心形右侧。 */
    s_view.watch.heart = ui_label(entry, "--", 26, 3, &lv_font_montserrat_14, UI_TEXT);
    /* 整个心率容器作为入口，点击跳到心率详情页。 */
    make_entry(entry, &heart);
    /* 右下温湿度入口容器: 64x32，位置(166,220)。 */
    entry = ui_panel(s_body, 166, 220, 64, 32, UI_BG);
    /* 容器背景透明。 */
    lv_obj_set_style_bg_opa(entry, LV_OPA_TRANSP, 0);
    /* 两行文本: 第一行温度、第二行湿度，先填"--C\n--%"。 */
    s_view.watch.temperature = ui_label(entry, "--C\n--%", 0, 0, &lv_font_montserrat_14, UI_TEXT);
    /* 两行之间行距设为0，压缩容器高度。 */
    lv_obj_set_style_text_line_space(s_view.watch.temperature, 0, 0);
    /* 温湿度容器作为入口，点击跳到AHT21详情页。 */
    make_entry(entry, &climate);
    /* 把全局电量文字重新挂到表盘body下，供本页显示。 */
    lv_obj_set_parent(s_view.battery.label, s_body);
    /* 电量文字放到(78,254)。 */
    lv_obj_set_pos(s_view.battery.label, 78, 254);
    /* 电量文字宽度130像素。 */
    lv_obj_set_width(s_view.battery.label, 130);
    /* 电量文字改用14号字体。 */
    lv_obj_set_style_text_font(s_view.battery.label, &lv_font_montserrat_14, 0);
    /* 电量文字水平居中。 */
    lv_obj_set_style_text_align(s_view.battery.label, LV_TEXT_ALIGN_CENTER, 0);
    /* 电量条重新挂到表盘body下。 */
    lv_obj_set_parent(s_view.battery.bar, s_body);
    /* 电量条放到文字左侧(34,257)。 */
    lv_obj_set_pos(s_view.battery.bar, 34, 257);
}

/* 表盘刷新：亮屏才读RTC，按秒值更新指针与摘要；未校时显示"--:--"并隐藏指针。
 * 同一秒内不重复取快照，熄屏直接返回。温湿度取AHT21快照，心率取最近有效BPM。 */
static void refresh_clock(void)
{
    /* RTC读出的时/分/秒结构。 */
    watch_time_struct time;
    /* AHT21温湿度快照。 */
    app_aht21_struct climate;
    /* 心率快照，取最近有效BPM。 */
    app_heart_struct heart;
    /* watch_rtc_get_time的HAL返回状态。 */
    HAL_StatusTypeDef status;
    /* 本次刷新时刻，单位LVGL tick(毫秒)。 */
    uint32_t now = lv_tick_get();
    /* 当天已过秒数，用于同秒判断。 */
    uint32_t seconds;
    /* 温度绝对值放大10倍后的整数。 */
    int32_t magnitude;
    /* 湿度放大10倍后的整数。 */
    int32_t humidity_x10;
    /* 温度为负时输出的符号，默认空串。 */
    const char *sign = "";
    /* 熄屏时直接返回，不做任何RTC读取或控件更新。 */
    if (s_screen_on == 0)
    {
        return; /* 熄屏不读取RTC、不更新指针；RTC硬件继续独立走时。 */
    }
    /* 读取当前RTC时间，结果同时用于判断是否已校时。 */
    status = watch_rtc_get_time(&time);
    /* RTC读取成功且已校时，才计算当天秒数并做同秒过滤。 */
    if (status == HAL_OK && watch_rtc_is_synced())
    {
        /* 时*3600+分*60+秒，得到当天0点起的秒数。 */
        seconds = (uint32_t)time.hours * 3600 + (uint32_t)time.minutes * 60 + time.seconds;
        /* 与上次刷新处在同一秒时不再重复刷新。 */
        if (seconds == s_view.watch.last_second)
        {
            return; /* 只读RTC秒值判断节拍；同一秒不重取摘要、不重绘指针。 */
        }
    }
    /* 尚未校时且上次也未校时，距上次刷新不足1000ms就返回，避免每轮轮询都刷摘要。 */
    else if (s_view.watch.last_second == UINT32_MAX && lv_tick_elaps(s_view.watch.refresh_tick) < 1000)
    {
        return;
    }
    /* 记录本次刷新tick，作为下一次节流的基准。 */
    s_view.watch.refresh_tick = now;
    /* 先刷新电量显示，与时钟摘要同节拍更新。 */
    refresh_battery();
    /* RTC读取失败或未校时，进入无时间显示分支。 */
    if (status != HAL_OK || !watch_rtc_is_synced())
    {
        /* 时间位置显示占位"--:--"。 */
        ui_set_label_text(s_view.watch.time, "--:--");
        /* 读取成功但未校时，提示通过蓝牙校时。 */
        if (status == HAL_OK)
        {
            ui_set_label_text(s_view.watch.hint, "SET VIA BLE");
        }
        /* 读取本身失败，提示RTC错误。 */
        else
        {
            ui_set_label_text(s_view.watch.hint, "RTC ERROR");
        }
        /* 指针当前未隐藏时，一次性把三根指针全部隐藏。 */
        if (!lv_obj_has_flag(s_view.watch.hour_hand, LV_OBJ_FLAG_HIDDEN))
        {
            /* 隐藏时针。 */
            lv_obj_add_flag(s_view.watch.hour_hand, LV_OBJ_FLAG_HIDDEN);
            /* 隐藏分针。 */
            lv_obj_add_flag(s_view.watch.minute_hand, LV_OBJ_FLAG_HIDDEN);
            /* 隐藏秒针。 */
            lv_obj_add_flag(s_view.watch.second_hand, LV_OBJ_FLAG_HIDDEN);
        }
        /* 用UINT32_MAX标记"当前无有效秒值"。 */
        s_view.watch.last_second = UINT32_MAX;
    }
    else
    {
        /* 已校时，提示时间源为RTC/LSE。 */
        ui_set_label_text(s_view.watch.hint, "RTC / LSE");
        /* 指针此前被隐藏时，把三根指针重新显示出来。 */
        if (lv_obj_has_flag(s_view.watch.hour_hand, LV_OBJ_FLAG_HIDDEN))
        {
            /* 显示时针。 */
            lv_obj_clear_flag(s_view.watch.hour_hand, LV_OBJ_FLAG_HIDDEN);
            /* 显示分针。 */
            lv_obj_clear_flag(s_view.watch.minute_hand, LV_OBJ_FLAG_HIDDEN);
            /* 显示秒针。 */
            lv_obj_clear_flag(s_view.watch.second_hand, LV_OBJ_FLAG_HIDDEN);
        }
        /* 重新计算当天秒数，作为本次刷新的时间基准。 */
        seconds = (uint32_t)time.hours * 3600 + (uint32_t)time.minutes * 60 + time.seconds;
        /* 秒值变化时才更新文本和指针。 */
        if (seconds != s_view.watch.last_second)
        {
            /* 记录新的秒值，下一轮据此判断是否同秒。 */
            s_view.watch.last_second = seconds;
            /* 时间文本格式化为HH:MM，两位补零。 */
            ui_set_label_text_fmt(s_view.watch.time, "%02u:%02u", (unsigned)time.hours, (unsigned)time.minutes);
            /* 时针角度: 12小时制*30度 + 分钟*0.5度 + 秒/120度(秒对时针的微调)。 */
            set_clock_hand(s_view.watch.hour_hand, s_hour_points, (time.hours % 12) * 30.0f + time.minutes * 0.5f + time.seconds / 120.0f, 44);
            /* 分针角度: 分钟*6度 + 秒*0.1度。 */
            set_clock_hand(s_view.watch.minute_hand, s_minute_points, time.minutes * 6.0f + time.seconds * 0.1f, 64);
            /* 秒针角度: 秒*6度，半径82像素。 */
            set_clock_hand(s_view.watch.second_hand, s_second_points, time.seconds * 6.0f, 82);
        }
    }
    /* 取心率模块最新快照。 */
    app_heart_get_snapshot(&heart);
    /* 最近有效BPM非0时显示数值。 */
    if (heart.last_bpm != 0)
    {
        /* 心率数值格式化为无符号十进制。 */
        ui_set_label_text_fmt(s_view.watch.heart, "%u", (unsigned)heart.last_bpm);
    }
    else
    {
        /* 没有有效心率时显示"--"。 */
        ui_set_label_text(s_view.watch.heart, "--");
    }
    /* 取AHT21温湿度快照。 */
    app_aht21_get_snapshot(&climate);
    /* 快照有效时把温度、湿度各放大10倍转成整数显示。 */
    if (climate.valid != 0)
    {
        /* 温度*10后取整，得到一位小数的定点值。 */
        magnitude = (int32_t)(climate.sample.temperature_c * 10.0f);
        /* 湿度*10后取整，同样保留一位小数。 */
        humidity_x10 = (int32_t)(climate.sample.humidity_rh * 10.0f);
        /* 温度为负时取绝对值，并记录负号。 */
        if (magnitude < 0)
        {
            /* 转成正数便于拆分整数位和小数位。 */
            magnitude = -magnitude;
            /* 显示时补负号。 */
            sign = "-";
        }
        /* 第一行温度(整数位/小数位)，第二行湿度，均按一位小数输出。 */
        ui_set_label_text_fmt(s_view.watch.temperature, "%s%ld.%ldC\n%ld.%ld%%", sign, (long)(magnitude / 10), (long)(magnitude % 10),
                           (long)(humidity_x10 / 10), (long)(humidity_x10 % 10));
    }
    else
    {
        /* 快照无效时两行都显示占位。 */
        ui_set_label_text(s_view.watch.temperature, "--C\n--%");
    }
}

static lv_obj_t *metric(const char *title, const char *unit, int y, uint32_t color)
{
    /* 在页面容器s_body的y位置建一张212x88卡片，作为单项指标的底框。 */
    lv_obj_t *obj = ui_panel(s_body, 0, y, 212, 88, UI_CARD);
    /* 数值标签句柄，创建后返回给刷新函数使用。 */
    lv_obj_t *value;
    /* 卡片左上角写指标标题，10号字体、灰色。 */
    ui_label(obj, title, 14, 12, &lv_font_montserrat_10, UI_MUTED);
    /* 卡片右上角写单位，颜色取本项指标的主题色。 */
    ui_label(obj, unit, 173, 12, &lv_font_montserrat_10, color);
    /* 卡片下部创建初值为"--"的数值标签，30号字体、主题色。 */
    value = ui_label(obj, "--", 14, 35, &lv_font_montserrat_30, color);
    /* 固定数值标签宽度184像素，长数字不会越出卡片。 */
    lv_obj_set_width(value, 184);
    /* 返回数值标签，后续刷新只改这个标签的文本。 */
    return value;
}

/* 磁力计校准开关：运动模式下拒绝开始；其余情况按当前状态取反提交给IMU模块。
 * 校准在sensor任务推进，本回调不访问I2C。 */
static void mag_calibrate_cb(lv_event_t *event)
{
    /* 本函数用的IMU快照，读取当前磁校准状态。 */
    app_imu_struct imu;
    (void)event;
    /* 切页动画未结束时忽略点击，避免在旧页控件上启动校准。 */
    if (s_transition)
    {
        return;
    }
    /* 取一次IMU快照；返回值不检查，快照结构体由模块填充。 */
    (void)App_IMU_GetSnapshot(&imu);
    /* 运动模式且校准未在运行时拒绝开始，只把提示写到状态标签。 */
    if (s_sport_mode != 0 && imu.mag_cal_state != APP_MAG_CAL_RUNNING)
    {
        ui_set_label_text(s_view.motion.cal_status, "Turn SPORT off to calibrate");
        return;
    }
    /* 未在运行时提交1启动，正在运行时提交0停止。 */
    App_IMU_MagCalibrate(imu.mag_cal_state != APP_MAG_CAL_RUNNING);
}

/* 同单位三轴共用一张卡片，减少合并页面同时驻留的控件。 */
static void motion_metrics(const char *const titles[3], const char *unit, int y, lv_obj_t *values[3])
{
    /* 三轴依次使用蓝/橙/绿，和标题、数值标签颜色对应。 */
    static const uint32_t colors[3] = {UI_BLUE, UI_ORANGE, UI_GREEN};
    /* 在s_body的y位置建一张212x288卡片，容纳三行指标。 */
    lv_obj_t *obj = ui_panel(s_body, 0, y, 212, 288, UI_CARD);
    /* 轴序号，0..2。 */
    uint8_t i;
    /* 卡片右上角统一标注单位，灰色10号字体。 */
    ui_label(obj, unit, 173, 12, &lv_font_montserrat_10, UI_MUTED);
    /* 逐轴创建标题和数值标签，每行间隔100像素。 */
    for (i = 0; i < 3; ++i)
    {
        /* 第i行顶部写轴名，纵向偏移12+100*i。 */
        ui_label(obj, titles[i], 14, 12 + i * 100, &lv_font_montserrat_10, UI_MUTED);
        /* 第i行下方建数值标签，初值"--"，30号字体，取对应轴颜色。 */
        values[i] = ui_label(obj, "--", 14, 35 + i * 100, &lv_font_montserrat_30, colors[i]);
        /* 固定宽度184像素，给三轴数值留同一列宽。 */
        lv_obj_set_width(values[i], 184);
    }
}

static void baro_history_update(const app_baro_struct *baro, uint8_t online)
{
    /* sensor维护的海拔历史快照。 */
    app_baro_history_struct history;
    /* 历史点下标。 */
    uint16_t i;
    /* 本点写入图表的纵坐标。 */
    lv_coord_t value;
    lv_coord_t latest = LV_CHART_POINT_NONE;
    /* NOW使用当前有效快照，校准和参考变化不必等待十分钟历史推进。 */
    if (online != 0 && baro->pressure_pa > 0 && isfinite(baro->alt_m) &&
        baro->alt_m >= -30000.0f && baro->alt_m <= 30000.0f)
    {
        latest = (lv_coord_t)roundf(baro->alt_m);
    }
    /* 取一次历史环形缓冲快照。 */
    app_baro_get_history(&history);
    /* 历史与当前柱都没有变化时不重写图表。 */
    if (s_view.baro.history_sequence == history.sequence &&
        lv_chart_get_y_array(s_view.baro.trend.chart, s_view.baro.trend.series)[APP_BARO_HISTORY_POINTS - 1] == latest)
    {
        return;
    }
    /* 记录已处理的序号，下次据此判断是否有新数据。 */
    s_view.baro.history_sequence = history.sequence;
    /* 逐点把历史海拔搬到图表对应柱形。 */
    for (i = 0; i < APP_BARO_HISTORY_POINTS; ++i)
    {
        /* 默认该点无数据。 */
        value = LV_CHART_POINT_NONE;
        /* 历史槽不是空标记时取实际海拔(米)。 */
        if (i == APP_BARO_HISTORY_POINTS - 1)
        {
            value = latest;
        }
        else if (history.altitude_m[i] != APP_BARO_HISTORY_NONE)
        {
            value = history.altitude_m[i];
        }
        /* 按槽位写入图表，无数据的槽位写"无点"哨兵。 */
        lv_chart_set_value_by_id(s_view.baro.trend.chart, s_view.baro.trend.series, i, value);
    }
    /* 按整窗海拔重算纵轴范围，保留负值和最小跨度。 */
    ui_trend_update_range(&s_view.baro.trend, 1);
}

static void compass_draw_cb(lv_event_t *event)
{
    /* 取本次绘制的上下文，所有圆弧、刻度和指针都画在这块目标上。 */
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    /* 罗盘控件在屏幕上的绝对区域，作为内部相对坐标的原点。 */
    lv_area_t area;
    /* 刻度内/外端点、圆心、指针尖端和左右两个侧点。 */
    lv_point_t inner, outer, center, tip, left, right, needle[3];
    /* 刻度线样式。 */
    lv_draw_line_dsc_t style;
    /* 内外圆环样式。 */
    lv_draw_arc_dsc_t ring;
    /* 指针三角面的填充样式。 */
    lv_draw_rect_dsc_t face;
    /* 当前刻度角度和半径。 */
    int16_t angle, radius;
    /* 当前角度的正弦、余弦，按LV_TRIGO_SIN_MAX归一化。 */
    int32_t sine, cosine;
    /* 取控件绝对坐标，圆心取area左上角加(106,64)。 */
    lv_obj_get_coords(lv_event_get_target(event), &area);
    center.x = area.x1 + 106;
    center.y = area.y1 + 64;
    /* 先用页面底色画一个120x120圆，作为表盘面遮住卡片背景。 */
    weather_icon_shape(context, &area, 46, 4, 120, 120, UI_BG);
    /* 初始化圆弧样式，外圈UI_BORDER色、2像素宽。 */
    lv_draw_arc_dsc_init(&ring);
    ring.color = lv_color_hex(UI_BORDER);
    ring.width = 2;
    /* 以圆心半径60画整圈外边框。 */
    lv_draw_arc(context, &ring, &center, 60, 0, 360);
    /* 内圈改成1像素宽，半径47，再画一整圈。 */
    ring.width = 1;
    lv_draw_arc(context, &ring, &center, 47, 0, 360);
    /* 初始化刻度线样式，后续每条刻度复用并改宽/色。 */
    lv_draw_line_dsc_init(&style);
    /* 每5度一条刻度，共72条。 */
    for (angle = 0; angle < 360; angle += 5)
    {
        /* 普通刻度从半径54处起画。 */
        radius = 54;
        style.width = 1;
        style.color = lv_color_hex(UI_BORDER);
        /* 每30度为主刻度：内端缩到50、线宽2、颜色变亮。 */
        if (angle % 30 == 0)
        {
            radius = 50;
            style.width = 2;
            style.color = lv_color_hex(UI_MUTED);
        }
        /* 0度为北向标记，颜色改为红色。 */
        if (angle == 0)
        {
            style.color = lv_color_hex(UI_RED);
        }
        /* 计算该角度方向的单位正弦/余弦。 */
        sine = lv_trigo_sin(angle);
        cosine = lv_trigo_cos(angle);
        /* 内端点沿径向放在radius处，屏幕y向下故余弦取负。 */
        inner.x = center.x + sine * radius / LV_TRIGO_SIN_MAX;
        inner.y = center.y - cosine * radius / LV_TRIGO_SIN_MAX;
        /* 外端点固定放在半径57处，形成短刻度线。 */
        outer.x = center.x + sine * 57 / LV_TRIGO_SIN_MAX;
        outer.y = center.y - cosine * 57 / LV_TRIGO_SIN_MAX;
        /* 画一条从内到外的刻度线。 */
        lv_draw_line(context, &style, &inner, &outer);
    }
    /* 航向无效(-1)时只保留表盘刻度，不画指针。 */
    if (s_view.motion.compass_heading < 0)
    {
        return;
    }
    /* 用当前航向角求指针方向的正弦/余弦。 */
    sine = lv_trigo_sin(s_view.motion.compass_heading);
    cosine = lv_trigo_cos(s_view.motion.compass_heading);
    /* 左右两个侧点位于与指针垂直的方向，距圆心8像素。 */
    left.x = center.x - cosine * 8 / LV_TRIGO_SIN_MAX;
    left.y = center.y - sine * 8 / LV_TRIGO_SIN_MAX;
    right.x = center.x + cosine * 8 / LV_TRIGO_SIN_MAX;
    right.y = center.y + sine * 8 / LV_TRIGO_SIN_MAX;
    /* 初始化三角面填充样式，颜色在下面按南北端分别设置。 */
    lv_draw_rect_dsc_init(&face);
    /* 三角形的第三个点固定为圆心。 */
    needle[2] = center;
    /* 四个三角面组成双色菱形指针，不增加位图或常驻控件。 */
    for (angle = 0; angle < 2; ++angle)
    {
        /* 第0轮画指向航向的北半段，用s_compass_color。 */
        if (angle == 0)
        {
            tip.x = center.x + sine * 30 / LV_TRIGO_SIN_MAX;
            tip.y = center.y - cosine * 30 / LV_TRIGO_SIN_MAX;
            face.bg_color = lv_color_hex(s_compass_color);
        }
        else
        {
            /* 第1轮画相反方向的南半段，长度24像素，用蓝色。 */
            tip.x = center.x - sine * 24 / LV_TRIGO_SIN_MAX;
            tip.y = center.y + cosine * 24 / LV_TRIGO_SIN_MAX;
            face.bg_color = lv_color_hex(UI_BLUE);
        }
        /* 先画圆心-尖端-左侧点组成的三角面。 */
        needle[0] = tip;
        needle[1] = left;
        lv_draw_triangle(context, &face, needle);
        /* 同色压暗30%后，用圆心-尖端-右侧点画另一半，形成明暗分界。 */
        face.bg_color = lv_color_darken(face.bg_color, LV_OPA_30);
        needle[1] = right;
        lv_draw_triangle(context, &face, needle);
    }
    /* 圆心先画8x8白色圆点，再叠4x4底色圆点，形成空心轴帽。 */
    weather_icon_shape(context, &area, 102, 60, 8, 8, UI_TEXT);
    weather_icon_shape(context, &area, 104, 62, 4, 4, UI_BG);
}

/* 运动模式开关：切换按钮文案/颜色后立即重算蓝牙供电与电源标志；
 * 采样模式由sensor任务下一轮响应，蓝牙供电在本回调调用app_ble_update_power时同步更新。 */
static void sport_mode_cb(lv_event_t *event)
{
    (void)event;
    /* 在0/1之间翻转运动模式标志，作为后续采样与蓝牙策略的输入。 */
    s_sport_mode = s_sport_mode == 0;
    if (s_sport_mode != 0)
    {
        /* 进入运动模式：按钮文案改成停止，底色改红。 */
        ui_set_label_text(s_view.sport.label, "STOP SPORT");
        lv_obj_set_style_bg_color(s_view.sport.button, lv_color_hex(UI_RED), 0);
    }
    else
    {
        /* 退出运动模式：按钮文案改回开始，底色改绿。 */
        ui_set_label_text(s_view.sport.label, "START SPORT");
        lv_obj_set_style_bg_color(s_view.sport.button, lv_color_hex(UI_GREEN), 0);
    }
    /* 按新的运动模式重算蓝牙供电策略。 */
    app_ble_update_power(s_screen_on, s_sport_mode, 0);
    /* 把新的电源相关标志发布给任务侧。 */
    publish_power_state();
}

static void sport_page(void)
{
    /* 页面顶部建212x96卡片，容纳标题和模式按钮。 */
    lv_obj_t *card = ui_panel(s_body, 0, 0, 212, 96, UI_CARD);
    /* 卡片左上角写页面标题。 */
    ui_label(card, "SPORT MODE", 14, 12, &lv_font_montserrat_14, UI_BLUE);
    /* 卡片下部建184x40绿色按钮底框，保存到全局供回调改色。 */
    s_view.sport.button = ui_panel(card, 14, 42, 184, 40, UI_GREEN);
    /* 按钮上创建文案标签，初始为START SPORT，颜色UI_TEXT。 */
    s_view.sport.label = ui_label(s_view.sport.button, "START SPORT", 0, 0, &lv_font_montserrat_14, UI_TEXT);
    /* 文案在按钮内居中。 */
    lv_obj_center(s_view.sport.label);
    /* 文案不接收点击，点击落到下面的按钮底框。 */
    lv_obj_clear_flag(s_view.sport.label, LV_OBJ_FLAG_CLICKABLE);
    /* 按钮底框可点击。 */
    lv_obj_add_flag(s_view.sport.button, LV_OBJ_FLAG_CLICKABLE);
    /* 页面重建时若运动模式已开，按当前状态显示停止文案和红色底色。 */
    if (s_sport_mode != 0)
    {
        ui_set_label_text(s_view.sport.label, "STOP SPORT");
        lv_obj_set_style_bg_color(s_view.sport.button, lv_color_hex(UI_RED), 0);
    }
    /* 注册点击回调，切换运动模式。 */
    lv_obj_add_event_cb(s_view.sport.button, sport_mode_cb, LV_EVENT_CLICKED, NULL);
    /* 两条说明文字，分别说明连续采样和蓝牙/气压计暂停。 */
    ui_note(s_body, "Continuous motion and heart rate, even with screen off.", 112);
    ui_note(s_body, "Bluetooth and barometer pause until sport mode is off.", 174);
}

static void compass_create(int y)
{
    /* 在s_body的y位置建212x174卡片，作为罗盘容器并接收自绘回调。 */
    s_view.motion.compass = ui_panel(s_body, 0, y, 212, 174, UI_CARD);
    /* 按上北下南、左西右东放置四个方位字母，北向用红色。 */
    ui_label(s_view.motion.compass, "N", 102, 22, &lv_font_montserrat_10, UI_RED);
    ui_label(s_view.motion.compass, "E", 142, 59, &lv_font_montserrat_10, UI_TEXT);
    ui_label(s_view.motion.compass, "S", 102, 102, &lv_font_montserrat_10, UI_TEXT);
    ui_label(s_view.motion.compass, "W", 61, 59, &lv_font_montserrat_10, UI_TEXT);
    /* 罗盘下方建航向数值标签，初值"--"。 */
    s_view.motion.compass_value = ui_label(s_view.motion.compass, "--", 14, 122, &lv_font_montserrat_30, UI_TEXT);
    /* 数值标签宽184像素并水平居中。 */
    lv_obj_set_width(s_view.motion.compass_value, 184);
    lv_obj_set_style_text_align(s_view.motion.compass_value, LV_TEXT_ALIGN_CENTER, 0);
    /* 数值下方建状态标签，初值NO IMU DATA，同样居中。 */
    s_view.motion.compass_status = ui_label(s_view.motion.compass, "NO IMU DATA", 14, 156, &lv_font_montserrat_14, UI_MUTED);
    lv_obj_set_width(s_view.motion.compass_status, 184);
    lv_obj_set_style_text_align(s_view.motion.compass_status, LV_TEXT_ALIGN_CENTER, 0);
    /* 在LV_EVENT_DRAW_MAIN阶段调用compass_draw_cb绘制表盘和指针。 */
    lv_obj_add_event_cb(s_view.motion.compass, compass_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
}

static void compass_invalidate_needle(int16_t previous_heading, int16_t heading)
{
    /* 待刷新的脏区域和罗盘控件坐标。 */
    lv_area_t area, coords;
    /* 圆心和指针四个端点。 */
    lv_point_t center, points[4];
    /* 旧、新两个航向，分别用于算需要重绘的区域。 */
    int16_t headings[2] = {previous_heading, heading};
    /* 当前航向的正弦/余弦。 */
    int32_t sine, cosine;
    /* 航向下标和端点下标。 */
    uint8_t i, j;

    /* 取罗盘控件绝对坐标，圆心与绘制回调一致为左上角加(106,64)。 */
    lv_obj_get_coords(s_view.motion.compass, &coords);
    center.x = coords.x1 + 106;
    center.y = coords.y1 + 64;
    /* 脏区域先取圆心周围9x9，覆盖中心圆点。 */
    lv_area_set(&area, center.x - 4, center.y - 4, center.x + 4, center.y + 4);
    /* 对旧、新两个航向各算一次指针包围盒。 */
    for (i = 0; i < 2; ++i)
    {
        /* 航向为-1表示无指针，跳过。 */
        if (headings[i] < 0)
        {
            continue;
        }
        /* 求该航向的三角函数值。 */
        sine = lv_trigo_sin(headings[i]);
        cosine = lv_trigo_cos(headings[i]);
        /* 北向尖端，长30像素。 */
        points[0].x = center.x + sine * 30 / LV_TRIGO_SIN_MAX;
        points[0].y = center.y - cosine * 30 / LV_TRIGO_SIN_MAX;
        /* 南向尾端，长24像素。 */
        points[1].x = center.x - sine * 24 / LV_TRIGO_SIN_MAX;
        points[1].y = center.y + cosine * 24 / LV_TRIGO_SIN_MAX;
        /* 与指针垂直的左右侧点，各距圆心8像素。 */
        points[2].x = center.x - cosine * 8 / LV_TRIGO_SIN_MAX;
        points[2].y = center.y - sine * 8 / LV_TRIGO_SIN_MAX;
        points[3].x = center.x + cosine * 8 / LV_TRIGO_SIN_MAX;
        points[3].y = center.y + sine * 8 / LV_TRIGO_SIN_MAX;
        /* 把四个端点并入脏区域包围盒。 */
        for (j = 0; j < 4; ++j)
        {
            area.x1 = LV_MIN(area.x1, points[j].x);
            area.y1 = LV_MIN(area.y1, points[j].y);
            area.x2 = LV_MAX(area.x2, points[j].x);
            area.y2 = LV_MAX(area.y2, points[j].y);
        }
    }
    /* 合并旧/新指针及中心圆点，另留2像素覆盖三角形边缘的抗锯齿。 */
    area.x1 -= 2;
    area.y1 -= 2;
    area.x2 += 2;
    area.y2 += 2;
    /* 只把这块区域标记为需要重绘，避免整页刷新。 */
    lv_obj_invalidate_area(s_view.motion.compass, &area);
}

static void compass_update(const app_imu_struct *imu, uint8_t online)
{
    /* 8个45度扇区对应的方位缩写，按顺时针排列。 */
    static const char *const directions[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    /* 本帧最终显示的整数航向，-1表示无有效航向。 */
    int16_t heading = -1;
    /* 指针颜色，默认灰色表示仅陀螺。 */
    uint32_t color = UI_MUTED;
    /* 状态标签颜色，默认灰色。 */
    uint32_t status_color = UI_MUTED;
    /* 本轮刷新时刻，LVGL毫秒tick。 */
    uint32_t now = lv_tick_get();
    /* 距上次刷新的毫秒数，上限UI_REFRESH_MS，用于换算本轮滤波步长和限速。 */
    uint32_t elapsed = LV_MIN((uint32_t)(now - s_view.motion.compass_tick), UI_REFRESH_MS);
    /* 目标角度、最短角差、本轮步长、步长限幅。 */
    float degrees, delta, step, limit;
    /* 在线且yaw是有限数时才更新航向。 */
    if (online && isfinite(imu->yaw_deg))
    {
        /* 世界+Z向上：右手yaw取反得到顺时针磁航向，参考方向为MPU的+X轴。 */
        degrees = fmodf(-imu->yaw_deg, 360.0f);
        /* fmodf可能得到负数，加360归一化到[0,360)。 */
        if (degrees < 0.0f)
        {
            degrees += 360.0f;
        }
        /* 还没有显示过航向时直接采用目标角，不做滤波。 */
        if (s_view.motion.compass_heading < 0)
        {
            s_compass_display_deg = degrees;
        }
        else
        {
            /* 跨北点走最短角差；100ms低通并限速360°/s，停屏后也不积攒一次大跳。 */
            delta = fmodf(degrees - s_compass_display_deg + 540.0f, 360.0f) - 180.0f;
            /* 一阶低通：本轮只走与elapsed/(100+elapsed)成比例的一段。 */
            step = delta * (float)elapsed / (UI_COMPASS_FILTER_MS + (float)elapsed);
            /* 按360°/s与elapsed换算出本轮允许的最大角度变化。 */
            limit = UI_COMPASS_MAX_DPS * (float)elapsed / 1000.0f;
            /* 步长限制在±limit内。 */
            step = LV_CLAMP(-limit, step, limit);
            /* 叠加步长后加360再取模，保持显示角在[0,360)。 */
            s_compass_display_deg = fmodf(s_compass_display_deg + step + 360.0f, 360.0f);
        }
        /* 四舍五入成整数度，并对360取模。 */
        heading = (int16_t)(s_compass_display_deg + 0.5f) % 360;
        /* 显示三位角度加度符号，扇区下标按(heading+22)/45取整再模8。 */
        ui_set_label_text_fmt(s_view.motion.compass_value, "%03d\xC2\xB0 %s", (int)heading, directions[((heading + 22) / 45) % 8]);
        /* 磁融合确实在参与姿态解算时才讨论磁北。 */
        if (imu->mag_fused && imu->mag_fusion_state == APP_MAG_FUSION_ACTIVE)
        {
            if (imu->mag_calibrated)
            {
                /* 已校准：指针红色，状态文字绿色MAG NORTH。 */
                color = UI_RED;
                status_color = UI_GREEN;
                ui_set_label_text(s_view.motion.compass_status, "MAG NORTH");
            }
            else
            {
                /* 未校准：指针和状态都用橙色提示UNCALIBRATED。 */
                color = UI_ORANGE;
                status_color = UI_ORANGE;
                ui_set_label_text(s_view.motion.compass_status, "UNCALIBRATED");
            }
        }
        else
        {
            /* 没有磁融合时，若正在校准则提示CALIBRATING，否则只是陀螺航向。 */
            if (imu->mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                status_color = UI_ORANGE;
                ui_set_label_text(s_view.motion.compass_status, "CALIBRATING");
            }
            else
            {
                ui_set_label_text(s_view.motion.compass_status, "GYRO ONLY");
            }
        }
    }
    else
    {
        /* 离线或yaw无效：数值和状态都显示占位，状态用红色。 */
        ui_set_label_text(s_view.motion.compass_value, "--");
        ui_set_label_text(s_view.motion.compass_status, "NO IMU DATA");
        status_color = UI_RED;
    }
    /* 应用本轮算出的状态文字颜色。 */
    ui_set_text_color(s_view.motion.compass_status, status_color);
    /* 记录本轮刷新时刻，供下一轮elapsed计算。 */
    s_view.motion.compass_tick = now;
    /* 航向或指针颜色变化时才标记局部重绘，并同步显示状态。 */
    if (heading != s_view.motion.compass_heading || color != s_compass_color)
    {
        /* 用旧航向和新航向的并集作为脏区。 */
        compass_invalidate_needle(s_view.motion.compass_heading, heading);
        s_view.motion.compass_heading = heading;
        s_compass_color = color;
    }
}

static void mag_calibration_open_cb(lv_event_t *event)
{
    (void)event;
    /* 非切页动画期间才滚动到磁校准卡片。 */
    if (!s_transition)
    {
        /* 以校准状态标签的父对象为目标，无动画滚动到可见位置。 */
        lv_obj_scroll_to_view(lv_obj_get_parent(s_view.motion.cal_label), LV_ANIM_OFF);
    }
}

static void imu_page(void)
{
    int y = 0;
    /* 角度三轴标题。 */
    static const char *const angles[3] = {"PITCH", "ROLL", "YAW"};
    /* 磁力计三轴标题。 */
    static const char *const magnetic[3] = {"MAG X", "MAG Y", "MAG Z"};
    /* 临时控件句柄。 */
    lv_obj_t *obj;
    /* 在页面顶部建罗盘卡片。 */
    compass_create(y);
    /* 罗盘下方建一条212x36的可点击条，作为校准入口。 */
    obj = ui_panel(s_body, 0, y + 182, 212, 36, UI_BORDER);
    /* 条内写校准入口文字和右箭头。 */
    s_view.motion.compass_calibration = ui_label(obj, "CALIBRATION  " LV_SYMBOL_RIGHT, 14, 11, &lv_font_montserrat_14, UI_TEXT);
    /* 文字本身不接收点击，点击落到条目底框。 */
    lv_obj_clear_flag(s_view.motion.compass_calibration, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    /* 点击条目滚动到下方的磁校准区。 */
    lv_obj_add_event_cb(obj, mag_calibration_open_cb, LV_EVENT_CLICKED, NULL);
    /* y推进228像素，开始角度区。 */
    y += 228;
    /* 角度区顶部建IMU状态标签，初值WAITING。 */
    s_view.motion.status = ui_label(s_body, "WAITING", 2, y + 4, &lv_font_montserrat_10, UI_MUTED);
    /* 建三轴角度卡片，值标签存到s_view.motion.angles，单位DEG。 */
    motion_metrics(angles, "DEG", y + 26, s_view.motion.angles);
    /* 角度卡片下方写航向来源说明。 */
    s_view.motion.yaw_source = ui_note(s_body, "GYRO ONLY / heading may drift", y + 326);
    /* 磁力计状态标签，初值MAG WAITING。 */
    s_view.motion.mag_status = ui_label(s_body, "MAG WAITING", 2, y + 360, &lv_font_montserrat_10, UI_MUTED);
    /* 三轴磁力计卡片，值标签存到s_view.motion.mag_values，单位uT。 */
    motion_metrics(magnetic, "uT", y + 384, s_view.motion.mag_values);
    /* 磁校准区建212x48可点击条。 */
    obj = ui_panel(s_body, 0, y + 684, 212, 48, UI_BORDER);
    /* 条内写CALIBRATE MAG文字。 */
    s_view.motion.cal_label = ui_label(obj, "CALIBRATE MAG", 14, 16, &lv_font_montserrat_14, UI_TEXT);
    /* 文字不接收点击，底框可点击。 */
    lv_obj_clear_flag(s_view.motion.cal_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    /* 点击底框提交磁校准开关。 */
    lv_obj_add_event_cb(obj, mag_calibrate_cb, LV_EVENT_CLICKED, NULL);
    /* 校准条下方建校准状态标签，初值CAL NOT RUN。 */
    s_view.motion.cal_status = ui_note(s_body, "CAL NOT RUN", y + 744);
    /* 最后写一段校准操作说明。 */
    ui_note(s_body, "Rotate every axis for 30-60s, then tap FINISH. EEPROM keeps successful calibration.", y + 790);
}

/* 校准输入只提交请求，下一轮sensor窗口取得新气压后完成。 */
static void baro_calibration_cb(lv_event_t *event)
{
    uintptr_t action = (uintptr_t)lv_event_get_user_data(event);
    if (s_transition)
    {
        return;
    }
    /* 0="-"步减，1="+"步增，调整spinbox当前选中位。 */
    if (action == 0)
    {
        lv_spinbox_decrement(s_view.baro.altitude_input);
    }
    else if (action == 1)
    {
        lv_spinbox_increment(s_view.baro.altitude_input);
    }
    /* 2="CALIBRATE"：运动模式禁止校准；先检查参考气压是否已同步，受理后进入等待态。 */
    else if (action == 2)
    {
        if (s_sport_mode != 0)
        {
            ui_set_label_text(s_view.baro.calibration_status, "EXIT SPORT TO CALIBRATE");
            return;
        }
        if (app_baro_calibrate_altitude((int16_t)lv_spinbox_get_value(s_view.baro.altitude_input)) != 0)
        {
            ui_set_label_text(s_view.baro.calibration_status, "SYNC PHONE TO CALIBRATE");
            return;
        }
        s_view.baro.calibration_waiting = 1;
        ui_set_label_text(s_view.baro.calibration_status, "CALIBRATING...");
    }
    /* 3="RESET"：清除本次开机的校准偏差与等待状态。 */
    else
    {
        app_baro_reset_calibration();
        s_view.baro.calibration_waiting = 0;
    }
    /* 回拨刷新计时，使下轮UI刷新立即更新状态文字与海拔。 */
    s_view.baro.refresh_tick = lv_tick_get() - UI_BARO_REFRESH_MS;
}

static void baro_page(void)
{
    int y = 0;
    static const struct
    {
        const char *text;
        lv_coord_t x, y, width;
    } controls[] = {{"-", 8, 27, 32}, {"+", 172, 27, 32}, {"CALIBRATE", 8, 76, 120}, {"RESET", 136, 76, 68}};
    lv_obj_t *control;
    app_baro_struct baro;
    uint8_t i;
    /* 在s_body的y位置建212x94卡片，容纳海拔和气压。 */
    lv_obj_t *card = ui_panel(s_body, 0, y, 212, 94, UI_CARD);
    /* 卡片左上角写ALTITUDE标题。 */
    ui_label(card, "ALTITUDE", 12, 10, &lv_font_montserrat_10, UI_BLUE);
    /* 右上角建气压计状态标签，初值WAITING，宽60右对齐。 */
    s_view.baro.status = ui_label(card, "WAITING", 140, 10, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_view.baro.status, 60);
    lv_obj_set_style_text_align(s_view.baro.status, LV_TEXT_ALIGN_RIGHT, 0);
    /* 标题下方建海拔数值标签，初值"--"。 */
    s_view.baro.altitude = ui_label(card, "--", 12, 27, &lv_font_montserrat_30, UI_TEXT);
    lv_obj_set_width(s_view.baro.altitude, 168);
    /* 数值右侧标单位m。 */
    ui_label(card, "m", 184, 40, &lv_font_montserrat_14, UI_MUTED);
    /* 画一条188x1分隔线。 */
    ui_panel(card, 12, 64, 188, 1, UI_BORDER);
    /* 分隔线下方写PRESSURE标题。 */
    ui_label(card, "PRESSURE", 12, 75, &lv_font_montserrat_10, UI_MUTED);
    /* 气压数值标签，初值"--"。 */
    s_view.baro.pressure = ui_label(card, "--", 92, 72, &lv_font_montserrat_14, UI_TEXT);
    /* 右侧标单位hPa。 */
    ui_label(card, "hPa", 182, 75, &lv_font_montserrat_10, UI_MUTED);
    /* 卡片下方建海拔趋势图，采样周期取气压历史周期APP_BARO_HISTORY_PERIOD_MS。 */
    ui_trend_create(&s_view.baro.trend, s_body, "ALTITUDE TREND / m", y + 102, 116, UI_BLUE, APP_BARO_HISTORY_PERIOD_MS);
    /* 卡片下方建已知海拔校准区(-1000..9000m输入、+/-、CALIBRATE、RESET与状态提示)。 */
    card = ui_panel(s_body, 0, y + 232, 212, 148, UI_CARD);
    ui_label(card, "KNOWN ALTITUDE / m", 12, 9, &lv_font_montserrat_10, UI_MUTED);
    s_view.baro.altitude_input = lv_spinbox_create(card);
    lv_obj_set_pos(s_view.baro.altitude_input, 48, 27);
    lv_obj_set_size(s_view.baro.altitude_input, 116, 38);
    lv_spinbox_set_digit_format(s_view.baro.altitude_input, 4, 0);
    lv_spinbox_set_range(s_view.baro.altitude_input, -1000, 9000);
    lv_spinbox_set_step(s_view.baro.altitude_input, 1);
    lv_obj_set_style_text_font(s_view.baro.altitude_input, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_view.baro.altitude_input, lv_color_hex(UI_TEXT), 0);
    lv_obj_set_style_bg_color(s_view.baro.altitude_input, lv_color_hex(UI_BG), 0);
    lv_obj_set_style_border_color(s_view.baro.altitude_input, lv_color_hex(UI_BORDER), 0);
    lv_obj_set_style_pad_all(s_view.baro.altitude_input, 10, 0);
    (void)App_Baro_GetSnapshot(&baro);
    /* 若当前已有有效海拔估算，则作为输入框初始值。 */
    if (isfinite(baro.alt_m))
    {
        lv_spinbox_set_value(s_view.baro.altitude_input, (int32_t)roundf(baro.alt_m));
    }
    for (i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
    {
        control = ui_label(card, controls[i].text, controls[i].x, controls[i].y, &lv_font_montserrat_14, UI_TEXT);
        lv_obj_set_size(control, controls[i].width, 32);
        lv_obj_set_style_bg_color(control, lv_color_hex(UI_BORDER), 0);
        lv_obj_set_style_bg_opa(control, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(control, 8, 0);
        lv_obj_set_style_pad_top(control, 8, 0);
        lv_obj_set_style_text_align(control, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(control, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(control, baro_calibration_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        /* +/- 按钮支持长按连续增减。 */
        if (i < 2)
        {
            lv_obj_add_event_cb(control, baro_calibration_cb, LV_EVENT_LONG_PRESSED_REPEAT, (void *)(uintptr_t)i);
        }
    }
    s_view.baro.calibration_status = ui_label(card, "SYNC PHONE TO CALIBRATE", 8, 122, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_view.baro.calibration_status, 196);
    ui_note(s_body, "Tap a digit, then use +/- to set known altitude. Calibration resets at restart.", y + 394);
    ui_note(s_body, "Fresh phone temperature corrects altitude. SPL006 / sample every 5 min.", y + 458);
}

/* 亮度滑条：值域UI_MIN_BRIGHTNESS..100(百分比)，熄屏时只记住值不点灯。 */
static void brightness_cb(lv_event_t *event)
{
    /* 取滑条当前值(百分比)，截断成8位存入s_brightness。 */
    s_brightness = (uint8_t)lv_slider_get_value(lv_event_get_target(event));
    /* 亮屏时按新亮度设置背光。 */
    if (s_screen_on)
    {
        LCD_Set_Light(s_brightness);
    }
    else
    {
        /* 熄屏时只记住值，背光保持0。 */
        LCD_Set_Light(0);
    }
    /* 在滑条旁显示新亮度百分比。 */
    ui_set_label_text_fmt(s_view.display.brightness, "%d%%", (int)s_brightness);
}

/* 服务只提供业务状态，蓝牙页面保有文案和配色。 */
typedef struct
{
    const char *text;
    uint32_t color;
} ui_bluetooth_tx_style_struct;

static const ui_bluetooth_tx_style_struct s_bluetooth_tx_styles[] =
{
    [APP_BLE_TX_READY] = {"TX READY", UI_MUTED},
    [APP_BLE_TX_UART_OK] = {"TX UART OK", UI_GREEN},
    [APP_BLE_TX_BUSY] = {"TX BUSY", UI_ORANGE},
    [APP_BLE_TX_TIMEOUT] = {"TX TIMEOUT", UI_ORANGE},
    [APP_BLE_TX_ERROR] = {"TX ERROR", UI_RED},
    [APP_BLE_TX_UPTIME_OK] = {"TX UPTIME UART OK", UI_GREEN},
    [APP_BLE_TX_UPTIME_ERROR] = {"TX UPTIME ERROR", UI_RED},
    [APP_BLE_TX_HEART_OK] = {"TX HEART OK", UI_GREEN},
    [APP_BLE_TX_HEART_ERROR] = {"TX HEART ERROR", UI_RED}
};

static void bluetooth_clear_cb(lv_event_t *event)
{
    (void)event;
    if (s_transition)
    {
        return;
    }
    app_ble_clear_log();
    refresh_bluetooth();
}

/* 切页期间不提交测试发送；UART结果由服务快照提供。 */
static void bluetooth_send_test_cb(lv_event_t *event)
{
    (void)event;
    if (APP_DEBUG_TELEMETRY == 0 || s_transition)
    {
        return;
    }
    app_ble_send_test();
    refresh_bluetooth();
}

/* ECO/保连选择交给服务维护，页面只刷新策略与供电快照。 */
static void bluetooth_power_cb(lv_event_t *event)
{
    app_ble_snapshot_struct snapshot;
    (void)event;
    app_ble_get_snapshot(&snapshot);
    app_ble_set_eco(snapshot.eco == 0);
    app_ble_update_power(s_screen_on, s_sport_mode, 0);
    refresh_bluetooth();
}

static void bluetooth_page(void)
{
    /* 页面上的按钮底框和按钮文字句柄。 */
    lv_obj_t *button, *caption;
    app_ble_snapshot_struct snapshot;
    app_ble_get_snapshot(&snapshot);
    /* 顶部建蓝牙初始化状态标签，初值INIT PENDING。 */
    s_view.bluetooth.status = ui_label(s_body, "INIT PENDING", 4, 4, &lv_font_montserrat_10, UI_MUTED);
    /* 状态下方显示蓝牙模块名KT6368A_BLE_NAME。 */
    ui_note(s_body, snapshot.device_name, 22);
    /* 第二行左侧建RX字节计数标签，初值RX 0 B，蓝色。 */
    s_view.bluetooth.count = ui_label(s_body, "RX 0 B", 4, 44, &lv_font_montserrat_10, UI_BLUE);
    /* 第二行右侧建TX结果显示标签，文本和颜色取当前保存值。 */
    s_view.bluetooth.tx_status = ui_label(s_body, s_bluetooth_tx_styles[snapshot.tx_state].text, 84, 44, &lv_font_montserrat_10,
                                          s_bluetooth_tx_styles[snapshot.tx_state].color);
    /* 建124x28蓝色SEND TEST按钮底框。 */
    button = ui_panel(s_body, 0, 62, 124, 28, UI_BLUE);
    /* 按钮内写SEND TEST文字并居中。 */
    caption = ui_label(button, "SEND TEST", 0, 0, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_center(caption);
    /* 按钮底框可点击，点击触发一次测试发送。 */
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button, bluetooth_send_test_cb, LV_EVENT_CLICKED, NULL);
    /* 调试遥测未编译时把按钮置为禁用态。 */
    if (APP_DEBUG_TELEMETRY == 0)
    {
        lv_obj_add_state(button, LV_STATE_DISABLED);
    }
    /* 右侧建76x28的CLEAR按钮底框。 */
    button = ui_panel(s_body, 136, 62, 76, 28, UI_BORDER);
    /* 按钮内写CLEAR文字并居中。 */
    caption = ui_label(button, "CLEAR", 0, 0, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_center(caption);
    /* 按钮可点击，点击清空接收显示历史。 */
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button, bluetooth_clear_cb, LV_EVENT_CLICKED, NULL);
    /* 建212x94日志框，作为可滚动容器保存接收文本。 */
    s_view.bluetooth.log_box = ui_panel(s_body, 0, 98, 212, 94, UI_CARD);
    /* 日志框允许滚动，方向为垂直。 */
    lv_obj_add_flag(s_view.bluetooth.log_box, LV_OBJ_FLAG_SCROLLABLE);
    /* 关闭弹性回弹和滚动链，滚到底不带动外层页面。 */
    lv_obj_clear_flag(s_view.bluetooth.log_box, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_view.bluetooth.log_box, LV_DIR_VER);
    /* 需要时自动显示滚动条，底部留10像素内边距。 */
    lv_obj_set_scrollbar_mode(s_view.bluetooth.log_box, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_bottom(s_view.bluetooth.log_box, 10, 0);
    /* 滚动条用蓝色、60%不透明度、2像素宽。 */
    lv_obj_set_style_bg_color(s_view.bluetooth.log_box, lv_color_hex(UI_BLUE), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(s_view.bluetooth.log_box, LV_OPA_60, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(s_view.bluetooth.log_box, 2, LV_PART_SCROLLBAR);
    /* 日志框内建文本标签，初值Waiting for data...。 */
    s_view.bluetooth.log = ui_label(s_view.bluetooth.log_box, "Waiting for data...", 10, 10, &lv_font_montserrat_14, UI_TEXT);
    /* 标签宽188像素，按换行模式自动折行，行距3像素。 */
    lv_obj_set_width(s_view.bluetooth.log, 188);
    lv_label_set_long_mode(s_view.bluetooth.log, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_line_space(s_view.bluetooth.log, 3, 0);
    /* 日志框下方建校时状态标签，初值提示发送HH:MM:SS。 */
    s_view.bluetooth.clock_status = ui_label(s_body, "Send HH:MM:SS to set time", 4, 200, &lv_font_montserrat_10, UI_MUTED);
    /* 建212x36的BLE电源策略按钮底框。 */
    button = ui_panel(s_body, 0, 228, 212, 36, UI_BORDER);
    /* 按钮内写BLE: KEEP CONNECTED并居中。 */
    s_view.bluetooth.power_label = ui_label(button, "BLE: KEEP CONNECTED", 0, 0, &lv_font_montserrat_10, UI_TEXT);
    lv_obj_center(s_view.bluetooth.power_label);
    /* 文字不接收点击，点击落到按钮底框，切换ECO/常连。 */
    lv_obj_clear_flag(s_view.bluetooth.power_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(button, bluetooth_power_cb, LV_EVENT_CLICKED, NULL);
    /* 两条ECO模式说明。 */
    ui_note(s_body, "ECO disconnects after screen off.", 274);
    ui_note(s_body, "Wake to reconnect. Resets on reboot.", 292);
    /* 页面重建后强制刷新一次日志内容。 */
    s_view.bluetooth.log_loaded = 0;
}

static void refresh_bluetooth(void)
{
    app_ble_snapshot_struct snapshot;
    /* 状态标签要显示的文本，默认初始化完成。 */
    const char *text = "INIT OK / BLE ONLY";
    /* 状态标签颜色，默认灰色。 */
    uint32_t color = UI_MUTED;
    /* 蓝牙页控件未创建(不在本页)时直接返回。 */
    if (s_view.bluetooth.status == NULL)
    {
        return;
    }
    app_ble_get_snapshot(&snapshot);
    /* 初始化结果不等于手机连接状态，KT_LED 没有接入 STM32。 */
    if (s_sport_mode != 0)
    {
        /* 运动模式强制关闭蓝牙，优先显示该状态。 */
        text = "OFF / SPORT MODE";
    }
    else if (snapshot.enabled == 0)
    {
        /* ECO策略已断电时显示ECO关闭。 */
        text = "OFF / ECO MODE";
    }
    else if (snapshot.init_result != APP_BLE_INIT_OK)
    {
        /* 初始化失败按HAL状态细分超时/忙/其他错误，文字统一用橙色。 */
        if (snapshot.init_result == APP_BLE_INIT_TIMEOUT)
        {
            text = "INIT TIMEOUT";
        }
        else if (snapshot.init_result == APP_BLE_INIT_BUSY)
        {
            text = "INIT BUSY";
        }
        else
        {
            text = "INIT ERROR";
        }
        color = UI_ORANGE;
    }
    /* 写入状态文字并应用颜色。 */
    ui_set_label_text(s_view.bluetooth.status, text);
    /* ECO开关打开时按钮显示熄屏断开，否则显示保持连接。 */
    if (snapshot.eco != 0)
    {
        ui_set_label_text(s_view.bluetooth.power_label, "BLE: ECO / SCREEN OFF");
    }
    else
    {
        ui_set_label_text(s_view.bluetooth.power_label, "BLE: KEEP CONNECTED");
    }
    ui_set_text_color(s_view.bluetooth.status, color);
    /* 刷新RX字节计数和最近一次TX结果。 */
    ui_set_label_text_fmt(s_view.bluetooth.count, "RX %lu B", (unsigned long)snapshot.rx_bytes);
    ui_set_label_text(s_view.bluetooth.tx_status, s_bluetooth_tx_styles[snapshot.tx_state].text);
    ui_set_text_color(s_view.bluetooth.tx_status, s_bluetooth_tx_styles[snapshot.tx_state].color);
    /* 按时钟模块最近一次解析结果显示校时状态。 */
    switch (snapshot.clock_result)
    {
    case APP_CLOCK_SYNCED:
        /* 校时成功：绿色TIME SYNCED。 */
        ui_set_label_text(s_view.bluetooth.clock_status, "TIME SYNCED");
        ui_set_text_color(s_view.bluetooth.clock_status, UI_GREEN);
        break;
    case APP_CLOCK_INVALID:
        /* 时间字段非法：红色TIME INVALID。 */
        ui_set_label_text(s_view.bluetooth.clock_status, "TIME INVALID");
        ui_set_text_color(s_view.bluetooth.clock_status, UI_RED);
        break;
    case APP_CLOCK_RTC_ERROR:
        /* 写RTC失败：红色RTC ERROR。 */
        ui_set_label_text(s_view.bluetooth.clock_status, "RTC ERROR");
        ui_set_text_color(s_view.bluetooth.clock_status, UI_RED);
        break;
    default:
        /* 尚未收到有效校时消息，保留操作提示。 */
        ui_set_label_text(s_view.bluetooth.clock_status, "Send HH:MM:SS to set time");
        break;
    }
    /* 接收文本有更新时才重写日志标签。 */
    if (s_view.bluetooth.log_loaded == 0 || s_view.bluetooth.log_revision != snapshot.log_revision)
    {
        if (snapshot.log_length)
        {
            /* 有内容就显示累积的接收文本。 */
            ui_set_label_text(s_view.bluetooth.log, snapshot.log);
        }
        else
        {
            /* 内容为空时显示等待提示。 */
            ui_set_label_text(s_view.bluetooth.log, "Waiting for data...");
        }
        /* 先让标签按新文本完成布局，再把日志框滚到最底部。 */
        lv_obj_update_layout(s_view.bluetooth.log_box);
        lv_obj_scroll_to_y(s_view.bluetooth.log_box, lv_obj_get_scroll_y(s_view.bluetooth.log_box) + lv_obj_get_scroll_bottom(s_view.bluetooth.log_box), LV_ANIM_OFF);
        /* 仅记录页面已显示的版本，不修改服务状态。 */
        s_view.bluetooth.log_revision = snapshot.log_revision;
        s_view.bluetooth.log_loaded = 1;
    }
}

/* 手动心率开关：按当前快照取反提交，运动模式下禁用；真正的采样由sensor任务执行。 */
static void heart_toggle_cb(lv_event_t *event)
{
    /* 当前心率快照，用于读取enabled状态。 */
    app_heart_struct heart;
    (void)event;
    /* 切页中或运动模式(由sensor强制连续测量)时不接受手动启停。 */
    if (s_transition || s_sport_mode != 0)
    {
        return;
    }
    /* 取当前心率状态快照。 */
    app_heart_get_snapshot(&heart);
    /* 按当前开关取反提交，真正采样由sensor任务执行。 */
    app_heart_set_enabled(!heart.enabled);
    /* 立即刷新页面，让按钮与状态跟上新请求。 */
    refresh_data();
}

static void heart_auto_cb(lv_event_t *event)
{
    (void)event;
    /* 切页动画期间忽略点击。 */
    if (s_transition)
    {
        return;
    }
    /* 后台自动心率独立开关：只提交请求，UI 不读I2C；不影响手动 START/STOP 与 SPORT。 */
    app_heart_set_auto_enabled(app_heart_get_auto_enabled() == 0);
    /* 提交后刷新一次页面显示。 */
    refresh_data();
}

static void heart_details_cb(lv_event_t *event)
{
    (void)event;
    /* 切页动画期间忽略点击。 */
    if (s_transition)
    {
        return;
    }
    /* 详情卡当前隐藏时展开：清隐藏标志，标题箭头改为向上。 */
    if (lv_obj_has_flag(s_view.heart.details, LV_OBJ_FLAG_HIDDEN))
    {
        lv_obj_clear_flag(s_view.heart.details, LV_OBJ_FLAG_HIDDEN);
        ui_set_label_text(s_view.heart.details_title, "DETAILS  " LV_SYMBOL_UP);
        /* 无动画滚动到展开后的详情卡。 */
        lv_obj_scroll_to_view(s_view.heart.details, LV_ANIM_OFF);
    }
    else
    {
        /* 已展开时收起：加隐藏标志，标题箭头改回向右。 */
        lv_obj_add_flag(s_view.heart.details, LV_OBJ_FLAG_HIDDEN);
        ui_set_label_text(s_view.heart.details_title, "DETAILS  " LV_SYMBOL_RIGHT);
        /* 收起后页面变短，滚回顶部。 */
        lv_obj_scroll_to_y(s_body, 0, LV_ANIM_OFF);
    }
}

static void heart_window(void)
{
    /* DETAILS开关条句柄。 */
    lv_obj_t *toggle;
    const char *auto_text = "AUTO / 10 MIN: OFF";
    uint32_t auto_color = UI_MUTED;
    if (app_heart_get_auto_enabled() != 0)
    {
        auto_text = "AUTO / 10 MIN: ON";
        auto_color = UI_GREEN;
    }
    /* 顶部建212x76心率卡片。 */
    s_view.heart.card = ui_panel(s_body, 0, 0, 212, 76, UI_CARD);
    /* 卡片左上角画心形图标。 */
    heart_icon(s_view.heart.card, 14, 6);
    /* 图标右侧写HEART RATE标题，红色。 */
    ui_label(s_view.heart.card, "HEART RATE", 40, 10, &lv_font_montserrat_10, UI_RED);
    /* 卡片左下建BPM数值标签，初值"--"，红色大号字。 */
    s_view.heart.value = ui_label(s_view.heart.card, "--", 14, 25, &lv_font_montserrat_30, UI_RED);
    /* 数值右侧标单位BPM。 */
    ui_label(s_view.heart.card, "BPM", 78, 40, &lv_font_montserrat_10, UI_MUTED);
    /* 卡片底部建状态摘要标签，初值STANDBY。 */
    s_view.heart.summary = ui_label(s_view.heart.card, "STANDBY", 14, 60, &lv_font_montserrat_10, UI_MUTED);
    /* 卡片右侧建START按钮文字，尺寸82x36。 */
    s_view.heart.button = ui_label(s_view.heart.card, "START", 118, 27, &lv_font_montserrat_14, UI_BG);
    lv_obj_set_size(s_view.heart.button, 82, 36);
    /* 按钮底色蓝色、不透明、圆角12像素，顶部内边距10像素，文字居中。 */
    lv_obj_set_style_bg_color(s_view.heart.button, lv_color_hex(UI_BLUE), 0);
    lv_obj_set_style_bg_opa(s_view.heart.button, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_view.heart.button, 12, 0);
    lv_obj_set_style_pad_top(s_view.heart.button, 10, 0);
    lv_obj_set_style_text_align(s_view.heart.button, LV_TEXT_ALIGN_CENTER, 0);
    /* 按钮可点击，点击切换手动测量。 */
    lv_obj_add_flag(s_view.heart.button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_view.heart.button, heart_toggle_cb, LV_EVENT_CLICKED, NULL);
    /* 卡片下方建心率趋势图，每UI_TREND_SAMPLE_MS(1秒)一格。 */
    ui_trend_create(&s_view.heart.trend, s_body, "HEART RATE TREND / BPM", 84, 98, UI_RED, UI_TREND_SAMPLE_MS);
    /* 趋势图下方建212x28的DETAILS开关条。 */
    toggle = ui_panel(s_body, 0, 190, 212, 28, UI_BORDER);
    /* 条内写DETAILS文字加向右箭头。 */
    s_view.heart.details_title = ui_label(toggle, "DETAILS  " LV_SYMBOL_RIGHT, 14, 8, &lv_font_montserrat_10, UI_TEXT);
    /* 文字不接收点击，点击落到开关条底框。 */
    lv_obj_clear_flag(s_view.heart.details_title, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(toggle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(toggle, heart_details_cb, LV_EVENT_CLICKED, NULL);
    /* 后台自动心率独立开关；按当前RAM状态显示，开机默认ON。 */
    s_view.heart.auto_button = ui_panel(s_body, 0, 226, 212, 28, UI_BORDER);
    /* 开关条内写自动测量文案并居中。 */
    s_view.heart.auto_label = ui_label(s_view.heart.auto_button, auto_text, 0, 0, &lv_font_montserrat_10, auto_color);
    lv_obj_center(s_view.heart.auto_label);
    /* 文字不接收点击，底框可点击切换自动测量。 */
    lv_obj_clear_flag(s_view.heart.auto_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_view.heart.auto_button, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_view.heart.auto_button, heart_auto_cb, LV_EVENT_CLICKED, NULL);
    /* 最下方建212x76详情卡，默认隐藏。 */
    s_view.heart.details = ui_panel(s_body, 0, 262, 212, 76, UI_CARD);
    /* 详情卡内建状态标签，初值STANDBY，宽184像素。 */
    s_view.heart.status = ui_label(s_view.heart.details, "STANDBY", 14, 12, &lv_font_montserrat_10, UI_MUTED);
    lv_obj_set_width(s_view.heart.status, 184);
    /* 初始收起，等用户点DETAILS展开。 */
    lv_obj_add_flag(s_view.heart.details, LV_OBJ_FLAG_HIDDEN);
}

static void refresh_heart(void)
{
    /* 本轮心率快照。 */
    app_heart_struct heart;
    /* 摘要行文本，默认STANDBY。 */
    const char *status = "STANDBY";
    /* 心率页控件未创建(不在本页)时直接返回。 */
    if (s_view.heart.card == NULL)
    {
        return;
    }
    /* 取一次心率快照，下面所有显示都基于这份数据。 */
    app_heart_get_snapshot(&heart);
    /* 测量开启、状态有效且BPM在40..200内才把BPM写入趋势图。 */
    if (heart.enabled && heart.state == APP_HEART_VALID && heart.bpm >= 40 && heart.bpm <= 200)
    {
        ui_trend_update(&s_view.heart.trend, (lv_coord_t)heart.bpm, 0);
    }
    else
    {
        /* 其他情况往趋势图写"无点"哨兵，该格留空。 */
        ui_trend_update(&s_view.heart.trend, LV_CHART_POINT_NONE, 0);
    }
    /* 状态有效时显示BPM数值，摘要行写TRACKING。 */
    if (heart.state == APP_HEART_VALID)
    {
        ui_set_label_text_fmt(s_view.heart.value, "%u", (unsigned)heart.bpm);
        status = "TRACKING";
    }
    else
    {
        /* 非有效状态数值显示"--"，摘要文本按具体状态分支。 */
        ui_set_label_text(s_view.heart.value, "--");
        switch (heart.state)
        {
        case APP_HEART_MEASURING:
            /* 正在采集/等待波峰。 */
            status = "MEASURING";
            break;
        case APP_HEART_NO_SIGNAL:
            /* 算法未找到有效脉搏。 */
            status = "NO SIGNAL";
            break;
        case APP_HEART_ERROR:
            /* 传感器访问或配置失败。 */
            status = "UNAVAILABLE";
            break;
        default:
            /* OFF等其余状态保留STANDBY。 */
            break;
        }
    }
    /* 把摘要文本写到卡片底部。 */
    ui_set_label_text(s_view.heart.summary, status);
    /* 有效为绿色，故障为红色，测量中/无信号为橙色，其余灰色。 */
    if (heart.state == APP_HEART_VALID)
    {
        ui_set_text_color(s_view.heart.summary, UI_GREEN);
    }
    else if (heart.state == APP_HEART_ERROR)
    {
        ui_set_text_color(s_view.heart.summary, UI_RED);
    }
    else if (heart.state == APP_HEART_MEASURING || heart.state == APP_HEART_NO_SIGNAL)
    {
        ui_set_text_color(s_view.heart.summary, UI_ORANGE);
    }
    else
    {
        ui_set_text_color(s_view.heart.summary, UI_MUTED);
    }
    /* 故障时保留实际失败步骤；读取失败不能只显示算法的 NO SIGNAL。 */
    if (heart.state == APP_HEART_ERROR)
    {
        /* 把驱动故障类型翻译成具体步骤名。 */
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
        /* PID/配置校验失败时显示相关寄存器地址及实际读到的值。 */
        if (heart.fault.kind == EM7028_FAULT_PID || heart.fault.kind == EM7028_FAULT_CONFIG)
        {
            ui_set_label_text_fmt(s_view.heart.status, "%s\n%s REG %02X\nVALUE %02X", status, reason, (unsigned)heart.fault.reg,
                               (unsigned)heart.fault.value);
        }
        else
        {
            /* 其他故障只显示相关寄存器号，并提示重新START。 */
            ui_set_label_text_fmt(s_view.heart.status, "%s\n%s REG %02X\nRetry START", status, reason, (unsigned)heart.fault.reg);
        }
    }
    else
    {
        /* 正常状态显示原始光强、交流幅度(0.1计数为单位拆成整数/小数)和波峰间隔毫秒。 */
        ui_set_label_text_fmt(s_view.heart.status, "%s\nHRS1 %u  AC %lu.%lu\nPERIOD %u ms", status, (unsigned)heart.raw,
                           (unsigned long)(heart.signal_x10 / 10), (unsigned long)(heart.signal_x10 % 10),
                           (unsigned)heart.interval_ms);
    }
    /* 故障详情红色，其余灰色。 */
    if (heart.state == APP_HEART_ERROR)
    {
        ui_set_text_color(s_view.heart.status, UI_RED);
    }
    else
    {
        ui_set_text_color(s_view.heart.status, UI_MUTED);
    }
    /* 运动模式下手动按钮禁用并显示SPORT。 */
    if (s_sport_mode != 0)
    {
        ui_set_label_text(s_view.heart.button, "SPORT");
        lv_obj_add_state(s_view.heart.button, LV_STATE_DISABLED);
    }
    else if (heart.enabled)
    {
        /* 手动测量进行中：恢复可点击，文案显示STOP。 */
        lv_obj_clear_state(s_view.heart.button, LV_STATE_DISABLED);
        ui_set_label_text(s_view.heart.button, "STOP");
    }
    else
    {
        /* 手动测量未开启：恢复可点击，文案显示START。 */
        lv_obj_clear_state(s_view.heart.button, LV_STATE_DISABLED);
        ui_set_label_text(s_view.heart.button, "START");
    }
    /* 独立显示后台自动心率开关，不依赖heart.enabled。 */
    if (app_heart_get_auto_enabled() != 0)
    {
        /* 自动测量已开：文案ON，绿色。 */
        ui_set_label_text(s_view.heart.auto_label, "AUTO / 10 MIN: ON");
        ui_set_text_color(s_view.heart.auto_label, UI_GREEN);
    }
    else
    {
        /* 自动测量已关：文案OFF，灰色。 */
        ui_set_label_text(s_view.heart.auto_label, "AUTO / 10 MIN: OFF");
        ui_set_text_color(s_view.heart.auto_label, UI_MUTED);
    }
}

/* 显示设置页：建亮度卡片，用滑条调整背光亮度，并在卡片下方显示重启后恢复的说明。 */
static void display_page(void)
{
    /* 亮度设置卡片，位于内容区顶部，尺寸212x128像素。 */
    lv_obj_t *obj = ui_panel(s_body, 0, 4, 212, 128, UI_CARD);
    /* 亮度滑条句柄，创建后逐项设置位置、范围与各部分样式。 */
    lv_obj_t *slider;
    /* 卡片左上角写蓝色DISPLAY标题。 */
    ui_label(obj, "DISPLAY", 14, 14, &lv_font_montserrat_10, UI_BLUE);
    /* 标题下方写Brightness说明文字。 */
    ui_label(obj, "Brightness", 14, 38, &lv_font_montserrat_14, UI_TEXT);
    /* 卡片右侧建亮度百分比标签，初值空串，由下一行填入。 */
    s_view.display.brightness = ui_label(obj, "", 151, 38, &lv_font_montserrat_14, UI_ORANGE);
    /* 按当前亮度s_brightness显示"NN%"，只读显示，不在这里改背光。 */
    ui_set_label_text_fmt(s_view.display.brightness, "%d%%", (int)s_brightness);
    /* 在卡片内创建亮度滑条。 */
    slider = lv_slider_create(obj);
    /* 清掉默认主题样式，轨道、指示器和旋钮全部由下面逐项指定。 */
    lv_obj_remove_style_all(slider);
    /* 滑条相对卡片定位到(24,82)。 */
    lv_obj_set_pos(slider, 24, 82);
    /* 滑条尺寸164x10像素。 */
    lv_obj_set_size(slider, 164, 10);
    /* 值域限定为UI_MIN_BRIGHTNESS..100，与亮度百分比一致。 */
    lv_slider_set_range(slider, UI_MIN_BRIGHTNESS, 100);
    /* 用当前亮度初始化滑块位置，关闭动画。 */
    lv_slider_set_value(slider, s_brightness, LV_ANIM_OFF);
    /* 轨道(MAIN)背景完全不透明。 */
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    /* 轨道颜色用暗色UI_BORDER。 */
    lv_obj_set_style_bg_color(slider, lv_color_hex(UI_BORDER), LV_PART_MAIN);
    /* 轨道两端做成圆形。 */
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    /* 已选段(INDICATOR)背景完全不透明。 */
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR);
    /* 已选段用蓝色UI_BLUE。 */
    lv_obj_set_style_bg_color(slider, lv_color_hex(UI_BLUE), LV_PART_INDICATOR);
    /* 已选段两端同样圆形收边。 */
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);
    /* 旋钮(KNOB)背景完全不透明。 */
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
    /* 旋钮用橙色UI_ORANGE突出显示。 */
    lv_obj_set_style_bg_color(slider, lv_color_hex(UI_ORANGE), LV_PART_KNOB);
    /* 旋钮做成圆形。 */
    lv_obj_set_style_radius(slider, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    /* 旋钮四周设7像素内边距。 */
    lv_obj_set_style_pad_all(slider, 7, LV_PART_KNOB);
    /* 把可点击范围向外扩16像素，细轨道也容易按住拖动。 */
    lv_obj_set_ext_click_area(slider, 16);
    /* A brightness adjustment must not navigate to another page. */
    /* 两个标志分别阻止手势冒泡到页面、阻止滑条自身滚动。 */
    lv_obj_clear_flag(slider, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SCROLLABLE);
    /* 值变化事件绑定brightness_cb，由它保存新亮度并立即控制背光。 */
    lv_obj_add_event_cb(slider, brightness_cb, LV_EVENT_VALUE_CHANGED, NULL);
    /* 卡片下方加灰色说明：亮度设置只在本次运行内有效，重启后恢复。 */
    ui_note(s_body, "Brightness resets after restart.", 150);
}

/* 温湿度页：建AHT21卡片，放状态、湿度、气温和样本计数。 */
static void climate_page(void)
{
    /* 212x212的温湿度卡片，位置在内容区顶部。 */
    s_view.climate.card = ui_panel(s_body, 0, 4, 212, 212, UI_CARD);
    /* 卡片标题写明器件与测量内容。 */
    ui_label(s_view.climate.card, "AHT21 / TEMP + HUMIDITY", 14, 12, &lv_font_montserrat_10, UI_BLUE);
    /* 状态标签初值WAITING，由refresh_aht21更新。 */
    s_view.climate.status = ui_label(s_view.climate.card, "WAITING", 14, 30, &lv_font_montserrat_10, UI_MUTED);
    /* 湿度区小标题。 */
    ui_label(s_view.climate.card, "HUMIDITY", 14, 54, &lv_font_montserrat_10, UI_MUTED);
    /* 湿度数值右侧标单位%RH。 */
    ui_label(s_view.climate.card, "%RH", 169, 80, &lv_font_montserrat_10, UI_GREEN);
    /* 湿度大号数值标签，初值"--"，宽146像素。 */
    s_view.climate.humidity = ui_label(s_view.climate.card, "--", 14, 70, &lv_font_montserrat_30, UI_GREEN);
    lv_obj_set_width(s_view.climate.humidity, 146);
    /* 气温区小标题。 */
    ui_label(s_view.climate.card, "AIR TEMPERATURE", 14, 116, &lv_font_montserrat_10, UI_MUTED);
    /* 气温数值右侧标单位C。 */
    ui_label(s_view.climate.card, "C", 184, 142, &lv_font_montserrat_10, UI_ORANGE);
    /* 气温大号数值标签，初值"--"，宽146像素。 */
    s_view.climate.temperature = ui_label(s_view.climate.card, "--", 14, 132, &lv_font_montserrat_30, UI_ORANGE);
    lv_obj_set_width(s_view.climate.temperature, 146);
    /* 卡片底部显示累计样本数，初值0。 */
    s_view.climate.samples = ui_label(s_view.climate.card, "SAMPLES 0 / 1s", 14, 186, &lv_font_montserrat_10, UI_MUTED);
}

/* 设置网格页：按表驱动逐项创建入口卡片，点击跳到对应功能页。 */
static void settings_page(void)
{
    /* 8个入口卡片的目标页，下标与下面titles/icons一一对应。 */
    static const ui_page_t destinations[] = {PAGE_HEART, PAGE_MOTION, PAGE_ENVIRONMENT, PAGE_AHT21,
                                             PAGE_WEATHER, PAGE_BLUETOOTH, PAGE_DISPLAY, PAGE_SPORT};
    /* 卡片显示名称，顺序与destinations一致。 */
    static const char *const titles[] = {"Heart rate", "Motion", "Barometer", "Climate", "Weather", "Bluetooth", "Display", "Sport mode"};
    /* 卡片左侧图标，使用LVGL内置符号字体。 */
    static const char *const icons[] = {LV_SYMBOL_TINT, LV_SYMBOL_REFRESH, LV_SYMBOL_UP, LV_SYMBOL_TINT,
                                        LV_SYMBOL_IMAGE, LV_SYMBOL_BLUETOOTH, LV_SYMBOL_SETTINGS, LV_SYMBOL_PLAY};
    /* 入口卡片下标。 */
    uint8_t i;
    /* 遍历全部入口，i同时作为卡片在网格中的位置序号。 */
    for (i = 0; i < sizeof(destinations) / sizeof(destinations[0]); ++i)
    {
        /* 用图标、名称、位置、颜色和目标页地址创建一张入口卡片。 */
        card(icons[i], titles[i], i, UI_BLUE, &destinations[i]);
    }
}

/* 返回点击回调：非切页动画期间提交返回请求。 */
static void back_cb(lv_event_t *event)
{
    /* 事件对象未使用，显式丢弃以避免未使用参数告警。 */
    (void)event;
    /* 切页动画进行中忽略点击，防止动画期间叠加新的切页。 */
    if (!s_transition)
    {
        /* 只登记返回请求，实际切页由ui_timer_cb在松手后执行。 */
        App_UI_Back();
    }
}

/* 天气页构造与其他页面使用相同入口，路由器只负责公共屏幕和动画。 */
static void weather_page(void)
{
    /* 天气页在内容区顶部建"PHONE WEATHER"指标卡，单位标注BLE。 */
    s_view.weather.value = metric("PHONE WEATHER", "BLE", 4, UI_BLUE);
    /* 把指标卡的父卡片高度改为132像素，容纳图标与文字。 */
    lv_obj_set_height(lv_obj_get_parent(s_view.weather.value), 132);
    /* 天气数值右移到(88,48)，宽度110像素，留出左侧图标位置。 */
    lv_obj_set_pos(s_view.weather.value, 88, 48);
    lv_obj_set_width(s_view.weather.value, 110);
    /* 数值改用20号字体。 */
    lv_obj_set_style_text_font(s_view.weather.value, &lv_font_montserrat_20, 0);
    /* 卡片左侧建64x64透明图标容器，背景透明以便自绘天气图标。 */
    s_view.weather.icon = ui_panel(lv_obj_get_parent(s_view.weather.value), 14, 27, 64, 64, UI_CARD);
    lv_obj_set_style_bg_opa(s_view.weather.icon, LV_OPA_TRANSP, 0);
    /* 图标只做显示，不响应点击。 */
    lv_obj_clear_flag(s_view.weather.icon, LV_OBJ_FLAG_CLICKABLE);
    /* 主绘制事件交给weather_icon_draw_cb按天气代码画图形。 */
    lv_obj_add_event_cb(s_view.weather.icon, weather_icon_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    /* 卡片下方建天气描述标签，初值"--"，宽184像素。 */
    s_view.weather.condition = ui_label(lv_obj_get_parent(s_view.weather.value), "--", 14, 100, &lv_font_montserrat_20, UI_TEXT);
    lv_obj_set_width(s_view.weather.condition, 184);
    /* 卡片外先写等待手机的提示，收到数据后由refresh_weather改写。 */
    s_view.weather.label = ui_note(s_body, "WAITING FOR PHONE", 148);
    ui_note(s_body, "Weather comes from your phone. Connect the app to sync.", 190);
}

static void refresh_motion(void);
static void refresh_baro(void);
static void refresh_aht21(void);
static void refresh_weather(void);

typedef struct
{
    const char *title;
    void (*create)(void);
    void (*refresh)(void);
} ui_page_descriptor_struct;

/* 与ui_page_t逐页对应；新增页面的创建与刷新只在这里注册。 */
static const ui_page_descriptor_struct s_pages[PAGE_COUNT] =
{
    [PAGE_WATCH] = {"WATCH", watch_page, refresh_clock},
    [PAGE_SETTINGS] = {"APPS", settings_page, NULL},
    [PAGE_BLUETOOTH] = {"BLUETOOTH", bluetooth_page, refresh_bluetooth},
    [PAGE_DISPLAY] = {"DISPLAY", display_page, NULL},
    [PAGE_HEART] = {"HEART RATE", heart_window, refresh_heart},
    [PAGE_MOTION] = {"COMPASS", imu_page, refresh_motion},
    [PAGE_ENVIRONMENT] = {"BAROMETER", baro_page, refresh_baro},
    [PAGE_AHT21] = {"CLIMATE", climate_page, refresh_aht21},
    [PAGE_WEATHER] = {"WEATHER", weather_page, refresh_weather},
    [PAGE_SPORT] = {"SPORT MODE", sport_page, NULL}
};

/* 切页只复位当前页面引用；LVGL对象由旧屏幕在动画结束后递归删除。 */
static void clear_page_refs(void)
{
    memset(&s_view, 0, sizeof(s_view));
    s_view.weather.icon_code = 255;
    s_view.watch.refresh_tick = lv_tick_get() - 1000;
    s_view.baro.refresh_tick = lv_tick_get() - UI_BARO_REFRESH_MS;
    s_view.motion.compass_heading = -1;
    s_view.motion.compass_tick = lv_tick_get();
    s_view.watch.last_second = UINT32_MAX;
}

/* 屏幕加载完成回调：切页动画结束时清除s_transition。 */
static void screen_loaded_cb(lv_event_t *event)
{
    /* 只处理当前屏幕自己的加载完成事件，忽略其他对象的同名事件。 */
    if (lv_event_get_target(event) == s_screen)
    {
        /* 动画结束，允许ui_timer_cb继续重建页面和刷新数据。 */
        s_transition = 0;
    }
}

/* 把当前主页(表盘或设置)使用的控件指针存入缓存槽，供切回时恢复。 */
static void main_page_save(ui_main_page_struct *page)
{
    /* 屏幕与内容区容器。 */
    page->screen = s_screen;
    page->body = s_body;
    /* 电量条与百分比标签。 */
    page->battery = s_view.battery.label;
    page->battery_bar = s_view.battery.bar;
    /* 时间、提示、心率、温度、步数标签。 */
    page->time = s_view.watch.time;
    page->hint = s_view.watch.hint;
    page->heart = s_view.watch.heart;
    page->temperature = s_view.watch.temperature;
    page->steps = s_view.watch.steps;
    /* 三根表针对象。 */
    page->hour_hand = s_view.watch.hour_hand;
    page->minute_hand = s_view.watch.minute_hand;
    page->second_hand = s_view.watch.second_hand;
}

/* 从缓存槽把控件指针写回全局，复用已建好的页面而不重建。 */
static void main_page_restore(const ui_main_page_struct *page)
{
    /* 恢复屏幕与内容区。 */
    s_screen = page->screen;
    s_body = page->body;
    /* 恢复电量条与百分比标签。 */
    s_view.battery.label = page->battery;
    s_view.battery.bar = page->battery_bar;
    /* 恢复表盘各数据标签。 */
    s_view.watch.time = page->time;
    s_view.watch.hint = page->hint;
    s_view.watch.heart = page->heart;
    s_view.watch.temperature = page->temperature;
    s_view.watch.steps = page->steps;
    /* 恢复三根表针。 */
    s_view.watch.hour_hand = page->hour_hand;
    s_view.watch.minute_hand = page->minute_hand;
    s_view.watch.second_hand = page->second_hand;
}

/* 页面缓存/历史的核心：表盘与设置页互切保留缓存；进入详情时清空主页缓存，非当前
 * 主页立即删除，当前旧屏幕由切页动画结束后删除。返回主页时滚动位置由s_history另存，
 * 缓存槽只保存控件指针。切页动画期间s_transition为1，LV_EVENT_SCREEN_LOADED回调到才清零。 */
static void create_page(ui_page_t page, lv_coord_t scroll_y, uint8_t back)
{
    /* 底部页签圆点与标题/返回文字的临时控件句柄。 */
    lv_obj_t *dot, *navigation;
    /* 切页前正在显示的旧屏幕，用于判断动画删除和缓存清理。 */
    lv_obj_t *old_screen = s_screen;
    /* 切页前的页面枚举，用于判断是否在两个缓存主页之间切换。 */
    ui_page_t old_page = s_page;
    /* 默认新页从右侧滑入；返回时下面改为从左侧滑入。 */
    lv_scr_load_anim_t animation_type = LV_SCR_LOAD_ANIM_OVER_LEFT;
    /* 屏幕切换动画句柄，用于追加缓动曲线。 */
    lv_anim_t *animation;
    /* 循环下标i，以及当前页对应的底部页签下标tab。 */
    uint8_t i, tab;
    /* 底部页签圆点的宽度与颜色，选中页会加大并变蓝。 */
    lv_coord_t dot_width;
    uint32_t dot_color;
    /* 是否让LVGL在切屏动画结束后自动删除旧屏幕，默认删除。 */
    uint8_t auto_delete = 1;
    /* 当前显示屏的水平分辨率，单位像素。 */
    int width = lv_disp_get_hor_res(NULL);
    /* 当前显示屏的垂直分辨率，单位像素。 */
    int height = lv_disp_get_ver_res(NULL);
    /* 默认页签指向APPS(设置网格)。 */
    tab = PAGE_SETTINGS;
    /* 表盘页自己就是一页，页签改指表盘。 */
    if (page == PAGE_WATCH)
    {
        tab = PAGE_WATCH;
    }
    /* 进入功能详情页(枚举值大于设置页)时，主页缓存不再需要。 */
    if (page > PAGE_SETTINGS)
    {
        /* 逐个检查两个缓存主页槽位。 */
        for (i = 0; i < 2; ++i)
        {
            /* 槽位有屏幕且不是当前正显示的旧屏幕时才删除，避免删掉正在用的屏幕。 */
            if (s_main_pages[i].screen != NULL && s_main_pages[i].screen != old_screen)
            {
                lv_obj_del(s_main_pages[i].screen);
            }
            /* 清空该槽位的控件指针记录。 */
            memset(&s_main_pages[i], 0, sizeof(s_main_pages[i]));
        }
    }
    /* 在两个缓存主页之间切换时，旧屏幕保留在缓存中供后续复用，不让LVGL自动删除。 */
    else if (old_page <= PAGE_SETTINGS)
    {
        auto_delete = 0;
    }
    /* 先把上一页所有控件指针清空，避免刷新函数误用已失效对象。 */
    clear_page_refs();
    /* 记录当前页枚举，后续刷新和返回逻辑据此分支。 */
    s_page = page;
    /* 目标页是缓存主页且已有保存的屏幕时，直接恢复缓存，不重建控件。 */
    if (page <= PAGE_SETTINGS && s_main_pages[page].screen != NULL)
    {
        main_page_restore(&s_main_pages[page]);
    }
    /* 否则新建一整块屏幕并搭建该页内容。 */
    else
    {
        /* 创建无父对象的顶层屏幕对象。 */
        s_screen = lv_obj_create(NULL);
        /* 清掉默认主题样式，背景和滚动行为全部显式指定。 */
        lv_obj_remove_style_all(s_screen);
        /* 屏幕背景色用UI_BG。 */
        lv_obj_set_style_bg_color(s_screen, lv_color_hex(UI_BG), 0);
        /* 背景完全不透明，遮住下层。 */
        lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
        /* 屏幕本身不滚动、不把手势冒泡给父级。 */
        lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_GESTURE_BUBBLE);
        /* 手势事件交给gesture_cb处理左右滑切页。 */
        lv_obj_add_event_cb(s_screen, gesture_cb, LV_EVENT_GESTURE, NULL);
        /* 屏幕加载完成事件交给screen_loaded_cb清除切页动画标志。 */
        lv_obj_add_event_cb(s_screen, screen_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);
        /* 表盘页不显示顶部标题，其余页在左上角写页名。 */
        if (page != PAGE_WATCH)
        {
            /* 用14号字体创建页名标签，坐标(14,13)。 */
            navigation = ui_label(s_screen, s_pages[page].title, 14, 13, &lv_font_montserrat_14, UI_TEXT);
            /* 功能详情页的页名兼作返回入口。 */
            if (page > PAGE_SETTINGS)
            {
                /* 让标题可点击，并把点击范围向外扩10像素。 */
                lv_obj_add_flag(navigation, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_ext_click_area(navigation, 10);
                /* 点击标题触发back_cb，由它提交返回请求。 */
                lv_obj_add_event_cb(navigation, back_cb, LV_EVENT_CLICKED, NULL);
            }
        }
        /* 右上角画电量条外框，水平位置按屏幕宽度右对齐。 */
        create_battery_bar(s_screen, width - 106, 15);
        /* 电量条右侧建百分比文字标签，初值"--%"，宽52像素右对齐。 */
        s_view.battery.label = ui_label(s_screen, "--%", width - 66, 15, &lv_font_montserrat_10, UI_MUTED);
        lv_obj_set_width(s_view.battery.label, 52);
        lv_obj_set_style_text_align(s_view.battery.label, LV_TEXT_ALIGN_RIGHT, 0);
        /* 在标题下方建212像素宽的内容区，高度扣掉顶部38和底部24像素。 */
        s_body = ui_panel(s_screen, (width - 212) / 2, 38, 212, height - 62, UI_BG);
        /* 内容区不要panel默认的16像素圆角，避免裁掉贴边内容。 */
        lv_obj_set_style_radius(s_body, 0, 0);
        /* 表盘页的内容区铺满整块屏幕，而不是212宽的窄条。 */
        if (page == PAGE_WATCH)
        {
            /* 使用整块矩形屏幕，摘要不再受圆形边缘裁切。 */
            lv_obj_set_pos(s_body, 0, 0);
            lv_obj_set_size(s_body, width, height);
        }
        /* 非表盘页内容可能超出一屏，允许内容区上下滚动。 */
        if (page != PAGE_WATCH)
        {
            lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
        }
        /* 关闭滚动回弹和滚动链，滚到边界即停，不带动父级。 */
        lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
        /* 表盘页固定不滚动，也不显示滚动条。 */
        if (page == PAGE_WATCH)
        {
            lv_obj_set_scroll_dir(s_body, LV_DIR_NONE);
            lv_obj_set_scrollbar_mode(s_body, LV_SCROLLBAR_MODE_OFF);
        }
        /* 其他页只允许纵向滚动，滚动条按内容是否超出自动显示。 */
        else
        {
            lv_obj_set_scroll_dir(s_body, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(s_body, LV_SCROLLBAR_MODE_AUTO);
        }
        /* 滚动条用蓝色、60%不透明度、2像素宽，并去掉右侧额外内边距。 */
        lv_obj_set_style_bg_color(s_body, lv_color_hex(UI_BLUE), LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_opa(s_body, LV_OPA_60, LV_PART_SCROLLBAR);
        lv_obj_set_style_width(s_body, 2, LV_PART_SCROLLBAR);
        lv_obj_set_style_pad_right(s_body, 0, LV_PART_SCROLLBAR);
        /* 每页的标题、创建和刷新入口统一在s_pages登记。 */
        s_pages[page].create();
        /* 非表盘页在屏幕底部画两个页签圆点和返回文字。 */
        if (page != PAGE_WATCH)
        {
            /* 两个圆点分别代表表盘和APPS两级。 */
            for (i = 0; i < 2; ++i)
            {
                /* 未选中圆点宽5像素、用边框暗色。 */
                dot_width = 5;
                dot_color = UI_BORDER;
                /* 当前页对应的圆点加宽到12像素并改为蓝色。 */
                if (i == tab)
                {
                    dot_width = 12;
                    dot_color = UI_BLUE;
                }
                /* 以屏幕水平中心为基准，第i个圆点右移16*i像素，纵向贴近底部。 */
                dot = ui_panel(s_screen, width / 2 - 12 + i * 16, height - 14, dot_width, 5, dot_color);
                /* 圆点两端做成圆形，且不接收点击。 */
                lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
                lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
            }
            /* 左下角写"< BACK"返回文字。 */
            navigation = ui_label(s_screen, LV_SYMBOL_LEFT " BACK", 14, height - 17, &lv_font_montserrat_10, UI_BLUE);
            /* 文字可点击，点击范围外扩8像素，点击触发back_cb。 */
            lv_obj_add_flag(navigation, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_set_ext_click_area(navigation, 8);
            lv_obj_add_event_cb(navigation, back_cb, LV_EVENT_CLICKED, NULL);
        }
        /* 只有两个缓存主页才保存控件指针，供返回时恢复。 */
        if (page <= PAGE_SETTINGS)
        {
            main_page_save(&s_main_pages[page]);
        }
    }
    /* 页面控件就绪后立即按快照填一次数据，避免首帧显示占位符。 */
    refresh_data();
    /* 强制计算一次布局，让下面按像素设置的滚动位置立即生效。 */
    lv_obj_update_layout(s_screen);
    /* 恢复到调用方给出的滚动位置，关闭动画；新建详情页时scroll_y为0。 */
    lv_obj_scroll_to_y(s_body, scroll_y, LV_ANIM_OFF);
    /* 首次建屏(没有旧屏幕)直接加载，不做切屏动画。 */
    if (old_screen == NULL)
    {
        lv_scr_load(s_screen);
    }
    /* 新旧屏幕不同才需要切屏动画。 */
    else if (old_screen != s_screen)
    {
        /* 标记切页动画进行中，期间ui_timer_cb推迟重建和刷新。 */
        s_transition = 1;
        /* 返回操作改成新页从左侧滑入，形成反向动画。 */
        if (back != 0)
        {
            animation_type = LV_SCR_LOAD_ANIM_OVER_RIGHT;
        }
        /* 旧页静止，新页滑入；只驱动一张屏幕的位置，减少每帧布局工作。 */
        /* 动画时长UI_ANIM_MS，auto_delete非0时动画结束由LVGL删除旧屏幕。 */
        lv_scr_load_anim(s_screen, animation_type, UI_ANIM_MS, 0, auto_delete != 0);
        /* 取刚创建的切屏动画，存在时把路径改为缓入缓出。 */
        animation = lv_anim_get(s_screen, NULL);
        if (animation != NULL)
        {
            lv_anim_set_path_cb(animation, lv_anim_path_ease_in_out);
        }
    }
}

/* 航向暂停原因直接来自采样服务，不通过“有磁场数字”猜测融合状态。
 * 把磁融合状态翻译成航向来源说明文字，返回静态字符串供标签显示。 */
static const char *mag_yaw_status(const app_imu_struct *imu)
{
    /* 按融合状态枚举选择对应的说明。 */
    switch (imu->mag_fusion_state)
    {
    case APP_MAG_FUSION_ACTIVE:
        /* 磁融合正在参与解算：再按是否已校准区分"磁北"和"未校准"。 */
        if (imu->mag_calibrated)
        {
            return "MAG NORTH / fused yaw";
        }
        else
        {
            return "MAG UNCAL / calibrate first";
        }
    case APP_MAG_FUSION_CALIBRATING:
        /* 正在采集校准数据，航向暂时只靠陀螺。 */
        return "GYRO ONLY / CALIBRATING";
    case APP_MAG_FUSION_FIELD_RANGE:
        /* 磁场幅值超出合理范围，融合被暂停。 */
        return "GYRO ONLY / FIELD RANGE";
    case APP_MAG_FUSION_HORIZONTAL_WEAK:
        /* 水平分量过弱，无法可靠求航向。 */
        return "GYRO ONLY / HORIZ <5uT";
    case APP_MAG_FUSION_IMU_NO_DATA:
        /* 没有新的IMU帧，融合无法推进。 */
        return "GYRO ONLY / NO IMU FRAME";
    default:
        /* 其余情况按磁力计无数据说明。 */
        return "GYRO ONLY / MAG NO DATA";
    }
}

/* 把磁校准状态、EEPROM保存状态和采样覆盖幅度写到校准状态标签。 */
static void mag_calibration_status(const app_imu_struct *imu)
{
    /* 当前要显示的结果文字，先由状态分支确定，再统一拼接样本数与跨度。 */
    const char *result;
    /* 没有可用校准记录时，区分"EEPROM读取失败"和"当前无校准结果"。 */
    if (imu->mag_cal_state == APP_MAG_CAL_NONE)
    {
        /* 开机加载校准记录失败时提示需要重新校准。 */
        if (imu->mag_persist_state == APP_MAG_PERSIST_LOAD_FAILED)
        {
            ui_set_label_text(s_view.motion.cal_status, "EEPROM READ FAILED\nCalibration required");
        }
        /* 否则只是当前没有可用校准记录，显示CAL NOT RUN。 */
        else
        {
            ui_set_label_text(s_view.motion.cal_status, "CAL NOT RUN");
        }
        return;
    }
    /* 校准正在采集数据。 */
    if (imu->mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        result = "COLLECTING";
    }
    /* 校准已完成时，再按EEPROM持久化状态细化提示。 */
    else if (imu->mag_cal_state == APP_MAG_CAL_DONE)
    {
        switch (imu->mag_persist_state)
        {
        case APP_MAG_PERSIST_RESTORED:
            /* 本次校准来自开机从EEPROM恢复的记录，已经可用于磁融合。 */
            ui_set_label_text(s_view.motion.cal_status, "CAL RESTORED / EEPROM\nReady for magnetic fusion");
            return;
        case APP_MAG_PERSIST_SAVING:
            /* 结果已算出，正在写EEPROM。 */
            ui_set_label_text(s_view.motion.cal_status, "CAL OK / SAVING EEPROM");
            return;
        case APP_MAG_PERSIST_SAVE_FAILED:
            /* 保存失败：本次仅RAM内有效，提示检查EEPROM与写保护。 */
            ui_set_label_text(s_view.motion.cal_status, "CAL OK / SAVE FAILED\nRAM only: check EEPROM/WP");
            return;
        case APP_MAG_PERSIST_SAVED:
            /* 已成功写入EEPROM。 */
            result = "CAL SAVED";
            break;
        default:
            /* 其他情况只说明校准成功，不声称已保存。 */
            result = "CAL OK";
            break;
        }
    }
    /* 校准失败。 */
    else
    {
        result = "CAL FAILED";
    }
    /* 显示帧数与三轴覆盖幅度，保留失败证据，方便再次旋转校准。 */
    /* 三轴跨度单位uT，用于判断旋转时各轴是否都覆盖到足够幅度。 */
    ui_set_label_text_fmt(s_view.motion.cal_status, "%s: %u samples\nX/Y/Z span: %ld/%ld/%ld uT", result, (unsigned)imu->mag_cal_samples,
                       (long)imu->mag_cal_span_uT[0], (long)imu->mag_cal_span_uT[1], (long)imu->mag_cal_span_uT[2]);
}

/* 按AHT21快照刷新温湿度数值、状态文字和累计样本数。 */
static void refresh_aht21(void)
{
    /* 本次读取的AHT21快照，数值和状态都取自这一份。 */
    app_aht21_struct snapshot;
    /* 状态标签文字，默认等待中。 */
    const char *status = "WAITING";
    /* 状态标签颜色，默认灰色。 */
    uint32_t color = UI_MUTED;
    /* 温湿度页控件未创建(不在本页)时直接返回。 */
    if (s_view.climate.card == NULL)
    {
        return;
    }
    /* 取一次一致快照。 */
    app_aht21_get_snapshot(&snapshot);
    /* 快照有效时显示湿度(%RH)和气温(C)，各保留一位小数。 */
    if (snapshot.valid != 0)
    {
        ui_set_decimal(s_view.climate.humidity, snapshot.sample.humidity_rh, 0);
        ui_set_decimal(s_view.climate.temperature, snapshot.sample.temperature_c, 0);
    }
    /* 快照无效时两个数值都显示占位符。 */
    else
    {
        ui_set_label_text(s_view.climate.humidity, "--");
        ui_set_label_text(s_view.climate.temperature, "--");
    }
    /* 有错误结果时优先显示错误，并按错误类型细分文字。 */
    if (snapshot.result != AHT21_OK)
    {
        color = UI_RED;
        switch (snapshot.result)
        {
        case AHT21_BUSY:
            /* 器件或驱动忙，本次未完成读取。 */
            status = "BUSY";
            break;
        case AHT21_I2C_ERROR:
            /* I2C传输失败。 */
            status = "I2C ERROR";
            break;
        case AHT21_CALIBRATION_ERROR:
            /* 器件内部校准状态异常。 */
            status = "NOT CALIBRATED";
            break;
        case AHT21_CRC_ERROR:
            /* 数据CRC校验不通过。 */
            status = "CRC ERROR";
            break;
        case AHT21_TIMEOUT:
            /* 等待转换完成超时。 */
            status = "CONVERSION TIMEOUT";
            break;
        default:
            /* 其余错误统一按读取失败显示。 */
            status = "READ ERROR";
            break;
        }
    }
    /* 无错误且正在测量时显示MEASURING，用橙色。 */
    else if (snapshot.measuring != 0)
    {
        status = "MEASURING";
        color = UI_ORANGE;
    }
    /* 无错误、未在测量且快照有效时显示在线且CRC通过，用绿色。 */
    else if (snapshot.valid != 0)
    {
        status = "ONLINE / CRC OK";
        color = UI_GREEN;
    }
    /* 写入状态文字并应用颜色。 */
    ui_set_label_text(s_view.climate.status, status);
    ui_set_text_color(s_view.climate.status, color);
    /* 底部显示累计采样计数，标签文案中的"1s"只是说明节奏。 */
    ui_set_label_text_fmt(s_view.climate.samples, "SAMPLES %lu / 1s", (unsigned long)snapshot.sample_count);
}

/* 按手机天气快照刷新温度、天气描述、图标和缓存/时效提示。 */
static void refresh_weather(void)
{
    /* 本次读取的天气快照。 */
    app_weather_struct weather;
    /* 天气页控件未创建(不在本页)时直接返回。 */
    if (s_view.weather.label == NULL)
    {
        return;
    }
    /* 取一次一致快照。 */
    app_weather_get_snapshot(&weather);
    /* 只有快照有效时才更新页面；无效时保留原有内容不动。 */
    if (weather.valid)
    {
        /* 温度绝对值(0.1摄氏度为单位)，用于拆分整数和小数。 */
        int32_t magnitude;
        /* 温度为负时的符号串，非负为空串。 */
        const char *sign;
        /* 快照温度是放大10倍的整数，负数先取绝对值再记录负号。 */
        if (weather.temperature_x10 < 0)
        {
            magnitude = -weather.temperature_x10;
            sign = "-";
        }
        /* 非负温度直接取原值，不加符号。 */
        else
        {
            magnitude = weather.temperature_x10;
            sign = "";
        }
        /* 大号数值显示"x.y C"，整数部分除以10、小数部分模10。 */
        ui_set_label_text_fmt(s_view.weather.value, "%s%ld.%ld C", sign, (long)(magnitude / 10), (long)(magnitude % 10));
        /* 描述文字由天气代码翻译，例如晴、多云。 */
        ui_set_label_text(s_view.weather.condition, app_weather_description(weather.code));
        /* 天气代码变化时才标记图标区域重绘，避免每轮都重画。 */
        if (s_view.weather.icon_code != weather.code)
        {
            s_view.weather.icon_code = weather.code;
            lv_obj_invalidate(s_view.weather.icon);
        }
        /* 数据来自缓存时提示需通过蓝牙更新。 */
        if (weather.cached != 0)
        {
            ui_set_label_text_fmt(s_view.weather.label, "%s%ld.%ldC / RH %u%%\nCACHED / UPDATE VIA BLE", sign,
                               (long)(magnitude / 10), (long)(magnitude % 10), (unsigned)weather.humidity_rh);
        }
        /* 非缓存时显示相对收到时刻的分钟数。 */
        else
        {
            /* lv_tick_elaps返回距received_ms的毫秒差，除60000换算成分钟。 */
            ui_set_label_text_fmt(s_view.weather.label, "%s%ld.%ldC / RH %u%%\n%lu min ago", sign,
                               (long)(magnitude / 10), (long)(magnitude % 10), (unsigned)weather.humidity_rh,
                               (unsigned long)(lv_tick_elaps(weather.received_ms) / 60000));
        }
        /* 缓存数据或超过1小时未更新时，文字降为灰色提示不再新鲜。 */
        if (weather.cached != 0 || lv_tick_elaps(weather.received_ms) >= 3600000)
        {
            ui_set_text_color(s_view.weather.label, UI_MUTED);
            ui_set_text_color(s_view.weather.condition, UI_MUTED);
        }
        /* 新鲜数据用正常文字颜色。 */
        else
        {
            ui_set_text_color(s_view.weather.label, UI_TEXT);
            ui_set_text_color(s_view.weather.condition, UI_TEXT);
        }
    }
}

/* 电量与步数摘要：valid才显示电压/百分比，否则按result区分"--%"与"ADC ERR"；
 * 电压单位mV，表盘页额外显示V(保留两位小数)，低电量阈值20%改红色。
 * 刷新电量时一并取IMU快照显示表盘步数。 */
static void refresh_battery(void)
{
    /* 本次读取的电量快照，含百分比、电压mV和采样结果。 */
    app_battery_struct battery;
    /* 电量条指示器颜色，默认电池绿色。 */
    uint32_t color = UI_BATTERY_GREEN;
    /* 电量标签未创建(不在本页)时直接返回。 */
    if (s_view.battery.label == NULL)
    {
        return;
    }
    /* 只有表盘页才有步数标签，存在时顺便刷新步数。 */
    if (s_view.watch.steps != NULL)
    {
        /* 步数来自IMU快照的计步累计值。 */
        app_imu_struct imu;
        App_IMU_GetSnapshot(&imu);
        /* 步数以无符号十进制显示。 */
        ui_set_label_text_fmt(s_view.watch.steps, "%lu", (unsigned long)imu.steps);
    }
    /* 取一次电量快照。 */
    app_battery_get_snapshot(&battery);
    /* 快照有效时更新电量条和文字。 */
    if (battery.valid)
    {
        /* 电量条此前被隐藏时先恢复显示。 */
        if (lv_obj_has_flag(s_view.battery.bar, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(s_view.battery.bar, LV_OBJ_FLAG_HIDDEN);
        }
        /* 把百分比写入进度条，关闭动画。 */
        lv_bar_set_value(s_view.battery.bar, battery.percent, LV_ANIM_OFF);
        /* 表盘页同时显示百分比和电压，电压由mV拆成V与两位小数。 */
        if (s_page == PAGE_WATCH)
        {
            /* voltage_mv/1000得整数伏，%1000/10得两位小数的毫伏部分。 */
            ui_set_label_text_fmt(s_view.battery.label, "~%u%% %lu.%02luV", (unsigned int)battery.percent,
                               (unsigned long)(battery.voltage_mv / 1000), (unsigned long)(battery.voltage_mv % 1000 / 10));
        }
        /* 其他页只显示百分比，节省顶部空间。 */
        else
        {
            ui_set_label_text_fmt(s_view.battery.label, "~%u%%", (unsigned int)battery.percent);
        }
        /* 电量不高于20%时文字和电量条都改为红色。 */
        if (battery.percent <= 20)
        {
            color = UI_RED;
            ui_set_text_color(s_view.battery.label, UI_RED);
        }
        /* 电量正常时文字用普通颜色，电量条保持绿色。 */
        else
        {
            ui_set_text_color(s_view.battery.label, UI_TEXT);
        }
        /* 仅颜色变化时设置样式，保持电量不变时不重复刷屏。 */
        /* 比较指示器当前颜色与目标颜色的完整颜色值，相同就跳过设置。 */
        if (lv_obj_get_style_bg_color(s_view.battery.bar, LV_PART_INDICATOR).full != lv_color_hex(color).full)
        {
            lv_obj_set_style_bg_color(s_view.battery.bar, lv_color_hex(color), LV_PART_INDICATOR);
        }
    }
    /* 快照无效时隐藏电量条，文字按采样结果区分。 */
    else
    {
        /* 电量条尚未隐藏时才隐藏，避免重复设置。 */
        if (!lv_obj_has_flag(s_view.battery.bar, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_add_flag(s_view.battery.bar, LV_OBJ_FLAG_HIDDEN);
        }
        /* 采样本身成功但数据尚不可用时只显示"--%"。 */
        if (battery.result == HAL_OK)
        {
            ui_set_label_text(s_view.battery.label, "--%");
            ui_set_text_color(s_view.battery.label, UI_MUTED);
        }
        /* ADC采样失败时显示错误文字。 */
        else
        {
            ui_set_label_text(s_view.battery.label, "ADC ERR");
            ui_set_text_color(s_view.battery.label, UI_RED);
        }
    }
}

/* 运动页独立读取和刷新自己的快照，页面表决定何时调用。 */
static void refresh_motion(void)
{
    app_imu_struct imu;
    uint8_t online;
    /* 取IMU一致快照，返回值只表示完整姿态是否在线；纯加速度档raw/steps仍可能有效。 */
    online = App_IMU_GetSnapshot(&imu);
    /* 顶部状态标签显示ONLINE或NO DATA。 */
    ui_set_status(s_view.motion.status, online);
    /* 罗盘指针、角度和状态文字由compass_update统一刷新。 */
    compass_update(&imu, online);
    /* 角度三轴标签已创建时才刷新角度区。 */
    if (s_view.motion.angles[0] != NULL)
    {
        /* 完整姿态在线时显示俯仰、横滚、偏航角。 */
        if (online)
        {
            /* 俯仰和横滚按一位小数显示。 */
            ui_set_decimal(s_view.motion.angles[0], imu.pitch_deg, 0);
            ui_set_decimal(s_view.motion.angles[1], imu.roll_deg, 0);
            /* 偏航角按角度环绕格式显示，折到(-180,180]。 */
            ui_set_decimal(s_view.motion.angles[2], imu.yaw_deg, 1);
        }
        /* 完整姿态未在线时三轴都显示占位符。 */
        else
        {
            ui_set_label_text(s_view.motion.angles[0], "--");
            ui_set_label_text(s_view.motion.angles[1], "--");
            ui_set_label_text(s_view.motion.angles[2], "--");
        }
        /* 航向来源说明：完整姿态在线时按融合状态翻译，未在线时沿用现有占位文案。 */
        if (online)
        {
            ui_set_label_text(s_view.motion.yaw_source, mag_yaw_status(&imu));
        }
        else
        {
            ui_set_label_text(s_view.motion.yaw_source, "GYRO ONLY / NO IMU FRAME");
        }
        /* 磁力计状态标签按校准进度分支显示。 */
        if (imu.mag_cal_state == APP_MAG_CAL_RUNNING)
        {
            /* 正在采集校准数据。 */
            ui_set_label_text(s_view.motion.mag_status, "MAG CALIBRATING");
        }
        else if (imu.mag_cal_state == APP_MAG_CAL_FAILED)
        {
            /* 校准失败，提示旋转所有轴重试。 */
            ui_set_label_text(s_view.motion.mag_status, "CAL FAILED / rotate all axes");
        }
        else if (imu.mag_cal_state == APP_MAG_CAL_DONE)
        {
            /* 校准完成但EEPROM保存失败时优先提示保存问题。 */
            if (imu.mag_persist_state == APP_MAG_PERSIST_SAVE_FAILED)
            {
                ui_set_label_text(s_view.motion.mag_status, "CAL OK / SAVE FAILED");
            }
            /* 校准来自EEPROM恢复时，再按磁力计是否在线区分。 */
            else if (imu.mag_persist_state == APP_MAG_PERSIST_RESTORED)
            {
                if (imu.mag_online)
                {
                    ui_set_label_text(s_view.motion.mag_status, "EEPROM CAL / MAG ONLINE");
                }
                else
                {
                    ui_set_label_text(s_view.motion.mag_status, "EEPROM CAL / MAG NO DATA");
                }
            }
            /* 本次新完成的校准，同样按磁力计在线状态显示。 */
            else
            {
                if (imu.mag_online)
                {
                    ui_set_label_text(s_view.motion.mag_status, "CAL OK / MAG ONLINE");
                }
                else
                {
                    ui_set_label_text(s_view.motion.mag_status, "CAL OK / MAG NO DATA");
                }
            }
        }
        /* 未校准(含未运行)时只按磁力计在线状态显示。 */
        else
        {
            if (imu.mag_online)
            {
                ui_set_label_text(s_view.motion.mag_status, "LSM303DLHC / ONLINE");
            }
            else
            {
                ui_set_label_text(s_view.motion.mag_status, "LSM303DLHC / NO DATA");
            }
        }
        /* 磁力计在线且没有保存失败时用绿色，否则红色。 */
        if (imu.mag_online && imu.mag_persist_state != APP_MAG_PERSIST_SAVE_FAILED)
        {
            ui_set_text_color(s_view.motion.mag_status, UI_GREEN);
        }
        else
        {
            ui_set_text_color(s_view.motion.mag_status, UI_RED);
        }
        /* 校准按钮文字随状态变化：采集中文案改为结束校准。 */
        if (imu.mag_cal_state == APP_MAG_CAL_RUNNING)
        {
            ui_set_label_text(s_view.motion.cal_label, "FINISH CALIBRATION");
        }
        else
        {
            ui_set_label_text(s_view.motion.cal_label, "CALIBRATE MAG");
        }
        /* 校准状态标签显示结果、样本数和三轴跨度。 */
        mag_calibration_status(&imu);
        {
            /* 三轴磁力计数值的下标。 */
            uint8_t i;
            /* 逐轴刷新磁力计数值。 */
            for (i = 0; i < 3; ++i)
            {
                /* 磁力计在线时显示各轴磁场，单位uT。 */
                if (imu.mag_online)
                {
                    ui_set_decimal(s_view.motion.mag_values[i], imu.mag_body_uT[i], 0);
                }
                /* 离线时显示占位符。 */
                else
                {
                    ui_set_label_text(s_view.motion.mag_values[i], "--");
                }
            }
        }
    }
}

static void refresh_baro(void)
{
    app_baro_struct baro;
    uint8_t online;
    /* 气压页控件存在，且处于等待校准或距上次刷新已满UI_BARO_REFRESH_MS时刷新气压区；这里不触发气压采样。 */
    if (s_view.baro.status != NULL && (s_view.baro.calibration_waiting != 0 || lv_tick_elaps(s_view.baro.refresh_tick) >= UI_BARO_REFRESH_MS))
    {
        /* 记录本次刷新tick，作为下一次限流基准。 */
        s_view.baro.refresh_tick = lv_tick_get();
        /* 取气压一致快照，返回值同时作为在线标志。 */
        online = App_Baro_GetSnapshot(&baro);
        /* 同步等待标志；处于PENDING时绕过5秒节流，确保校准完成立刻刷新显示。 */
        s_view.baro.calibration_waiting = baro.calibration_state == APP_BARO_CAL_PENDING;
        /* 按运动模式、校准状态及手机参考/温补状态更新底部校准提示文案。 */
        if (s_sport_mode != 0)
        {
            ui_set_label_text(s_view.baro.calibration_status, "EXIT SPORT TO CALIBRATE");
        }
        else if (baro.calibration_state == APP_BARO_CAL_PENDING)
        {
            ui_set_label_text(s_view.baro.calibration_status, "CALIBRATING...");
        }
        else if (baro.calibration_state == APP_BARO_CAL_FAILED)
        {
            ui_set_label_text(s_view.baro.calibration_status, "READ FAILED / RETRY");
        }
        else if (baro.calibration_state == APP_BARO_CAL_APPLIED)
        {
            if (baro.temperature_corrected != 0)
            {
                ui_set_label_text(s_view.baro.calibration_status, "CALIBRATED / PHONE TEMP");
            }
            else
            {
                ui_set_label_text(s_view.baro.calibration_status, "CALIBRATED / STANDARD");
            }
        }
        else if (baro.reference_ready == 0)
        {
            ui_set_label_text(s_view.baro.calibration_status, "SYNC PHONE TO CALIBRATE");
        }
        else if (baro.temperature_corrected != 0)
        {
            ui_set_label_text(s_view.baro.calibration_status, "PHONE TEMP / READY");
        }
        else
        {
            ui_set_label_text(s_view.baro.calibration_status, "STANDARD MODEL / READY");
        }
        /* 运动模式期间暂停气压显示。 */
        if (s_sport_mode != 0)
        {
            /* 强制按离线处理，状态文字写PAUSED。 */
            online = 0;
            ui_set_label_text(s_view.baro.status, "PAUSED");
        }
        /* 非运动模式按online标志显示ONLINE或NO DATA。 */
        else
        {
            ui_set_status(s_view.baro.status, online);
        }
        /* 用气压模块维护的历史快照更新海拔趋势图。 */
        baro_history_update(&baro, online);
        /* 气压数值标签存在时才刷新气压和海拔。 */
        if (s_view.baro.pressure != NULL)
        {
            /* 在线且气压为正时显示气压和海拔。 */
            if (online && baro.pressure_pa > 0)
            {
                /* pressure_pa单位Pa，除以100得hPa整数部分，模100得两位小数。 */
                ui_set_label_text_fmt(s_view.baro.pressure, "%ld.%02ld", (long)(baro.pressure_pa / 100), (long)(baro.pressure_pa % 100));
                /* 海拔按一位小数显示，单位米。 */
                ui_set_decimal(s_view.baro.altitude, baro.alt_m, 0);
            }
            /* 离线或气压值非法时两项都显示占位符。 */
            else
            {
                ui_set_label_text(s_view.baro.pressure, "--");
                ui_set_label_text(s_view.baro.altitude, "--");
            }
        }
    }
}

/* 公共顶部电量与当前页面分开刷新；表盘内部维护自己的电量刷新节拍。 */
static void refresh_data(void)
{
    if (s_page != PAGE_WATCH)
    {
        refresh_battery();
    }
    if (s_pages[s_page].refresh != NULL)
    {
        s_pages[s_page].refresh();
    }
}

void App_UI_Back(void)
{
    /* 只挂起返回请求；实际切页由ui_timer_cb在松手且动画结束后执行。
     * 仅限LVGL任务，或持有App_Display_Lock()时调用。 */
    /* 目标页设为PAGE_COUNT，表示当前没有等待切换的普通页面，只走返回分支。 */
    s_pending_page = PAGE_COUNT;
    /* 置位返回挂起标志，等ui_timer_cb确认松手且无动画后从历史栈回退。 */
    s_pending_back = 1;
}

/* PA5按键消抖：UI_KEY_MS内电平稳定才认一次边沿。按下只记录"已武装"并在亮屏时
 * 重置活动计时；松开且武装过才触发返回，避免开机按住被当成一次返回操作。 */
static void poll_key(void)
{
    /* 读取PA5原始电平并换算成按下语义：低电平为按下。 */
    uint8_t pressed = watch_key_is_back_pressed();
    /* 原始电平变化时记录新值，并把消抖计时起点更新到当前tick。 */
    if (pressed != s_key_raw)
    {
        s_key_raw = pressed;
        s_key_tick = lv_tick_get();
    }
    /* 原始电平与稳定电平不同且已持续UI_KEY_MS，才把新电平确认为稳定状态。 */
    if (s_key_raw != s_key_stable && lv_tick_elaps(s_key_tick) >= UI_KEY_MS)
    {
        s_key_stable = s_key_raw;
        /* 稳定为按下：记录本次按下是否发生在亮屏时，作为松开能否返回的条件。 */
        if (s_key_stable)
        {
            s_key_armed = s_screen_on;
            if (s_screen_on)
            {
                lv_disp_trig_activity(NULL); /* 亮屏按键操作也重新计时，避免返回过程中自动熄屏。 */
            }
        }
        /* 稳定为松开：若之前按下时已武装，则清除武装并在亮屏时触发一次返回。 */
        else if (s_key_armed)
        {
            /* 先清除武装，保证一次按下只产生一次返回。 */
            s_key_armed = 0;
            if (s_screen_on)
            {
                /* 亮屏下松开：刷新活动计时后提交返回请求。 */
                lv_disp_trig_activity(NULL);
                App_UI_Back();
            }
        }
    }
}

/* 保连(非ECO)不随屏幕切电；ECO在熄屏后等初始化、命令服务、TX与RX都空闲，
 * 再留UI_BT_ECO_QUIET_MS安静期才真正断电，期间任何活动都会重置计时。 */
/* 亮灭屏统一入口：熄灭时先关背光、停触摸采集、按已熄屏发布状态并暂停刷新定时器；
 * 随后请求LCD休眠/唤醒，失败只保留目标值，下一轮UI服务继续重试。熄灭路径在请求前
 * 就更新s_screen_on，亮屏则要等唤醒命令成功后才更新，并同步背光、触摸使能和刷新
 * 定时器。亮屏还要丢弃触摸残留按下，要求先松手再响应，避免熄屏期间的触点变成误点击。 */
static void set_screen_on(uint8_t screen_on)
{
    /* 触摸设备遍历指针，用于在函数末尾统一复位/使能。 */
    lv_indev_t *indev;

    /* 先记录本次屏幕目标状态，电源请求失败时由UI服务按它重试。 */
    s_screen_requested = screen_on;
    /* 由亮转灭：先按“已熄屏”发布状态并停下显示侧活动，再请求LCD休眠。 */
    if (screen_on == 0 && s_screen_on != 0)
    {
        /* 先把逻辑状态改为熄屏，后面的状态发布按新值计算。 */
        s_screen_on = 0;
        /* 关背光并停止触摸采集。 */
        LCD_Set_Light(0);
        app_touch_set_enabled(0);
        /* 按熄屏状态发布电源标志，并暂停显示刷新定时器。 */
        publish_power_state();
        lv_timer_pause(_lv_disp_get_refr_timer(NULL));
    }
    /* 亮屏命令成功后才发布亮屏；失败时保留目标，下一轮UI服务继续重试。 */
    s_screen_power_pending = 1;
    /* 向显示驱动请求休眠(目标为灭)或唤醒(目标为亮)，非0表示本次未完成。 */
    if (lv_port_disp_set_sleep(screen_on == 0) != 0)
    {
        /* 驱动未完成休眠/唤醒：保留目标与挂起标志，直接结束本次调用。 */
        return;
    }
    /* 电源命令已完成，清除挂起标志并同步逻辑屏幕状态。 */
    s_screen_power_pending = 0;
    s_screen_on = screen_on;
    /* 屏幕状态变化后重置蓝牙ECO安静计时，并按新状态评估蓝牙供电。 */
    app_ble_update_power(s_screen_on, s_sport_mode, 1);
    /* 触摸采集与屏幕状态保持一致：亮屏才允许采集。 */
    app_touch_set_enabled(s_screen_on);
    /* 亮屏时再发布一次状态，保留同一轮熄屏再抬腕亮屏的触摸唤醒通知。 */
    if (screen_on != 0)
    {
        publish_power_state(); /* 保留同一轮熄屏再抬腕亮屏的触摸唤醒通知。 */
    }
    /* 屏幕状态已切换为亮：刷新活动时间、恢复刷新定时器并标记到期，供后续lv_timer_handler尽快处理。 */
    if (s_screen_on)
    {
        /* 刷新无操作计时，避免刚亮屏就因旧计时再次熄屏。 */
        lv_disp_trig_activity(NULL);
        /* 恢复刷新定时器并标记到期，供后续lv_timer_handler尽快处理。 */
        lv_timer_resume(_lv_disp_get_refr_timer(NULL));
        lv_timer_ready(_lv_disp_get_refr_timer(NULL));
        /* 清掉已记录的秒值，并把时钟/数据刷新基准回拨，使亮屏后的首轮UI服务立即刷新。 */
        s_view.watch.last_second = UINT32_MAX;
        s_view.watch.refresh_tick = lv_tick_get() - 1000;
        s_refresh_tick = lv_tick_get() - UI_REFRESH_MS;
        /* 标脏整屏，要求LVGL重绘当前页面。 */
        lv_obj_invalidate(s_screen);
        /* 恢复用户设定的背光亮度。 */
        LCD_Set_Light(s_brightness);
    }
    else
    {
        /* 屏幕状态已切换为灭：关闭背光并暂停刷新定时器。 */
        LCD_Set_Light(0);
        lv_timer_pause(_lv_disp_get_refr_timer(NULL));
    }
    /* 熄屏不接收触摸；亮屏时等待原触点松开，避免误点击。 */
    /* 从链表头遍历所有输入设备。 */
    for (indev = lv_indev_get_next(NULL); indev != NULL; indev = lv_indev_get_next(indev))
    {
        /* 只处理指针类输入设备，按键等其它类型不在此复位。 */
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER)
        {
            /* 请求复位输入处理状态，并清除残留目标对象引用。 */
            lv_indev_reset(indev, NULL);
            /* 按新的屏幕状态启用(亮屏)或禁用(熄屏)该输入设备。 */
            lv_indev_enable(indev, s_screen_on != 0);
            /* 亮屏后先要求松手，本次残留按压不会直接作用到控件。 */
            if (s_screen_on)
            {
                lv_indev_wait_release(indev);
            }
        }
    }
}

/* PA4 上升沿中断由 HAL 清除；UI 任务扫描电平，按下和松开均消抖，不在 ISR 操作屏幕。 */
static void poll_wake_key(void)
{
    /* 读取PA4原始电平并换算成按下语义：高电平为按下。 */
    uint8_t pressed = watch_key_is_wake_pressed();

    /* 原始电平变化时记录新值，并把消抖计时起点更新到当前tick。 */
    if (pressed != s_wake_raw)
    {
        s_wake_raw = pressed;
        s_wake_tick = lv_tick_get();
    }
    /* 原始电平与稳定电平不同且已持续UI_KEY_MS，才确认新的稳定状态。 */
    if (s_wake_raw != s_wake_stable && lv_tick_elaps(s_wake_tick) >= UI_KEY_MS)
    {
        s_wake_stable = s_wake_raw;
        /* 只对按下沿动作：把当前目标状态取反，在亮屏与熄屏之间切换。 */
        if (s_wake_stable)
        {
            set_screen_on(!s_screen_requested);
        }
    }
}

/* 辅助抬腕：sensor每次判定抬腕都会递增wrist_raise_sequence；这里只在序号变化且
 * 事件发生在最近800ms内才亮屏，避免sensor长时间停更后补亮一次过期事件。 */
static void poll_wrist_raise(void)
{
    /* 取一次IMU快照，读取其中的抬腕序号和事件时间戳。 */
    app_imu_struct imu;
    /* 把sensor侧快照复制到本地栈变量，后续判断只用这一份。 */
    App_IMU_GetSnapshot(&imu);
    /* 序号与上次记录不同，说明sensor判定过新的抬腕事件。 */
    if (imu.wrist_raise_sequence != s_wrist_sequence)
    {
        /* 先记录新序号，同一事件不会被后续轮次重复触发。 */
        s_wrist_sequence = imu.wrist_raise_sequence;
        /* 只在当前熄屏且事件时间距今不超过800ms时点亮，过期事件不补亮。 */
        if (s_screen_on == 0 && (uint32_t)(HAL_GetTick() - imu.wrist_raise_ms) <= 800)
        {
            /* 满足条件即请求亮屏，背光与刷新由set_screen_on统一处理。 */
            set_screen_on(1);
        }
    }
}

static uint8_t touch_pressed(void)
{
    /* 从链表头开始遍历输入设备。 */
    lv_indev_t *indev = NULL;
    /* 逐个取下一个输入设备，直到链表结束。 */
    while ((indev = lv_indev_get_next(indev)) != NULL)
    {
        /* 指针类设备当前处于按下状态即返回1，不继续检查其余设备。 */
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER && indev->proc.state == LV_INDEV_STATE_PRESSED)
        {
            return 1;
        }
    }
    /* 遍历完没有指针设备处于按下状态，返回0。 */
    return 0;
}

/* 触摸(含持续按住)刷新活动时间；超时后与PA4一样走完整熄屏流程。 */
static void poll_touch_idle(void)
{
    /* 仅亮屏时检查LVGL记录的无操作时长，超过UI_TOUCH_IDLE_MS就走熄屏流程。 */
    if (s_screen_on != 0 && lv_disp_get_inactive_time(NULL) >= UI_TOUCH_IDLE_MS)
    {
        /* 请求熄屏，背光、触摸和刷新定时器由set_screen_on统一关闭。 */
        set_screen_on(0);
    }
}

/* 把UI状态打包成电源标志：熄屏即idle，姿态页且亮屏才算motion，运动模式独立。
 * 只发布，不在此处操作硬件。 */
static void publish_power_state(void)
{
    /* 熄屏即视为idle；motion要求未idle且当前页为姿态页。 */
    uint8_t idle = s_screen_on == 0;
    /* screen_off与idle同源，sport直接透传当前运动模式标志。 */
    app_task_set_power_state(idle, idle == 0 && s_page == PAGE_MOTION, s_screen_on == 0, s_sport_mode);
}

static void ui_timer_cb(lv_timer_t *timer)
{
    /* 保存从历史栈取出的上一页信息（页码与滚动位置）。 */
    ui_history_t previous;
    /* 保存本轮待切换的目标页。 */
    ui_page_t next;
    /* 定时器回调不使用句柄参数，显式丢弃以避免未使用参数告警。 */
    (void)timer;
    /* 上一轮屏幕电源请求尚未完成时，按目标状态重试一次，失败会保留挂起标志。 */
    if (s_screen_power_pending != 0)
    {
        set_screen_on(s_screen_requested);
    }
    /* 本轮先取一次串口数据；该函数同时分发各解析器并维护显示日志。 */
    if (app_ble_service(s_screen_on, s_sport_mode) != 0)
    {
        lv_disp_trig_activity(NULL);
    }
    /* 依次扫描PA4唤醒键、PA5返回键、触摸空闲、抬腕事件。 */
    poll_wake_key();
    poll_key();
    poll_touch_idle();
    poll_wrist_raise();
    /* 把最新UI状态同步给任务电源管理，并按ECO规则更新蓝牙供电。 */
    publish_power_state();
    app_ble_update_power(s_screen_on, s_sport_mode, 0);
    /* Rebuild only after release and after the previous animation, so event
     * targets stay valid and a gesture cannot turn into a card click. */
    /* 只有亮屏、无切页动画且没有触摸按压时才执行挂起的切页请求。 */
    if (s_screen_on && !s_transition && !touch_pressed())
    {
        /* 返回请求优先于普通目标页请求。 */
        if (s_pending_back)
        {
            /* 清除返回挂起并复位目标页，避免同一次返回被后续轮次重复处理。 */
            s_pending_back = 0;
            s_pending_page = PAGE_COUNT;
            /* 历史栈非空时弹出一页，按记录的页码和滚动位置重建，并带切页动画。 */
            if (s_history_count != 0)
            {
                /* 先把计数减一，再取出被弹出的栈顶记录。 */
                previous = s_history[--s_history_count];
                create_page(previous.page, previous.scroll_y, 1);
            }
        }
        /* 没有返回请求时，处理普通的目标页切换。 */
        else if (s_pending_page != PAGE_COUNT)
        {
            /* 先取出目标页并立即清空挂起值，本轮只处理一次切换。 */
            next = s_pending_page;
            s_pending_page = PAGE_COUNT;
            /* 目标是设置页且当前位于其子页面：右滑固定回功能页。 */
            if (next == PAGE_SETTINGS && s_page > PAGE_SETTINGS)
            {
                /* 右滑固定回功能页；来自网格时取回滚动位置，不把详情压入历史。 */
                previous.scroll_y = 0;
                /* 栈顶正好是设置页时弹出它并复用其滚动位置，避免重复入栈。 */
                if (s_history_count != 0 && s_history[s_history_count - 1].page == PAGE_SETTINGS)
                {
                    previous = s_history[--s_history_count];
                }
                /* 按恢复出的滚动位置重建设置页，并带反向切页动画。 */
                create_page(PAGE_SETTINGS, previous.scroll_y, 1);
            }
            /* 目标页与当前页不同才需要切页并维护历史栈。 */
            else if (next != s_page)
            {
                /* 历史栈已满时整体前移一格，丢弃最旧记录，为本次入栈腾出位置。 */
                if (s_history_count == UI_HISTORY)
                {
                    memmove(s_history, s_history + 1, sizeof(s_history[0]) * (UI_HISTORY - 1));
                    --s_history_count;
                }
                /* 记录当前页及其滚动位置，供返回时恢复。 */
                s_history[s_history_count].page = s_page;
                s_history[s_history_count].scroll_y = lv_obj_get_scroll_y(s_body);
                /* 入栈完成后计数加一，指向下一个空槽位。 */
                ++s_history_count;
                /* 新建目标页；仅当两页都在功能页范围内且目标页序号小于当前页序号时，动画参数传1。 */
                create_page(next, 0, next <= PAGE_SETTINGS && s_page <= PAGE_SETTINGS && next < s_page);
            }
        }
    }
    /* 亮屏、无动画且到达UI_REFRESH_MS周期时刷新数据，并更新下一次刷新基准。 */
    if (s_screen_on && !s_transition && lv_tick_elaps(s_refresh_tick) >= UI_REFRESH_MS)
    {
        s_refresh_tick = lv_tick_get();
        refresh_data();
    }
}

uint8_t App_UI_IsScreenOn(void)
{
    /* 返回当前逻辑亮屏标志：非0为亮屏，0为熄屏。 */
    return s_screen_on;
}

uint8_t App_UI_IsWakeKeyActive(void)
{
    /* 消抖未稳定，或稳定状态为按下时，都认为PA4按键仍处于活动状态。 */
    return s_wake_raw != s_wake_stable || s_wake_stable != 0;
}

/* 熄屏服务周期：仍重试挂起的屏幕电源，并继续消费UART、回收发送、消抖PA4/PA5与处理抬腕；
 * 不运行LVGL动画/刷新，也不触发切页与数据重绘。 */
void App_UI_ServiceSleeping(void)
{
    /* 熄屏服务入口：上一轮屏幕电源请求未完成时按目标状态重试一次。 */
    if (s_screen_power_pending != 0)
    {
        set_screen_on(s_screen_requested);
    }
    /* 继续消费串口接收，并推进管理回执、开机时长及可选心率波形发送，熄屏期间收到的数据仍会保存。 */
    if (app_ble_service(s_screen_on, s_sport_mode) != 0)
    {
        lv_disp_trig_activity(NULL);
    }
    /* 熄屏期间仍扫描PA4唤醒键、PA5返回键和抬腕事件；PA4与抬腕可请求改变屏幕状态，PA5只返回并重置活动，不切亮灭。 */
    poll_wake_key();
    poll_key();
    poll_wrist_raise();
    /* 同步电源状态并评估蓝牙ECO供电。 */
    publish_power_state();
    app_ble_update_power(s_screen_on, s_sport_mode, 0);
}

/* 首次建UI：boot screen仅保存原活动启动屏，create_page内用lv_obj_create(NULL)独立创建顶层表盘，
 * 加载后删除原启动屏；读取PA5/PA4初始电平，让开机时按住的键必须先松开才生效；
 * 最后建UI_POLL_MS定时器并发布初始电源状态。 */
void App_UI_Create(void)
{
    /* 临时保存启动时的原活动屏幕，表盘加载完成后删除它。 */
    lv_obj_t *boot_screen;
    /* 屏幕已创建过则直接返回，避免重复建表盘和定时器。 */
    if (s_screen != NULL)
    {
        return;
    }
    /* 取当前活动屏幕保存为原启动屏，表盘在create_page内独立创建。 */
    boot_screen = lv_scr_act();
    /* 表盘页在create_page内独立创建顶层屏幕，首次建屏不做切页动画。 */
    create_page(PAGE_WATCH, 0, 0);
    /* 表盘已切换为活动屏幕，删除不再需要的原启动屏。 */
    lv_obj_del(boot_screen);
    /* 按保存的亮度点亮背光。 */
    LCD_Set_Light(s_brightness);
    /* A key held during boot must be released before it can trigger Back. */
    /* 把PA5当前电平同时写入原始和稳定状态，开机时按住的键要先松开才会产生边沿。 */
    s_key_raw = s_key_stable = watch_key_is_back_pressed();
    /* 开机时按住 SW2 只用于上电，松开不立即熄屏。 */
    s_wake_raw = s_wake_stable = watch_key_is_wake_pressed();
    /* 三个tick基准统一取当前时刻，作为按键消抖和周期刷新的起始计时。 */
    s_wake_tick = s_key_tick = s_refresh_tick = lv_tick_get();
    /* 触发一次活动计时，使刚建好的界面从当前时刻开始计算熄屏超时。 */
    lv_disp_trig_activity(NULL);
    /* 建立UI轮询定时器，周期UI_POLL_MS，回调为ui_timer_cb。 */
    lv_timer_create(ui_timer_cb, UI_POLL_MS, NULL);
    /* 建屏完成后发布一次初始电源状态。 */
    publish_power_state();
}
