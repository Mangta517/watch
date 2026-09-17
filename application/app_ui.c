/**
  ******************************************************************************
  * @file    app_ui.c
  * @brief   应用层界面 (240x280 竖屏, 字体启用 Montserrat 10/14/30)
  *          当前页 = 传感器显示: MPU6050 三轴姿态角(Pitch/Roll/Yaw, 融合解算) + SPL006 气压(hPa)。
  *          数据只经应用层快照服务 App_IMU_GetSnapshot()/App_Baro_GetSnapshot() 获取,
  *          本文件不直调 my_mpu6050 / spl006 系列接口 (总线归口 sensor 任务, 见 app_task.c)。
  *          刷新: lv_timer 100ms —— 回调与 lv_timer_handler 同在 lvgl 任务上下文,
  *          无跨任务调 LVGL 的问题; 快照读取加互斥, 与 sensor 任务生产者解耦。
  *          旧 spinbox 教程页、tabview 版 Clock 页均整体 #if 0 保留备用。
  ******************************************************************************
***/
#include "app_ui.h"

#include "lvgl.h"
#include "lcd.h"          /* LCD_Set_Light: 背光档位 */
#include "FreeRTOS.h"
#include "task.h"         /* #if 0 保留的 Clock 演示仍引用 tick 时基 */
#include "app_imu.h"      /* IMU 快照服务 */
#include "app_baro.h"     /* 气压快照服务 */

/*********************
 *  参数
 *********************/
/* UI 重画周期(ms): 生产端 IMU 10ms / 气压 30ms, 100ms 抽帧显示绰绰有余;
 * 再快只是刷总线锁, 没有新数据可看 */
#define UI_SENSOR_REFRESH_MS  100

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t * s_imu_lbl;
static lv_obj_t * s_baro_lbl;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void sensor_tick_cb(lv_timer_t *t);

/* 获取屏幕宽高: App_UI_Create 里第一次 lv_timer_handler 尚未跑,
 * 布局未刷新, lv_obj_get_width(lv_scr_act()) 读到的是 0, 拿它算尺寸
 * 会得到 0x0 的窗口(居中自然"看不出效果")。改读显示驱动注册的分辨率。 */
#define scr_act_width()  lv_disp_get_hor_res(NULL)
#define scr_act_height() lv_disp_get_ver_res(NULL)

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_UI_Create(void)
{
    lv_obj_t * title;
    lv_obj_t * cap;

    lv_obj_set_scrollbar_mode(lv_scr_act(), LV_SCROLLBAR_MODE_OFF); /* 关闭滚动条 */

    title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "SENSORS");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    cap = lv_label_create(lv_scr_act());                            /* 小标题: MPU6050 */
    lv_label_set_text(cap, "MPU6050");
    lv_obj_align(cap, LV_ALIGN_TOP_LEFT, 8, 36);

    s_imu_lbl = lv_label_create(lv_scr_act());
    lv_label_set_text(s_imu_lbl, "--");
    lv_obj_align(s_imu_lbl, LV_ALIGN_TOP_LEFT, 8, 56);

    cap = lv_label_create(lv_scr_act());                            /* 小标题: SPL006 */
    lv_label_set_text(cap, "SPL006");
    lv_obj_align(cap, LV_ALIGN_TOP_LEFT, 8, 128);

    s_baro_lbl = lv_label_create(lv_scr_act());
    lv_label_set_text(s_baro_lbl, "--");
    lv_obj_align(s_baro_lbl, LV_ALIGN_TOP_LEFT, 8, 148);

    lv_timer_create(sensor_tick_cb, UI_SENSOR_REFRESH_MS, NULL);

    LCD_Set_Light(30);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 传感器页刷新: 取快照 → 组文本。全整数定点打印 (规避 %f 依赖):
 * 角度 ×10 保留 1 位小数, 符号单独占位 (防 -0.x 丢负号);
 * 气压 1Pa=0.01hPa, 整数除法/取余即得两位小数。 */
static void sensor_tick_cb(lv_timer_t *t)
{
    app_imu_struct  imu;
    app_baro_struct baro;
    int32_t p10, r10, y10;
    uint8_t ok;

    (void)t;

    ok = App_IMU_GetSnapshot(&imu);
    if (ok == 0)
    {
        lv_label_set_text(s_imu_lbl, "OFFLINE\nSDA=PB13 SCL=PB14");
    }
    else
    {
        p10 = (int32_t)(imu.pitch_deg * 10.0f);
        r10 = (int32_t)(imu.roll_deg * 10.0f);
        /* yaw 是 Z 轴角速度纯积分累加, 无磁参考、数值不设界 (转十圈就是 3600°);
         * 显示层回绕到 (-180,180] 折角域, 解算状态本身不动 */
        y10 = (int32_t)(imu.yaw_deg * 10.0f) % 3600;
        if (y10 > 1800)  y10 -= 3600;
        if (y10 <= -1800) y10 += 3600;
        lv_label_set_text_fmt(s_imu_lbl,
            "Pitch %c%d.%d deg\nRoll  %c%d.%d deg\nYaw   %c%d.%d deg",
            (p10 < 0 ? '-' : '+'), (int)(p10 < 0 ? -p10 : p10) / 10,
                                      (int)(p10 < 0 ? -p10 : p10) % 10,
            (r10 < 0 ? '-' : '+'), (int)(r10 < 0 ? -r10 : r10) / 10,
                                      (int)(r10 < 0 ? -r10 : r10) % 10,
            (y10 < 0 ? '-' : '+'), (int)(y10 < 0 ? -y10 : y10) / 10,
                                      (int)(y10 < 0 ? -y10 : y10) % 10);
    }

    ok = App_Baro_GetSnapshot(&baro);
    if (ok == 0)
    {
        lv_label_set_text(s_baro_lbl, "OFFLINE\nSDO=GND addr 0x76");
    }
    else
    {
        /* Alt 为相对标准海平面气压高度, 会随天气整体漂移 (非 GPS 真高) */
        int32_t a10 = (int32_t)(baro.alt_m * 10.0f);
        if (a10 < 0) a10 = -a10;
        lv_label_set_text_fmt(s_baro_lbl,
            "P   %d.%02d hPa\nAlt %c%d.%d m",
            (int)(baro.pressure_pa / 100), (int)(baro.pressure_pa % 100),
            (baro.alt_m < 0.0f ? '-' : '+'), (int)(a10 / 10), (int)(a10 % 10));
    }
}

/**********************
 *  保留备用页 (#if 0)
 **********************/
/* [教程改版] spinbox+按钮页: 恢复时把 App_UI_Create 主体换回这套控件 */
#if 0
static lv_obj_t *spinbox;
static lv_obj_t * btn1;
static lv_obj_t * btn2;

static void btn1_event_cb(lv_event_t *e)
{
    lv_obj_t *target=lv_event_get_target(e);
    if(target==btn1)
    {
        lv_spinbox_increment(spinbox);
    }
    else if(target==btn2)
    {
        lv_spinbox_decrement(spinbox);
    }
}

static void spinbox_page_create(void)
{
    /* 光标只在 FOCUSED 状态下被主题画出, 而 FOCUSED 状态须由 lv_group 聚焦置位;
     * spinbox 创建时会自动加入"当时的默认分组", 所以分组必须先建好。 */
    if(lv_group_get_default() == NULL) {
        lv_group_set_default(lv_group_create());
    }
    spinbox = lv_spinbox_create(lv_scr_act());
    lv_spinbox_set_range(spinbox, 0, 100);
    lv_spinbox_set_step(spinbox, 5);
    lv_spinbox_set_digit_format(spinbox,4,2);
    lv_spinbox_set_value(spinbox, 50);
    lv_obj_center(spinbox);
    lv_obj_set_size(spinbox, scr_act_width() / 3, scr_act_height() / 6);
    lv_obj_update_layout(spinbox);

    btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, btn1_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(btn1, scr_act_width() / 8, scr_act_height() / 8);
    lv_obj_align_to(btn1, spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, btn1_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(btn2, scr_act_width() / 8, scr_act_height() / 8);
    lv_obj_align_to(btn2, spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
}
#endif

/* [教程改版] tabview 版 Clock 页 (200ms 轮询, 秒变更才重写):
 * 时间源暂用 FreeRTOS tick 模拟 (CubeMX 未启 RTC), 接 RTC 后仅换取时一行。 */
#if 0
#define CLOCK_BASE_SEC  (9 * 3600 + 30 * 60)     /* 模拟起点 09:30:00 */
static lv_obj_t * s_clock_lbl;
static lv_obj_t * s_clock_arc;

static void clock_tick_cb(lv_timer_t *t)
{
    static uint32_t last_ss = 0xFFFFFFFF;
    uint32_t sec, ss;

    (void)t;
    sec = (uint32_t)(xTaskGetTickCount() / (TickType_t)configTICK_RATE_HZ) + CLOCK_BASE_SEC;
    ss  = sec % 60;
    if (ss == last_ss) return;
    last_ss = ss;

    lv_label_set_text_fmt(s_clock_lbl, "%02d:%02d:%02d",
                          (int)((sec / 3600) % 24),
                          (int)((sec / 60) % 60),
                          (int)ss);
    lv_arc_set_value(s_clock_arc, (int16_t)ss);
}
#endif /* 0: 备用页保留 */
