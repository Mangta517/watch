/**
  ******************************************************************************
  * @file    lv_port_indev_template.c
  * @brief   LVGL v8 输入移植层 — 接 interface/touch.c (CST816, 软件 I2C)
  *          移植自 Watch 工程同名文件 (源头 FryPi OV_Watch lv_port_indev.c,
  *          GPL-3.0)。本工程无 TP_INT 引脚, 驱动纯轮询; 其余接法不变。
  *          touch任务采集硬件；本层只在LVGL任务读取缓存事件，不访问I2C。
  ******************************************************************************
***/
#include "lv_port_indev_template.h"
#include "lvgl.h"
#include "app_touch.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t *indev_touchpad;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    /* Accumulate displacement even when CST816 repeats a coordinate between
     * scans. Otherwise LVGL's velocity gate drops slow horizontal swipes. */
    indev_drv.gesture_min_velocity = 0;
    indev_drv.gesture_limit = 40;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    app_touch_struct sample;

    (void)indev_drv;
    app_touch_read(&sample);
    if (sample.canceled != 0)
    {
        lv_indev_wait_release(indev_touchpad);
    }
    if (sample.pressed != 0)
    {
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    data->point.x = (lv_coord_t)sample.x;
    data->point.y = (lv_coord_t)sample.y;
    data->continue_reading = sample.more != 0;
}
