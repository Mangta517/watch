/**
  ******************************************************************************
  * @file    lv_port_indev_template.c
  * @brief   LVGL v8 输入移植层 — 接 interface/touch.c (CST816, 软件 I2C)
  *          移植自 Watch 工程同名文件 (源头 FryPi OV_Watch lv_port_indev.c,
  *          GPL-3.0)。本工程无 TP_INT 引脚, 驱动纯轮询; 其余接法不变。
  ******************************************************************************
***/
#include "lv_port_indev_template.h"
#include "lvgl.h"
#include "touch.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_init(void);
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

    touchpad_init();

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
static void touchpad_init(void)
{
    (void)Touch_Init(); /* 返回 ChipID (CST816S≈0xB5); 0 表示总线未见器件 */
}

static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;
    uint16_t tx = 0, ty = 0;

    (void)indev_drv;
    if (Touch_GetPoint(&tx, &ty))
    { /* 一次调用即判压+取坐标 */
        last_x = (lv_coord_t)tx;
        last_y = (lv_coord_t)ty;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED; /* 坐标保持 last_x/y 不变 */
    }
    data->point.x = last_x;
    data->point.y = last_y;
}
