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
static lv_indev_t *indev_touchpad;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_port_indev_init(void)
{
    /* 静态驱动描述符：注册后 LVGL 持有其地址，不能放栈上。 */
    static lv_indev_drv_t indev_drv;

    /* 初始化为默认值，再按指针型输入设备填写。 */
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    /* Accumulate displacement even when CST816 repeats a coordinate between
     * scans. Otherwise LVGL's velocity gate drops slow horizontal swipes. */
    indev_drv.gesture_min_velocity = 0;
    indev_drv.gesture_limit = 40;
    /* 注册后保存句柄，触摸读取里用它作废手势。 */
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    /* 存放本次从应用层缓存取到的事件：坐标、按下态、是否还有后续事件、手势是否作废。 */
    app_touch_struct sample;

    /* 本回调不使用驱动指针，显式丢弃以避免未使用告警。 */
    (void)indev_drv;
    /* 从缓存读取一条事件，不直接访问 CST816 的 I2C 总线。 */
    app_touch_read(&sample);
    /* 缓存报告上一段手势已作废时，让 LVGL 丢弃当前手势状态。 */
    if (sample.canceled != 0)
    {
        lv_indev_wait_release(indev_touchpad);
    }
    /* 按事件里的按下标志转换成 LVGL 的按下/松开状态。 */
    if (sample.pressed != 0)
    {
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    /* 坐标单位为屏幕像素，直接填入 LVGL 的触点位置。 */
    data->point.x = (lv_coord_t)sample.x;
    data->point.y = (lv_coord_t)sample.y;
    /* more=1 表示队列还有事件，LVGL 应在同一拍继续调用本回调读取。 */
    data->continue_reading = sample.more != 0;
}
