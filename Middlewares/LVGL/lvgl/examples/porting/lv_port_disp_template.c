/**
  ******************************************************************************
  * @file    lv_port_disp_template.c
  * @brief   LVGL v8 显示移植层 — 接 interface/lcd.c (ST7789 240x280, SPI1)
  *          结构参照 Watch 工程同名文件 (源头 FryPi OV_Watch lv_port_disp.c,
  *          GPL-3.0): DMA 异步 flush — disp_flush 发起 SPI1_TX DMA 即返回,
  *          DMA 完成中断经 HAL_SPI_TxCpltCallback(lcd.c) 回调 lv_disp_flush_ready,
  *          传输期间 LVGL 任务可渲染另一块缓冲。
  *          缓冲策略: 双 partial buffer, 每块 1/10 屏 ≈ 13.4KB
  *          (双缓冲正是 DMA 异步的前提: A 在搬运, LVGL 渲染 B)。
  *          前提: lv_conf.h 中 LV_COLOR_16_SWAP=1 (SPI 大端字节序)。
  ******************************************************************************
***/
#include "lv_port_disp_template.h"
#include "lvgl.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "task.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);
static void lcd_flush_ready_cb(void);
static void disp_wait(lv_disp_drv_t *disp_drv);

/**********************
 *  STATIC VARIABLES
 **********************/
/* 双块 1/10 屏 partial buffer, 内部 SRAM (.bss): buf_1 DMA 搬运期间 LVGL 渲染 buf_2 */
#define LVGL_BUF_PX (LCD_WIDTH * LCD_HEIGHT / 10)
static lv_color_t buf_1[LVGL_BUF_PX];
static lv_color_t buf_2[LVGL_BUF_PX];

/* 本次在途 flush 的驱动指针: LVGL 保证 flush_ready 前不会再进 flush_cb, 单飞安全 */
static lv_disp_drv_t *volatile s_flush_drv = NULL;
static TaskHandle_t s_flush_task;
static uint8_t s_redraw_pending;
static void (*s_progress_callback)(void);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_port_disp_init(void)
{
    /* 先初始化物理屏 (含刷黑与背光, 均为阻塞路径) */
    disp_init();

    /* DMA 完成通知接线 (在任何 flush_cb 之前) */
    LCD_Set_Flush_Complete_Callback(lcd_flush_ready_cb);

    /* 绘制缓冲描述符：静态存储，登记双缓冲起始地址与单块像素容量。 */
    static lv_disp_draw_buf_t draw_buf_dsc;
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2, LVGL_BUF_PX);

    /* 显示驱动描述符：静态存储，填分辨率、flush/wait 回调与缓冲后注册。 */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = disp_flush;
    disp_drv.wait_cb = disp_wait;
    disp_drv.draw_buf = &draw_buf_dsc;
    lv_disp_drv_register(&disp_drv);
}

void lv_port_disp_set_progress_callback(void (*callback)(void))
{
    /* callback: 每次flush调用底层填充接口后执行的进度钩子，可为NULL表示不通知。 */
    s_progress_callback = callback;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void disp_init(void)
{
    /* 物理屏初始化交给 lcd.c，包含复位、ST7789 寄存器序列与背光开启。 */
    LCD_Init();
}

/* 常规小块异步发送像素，完成由DMA回调上报；大块阻塞回退和失败由驱动完成/服务路径收尾。 */
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    /* 先清空本拍记录的任务句柄，未启动调度器时不会做任务通知等待。 */
    s_flush_task = NULL;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 记录当前（LVGL）任务句柄，供完成中断定向发通知。 */
        s_flush_task = xTaskGetCurrentTaskHandle();
        (void)ulTaskNotifyTake(pdTRUE, 0); /* 清除上一块已完成、无需等待的通知。 */
    }
    /* 记录驱动指针，完成回调据此调用 lv_disp_flush_ready。 */
    s_flush_drv = disp_drv;
    /* 小块设窗口后异步发送像素，大块由底层阻塞分片发送。 */
    LCD_Color_Fill_DMA((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)area->x2, (uint16_t)area->y2, (const uint16_t *)color_p);
    /* 调用底层填充接口后让步并检查喂狗；常规DMA路径下可让sensor运行与像素传输重叠。 */
    if (s_progress_callback != NULL)
    {
        s_progress_callback();
    }
}

/* DMA完成即可唤醒LVGL；等待期间sensor可运行，不额外等待下一个1ms tick。 */
static void disp_wait(lv_disp_drv_t *disp_drv)
{
    /* 本回调不使用驱动指针，显式丢弃以避免未使用告警。 */
    (void)disp_drv;
    /* 先做一次超时/错误收尾；返回非 0 表示本次异常收尾，需要整屏重绘。 */
    if (LCD_Flush_Service() != 0)
    {
        s_redraw_pending = 1;
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 最多等 1 个 tick 的完成通知；已被清除的旧通知不会误唤醒本次等待。 */
        (void)ulTaskNotifyTake(pdTRUE, 1);
    }
}

/* 正常完成来自优先级12的DMA中断；异常收尾/阻塞回退来自显示任务。 */
static void lcd_flush_ready_cb(void)
{
    /* 取出本次在途 flush 的驱动指针；为空说明没有待通知的刷屏。 */
    lv_disp_drv_t *drv = (lv_disp_drv_t *)s_flush_drv;
    /* ISR 中用于记录是否需要退出中断时切换任务。 */
    BaseType_t woken = pdFALSE;
    if (drv != NULL)
    {
        /* 先清驱动指针，保证同一笔只上报一次完成。 */
        s_flush_drv = NULL;
        lv_disp_flush_ready(drv);
        /* 仅在中断上下文且确实记录了等待任务时发通知，避免在任务上下文误用 ISR 接口。 */
        if (s_flush_task != NULL && __get_IPSR() != 0)
        {
            vTaskNotifyGiveFromISR(s_flush_task, &woken);
            portYIELD_FROM_ISR(woken);
        }
    }
}

void lv_port_disp_service(void)
{
    /* 周期检查超时/错误收尾；发生异常收尾则标记需要整屏重绘。 */
    if (LCD_Flush_Service() != 0)
    {
        s_redraw_pending = 1;
    }
    /* 有待重绘标志时清除它并使当前屏幕失效，触发 LVGL 重画。 */
    if (s_redraw_pending != 0)
    {
        s_redraw_pending = 0;
        lv_obj_invalidate(lv_scr_act());
    }
}

uint8_t lv_port_disp_set_sleep(uint8_t sleeping)
{
    /* 先等所有在途 flush 收尾；每轮复用 disp_wait 的正常完成或超时路径。 */
    while (s_flush_drv != NULL)
    {
        disp_wait((lv_disp_drv_t *)s_flush_drv); /* 复用正常完成及100ms超时收尾。 */
    }
    if (sleeping != 0)
    {
        /* 非 0 表示进入熄屏：关背光、发睡眠命令并关外设时钟。 */
        return LCD_ST7789_SleepIn();
    }
    else
    {
        /* 0 表示亮屏：恢复外设时钟并唤醒面板。 */
        return LCD_ST7789_SleepOut();
    }
}
