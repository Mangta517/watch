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
static volatile lv_disp_drv_t *s_flush_drv = NULL;
static TaskHandle_t s_flush_task;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lv_port_disp_init(void)
{
    /* 先初始化物理屏 (含刷黑与背光, 均为阻塞路径) */
    disp_init();

    /* DMA 完成通知接线 (在任何 flush_cb 之前) */
    LCD_Set_Flush_Complete_Callback(lcd_flush_ready_cb);

    static lv_disp_draw_buf_t draw_buf_dsc;
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2, LVGL_BUF_PX);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = disp_flush;
    disp_drv.wait_cb = disp_wait;
    disp_drv.draw_buf = &draw_buf_dsc;
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void disp_init(void)
{
    LCD_Init();
}

/* 发起 DMA 传输即返回; ready 由 TxCplt 中断里的 lcd_flush_ready_cb 报 */
static void disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    s_flush_task = NULL;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        s_flush_task = xTaskGetCurrentTaskHandle();
        (void)ulTaskNotifyTake(pdTRUE, 0); /* 清除上一块已完成、无需等待的通知。 */
    }
    s_flush_drv = disp_drv;
    LCD_Color_Fill_DMA((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)area->x2, (uint16_t)area->y2, (const uint16_t *)color_p);
}

/* DMA完成即可唤醒LVGL；等待期间sensor可运行，不额外等待下一个1ms tick。 */
static void disp_wait(lv_disp_drv_t *disp_drv)
{
    (void)disp_drv;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        (void)ulTaskNotifyTake(pdTRUE, 1U);
    }
}

/* lcd.c 的 HAL_SPI_TxCpltCallback 在 DMA2_Stream2 中断上下文调用此处。
 * LCD_Init已将DMA2_Stream2设为优先级12，满足本工程FreeRTOS的ISR API限制。 */
static void lcd_flush_ready_cb(void)
{
    lv_disp_drv_t *drv = (lv_disp_drv_t *)s_flush_drv;
    BaseType_t woken = pdFALSE;
    if (drv != NULL)
    {
        s_flush_drv = NULL;
        lv_disp_flush_ready(drv);
        if (s_flush_task != NULL)
        {
            vTaskNotifyGiveFromISR(s_flush_task, &woken);
            portYIELD_FROM_ISR(woken);
        }
    }
}
