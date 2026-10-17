/**
  ******************************************************************************
  * @file    app_display.c
  * @brief   应用层-显示服务实现 (详见 app_display.h 的分层说明)
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          App_Display_Init()/App_Display_Service() 由 lvgl 任务体调用。
  *          结构: 参考 lvgl_zet6 的 lvgl_demo 骨架, 带 LVGL 互斥锁的正式形态。
  *          内核版本: Middlewares/FreeRTOS V202212.01 (RVDS/ARM_CM4F port)。
  ******************************************************************************
***/
#include "app_display.h"
#include "main.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "lvgl.h"
#include "lv_port_disp_template.h"  /* → interface/lcd.c   (ST7789, SPI1+背光PWM) */
#include "lv_port_indev_template.h" /* → interface/touch.c (CST816, 软件I2C 轮询) */

/*********************
 *  锁
 *********************/
static SemaphoreHandle_t s_lvgl_mux;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_Display_Init(void)
{
    /* mutex 在任务上下文创建: 此时调度器已运行, 临界区计数才真正归零工作
     * (CM4F port 的 uxCriticalNesting 以 0xaaaaaaaa 哨兵初值, 调度器前的
     *  enter/exit 不解除 BASEPRI, 属设计行为, 见 SVC 启动时统一清零) */
    s_lvgl_mux = xSemaphoreCreateMutex();
    if (s_lvgl_mux == NULL)
    {
        Error_Handler();
        return;
    }

    lv_init();

    /* 硬件拉起 + LVGL 注册 + 建树, 全程跑在调用方(lvgl)任务栈上 */
    lv_port_disp_init();  /* → LCD_Init: ST7789 序列+刷黑+背光 80%      */
    lv_port_indev_init(); /* → Touch_Init: CST816 复位+配置 (软件I2C轮询) */
    App_UI_Create();      /* 屏幕/触摸测试页                             */
}

void App_Display_Service(void)
{
    App_Display_Lock();
    lv_port_disp_service();
    lv_timer_handler();
    App_Display_Unlock();
}

void App_Display_Lock(void)
{
    if (s_lvgl_mux != NULL)
    {
        xSemaphoreTake(s_lvgl_mux, portMAX_DELAY);
    }
}

void App_Display_Unlock(void)
{
    if (s_lvgl_mux != NULL)
    {
        xSemaphoreGive(s_lvgl_mux);
    }
}
