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
/* 显示初始化：创建LVGL互斥锁→lv_init→LCD/背光→注册缓存输入→建UI树。
 * 全部在调用方(lvgl)任务栈上完成，硬件失败由各驱动自己的错误路径处理。 */
void App_Display_Init(void)
{
    /* mutex 在任务上下文创建: 此时调度器已运行, 临界区计数才真正归零工作
     * (CM4F port 的 uxCriticalNesting 以 0xaaaaaaaa 哨兵初值, 调度器前的
     *  enter/exit 不解除 BASEPRI, 属设计行为, 见 SVC 启动时统一清零) */
    /* 创建二值互斥锁：用于串行化各任务对 LVGL 的访问，返回 NULL 表示内存不足。 */
    s_lvgl_mux = xSemaphoreCreateMutex();
    if (s_lvgl_mux == NULL)
    {
        /* 锁句柄创建失败：进入工程统一的错误处理，不再继续初始化显示。 */
        Error_Handler();
        /* 错误处理若返回，这里直接退出，避免后续代码解引用空锁句柄。 */
        return;
    }

    /* 初始化 LVGL 内核及其内部对象/定时器管理，必须在创建控件之前完成。 */
    lv_init();

    /* 硬件拉起 + LVGL 注册 + 建树, 全程跑在调用方(lvgl)任务栈上 */
    lv_port_disp_init();  /* → LCD_Init: ST7789 序列+刷黑+背光 80%      */
    lv_port_indev_init(); /* 只注册缓存输入；触摸硬件由独立任务初始化/轮询。 */
    App_UI_Create();      /* 建表盘首页；后续切页由UI定时器推进           */
}

/* 显示服务每轮一次：锁内先让移植层收尾在途DMA（必要时登记整屏重绘），
 * 亮屏走LVGL刷新定时器，熄屏只跑UI业务（按键/蓝牙/抬腕），不带动画和刷屏开销。
 * 全部LVGL调用都在本任务持锁进行，其他任务不得绕过互斥锁。 */
void App_Display_Service(void)
{
    /* 先取得 LVGL 互斥锁，保证本轮的移植层收尾与 LVGL 调用不被其他任务并发打断。 */
    App_Display_Lock();
    /* 让显示移植层处理上一轮刷屏的在途 DMA，必要时登记整屏重绘。 */
    lv_port_disp_service();
    /* 读取当前屏幕开关状态：非 0 表示亮屏。 */
    if (App_UI_IsScreenOn() != 0)
    {
        /* 亮屏：运行 LVGL 定时器处理，推进刷新、动画和输入等周期工作。 */
        lv_timer_handler();
    }
    else
    {
        /* 熄屏：只跑 UI 业务逻辑（按键/蓝牙/抬腕），不驱动动画和刷屏以省开销。 */
        App_UI_ServiceSleeping();
    }
    /* 本轮工作结束，释放互斥锁供其他任务使用。 */
    App_Display_Unlock();
}

/* 互斥锁在调度器运行后创建；未创建时(初始化早期)两个接口为空操作，
 * 避免其他任务在锁尚未存在时阻塞或解引用空句柄。 */
void App_Display_Lock(void)
{
    /* 锁句柄存在时才取锁；portMAX_DELAY 表示一直等待直到成功，不设超时。 */
    if (s_lvgl_mux != NULL)
    {
        xSemaphoreTake(s_lvgl_mux, portMAX_DELAY);
    }
}

/* 释放互斥锁；未创建时同样是空操作，与App_Display_Lock成对使用。 */
void App_Display_Unlock(void)
{
    /* 锁句柄存在时才释放，与取锁接口的空操作判断保持一致。 */
    if (s_lvgl_mux != NULL)
    {
        xSemaphoreGive(s_lvgl_mux);
    }
}
