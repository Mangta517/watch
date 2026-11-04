/**
  ******************************************************************************
  * @file    app_display.h
  * @brief   应用层-显示服务: 集成 interface 外设驱动(lcd/touch) + LVGL
  *          分层: application(app_task 建任务 → 本层提供任务体调用的服务)
  *                → Middlewares(lv_port 移植层/lvgl/FreeRTOS)
  *                → interface(驱动) → Core(HAL)
  *          注意: 任务的创建与调度器启动不在本层 (见 app_task.h)。
  *          职责: 提供LVGL互斥锁、一次性显示初始化和每轮显示心跳；亮屏才跑LVGL定时器，
  *          熄屏只推进UI业务，SPI DMA的收尾由移植层负责。
  *          移植自 D:/3251/f411RET6/Watch 工程, 按本工程分层要求调整。
  ******************************************************************************
***/
#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "app_ui.h"

    /**
 * @brief 显示服务初始化 —— 必须在 FreeRTOS 任务上下文里调用一次:
 *        创建 LVGL 互斥锁 → lv_init → 初始化LCD并注册缓存输入(CST816由touch任务初始化)
 *        → 构建手表界面（App_UI_Create）。完成后返回。
 *        不创建任务、不启动调度器；互斥锁必须在调度器运行后创建。
 */
    void App_Display_Init(void);

    /**
 * @brief 显示服务心跳：亮屏运行LVGL；熄屏只处理业务/按键并收尾在途DMA。
 *        每轮先让移植层完成上一笔SPI DMA，必要时登记整屏重绘，再决定是否跑lv_timer_handler。
 *        由lvgl任务按自身节拍调用，内部自行加锁。
 */
    void App_Display_Service(void);

    /* LVGL 非线程安全: 其他 FreeRTOS 任务调用任何 lv_* API 前必须包锁。
       互斥锁在App_Display_Init中创建，未创建时两个函数为空操作。 */
    void App_Display_Lock(void);
    void App_Display_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H */
