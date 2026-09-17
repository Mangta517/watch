/**
  ******************************************************************************
  * @file    app_display.h
  * @brief   应用层-显示服务: 集成 interface 外设驱动(lcd/touch) + LVGL
  *          分层: application(app_task 建任务 → 本层提供任务体调用的服务)
  *                → Middlewares(lv_port 移植层/lvgl/FreeRTOS)
  *                → interface(驱动) → Core(HAL)
  *          注意: 任务的创建与调度器启动不在本层 (见 app_task.h)。
  *          移植自 D:/3251/f411RET6/Watch 工程, 按本工程分层要求调整。
  ******************************************************************************
***/
#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_ui.h"

/**
 * @brief 显示服务初始化 —— 必须在 FreeRTOS 任务上下文里调用一次:
 *        创建 LVGL 互斥锁 → lv_init → 显示/触摸移植层初始化(拉起 LCD 与 CST816)
 *        → 构建屏幕/触摸测试 UI。完成后返回。
 */
void App_Display_Init(void);

/**
 * @brief 显示服务心跳 —— 任务循环里周期调用: 加锁 → lv_timer_handler → 解锁
 */
void App_Display_Service(void);

/* LVGL 非线程安全: 其他 FreeRTOS 任务调用任何 lv_* API 前必须包锁 */
void App_Display_Lock(void);
void App_Display_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H */
