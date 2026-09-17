/**
  ******************************************************************************
  * @file    app_task.h
  * @brief   应用层-FreeRTOS 任务管理 (全工程唯一的任务创建 / 调度器启动入口)
  *          分层约束: app_ui / app_display 等业务模块不得再出现 xTaskCreate /
  *          vTaskStartScheduler, 需要新任务时统一在 App_Task_Create() 里登记。
  *
  *          当前任务清单:
  *            lvgl     prio 3, 4KB 栈 — 驱动 LVGL 心跳, 承载屏幕/触摸测试 UI
  *            sensor   prio 2, 2KB 栈 — SENSOR 总线 (MPU6050+SPL006) 任务级 owner:
  *                       IMU 10ms、气压 100ms 轮询 (器件 1Hz); 数据经
  *                       App_IMU_GetSnapshot() / App_Baro_GetSnapshot() 供 UI 消费
  ******************************************************************************
***/
#ifndef __APP_TASK_H
#define __APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建全部应用任务 (在启动调度器前、调度器未运行时调用一次)
 */
void App_Task_Create(void);

/**
 * @brief 启动 FreeRTOS 调度器 (正常情况永不返回)
 */
void App_Task_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */
