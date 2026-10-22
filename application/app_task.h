/**
  ******************************************************************************
  * @file    app_task.h
  * @brief   应用层-FreeRTOS 任务管理 (全工程唯一的任务创建 / 调度器启动入口)
  *          分层约束: app_ui / app_display 等业务模块不得再出现 xTaskCreate /
  *          vTaskStartScheduler, 任务创建统一在 app_task.c 内安排。
  *
  *          当前任务清单:
  *            lvgl     prio 3, 4KB 栈 — 首帧提交后创建sensor/touch，UI循环分步推进RTC/蓝牙启动
  *            sensor   prio 2, 2KB 栈 — SENSOR 总线 (MPU6050+SPL006+LSM303DLHC+EM7028+AHT21) owner:
  *                       sensor按使用状态20/40ms调度，MPU50/25Hz、磁力计30/7.5Hz、气压1s(器件1Hz); 数据经
  *                       App_IMU_GetSnapshot() / App_Baro_GetSnapshot() 供 UI 消费
  *                       EEPROM(PA11/PA12)同由sensor任务读写，只在开机恢复/校准成功时访问
  *            touch    prio 1, 1KB 栈 — 首帧后初始化触摸，活跃5ms/待机20ms采集；LVGL读取缓存事件
  ******************************************************************************
***/
#ifndef __APP_TASK_H
#define __APP_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
 * @brief 创建显示任务 (调度器启动前调用一次；首帧后创建sensor/touch，RTC/蓝牙等待不阻塞采集)
 */
    void App_Task_Create(void);

    /* PA3保持供电后、HAL/时钟初始化前启用；正常运行共用双任务进度喂狗。 */
    void app_boot_watchdog_start(void);

    /**
 * @brief 启动 FreeRTOS 调度器 (正常情况永不返回)
 */
    void App_Task_Start(void);
    /* UI单写、任务只读：发布待机和运动页需求，不操作硬件。 */
    void app_task_set_power_state(uint8_t idle, uint8_t motion);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */
