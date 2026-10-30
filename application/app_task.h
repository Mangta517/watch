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
  *                       sensor普通20/40ms调度，心率/运动模式10ms；非姿态需求为5Hz加速度低功耗循环，计步与辅助抬腕持续运行。
  *                       气压每5分钟短采样，温湿度亮屏1s/熄屏1分钟；运动模式暂停气压和校准保存。
  *                       熄屏后UI等待PA4/UART/sensor通知或200ms期限；心率每10分钟最多10s。
  *                       心率/运动仅保留100Hz加速度参考，姿态页或校准需要时恢复完整IMU。
  *                       App_IMU_GetSnapshot() / App_Baro_GetSnapshot() 供 UI 消费
  *                       EEPROM(PA11/PA12)同由sensor任务读写，开机恢复/校准保存、天气保存及每10分钟记录开机秒数。
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

    /**
 * @brief 启动 FreeRTOS 调度器 (正常情况永不返回)
 */
    void App_Task_Start(void);
    /* UI单写、任务只读：发布待机、姿态页、全熄屏及独立运动模式，不操作硬件。 */
    void app_task_set_power_state(uint8_t idle, uint8_t motion, uint8_t screen_off, uint8_t sport);
    /* IDLE读取屏幕状态；STOP资格还须检查蓝牙及在途外设。 */
    uint8_t app_task_is_screen_off(void);
    /* PA4中断只通知显示任务；GPIO消抖和页面操作仍留在任务上下文。 */
    void app_task_wake_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */
