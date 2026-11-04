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
  *                       气压每5分钟短采样，温湿度亮屏1s/熄屏10分钟；运动模式暂停气压和校准保存。
  *                       熄屏UI按200ms等待通知；加速度稳定约10秒且无其它业务时，sensor读取与UI等待一起放宽至1000ms。
  *                       MPU硬件仍为5Hz；运动、恢复或在途保存退出慢待机。AUTO心率默认关，启用后每10分钟最多测10s。
  *                       心率/运动仅保留100Hz加速度参考，姿态页或校准需要时恢复完整IMU。
  *                       App_IMU_GetSnapshot() / App_Baro_GetSnapshot() 供 UI 消费
  *                       EEPROM(PA11/PA12)同由sensor任务串行推进：IMU零偏、磁校准、开机秒数、
  *                       天气与OTA标志共用同一写通道；OTA确认后停止其他EEPROM写并等待复位。
  *                       sensor发布的是共享等待毫秒(app_task_sleep_poll_ms)，LVGL在自己的等待里读取；
  *                       sensor在熄屏轮次另发节拍通知唤醒LVGL，但不是逐轮驱动UI刷新。
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
 *        先初始化外部看门狗芯片，再创建显示任务；任务创建失败直接进Error_Handler。
 */
    void App_Task_Create(void);

    /**
 * @brief 启动 FreeRTOS 调度器 (正常情况永不返回)
 */
    void App_Task_Start(void);
    /* UI单写、任务只读：发布待机、姿态页、全熄屏及独立运动模式，不操作硬件。
       四个参数均为非0即置位；熄屏→亮屏的边沿会额外通知touch任务退出休眠等待。 */
    void app_task_set_power_state(uint8_t idle, uint8_t motion, uint8_t screen_off, uint8_t sport);
    /* IDLE读取屏幕状态；STOP资格还须检查蓝牙及在途外设。 */
    uint8_t app_task_is_screen_off(void);
    /* sensor单写、LVGL单读：熄屏后台稳定等待节拍，静止慢待机放宽到1000ms，亮屏或活动回到200ms。
       返回单位毫秒，LVGL用它作为通知等待超时；sensor另在熄屏轮次发通知唤醒LVGL。 */
    uint32_t app_task_sleep_poll_ms(void);
    /* 仅tickless读取：sensor已启动且本次HAL_GetTick距最近一次由新sensor进度确认的喂狗不超过200ms时
       返回1，为静止1秒待机的空闲等待临时暂停TPS3823超时检测；单次STOP仍不超过1000ms，
       本函数不喂狗、不更新授权。 */
    uint8_t app_task_watchdog_stop_allowed(void);
    /* PA4中断只通知显示任务；GPIO消抖和页面操作仍留在任务上下文。 */
    void app_task_wake_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */
