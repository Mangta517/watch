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
  *                       熄屏UI按200ms等待通知；静止时sensor仍每200ms读取加速度，不降低计步/抬腕输入频率。
  *                       MPU硬件保持5Hz；AUTO心率默认开，启用后每10分钟最多测10s。
  *                       心率/运动仅保留100Hz加速度参考，姿态页或校准需要时恢复完整IMU。
  *                       App_IMU_GetSnapshot() / App_Baro_GetSnapshot() 供 UI 消费
  *                       EEPROM(PA11/PA12)同由sensor任务串行推进：IMU零偏、磁校准、开机秒数、
  *                       天气与OTA标志共用同一写通道；OTA确认后停止其他EEPROM写并等待复位。
  *                       app_task_sleep_poll_ms返回固定200ms等待上限，LVGL在自己的等待里读取；
  *                       sensor在熄屏轮次另发节拍通知唤醒LVGL，但不是逐轮驱动UI刷新。
  *            touch    prio 1, 1KB 栈 — 首帧后初始化触摸，活跃10ms/待机20ms采集；LVGL读取缓存事件
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
    /* 返回熄屏后台等待上限200ms，与sensor的5Hz加速度读取同步；sensor完成后另通知LVGL。 */
    uint32_t app_task_sleep_poll_ms(void);
    /* PA4中断只通知显示任务；GPIO消抖和页面操作仍留在任务上下文。 */
    void app_task_wake_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H */
