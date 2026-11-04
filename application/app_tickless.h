/** 空闲时合并SysTick；蓝牙开启时Sleep，关闭且熄屏、外设空闲时STOP，醒后恢复64MHz。
 *  RTC可用时以其一分钟内递增32768Hz计数为主参考；Sleep中RTC参考失效时用本次加长SysTick回退计时。 */
#ifndef APP_TICKLESS_H
#define APP_TICKLESS_H

#include <stdint.h>

/* 蓝牙实时接收使用Sleep；STOP仍检查屏幕及在途外设。 */
#ifndef APP_TICKLESS_ENABLE_STOP
#define APP_TICKLESS_ENABLE_STOP 1
#endif

/* 由 FreeRTOS 的 portSUPPRESS_TICKS_AND_SLEEP 在 IDLE 内调用，全程关着中断执行。
   expected_ticks 为调度器给出的本次空闲可跳过 tick 数（1 tick = 1ms），实际时长还会被
   1000ms 上限、最近任务截止时间和 RTC/SysTick 条件收紧；无 sensor 进度授权时超过
   400ms 的请求进一步收紧至 400ms 以保持硬件超时。
   app_task_watchdog_stop_allowed() 授权时，本轮所有实际等待路径（含1ms回退、Sleep、STOP）
   均在实际 WFI/STOP 前暂停 TPS3823 超时检测，退出等待后立即恢复（STOP 路径恢复在 PLL 之前）。
   返回前必须把真实经过的时间补回 uwTick 和 vTaskStepTick，否则 HAL 超时与任务期限都会失准。 */
void app_tickless_sleep(uint32_t expected_ticks);

#endif
