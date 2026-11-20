/** 空闲时合并SysTick；蓝牙开启时Sleep，关闭且熄屏、外设空闲时STOP，醒后恢复84MHz。
 *  RTC可用时以其一分钟内递增32768Hz计数为主参考；Sleep中RTC参考失效时用本次加长SysTick回退计时。 */
#ifndef APP_TICKLESS_H
#define APP_TICKLESS_H

#include <stdint.h>

/* 蓝牙实时接收使用Sleep；STOP仍检查屏幕及在途外设。 */
#ifndef APP_TICKLESS_ENABLE_STOP
#define APP_TICKLESS_ENABLE_STOP 1
#endif

/* 由 FreeRTOS 的 portSUPPRESS_TICKS_AND_SLEEP 在 IDLE 内、调度器挂起时调用。
   expected_ticks 为调度器给出的本次空闲可跳过 tick 数（1 tick = 1ms），实际时长还会被
   400ms 上限、最近任务截止时间和 RTC/SysTick 条件收紧。
   所有等待均保持TPS3823超时检测，不从IDLE补喂；RTC/LSE不再唤醒时仍可触发硬件复位。
   直接补齐 uwTick，FreeRTOS 在恢复调度时逐拍处理到期和超期的节拍。 */
void app_tickless_sleep(uint32_t expected_ticks);

#endif
