/** 空闲时合并SysTick；蓝牙开启时Sleep，关闭且熄屏、外设空闲时STOP，醒后恢复64MHz。 */
#ifndef APP_TICKLESS_H
#define APP_TICKLESS_H

#include <stdint.h>

/* 蓝牙实时接收使用Sleep；STOP仍检查屏幕及在途外设。 */
#ifndef APP_TICKLESS_ENABLE_STOP
#define APP_TICKLESS_ENABLE_STOP 1
#endif

void app_tickless_sleep(uint32_t expected_ticks);

#endif
