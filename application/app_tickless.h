/** 空闲时合并SysTick；BLE关闭且熄屏时使用RTC唤醒STOP，其余保留Sleep。 */
#ifndef APP_TICKLESS_H
#define APP_TICKLESS_H

#include <stdint.h>

void app_tickless_sleep(uint32_t expected_ticks);

#endif
