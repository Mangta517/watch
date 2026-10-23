/** Sleep下合并SysTick；RTC只测量休眠间隔，不使用可被手机修改的日历差值。 */
#ifndef APP_TICKLESS_H
#define APP_TICKLESS_H

#include <stdint.h>

void app_tickless_sleep(uint32_t expected_ticks);

#endif
