#ifndef PLATFORM_DELAY_H
#define PLATFORM_DELAY_H

#include <stdint.h>

/* 任务运行时让出CPU；调度器未运行及Bootloader中使用HAL等待。 */
void platform_delay_ms(uint32_t ms);
/* 硬件最短等待额外留一拍，避免调用落在tick末尾时不足指定毫秒。 */
void platform_delay_min_ms(uint32_t ms);

#endif
