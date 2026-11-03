#ifndef BOOT_PLATFORM_H
#define BOOT_PLATFORM_H

#include <stdint.h>

/* 主引导和独立保底共用：PA3保电、屏蔽TPS3823复位、HSI PLL 100MHz。 */
void boot_platform_gpio_init(void);
void boot_platform_clock_init(void);
/* 调用方先检查目标向量，并关闭自己启用的UART/DMA。 */
void boot_platform_jump(uint32_t address);

#endif
