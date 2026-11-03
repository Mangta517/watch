#ifndef APP_BOOT_SELECT_H
#define APP_BOOT_SELECT_H

#include <stdint.h>
#include "boot_layout.h"

/* 返回APP或保底起点；目标无有效向量时返回0，主引导停留等待SWD维护。 */
uint32_t app_boot_select_target(uint8_t force_recovery);

#endif
