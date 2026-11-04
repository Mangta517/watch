/**
 * @file app_boot_select.h
 * @brief 主引导(0x08000000)的启动目标选择：只判断并给出地址，不搬运固件、不擦写APP区。
 *        无RTOS环境下由boot_main.c调用，除读Flash向量表和EEPROM标志外不操作其他外设。
 */
#ifndef APP_BOOT_SELECT_H
#define APP_BOOT_SELECT_H

#include <stdint.h>
#include "boot_layout.h"

/* 返回本次允许启动的APP(0x08010000)或保底(0x08008000)起点；没有符合本次启动条件的目标则返回0，由主引导停留等待SWD维护。
 * force_recovery=1(开机长按按键)跳过APP；EEPROM读失败时也只尝试保底。 */
uint32_t app_boot_select_target(uint8_t force_recovery);

#endif
