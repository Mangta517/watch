#ifndef APP_BOOTLOADER_H
#define APP_BOOTLOADER_H

#include "int_bootloader.h"

typedef enum
{
    BOOTLOADER_STATE_DEFAULT,
    BOOTLOADER_STATE_INIT,
    BOOTLOADER_STATE_RUN,
    BOOTLOADER_STATE_REC_DATA,
    BOOTLOADER_STATE_CHECK_DATA,
    BOOTLOADER_STATE_JUMP_TO_APP
} Bootloader_state;

extern Bootloader_state boot_state;
void app_bootloader_init(void);
void app_bootloader_work(void);

#endif
