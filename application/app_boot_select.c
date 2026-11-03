#include "app_boot_select.h"
#include "bl24c02.h"

static uint8_t app_boot_select_vectors_valid(uint32_t start, uint32_t end)
{
    uint32_t stack = *(volatile uint32_t *)(uintptr_t)start;
    uint32_t reset = *(volatile uint32_t *)(uintptr_t)(start + 4);
    return stack > 0x20000000 && stack <= 0x20020000 && (stack & 7) == 0 &&
           (reset & 1) != 0 && (reset & ~1U) >= start + 8 && (reset & ~1U) < end;
}

uint32_t app_boot_select_target(uint8_t force_recovery)
{
    uint8_t record[4];
    uint8_t allow_app;

    /* 首装区分：SWD单独下载APP时EEPROM空白或遗留非BL记录，向量有效即允许启动；
     * 以BL(0x42,0x4C)开头的记录只有精确有效标志才放行，升级中/畸形/撕裂一律进保底。 */
    allow_app = 0;
    if (force_recovery == 0 && app_boot_select_vectors_valid(APP_START_ADDR, APP_FLASH_END_ADDR) != 0 &&
        bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record)) == 0)
    {
        if (!(record[0] == 0x42 && record[1] == 0x4C) ||
            (record[2] == BOOTLOADER_FLAG_VALID && record[3] == BOOTLOADER_FLAG_UPDATING))
        {
            allow_app = 1;
        }
    }
    if (allow_app != 0)
    {
        return APP_START_ADDR;
    }
    if (app_boot_select_vectors_valid(RECOVERY_START_ADDR, APP_START_ADDR) != 0)
    {
        return RECOVERY_START_ADDR;
    }
    return 0;
}
