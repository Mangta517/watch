#ifndef BOOT_LAYOUT_H
#define BOOT_LAYOUT_H

/* sector 0..1主引导，2..3独立保底，4..6手表APP，7电压日志。 */
#define BOOTLOADER_START_ADDR          0x08000000
#define RECOVERY_START_ADDR            0x08008000
#define APP_START_ADDR                 0x08010000
#define APP_FLASH_END_ADDR             0x08060000
#define APP_MAX_SIZE                   (APP_FLASH_END_ADDR - APP_START_ADDR)
#define BOOTLOADER_EEPROM_FLAG_ADDR    0xC0
#define BOOTLOADER_FLAG_UPDATING       0x5A
#define BOOTLOADER_FLAG_VALID          0xA5

#endif
