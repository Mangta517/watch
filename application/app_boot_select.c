/**
 * @file app_boot_select.c
 * @brief 主引导据"APP向量是否可用 + EEPROM升级标志"决定跳到APP还是保底。
 *        标志为4字节 'B''L' + 状态 + 状态反码(UPDATING=0x5A、VALID=0xA5)；
 *        收固件一方先写UPDATING再擦APP，全部校验通过后才改VALID，
 *        因此升级中途掉电后下次上电仍留在保底收包，不会跳进半幅程序。
 */
#include "app_boot_select.h"
#include "bl24c02.h"

/* 检查镜像头两个向量：栈顶须落在SRAM且8字节对齐，Reset_Handler清Thumb位后须落在[start+8,end)。 */
static uint8_t app_boot_select_vectors_valid(uint32_t start, uint32_t end)
{
    /* 读向量表第 0 个字作为初始 MSP 候选值。 */
    uint32_t stack = *(volatile uint32_t *)(uintptr_t)start;
    /* 读向量表第 1 个字作为复位处理函数地址，最低位为 Thumb 标志。 */
    uint32_t reset = *(volatile uint32_t *)(uintptr_t)(start + 4);
    /* 依次检查：MSP 落在 SRAM 区间且 8 字节对齐；复位地址带 Thumb 位，
     * 且去掉 Thumb 位后的入口落在 [start+8, end) 范围内。 */
    return stack > 0x20000000 && stack <= 0x20020000 && (stack & 7) == 0 &&
           (reset & 1) != 0 && (reset & ~1U) >= start + 8 && (reset & ~1U) < end;
}

/* 判定顺序：允许启动APP → 否则保底 → 保底向量也不可用则返回0由调用方停留。 */
uint32_t app_boot_select_target(uint8_t force_recovery)
{
    /* 暂存从 EEPROM 读出的 4 字节启动标志记录。 */
    uint8_t record[4];
    /* 本次是否允许启动 APP 的判定结果，1 表示允许。 */
    uint8_t allow_app;

    /* 首装区分：SWD单独下载APP时EEPROM空白或遗留非BL记录，向量有效即允许启动；
     * 以BL(0x42,0x4C)开头的记录只有精确有效标志才放行，升级中/畸形/撕裂一律进保底。 */
    /* 默认不允许启动 APP，只有下面三项条件都满足才改写。 */
    allow_app = 0;
    /* 条件一：没有强制进保底；条件二：APP 区前两个向量通过有效性检查；
     * 条件三：从 EEPROM 标志地址成功读出 4 字节（返回 0 表示读取成功）。 */
    if (force_recovery == 0 && app_boot_select_vectors_valid(APP_START_ADDR, APP_FLASH_END_ADDR) != 0 &&
        bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record)) == 0)
    {
        /* 记录不是以 'B''L' 开头（空白或非本协议记录）时直接放行；
         * 是 BL 记录时，要求第 3 字节为 VALID(0xA5)、第 4 字节为其反码 0x5A
         * （即 BOOTLOADER_FLAG_UPDATING 的取值），两项同时成立才算升级已完成。 */
        if (!(record[0] == 0x42 && record[1] == 0x4C) ||
            (record[2] == BOOTLOADER_FLAG_VALID && record[3] == BOOTLOADER_FLAG_UPDATING))
        {
            /* 标记本次可以跳 APP。 */
            allow_app = 1;
        }
    }
    /* 允许启动 APP 时返回 APP 镜像起址。 */
    if (allow_app != 0)
    {
        /* 调用方将把该地址作为跳转目标。 */
        return APP_START_ADDR;
    }
    /* 本次未允许启动APP时，检查保底镜像向量。 */
    if (app_boot_select_vectors_valid(RECOVERY_START_ADDR, APP_START_ADDR) != 0)
    {
        /* 保底向量有效，返回保底固件起址。 */
        return RECOVERY_START_ADDR;
    }
    /* 没有符合本次启动条件的目标，返回0，由调用方停留等待。 */
    return 0;
}
