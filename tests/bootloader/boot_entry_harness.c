/* 主机执行真实boot_main.c引导入口：包含源码并以桩替换平台/选择逻辑，验证按键长按进保底的判定。 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include "main.h"

static uint32_t entry_tick;
static uint32_t entry_press_count;
static uint32_t entry_select_force;
static jmp_buf entry_wfi_exit;

void entry_wfi(void)
{
    longjmp(entry_wfi_exit, 1);
}

uint32_t HAL_GetTick(void)
{
    return entry_tick;
}

void HAL_Delay(uint32_t milliseconds)
{
    entry_tick += milliseconds;
}

/* 前entry_press_count次读返回低（按住），之后返回高（松开）。 */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    if (entry_press_count > 0)
    {
        --entry_press_count;
        return GPIO_PIN_RESET;
    }
    return GPIO_PIN_SET;
}

HAL_StatusTypeDef HAL_Init(void)
{
    return HAL_OK;
}

void HAL_IncTick(void)
{
}

void NVIC_SystemReset(void)
{
    entry_tick = 0;
}

#define main               boot_entry_main
#define boot_recovery_requested entry_boot_recovery_requested
#include "../../bootloader/boot_main.c"
#undef main
#undef boot_recovery_requested

void boot_platform_gpio_init(void)
{
}

void boot_platform_clock_init(void)
{
}

void boot_platform_jump(uint32_t address)
{
    (void)address;
    abort();
}

uint32_t app_boot_select_target(uint8_t force_recovery)
{
    entry_select_force = force_recovery;
    return 0;
}

static uint8_t run_helper(uint32_t press_count, uint32_t tick_base)
{
    entry_press_count = press_count;
    entry_tick = tick_base;
    return entry_boot_recovery_requested();
}

int main(void)
{
    /* 空闲高电平：立即返回0且完全不延时（tick不前进）。 */
    assert(run_helper(0, 12345) == 0 && entry_tick == 12345);
    /* 短按松开：提前返回0，未达2000ms阈值。 */
    assert(run_helper(31, 0) == 0 && entry_tick < 2000);
    /* 按住1990ms后松开：最后一次低电平轮询elapsed=1990，仍返回0。 */
    assert(run_helper(199, 0) == 0 && entry_tick == 1990);
    /* 连续按住满2000ms：返回1。 */
    assert(run_helper(201, 0) == 1 && entry_tick == 2000);
    /* tick跨UINT32回绕仍能正确累计到2000ms返回1（起始1500ms前于回绕点）。 */
    assert(run_helper(5000, UINT32_MAX - 1500) == 1);
    /* 完整入口空闲路径：helper返回0传给选择逻辑，目标0后停在WFI等待维护。 */
    entry_press_count = 0;
    entry_tick = 0;
    entry_select_force = 0xFF;
    if (setjmp(entry_wfi_exit) == 0)
    {
        boot_entry_main();
        assert(0 && "boot_main idle path must reach WFI");
    }
    assert(entry_select_force == 0);
    puts("PASS: boot entry key hold — idle immediate, short release, release 1990ms, hold 2000ms, tick wrap and WFI fallback");
    return 0;
}
