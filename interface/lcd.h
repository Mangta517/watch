/**
  ******************************************************************************
  * @file    lcd.h
  * @brief   ST7789 240x280 LCD 驱动 (SPI1 + 软件CS + DC/RST GPIO + PB0背光PWM)
  *          移植自 D:/3251/f411RET6/Watch 工程 interface/lcd.h (源自 FryPi OV_Watch
  *          BSP/LCD, GPL-3.0), 按本工程 CubeMX 引脚重新适配:
  *          引脚: SCK=PB3(SPI1) MOSI=PB5(SPI1) CS=PB8 DC=PB9 RST=PB7
  *                BLK=PB0(TIM3_CH3, PWM1 高极性, 高电平点亮)
  *                SPI1_TX DMA=DMA2_Stream2 (异步刷屏, CubeMX 已配置)
  *          与 Watch 版的差异: ①背光定时器 TIM2_CH1→TIM3_CH3, 极性 LOW→HIGH
  *          (CCR 不再反转); ②DMA 完成中断优先级在 LCD_Init 里设为 12
  *          (≥configMAX_SYSCALL, 与 FreeRTOS 临界区共处更稳)。
  ******************************************************************************
***/
#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

/* 面板参数: 竖屏 USE_HORIZONTAL=1, MADCTL=0xC0, ST7789 显存320行 -> 280行面板 Y 偏移20 */
#define LCD_WIDTH    240
#define LCD_HEIGHT   280
#define LCD_OFFSET_Y 20

/* 背光: TIM3_CH3 @100MHz/100(PSC+1) = 1MHz 计数, ARR=300 -> 约3.33kHz。
 * OCPolarity=HIGH 高电平点亮, 占空比 = CCR/ARR, 无需反转映射。 */
#define LCD_BL_CCR_INVERT 0
#define LCD_BL_ARR        300

/* RGB565 常用色 */
#define LCD_BLACK 0x0000
#define LCD_WHITE 0xFFFF
#define LCD_RED   0xF800
#define LCD_GREEN 0x07E0
#define LCD_BLUE  0x001F

    /*--- 初始化 / 电源 ---*/
    void LCD_Init(void);            /* 复位+ST7789初始化序列, 结束后CS常拉低 */
    void LCD_ST7789_SleepIn(void);  /* 0x10 省电 */
    void LCD_ST7789_SleepOut(void); /* 0x11 唤醒 */

    /*--- 基础写 ---*/
    void LCD_WR_REG(uint8_t reg);                                             /* DC=0 写命令 */
    void LCD_WR_DATA8(uint8_t dat);                                           /* DC=1 写一字节 */
    void LCD_WR_DATA(uint16_t dat);                                           /* DC=1 写一字(高字节先出) */
    void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2); /* 已含 OFFSET_Y 补偿 */

    /*--- 刷屏接口 ---*/
    void LCD_Color_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p); /* RGB565块传输(阻塞) */

    /* DMA 异步刷屏 (SPI1_TX DMA @DMA2_Stream2, OV_Watch/Watch 同款): 发起即返回,
 * 传输完成经 HAL_SPI_TxCpltCallback → LCD_Set_Flush_Complete_Callback 注册的回调通知 */
    typedef void (*LCD_CallbackFunc_t)(void);
    void LCD_Set_Flush_Complete_Callback(LCD_CallbackFunc_t cb);
    void LCD_Color_Fill_DMA(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p);

    void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color); /* 单色填充 */
    void LCD_Clear(uint16_t color);                                                            /* 全屏填充 */
    void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);

    /*--- 背光 ---*/
    void LCD_Open_Light(void);           /* 启动 TIM3_CH3 PWM (含 ARR 改写) */
    void LCD_Set_Light(uint8_t percent); /* 0~100 亮度 */
    void LCD_Close_Light(void);          /* 关背光 */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */
