/**
  ******************************************************************************
  * @file    lcd.c
  * @brief   ST7789 (240x280, 竖屏) SPI1 驱动 — 移植自 Watch 工程 interface/lcd.c
  *          (源头为 FryPi OV_Watch BSP/LCD, GPL-3.0)。
  *          本工程适配点:
  *          ① CS/DC/RST 直接复用 CubeMX 生成的引脚宏 (PB8/PB9/PB7);
  *          ② SPI1_TX DMA(DMA2_Stream2) 已由 CubeMX 配置 → 提供 Watch 同款
  *             "阻塞整块发送" + "DMA 异步刷屏(经 TxCplt 回调报完成)" 双路径;
  *          ③ 背光为 TIM3_CH3/PB0, OCPolarity=HIGH → CCR 正映射 (不反转);
  *             CubeMX 已配 PSC=99/ARR=300 → 1MHz/300 ≈ 3.33kHz, LCD_Open_Light
  *             里仍按 LCD_BL_ARR 重写一次 ARR 作防御 (AutoReloadPreload=DISABLE,
  *             立即生效), 保证与驱动宏一致。
  *          依赖: hspi1(spi.c, Mode3: CPOL=High/CPHA=2Edge, MSB, DIV2), htim3(tim.c),
  *                MX_GPIO_Init 已配置 CS/DC/RST 推挽输出。
  ******************************************************************************
***/
#include "lcd.h"
#include "spi.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/* 引脚快捷宏 (全部来自 CubeMX 生成的 main.h) */
#define LCD_CS_Clr()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_Set()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_DC_Clr()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_Set()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_RES_Clr() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RES_Set() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

/* SPI 单笔传输超时(ms)。HAL 用 HAL_GetTick 计时, 内核运行后由 it.c 的
 * SysTick_Handler → HAL_IncTick 持续维护。总线异常时不会永久卡死。 */
#define LCD_SPI_TMO 1000

static void lcd_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免调用落在tick末尾时缩短硬件最小等待时间。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
    }
    else
    {
        HAL_Delay(ms);
    }
}

/******************************************************************************
 * 底层: 硬件SPI 写 (SPI1 主模式, CubeMX 实配 CPOL=1/CPHA=1 即 Mode3, MSB first;
 *       ST7789 4线时序兼容 Mode0 与 Mode3, 本工程用 Mode3)
 *****************************************************************************/
static void LCD_Writ_Bus(uint8_t dat)
{
    if (HAL_SPI_Transmit(&hspi1, &dat, 1, LCD_SPI_TMO) != HAL_OK)
    {
        Error_Handler(); /* SPI 总线故障, 停机便于调试 */
    }
}

void LCD_WR_DATA8(uint8_t dat)
{
    LCD_Writ_Bus(dat);
}

/* 16bit 数据高字节先出 (ST7789 大端要求, 与 LVGL 的 LV_COLOR_16_SWAP=1 配套) */
void LCD_WR_DATA(uint16_t dat)
{
    uint8_t tmp[2];
    tmp[0] = (uint8_t)(dat >> 8);
    tmp[1] = (uint8_t)(dat);
    if (HAL_SPI_Transmit(&hspi1, tmp, 2, LCD_SPI_TMO) != HAL_OK)
    {
        Error_Handler();
    }
}

void LCD_WR_REG(uint8_t reg)
{
    LCD_DC_Clr(); /* 命令 */
    LCD_Writ_Bus(reg);
    LCD_DC_Set(); /* 恢复数据模式 */
}

/******************************************************************************
 * 地址窗口 (自动加 Y 偏移; 0x2A 高16bit恒0, 0x2B 按16bit发)
 *****************************************************************************/
static HAL_StatusTypeDef lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint8_t data[11];
    uint8_t i;
    HAL_StatusTypeDef status;
    y1 += LCD_OFFSET_Y;
    y2 += LCD_OFFSET_Y;
    data[0] = 0x2A;
    data[1] = 0;
    data[2] = (uint8_t)x1;
    data[3] = 0;
    data[4] = (uint8_t)x2;
    data[5] = 0x2B;
    data[6] = (uint8_t)(y1 >> 8);
    data[7] = (uint8_t)y1;
    data[8] = (uint8_t)(y2 >> 8);
    data[9] = (uint8_t)y2;
    data[10] = 0x2C;
    for (i = 0; i < sizeof(data); ++i)
    {
        if (i == 0 || i == 5 || i == 10)
        {
            LCD_DC_Clr();
        }
        status = HAL_SPI_Transmit(&hspi1, &data[i], 1, LCD_SPI_TMO);
        LCD_DC_Set();
        if (status != HAL_OK)
        {
            return status;
        }
    }
    return HAL_OK;
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (lcd_address_set(x1, y1, x2, y2) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************
 * 刷屏接口
 *****************************************************************************/
/* LVGL flush: 把 w*h 个 RGB565 像素发到指定窗口 (字节序由 LV_COLOR_16_SWAP=1 保证)。
 * HAL_SPI_Transmit 的 Size 是 uint16_t, 超过 64KB 的块自动分片;
 * GRAM 游标在窗口内自动前进, 分片之间不需要重设地址。 */
static HAL_StatusTypeDef lcd_color_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p)
{
    uint32_t bytes = (uint32_t)(xend - xsta + 1) * (uint32_t)(yend - ysta + 1) * 2;
    const uint8_t *p = (const uint8_t *)color_p;
    HAL_StatusTypeDef st = HAL_OK;

    st = lcd_address_set(xsta, ysta, xend, yend);
    if (st != HAL_OK)
    {
        return st;
    }
    while (bytes > 0xFFFF)
    { /* 每片 <=64KB-2, 保证整像素边界 */
        st = HAL_SPI_Transmit(&hspi1, (uint8_t *)p, 0xFFFE, LCD_SPI_TMO);
        if (st != HAL_OK)
        {
            break;
        }
        p += 0xFFFE;
        bytes -= 0xFFFE;
    }
    if (st == HAL_OK && bytes)
    {
        st = HAL_SPI_Transmit(&hspi1, (uint8_t *)p, (uint16_t)bytes, LCD_SPI_TMO);
    }
    return st;
}

void LCD_Color_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p)
{
    if (lcd_color_fill(xsta, ysta, xend, yend, color_p) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************
 * DMA 异步刷屏 (Watch/OV_Watch 同款): 发起即返回, 完成经 TxCplt 中断通知
 * 约束: 同一时刻仅一笔在飞 (LVGL 在 flush_ready 前不会再调 flush_cb,
 *       且命令/参数写都在上一笔 ready 之后, 天然串行)
 *****************************************************************************/
static LCD_CallbackFunc_t s_lcd_flush_done_cb = NULL;
static volatile uint8_t s_lcd_flush_active;
static volatile uint8_t s_lcd_flush_failed;
static volatile uint32_t s_lcd_flush_errors; /* 调试器可观察累计失败次数。 */

void LCD_Set_Flush_Complete_Callback(LCD_CallbackFunc_t cb)
{
    s_lcd_flush_done_cb = cb;
}

void LCD_Color_Fill_DMA(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p)
{
    uint32_t bytes = (uint32_t)(xend - xsta + 1) * (uint32_t)(yend - ysta + 1) * 2;
    HAL_StatusTypeDef status;

    s_lcd_flush_active = 1;

    if (bytes > 0xFFFE)
    { /* F4 HAL Size 为 uint16: 超 64KB 罕见大块, 退回阻塞分片 */
        status = lcd_color_fill(xsta, ysta, xend, yend, color_p);
        if (status == HAL_OK)
        {
            s_lcd_flush_active = 0;
            if (s_lcd_flush_done_cb != NULL)
            {
                s_lcd_flush_done_cb();
            }
        }
    }
    else
    {
        status = lcd_address_set(xsta, ysta, xend, yend);
        if (status == HAL_OK)
        {
            status = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)color_p, (uint16_t)bytes);
        }
    }
    if (status != HAL_OK)
    {
        s_lcd_flush_failed = 1;
    }
}

/* HAL已检查SPI收尾；这里只释放本次刷新并经移植层发送ISR任务通知。 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1 && s_lcd_flush_active != 0 && s_lcd_flush_failed == 0)
    {
        s_lcd_flush_active = 0;
        if (s_lcd_flush_done_cb != NULL)
        {
            s_lcd_flush_done_cb();
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1 && s_lcd_flush_active != 0)
    {
        s_lcd_flush_failed = 1; /* ISR只记错，阻塞式中止留给显示任务。 */
    }
}

uint8_t LCD_Flush_Service(void)
{
    if (s_lcd_flush_failed == 0)
    {
        return 0;
    }
    (void)HAL_SPI_DMAStop(&hspi1);
    if (hspi1.hdmatx != NULL && (hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) != 0)
    {
        return 0; /* DMA尚未停止访问像素缓冲，不能通知LVGL复用。 */
    }
    __HAL_SPI_DISABLE(&hspi1);
    (void)HAL_SPI_Abort(&hspi1);
    ++s_lcd_flush_errors;
    s_lcd_flush_active = 0;
    s_lcd_flush_failed = 0;
    if (s_lcd_flush_done_cb != NULL)
    {
        s_lcd_flush_done_cb();
    }
    return 1;
}

/* 单色填充(局部): 用 64 像素的重复色缓冲分片写 */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint32_t px = (uint32_t)(xend - xsta + 1) * (uint32_t)(yend - ysta + 1);
    uint16_t buf[64];
    uint32_t i;

    for (i = 0; i < 64; i++)
    {
        buf[i] = color;
    }

    LCD_Address_Set(xsta, ysta, xend, yend);
    while (px >= 64)
    {
        if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 128, LCD_SPI_TMO) != HAL_OK)
        {
            Error_Handler();
        }
        px -= 64;
    }
    if (px)
    {
        if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, (uint16_t)(px * 2), LCD_SPI_TMO) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void LCD_Clear(uint16_t color)
{
    LCD_Fill(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, color);
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

/******************************************************************************
 * 背光 (TIM3_CH3 PWM PB0, ARR=300, 频率约3.33kHz, 高电平点亮)
 *****************************************************************************/
void LCD_Open_Light(void)
{
    /* 防御性对齐: CubeMX(.ioc) 已配 Period=300, 此处按 LCD_BL_ARR 重写一次,
     * 防止 .ioc 的 Period 被改动后频率漂移 (Preload=DISABLE, 写 AR 立即生效) */
    __HAL_TIM_SET_AUTORELOAD(&htim3, LCD_BL_ARR);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); /* 先熄灭再启动, 避免闪一下满亮 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void LCD_Set_Light(uint8_t percent)
{
    uint32_t ccr;
    if (percent > 100)
    {
        percent = 100;
    }
#if LCD_BL_CCR_INVERT
    ccr = (uint32_t)(100 - percent) * LCD_BL_ARR / 100;
#else
    ccr = (uint32_t)percent * LCD_BL_ARR / 100; /* 高极性: CCR 越大越亮 */
#endif
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
}

void LCD_Close_Light(void)
{
    LCD_Set_Light(0);
}

/******************************************************************************
 * ST7789 初始化序列 (OV_Watch lcd_init.c, USE_HORIZONTAL==1 竖屏分支)
 *****************************************************************************/
void LCD_Init(void)
{
    /* CS/DC/RST 已由 CubeMX 的 MX_GPIO_Init() 配好推挽输出, 这里只管电平 */
    LCD_CS_Clr(); /* 单从机, CS 常拉低 (同 OV_Watch) */

    /* SPI1_TX DMA 完成中断优先级: 从 CubeMX 的 0 提到 12 (≥configMAX_SYSCALL
     * 等效优先级 11 之下)，允许完成回调使用任务通知唤醒LVGL。 */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 12, 0);

    LCD_RES_Clr(); /* 硬复位 */
    lcd_delay_ms(100);
    LCD_RES_Set();
    lcd_delay_ms(100);

    LCD_WR_REG(0x11); /* Sleep Out */
    lcd_delay_ms(120);

    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0xC0); /* MADCTL: 竖屏 */
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05); /* 16bit RGB565 */

    LCD_WR_REG(0xB2); /* Porch: 0C 0C 00 33 33 */
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x35); /* Gate Control */
    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x19); /* VCOM */
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x2C); /* LCM */
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01); /* VDV/VRH Enable */
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x12); /* VRH Set */
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20); /* VDV Set */
    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x0F); /* Frame Rate 60Hz */
    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1); /* Power Enable */

    LCD_WR_REG(0xE0); /* GTCT 正极性 14 字节 */
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x2B);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x4C);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x0B);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x23);

    LCD_WR_REG(0xE1); /* GTCT 负极性 14 字节 */
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x51);
    LCD_WR_DATA8(0x2F);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x20);
    LCD_WR_DATA8(0x23);

    LCD_WR_REG(0x21); /* Display Inversion ON (OV_Watch 实配) */
    LCD_WR_REG(0x29); /* Display ON */

    LCD_Clear(LCD_BLACK); /* 上电 GRAM 是随机数据, 立即刷黑, 干净启动 */

    LCD_Open_Light();
    LCD_Set_Light(80); /* 默认 80% 亮度 */
}

/******************************************************************************
 * 省电
 *****************************************************************************/
void LCD_ST7789_SleepIn(void)
{
    LCD_WR_REG(0x10);
    lcd_delay_ms(100);
}

void LCD_ST7789_SleepOut(void)
{
    LCD_WR_REG(0x11);
    lcd_delay_ms(100);
}
