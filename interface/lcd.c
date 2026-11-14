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
 *             CubeMX 已配 PSC=63/ARR=300 → 1MHz/301 ≈ 3.32kHz, LCD_Open_Light
  *             里仍按 LCD_BL_ARR 重写一次 ARR 作防御 (AutoReloadPreload=DISABLE,
  *             立即生效), 保证与驱动宏一致。
  *          依赖: hspi1(spi.c, Mode3: CPOL=High/CPHA=2Edge, MSB, DIV2), htim3(tim.c),
  *                MX_GPIO_Init 已配置 CS/DC/RST 推挽输出。
  ******************************************************************************
***/
#include "lcd.h"
#include "spi.h"
#include "tim.h"
#include "platform_delay.h"

uint8_t lcd_is_stop_idle(void)
{
    return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3) == 0 &&
           (SPI1->SR & SPI_SR_BSY) == 0 && (hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) == 0;
}

/* 引脚快捷宏 (全部来自 CubeMX 生成的 main.h) */
#define LCD_CS_Clr()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_Set()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_DC_Clr()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_Set()  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_RES_Clr() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RES_Set() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)

/* 最大65534字节在32MHz下约16.4ms；50ms保留调度余量并限制故障阻塞时间。 */
#define LCD_SPI_TMO 50
/* 最大65534字节在32MHz下约16.4ms；100ms后仍无回调则按失败收尾。 */
#define LCD_DMA_TIMEOUT_MS 100

/******************************************************************************
 * 底层: 硬件SPI 写 (SPI1 主模式, CubeMX 实配 CPOL=1/CPHA=1 即 Mode3, MSB first;
 *       ST7789 4线时序兼容 Mode0 与 Mode3, 本工程用 Mode3)
 *****************************************************************************/
static void LCD_Writ_Bus(uint8_t dat)
{
    /* dat: 要发往 SPI1 的单个字节；DC 电平由调用方在调用前后控制。 */
    /* 阻塞发送 1 字节，超时上限为 LCD_SPI_TMO（50ms）。 */
    if (HAL_SPI_Transmit(&hspi1, &dat, 1, LCD_SPI_TMO) != HAL_OK)
    {
        Error_Handler(); /* SPI 总线故障, 停机便于调试 */
    }
}

void LCD_WR_DATA8(uint8_t dat)
{
    /* dat: 单字节数据；直接复用底层字节发送，此时 DC 应已为数据电平。 */
    LCD_Writ_Bus(dat);
}

/* 16bit 数据高字节先出 (ST7789 大端要求, 与 LVGL 的 LV_COLOR_16_SWAP=1 配套) */
void LCD_WR_DATA(uint16_t dat)
{
    /* 存放拆分后的高、低字节，按 ST7789 大端顺序发送。 */
    uint8_t tmp[2];
    /* 取高字节放在数组首字节，SPI 先发它。 */
    tmp[0] = (uint8_t)(dat >> 8);
    /* 取低字节放在次字节。 */
    tmp[1] = (uint8_t)(dat);
    /* 一次阻塞发送 2 字节，失败则进入停机处理。 */
    if (HAL_SPI_Transmit(&hspi1, tmp, 2, LCD_SPI_TMO) != HAL_OK)
    {
        Error_Handler();
    }
}

void LCD_WR_REG(uint8_t reg)
{
    /* reg: 命令字节；DC 拉低表示接下来发送的是命令而非显存数据。 */
    LCD_DC_Clr(); /* 命令 */
    LCD_Writ_Bus(reg);
    LCD_DC_Set(); /* 恢复数据模式 */
}

/******************************************************************************
 * 地址窗口 (自动加 Y 偏移; 0x2A 高8bit恒0, 0x2B 按16bit发)
 *****************************************************************************/
static uint8_t lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    /* 11 字节依次为 0x2A+4 参数、0x2B+4 参数、0x2C 写显存命令。 */
    uint8_t data[11];
    /* 分组发送的下标，每次前进 5 字节（1 个命令 + 4 个参数）。 */
    uint8_t i;
    /* 暂存命令字节发送结果，便于失败时直接返回。 */
    HAL_StatusTypeDef status;
    /* 面板显存起始行有纵向偏移，两个 Y 坐标都加上 LCD_OFFSET_Y。 */
    y1 += LCD_OFFSET_Y;
    y2 += LCD_OFFSET_Y;
    /* 列地址设置命令 0x2A；X 坐标高 8 位固定为 0（屏宽 < 256）。 */
    data[0] = 0x2A;
    data[1] = 0;
    data[2] = (uint8_t)x1;
    data[3] = 0;
    data[4] = (uint8_t)x2;
    /* 行地址设置命令 0x2B；Y 坐标按高字节在前、低字节在后排列。 */
    data[5] = 0x2B;
    data[6] = (uint8_t)(y1 >> 8);
    data[7] = (uint8_t)y1;
    data[8] = (uint8_t)(y2 >> 8);
    data[9] = (uint8_t)y2;
    /* 0x2C 为写显存命令，之后的数据都写入刚设定的窗口。 */
    data[10] = 0x2C;
    /* 命令单字节、同组4字节参数连续发送，保持原有DC电平和线上字节序。 */
    for (i = 0; i < sizeof(data); i += 5)
    {
        /* 每组先拉低 DC 发命令字节，发完立即恢复数据电平。 */
        LCD_DC_Clr();
        status = HAL_SPI_Transmit(&hspi1, &data[i], 1, LCD_SPI_TMO);
        LCD_DC_Set();
        /* 命令字节未发出则返回 1，由上层决定是否停机。 */
        if (status != HAL_OK)
        {
            return 1;
        }
        /* 前两组各跟 4 字节坐标参数；最后一组 0x2C 后没有参数，靠 i<10 跳过。 */
        if (i < 10 && HAL_SPI_Transmit(&hspi1, &data[i + 1], 4, LCD_SPI_TMO) != HAL_OK)
        {
            return 1;
        }
    }
    /* 三组命令全部发送成功。 */
    return 0;
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    /* 公共接口：坐标窗口设置失败时直接停机，不向上返回错误。 */
    if (lcd_address_set(x1, y1, x2, y2) != 0)
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
static uint8_t lcd_color_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p)
{
    /* 像素总数 × 2 得到总字节数；先转 uint32 再乘，避免 16 位截断。 */
    uint32_t bytes = (uint32_t)(xend - xsta + 1) * (uint32_t)(yend - ysta + 1) * 2;
    /* 以字节指针遍历像素缓冲，便于按任意字节数分片发送。 */
    const uint8_t *p = (const uint8_t *)color_p;
    /* 记录当前分片发送结果，后续分片只在仍成功时继续。 */
    HAL_StatusTypeDef st = HAL_OK;

    /* 先设置 GRAM 窗口；失败则直接返回 1，不再发送像素。 */
    if (lcd_address_set(xsta, ysta, xend, yend) != 0)
    {
        return 1;
    }
    /* 剩余字节超过单次 HAL 上限（0xFFFF）时循环分片。 */
    while (bytes > 0xFFFF)
    {
        /* 每片 <=64KB-2, 保证整像素边界 */
        st = HAL_SPI_Transmit(&hspi1, (uint8_t *)p, 0xFFFE, LCD_SPI_TMO);
        /* 任一分片失败就跳出循环，尾部数据不再发送。 */
        if (st != HAL_OK)
        {
            break;
        }
        /* 指针前移一片，剩余字节数相应扣减。 */
        p += 0xFFFE;
        bytes -= 0xFFFE;
    }
    /* 全部分片成功且还有不足 0xFFFF 的尾块时，按实际字节数发送。 */
    if (st == HAL_OK && bytes)
    {
        st = HAL_SPI_Transmit(&hspi1, (uint8_t *)p, (uint16_t)bytes, LCD_SPI_TMO);
    }
    /* 0 表示全部发送成功，非 0 表示失败。 */
    return st != HAL_OK;
}

void LCD_Color_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p)
{
    /* 阻塞版整块填充：同步等待整块发送完成；失败进入Error_Handler。 */
    if (lcd_color_fill(xsta, ysta, xend, yend, color_p) != 0)
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
static uint32_t s_lcd_flush_started_ms;

void LCD_Set_Flush_Complete_Callback(LCD_CallbackFunc_t cb)
{
    /* cb: 每次异步刷屏收尾（完成、失败或超时）后要调用的上层回调，可为 NULL。 */
    s_lcd_flush_done_cb = cb;
}

void LCD_Color_Fill_DMA(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, const uint16_t *color_p)
{
    /* 像素数 × 2 得到 DMA 需要搬运的总字节数。 */
    uint32_t bytes = (uint32_t)(xend - xsta + 1) * (uint32_t)(yend - ysta + 1) * 2;
    /* 记录本次发起结果：0 表示已成功启动或已完成，非 0 表示失败。 */
    uint8_t result;

    /* 先记起始时刻供超时判定，再置在途标志，顺序保证服务函数看到有效时间戳。 */
    s_lcd_flush_started_ms = HAL_GetTick();
    s_lcd_flush_active = 1;

    if (bytes > 0xFFFE)
    {
        /* F4 HAL Size 为 uint16: 超 64KB 罕见大块, 退回阻塞分片 */
        result = lcd_color_fill(xsta, ysta, xend, yend, color_p);
        /* 阻塞路径已完成，立即清在途标志并通知上层，不经过 DMA 完成中断。 */
        if (result == 0)
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
        /* 小块走异步：先设窗口，成功后再启动 DMA 发送像素。 */
        result = lcd_address_set(xsta, ysta, xend, yend);
        if (result == 0)
        {
            result = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)color_p, (uint16_t)bytes) != HAL_OK;
        }
    }
    /* 任一步失败则置失败标志，等待显示任务中的LCD_Flush_Service中止传输并通知上层。 */
    if (result != 0)
    {
        s_lcd_flush_failed = 1;
    }
}

/* HAL已检查SPI收尾；这里只释放本次刷新并经移植层发送ISR任务通知。 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* hspi: HAL 传入的完成中断对应句柄，只处理 SPI1 且仅当确有在途刷屏。 */
    if (hspi->Instance == SPI1 && s_lcd_flush_active != 0 && s_lcd_flush_failed == 0)
    {
        /* 先清在途标志，避免回调里再次发起时状态仍为旧值。 */
        s_lcd_flush_active = 0;
        if (s_lcd_flush_done_cb != NULL)
        {
            s_lcd_flush_done_cb();
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    /* 仅对 SPI1 的在途刷屏记错；未在途的中断不改变状态。 */
    if (hspi->Instance == SPI1 && s_lcd_flush_active != 0)
    {
        s_lcd_flush_failed = 1; /* ISR只记错，阻塞式中止留给显示任务。 */
    }
}

uint8_t LCD_Flush_Service(void)
{
    /* 保存并随后恢复PRIMASK，与SPI/DMA等可屏蔽中断互斥判定超时。 */
    uint32_t primask = __get_PRIMASK();
    /* 与完成ISR互斥认领超时；HAL中止及完成通知均在恢复中断后执行。 */
    __disable_irq();
    /* 在途且距发起已满 LCD_DMA_TIMEOUT_MS（100ms）时，标记本次刷屏失败。 */
    if (s_lcd_flush_active != 0 && (uint32_t)(HAL_GetTick() - s_lcd_flush_started_ms) >= LCD_DMA_TIMEOUT_MS)
    {
        s_lcd_flush_failed = 1;
    }
    __set_PRIMASK(primask);
    /* 没有失败标志时无需收尾，直接返回 0。 */
    if (s_lcd_flush_failed == 0)
    {
        return 0;
    }
    /* 有失败标志：先请求停止 DMA，返回值不参与判断。 */
    (void)HAL_SPI_DMAStop(&hspi1);
    /* DMA 通道仍使能说明还在访问像素缓冲，此时不能通知 LVGL 复用该缓冲。 */
    if (hspi1.hdmatx != NULL && (hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) != 0)
    {
        return 0; /* DMA尚未停止访问像素缓冲，不能通知LVGL复用。 */
    }
    /* DMA 已停：关闭 SPI 外设并中止 HAL 侧传输状态，便于下一笔重新开始。 */
    __HAL_SPI_DISABLE(&hspi1);
    (void)HAL_SPI_Abort(&hspi1);
    /* 累计失败次数供调试观察，然后清在途与失败标志。 */
    ++s_lcd_flush_errors;
    s_lcd_flush_active = 0;
    s_lcd_flush_failed = 0;
    if (s_lcd_flush_done_cb != NULL)
    {
        s_lcd_flush_done_cb();
    }
    /* 1 表示本次服务确实完成了一次异常收尾。 */
    return 1;
}

/* 单色填充(局部): 用 64 像素的重复色缓冲分片写 */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    /* 待填充的像素总数，用 32 位保存避免大区域乘出 16 位溢出。 */
    uint32_t px = (uint32_t)(xend - xsta + 1) * (uint32_t)(yend - ysta + 1);
    /* 128 字节缓冲 = 64 个 RGB565 像素，可被 64 像素步长整除。 */
    uint8_t buf[128];
    /* 预填充缓冲的循环下标。 */
    uint32_t i;

    /* 把同一个颜色按大端字节序重复写入 64 份，作为后续分片的数据源。 */
    for (i = 0; i < 64; i++)
    {
        buf[i * 2] = (uint8_t)(color >> 8);
        buf[i * 2 + 1] = (uint8_t)color;
    }

    /* 设置目标窗口，之后写入的像素按窗口自动推进。 */
    LCD_Address_Set(xsta, ysta, xend, yend);
    /* 每满 64 像素就整缓冲发送一次。 */
    while (px >= 64)
    {
        if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 128, LCD_SPI_TMO) != HAL_OK)
        {
            Error_Handler();
        }
        px -= 64;
    }
    /* 剩余不足 64 像素时，只发实际需要的 px*2 字节。 */
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
    /* 以给定颜色覆盖整屏，坐标取 0 到宽高减一。 */
    LCD_Fill(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, color);
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    /* 单像素窗口：起止坐标相同，再写一个 16 位颜色值。 */
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

/******************************************************************************
 * 背光 (TIM3_CH3 PWM PB0, ARR=300, 频率约3.32kHz, 高电平点亮)
 *****************************************************************************/
void LCD_Open_Light(void)
{
    /* 防御性对齐: CubeMX(.ioc) 已配 Period=300, 此处按 LCD_BL_ARR 重写一次,
     * 防止 .ioc 的 Period 被改动后频率漂移 (Preload=DISABLE, 写 AR 立即生效) */
    __HAL_TIM_SET_AUTORELOAD(&htim3, LCD_BL_ARR);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); /* 先熄灭再启动, 避免闪一下满亮 */
    /* 启动 TIM3 通道 3 的 PWM 输出；此后比较值由 LCD_Set_Light 决定。 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void LCD_Set_Light(uint8_t percent)
{
    /* 换算后的比较值，写入 TIM3_CH3 决定背光占空比。 */
    uint32_t ccr;
    /* 入参按百分比解释，超过 100 一律压到满亮。 */
    if (percent > 100)
    {
        percent = 100;
    }
#if LCD_BL_CCR_INVERT
    /* 反极性接法：亮度百分比越高，比较值越小。 */
    ccr = (uint32_t)(100 - percent) * LCD_BL_ARR / 100;
#else
    ccr = (uint32_t)percent * LCD_BL_ARR / 100; /* 高极性: CCR 越大越亮 */
#endif
    /* 立即更新通道 3 比较值，改变占空比。 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
}

void LCD_Close_Light(void)
{
    /* 亮度设为 0，即占空比为 0。 */
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
    /* SPI1 自身中断同样设为 12，并显式使能，供 HAL 的 SPI 状态机使用。 */
    HAL_NVIC_SetPriority(SPI1_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    /* 硬复位：RST 拉低保持 100ms，再释放并等待 100ms 让面板稳定。 */
    LCD_RES_Clr(); /* 硬复位 */
    platform_delay_min_ms(100);
    LCD_RES_Set();
    platform_delay_min_ms(100);

    /* 0x11 Sleep Out：退出睡眠后等待 120ms 再继续配置。 */
    LCD_WR_REG(0x11); /* Sleep Out */
    platform_delay_min_ms(120);

    /* 0x36 MADCTL 设扫描方向为竖屏，0x3A 设像素格式为 16bit RGB565。 */
    LCD_WR_REG(0x36);
    LCD_WR_DATA8(0xC0); /* MADCTL: 竖屏 */
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05); /* 16bit RGB565 */

    /* 0xB2 Porch 设置：按 OV_Watch 实配的 5 字节顺序发送。 */
    LCD_WR_REG(0xB2); /* Porch: 0C 0C 00 33 33 */
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    /* 0xB7 写 Gate Control 寄存器 0x35。 */
    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x35); /* Gate Control */
    /* 0xBB 写 VCOM 寄存器 0x19。 */
    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x19); /* VCOM */
    /* 0xC0 写 LCM 控制字节 0x2C。 */
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x2C); /* LCM */
    /* 0xC2 写 VDV/VRH 使能寄存器 0x01。 */
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01); /* VDV/VRH Enable */
    /* 0xC3 写 VRH 设定值 0x12。 */
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x12); /* VRH Set */
    /* 0xC4 写 VDV 设定值 0x20。 */
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20); /* VDV Set */
    /* 0xC6 写帧率寄存器 0x0F。 */
    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x0F); /* Frame Rate 60Hz */
    /* 0xD0 写电源控制两字节：0xA4、0xA1。 */
    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1); /* Power Enable */

    /* 0xE0 写正极性伽马表，共 14 字节，顺序按 OV_Watch 实配。 */
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

    /* 0xE1 写负极性伽马表，同样是 14 字节。 */
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

    /* 打开显示反显，再发 0x29 开显示。 */
    LCD_WR_REG(0x21); /* Display Inversion ON (OV_Watch 实配) */
    LCD_WR_REG(0x29); /* Display ON */

    LCD_Clear(LCD_BLACK); /* 上电 GRAM 是随机数据, 立即刷黑, 干净启动 */

    /* 启动背光 PWM，开机默认亮度使用共享宏，避免上电高亮。 */
    LCD_Open_Light();
    LCD_Set_Light(LCD_DEFAULT_BRIGHTNESS_PERCENT); /* 默认 LCD_DEFAULT_BRIGHTNESS_PERCENT 亮度 */
}

/******************************************************************************
 * 省电
 *****************************************************************************/
static uint8_t lcd_power_command(uint8_t command)
{
    /* command: 电源类单字节命令（0x10 睡眠、0x11 唤醒）；result 暂存发送结果。 */
    HAL_StatusTypeDef result;
    LCD_DC_Clr();
    result = HAL_SPI_Transmit(&hspi1, &command, 1, LCD_SPI_TMO);
    LCD_DC_Set();
    /* 发送失败时解除 SPI 的 BUSY/错误状态，使后续电源命令还能重试。 */
    if (result != HAL_OK)
    {
        /* 此处DMA已收尾；解除SPI的BUSY/错误状态，让下一轮电源命令能够重试。 */
        (void)HAL_SPI_Abort(&hspi1);
        return 1;
    }
    /* 0 表示命令已发出，1 表示发送失败并尝试中止。 */
    return 0;
}

uint8_t LCD_ST7789_SleepIn(void)
{
    /* 先把背光占空比降到 0，避免面板休眠过程中仍亮着。 */
    LCD_Set_Light(0);
    /* 发送 0x10 进入睡眠；失败直接返回 1，后续断电步骤不执行。 */
    if (lcd_power_command(0x10) != 0)
    {
        return 1;
    }
    /* 命令发出后等 100ms，再停时钟、关外设。 */
    platform_delay_min_ms(100);
    /* 移植层已等DMA收尾；CCR=0保持PB0低电平，停计数并关闭显示外设时钟。 */
    htim3.Instance->CR1 &= ~TIM_CR1_CEN;
    __HAL_SPI_DISABLE(&hspi1);
    __HAL_RCC_TIM3_CLK_DISABLE();
    __HAL_RCC_SPI1_CLK_DISABLE();
    __HAL_RCC_DMA2_CLK_DISABLE();
    /* 0 表示休眠命令已成功发出并完成外设关闭。 */
    return 0;
}

uint8_t LCD_ST7789_SleepOut(void)
{
    /* 依次恢复 DMA2、SPI1、TIM3 的时钟，供后续命令与 PWM 使用。 */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_TIM_ENABLE(&htim3); /* 先以CCR=0恢复，应用层在面板醒后设置原亮度。 */
    /* 发送 0x11 退出睡眠；失败返回 1，调用方需据此决定是否继续显示。 */
    if (lcd_power_command(0x11) != 0)
    {
        return 1;
    }
    platform_delay_min_ms(120); /* 与初始化Sleep Out相同，等待面板恢复后再绘图、开背光。 */
    /* 0 表示唤醒命令已发送并等待了恢复时间。 */
    return 0;
}
