/**
  ******************************************************************************
  * @file    touch.c
  * @brief   CST816 电容触摸驱动 (软件模拟 I2C) — 移植自 Watch 工程
  *          interface/touch.c (源头 FryPi OV_Watch BSP/TOUCH/CST816.c, GPL-3.0)。
  *          本工程适配点 (相对 Watch 版):
  *          ① Watch 用硬件 I2C1; 本板 PB7 已作 LCD_RST, 硬件 I2C 走不开 →
  *            回归 OV_Watch 原版软件 I2C: SCL=PB6(TP_SCL), SDA=PB4(TP_SDA,
  *            读写时切换输入/输出方向), RST=PA15(TP_RST)。
  *            (实测本板 TP 的 SDA/SCL 与早期引脚表相反, 已在 CubeMX 对调标签)
  *          ② 本板无 TP_INT 引脚 → 关闭芯片 5s 自动休眠 (0xF9=0), 芯片保持
  *            正常扫描态, 纯轮询永不 NACK 失联; 仍保留"连续失败 1s → 复位
  *            重探测"自愈兜底 (总线卡死/芯片异常时拉起)。
  *          ③ CST816 RST 释放后固件引导需要 ~200ms+, 故复位后等 300ms,
  *            ChipID 读取带重试 (5 次×100ms), 且探测 0x15/0x14 两个候选
  *            从机地址 (不同批次/兼容片)。
  *          软 I2C 速率约 100~150kHz (半周期 ~3µs), 满足 CST816 ≤400kHz。
  ******************************************************************************
***/
#include "touch.h"
#include "my_i2c.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/*--- CST816 寄存器 (同 OV_Watch CST816.h) ---*/
#define REG_GestureID     0x01
#define REG_FingerNum     0x02
#define REG_XposH         0x03 /* 0x03~0x06: XH XL YH YL */
#define REG_ChipID        0xA7
#define REG_SLEEP_CTRL    0xE5
#define REG_AutoSleepTime 0xF9

/* 候选 7bit 从机地址 (转 8bit 写格式): 0x15=CST816S 官方, 0x14=部分兼容片 */
#define CST816_ADDR_1 (0x15 << 1)
#define CST816_ADDR_2 (0x14 << 1)
static uint8_t s_tp_addr8 = CST816_ADDR_1; /* 当前在用的 8bit 写地址 */

/* 触摸面板与 LCD 装配差 180°(同 OV_Watch REVERSE=1): 坐标需镜像。
 * 若实测按下点与显示点呈中心对称错位, 将此宏改 0 即可。 */
#define TOUCH_REVERSE  1
#define TOUCH_SCREEN_W 240
#define TOUCH_SCREEN_H 280

#define TP_RST_0() HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_RESET)
#define TP_RST_1() HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, GPIO_PIN_SET)

/*--- 软 I2C 引脚原语 ---*/
#define TP_SCL_0()  HAL_GPIO_WritePin(TP_SCL_GPIO_Port, TP_SCL_Pin, GPIO_PIN_RESET)
#define TP_SCL_1()  HAL_GPIO_WritePin(TP_SCL_GPIO_Port, TP_SCL_Pin, GPIO_PIN_SET)
#define TP_SDA_0()  HAL_GPIO_WritePin(TP_SDA_GPIO_Port, TP_SDA_Pin, GPIO_PIN_RESET)
#define TP_SDA_1()  HAL_GPIO_WritePin(TP_SDA_GPIO_Port, TP_SDA_Pin, GPIO_PIN_SET)
#define TP_SDA_RD() (HAL_GPIO_ReadPin(TP_SDA_GPIO_Port, TP_SDA_Pin) == GPIO_PIN_SET)

volatile uint8_t g_tp_chip_id = 0; /* 诊断: 最近一次读到的 CST816 ChipID (0xB5/0xB7=正常) */

static void tp_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免调用落在tick末尾时缩短复位/引导等待时间。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
    }
    else
    {
        HAL_Delay(ms);
    }
}

/******************************************************************************
 * 位周期延时 (~3µs/半周期 @100MHz, volatile 防 -O3 吞掉)
 *****************************************************************************/
static void tp_delay(void)
{
    volatile uint32_t n = 80;
    while (n--)
    {
    }
}

/* SDA 方向切换: 输出(主机发送) / 输入+上拉(读 ACK 与数据)
 * 加 PRIMASK 临界区的原因: HAL_GPIO_Init 对 GPIOB 的 MODER/PUPDR 是读-改-写,
 * sensor任务也通过common/my_i2c.c切换同端口PB13的方向。高优先级触摸任务
 * 若打断sensor的读改写，sensor恢复后可能用旧快照覆盖PB4方向。
 * 两边的短临界区让GPIO寄存器更新互斥；MPU EXTI只置标志，不操作总线。 */
static void tp_sda_out(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t primask = __get_PRIMASK();
    g.Pin = TP_SDA_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    __disable_irq();
    HAL_GPIO_Init(TP_SDA_GPIO_Port, &g);
    __set_PRIMASK(primask);
}

static void tp_sda_in(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t primask = __get_PRIMASK();
    g.Pin = TP_SDA_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    __disable_irq();
    HAL_GPIO_Init(TP_SDA_GPIO_Port, &g);
    __set_PRIMASK(primask);
}

/******************************************************************************
 * 软 I2C 基础时序
 *****************************************************************************/
static void iic_start(void)
{
    tp_sda_out();
    TP_SDA_1();
    TP_SCL_1();
    tp_delay();
    TP_SDA_0(); /* SCL 高电平期间 SDA 下降沿 = START */
    tp_delay();
    TP_SCL_0();
    tp_delay();
}

static void iic_stop(void)
{
    tp_sda_out();
    TP_SDA_0();
    tp_delay();
    TP_SCL_1();
    tp_delay();
    TP_SDA_1(); /* SCL 高电平期间 SDA 上升沿 = STOP */
    tp_delay();
}

/* 写 1 字节, 返回 1=收到 NACK(含总线空闲无应答), 0=ACK 成功 */
static uint8_t iic_write_byte(uint8_t dat)
{
    uint8_t i, nack;

    tp_sda_out();
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
        {
            TP_SDA_1();
        }
        else
        {
            TP_SDA_0();
        }
        dat = (uint8_t)(dat << 1);
        tp_delay();
        TP_SCL_1();
        tp_delay();
        TP_SCL_0();
        tp_delay();
    }
    tp_sda_in(); /* 释放总线, 读从机 ACK */
    TP_SCL_1();
    tp_delay();
    if (TP_SDA_RD())
    {
        nack = 1;
    }
    else
    {
        nack = 0;
    }
    TP_SCL_0();
    tp_delay();
    tp_sda_out();
    return nack;
}

/* 读 1 字节, ack=1 应答(继续读), ack=0 发 NACK(最后一字节) */
static uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, dat = 0;

    tp_sda_in();
    for (i = 0; i < 8; i++)
    {
        TP_SCL_1();
        tp_delay();
        if (TP_SDA_RD())
        {
            dat = (uint8_t)((dat << 1) | 1);
        }
        else
        {
            dat = (uint8_t)((dat << 1) | 0);
        }
        TP_SCL_0();
        tp_delay();
    }
    tp_sda_out();
    if (ack)
    {
        TP_SDA_0();
    }
    else
    {
        TP_SDA_1();
    }
    tp_delay();
    TP_SCL_1();
    tp_delay();
    TP_SCL_0();
    tp_delay();
    TP_SDA_1();
    return dat;
}

/******************************************************************************
 * I2C 读写原语 (语义与 Watch 版 HAL_I2C_Mem_Read/Write 对齐, 地址用运行时值)
 *****************************************************************************/
static uint8_t tp_read_regs_at(uint8_t addr8, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    iic_start();
    if (iic_write_byte(addr8))
    {
        iic_stop();
        return 0;
    } /* 从机地址 NACK */
    if (iic_write_byte(reg))
    {
        iic_stop();
        return 0;
    } /* 寄存器地址 NACK */
    iic_start(); /* 重复起始, 转读 */
    if (iic_write_byte((uint8_t)(addr8 | 1)))
    {
        iic_stop();
        return 0;
    }
    for (i = 0; i < len; i++)
    {
        if (i + 1 < len)
        {
            buf[i] = iic_read_byte(1);
        }
        else
        {
            buf[i] = iic_read_byte(0);
        }
    }
    iic_stop();
    return 1;
}

static uint8_t tp_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return tp_read_regs_at(s_tp_addr8, reg, buf, len);
}

static uint8_t tp_write_reg(uint8_t reg, uint8_t val)
{
    iic_start();
    if (iic_write_byte(s_tp_addr8))
    {
        iic_stop();
        return 0;
    }
    if (iic_write_byte(reg))
    {
        iic_stop();
        return 0;
    }
    if (iic_write_byte(val))
    {
        iic_stop();
        return 0;
    }
    iic_stop();
    return 1;
}

/******************************************************************************
 * 复位: 拉低10ms, 释放后等 300ms 固件引导就绪 (CST816 RST→I2C 可用需 200ms+)
 *****************************************************************************/
void Touch_Reset(void)
{
    TP_RST_0();
    tp_delay_ms(10);
    TP_RST_1();
    tp_delay_ms(300);
}

uint8_t Touch_GetChipID(void)
{
    uint8_t id = 0;
    (void)tp_read_regs(REG_ChipID, &id, 1);
    g_tp_chip_id = id; /* 调试器/UI 读这个: 0xB5/0xB7=通, 0=总线未见器件 */
    return id;         /* CST816S≈0xB5, CST816D≈0xB7 */
}

/* 双候选地址探测 ChipID: 命中则记住地址; retries_per_addr 控制每个地址重试次数
 * (上电给 5 覆盖引导窗口; 运行时自愈给 1, 避免 lvgl 任务被拖住 1s+) */
static uint8_t tp_probe(uint8_t retries_per_addr)
{
    static const uint8_t addrs[2] = {CST816_ADDR_1, CST816_ADDR_2};
    uint8_t a, try_n, id;

    for (a = 0; a < 2; a++)
    {
        for (try_n = 0; try_n < retries_per_addr; try_n++)
        {
            if (tp_read_regs_at(addrs[a], REG_ChipID, &id, 1) && id != 0x00 && id != 0xFF)
            {
                s_tp_addr8 = addrs[a];
                g_tp_chip_id = id;
                return id;
            }
            tp_delay_ms(100);
        }
    }
    g_tp_chip_id = 0;
    return 0;
}

uint8_t Touch_Init(void)
{
    uint8_t id;

    /* 总线初始态: SCL/SDA 释放为高 (SDA 先配输出拉高, 后续逐事务切换方向) */
    tp_sda_out();
    TP_SCL_1();
    TP_SDA_1();
    tp_delay_ms(2);

    Touch_Reset();
    id = tp_probe(5);

    /* 本板无 INT 引脚: 关自动休眠, 芯片保持正常扫描态 → 轮询永不失联。
     * 探测失败也照写 (地址可能尚未定, 写失败无害, GetPoint 兜底自愈再关) */
    (void)tp_write_reg(REG_AutoSleepTime, 0x00);

    if (id == 0)
    {
        (void)my_i2c_recover(MY_I2C_BUS_TP); /* 未见器件: 先清一次可能的总线卡死再进运行时自愈 */
    }
    return id;
}

/******************************************************************************
 * 读一个触点坐标 (纯轮询 + 1s 节奏自愈)
 * 返回: 手指数量(0=无触摸/总线失败, 一般 1); x/y 出参为 LCD 对齐坐标
 *
 * 休眠已关闭, 正常态轮询必然得到应答; 若连续失败约 1s (总线卡死/芯片异常),
 * 做一次 "复位+重新探测地址+重关休眠" 自愈; 没有永久放弃的死角 ——
 * 触摸后来才接好/换料也能自动恢复。
 *****************************************************************************/
#define TP_FAIL_STREAK_RECOVER 200 /* 5ms/次 约1s，与LVGL输入周期一致 */
static uint32_t s_tp_fail_streak = 0;

uint8_t Touch_GetPoint(uint16_t *x, uint16_t *y)
{
    uint8_t num, d[4];
    uint16_t px, py;

    if (!tp_read_regs(REG_FingerNum, &num, 1))
    {
        if (++s_tp_fail_streak >= TP_FAIL_STREAK_RECOVER)
        {
            s_tp_fail_streak = 0;
            Touch_Wakeup(); /* 恢复+复位+重探测，约310~510ms；等待时让出CPU */
        }
        return 0;
    }
    s_tp_fail_streak = 0;

    if (num == 0xFF)
    {
        return 0; /* 芯片在休眠扫描(理论已关) */
    }
    if (num == 0x00)
    {
        return 0; /* 无手指 */
    }

    if (!tp_read_regs(REG_XposH, d, 4))
    {
        return 0; /* XposH..YposL 连读 */
    }
    px = (uint16_t)(((uint16_t)(d[0] & 0x0F) << 8) | d[1]);
    py = (uint16_t)(((uint16_t)(d[2] & 0x0F) << 8) | d[3]);

#if TOUCH_REVERSE
    px = TOUCH_SCREEN_W - 1 - px;
    py = TOUCH_SCREEN_H - 1 - py;
#endif
    *x = px;
    *y = py;
    return num;
}

void Touch_Sleep(void)
{
    /* 手动进睡眠 (本工程默认不用; 无 INT 时唤醒只能靠复位) */
    (void)tp_write_reg(REG_SLEEP_CTRL, 0x03);
}

void Touch_Wakeup(void)
{
    (void)my_i2c_recover(MY_I2C_BUS_TP);
    Touch_Reset();
    (void)tp_probe(1);                         /* 两个候选地址各探测一次 */
    (void)tp_write_reg(REG_AutoSleepTime, 0x00); /* 复位后配置丢失, 重关休眠 */
}
