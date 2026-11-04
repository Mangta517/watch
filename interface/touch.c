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
  *            正常扫描并轮询, 通信仍可能失败; 连续失败200次后由后续轮询分步
  *            复位重探测, 经过时间随调度变化 (总线卡死/芯片异常时拉起)。
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
    /* 先确认调度器已启动：只有运行中才能调用 FreeRTOS 的阻塞延时 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免调用落在tick末尾时缩短复位/引导等待时间。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1);
    }
    else
    {
        /* 调度器未启动（如 main 初始化阶段）时改用 HAL 忙等，参数单位为 ms */
        HAL_Delay(ms);
    }
}

/******************************************************************************
 * APP使用64MHz的51次位延时；volatile防止优化掉，实际总线频率需实测。
 *****************************************************************************/
static void tp_delay(void)
{
    /* n 为循环次数；volatile 阻止编译器把这段空循环整体优化掉 */
    volatile uint32_t n = 51;
    /* 逐次自减到 0，用空循环消耗时间，构成软 I2C 的半周期延时 */
    while (n--)
    {
    }
}

/* SDA 方向切换: 输出(主机发送) / 输入+上拉(读 ACK 与数据)
 * 加 PRIMASK 临界区的原因: HAL_GPIO_Init 对 GPIOB 的 MODER/PUPDR 是读-改-写,
 * sensor可能抢占touch对GPIOB的读改写，touch恢复后可能用旧值覆盖sensor的SDA方向；
 * 两边短临界区使同端口GPIO更新互斥。MPU EXTI只置标志，不操作总线。 */
static void tp_sda_out(void)
{
    /* g 描述本次要写入的 SDA 引脚配置；先整体清零，避免未赋值的字段带随机值 */
    GPIO_InitTypeDef g = {0};
    /* 保存当前 PRIMASK，进入临界区后据此恢复中断原来的开关状态 */
    uint32_t primask = __get_PRIMASK();
    /* 指定只重配 SDA 这一根引脚，不动同端口的其他引脚 */
    g.Pin = TP_SDA_Pin;
    /* 推挽输出：主机需要主动把 SDA 拉低或拉高 */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    /* 使能内部上拉，与后续输入态配置保持一致 */
    g.Pull = GPIO_PULLUP;
    /* 高速档，缩短引脚翻转时间，减小软 I2C 边沿延迟 */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    /* 关中断，避免别的任务在读改写 GPIOB 寄存器时插入本次配置 */
    __disable_irq();
    /* 真正写入 MODER/PUPDR 等寄存器，把 SDA 切成输出 */
    HAL_GPIO_Init(TP_SDA_GPIO_Port, &g);
    /* 恢复进入前的 PRIMASK，即回到原来的中断开关状态 */
    __set_PRIMASK(primask);
}

static void tp_sda_in(void)
{
    /* g 描述本次要写入的 SDA 引脚配置；先清零，保证其余字段有确定值 */
    GPIO_InitTypeDef g = {0};
    /* 记录当前 PRIMASK，临界区结束后恢复原有中断状态 */
    uint32_t primask = __get_PRIMASK();
    /* 只操作 SDA 引脚 */
    g.Pin = TP_SDA_Pin;
    /* 输入模式：让从机可以驱动 SDA，主机只读取电平 */
    g.Mode = GPIO_MODE_INPUT;
    /* 输入态启用上拉，总线无人驱动时读到高电平 */
    g.Pull = GPIO_PULLUP;
    /* 输入模式下速度字段仍按高速填写，保持与输出配置一致 */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    /* 关中断，与 tp_sda_out 以及 my_i2c 的端口读改写互斥 */
    __disable_irq();
    /* 写入寄存器，把 SDA 切成输入 */
    HAL_GPIO_Init(TP_SDA_GPIO_Port, &g);
    /* 恢复进入前的中断屏蔽状态 */
    __set_PRIMASK(primask);
}

/******************************************************************************
 * 软 I2C 基础时序
 *****************************************************************************/
static void iic_start(void)
{
    /* 先把 SDA 预置高电平，保证下面切输出时不会误拉低总线 */
    TP_SDA_1();
    /* 把 SDA 切成推挽输出，之后主机才能主动产生边沿 */
    tp_sda_out();
    /* 释放 SCL 为高，准备在 SCL 高电平期间制造 START 条件 */
    TP_SCL_1();
    /* 等待电平稳定，保证 SCL 已经为高 */
    tp_delay();
    TP_SDA_0(); /* SCL 高电平期间 SDA 下降沿 = START */
    /* 保持该下降沿一段时间，满足从机的建立时间要求 */
    tp_delay();
    /* 拉低 SCL，进入第一个数据位的低电平准备阶段 */
    TP_SCL_0();
    /* 等 SCL 低电平稳定后再返回，调用方可以开始写字节 */
    tp_delay();
}

static void iic_stop(void)
{
    /* 先把 SDA 预置为低，切输出后即把总线数据线驱动为低 */
    TP_SDA_0();
    /* SDA 切换为推挽输出，主机主动控制数据线 */
    tp_sda_out();
    /* 等待低电平建立 */
    tp_delay();
    /* 释放 SCL 为高，准备制造 STOP 的上升沿 */
    TP_SCL_1();
    /* 等待 SCL 高电平稳定 */
    tp_delay();
    TP_SDA_1(); /* SCL 高电平期间 SDA 上升沿 = STOP */
    /* 保持 STOP 条件一段时间，让从机识别总线结束 */
    tp_delay();
}

/* 写 1 字节, 返回 1=收到 NACK(含总线空闲无应答), 0=ACK 成功 */
static uint8_t iic_write_byte(uint8_t dat)
{
    /* i 为当前正在发送的位序号（0~7）；nack 保存第 9 拍读到的应答结果 */
    uint8_t i, nack;

    /* 发送前确保 SDA 由主机驱动 */
    tp_sda_out();
    /* 从最高位开始逐位送出，共 8 位 */
    for (i = 0; i < 8; i++)
    {
        /* 判断当前最高位是否为 1，决定 SDA 输出高还是低 */
        if (dat & 0x80)
        {
            /* 该位为 1：SDA 输出高电平 */
            TP_SDA_1();
        }
        else
        {
            /* 该位为 0：SDA 输出低电平 */
            TP_SDA_0();
        }
        /* 左移一位，把下一位送到最高位，供下一轮判断 */
        dat = (uint8_t)(dat << 1);
        /* 等待 SDA 数据建立 */
        tp_delay();
        /* 抬高 SCL，从机在此上升沿采样数据位 */
        TP_SCL_1();
        /* 保持 SCL 高电平一个位时间 */
        tp_delay();
        /* 拉低 SCL，结束本位的传输 */
        TP_SCL_0();
        /* 等低电平稳定后再送下一位 */
        tp_delay();
    }
    tp_sda_in(); /* 释放总线, 读从机 ACK */
    /* 第 9 拍把 SCL 抬高，从机在这期间拉低 SDA 表示应答 */
    TP_SCL_1();
    /* 等待从机把 ACK 电平建立起来 */
    tp_delay();
    /* 读到高电平说明从机没有拉低 SDA，即 NACK 或总线无人应答 */
    if (TP_SDA_RD())
    {
        /* 记录本次传输未获应答 */
        nack = 1;
    }
    else
    {
        /* 读到低电平说明从机拉低了 SDA，即 ACK */
        nack = 0;
    }
    /* 拉低 SCL，结束 ACK 这一拍 */
    TP_SCL_0();
    /* 等 SCL 低电平稳定 */
    tp_delay();
    /* ACK 后保持释放；读地址后的下一位已由从机发送。 */
    return nack;
}

/* 读 1 字节, ack=1 应答(继续读), ack=0 发 NACK(最后一字节) */
static uint8_t iic_read_byte(uint8_t ack)
{
    /* i 为已读位数；dat 逐位拼出从机送来的一个字节，初值清零 */
    uint8_t i, dat = 0;

    /* 读取前把 SDA 切成输入，交由从机驱动数据线 */
    tp_sda_in();
    /* 连续读 8 位，从最高位开始拼装 */
    for (i = 0; i < 8; i++)
    {
        /* 抬高 SCL，从机在 SCL 高电平期间输出当前数据位 */
        TP_SCL_1();
        /* 等待从机数据位稳定 */
        tp_delay();
        /* 采样 SDA：高电平表示该位为 1 */
        if (TP_SDA_RD())
        {
            /* 左移一位后把最低位置 1，完成该位的拼装 */
            dat = (uint8_t)((dat << 1) | 1);
        }
        else
        {
            /* 该位为 0：左移后最低位保持 0 */
            dat = (uint8_t)((dat << 1) | 0);
        }
        /* 拉低 SCL，结束本位的读取 */
        TP_SCL_0();
        /* 等低电平稳定后读下一位 */
        tp_delay();
    }
    /* ack=1 表示还要继续读，需要主机回 ACK 让从机送出下一字节 */
    if (ack)
    {
        TP_SDA_0(); /* 先预置低电平，再接管 SDA 发 ACK。 */
        /* 预置好低电平后再切输出，避免切换瞬间产生毛刺 */
        tp_sda_out();
    }
    else
    {
        tp_sda_in(); /* 最后一字节的 NACK 由释放 SDA 形成。 */
    }
    /* 等待 ACK/NACK 电平建立 */
    tp_delay();
    /* 抬高 SCL，让从机在第九拍采样应答 */
    TP_SCL_1();
    /* 保持 SCL 高电平一个位时间 */
    tp_delay();
    /* 拉低 SCL，结束应答拍 */
    TP_SCL_0();
    tp_sda_in(); /* 第九拍结束即交还 SDA，允许从机准备下一字节。 */
    /* 等总线稳定后返回读到的字节 */
    tp_delay();
    return dat;
}

/******************************************************************************
 * I2C 读写原语 (0=成功、1=失败，地址用运行时值)
 *****************************************************************************/
static uint8_t tp_read_regs_at(uint8_t addr8, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* i 为本次要读入的字节下标，从 0 到 len-1 */
    uint16_t i;

    /* 产生 START，开始一次“先写寄存器地址、再读数据”的事务 */
    iic_start();
    /* 发送 8bit 写地址；有 NACK 说明从机不在或地址不对 */
    if (iic_write_byte(addr8))
    {
        /* 发送 STOP 释放总线，避免把总线留在占用状态 */
        iic_stop();
        /* 返回 1 表示本次读寄存器失败 */
        return 1;
    }
    /* 写入要读取的寄存器起始地址 */
    if (iic_write_byte(reg))
    {
        /* 寄存器地址未被应答，同样先结束事务 */
        iic_stop();
        /* 返回 1 表示失败 */
        return 1;
    }
    iic_start(); /* 重复起始, 转读 */
    /* 发送读地址：最低位置 1 表示本事务进入读方向 */
    if (iic_write_byte((uint8_t)(addr8 | 1)))
    {
        /* 从机未应答读地址，收尾后返回失败 */
        iic_stop();
        /* 返回 1 表示失败 */
        return 1;
    }
    /* 按 len 逐字节读入缓冲区 */
    for (i = 0; i < len; i++)
    {
        /* 不是最后一字节时回 ACK，通知从机继续送下一个字节 */
        if (i + 1 < len)
        {
            /* 读一个字节并应答，结果存入缓冲区第 i 个位置 */
            buf[i] = iic_read_byte(1);
        }
        else
        {
            /* 最后一字节回 NACK，读完即结束 */
            buf[i] = iic_read_byte(0);
        }
    }
    /* 产生 STOP，结束整次读事务 */
    iic_stop();
    /* 返回 0 表示本次读寄存器成功 */
    return 0;
}

static uint8_t tp_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* 使用探测阶段确定的当前从机地址，转调带地址参数的读函数；0=成功、1=失败 */
    return tp_read_regs_at(s_tp_addr8, reg, buf, len);
}

static uint8_t tp_write_reg(uint8_t reg, uint8_t val)
{
    /* 产生 START，开始一次写寄存器事务 */
    iic_start();
    /* 发送写地址；NACK 表示从机没有响应 */
    if (iic_write_byte(s_tp_addr8))
    {
        /* 结束事务，释放总线 */
        iic_stop();
        /* 返回 1 表示写失败 */
        return 1;
    }
    /* 发送目标寄存器地址 */
    if (iic_write_byte(reg))
    {
        /* 寄存器地址未被应答，先发 STOP */
        iic_stop();
        /* 返回 1 表示写失败 */
        return 1;
    }
    /* 发送要写入的 8bit 数据 */
    if (iic_write_byte(val))
    {
        /* 数据未被应答，同样收尾 */
        iic_stop();
        /* 返回 1 表示写失败 */
        return 1;
    }
    /* 三步都被应答，产生 STOP 结束写事务 */
    iic_stop();
    /* 返回 0 表示写寄存器成功 */
    return 0;
}

/******************************************************************************
 * 复位: 拉低10ms, 释放后等 300ms 固件引导就绪 (CST816 RST→I2C 可用需 200ms+)
 *****************************************************************************/
void Touch_Reset(void)
{
    /* 拉低 RST，让 CST816 进入复位状态 */
    TP_RST_0();
    /* 保持复位低电平 10ms */
    tp_delay_ms(10);
    /* 释放 RST，芯片开始重新引导 */
    TP_RST_1();
    /* 再等 300ms，给芯片固件引导留出时间，之后才尝试 I2C 访问 */
    tp_delay_ms(300);
}

uint8_t Touch_GetChipID(void)
{
    /* id 保存本次读到的 ChipID；初值 0 便于总线无响应时区分 */
    uint8_t id = 0;
    /* 读 ChipID 寄存器 1 字节；这里忽略返回值，读失败时 id 保持 0 */
    (void)tp_read_regs(REG_ChipID, &id, 1);
    g_tp_chip_id = id; /* 调试器/UI 读这个: 0xB5/0xB7=通, 0=总线未见器件 */
    return id;         /* CST816S≈0xB5, CST816D≈0xB7 */
}

/* 双候选地址探测 ChipID: 命中则记住地址; retries_per_addr 控制每个地址重试次数
 * (上电给 5 覆盖引导窗口; 运行时自愈给 1, 避免 lvgl 任务被拖住 1s+) */
static uint8_t tp_probe(uint8_t retries_per_addr)
{
    /* 两个候选 8bit 写地址：不同批次/兼容片可能只响应其中之一 */
    static const uint8_t addrs[2] = {CST816_ADDR_1, CST816_ADDR_2};
    /* a 为地址下标，try_n 为当前地址的尝试次数，id 为读到的 ChipID */
    uint8_t a, try_n, id;

    /* 依次尝试两个候选地址 */
    for (a = 0; a < 2; a++)
    {
        /* 每个地址最多尝试 retries_per_addr 次，覆盖芯片引导窗口 */
        for (try_n = 0; try_n < retries_per_addr; try_n++)
        {
            /* 读成功且 ChipID 既非 0x00 也非 0xFF，才视为探测到有效器件 */
            if (tp_read_regs_at(addrs[a], REG_ChipID, &id, 1) == 0 && id != 0x00 && id != 0xFF)
            {
                /* 记住命中的地址，后续读写都用它 */
                s_tp_addr8 = addrs[a];
                /* 同步诊断用的全局 ChipID */
                g_tp_chip_id = id;
                /* 返回读到的 ChipID，探测成功 */
                return id;
            }
            /* 本次失败，等 100ms 再试，给芯片上电/复位后的引导留时间 */
            tp_delay_ms(100);
        }
    }
    /* 两个地址都没探到有效 ID，清掉诊断值 */
    g_tp_chip_id = 0;
    /* 返回 0 表示未探测到器件 */
    return 0;
}

static uint8_t tp_configure(void);

uint8_t Touch_Init(void)
{
    /* id 保存探测到的 ChipID；0 表示未探到器件 */
    uint8_t id;

    /* 总线初始态: SCL/SDA 释放为高 (SDA 先配输出拉高, 后续逐事务切换方向) */
    tp_sda_out();
    /* 释放 SCL 为高，使总线处于空闲电平 */
    TP_SCL_1();
    /* 把 SDA 也拉高，完成空闲态建立 */
    TP_SDA_1();
    /* 等 2ms 让两条线的电平稳定后再发起复位 */
    tp_delay_ms(2);

    /* 硬复位触摸芯片，并等待其固件引导完成 */
    Touch_Reset();
    /* 每个候选地址重试 5 次，覆盖上电引导窗口 */
    id = tp_probe(5);

    /* 本板无 INT 引脚；关闭自动休眠失败时，由后续轮询继续配置和恢复。 */
    if (tp_configure() != 0)
    {
        /* 配置失败说明当前通信不可用，对外按未初始化处理 */
        id = 0;
    }

    /* 未探到器件时尝试一次总线恢复，清掉可能被从机拉死的总线 */
    if (id == 0)
    {
        (void)my_i2c_recover(MY_I2C_BUS_TP); /* 初始化未成功: 先清一次可能的总线卡死再进运行时自愈 */
    }
    /* 返回ChipID；0表示本次初始化未成功（探测或配置失败）。 */
    return id;
}

/******************************************************************************
 * 读一个触点坐标，纯轮询；连续失败200次后由后续轮询逐拍推进恢复。
 * 返回: 0=手指已松开，TOUCH_READ_ERROR=读取不可用，其余为手指数量。
 * 有触点时才更新x/y屏幕坐标，读取失败或恢复中不能据此确认松手。
 * 恢复流程包括复位、地址探测和关闭自动休眠，实际耗时随任务调度变化。
 *****************************************************************************/
#define TP_FAIL_STREAK_RECOVER 200 /* touch任务目标5ms/次；实际恢复时间随抢占延长。 */
static uint32_t s_tp_fail_streak = 0;

enum
{
    TP_RECOVER_IDLE,
    TP_RECOVER_RESET_WAIT,
    TP_RECOVER_BOOT_WAIT,
    TP_RECOVER_PROBE,
    TP_RECOVER_PROBE_WAIT,
    TP_RECOVER_CONFIG
};
static struct
{
    uint32_t phase_ms;
    uint8_t phase, address_index;
} tp_recovery;

static void tp_recover_begin(void)
{
    /* 先做一次总线恢复，尝试让被拉低的 SDA/SCL 回到空闲态 */
    (void)my_i2c_recover(MY_I2C_BUS_TP);
    /* 拉低 RST，启动一次硬复位序列 */
    TP_RST_0();
    /* 记录本阶段起点，后续按毫秒时间推进复位/引导等待 */
    tp_recovery.phase_ms = HAL_GetTick();
    /* 进入“等待复位低电平保持时间”阶段 */
    tp_recovery.phase = TP_RECOVER_RESET_WAIT;
    /* 地址探测从第一个候选地址开始 */
    tp_recovery.address_index = 0;
    /* 复位期间芯片不可读，清掉诊断 ChipID */
    g_tp_chip_id = 0;
}

/* 手指数和坐标共享连续失败计数，复位后的等待由后续输入轮询推进。 */
static void tp_read_failed(void)
{
    /* 先累加连续失败次数，再判断是否达到触发恢复的阈值 */
    if (++s_tp_fail_streak >= TP_FAIL_STREAK_RECOVER)
    {
        /* 达到阈值后清零计数，下一轮失败重新累计 */
        s_tp_fail_streak = 0;
        /* 启动复位+重新探测的异步恢复流程 */
        tp_recover_begin();
    }
}

/* 初始化、唤醒和运行时恢复共用配置结果，成功前不发布触摸采样。 */
static uint8_t tp_configure(void)
{
    /* 写自动休眠时间寄存器为 0，关闭芯片自动休眠（本板无 INT，需保持扫描） */
    if (tp_write_reg(REG_AutoSleepTime, 0x00) != 0)
    {
        /* 写失败时把恢复状态机指向配置阶段，等后续轮询重试 */
        tp_recovery.phase = TP_RECOVER_CONFIG;
        /* 记一次失败，累计到阈值会重新走复位恢复 */
        tp_read_failed();
        /* 返回 1 表示本次配置失败 */
        return 1;
    }
    /* 配置成功，清空连续失败计数 */
    s_tp_fail_streak = 0;
    /* 恢复状态机回到空闲，允许 Touch_GetPoint 正常读点 */
    tp_recovery.phase = TP_RECOVER_IDLE;
    /* 返回 0 表示配置成功 */
    return 0;
}

/* 与输入读取同属touch任务；每次至多一笔事务，毫秒等待只检查墙钟。 */
static void tp_recover_step(void)
{
    /* 恢复流程内部使用的两个候选 8bit 写地址，与 tp_probe 保持一致 */
    static const uint8_t addrs[2] = {CST816_ADDR_1, CST816_ADDR_2};
    /* now_ms 为本拍进入时的墙钟毫秒，用于判断各等待阶段是否到时 */
    uint32_t now_ms = HAL_GetTick();
    /* id 暂存探测阶段读到的 ChipID */
    uint8_t id;
    /* 按当前阶段只推进一步，不在一次调用里等待或循环 */
    switch (tp_recovery.phase)
    {
    case TP_RECOVER_RESET_WAIT:
        /* 复位低电平需保持 10ms，用无符号差值计算已过时间 */
        if ((uint32_t)(now_ms - tp_recovery.phase_ms) >= 10)
        {
            /* 到时释放 RST，芯片开始引导 */
            TP_RST_1();
            /* 重新记录时间戳，作为引导等待的起点 */
            tp_recovery.phase_ms = HAL_GetTick();
            /* 进入引导等待阶段 */
            tp_recovery.phase = TP_RECOVER_BOOT_WAIT;
        }
        /* 未到 10ms 则本拍直接结束，下次轮询再检查 */
        break;
    case TP_RECOVER_BOOT_WAIT:
        /* 释放 RST 后需等 300ms 让固件引导完成、I2C 可用 */
        if ((uint32_t)(now_ms - tp_recovery.phase_ms) >= 300)
        {
            /* 引导等待结束，进入地址探测阶段 */
            tp_recovery.phase = TP_RECOVER_PROBE;
        }
        /* 未到 300ms 则本拍直接结束 */
        break;
    case TP_RECOVER_PROBE:
        /* 用当前候选地址读 ChipID，成功且 ID 非 0x00/0xFF 才算探到器件 */
        if (tp_read_regs_at(addrs[tp_recovery.address_index], REG_ChipID, &id, 1) == 0 && id != 0 && id != 0xFF)
        {
            /* 记住命中的地址供后续读写使用 */
            s_tp_addr8 = addrs[tp_recovery.address_index];
            /* 更新诊断用 ChipID */
            g_tp_chip_id = id;
            /* 探测成功，进入配置阶段 */
            tp_recovery.phase = TP_RECOVER_CONFIG;
        }
        else
        {
            /* 本地址本次未探到，记录时间戳后进入重试间隔 */
            tp_recovery.phase_ms = HAL_GetTick();
            /* 进入探测间隔等待阶段 */
            tp_recovery.phase = TP_RECOVER_PROBE_WAIT;
        }
        /* 探测只做一笔事务，结束本拍 */
        break;
    case TP_RECOVER_PROBE_WAIT:
        /* 同一地址两次探测之间至少间隔 100ms */
        if ((uint32_t)(now_ms - tp_recovery.phase_ms) >= 100)
        {
            /* 间隔到，切到下一个候选地址 */
            ++tp_recovery.address_index;
            /* 还有候选地址就回到探测阶段继续尝试 */
            if (tp_recovery.address_index < 2)
            {
                /* 用下一个地址再探一次 */
                tp_recovery.phase = TP_RECOVER_PROBE;
            }
            else
            {
                /* 两个地址都试过仍失败，仍进入配置阶段，由写寄存器结果决定后续 */
                tp_recovery.phase = TP_RECOVER_CONFIG;
            }
        }
        /* 未满 100ms 则本拍直接结束 */
        break;
    case TP_RECOVER_CONFIG:
        /* 重新写自动休眠配置；失败时 tp_configure 内部会安排下一轮恢复 */
        (void)tp_configure();
        /* 本拍结束，成功与否由 tp_configure 更新状态 */
        break;
    default:
        /* 阶段值不匹配上面任一分支时不做动作，本拍直接结束 */
        break;
    }
}

uint8_t Touch_GetPoint(uint16_t *x, uint16_t *y)
{
    /* num 为读到的触点数量；d 暂存 X/Y 共 4 个坐标字节 */
    uint8_t num, d[4];
    /* px 暂存 X 坐标、py 暂存 Y 坐标；均为 12bit 原始值，镜像后才成为屏幕坐标 */
    uint16_t px, py;

    /* 恢复流程未回到空闲时，本拍只推进一步恢复，不读取坐标 */
    if (tp_recovery.phase != TP_RECOVER_IDLE)
    {
        /* 推进复位/引导/探测/配置状态机的一步 */
        tp_recover_step();
        /* 返回读取不可用，调用方不能据此认定已松手 */
        return TOUCH_READ_ERROR;
    }
    /* 先读手指数寄存器，判断当前是否有触点 */
    if (tp_read_regs(REG_FingerNum, &num, 1) != 0)
    {
        /* 读失败累计一次，达到阈值后启动异步恢复 */
        tp_read_failed();
        /* 返回读取不可用 */
        return TOUCH_READ_ERROR;
    }

    /* 0xFF 不是有效手指数，按扫描异常处理 */
    if (num == 0xFF)
    {
        /* 同样累计失败，持续异常会触发恢复 */
        tp_read_failed();
        return TOUCH_READ_ERROR; /* 持续无效扫描同样触发恢复，不能据此确认松手。 */
    }
    /* 手指数为 0 表示当前没有手指按下 */
    if (num == 0x00)
    {
        /* 读到有效无触点结果，清空连续失败计数 */
        s_tp_fail_streak = 0;
        return 0; /* 无手指 */
    }

    /* 有触点时连读 4 字节坐标：XH XL YH YL */
    if (tp_read_regs(REG_XposH, d, 4) != 0)
    {
        tp_read_failed(); /* 坐标失败同样累计；后续轮询推进复位，不阻塞LVGL。 */
        return TOUCH_READ_ERROR; /* XposH..YposL 连读 */
    }
    /* 坐标读取成功，说明通信正常，清空连续失败计数 */
    s_tp_fail_streak = 0;
    /* X 为 12bit：高字节低 4 位是高位，拼上低字节构成完整坐标 */
    px = (uint16_t)(((uint16_t)(d[0] & 0x0F) << 8) | d[1]);
    /* Y 同样按 12bit 拆装：d[2] 的低 4 位为高 4 位，d[3] 为低 8 位 */
    py = (uint16_t)(((uint16_t)(d[2] & 0x0F) << 8) | d[3]);

#if TOUCH_REVERSE
    /* 面板与 LCD 装配相差 180°，X 轴按屏宽做镜像 */
    px = TOUCH_SCREEN_W - 1 - px;
    /* Y 轴按屏高做镜像，使触摸点与显示坐标一致 */
    py = TOUCH_SCREEN_H - 1 - py;
#endif
    /* 把换算后的屏幕 X 坐标写回调用方缓冲区 */
    *x = px;
    /* 把换算后的屏幕 Y 坐标写回调用方缓冲区 */
    *y = py;
    /* 返回手指数，非 0 表示当前有触点 */
    return num;
}

uint8_t Touch_Sleep(void)
{
    /* 熄屏后由touch任务进入睡眠；无INT时唤醒只能靠复位 */
    /* 向0xE5睡眠控制寄存器写0x03；返回0表示写事务获ACK，1表示失败。 */
    return tp_write_reg(REG_SLEEP_CTRL, 0x03);
}

void Touch_Wakeup(void)
{
    /* 唤醒前先恢复总线，清掉睡眠期间可能出现的线路占用 */
    (void)my_i2c_recover(MY_I2C_BUS_TP);
    /* 硬复位芯片，退出睡眠并重新引导 */
    Touch_Reset();
    /* 每个候选地址只探测一次，尽量缩短唤醒耗时 */
    (void)tp_probe(1);                           /* 两个候选地址各探测一次 */
    /* 复位后重新关闭自动休眠；写失败时由内部状态机继续异步恢复 */
    (void)tp_configure(); /* 复位后配置丢失；写失败时继续异步恢复。 */
}
