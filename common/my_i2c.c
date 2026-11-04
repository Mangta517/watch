/**
 ******************************************************************************
 * @file    my_i2c.c
 * @brief   软件 I2C（GPIO 模拟）驱动 —— STM32F411CEU6，三条独立总线
 *
 * 总线分配：
 *   MY_I2C_BUS_SENSOR : SCL = PB14 (SENSOR_SCL)  SDA = PB13 (SENSOR_SDA)
 *   MY_I2C_BUS_TP     : SCL = PB6  (TP_SCL)      SDA = PB4  (TP_SDA)
 *   MY_I2C_BUS_EEPROM : SCL = PA12 (EEPROM_SCL) SDA = PA11 (EEPROM_SDA)，开漏+外部上拉
 *
 * SENSOR/EEPROM 使用开漏输出，必须有外部上拉到 3.3V；内部上拉不能替代外部上拉。
 * TP 保留原推挽发送实现；SDA 读 ACK/数据和发送 NACK 时切换为输入释放。
 * SENSOR 总线将原位延时加倍，增加从机准备数据及线路稳定的余量；实际频率需实测。
 * TP/EEPROM 位延时保持原值；ACK拉低前预置输出，NACK以输入释放SDA。
 ******************************************************************************
 */
#include "my_i2c.h"

/* APP按64MHz缩放位延时，SENSOR仍加倍；Bootloader保留100MHz原值。 */
#if defined(F411_BOOTLOADER)
#define MY_I2C_DELAY_LOOPS        67
#define MY_I2C_SENSOR_DELAY_LOOPS 134
#else
#define MY_I2C_DELAY_LOOPS        51
#define MY_I2C_SENSOR_DELAY_LOOPS 102
#endif

/* ================================================================ 总线表 */
typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
} my_i2c_pin_t;

static const my_i2c_pin_t my_i2c_pin[MY_I2C_BUS_COUNT] = {
    /* SENSOR 总线: MPU6050 等 */
    {SENSOR_SCL_GPIO_Port, SENSOR_SCL_Pin, SENSOR_SDA_GPIO_Port, SENSOR_SDA_Pin},
    /* TP 总线: 触摸屏 (touch.c 用自己的位翻实现, 本表项为其保留) */
    {TP_SCL_GPIO_Port, TP_SCL_Pin, TP_SDA_GPIO_Port, TP_SDA_Pin},
    {EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin},
};
/* 各总线由原owner串行操作；0未知、1输出、2输入，仅省略同方向的重复配置。 */
static uint8_t s_sda_direction[MY_I2C_BUS_COUNT];

/* ================================================================ 私有工具 */
/**
 * @brief 简单忙等待延时，控制 SCL 翻转速度
 */
static void my_i2c_delay(my_i2c_bus_t bus)
{
    /* n：本总线的空循环次数；volatile 防止编译器把空循环整体优化掉 */
    volatile uint32_t n;
    /* 按总线选择延时：SENSOR 的循环次数在文件开头已定义为加倍值，其余总线用基础值 */
    if (bus == MY_I2C_BUS_SENSOR)
    {
        n = MY_I2C_SENSOR_DELAY_LOOPS;
    }
    else
    {
        n = MY_I2C_DELAY_LOOPS;
    }
    /* 逐次递减并空转，每圈执行一条 NOP，用于拉开 SCL/SDA 的边沿间隔 */
    while (n--)
    {
        __NOP();
    }
}

static inline void scl_hi(my_i2c_bus_t bus)
{
    /* 把该总线 SCL 引脚置高：开漏时是释放、由上拉拉高，推挽时直接驱动高 */
    HAL_GPIO_WritePin(my_i2c_pin[bus].scl_port, my_i2c_pin[bus].scl_pin, GPIO_PIN_SET);
}

static inline void scl_lo(my_i2c_bus_t bus)
{
    /* 把该总线 SCL 引脚置低：开漏输出为主动下拉，推挽输出为直接驱动低电平 */
    HAL_GPIO_WritePin(my_i2c_pin[bus].scl_port, my_i2c_pin[bus].scl_pin, GPIO_PIN_RESET);
}

static inline void sda_hi(my_i2c_bus_t bus)
{
    /* 把该总线 SDA 引脚置高：开漏时是释放总线，推挽时是主动输出高电平 */
    HAL_GPIO_WritePin(my_i2c_pin[bus].sda_port, my_i2c_pin[bus].sda_pin, GPIO_PIN_SET);
}

static inline void sda_lo(my_i2c_bus_t bus)
{
    /* 把该总线 SDA 引脚置低：主动拉低数据线，用于发送 0 或产生边沿 */
    HAL_GPIO_WritePin(my_i2c_pin[bus].sda_port, my_i2c_pin[bus].sda_pin, GPIO_PIN_RESET);
}

static inline uint8_t sda_read(my_i2c_bus_t bus)
{
    /* 直接读引脚电平而不是输出寄存器，得到从机或上拉电阻实际建立的电平 */
    if (HAL_GPIO_ReadPin(my_i2c_pin[bus].sda_port, my_i2c_pin[bus].sda_pin) == GPIO_PIN_SET)
    {
        /* 高电平归一化为 1，调用方用它判断 ACK 释放或读回的数据位 */
        return 1;
    }
    else
    {
        /* 低电平归一化为 0，调用方用它判断 ACK 拉低或读回的数据位 */
        return 0;
    }
}

/**
 * @brief 切换SDA方向：0=输出(SENSOR/EEPROM开漏，TP推挽)，1=输入+上拉(读ACK/数据)
 * @note  HAL_GPIO_Init 对 GPIOB 的 MODER/PUPDR 是非原子读-改-写; 本工程的
 *        touch.c 每个字节也会切 PB4 方向, sensor 任务会切 PB13 方向 ——
 *        两边互相可能把对方刚写入的位用旧快照覆盖。故整个切换用 PRIMASK
 *        临界区包住 (保存-恢复式, 任务/ISR 上下文均安全)。
 */
static void my_i2c_sda_dir(my_i2c_bus_t bus, uint8_t as_input)
{
    /* gpio：本次要写入 SDA 引脚的初始化参数，先清零保证未用字段为默认值 */
    GPIO_InitTypeDef gpio = {0};
    /* primask：进入临界区前的中断屏蔽状态，退出时原样恢复 */
    uint32_t primask = __get_PRIMASK();
    /* direction：把布尔参数编码成缓存值，1=输出、2=输入、0 留给“未知” */
    uint8_t direction = as_input + 1;
    /* 方向与上次相同时直接返回，省掉一次 HAL_GPIO_Init 的读-改-写 */
    if (s_sda_direction[bus] == direction)
    {
        return;
    }

    /* 只配置本总线的 SDA 引脚，SCL 的配置不受影响 */
    gpio.Pin = my_i2c_pin[bus].sda_pin;
    /* 输入模式用于读 ACK/数据：引脚由上拉和从机共同决定电平 */
    if (as_input != 0)
    {
        gpio.Mode = GPIO_MODE_INPUT;
    }
    else
    {
        /* 先按推挽输出准备，TP 总线最终保持推挽发送 */
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
    }
    /* SENSOR/EEPROM 发送时改用开漏输出，避免与从机拉低产生对顶电流 */
    if (!as_input && bus != MY_I2C_BUS_TP)
    {
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
    }
    /* 输入态靠上拉把空闲线保持为高；输出态该字段仍按上拉填写 */
    gpio.Pull = GPIO_PULLUP;
    /* 高速档缩短引脚翻转时间，配合位延时满足时序 */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 进入临界区：把 HAL_GPIO_Init 的读-改-写与其它任务/ISR 的引脚操作隔开 */
    __disable_irq();
    HAL_GPIO_Init(my_i2c_pin[bus].sda_port, &gpio);
    /* 记录本次方向，供下次同方向调用时跳过重复配置 */
    s_sda_direction[bus] = direction;
    /* 恢复进入前的中断屏蔽状态，而不是无条件开中断 */
    __set_PRIMASK(primask);
}

/* 固定次数等待，SCL持续被拉低时不依赖tick/中断，也不冒充有效恢复时钟。 */
static uint8_t my_i2c_scl_released(my_i2c_bus_t bus)
{
    /* guard：最多查询的次数，用来给“SCL 是否被释放”一个固定上限 */
    uint8_t guard;
    /* 每次先读电平，未释放就等一个位延时再重试，最多 16 次 */
    for (guard = 0; guard < 16; ++guard)
    {
        /* SCL 读到高电平说明线已释放，立即返回 1 */
        if (HAL_GPIO_ReadPin(my_i2c_pin[bus].scl_port, my_i2c_pin[bus].scl_pin) == GPIO_PIN_SET)
        {
            return 1;
        }
        /* 从机仍在拉低 SCL，等一个位时间后再次查询 */
        my_i2c_delay(bus);
    }
    /* 16 次都没读到高电平，返回 0 表示 SCL 仍被占用 */
    return 0;
}

uint8_t my_i2c_recover(my_i2c_bus_t bus)
{
    /* i：恢复时钟脉冲的计数，最多补 9 个时钟尝试让卡住的从机推进位计数 */
    uint8_t i;
    /* result：默认 1 表示恢复未确认成功，只有最后 SDA 读到高才改成 0 */
    uint8_t result = 1;
    /* 总线编号越界时直接返回失败，避免数组越界访问 */
    if (bus >= MY_I2C_BUS_COUNT)
    {
        return 1;
    }
    /* touch.c自行配置PB4；不能用上次公共调用的缓存跳过本次释放。 */
    if (bus == MY_I2C_BUS_TP)
    {
        /* 清零缓存，强制下面的 sda_dir 真正重配一次 TP 的 SDA 引脚 */
        s_sda_direction[bus] = 0;
    }
    /* 先把 SDA 切成输入，恢复期间不主动驱动数据线 */
    my_i2c_sda_dir(bus, 1);
    /* 预置SDA输出锁存值为高；当前输入态由已配置的上拉释放线路。 */
    sda_hi(bus);
    /* 逐个补时钟：拉低 SCL 再释放，让卡住的从机有机会推进它的位计数 */
    for (i = 0; i < 9; ++i)
    {
        /* SCL 拉低，形成一个时钟的低相 */
        scl_lo(bus);
        /* 保持低电平一个位时间 */
        my_i2c_delay(bus);
        /* 释放 SCL，形成上升沿 */
        scl_hi(bus);
        /* 若 SCL 一直拉不高，说明线路仍被占用，放弃后续时钟直接收尾 */
        if (my_i2c_scl_released(bus) == 0)
        {
            goto release;
        }
        /* 高相保持一个位时间，再进入下一个时钟 */
        my_i2c_delay(bus);
    }
    /* STOP：先预置低电平再切输出；结束用输入释放SDA，不与卡低的从机争用。 */
    scl_lo(bus);
    sda_lo(bus);
    /* 切回输出，把刚预置的低电平真正驱动到线上 */
    my_i2c_sda_dir(bus, 0);
    /* 等 SDA 低电平建立后再抬高 SCL */
    my_i2c_delay(bus);
    scl_hi(bus);
    /* SCL 无法释放时同样跳到收尾，不继续制造 STOP 边沿 */
    if (my_i2c_scl_released(bus) == 0)
    {
        goto release;
    }
    /* SCL 高电平期间保持，准备产生 STOP 的上升沿 */
    my_i2c_delay(bus);
    /* 释放 SDA：切回输入后由上拉拉高，形成 STOP 条件的上升沿 */
    my_i2c_sda_dir(bus, 1);
    sda_hi(bus);
    /* 等 STOP 条件在总线上稳定 */
    my_i2c_delay(bus);
    /* 只有 SDA 确实回到高电平，才认为总线已释放并标记成功 */
    if (sda_read(bus) != 0)
    {
        result = 0;
    }
release:
    /* 无论成功失败都将SDA切回输入，主机不再主动拉低 */
    my_i2c_sda_dir(bus, 1);
    sda_hi(bus);
    /* 请求释放SCL；实际线路是否已恢复由本次result表示 */
    scl_hi(bus);
    /* TP 的引脚会被 touch.c 直接改写，清缓存以便下次重新配置 */
    if (bus == MY_I2C_BUS_TP)
    {
        s_sda_direction[bus] = 0;
    }
    /* 0=恢复后 SDA 已释放为高，1=参数错误或线路仍异常 */
    return result;
}

uint8_t my_i2c_prepare(my_i2c_bus_t bus)
{
    /* 总线编号越界时无法操作引脚，直接返回失败 */
    if (bus >= MY_I2C_BUS_COUNT)
    {
        return 1;
    }
    /* TP 引脚被 touch.c 直接配置过，清缓存保证下面真正重新初始化 */
    if (bus == MY_I2C_BUS_TP)
    {
        s_sda_direction[bus] = 0;
    }
    /* 先释放 SDA（输入+上拉），避免主机在检查空闲前驱动数据线 */
    my_i2c_sda_dir(bus, 1);
    sda_hi(bus);
    /* 释放 SCL，使总线进入空闲高电平 */
    scl_hi(bus);
    /* 等两个引脚的电平建立 */
    my_i2c_delay(bus);
    /* SCL 与 SDA 都为高说明总线空闲，可以开始一次传输 */
    if (my_i2c_scl_released(bus) != 0 && sda_read(bus) != 0)
    {
        return 0;
    }
    /* 总线不空闲时尝试补时钟恢复，返回值直接作为本次准备结果 */
    return my_i2c_recover(bus);
}

/* ================================================================ 时序原语 */
/* 将SCL/SDA切到无上下拉模拟态，释放GPIO驱动。 */
void my_i2c_suspend(my_i2c_bus_t bus)
{
    /* gpio：配置 SCL/SDA 为模拟输入用的参数，两个引脚共用同一套字段 */
    GPIO_InitTypeDef gpio = {0};
    /* primask：修改引脚前的中断屏蔽状态，配置完成后恢复 */
    uint32_t primask;
    /* 总线编号越界时没有对应引脚可配置，直接返回 */
    if (bus >= MY_I2C_BUS_COUNT)
    {
        return;
    }
    /* 进入模拟态前先把两根线写成高，避免切换瞬间把总线拉低 */
    scl_hi(bus);
    sda_hi(bus);
    /* 模拟输入关闭数字输入缓冲，也停止 GPIO 对引脚的驱动 */
    gpio.Mode = GPIO_MODE_ANALOG;
    /* 模拟态不需要内部上/下拉，外部上拉仍可把线保持在空闲高 */
    gpio.Pull = GPIO_NOPULL;
    /* 保存当前中断状态，准备进入短临界区 */
    primask = __get_PRIMASK();
    __disable_irq();
    /* 先配置 SCL 引脚 */
    gpio.Pin = my_i2c_pin[bus].scl_pin;
    HAL_GPIO_Init(my_i2c_pin[bus].scl_port, &gpio);
    /* 再配置 SDA 引脚，两个引脚都进入高阻的模拟态 */
    gpio.Pin = my_i2c_pin[bus].sda_pin;
    HAL_GPIO_Init(my_i2c_pin[bus].sda_port, &gpio);
    /* 引脚已不是输出/输入，缓存方向标记为未知，下次切换会重新配置 */
    s_sda_direction[bus] = 0;
    /* 恢复进入临界区前的中断屏蔽状态 */
    __set_PRIMASK(primask);
}

/**
 * @brief 初始化软件I2C：SENSOR/EEPROM开漏，TP推挽，空闲释放为高
 * @param bus 要初始化的总线
 * @note  需在 HAL_Init()/时钟配置完成后调用; 会覆盖 MX_GPIO_Init 中对应引脚
 *        的推挽初始配置。重复调用无害(幂等)。
 */
void my_i2c_init(my_i2c_bus_t bus)
{
    /* gpio：SCL 引脚的初始化参数，SDA 随后由 sda_dir 单独配置 */
    GPIO_InitTypeDef gpio = {0};
    /* primask：配置 SCL 期间保存的中断屏蔽状态，配置后恢复 */
    uint32_t primask = __get_PRIMASK();

    /* 总线编号越界时没有对应引脚可初始化，直接返回 */
    if (bus >= MY_I2C_BUS_COUNT)
    {
        return;
    }

    /* EEPROM 总线在 GPIOA，打开对应端口时钟才能写寄存器 */
    if (bus == MY_I2C_BUS_EEPROM)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else
    {
        /* SENSOR 与 TP 总线都在 GPIOB，打开 GPIOB 时钟 */
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }

    /* SCL: SENSOR/EEPROM 开漏；SDA 方向随后由 sda_dir 逐相切换。 */
    __disable_irq();
    gpio.Pin = my_i2c_pin[bus].scl_pin;
    /* 默认先按推挽输出，TP 总线保持这种模式 */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    /* SENSOR/EEPROM 的 SCL 改为开漏，高电平靠外部上拉 */
    if (bus != MY_I2C_BUS_TP)
    {
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
    }
    /* 使能内部上拉；开漏输出时仍需外部上拉才能得到可靠高电平 */
    gpio.Pull = GPIO_PULLUP;
    /* 高速档让边沿更快，实际速率仍由位延时决定 */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    /* 在临界区内写 SCL 引脚配置，避免与其它任务/ISR 的引脚操作交错 */
    HAL_GPIO_Init(my_i2c_pin[bus].scl_port, &gpio);
    /* 恢复中断屏蔽状态 */
    __set_PRIMASK(primask);

    /* 初始化完成后先把 SCL 释放为高 */
    scl_hi(bus);
    s_sda_direction[bus] = 0; /* 重新初始化时强制恢复SDA硬件配置。 */
    my_i2c_sda_dir(bus, 0);   /* SDA 配为输出 */
    sda_hi(bus);              /* 释放总线, 空闲高 */
    /* 等空闲电平建立后再返回 */
    my_i2c_delay(bus);
}

/**
 * @brief 起始信号：SCL 为高期间 SDA 产生下降沿
 */
void my_i2c_start(my_i2c_bus_t bus)
{
    /* 先把 SDA 切成输出，下面才能主动制造下降沿 */
    my_i2c_sda_dir(bus, 0);
    /* 输出高电平，保证 START 之前 SDA 处于释放状态 */
    sda_hi(bus);
    /* SCL 拉高，进入 START 要求的“SCL 高”前提 */
    scl_hi(bus);
    /* 等 SCL 与 SDA 的高电平稳定 */
    my_i2c_delay(bus);
    sda_lo(bus); /* SCL 高电平期间 SDA 拉低 = START */
    /* 保持 SDA 低电平，让从机识别到起始条件 */
    my_i2c_delay(bus);
    /* 拉低 SCL，为后续逐位发送准备低相 */
    scl_lo(bus);
    /* 等 SCL 低电平建立后返回 */
    my_i2c_delay(bus);
}

/**
 * @brief 停止信号：SCL 为高期间 SDA 产生上升沿
 */
void my_i2c_stop(my_i2c_bus_t bus)
{
    /* 切 SDA 为输出，才能在 STOP 中先拉低再释放 */
    my_i2c_sda_dir(bus, 0);
    /* 先把 SDA 驱动为低电平 */
    sda_lo(bus);
    my_i2c_delay(bus); /* SCL 拉高前建立 SDA 低电平 */
    /* 在 SDA 为低时抬高 SCL */
    scl_hi(bus);
    /* 等 SCL 高电平稳定 */
    my_i2c_delay(bus);
    sda_hi(bus); /* SCL 高电平期间 SDA 释放 = STOP */
    /* 保持该状态，让从机识别到停止条件 */
    my_i2c_delay(bus);
}

/**
 * @brief 写一个字节（MSB 在前）
 */
void my_i2c_write_byte(my_i2c_bus_t bus, uint8_t dat)
{
    /* i：已发送的位数，循环 8 次发完一个字节 */
    uint8_t i;

    my_i2c_sda_dir(bus, 0); /* 发送相: SDA 输出 */
    /* 从最高位开始逐位输出，每圈发送 dat 的当前最高位 */
    for (i = 0; i < 8; i++)
    {
        scl_lo(bus); /* SCL 低电平期间换数据 */
        /* 最高位为 1 时释放/驱动 SDA 为高 */
        if (dat & 0x80)
        {
            sda_hi(bus);
        }
        else
        {
            /* 最高位为 0 时把 SDA 拉低 */
            sda_lo(bus);
        }
        my_i2c_delay(bus); /* 数据建立 */
        scl_hi(bus);       /* SCL 上升沿从机采样 */
        /* SCL 保持高电平一个位时间，让从机完成采样 */
        my_i2c_delay(bus);
        scl_lo(bus); /* 下降沿后再移位 */
        /* 左移一位，让下一位成为新的最高位 */
        dat <<= 1;
    }
}

/**
 * @brief 读一个字节（MSB 在前），随后由主机发送应答
 * @param ack 0=发送 ACK（继续读），1=发送 NACK（结束读）
 */
uint8_t my_i2c_read_byte(my_i2c_bus_t bus, uint8_t ack)
{
    /* i：已读取的位数，循环 8 次凑满一个字节 */
    uint8_t i;
    /* dat：接收缓冲区，每读一位左移后填入最低位，初始为 0 */
    uint8_t dat = 0;

    my_i2c_sda_dir(bus, 1); /* 接收相: SDA 输入, 交给从机 */
    /* 逐位读取，先读到的是字节的最高位 */
    for (i = 0; i < 8; i++)
    {
        /* 拉低 SCL，通知从机准备下一位数据 */
        scl_lo(bus);
        /* 等 SCL 低电平建立 */
        my_i2c_delay(bus);
        scl_hi(bus);       /* 从机在 SCL 低期间准备数据，高期间保持稳定 */
        my_i2c_delay(bus); /* 等线路稳定后再采样 (关键!) */
        /* 左移一位，为本次采样腾出最低位 */
        dat <<= 1;
        /* SDA 为高则把这一位记作 1；为低时保持刚移入的 0 */
        if (sda_read(bus))
        {
            dat |= 0x01;
        }
    }
    /* 8 位读完，拉低 SCL 为第 9 个应答时钟做准备 */
    scl_lo(bus);

    my_i2c_send_ack(bus, ack); /* 主机产生第 9 个时钟 + ACK/NACK */
    /* 返回从机发来的完整字节 */
    return dat;
}

/**
 * @brief 等待从机 ACK（第 9 个时钟）
 * @return 0=收到 ACK，1=NACK（从机无应答）
 */
uint8_t my_i2c_wait_ack(my_i2c_bus_t bus)
{
    /* ack：第 9 个时钟采样到的 SDA 电平，0=从机拉低应答，1=未应答 */
    uint8_t ack;

    my_i2c_sda_dir(bus, 1); /* 释放 SDA 让从机拉低 */
    /* 拉低 SCL，开始第 9 个时钟的低相 */
    scl_lo(bus);
    /* 等低电平建立，给从机留出拉低 SDA 的时间 */
    my_i2c_delay(bus);
    /* 抬高 SCL，进入应答位的采样窗口 */
    scl_hi(bus);
    my_i2c_delay(bus);   /* 等从机驱动建立 */
    ack = sda_read(bus); /* 从机拉低 = ACK */
    /* 采样结束，拉低 SCL 完成第 9 个时钟 */
    scl_lo(bus);
    /* 等 SCL 低电平稳定后返回 */
    my_i2c_delay(bus);
    /* 保持输入释放 SDA；读地址 ACK 后从机已开始驱动第一位。
     * 下一次 write/start/stop 自己切回输出，不能在这里推挽拉高。 */

    return ack; /* 0=ACK, 1=NACK */
}

/**
 * @brief 主机发送应答位（第 9 个时钟）
 * @param ack 0=ACK（拉低 SDA），1=NACK（释放 SDA）
 */
void my_i2c_send_ack(my_i2c_bus_t bus, uint8_t ack)
{
    /* 先拉低 SCL，应答位必须在低电平期间改变 SDA */
    scl_lo(bus);
    /* ack 非 0 表示 NACK：不拉低 SDA，交给上拉保持高电平 */
    if (ack)
    {
        my_i2c_sda_dir(bus, 1); /* NACK释放SDA，由上拉形成高电平。 */
    }
    else
    {
        /* ACK：先让输出寄存器为低，再把引脚切成输出，避免切换瞬间产生高电平 */
        sda_lo(bus); /* 先预置低电平，避免切输出瞬间推挽拉高。 */
        my_i2c_sda_dir(bus, 0);
    }
    /* 等 SDA 在 SCL 低期间建立 */
    my_i2c_delay(bus);
    /* 抬高 SCL，从机在此时钟采样应答位 */
    scl_hi(bus);
    /* 保持高电平一个位时间 */
    my_i2c_delay(bus);
    /* 拉低 SCL，结束第 9 个时钟 */
    scl_lo(bus);
    my_i2c_sda_dir(bus, 1); /* 数据阶段前释放 SDA (输入+上拉=高) */
    /* 释放SDA后再等一个位延时。 */
    my_i2c_delay(bus);
}

/* ============================================================ 成块读写封装 */
uint8_t my_i2c_probe(my_i2c_bus_t bus, uint8_t dev_addr)
{
    /* result：等待从机 ACK 的结果，0=该地址有应答，1=无应答 */
    uint8_t result;
    /* 总线编号越界或地址超过 7 位范围时无法发起探测，返回失败 */
    if (bus >= MY_I2C_BUS_COUNT || dev_addr > 0x7F)
    {
        return 1;
    }
    /* 先确认总线空闲，不空闲且恢复失败则不再尝试通信 */
    if (my_i2c_prepare(bus) != 0)
    {
        return 1;
    }
    /* 产生起始条件 */
    my_i2c_start(bus);
    /* 发送 7 位地址左移一位后的写方向字节，最低位为 0 */
    my_i2c_write_byte(bus, (uint8_t)(dev_addr << 1));
    /* 第 9 个时钟读从机应答，作为探测结果 */
    result = my_i2c_wait_ack(bus);
    /* 无论有无应答都发停止条件，把总线还给空闲态 */
    my_i2c_stop(bus);
    /* 返回 0 表示该地址有从机应答，1 表示无应答 */
    return result;
}

/**
 * @brief 连续写多字节：START -> 器件地址(W) -> 寄存器地址 -> 数据... -> STOP
 * @param dev_addr 7 位器件地址（如 MPU6050 为 0x68）
 * @return 0=完成；1=参数/总线准备失败或NACK；传输阶段遇NACK会发STOP收尾。
 */
uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *dat, uint16_t len)
{
    /* i：已写入的数据字节序号，从 0 递增到 len-1 */
    uint16_t i;

    /* 总线越界，或长度为非 0 却没有数据指针时无法发送，返回参数错误 */
    if (bus >= MY_I2C_BUS_COUNT || (dat == NULL && len > 0))
    {
        return 1;
    }

    /* 总线不空闲且恢复失败时直接返回，不再继续本次传输 */
    if (my_i2c_prepare(bus) != 0)
    {
        return 1;
    }

    /* 产生起始条件，开始一帧写传输 */
    my_i2c_start(bus);
    my_i2c_write_byte(bus, (uint8_t)(dev_addr << 1)); /* 地址 + 写位 */
    /* 器件地址无应答说明没有从机确认该地址，发 STOP 后返回失败 */
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    /* 发送要写入的寄存器地址 */
    my_i2c_write_byte(bus, reg_addr);
    /* 寄存器地址无应答同样收尾返回失败 */
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    /* 逐字节发送数据区，len 为 0 时本循环不执行 */
    for (i = 0; i < len; i++)
    {
        my_i2c_write_byte(bus, dat[i]);
        /* 任一数据字节未被应答即中止，并补发 STOP 释放总线 */
        if (my_i2c_wait_ack(bus))
        {
            my_i2c_stop(bus);
            return 1;
        }
    }

    /* 所有字节都收到 ACK，发停止条件结束本帧 */
    my_i2c_stop(bus);
    /* 0 表示整帧写入过程未遇到 NACK 或参数错误 */
    return 0;
}

/**
 * @brief 连续读多字节：START -> 器件地址(W) -> 寄存器地址 -> 重复START ->
 *        器件地址(R) -> 读数据... -> STOP
 * @param dev_addr 7 位器件地址（如 MPU6050 为 0x68）
 * @return 0=完成；1=参数/总线准备失败或NACK；传输阶段遇NACK会发STOP收尾。
 */
uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *dat, uint16_t len)
{
    /* i：已读取的数据字节序号，从 0 递增到 len-1 */
    uint16_t i;

    /* 总线越界、数据指针为空或长度为 0 时没有可执行的读操作，返回失败 */
    if (bus >= MY_I2C_BUS_COUNT || dat == NULL || len == 0)
    {
        return 1;
    }

    /* 第一阶段：写入要读的寄存器地址 */
    if (my_i2c_prepare(bus) != 0)
    {
        return 1;
    }
    /* 起始条件后先以写方向访问器件 */
    my_i2c_start(bus);
    my_i2c_write_byte(bus, (uint8_t)(dev_addr << 1));
    /* 器件地址无应答则发 STOP 收尾并返回失败 */
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    /* 写入要读取的寄存器地址，为后续重复起始定位数据源 */
    my_i2c_write_byte(bus, reg_addr);
    /* 寄存器地址无应答同样发 STOP 并返回失败 */
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    /* 第二阶段：重复起始 + 读方向 */
    my_i2c_start(bus);
    /* 地址最低位置 1 表示读方向 */
    my_i2c_write_byte(bus, (uint8_t)((dev_addr << 1) | 0x01));
    /* 读方向地址无应答说明器件未接受读请求，收尾返回失败 */
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    /* 逐字节读取，直到填满 len 个字节 */
    for (i = 0; i < len; i++)
    {
        /* 最后一个字节回 NACK，其余回 ACK */
        if (i == len - 1)
        {
            /* 末字节回 NACK，通知从机停止继续发送 */
            dat[i] = my_i2c_read_byte(bus, 1);
        }
        else
        {
            /* 非末字节回 ACK，表示还要继续读下一个字节 */
            dat[i] = my_i2c_read_byte(bus, 0);
        }
    }

    /* 读满后发停止条件释放总线 */
    my_i2c_stop(bus);
    /* 0 表示地址与寄存器地址均被应答且数据已全部读入 */
    return 0;
}
