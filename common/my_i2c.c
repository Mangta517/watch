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
 * 驱动方式 (与 interface/touch.c 实测可用的 51 风格一致, 取代最初"开漏+内部
 * 上拉"方案 —— 内部上拉 ~40k 的上升沿 RC 在 200kHz 节拍下不满足建立时间,
 * 外部无可靠上拉时总线直接失效):
 *   SCL: 推挽输出, 全程持有;
 *   SDA: 发送相 = 推挽输出, 读 ACK/数据相 = 输入+上拉, 逐相切换方向。
 * 半周期延时 ~3µs → 总线约 100~150kHz, 低于 MPU6050/CST816 的 400kHz 上限,
 * 给线路寄生电容留足上升余量。
 ******************************************************************************
 */
#include "my_i2c.h"

/* 位半周期延时: 与 touch.c 的 tp_delay(n=80) 同档, ~3µs @100MHz */
#define MY_I2C_DELAY_LOOPS   80

/* ================================================================ 总线表 */
typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
} my_i2c_pin_t;

static const my_i2c_pin_t my_i2c_pin[MY_I2C_BUS_COUNT] =
{
    /* SENSOR 总线: MPU6050 等 */
    { SENSOR_SCL_GPIO_Port, SENSOR_SCL_Pin, SENSOR_SDA_GPIO_Port, SENSOR_SDA_Pin },
    /* TP 总线: 触摸屏 (touch.c 用自己的位翻实现, 本表项为其保留) */
    { TP_SCL_GPIO_Port,     TP_SCL_Pin,     TP_SDA_GPIO_Port,     TP_SDA_Pin     },
    { EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin },
};

/* ================================================================ 私有工具 */
/**
 * @brief 简单忙等待延时，控制 SCL 翻转速度
 */
static void my_i2c_delay(void)
{
    volatile uint32_t n = MY_I2C_DELAY_LOOPS;
    while (n--)
    {
        __NOP();
    }
}

static inline void scl_hi(my_i2c_bus_t bus)
{
    HAL_GPIO_WritePin(my_i2c_pin[bus].scl_port, my_i2c_pin[bus].scl_pin, GPIO_PIN_SET);
}

static inline void scl_lo(my_i2c_bus_t bus)
{
    HAL_GPIO_WritePin(my_i2c_pin[bus].scl_port, my_i2c_pin[bus].scl_pin, GPIO_PIN_RESET);
}

static inline void sda_hi(my_i2c_bus_t bus)
{
    HAL_GPIO_WritePin(my_i2c_pin[bus].sda_port, my_i2c_pin[bus].sda_pin, GPIO_PIN_SET);
}

static inline void sda_lo(my_i2c_bus_t bus)
{
    HAL_GPIO_WritePin(my_i2c_pin[bus].sda_port, my_i2c_pin[bus].sda_pin, GPIO_PIN_RESET);
}

static inline uint8_t sda_read(my_i2c_bus_t bus)
{
    return (HAL_GPIO_ReadPin(my_i2c_pin[bus].sda_port, my_i2c_pin[bus].sda_pin)
            == GPIO_PIN_SET) ? 1 : 0;
}

/**
 * @brief 切换SDA方向：0=输出(EEPROM开漏，其余推挽)，1=输入+上拉(读ACK/数据)
 * @note  HAL_GPIO_Init 对 GPIOB 的 MODER/PUPDR 是非原子读-改-写; 本工程的
 *        touch.c 每个字节也会切 PB4 方向, sensor 任务会切 PB13 方向 ——
 *        两边互相可能把对方刚写入的位用旧快照覆盖。故整个切换用 PRIMASK
 *        临界区包住 (保存-恢复式, 任务/ISR 上下文均安全)。
 */
static void my_i2c_sda_dir(my_i2c_bus_t bus, uint8_t as_input)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t primask = __get_PRIMASK();

    gpio.Pin   = my_i2c_pin[bus].sda_pin;
    gpio.Mode  = as_input ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP;
    if (!as_input && bus == MY_I2C_BUS_EEPROM)
    {
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
    }
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    __disable_irq();
    HAL_GPIO_Init(my_i2c_pin[bus].sda_port, &gpio);
    __set_PRIMASK(primask);
}

/* ================================================================ 时序原语 */
/**
 * @brief 初始化软件I2C：EEPROM开漏，其余总线推挽，空闲释放为高
 * @param bus 要初始化的总线
 * @note  需在 HAL_Init()/时钟配置完成后调用; 会覆盖 MX_GPIO_Init 中对应引脚
 *        的推挽初始配置。重复调用无害(幂等)。
 */
void my_i2c_init(my_i2c_bus_t bus)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t primask = __get_PRIMASK();

    if (bus >= MY_I2C_BUS_COUNT)
    {
        return;
    }

    if (bus == MY_I2C_BUS_EEPROM)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }

    /* SCL: 推挽输出, 初始高 (SDA 方向随后由 sda_dir 逐相切换) */
    __disable_irq();
    gpio.Pin   = my_i2c_pin[bus].scl_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    if (bus == MY_I2C_BUS_EEPROM)
    {
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
    }
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(my_i2c_pin[bus].scl_port, &gpio);
    __set_PRIMASK(primask);

    scl_hi(bus);
    my_i2c_sda_dir(bus, 0);   /* SDA 配为输出 */
    sda_hi(bus);               /* 释放总线, 空闲高 */
    my_i2c_delay();
}

/**
 * @brief 起始信号：SCL 为高期间 SDA 产生下降沿
 */
void my_i2c_start(my_i2c_bus_t bus)
{
    my_i2c_sda_dir(bus, 0);
    sda_hi(bus);
    scl_hi(bus);
    my_i2c_delay();
    sda_lo(bus);     /* SCL 高电平期间 SDA 拉低 = START */
    my_i2c_delay();
    scl_lo(bus);
    my_i2c_delay();
}

/**
 * @brief 停止信号：SCL 为高期间 SDA 产生上升沿
 */
void my_i2c_stop(my_i2c_bus_t bus)
{
    my_i2c_sda_dir(bus, 0);
    sda_lo(bus);
    scl_hi(bus);
    my_i2c_delay();
    sda_hi(bus);     /* SCL 高电平期间 SDA 释放 = STOP */
    my_i2c_delay();
}

/**
 * @brief 写一个字节（MSB 在前）
 */
void my_i2c_write_byte(my_i2c_bus_t bus, uint8_t dat)
{
    uint8_t i;

    my_i2c_sda_dir(bus, 0);                   /* 发送相: SDA 输出 */
    for (i = 0; i < 8; i++)
    {
        scl_lo(bus);                           /* SCL 低电平期间换数据 */
        if (dat & 0x80)
        {
            sda_hi(bus);
        }
        else
        {
            sda_lo(bus);
        }
        my_i2c_delay();                        /* 数据建立 */
        scl_hi(bus);                           /* SCL 上升沿从机采样 */
        my_i2c_delay();
        scl_lo(bus);                           /* 下降沿后再移位 */
        dat <<= 1;
    }
}

/**
 * @brief 读一个字节（MSB 在前），随后由主机发送应答
 * @param ack 0=发送 ACK（继续读），1=发送 NACK（结束读）
 */
uint8_t my_i2c_read_byte(my_i2c_bus_t bus, uint8_t ack)
{
    uint8_t i;
    uint8_t dat = 0;

    my_i2c_sda_dir(bus, 1);                   /* 接收相: SDA 输入, 交给从机 */
    for (i = 0; i < 8; i++)
    {
        scl_lo(bus);
        my_i2c_delay();
        scl_hi(bus);                           /* SCL 上升沿从机更新数据 */
        my_i2c_delay();                        /* 等线路稳定后再采样 (关键!) */
        dat <<= 1;
        if (sda_read(bus))
        {
            dat |= 0x01;
        }
    }
    scl_lo(bus);

    my_i2c_send_ack(bus, ack);                 /* 主机产生第 9 个时钟 + ACK/NACK */
    return dat;
}

/**
 * @brief 等待从机 ACK（第 9 个时钟）
 * @return 0=收到 ACK，1=NACK（从机无应答）
 */
uint8_t my_i2c_wait_ack(my_i2c_bus_t bus)
{
    uint8_t ack;

    my_i2c_sda_dir(bus, 1);                   /* 释放 SDA 让从机拉低 */
    scl_lo(bus);
    my_i2c_delay();
    scl_hi(bus);
    my_i2c_delay();                            /* 等从机驱动建立 */
    ack = sda_read(bus);                       /* 从机拉低 = ACK */
    scl_lo(bus);
    my_i2c_delay();
    my_i2c_sda_dir(bus, 0);                   /* 恢复输出相 */
    sda_hi(bus);

    return ack;                                /* 0=ACK, 1=NACK */
}

/**
 * @brief 主机发送应答位（第 9 个时钟）
 * @param ack 0=ACK（拉低 SDA），1=NACK（释放 SDA）
 */
void my_i2c_send_ack(my_i2c_bus_t bus, uint8_t ack)
{
    my_i2c_sda_dir(bus, 0);
    scl_lo(bus);
    if (ack)
    {
        sda_hi(bus);                           /* NACK */
    }
    else
    {
        sda_lo(bus);                           /* ACK  */
    }
    my_i2c_delay();
    scl_hi(bus);
    my_i2c_delay();
    scl_lo(bus);
    my_i2c_sda_dir(bus, 1);                   /* 数据阶段前释放 SDA (输入+上拉=高) */
    my_i2c_delay();
}

/* ============================================================ 成块读写封装 */
uint8_t my_i2c_probe(my_i2c_bus_t bus, uint8_t dev_addr)
{
    uint8_t result;
    if (bus >= MY_I2C_BUS_COUNT || dev_addr > 0x7F)
    {
        return 1;
    }
    my_i2c_start(bus);
    my_i2c_write_byte(bus, (uint8_t)(dev_addr << 1));
    result = my_i2c_wait_ack(bus);
    my_i2c_stop(bus);
    return result;
}

/**
 * @brief 连续写多字节：START -> 器件地址(W) -> 寄存器地址 -> 数据... -> STOP
 * @param dev_addr 7 位器件地址（如 MPU6050 为 0x68）
 * @return 0=成功，1=NACK/参数错误（已自动发 STOP）
 */
uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr,
                           const uint8_t *dat, uint16_t len)
{
    uint16_t i;

    if (bus >= MY_I2C_BUS_COUNT || (dat == NULL && len > 0))
    {
        return 1;
    }

    my_i2c_start(bus);
    my_i2c_write_byte(bus, (uint8_t)(dev_addr << 1));        /* 地址 + 写位 */
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    my_i2c_write_byte(bus, reg_addr);
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    for (i = 0; i < len; i++)
    {
        my_i2c_write_byte(bus, dat[i]);
        if (my_i2c_wait_ack(bus))
        {
            my_i2c_stop(bus);
            return 1;
        }
    }

    my_i2c_stop(bus);
    return 0;
}

/**
 * @brief 连续读多字节：START -> 器件地址(W) -> 寄存器地址 -> 重复START ->
 *        器件地址(R) -> 读数据... -> STOP
 * @param dev_addr 7 位器件地址（如 MPU6050 为 0x68）
 * @return 0=成功，1=NACK/参数错误（已自动发 STOP）
 */
uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg_addr,
                          uint8_t *dat, uint16_t len)
{
    uint16_t i;

    if (bus >= MY_I2C_BUS_COUNT || dat == NULL || len == 0)
    {
        return 1;
    }

    /* 第一阶段：写入要读的寄存器地址 */
    my_i2c_start(bus);
    my_i2c_write_byte(bus, (uint8_t)(dev_addr << 1));
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    my_i2c_write_byte(bus, reg_addr);
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    /* 第二阶段：重复起始 + 读方向 */
    my_i2c_start(bus);
    my_i2c_write_byte(bus, (uint8_t)((dev_addr << 1) | 0x01));
    if (my_i2c_wait_ack(bus))
    {
        my_i2c_stop(bus);
        return 1;
    }

    for (i = 0; i < len; i++)
    {
        /* 最后一个字节回 NACK，其余回 ACK */
        dat[i] = my_i2c_read_byte(bus, (i == len - 1) ? 1 : 0);
    }

    my_i2c_stop(bus);
    return 0;
}
