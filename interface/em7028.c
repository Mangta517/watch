/**
 * @file em7028.c
 * @brief EM7028 手册 V1.0，第 14～21 页；仅实现本板接线对应的 HRS1 模式。
 */
#include "em7028.h"
#include "my_i2c.h"
#include <stddef.h>

#define EM7028_REG_PID       0x00
#define EM7028_REG_CONFIG    0x01
#define EM7028_REG_HRS1_CTRL 0x0D
#define EM7028_REG_INT_CTRL  0x0E
#define EM7028_REG_DATA_LOW  0x28
#define EM7028_REG_DATA_HIGH 0x29
#define EM7028_HRS1_EN       0x08
#define EM7028_MODE_MASK     0x88 /* HRS2_EN(bit7) + HRS1_EN(bit3)，不能同时开启 */
#define EM7028_INT_EN        0x80
#define EM7028_HRS1_CTRL     0x47 /* 同款 OV-Watch 配置：gain=1，range=8，freq=000，res=11，IR_MODE=1 */

static uint8_t em7028_ready;
static uint8_t em7028_running;
static uint32_t em7028_start_tick;
static em7028_fault em7028_last_fault;

static uint8_t em7028_write_reg(uint8_t reg, uint8_t value)
{
    /* 按“寄存器号 + 1 字节数据”写一个寄存器；底层返回非 0 即写失败。 */
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, EM7028_ADDR, reg, &value, 1) != 0)
    {
        /* 记录失败类型和目标寄存器，供上层查询最近一次故障。 */
        em7028_last_fault.kind = EM7028_FAULT_WRITE;
        em7028_last_fault.reg = reg;
        return 1;
    }
    return 0;
}

/* 手册 Fig.13 的随机读在寄存器地址后先 STOP，再 START 读一个字节。
 * 复用现有底层时序，不假设器件支持寄存器自动递增或重复 START。 */
static uint8_t em7028_read_reg(uint8_t reg, uint8_t *value)
{
    /* 先记下目标寄存器号，后续任一失败都能对上是哪个寄存器。 */
    em7028_last_fault.reg = reg;
    /* 准备 SENSOR 总线 GPIO 方向，失败即读故障。 */
    if (my_i2c_prepare(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        return 1;
    }
    /* 第一段事务：START 后写器件地址（写方向），用于发送寄存器号。 */
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, (uint8_t)(EM7028_ADDR << 1));
    /* 等器件应答，不应答则停止总线并返回读失败。 */
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return 1;
    }
    /* 发送要读的寄存器地址。 */
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, reg);
    /* 再次等待应答，失败同样结束事务并返回读失败。 */
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return 1;
    }
    /* 按手册先发 STOP，结束写寄存器地址这一段。 */
    my_i2c_stop(MY_I2C_BUS_SENSOR);

    /* 第二段事务前重新准备总线，失败即读故障。 */
    if (my_i2c_prepare(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        return 1;
    }
    /* 重新 START，并发送读方向地址（最低位置 1）。 */
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, (uint8_t)((EM7028_ADDR << 1) | 1));
    /* 等器件应答读地址，失败则停止并返回读失败。 */
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return 1;
    }
    *value = my_i2c_read_byte(MY_I2C_BUS_SENSOR, 1); /* 最后一个字节发送 NACK */
    /* 读完后发 STOP，释放总线。 */
    my_i2c_stop(MY_I2C_BUS_SENSOR);
    return 0;
}

static uint8_t em7028_set_mode(uint8_t mode)
{
    /* CONFIG 寄存器当前值，用于保留模式位以外的位。 */
    uint8_t config;
    /* 先读回 CONFIG，读失败直接返回。 */
    if (em7028_read_reg(EM7028_REG_CONFIG, &config) != 0)
    {
        return 1;
    }
    /* 主表和逐位表的保留位默认值不一致，按实际读回值保留。 */
    /* 清掉 HRS2/HRS1 两个模式位，再并入请求的模式值。 */
    config = (uint8_t)((config & (uint8_t)~EM7028_MODE_MASK) | mode);
    /* 写回配置并立即回读校验；任一环节失败都返回 1。 */
    if (em7028_write_reg(EM7028_REG_CONFIG, config) != 0 || em7028_read_reg(EM7028_REG_CONFIG, &config) != 0)
    {
        return 1;
    }
    /* 回读的模式位与请求不一致，记录配置故障和实际读回值。 */
    if ((config & EM7028_MODE_MASK) != mode)
    {
        em7028_last_fault.kind = EM7028_FAULT_CONFIG;
        em7028_last_fault.value = config;
        return 1;
    }
    return 0;
}

em7028_result em7028_init(void)
{
    /* 通用读回值：先放 PID，后面放 INT_CTRL、HRS1_CTRL。 */
    uint8_t value;
    /* PID 读取重试计数。 */
    uint8_t attempt;
    /* 先把模块状态清零，避免沿用上一次初始化留下的标志。 */
    em7028_ready = 0;
    em7028_running = 0;
    em7028_last_fault.kind = EM7028_FAULT_NONE;
    em7028_last_fault.value = 0;
    /* 拉低外部心率照明使能，初始化期间保持照明关闭。 */
    HAL_GPIO_WritePin(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_PIN_RESET);

    /* 偶发错误PID可重新读；三次仍不匹配才失败，识别成功前不写配置。 */
    for (attempt = 0; attempt < 3; ++attempt)
    {
        /* 读 PID 寄存器，总线失败直接结束初始化。 */
        if (em7028_read_reg(EM7028_REG_PID, &value) != 0)
        {
            return EM7028_ERROR;
        }
        /* 读到的 PID 与预期一致即可跳出重试循环。 */
        if (value == EM7028_PRODUCT_ID)
        {
            break;
        }
    }
    /* 三次都没读到正确 PID，记录故障和最后读到的值。 */
    if (value != EM7028_PRODUCT_ID)
    {
        em7028_last_fault.kind = EM7028_FAULT_PID;
        em7028_last_fault.value = value;
        return EM7028_ERROR;
    }
    /* 先关闭所有 HRS 模式，并读回中断控制寄存器当前值。 */
    if ((em7028_set_mode(0) != 0) || (em7028_read_reg(EM7028_REG_INT_CTRL, &value) != 0))
    {
        return EM7028_ERROR;
    }
    /* 清除中断使能位，只改动这一位，其余配置保留。 */
    value = (uint8_t)(value & (uint8_t)~EM7028_INT_EN);
    /* 写回中断控制，并写入 HRS1 的工作参数。 */
    if ((em7028_write_reg(EM7028_REG_INT_CTRL, value) != 0) || (em7028_write_reg(EM7028_REG_HRS1_CTRL, EM7028_HRS1_CTRL) != 0))
    {
        return EM7028_ERROR;
    }
    /* 回读 HRS1 控制寄存器做配置校验。 */
    if (em7028_read_reg(EM7028_REG_HRS1_CTRL, &value) != 0)
    {
        return EM7028_ERROR;
    }
    /* 回读值与写入值不一致，记录配置故障。 */
    if (value != EM7028_HRS1_CTRL)
    {
        em7028_last_fault.kind = EM7028_FAULT_CONFIG;
        em7028_last_fault.value = value;
        return EM7028_ERROR;
    }
    /* PID 与配置都已确认，标记模块就绪。 */
    em7028_ready = 1;
    return EM7028_OK;
}

em7028_result em7028_start(void)
{
    /* 已在运行则不重复启动，直接返回成功。 */
    if (em7028_running != 0)
    {
        return EM7028_OK;
    }
    /* START 请求重新确认 PID 和配置，不能仅信任开机时缓存的 ready。 */
    if (em7028_init() != EM7028_OK)
    {
        return EM7028_ERROR;
    }
    /* 打开 HRS1 模式，开始按配置周期采样。 */
    if (em7028_set_mode(EM7028_HRS1_EN) != 0)
    {
        return EM7028_ERROR;
    }
    /* 拉高LED_EN开启外部心率照明。 */
    HAL_GPIO_WritePin(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_PIN_SET);
    /* 记录启动时刻，作为首次允许读数的周期起点。 */
    em7028_start_tick = HAL_GetTick();
    em7028_running = 1;
    return EM7028_OK;
}

em7028_result em7028_stop(void)
{
    /* 未就绪路径下用于确认器件身份的 PID 读回值。 */
    uint8_t pid;
    /* 关闭外部照明并清软件运行标志；芯片停止结果由后续返回码表示。 */
    HAL_GPIO_WritePin(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_PIN_RESET);
    em7028_running = 0;
    /* 模块未就绪时需先确认 PID，避免对错误器件写配置。 */
    if (em7028_ready == 0)
    {
        /* 部分启动失败也可能留下HRS使能；确认器件身份后仍尝试关闭。 */
        if (em7028_read_reg(EM7028_REG_PID, &pid) != 0)
        {
            return EM7028_ERROR;
        }
        /* PID 不符则记录故障，不继续写配置。 */
        if (pid != EM7028_PRODUCT_ID)
        {
            em7028_last_fault.kind = EM7028_FAULT_PID;
            em7028_last_fault.value = pid;
            return EM7028_ERROR;
        }
    }
    /* 关闭所有 HRS 模式，停止采样。 */
    if (em7028_set_mode(0) != 0)
    {
        return EM7028_ERROR;
    }
    return EM7028_OK;
}

em7028_result em7028_read_raw(uint16_t *raw)
{
    /* 数据高字节、低字节，以及复读的高字节。 */
    uint8_t high, low, high_check;
    /* 输出指针为空时无处写结果，按错误返回。 */
    if (raw == NULL)
    {
        return EM7028_ERROR;
    }
    /* 未运行或距启动不足一个采样周期时，尚无有效数据。 */
    if ((em7028_running == 0) || ((uint32_t)(HAL_GetTick() - em7028_start_tick) < EM7028_SAMPLE_PERIOD_MS))
    {
        return EM7028_NOT_READY;
    }
    /* 依次读高字节、低字节，再复读高字节，任一失败按错误返回。 */
    if ((em7028_read_reg(EM7028_REG_DATA_HIGH, &high) != 0) || (em7028_read_reg(EM7028_REG_DATA_LOW, &low) != 0) ||
        (em7028_read_reg(EM7028_REG_DATA_HIGH, &high_check) != 0))
    {
        return EM7028_ERROR;
    }
    /* 两次高字节不同说明读取期间数据被刷新，本帧丢弃。 */
    if (high != high_check)
    {
        em7028_last_fault.kind = EM7028_FAULT_DATA_CHANGED;
        em7028_last_fault.reg = EM7028_REG_DATA_HIGH;
        return EM7028_NOT_READY;
    }
    /* 高字节在前拼成 16 位原始值。 */
    *raw = (uint16_t)(((uint16_t)high << 8) | low);
    /* 成功取到一帧，清除上一次故障记录。 */
    em7028_last_fault.kind = EM7028_FAULT_NONE;
    return EM7028_OK;
}

void em7028_get_fault(em7028_fault *fault)
{
    /* 只在调用方提供缓冲区时复制最近一次故障快照。 */
    if (fault != NULL)
    {
        *fault = em7028_last_fault;
    }
}
