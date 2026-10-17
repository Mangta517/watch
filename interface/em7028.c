/**
 * @file em7028.c
 * @brief EM7028 手册 V1.0，第 14～21 页；仅实现本板接线对应的 HRS1 模式。
 */
#include "em7028.h"
#include "my_i2c.h"
#include <stddef.h>

#define EM7028_REG_PID       0x00U
#define EM7028_REG_CONFIG    0x01U
#define EM7028_REG_HRS1_CTRL 0x0DU
#define EM7028_REG_INT_CTRL  0x0EU
#define EM7028_REG_DATA_LOW  0x28U
#define EM7028_REG_DATA_HIGH 0x29U
#define EM7028_HRS1_EN       0x08U
#define EM7028_MODE_MASK     0x88U /* HRS2_EN(bit7) + HRS1_EN(bit3)，不能同时开启 */
#define EM7028_INT_EN        0x80U
#define EM7028_HRS1_CTRL     0x47U /* 同款 OV-Watch 配置：gain=1，range=8，freq=000，res=11，IR_MODE=1 */

static uint8_t em7028_ready;
static uint8_t em7028_running;
static uint32_t em7028_start_tick;
static em7028_fault em7028_last_fault;

static uint8_t em7028_write_reg(uint8_t reg, uint8_t value)
{
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, EM7028_ADDR, reg, &value, 1) != 0)
    {
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
    em7028_last_fault.reg = reg;
    if (my_i2c_prepare(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        return 1;
    }
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, (uint8_t)(EM7028_ADDR << 1));
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return 1;
    }
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, reg);
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return 1;
    }
    my_i2c_stop(MY_I2C_BUS_SENSOR);

    if (my_i2c_prepare(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        return 1;
    }
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, (uint8_t)((EM7028_ADDR << 1) | 1U));
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        em7028_last_fault.kind = EM7028_FAULT_READ;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return 1;
    }
    *value = my_i2c_read_byte(MY_I2C_BUS_SENSOR, 1); /* 最后一个字节发送 NACK */
    my_i2c_stop(MY_I2C_BUS_SENSOR);
    return 0;
}

static uint8_t em7028_set_mode(uint8_t mode)
{
    uint8_t config;
    if (em7028_read_reg(EM7028_REG_CONFIG, &config) != 0)
    {
        return 1;
    }
    /* 主表和逐位表的保留位默认值不一致，按实际读回值保留。 */
    config = (uint8_t)((config & (uint8_t)~EM7028_MODE_MASK) | mode);
    if (em7028_write_reg(EM7028_REG_CONFIG, config) != 0 || em7028_read_reg(EM7028_REG_CONFIG, &config) != 0)
    {
        return 1;
    }
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
    uint8_t value;
    uint8_t attempt;
    em7028_ready = 0;
    em7028_running = 0;
    em7028_last_fault.kind = EM7028_FAULT_NONE;
    em7028_last_fault.value = 0;
    HAL_GPIO_WritePin(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_PIN_RESET);

    /* 偶发错误PID可重新读；三次仍不匹配才失败，识别成功前不写配置。 */
    for (attempt = 0; attempt < 3U; ++attempt)
    {
        if (em7028_read_reg(EM7028_REG_PID, &value) != 0)
        {
            return EM7028_ERROR;
        }
        if (value == EM7028_PRODUCT_ID)
        {
            break;
        }
    }
    if (value != EM7028_PRODUCT_ID)
    {
        em7028_last_fault.kind = EM7028_FAULT_PID;
        em7028_last_fault.value = value;
        return EM7028_ERROR;
    }
    if ((em7028_set_mode(0) != 0) || (em7028_read_reg(EM7028_REG_INT_CTRL, &value) != 0))
    {
        return EM7028_ERROR;
    }
    value = (uint8_t)(value & (uint8_t)~EM7028_INT_EN);
    if ((em7028_write_reg(EM7028_REG_INT_CTRL, value) != 0) || (em7028_write_reg(EM7028_REG_HRS1_CTRL, EM7028_HRS1_CTRL) != 0))
    {
        return EM7028_ERROR;
    }
    if (em7028_read_reg(EM7028_REG_HRS1_CTRL, &value) != 0)
    {
        return EM7028_ERROR;
    }
    if (value != EM7028_HRS1_CTRL)
    {
        em7028_last_fault.kind = EM7028_FAULT_CONFIG;
        em7028_last_fault.value = value;
        return EM7028_ERROR;
    }
    em7028_ready = 1;
    return EM7028_OK;
}

em7028_result em7028_start(void)
{
    if (em7028_running != 0)
    {
        return EM7028_OK;
    }
    /* START 请求重新确认 PID 和配置，不能仅信任开机时缓存的 ready。 */
    if (em7028_init() != EM7028_OK)
    {
        return EM7028_ERROR;
    }
    if (em7028_set_mode(EM7028_HRS1_EN) != 0)
    {
        return EM7028_ERROR;
    }
    HAL_GPIO_WritePin(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_PIN_SET);
    em7028_start_tick = HAL_GetTick();
    em7028_running = 1;
    return EM7028_OK;
}

em7028_result em7028_stop(void)
{
    HAL_GPIO_WritePin(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_PIN_RESET);
    em7028_running = 0;
    if (em7028_ready == 0)
    {
        return EM7028_NOT_READY;
    }
    if (em7028_set_mode(0) != 0)
    {
        return EM7028_ERROR;
    }
    return EM7028_OK;
}

em7028_result em7028_read_raw(uint16_t *raw)
{
    uint8_t high, low, high_check;
    if (raw == NULL)
    {
        return EM7028_ERROR;
    }
    if ((em7028_running == 0) || ((uint32_t)(HAL_GetTick() - em7028_start_tick) < EM7028_SAMPLE_PERIOD_MS))
    {
        return EM7028_NOT_READY;
    }
    if ((em7028_read_reg(EM7028_REG_DATA_HIGH, &high) != 0) || (em7028_read_reg(EM7028_REG_DATA_LOW, &low) != 0) ||
        (em7028_read_reg(EM7028_REG_DATA_HIGH, &high_check) != 0))
    {
        return EM7028_ERROR;
    }
    if (high != high_check)
    {
        em7028_last_fault.kind = EM7028_FAULT_DATA_CHANGED;
        em7028_last_fault.reg = EM7028_REG_DATA_HIGH;
        return EM7028_NOT_READY;
    }
    *raw = (uint16_t)(((uint16_t)high << 8) | low);
    em7028_last_fault.kind = EM7028_FAULT_NONE;
    return EM7028_OK;
}

void em7028_get_fault(em7028_fault *fault)
{
    if (fault != NULL)
    {
        *fault = em7028_last_fault;
    }
}
