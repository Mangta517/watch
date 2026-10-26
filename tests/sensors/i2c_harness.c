/* 真实软件 I2C 的 GPIO 交接/边沿回归；不模拟电压和实板传播延迟。 */
#include "my_i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef host_gpio_a, host_gpio_b;
static GPIO_PinState scl = GPIO_PIN_SET;
static GPIO_PinState sda_latch = GPIO_PIN_SET;
static uint32_t sda_mode, primask, nop_count, scl_high_tick, sda_low_tick, slave_release_tick;
static uint8_t input_bits[11];
static unsigned input_pos, read_edges, response_edges;
static uint8_t read_data, slave_low, check_stop, nack_pending;
static uint8_t expected_ack;
static unsigned sda_init_calls, redundant_sda_inits;
static unsigned analog_pins;

void host_i2c_nop(void)
{
    ++nop_count;
    if (slave_release_tick != 0 && nop_count >= slave_release_tick)
    {
        slave_low = 0;
        slave_release_tick = 0;
    }
}

uint32_t __get_PRIMASK(void)
{
    return primask;
}

void __disable_irq(void)
{
    primask = 1;
}

void __set_PRIMASK(uint32_t mask)
{
    primask = mask;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    assert(port == GPIOB && primask == 1);
    if (config->Mode == GPIO_MODE_ANALOG)
    {
        assert(config->Pull == GPIO_NOPULL);
        assert(config->Pin == SENSOR_SCL_Pin || config->Pin == SENSOR_SDA_Pin);
        if (config->Pin == SENSOR_SDA_Pin)
        {
            sda_mode = config->Mode;
        }
        ++analog_pins;
        return;
    }
    if (config->Pin == SENSOR_SDA_Pin)
    {
        assert(config->Mode == GPIO_MODE_INPUT || config->Mode == GPIO_MODE_OUTPUT_OD);
        if (sda_init_calls != 0 && sda_mode == config->Mode)
        {
            ++redundant_sda_inits;
        }
        ++sda_init_calls;
        /* 读地址 ACK 后，从机已在 SCL 低期间准备第一位，主机必须释放 SDA。 */
        assert(!(config->Mode == GPIO_MODE_OUTPUT_PP && slave_low != 0 && sda_latch == GPIO_PIN_SET));
        sda_mode = config->Mode;
    }
    else
    {
        assert(config->Pin == SENSOR_SCL_Pin);
        assert(config->Mode == GPIO_MODE_OUTPUT_OD);
    }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOB);
    if (pin == SENSOR_SCL_Pin)
    {
        if (scl == GPIO_PIN_RESET && state == GPIO_PIN_SET)
        {
            if (check_stop != 0)
            {
                assert(nop_count - sda_low_tick >= 80);
            }
            scl_high_tick = nop_count;
            if (read_data != 0)
            {
                assert(sda_mode == GPIO_MODE_INPUT);
                ++read_edges;
            }
            else if (nack_pending != 0)
            {
                if (expected_ack != 0)
                {
                    assert(sda_mode == GPIO_MODE_INPUT); /* NACK必须释放，不推挽驱动高电平。 */
                }
                else
                {
                    assert(sda_mode != GPIO_MODE_INPUT && sda_latch == GPIO_PIN_RESET);
                }
                ++response_edges;
                nack_pending = 0;
            }
        }
        if (scl == GPIO_PIN_SET && state == GPIO_PIN_RESET && input_pos >= 3)
        {
            uint8_t was_reading = read_data;
            read_data = input_pos < sizeof(input_bits);
            if (was_reading != 0 && read_data == 0 && input_bits[10] == 0)
            {
                /* 最后一位为0时，从机可以在下降沿后继续短暂保持低电平。 */
                slave_low = 1;
                slave_release_tick = nop_count + 8;
            }
            else
            {
                slave_low = read_data != 0 && input_bits[input_pos] == 0;
            }
        }
        scl = state;
    }
    else
    {
        assert(pin == SENSOR_SDA_Pin);
        assert(!(sda_mode == GPIO_MODE_OUTPUT_PP && slave_low != 0 && state == GPIO_PIN_SET));
        if (state == GPIO_PIN_RESET)
        {
            sda_low_tick = nop_count;
        }
        if (check_stop != 0 && sda_latch == GPIO_PIN_RESET && state == GPIO_PIN_SET && scl == GPIO_PIN_SET)
        {
            assert(nop_count - scl_high_tick >= 80);
        }
        sda_latch = state;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    assert(port == GPIOB && pin == SENSOR_SDA_Pin && sda_mode == GPIO_MODE_INPUT && scl == GPIO_PIN_SET);
    assert(input_pos < sizeof(input_bits));
    ++input_pos;
    if (input_pos == sizeof(input_bits))
    {
        nack_pending = 1;
    }
    if (input_bits[input_pos - 1] != 0)
    {
        return GPIO_PIN_SET;
    }
    else
    {
        return GPIO_PIN_RESET;
    }
}

static void read_pid(uint8_t pid, uint8_t ack)
{
    unsigned i;
    input_pos = read_edges = response_edges = 0;
    read_data = slave_low = 0;
    slave_release_tick = 0;
    expected_ack = ack;
    memset(input_bits, 0, sizeof(input_bits)); /* 写地址、寄存器地址、读地址三个 ACK */
    for (i = 0; i < 8; ++i)
    {
        input_bits[i + 3] = (uint8_t)((pid >> (7U - i)) & 1U);
    }
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, 0x48);
    assert(my_i2c_wait_ack(MY_I2C_BUS_SENSOR) == 0);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, 0x00);
    assert(my_i2c_wait_ack(MY_I2C_BUS_SENSOR) == 0);
    my_i2c_stop(MY_I2C_BUS_SENSOR);
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, 0x49);
    assert(my_i2c_wait_ack(MY_I2C_BUS_SENSOR) == 0);
    assert(sda_mode == GPIO_MODE_INPUT);
    assert(my_i2c_read_byte(MY_I2C_BUS_SENSOR, ack) == pid);
    assert(input_pos == sizeof(input_bits) && read_edges == 8 && response_edges == 1);
    my_i2c_stop(MY_I2C_BUS_SENSOR);
}

int main(int argc, char **argv)
{
    my_i2c_init(MY_I2C_BUS_SENSOR);
    if (argc > 1 && strcmp(argv[1], "suspend") == 0)
    {
        my_i2c_suspend(MY_I2C_BUS_SENSOR);
        assert(analog_pins == 2 && primask == 0 && scl == GPIO_PIN_SET && sda_latch == GPIO_PIN_SET);
        my_i2c_init(MY_I2C_BUS_SENSOR);
        assert(sda_mode == GPIO_MODE_OUTPUT_OD);
        read_pid(0x36, 1);
        puts("PASS: only SENSOR SCL/SDA enter analog/no-pull, direction cache restores and real transaction resumes.");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "stop") == 0)
    {
        my_i2c_start(MY_I2C_BUS_SENSOR);
        check_stop = 1;
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        assert(scl == GPIO_PIN_SET && sda_latch == GPIO_PIN_SET && primask == 0);
        puts("PASS: STOP holds SDA low before SCL rise and SCL high before SDA release.");
    }
    else
    {
        read_pid(0x36, 1);
        read_pid(0x7E, 1);
        read_pid(0x00, 1);
        read_pid(0xFF, 1);
        read_pid(0x36, 0);
        assert(redundant_sda_inits == 0);
        puts("PASS: software I2C releases NACK, presets ACK low without contention, preserves PID bits and read clocks.");
    }
    return 0;
}
