/* 真实my_i2c事务：GPIO桩区分空闲采样、ACK/数据与恢复时钟。 */
#include "my_i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef host_gpio_a, host_gpio_b;
static my_i2c_bus_t selected_bus;
static GPIO_PinState scl = GPIO_PIN_SET, sda = GPIO_PIN_SET;
static uint32_t sda_mode, primask, nops;
static uint16_t scl_pin = SENSOR_SCL_Pin, sda_pin = SENSOR_SDA_Pin;
static GPIO_TypeDef *port = GPIOB;
static uint8_t stuck_sda, stuck_scl, in_transaction, release_after;
static unsigned clocks, starts, stops, response_pos, scl_reads;
static const uint8_t read_responses[] = {0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1};
static uint8_t nack_only;

void host_i2c_nop(void)
{
    ++nops;
}
uint32_t __get_PRIMASK(void)
{
    return primask;
}
void __disable_irq(void)
{
    primask = 1;
}
void __set_PRIMASK(uint32_t value)
{
    primask = value;
}
void HAL_GPIO_Init(GPIO_TypeDef *gpio, GPIO_InitTypeDef *config)
{
    assert(gpio == port && primask == 1);
    if (config->Pin == sda_pin)
    {
        sda_mode = config->Mode;
    }
    else
    {
        assert(config->Pin == scl_pin);
        if (selected_bus != MY_I2C_BUS_TP)
        {
            assert(config->Mode == GPIO_MODE_OUTPUT_OD);
        }
    }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState value)
{
    assert(gpio == port);
    if (pin == scl_pin)
    {
        if (scl == GPIO_PIN_RESET && value == GPIO_PIN_SET && in_transaction == 0)
        {
            assert(sda_mode == GPIO_MODE_INPUT || sda == GPIO_PIN_RESET);
            ++clocks;
            if (release_after != 0 && clocks >= release_after)
            {
                stuck_sda = 0;
            }
        }
        scl = value;
    }
    else
    {
        assert(pin == sda_pin);
        if (scl == GPIO_PIN_SET && sda == GPIO_PIN_SET && value == GPIO_PIN_RESET &&
            sda_mode != GPIO_MODE_INPUT && stuck_sda == 0 && stuck_scl == 0)
        {
            ++starts;
            in_transaction = 1;
        }
        if (scl == GPIO_PIN_SET && sda == GPIO_PIN_RESET && value == GPIO_PIN_SET)
        {
            ++stops;
            in_transaction = 0;
        }
        sda = value;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio, uint16_t pin)
{
    assert(gpio == port && primask == 0);
    if (pin == scl_pin)
    {
        ++scl_reads;
        if (stuck_scl != 0)
        {
            return GPIO_PIN_RESET;
        }
        return scl;
    }
    assert(pin == sda_pin && sda_mode == GPIO_MODE_INPUT && scl == GPIO_PIN_SET);
    if (in_transaction != 0)
    {
        ++response_pos;
        if (nack_only != 0)
        {
            return GPIO_PIN_SET;
        }
        assert(response_pos <= sizeof(read_responses));
        if (read_responses[response_pos - 1U] != 0)
        {
            return GPIO_PIN_SET;
        }
        return GPIO_PIN_RESET;
    }
    if (stuck_sda != 0)
    {
        return GPIO_PIN_RESET;
    }
    return GPIO_PIN_SET;
}

int main(int argc, char **argv)
{
    uint8_t value = 0x5A;
    assert(argc == 2);
    if (strcmp(argv[1], "eeprom_nack") == 0)
    {
        selected_bus = MY_I2C_BUS_EEPROM;
        port = GPIOA;
        scl_pin = EEPROM_SCL_Pin;
        sda_pin = EEPROM_SDA_Pin;
        nack_only = 1;
    }
    else if (strcmp(argv[1], "tp_cache") == 0)
    {
        selected_bus = MY_I2C_BUS_TP;
        scl_pin = TP_SCL_Pin;
        sda_pin = TP_SDA_Pin;
    }
    my_i2c_init(selected_bus);
    if (strcmp(argv[1], "sda_release") == 0)
    {
        stuck_sda = 1;
        release_after = 9;
    }
    else if (strcmp(argv[1], "sda_stuck") == 0)
    {
        stuck_sda = 1;
    }
    else if (strcmp(argv[1], "scl_stuck") == 0)
    {
        stuck_scl = 1;
    }
    if (nack_only != 0)
    {
        for (unsigned i = 0; i < 6; ++i)
        {
            assert(my_i2c_probe(selected_bus, 0x50) == 1);
        }
        assert(starts == 6 && stops == 6 && clocks == 0 && response_pos == 6);
    }
    else if (selected_bus == MY_I2C_BUS_TP)
    {
        assert(my_i2c_recover(selected_bus) == 0);
        /* 模拟touch自己的GPIO配置，使上次公共调用的缓存不再描述硬件。 */
        sda_mode = GPIO_MODE_OUTPUT_PP;
        assert(my_i2c_recover(selected_bus) == 0);
        assert(clocks == 20 && sda_mode == GPIO_MODE_INPUT);
    }
    else if (stuck_sda != 0 && release_after == 0)
    {
        assert(my_i2c_read_bytes(selected_bus, 0x68, 0x3A, &value, 1) == 1);
        assert(value == 0x5A && starts == 0 && clocks == 10);
    }
    else if (stuck_scl != 0)
    {
        assert(my_i2c_read_bytes(selected_bus, 0x68, 0x3A, &value, 1) == 1);
        assert(value == 0x5A && starts == 0 && scl_reads <= 32 && nops < 10000);
    }
    else
    {
        assert(my_i2c_read_bytes(selected_bus, 0x68, 0x3A, &value, 1) == 0);
        assert(value == 0xA5 && starts == 2 && response_pos == sizeof(read_responses));
        if (release_after != 0)
        {
            assert(clocks == 10 && stops == 2);
        }
        else
        {
            assert(clocks == 0 && stops == 1);
        }
    }
    puts("PASS: real bus transaction/repeated START, bounded line recovery, NACK exemption and TP cache.");
    return 0;
}
