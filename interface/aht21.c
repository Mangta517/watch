/**
 * @file aht21.c
 * @brief 奥松 AHT21 手册 V1.2，第 8～9 页：初始化、触发、直接读和 CRC8。
 */
#include "aht21.h"
#include "my_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

#define AHT21_STATUS_BUSY       0x80U
#define AHT21_STATUS_CALIBRATED 0x08U

static void aht21_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免tick边界缩短硬件最小等待；等待期间让出CPU。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
    }
    else
    {
        HAL_Delay(ms);
    }
}

/* 0x71 是读地址字节，不是寄存器号；不能使用先写寄存器的 read_bytes。 */
static aht21_result aht21_read_bytes(uint8_t *data, uint8_t size)
{
    uint8_t i;
    if (my_i2c_prepare(MY_I2C_BUS_SENSOR) != 0)
    {
        return AHT21_I2C_ERROR;
    }
    my_i2c_start(MY_I2C_BUS_SENSOR);
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, (uint8_t)((AHT21_ADDR << 1) | 1U));
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return AHT21_I2C_ERROR;
    }
    for (i = 0; i < size; ++i)
    {
        if (i == size - 1U)
        {
            data[i] = my_i2c_read_byte(MY_I2C_BUS_SENSOR, 1U);
        }
        else
        {
            data[i] = my_i2c_read_byte(MY_I2C_BUS_SENSOR, 0U);
        }
    }
    my_i2c_stop(MY_I2C_BUS_SENSOR);
    return AHT21_OK;
}

static uint8_t aht21_crc8(const uint8_t *data)
{
    uint8_t crc = 0xFFU;
    uint8_t i, bit;
    for (i = 0; i < 6U; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31U);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

aht21_result aht21_init(void)
{
    static const uint8_t config[2] = {0x08U, 0x00U};
    uint8_t status;
    aht21_delay_ms(40U);
    if (aht21_read_bytes(&status, 1) != AHT21_OK)
    {
        return AHT21_I2C_ERROR;
    }
    if ((status & AHT21_STATUS_BUSY) != 0)
    {
        return AHT21_BUSY;
    }
    if ((status & AHT21_STATUS_CALIBRATED) == 0)
    {
        if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, AHT21_ADDR, 0xBEU, config, sizeof(config)) != 0)
        {
            return AHT21_I2C_ERROR;
        }
        aht21_delay_ms(10U);
        if (aht21_read_bytes(&status, 1) != AHT21_OK)
        {
            return AHT21_I2C_ERROR;
        }
        if ((status & AHT21_STATUS_BUSY) != 0)
        {
            return AHT21_BUSY;
        }
        if ((status & AHT21_STATUS_CALIBRATED) == 0)
        {
            return AHT21_CALIBRATION_ERROR;
        }
    }
    return AHT21_OK;
}

aht21_result aht21_trigger(void)
{
    static const uint8_t params[2] = {0x33U, 0x00U};
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, AHT21_ADDR, 0xACU, params, sizeof(params)) != 0)
    {
        return AHT21_I2C_ERROR;
    }
    return AHT21_OK;
}

aht21_result aht21_read(aht21_struct *sample)
{
    uint8_t data[7];
    if (sample == NULL)
    {
        return AHT21_INVALID_ARGUMENT;
    }
    if (aht21_read_bytes(data, sizeof(data)) != AHT21_OK)
    {
        return AHT21_I2C_ERROR;
    }
    if ((data[0] & AHT21_STATUS_BUSY) != 0)
    {
        return AHT21_BUSY;
    }
    if (aht21_crc8(data) != data[6])
    {
        return AHT21_CRC_ERROR;
    }
    if ((data[0] & AHT21_STATUS_CALIBRATED) == 0)
    {
        return AHT21_CALIBRATION_ERROR;
    }
    sample->humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
    sample->temperature_raw = ((uint32_t)(data[3] & 0x0FU) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];
    sample->humidity_rh = (float)sample->humidity_raw * 100.0f / 1048576.0f;
    sample->temperature_c = (float)sample->temperature_raw * 200.0f / 1048576.0f - 50.0f;
    sample->status = data[0];
    return AHT21_OK;
}
