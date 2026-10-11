/**
 * @file lsm303dlhc.c
 * @brief 仅启用磁力计：75Hz、±1.3gauss、温度关闭，连续转换。
 *        SENSOR 总线运行时由 sensor 任务串行访问，ISR 不操作总线。
 *        手册：https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf
 */
#include "lsm303dlhc.h"
#include "my_i2c.h"
#include <stddef.h>

#define MAG_CRA     0x00
#define MAG_CRB     0x01
#define MAG_MODE    0x02
#define MAG_OUT_X_H 0x03
#define MAG_STATUS  0x09
#define MAG_ID_A    0x0A

static uint8_t s_ready;

static uint8_t mag_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    return my_i2c_read_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, reg, data, len);
}

uint8_t lsm303dlhc_mag_init(void)
{
    uint8_t id[3];
    const uint8_t config[3] = {0x18, 0x20, 0x00}; /* 75Hz、±1.3gauss、连续转换 */
    const uint8_t regs[3] = {MAG_CRA, MAG_CRB, MAG_MODE};
    uint8_t i;

    s_ready = 0;
    if (mag_read(MAG_ID_A, id, 3) != 0 || id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33)
    {
        return 1;
    }

    /* 按CRA → CRB → MR顺序设置速率、量程，最后启动连续转换。 */
    for (i = 0; i < 3; ++i)
    {
        if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, regs[i], &config[i], 1) != 0)
        {
            return 1;
        }
    }
    s_ready = 1;
    return 0;
}

lsm303dlhc_mag_result lsm303dlhc_mag_read(lsm303dlhc_mag_struct *mag)
{
    uint8_t status, buf[6];
    lsm303dlhc_mag_struct sample;
    uint8_t i;

    if (!s_ready || mag == NULL || mag_read(MAG_STATUS, &status, 1) != 0)
    {
        return LSM303DLHC_MAG_ERROR;
    }
    if ((status & 0x03) == 0)
    {
        return LSM303DLHC_MAG_NOT_READY;
    }

    /* LOCK 残留也完整读取以解锁；磁寄存器自动递增，不给 0x03 加 0x80。 */
    if (mag_read(MAG_OUT_X_H, buf, 6) != 0)
    {
        return LSM303DLHC_MAG_ERROR;
    }
    if ((status & 0x01) == 0)
    {
        return LSM303DLHC_MAG_NOT_READY;
    }

    /* 寄存器为X高/低、Z高/低、Y高/低；快照统一存成X/Y/Z。 */
    sample.raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    sample.raw[2] = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    sample.raw[1] = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    for (i = 0; i < 3; ++i)
    {
        /* 正常输出 -2048..2047；包含 -4096 溢出码在内的异常值不能融合。 */
        if (sample.raw[i] < -2048 || sample.raw[i] > 2047)
        {
            return LSM303DLHC_MAG_ERROR;
        }
        sample.field_uT[i] = sample.raw[i] * 100.0f / (i == 2 ? 980.0f : 1100.0f);
    }
    /* 只有整帧读取和转换成功才发布，失败时保留调用方的上一帧。 */
    *mag = sample;
    return LSM303DLHC_MAG_OK;
}
