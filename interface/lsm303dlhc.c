/**
 * @file lsm303dlhc.c
 * @brief 仅启用磁力计：75Hz、±1.3gauss、温度关闭，连续转换。
 *        SENSOR 总线运行时由 sensor 任务串行访问，ISR 不操作总线。
 *        手册：https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf
 */
#include "lsm303dlhc.h"
#include "my_i2c.h"
#include <stddef.h>

#define MAG_CRA               0x00
#define MAG_CRB               0x01
#define MAG_MODE              0x02
#define MAG_OUT_X_H           0x03
#define MAG_STATUS            0x09
#define MAG_ID_A              0x0A
#define MAG_SAMPLE_TIMEOUT_MS 1000 /* 累计活动1s没有有效帧，进入原有分步重配置；正常睡眠不计时。 */

static uint8_t s_ready;
static uint16_t s_rate_hz_x10 = 750;

static uint8_t mag_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    return my_i2c_read_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, reg, data, len);
}

enum
{
    MAG_RECOVER_ID,
    MAG_RECOVER_CRA,
    MAG_RECOVER_CRB,
    MAG_RECOVER_MODE,
    MAG_RECOVER_FAILED,
    MAG_RECOVER_DONE
};
static struct
{
    uint32_t phase_ms, fault_ms, sample_ms, sleep_ms;
    uint8_t phase, faults, no_data, sleeping;
} mag_recovery;

void lsm303dlhc_mag_recover_begin(void)
{
    s_ready = 0;
    s_rate_hz_x10 = 750;
    mag_recovery.phase = MAG_RECOVER_ID;
    mag_recovery.faults = 0;
    mag_recovery.no_data = 0;
    mag_recovery.sleeping = 0;
    mag_recovery.phase_ms = HAL_GetTick();
}

static uint8_t mag_recover_failed(void)
{
    s_ready = 0;
    mag_recovery.phase = MAG_RECOVER_FAILED;
    mag_recovery.phase_ms = HAL_GetTick();
    return 1;
}

static void mag_read_fault(void)
{
    if (mag_recovery.faults < 3 && ++mag_recovery.faults == 3)
    {
        mag_recovery.fault_ms = HAL_GetTick();
    }
}

uint8_t lsm303dlhc_mag_recover_step(uint32_t now_ms)
{
    static const uint8_t config[] = {0x18, 0x20, 0x00};
    static const uint8_t regs[] = {MAG_CRA, MAG_CRB, MAG_MODE};
    uint8_t id[3];
    uint8_t index;
    if (s_ready != 0)
    {
        if ((mag_recovery.no_data != 0 && (uint32_t)(now_ms - mag_recovery.sample_ms) >= MAG_SAMPLE_TIMEOUT_MS) ||
            (mag_recovery.faults >= 3 && (uint32_t)(now_ms - mag_recovery.fault_ms) >= 1000))
        {
            lsm303dlhc_mag_recover_begin();
            return 2;
        }
        return 0;
    }
    if (mag_recovery.phase == MAG_RECOVER_FAILED)
    {
        if ((uint32_t)(now_ms - mag_recovery.phase_ms) >= 1000)
        {
            lsm303dlhc_mag_recover_begin();
            return 2;
        }
        return 1;
    }
    if (mag_recovery.phase == MAG_RECOVER_ID)
    {
        if (mag_read(MAG_ID_A, id, 3) != 0 || id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33)
        {
            return mag_recover_failed();
        }
        mag_recovery.phase = MAG_RECOVER_CRA;
        return 2;
    }
    index = mag_recovery.phase - 1;
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, regs[index], &config[index], 1) != 0)
    {
        return mag_recover_failed();
    }
    if (++mag_recovery.phase == MAG_RECOVER_FAILED)
    {
        mag_recovery.phase = MAG_RECOVER_DONE;
        s_ready = 1;
        mag_recovery.sample_ms = HAL_GetTick();
        return 0;
    }
    return 2;
}

uint8_t lsm303dlhc_mag_init(void)
{
    uint8_t result;
    lsm303dlhc_mag_recover_begin();
    while ((result = lsm303dlhc_mag_recover_step(HAL_GetTick())) == 2)
    {
    }
    return result;
}

uint8_t lsm303dlhc_mag_set_sleep(uint8_t sleeping)
{
    uint8_t mode = 0;
    uint32_t now_ms, elapsed_ms;
    if (sleeping != 0)
    {
        mode = 0x03; /* MR_REG_M: sleep；唤醒恢复连续转换。 */
    }
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, MAG_MODE, &mode, 1) != 0)
    {
        lsm303dlhc_mag_recover_begin();
        return 1;
    }
    now_ms = HAL_GetTick();
    if (sleeping != 0 && mag_recovery.sleeping == 0)
    {
        mag_recovery.sleep_ms = now_ms;
        mag_recovery.sleeping = 1;
    }
    else if (sleeping == 0 && mag_recovery.sleeping != 0)
    {
        /* 休眠只暂停超时，保留跨唤醒的读失败及无数据状态，直到真正取得新帧。 */
        elapsed_ms = now_ms - mag_recovery.sleep_ms;
        mag_recovery.sample_ms += elapsed_ms;
        if (mag_recovery.faults >= 3)
        {
            mag_recovery.fault_ms += elapsed_ms;
        }
        mag_recovery.sleeping = 0;
    }
    return 0;
}

uint8_t lsm303dlhc_mag_set_rate(uint16_t rate_hz_x10)
{
    uint8_t config;
    if (rate_hz_x10 != 750 && rate_hz_x10 != 300 && rate_hz_x10 != 75)
    {
        return 1;
    }
    if (s_ready == 0)
    {
        return 1;
    }
    if (s_rate_hz_x10 == rate_hz_x10)
    {
        return 0;
    }
    config = 0x14; /* 30Hz，保留温度关闭。 */
    if (rate_hz_x10 == 750)
    {
        config = 0x18; /* 运动模式恢复75Hz。 */
    }
    else if (rate_hz_x10 == 75)
    {
        config = 0x0C; /* 7.5Hz。 */
    }
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, MAG_CRA, &config, 1) != 0)
    {
        lsm303dlhc_mag_recover_begin();
        return 1;
    }
    s_rate_hz_x10 = rate_hz_x10;
    return 0;
}

lsm303dlhc_mag_result lsm303dlhc_mag_read(lsm303dlhc_mag_struct *mag)
{
    uint8_t status, buf[6];
    lsm303dlhc_mag_struct sample;
    uint8_t i;
    float sensitivity;

    if (!s_ready || mag == NULL)
    {
        return LSM303DLHC_MAG_ERROR;
    }
    if (mag_read(MAG_STATUS, &status, 1) != 0)
    {
        mag_read_fault();
        return LSM303DLHC_MAG_ERROR;
    }
    mag_recovery.no_data = 1;
    if ((status & 0x03) == 0)
    {
        mag_recovery.faults = 0;
        return LSM303DLHC_MAG_NOT_READY;
    }

    /* LOCK 残留也完整读取以解锁；磁寄存器自动递增，不给 0x03 加 0x80。 */
    if (mag_read(MAG_OUT_X_H, buf, 6) != 0)
    {
        mag_read_fault();
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
        if (i == 2)
        {
            sensitivity = 980.0f;
        }
        else
        {
            sensitivity = 1100.0f;
        }
        sample.field_uT[i] = sample.raw[i] * 100.0f / sensitivity;
    }
    /* 只有整帧读取和转换成功才发布，失败时保留调用方的上一帧。 */
    mag_recovery.faults = 0;
    mag_recovery.no_data = 0;
    mag_recovery.sample_ms = HAL_GetTick();
    *mag = sample;
    return LSM303DLHC_MAG_OK;
}
