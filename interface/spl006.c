/**
 ******************************************************************************
 * @file    spl006.c
 * @brief   SPL006 气压计驱动实现 —— 软件 I2C SENSOR 总线, 7 位地址 0x76 (SDO 接地)
 *          移植自 flight_hal/interface/spl006.c: 校准系数解析与补偿公式原样保留,
 *          HAL_I2C_Mem_* (0xEE/0xEF 移位地址) 替换为 my_i2c_* (内部自动拼读写位)。
 ******************************************************************************
 */
#include "spl006.h"

#include "my_i2c.h"

#define SPL006_I2C_BUS   MY_I2C_BUS_SENSOR

/* ---------------------------------------------------------- 出厂校准系数 */
static int32_t spl006_c00;
static int32_t spl006_c10;
static int16_t spl006_c01;
static int16_t spl006_c11;
static int16_t spl006_c20;
static int16_t spl006_c21;
static int16_t spl006_c30;
static uint8_t spl006_coef_ready;   /* 1=init 已成功读入系数, process 才可用 */

/* ---------------------------------------------------------- 总线独占保护 */
/* 与 MPU6050/LSM303DLHC 同属 sensor 任务串行访问；ISR 不操作 I2C。
 * GPIO 方向切换仍由 my_i2c 内的短临界区保护，无需整次事务关中断。 */
static uint8_t spl006_bus_write(uint8_t reg, const uint8_t *dat, uint16_t len)
{
    return my_i2c_write_bytes(SPL006_I2C_BUS, SPL006_ADDR, reg, dat, len);
}

static uint8_t spl006_bus_read(uint8_t reg, uint8_t *dat, uint16_t len)
{
    return my_i2c_read_bytes(SPL006_I2C_BUS, SPL006_ADDR, reg, dat, len);
}

static uint8_t spl006_write_reg(uint8_t reg, uint8_t data)
{
    return spl006_bus_write(reg, &data, 1);
}

static uint8_t spl006_read_reg(uint8_t reg, uint8_t *data)
{
    return spl006_bus_read(reg, data, 1);
}

/* ------------------------------------------------------------ 初始化 */
uint8_t spl006_init(void)
{
    uint8_t id = 0;
    uint8_t status = 0;
    uint8_t hb, mb, lb;
    int32_t raw;

    /* 通信自检: 不校验 ID 具体值 (0x07/0x08 随子型号而异), NACK 即认为器件不在 */
    if (spl006_read_reg(SPL06_ID, &id) != 0)
    {
        return 1;
    }

    /* 软复位 → 等传感器与系数就绪 (时序沿用旧工程实测值: 20ms + 50ms) */
    if (spl006_write_reg(SPL06_RESET, SPL06_SOFT_RESET) != 0)
    {
        return 1;
    }
    HAL_Delay(20);
    if ((spl006_read_reg(SPL06_MEAS_CFG, &status) != 0) ||
        ((status & SPL06_SENSOR_RDY) == 0))
    {
        return 1;
    }
    HAL_Delay(50);
    if ((spl006_read_reg(SPL06_MEAS_CFG, &status) != 0) ||
        ((status & SPL06_COEF_RDY) == 0))
    {
        return 1;
    }

    /* 读校准系数 (解析与旧工程逐位一致, 勿"顺手优化"成连续突发读) */
    spl006_read_reg(SPL06_C00_BYTE2, &hb);
    spl006_read_reg(SPL06_C00_BYTE1, &mb);
    spl006_read_reg(SPL06_C00_BYTE0, &lb);
    raw = ((int32_t)hb << 12) | ((int32_t)mb << 4) | (lb >> 4);
    if (raw & 0x80000) raw |= (int32_t)0xFFF00000;      /* 20bit 符号扩展 */
    spl006_c00 = raw;

    spl006_read_reg(SPL06_C00_BYTE0, &hb);              /* 0x15 低 4 位 = c10[19:16] */
    spl006_read_reg(SPL06_C10_BYTE1, &mb);
    spl006_read_reg(SPL06_C10_BYTE0, &lb);
    raw = ((int32_t)(hb & 0x0F) << 16) | ((int32_t)mb << 8) | lb;
    if (raw & 0x80000) raw |= (int32_t)0xFFF00000;
    spl006_c10 = raw;

    spl006_read_reg(SPL06_C01_BYTE1, &hb);
    spl006_read_reg(SPL06_C01_BYTE0, &lb);
    spl006_c01 = (int16_t)(((uint16_t)hb << 8) | lb);   /* 16bit 补码直存 */

    spl006_read_reg(SPL06_C11_BYTE1, &hb);
    spl006_read_reg(SPL06_C11_BYTE0, &lb);
    spl006_c11 = (int16_t)(((uint16_t)hb << 8) | lb);

    spl006_read_reg(SPL06_C20_BYTE1, &hb);
    spl006_read_reg(SPL06_C20_BYTE0, &lb);
    spl006_c20 = (int16_t)(((uint16_t)hb << 8) | lb);

    spl006_read_reg(SPL06_C21_BYTE1, &hb);
    spl006_read_reg(SPL06_C21_BYTE0, &lb);
    spl006_c21 = (int16_t)(((uint16_t)hb << 8) | lb);

    spl006_read_reg(SPL06_C30_BYTE1, &hb);
    spl006_read_reg(SPL06_C30_BYTE0, &lb);
    spl006_c30 = (int16_t)(((uint16_t)hb << 8) | lb);

    spl006_coef_ready = 1;

    /* 压力/温度均配 16 倍过采样 + 1 次/秒背景模式 (手表场景, 从飞控的 64Hz 降档;
     * 16X>8X 故 P_SHIFT/T_SHIFT 置 1, 与 SPL06_SCALE_FACTOR_16X 绑定, 改动需两边同步) */
    if (spl006_write_reg(SPL06_PRS_CFG, SPL06_PM_PRC_16X | SPL06_PM_RATE_1PS) != 0)
    {
        return 1;
    }
    if (spl006_write_reg(SPL06_TMP_CFG, SPL06_TMP_EXT | SPL06_PM_RATE_1PS | SPL06_TMP_PRC_16X) != 0)
    {
        return 1;
    }
    if (spl006_write_reg(SPL06_CFG_REG, SPL06_P_SHIFT | SPL06_T_SHIFT) != 0)
    {
        return 1;
    }
    if (spl006_write_reg(SPL06_MEAS_CFG, SPL06_MEAS_BG_PRSTMP) != 0)
    {
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------ 原始读数 */
uint8_t spl006_read_pressure(int32_t *pressure_raw)
{
    uint8_t meas_cfg;
    uint8_t buf[3];

    if (spl006_read_reg(SPL06_MEAS_CFG, &meas_cfg) != 0)
        return 2;
    if ((meas_cfg & SPL06_PRS_RDY) == 0)
        return 1;

    /* 突发读 3 字节 (0x00~0x02), 替代 3 次单字节读 */
    if (spl006_bus_read(SPL06_PRS_B2, buf, 3) != 0)
        return 2;

    *pressure_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (*pressure_raw & 0x800000)
        *pressure_raw |= (int32_t)0xFF000000;           /* 24bit 补码符号扩展 */
    return 0;
}

uint8_t spl006_read_temperature(int32_t *temperature_raw)
{
    uint8_t meas_cfg;
    uint8_t buf[3];

    if (spl006_read_reg(SPL06_MEAS_CFG, &meas_cfg) != 0)
        return 2;
    if ((meas_cfg & SPL06_TMP_RDY) == 0)
        return 1;

    /* 突发读 3 字节 (0x03~0x05) */
    if (spl006_bus_read(SPL06_TMP_B2, buf, 3) != 0)
        return 2;

    *temperature_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (*temperature_raw & 0x800000)
        *temperature_raw |= (int32_t)0xFF000000;
    return 0;
}

/* ------------------------------------------------------------ 温度补偿 */
uint8_t spl006_process_pressure(int32_t pressure_raw, int32_t temperature_raw,
                                int32_t *compensated_pressure)
{
    float p_sc, t_sc;

    if (spl006_coef_ready == 0)
        return 1;                                      /* 系数未就绪, 输出不可信 */

    p_sc = (float)pressure_raw / SPL06_SCALE_FACTOR_16X;
    t_sc = (float)temperature_raw / SPL06_SCALE_FACTOR_16X;

    *compensated_pressure = (int32_t)(
        (float)spl006_c00
        + p_sc * ((float)spl006_c10 + p_sc * ((float)spl006_c20 + p_sc * (float)spl006_c30))
        + t_sc * (float)spl006_c01
        + t_sc * p_sc * ((float)spl006_c11 + p_sc * (float)spl006_c21));
    return 0;
}
