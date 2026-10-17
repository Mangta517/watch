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
#include "FreeRTOS.h"
#include "task.h"

#define SPL006_I2C_BUS MY_I2C_BUS_SENSOR

static void spl006_delay_ms(uint32_t ms)
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

/* ---------------------------------------------------------- 出厂校准系数 */
static int32_t spl006_c00;
static int32_t spl006_c10;
static int16_t spl006_c01;
static int16_t spl006_c11;
static int16_t spl006_c20;
static int16_t spl006_c21;
static int16_t spl006_c30;
static uint8_t spl006_coef_ready; /* 1=系数读取与测量配置均成功, process 才可用 */

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
enum
{
    SPL_RECOVER_ID,
    SPL_RECOVER_RESET,
    SPL_RECOVER_SENSOR,
    SPL_RECOVER_COEF_READY,
    SPL_RECOVER_COEFFICIENTS,
    SPL_RECOVER_CONFIG,
    SPL_RECOVER_FAILED,
    SPL_RECOVER_DONE
};
static struct
{
    uint32_t phase_ms, fault_ms, sample_ms;
    uint8_t phase, index, faults;
    uint8_t coef[16];
} spl_recovery;

void spl006_recover_begin(void)
{
    spl006_coef_ready = 0;
    spl_recovery.phase = SPL_RECOVER_ID;
    spl_recovery.index = spl_recovery.faults = 0;
    spl_recovery.phase_ms = HAL_GetTick();
}

static uint8_t spl006_recover_failed(void)
{
    spl006_coef_ready = 0;
    spl_recovery.phase = SPL_RECOVER_FAILED;
    spl_recovery.phase_ms = HAL_GetTick();
    return 1;
}

static void spl006_read_fault(void)
{
    if (spl_recovery.faults < 3U && ++spl_recovery.faults == 3U)
    {
        spl_recovery.fault_ms = HAL_GetTick();
    }
}

static void spl006_decode_coefficients(void)
{
    const uint8_t *c = spl_recovery.coef;
    int32_t raw = ((int32_t)c[0] << 12) | ((int32_t)c[1] << 4) | (c[2] >> 4);
    if ((raw & 0x80000) != 0)
    {
        raw |= (int32_t)0xFFF00000;
    }
    spl006_c00 = raw;
    raw = ((int32_t)(c[3] & 0x0F) << 16) | ((int32_t)c[4] << 8) | c[5];
    if ((raw & 0x80000) != 0)
    {
        raw |= (int32_t)0xFFF00000;
    }
    spl006_c10 = raw;
    spl006_c01 = (int16_t)(((uint16_t)c[6] << 8) | c[7]);
    spl006_c11 = (int16_t)(((uint16_t)c[8] << 8) | c[9]);
    spl006_c20 = (int16_t)(((uint16_t)c[10] << 8) | c[11]);
    spl006_c21 = (int16_t)(((uint16_t)c[12] << 8) | c[13]);
    spl006_c30 = (int16_t)(((uint16_t)c[14] << 8) | c[15]);
}

uint8_t spl006_recover_step(uint32_t now_ms)
{
    /* 保留原逐寄存器顺序，0x14的共享字节也与旧路径一样读取两次。 */
    static const uint8_t coef_regs[] = {
        SPL06_C00_BYTE2, SPL06_C00_BYTE1, SPL06_C00_BYTE0,
        SPL06_C00_BYTE0, SPL06_C10_BYTE1, SPL06_C10_BYTE0,
        SPL06_C01_BYTE1, SPL06_C01_BYTE0, SPL06_C11_BYTE1, SPL06_C11_BYTE0,
        SPL06_C20_BYTE1, SPL06_C20_BYTE0, SPL06_C21_BYTE1, SPL06_C21_BYTE0,
        SPL06_C30_BYTE1, SPL06_C30_BYTE0};
    static const uint8_t config[][2] = {
        {SPL06_PRS_CFG, SPL06_PM_PRC_16X | SPL06_PM_RATE_1PS},
        {SPL06_TMP_CFG, SPL06_TMP_EXT | SPL06_PM_RATE_1PS | SPL06_TMP_PRC_16X},
        {SPL06_CFG_REG, SPL06_P_SHIFT | SPL06_T_SHIFT},
        {SPL06_MEAS_CFG, SPL06_MEAS_BG_PRSTMP}};
    uint8_t status;
    if (spl006_coef_ready != 0)
    {
        if ((uint32_t)(now_ms - spl_recovery.sample_ms) >= SPL006_SAMPLE_MAX_AGE_MS ||
            (spl_recovery.faults >= 3U && (uint32_t)(now_ms - spl_recovery.fault_ms) >= 1000U))
        {
            spl006_recover_begin();
            return 2;
        }
        return 0;
    }
    switch (spl_recovery.phase)
    {
    case SPL_RECOVER_FAILED:
        if ((uint32_t)(now_ms - spl_recovery.phase_ms) >= 1000U)
        {
            spl006_recover_begin();
            return 2;
        }
        return 1;
    case SPL_RECOVER_ID:
        if (spl006_read_reg(SPL06_ID, &status) != 0)
        {
            return spl006_recover_failed();
        }
        spl_recovery.phase = SPL_RECOVER_RESET;
        return 2;
    case SPL_RECOVER_RESET:
        if (spl006_write_reg(SPL06_RESET, SPL06_SOFT_RESET) != 0)
        {
            return spl006_recover_failed();
        }
        spl_recovery.phase = SPL_RECOVER_SENSOR;
        spl_recovery.phase_ms = HAL_GetTick();
        return 2;
    case SPL_RECOVER_SENSOR:
    case SPL_RECOVER_COEF_READY:
        if ((uint32_t)(now_ms - spl_recovery.phase_ms) >= 500U)
        {
            return spl006_recover_failed();
        }
        if ((spl_recovery.phase == SPL_RECOVER_SENSOR && (uint32_t)(now_ms - spl_recovery.phase_ms) < 20U) ||
            (spl_recovery.phase == SPL_RECOVER_COEF_READY && (uint32_t)(now_ms - spl_recovery.phase_ms) < 50U))
        {
            return 2;
        }
        if (spl006_read_reg(SPL06_MEAS_CFG, &status) != 0)
        {
            return 2;
        }
        if (spl_recovery.phase == SPL_RECOVER_SENSOR && (status & SPL06_SENSOR_RDY) != 0)
        {
            spl_recovery.phase = SPL_RECOVER_COEF_READY;
            spl_recovery.phase_ms = HAL_GetTick();
        }
        else if (spl_recovery.phase == SPL_RECOVER_COEF_READY && (status & SPL06_COEF_RDY) != 0)
        {
            spl_recovery.phase = SPL_RECOVER_COEFFICIENTS;
        }
        return 2;
    case SPL_RECOVER_COEFFICIENTS:
        if (spl006_read_reg(coef_regs[spl_recovery.index], &spl_recovery.coef[spl_recovery.index]) != 0)
        {
            return spl006_recover_failed();
        }
        if (++spl_recovery.index >= sizeof(coef_regs))
        {
            spl006_decode_coefficients();
            spl_recovery.index = 0;
            spl_recovery.phase = SPL_RECOVER_CONFIG;
        }
        return 2;
    case SPL_RECOVER_CONFIG:
        if (spl006_write_reg(config[spl_recovery.index][0], config[spl_recovery.index][1]) != 0)
        {
            return spl006_recover_failed();
        }
        if (++spl_recovery.index >= sizeof(config) / sizeof(config[0]))
        {
            spl006_coef_ready = 1;
            spl_recovery.phase = SPL_RECOVER_DONE;
            spl_recovery.sample_ms = HAL_GetTick();
            return 0;
        }
        return 2;
    default:
        return 1;
    }
}

uint8_t spl006_init(void)
{
    uint8_t result;
    spl006_recover_begin();
    while ((result = spl006_recover_step(HAL_GetTick())) == 2U)
    {
        if (spl_recovery.phase == SPL_RECOVER_SENSOR)
        {
            spl006_delay_ms(20);
        }
        else if (spl_recovery.phase == SPL_RECOVER_COEF_READY)
        {
            spl006_delay_ms(50);
        }
    }
    return result;
}

uint8_t spl006_read_pressure(int32_t *pressure_raw)
{
    uint8_t meas_cfg;
    uint8_t buf[3];

    if (spl006_read_reg(SPL06_MEAS_CFG, &meas_cfg) != 0)
    {
        return 2;
    }
    if ((meas_cfg & SPL06_PRS_RDY) == 0)
    {
        return 1;
    }

    /* 突发读 3 字节 (0x00~0x02), 替代 3 次单字节读 */
    if (spl006_bus_read(SPL06_PRS_B2, buf, 3) != 0)
    {
        return 2;
    }

    *pressure_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (*pressure_raw & 0x800000)
    {
        *pressure_raw |= (int32_t)0xFF000000; /* 24bit 补码符号扩展 */
    }
    return 0;
}

uint8_t spl006_read_temperature(int32_t *temperature_raw)
{
    uint8_t meas_cfg;
    uint8_t buf[3];

    if (spl006_read_reg(SPL06_MEAS_CFG, &meas_cfg) != 0)
    {
        return 2;
    }
    if ((meas_cfg & SPL06_TMP_RDY) == 0)
    {
        return 1;
    }

    /* 突发读 3 字节 (0x03~0x05) */
    if (spl006_bus_read(SPL06_TMP_B2, buf, 3) != 0)
    {
        return 2;
    }

    *temperature_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (*temperature_raw & 0x800000)
    {
        *temperature_raw |= (int32_t)0xFF000000;
    }
    return 0;
}

uint16_t spl006_read_sample(int32_t *pressure_raw, int32_t *temperature_raw)
{
    uint8_t status, data[6];
    uint16_t result = 0;
    int32_t pressure, temperature;
    if (spl006_coef_ready == 0)
    {
        return 0x0202U;
    }
    if (spl006_read_reg(SPL06_MEAS_CFG, &status) != 0)
    {
        spl006_read_fault();
        return 0x0202U;
    }
    if ((status & SPL06_PRS_RDY) == 0)
    {
        result |= 1U;
    }
    if ((status & SPL06_TMP_RDY) == 0)
    {
        result |= 0x0100U;
    }
    if (result != 0)
    {
        spl_recovery.faults = 0;
        return result; /* 等整对数据就绪，不提前消耗单个就绪标志。 */
    }
    if (spl006_bus_read(SPL06_PRS_B2, data, sizeof(data)) != 0)
    {
        spl006_read_fault();
        return 0x0202U;
    }
    pressure = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    temperature = ((int32_t)data[3] << 16) | ((int32_t)data[4] << 8) | data[5];
    if ((pressure & 0x800000) != 0)
    {
        pressure |= (int32_t)0xFF000000;
    }
    if ((temperature & 0x800000) != 0)
    {
        temperature |= (int32_t)0xFF000000;
    }
    spl_recovery.faults = 0;
    spl_recovery.sample_ms = HAL_GetTick();
    *pressure_raw = pressure;
    *temperature_raw = temperature;
    return 0;
}

/* ------------------------------------------------------------ 温度补偿 */
uint8_t spl006_process_pressure(int32_t pressure_raw, int32_t temperature_raw, int32_t *compensated_pressure)
{
    float p_sc, t_sc;

    if (spl006_coef_ready == 0)
    {
        return 1; /* 系数未就绪, 输出不可信 */
    }

    p_sc = (float)pressure_raw / SPL06_SCALE_FACTOR_16X;
    t_sc = (float)temperature_raw / SPL06_SCALE_FACTOR_16X;

    *compensated_pressure =
        (int32_t)((float)spl006_c00 + p_sc * ((float)spl006_c10 + p_sc * ((float)spl006_c20 + p_sc * (float)spl006_c30)) +
                  t_sc * (float)spl006_c01 + t_sc * p_sc * ((float)spl006_c11 + p_sc * (float)spl006_c21));
    return 0;
}
