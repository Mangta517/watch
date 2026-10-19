/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 驱动实现 —— 软件 I2C (common/my_i2c.c, SENSOR 总线)
 *          移植自旧工程 flight_hal/interface/my_mpu6050.c：
 *          手表运动档：±1000°/s、±4g、100Hz，配置/灵敏度集中在头文件。
 *          数据就绪中断: 器件 INT_PIN_CFG=0x20 (高有效/推挽/锁存, 读 0x3A 清除),
 *          INT_ENABLE=0x01 (DATA_RDY)；PB12 EXTI 只记事件，任务突发读释放锁存。
 ******************************************************************************
 */
#include "mpu6050.h"
#include "my_i2c.h"

#include "FreeRTOS.h"
#include "task.h"   /* 初始化重试与校准循环中的任务级延时 */
#include <stdlib.h> /* abs */
#include <string.h>

/* MPU6050 挂在软件 I2C 的传感器总线上 */
#define MPU6050_I2C_BUS MY_I2C_BUS_SENSOR

static void mpu6050_delay_ms(uint32_t ms)
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

static volatile uint8_t mpu6050_int_flag;
static volatile uint8_t mpu6050_int_status; /* sensor 任务读回的状态 */
static volatile uint16_t mpu6050_int_edges;
static uint8_t mpu6050_ready;

Gyro_struct gyro_data = {0};
Accel_struct accel_data = {0};

int16_t offset_accel_x;
int16_t offset_accel_y;
int16_t offset_accel_z;

int16_t offset_gyro_x;
int16_t offset_gyro_y;
int16_t offset_gyro_z;

/* ---------------------------------------------------------- 总线独占保护 */
/* 运行时仅 sensor 任务访问 SENSOR 总线；EXTI 不做 I2C。
 * 无需整次事务关中断，GPIO 方向切换的短临界区仍在 my_i2c 内。 */
static uint8_t mpu6050_bus_write(uint8_t reg, const uint8_t *dat, uint16_t len)
{
    return my_i2c_write_bytes(MPU6050_I2C_BUS, MPU6050_ADDR, reg, dat, len);
}

static uint8_t mpu6050_bus_read(uint8_t reg, uint8_t *dat, uint16_t len)
{
    return my_i2c_read_bytes(MPU6050_I2C_BUS, MPU6050_ADDR, reg, dat, len);
}

/* -------------------------------------------------------------- 单寄存器访问 */
/**
 * @brief 写寄存器
 * @return 0=成功, 1=总线失败
 */
uint8_t my_mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    return mpu6050_bus_write(reg, &data, 1);
}

/**
 * @brief 读单个寄存器
 * @return 0=成功, 1=总线失败
 */
uint8_t my_mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    return mpu6050_bus_read(reg, data, 1);
}

/* ---------------------------------------------------------------- 初始化 */
/**
 * @brief 初始化 MPU6050：复位 -> 唤醒 -> 量程/采样率/DLPF 配置
 * @return 0=成功, 1=复位等待超时或任一配置读写失败
 * @note  调用前需先执行 my_i2c_init(MY_I2C_BUS_SENSOR)
 */
uint8_t my_mpu6050_init(void)
{
    uint8_t data = 0;
    uint8_t id = 0;
    uint8_t retry;
    uint16_t guard;

    mpu6050_ready = 0;

    /* 上电就绪门: MPU6050 POR 后 I2C 接口需要 ~100ms 才能应答。
     * 本工程 init 由sensor任务启动时调用；若器件还未就绪便首写，
     * NACK 直接返回, 器件将停留在默认 SLEEP 态 → 输出寄存器全 0、
     * INT 永远不触发, 表现为"陀螺仪没有数据"。
     * 以 WHO_AM_I 是否获得 ACK 为准 (不比较数值, 兼容 0x68/0x98/0x69 克隆片);
     * 调度器运行时重试等待让出CPU；重试次数和50ms等待保持不变，总等待约1s。 */
    retry = 0;
    for (;;)
    {
        if (my_mpu6050_read_reg(MPU6050_WHO_AM_I, &id) == 0)
        {
            break; /* 器件已在总线上应答 */
        }
        if (++retry >= 20)
        {
            return 1; /* 1s 无应答: 未接/供电异常 */
        }
        mpu6050_delay_ms(50);
    }

    /* 重置 MPU6050：对电源管理寄存器 1 的 RESET 位置 1 */
    if (my_mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80) != 0)
    {
        return 1;
    }

    /* 重置后 PWR_MGMT_1 应为 0x40 (Sleep bit 自持)，加超时防总线卡死 */
    for (guard = 0; guard < 1000; guard++)
    {
        if (my_mpu6050_read_reg(MPU6050_REG_PWR_MGMT_1, &data) != 0)
        {
            return 1;
        }
        if (data == 0x40)
        {
            break;
        }
    }
    if (guard >= 1000)
    {
        return 1; /* 等待复位完成超时 */
    }

    /* 唤醒 MPU6050 */
    if (my_mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00) != 0)
    {
        return 1;
    }
    /* 量程同时供公共层换算、1g判断和零偏校准引用。 */
    if (my_mpu6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_RANGE) != 0)
    {
        return 1;
    }
    if (my_mpu6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE) != 0)
    {
        return 1;
    }
    /* 中断引脚配置 (0x37) = 0x20: INT 高有效、推挽输出、中断锁存,
     * 仅读 INT_STATUS(0x3A) 后清除锁存释放 INT —— 配合 PB12 上升沿 EXTI */
    if (my_mpu6050_write_reg(MPU6050_INT_PIN_CFG, 0x20) != 0)
    {
        return 1;
    }
    /* 使能数据就绪中断 (0x38 bit0 DATA_RDY): 采样数据更新时 INT 拉高 */
    if (my_mpu6050_write_reg(MPU6050_INT_ENABLE, 0x01) != 0)
    {
        return 1;
    }
    /* 用户控制寄存器：不用 FIFO 和外设 I2C 主机 */
    if (my_mpu6050_write_reg(MPU6050_USER_CTRL, 0x00) != 0)
    {
        return 1;
    }
    /* 100Hz输出与sensor任务同步；42/44Hz低通保留转腕响应并抑制高频噪声。 */
    if (my_mpu6050_write_reg(MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_PERIOD_MS - 1U) != 0)
    {
        return 1;
    }
    if (my_mpu6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_CFG) != 0)
    {
        return 1;
    }
    /* 时钟源选内部 X 轴陀螺仪 */
    if (my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01) != 0)
    {
        return 1;
    }
    /* 使能加速度计和陀螺仪 (PWR_MGMT_2 全部轴不关断) */
    if (my_mpu6050_write_reg(MPU6050_PWR_MGMT_2, 0x00) != 0)
    {
        return 1;
    }

    /* 配置过程中可能已产生锁存的 DATA_RDY: 读一次 0x3A 清除锁存，
     * 确保 INT 初始为低，之后每个数据就绪都产生干净的上升沿 */
    if (my_mpu6050_read_reg(MPU6050_INT_STATUS, &data) != 0)
    {
        return 1;
    }
    mpu6050_ready = 1;

    /* 注意: 上电初始化不做零偏校准；应用任务使用begin/step，
     * 在正常采样中等待静止并累计校准帧。 */
    return 0;
}

/* ---------------------------------------------------------------- 零偏校准 */
typedef struct
{
    Accel_struct last_accel;
    int32_t gravity_z_lsb; /* 首帧确定本次校准的正/负1g基准，采样期间不切换。 */
    int32_t sum[6];
    uint32_t phase_ms;
    uint32_t last_sample_ms;
    uint16_t stable_count;
    uint16_t average_count;
    uint8_t have_sample;
    mpu6050_cal_state state;
    uint32_t diag[MPU_CAL_DIAG_FIELD_COUNT];
} mpu6050_cal_struct;

static mpu6050_cal_struct s_cal;
static const int32_t mpu_cal_still_lsb = (int32_t)(0.0183f * MPU6050_ACCEL_LSB_PER_G);
/* 水平容差0.15g，给初始零偏及小幅安装误差留余量；静止变化门限保持原值。 */
static const int32_t mpu_cal_level_lsb = (int32_t)(0.15f * MPU6050_ACCEL_LSB_PER_G);

const uint32_t *my_mpu6050_zero_offset_diag(void)
{
    s_cal.diag[MPU_CAL_DIAG_LEVEL_LIMIT] = (uint32_t)mpu_cal_level_lsb;
    s_cal.diag[MPU_CAL_DIAG_STILL_LIMIT] = (uint32_t)mpu_cal_still_lsb;
    s_cal.diag[MPU_CAL_DIAG_STATE] = (uint32_t)s_cal.state;
    return s_cal.diag;
}

void my_mpu6050_get_offset(mpu6050_offset_struct *offset)
{
    offset->accel[0] = offset_accel_x;
    offset->accel[1] = offset_accel_y;
    offset->accel[2] = offset_accel_z;
    offset->gyro[0] = offset_gyro_x;
    offset->gyro[1] = offset_gyro_y;
    offset->gyro[2] = offset_gyro_z;
}

void my_mpu6050_restore_offset(const mpu6050_offset_struct *offset)
{
    offset_accel_x = offset->accel[0];
    offset_accel_y = offset->accel[1];
    offset_accel_z = offset->accel[2];
    offset_gyro_x = offset->gyro[0];
    offset_gyro_y = offset->gyro[1];
    offset_gyro_z = offset->gyro[2];
    memset(&s_cal, 0, sizeof(s_cal));
    s_cal.state = MPU6050_CAL_RESTORED;
}

void my_mpu6050_zero_offset_begin(uint32_t now_ms)
{
    memset(&s_cal, 0, sizeof(s_cal));
    s_cal.phase_ms = now_ms;
    s_cal.state = MPU6050_CAL_WAIT_STILL;
}

static uint8_t mpu6050_cal_still(const Accel_struct *current)
{
    uint32_t reason = 0;
    s_cal.diag[MPU_CAL_DIAG_AX] = (uint32_t)(int32_t)current->Accel_x;
    s_cal.diag[MPU_CAL_DIAG_AY] = (uint32_t)(int32_t)current->Accel_y;
    s_cal.diag[MPU_CAL_DIAG_AZ] = (uint32_t)(int32_t)current->Accel_z;
    s_cal.diag[MPU_CAL_DIAG_DX] = (uint32_t)abs(current->Accel_x - s_cal.last_accel.Accel_x);
    s_cal.diag[MPU_CAL_DIAG_DY] = (uint32_t)abs(current->Accel_y - s_cal.last_accel.Accel_y);
    s_cal.diag[MPU_CAL_DIAG_DZ] = (uint32_t)abs(current->Accel_z - s_cal.last_accel.Accel_z);
    if (s_cal.diag[MPU_CAL_DIAG_DX] >= (uint32_t)mpu_cal_still_lsb)
    {
        reason |= MPU_CAL_REJECT_DX;
    }
    if (s_cal.diag[MPU_CAL_DIAG_DY] >= (uint32_t)mpu_cal_still_lsb)
    {
        reason |= MPU_CAL_REJECT_DY;
    }
    if (s_cal.diag[MPU_CAL_DIAG_DZ] >= (uint32_t)mpu_cal_still_lsb)
    {
        reason |= MPU_CAL_REJECT_DZ;
    }
    if (abs(current->Accel_x) >= mpu_cal_level_lsb)
    {
        reason |= MPU_CAL_REJECT_AX;
    }
    if (abs(current->Accel_y) >= mpu_cal_level_lsb)
    {
        reason |= MPU_CAL_REJECT_AY;
    }
    if (abs(current->Accel_z - s_cal.gravity_z_lsb) >= mpu_cal_level_lsb)
    {
        reason |= MPU_CAL_REJECT_AZ;
    }
    s_cal.diag[MPU_CAL_DIAG_REJECT] = reason;
    s_cal.diag[MPU_CAL_DIAG_REJECT_SEEN] |= reason;
    ++s_cal.diag[MPU_CAL_DIAG_CHECKED];
    if (reason == 0)
    {
        ++s_cal.diag[MPU_CAL_DIAG_ACCEPTED];
    }
    else
    {
        ++s_cal.diag[MPU_CAL_DIAG_REJECTED];
    }
    return reason == 0;
}

mpu6050_cal_state my_mpu6050_zero_offset_step(Accel_Gyro_struct *raw, uint8_t valid, uint32_t now_ms)
{
    uint32_t timeout_ms;
    uint8_t still;
    if (s_cal.state != MPU6050_CAL_WAIT_STILL && s_cal.state != MPU6050_CAL_AVERAGING)
    {
        return s_cal.state;
    }
    timeout_ms = 3000U;
    if (s_cal.state == MPU6050_CAL_AVERAGING)
    {
        timeout_ms = 2000U;
    }
    if ((uint32_t)(now_ms - s_cal.phase_ms) >= timeout_ms)
    {
        if (s_cal.state == MPU6050_CAL_WAIT_STILL)
        {
            s_cal.diag[MPU_CAL_DIAG_REJECT] = MPU_CAL_REJECT_WAIT_TIMEOUT;
        }
        else
        {
            s_cal.diag[MPU_CAL_DIAG_REJECT] = MPU_CAL_REJECT_AVERAGE_TIMEOUT;
        }
        s_cal.diag[MPU_CAL_DIAG_REJECT_SEEN] |= s_cal.diag[MPU_CAL_DIAG_REJECT];
        s_cal.state = MPU6050_CAL_FAILED;
        return s_cal.state;
    }
    if (valid == 0 || raw == NULL)
    {
        /* 丢帧不计入有效样本，原墙钟窗口仍继续计时，不覆盖已生效零偏。 */
        s_cal.diag[MPU_CAL_DIAG_REJECT] = MPU_CAL_REJECT_READ;
        s_cal.diag[MPU_CAL_DIAG_REJECT_SEEN] |= MPU_CAL_REJECT_READ;
        return s_cal.state;
    }
    if (s_cal.have_sample == 0)
    {
        s_cal.gravity_z_lsb = MPU6050_ACCEL_LSB_PER_G;
        if (raw->Accel.Accel_z < 0)
        {
            s_cal.gravity_z_lsb = -MPU6050_ACCEL_LSB_PER_G;
        }
        s_cal.diag[MPU_CAL_DIAG_AX] = (uint32_t)(int32_t)raw->Accel.Accel_x;
        s_cal.diag[MPU_CAL_DIAG_AY] = (uint32_t)(int32_t)raw->Accel.Accel_y;
        s_cal.diag[MPU_CAL_DIAG_AZ] = (uint32_t)(int32_t)raw->Accel.Accel_z;
        s_cal.last_accel = raw->Accel;
        s_cal.last_sample_ms = now_ms;
        s_cal.have_sample = 1;
        return s_cal.state;
    }
    if ((uint32_t)(now_ms - s_cal.last_sample_ms) < MPU6050_SAMPLE_PERIOD_MS)
    {
        return s_cal.state;
    }
    s_cal.diag[MPU_CAL_DIAG_INTERVAL_MS] = now_ms - s_cal.last_sample_ms;
    s_cal.last_sample_ms = now_ms;
    still = mpu6050_cal_still(&raw->Accel);
    s_cal.last_accel = raw->Accel;
    if (s_cal.state == MPU6050_CAL_WAIT_STILL)
    {
        if (still != 0)
        {
            ++s_cal.stable_count;
            if (s_cal.stable_count > s_cal.diag[MPU_CAL_DIAG_STABLE_PEAK])
            {
                s_cal.diag[MPU_CAL_DIAG_STABLE_PEAK] = s_cal.stable_count;
            }
        }
        else
        {
            s_cal.stable_count = 0;
        }
        if (s_cal.stable_count >= 100U)
        {
            s_cal.phase_ms = now_ms;
            s_cal.state = MPU6050_CAL_AVERAGING;
        }
        return s_cal.state;
    }
    if (still == 0)
    {
        s_cal.state = MPU6050_CAL_FAILED;
        return s_cal.state;
    }
    s_cal.sum[0] += raw->Accel.Accel_x;
    s_cal.sum[1] += raw->Accel.Accel_y;
    s_cal.sum[2] += raw->Accel.Accel_z - s_cal.gravity_z_lsb;
    s_cal.sum[3] += raw->Gyro.Gyro_x;
    s_cal.sum[4] += raw->Gyro.Gyro_y;
    s_cal.sum[5] += raw->Gyro.Gyro_z;
    if (++s_cal.average_count >= 100U)
    {
        /* 仍以旧偏移下的残差平均增量更新；只有整窗成功才一次提交。 */
        offset_accel_x += (int16_t)(s_cal.sum[0] / 100);
        offset_accel_y += (int16_t)(s_cal.sum[1] / 100);
        offset_accel_z += (int16_t)(s_cal.sum[2] / 100);
        offset_gyro_x += (int16_t)(s_cal.sum[3] / 100);
        offset_gyro_y += (int16_t)(s_cal.sum[4] / 100);
        offset_gyro_z += (int16_t)(s_cal.sum[5] / 100);
        raw->Accel.Accel_x -= (int16_t)(s_cal.sum[0] / 100);
        raw->Accel.Accel_y -= (int16_t)(s_cal.sum[1] / 100);
        raw->Accel.Accel_z -= (int16_t)(s_cal.sum[2] / 100);
        raw->Gyro.Gyro_x -= (int16_t)(s_cal.sum[3] / 100);
        raw->Gyro.Gyro_y -= (int16_t)(s_cal.sum[4] / 100);
        raw->Gyro.Gyro_z -= (int16_t)(s_cal.sum[5] / 100);
        s_cal.state = MPU6050_CAL_DONE;
    }
    return s_cal.state;
}

mpu6050_cal_state my_mpu6050_zero_offset_state(void)
{
    return s_cal.state;
}

void my_mpu6050_zero_offset_progress(uint16_t *stable, uint16_t *average)
{
    *stable = s_cal.stable_count;
    *average = s_cal.average_count;
}

/* 兼容旧调用方；手表启动只使用 begin/step，不调用此阻塞接口。 */
uint8_t my_mpu6050_zero_offset(void)
{
    Accel_Gyro_struct raw;
    uint8_t valid;
    mpu6050_cal_state state;
    my_mpu6050_zero_offset_begin(HAL_GetTick());
    do
    {
        valid = my_mpu6050_get_data(&raw) == 0;
        state = my_mpu6050_zero_offset_step(&raw, valid, HAL_GetTick());
        if (state == MPU6050_CAL_WAIT_STILL || state == MPU6050_CAL_AVERAGING)
        {
            vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_PERIOD_MS));
        }
    }
    while (state == MPU6050_CAL_WAIT_STILL || state == MPU6050_CAL_AVERAGING);
    if (state == MPU6050_CAL_DONE)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 静止时增量微调陀螺仪零偏 (见头文件注释)
 */
void my_mpu6050_gyro_trim(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z)
{
    offset_gyro_x += gyro_x;
    offset_gyro_y += gyro_y;
    offset_gyro_z += gyro_z;
}

/* ---------------------------------------------------------------- 数据读取 */
/**
 * @brief 读取三轴角速度 (已减零偏)
 * @return 0=成功, 1=总线失败(数据未更新)
 */
uint8_t my_mpu6050_get_gyro(Gyro_struct *gyro)
{
    uint8_t buf[6];

    if (gyro == NULL)
    {
        return 1;
    }

    /* 一次突发读 6 字节 (0x43~0x48) */
    if (mpu6050_bus_read(MPU6050_GYRO_XOUT_H, buf, 6) != 0)
    {
        return 1;
    }

    gyro->Gyro_x = (int16_t)(((buf[0] << 8) | buf[1]) - offset_gyro_x);
    gyro->Gyro_y = (int16_t)(((buf[2] << 8) | buf[3]) - offset_gyro_y);
    gyro->Gyro_z = (int16_t)(((buf[4] << 8) | buf[5]) - offset_gyro_z);
    return 0;
}

/**
 * @brief 读取三轴加速度 (已减零偏；抖动较严重，必要时上层再滤波)
 * @return 0=成功, 1=总线失败(数据未更新)
 */
uint8_t my_mpu6050_get_accel(Accel_struct *accel)
{
    uint8_t buf[6];

    if (accel == NULL)
    {
        return 1;
    }

    /* 一次突发读 6 字节 (0x3B~0x40) */
    if (mpu6050_bus_read(MPU6050_ACCEL_XOUT_H, buf, 6) != 0)
    {
        return 1;
    }

    accel->Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - offset_accel_x);
    accel->Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - offset_accel_y);
    accel->Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - offset_accel_z);
    return 0;
}

/**
 * @brief 一次突发读取15字节：INT_STATUS + 加速度6 + 温度2 + 陀螺仪6 (0x3A~0x48)
 * @return 0=新帧, 1=通信/配置失败, 2=未就绪(输出不变)
 */
uint8_t my_mpu6050_get_data(Accel_Gyro_struct *data)
{
    uint8_t packet[15];
    const uint8_t *buf = packet + 1;

    /* 初始化失败时，地址仍ACK也不能把配置不完整的输出发布为有效帧。 */
    if (data == NULL || mpu6050_ready == 0)
    {
        return 1;
    }

    /* 0x3A..0x48 同一次事务：先读状态释放 INT 锁存，再取完整六轴帧。 */
    if (mpu6050_bus_read(MPU6050_INT_STATUS, packet, 15) != 0)
    {
        return 1;
    }

    mpu6050_int_status = packet[0];
    if ((packet[0] & 0x01U) == 0)
    {
        return MPU6050_DATA_NOT_READY;
    }
    data->Accel.Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - offset_accel_x);
    data->Accel.Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - offset_accel_y);
    data->Accel.Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - offset_accel_z);
    data->Gyro.Gyro_x = (int16_t)(((buf[8] << 8) | buf[9]) - offset_gyro_x);
    data->Gyro.Gyro_y = (int16_t)(((buf[10] << 8) | buf[11]) - offset_gyro_y);
    data->Gyro.Gyro_z = (int16_t)(((buf[12] << 8) | buf[13]) - offset_gyro_z);
    return 0;
}

/* ------------------------------------------------------------ 数据就绪中断 */
/* HAL 已清 STM32 EXTI PR；这里只记事件，绝不抢 SENSOR 总线。
 * 器件 INT 锁存在 sensor 任务 my_mpu6050_get_data() 里释放。
 * 锁存期间多次 DATA_RDY 会合并，边沿计数不是器件采样率。 */

/**
 * @brief HAL GPIO EXTI 回调 (强符号覆盖 hal_gpio.c 中的 __weak 版本)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INT_Pin)
    {
        mpu6050_int_edges++;
        mpu6050_int_flag = 1;
    }
}

/**
 * @brief ISR 边沿计数快照 (uint16, 计频用相邻两次快照做无符号差分)
 */
uint16_t my_mpu6050_int_count(void)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t cnt;

    __disable_irq();
    cnt = mpu6050_int_edges;
    __set_PRIMASK(primask);
    return cnt;
}

/**
 * @brief 查询并清除"数据就绪"标志 (任务上下文调用)
 * @return 1=期间发生过中断(已清除), 0=无
 * @note  读取+清零做成原子, 防止恰好落在中间的 ISR 置位被抹掉
 */
uint8_t my_mpu6050_int_check_clear(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t flag;

    __disable_irq();
    flag = mpu6050_int_flag;
    mpu6050_int_flag = 0;
    __set_PRIMASK(primask);
    return flag;
}

/**
 * @brief 取 sensor 任务读回的 INT_STATUS(0x3A) 快照 (bit0=DATA_RDY, bit1=FIFO_OFLOW, *        bit2=I2C_MST_INT, bit7=DMP_INT)
 */
uint8_t my_mpu6050_get_int_status(void)
{
    return mpu6050_int_status;
}

/**
 * @brief 读取片内温度传感器 (0x41/0x42 突发读)
 * @param temp_01c 输出温度, 单位 0.01℃ (换算: T = raw/340 + 36.53)
 * @return 0=成功, 1=总线失败
 */
uint8_t my_mpu6050_get_temp(int16_t *temp_01c)
{
    uint8_t buf[2];
    int32_t raw;

    if (temp_01c == NULL)
    {
        return 1;
    }

    if (mpu6050_bus_read(MPU6050_TEMP_OUT_H, buf, 2) != 0)
    {
        return 1;
    }

    raw = (int16_t)((buf[0] << 8) | buf[1]);
    /* 先转 0.01℃ 定点 (raw*100/340 + 3653), 避免浮点运算 */
    *temp_01c = (int16_t)(raw * 100 / 340 + 3653);
    return 0;
}
