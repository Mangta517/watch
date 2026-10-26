/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 驱动实现 —— 软件 I2C (common/my_i2c.c, SENSOR 总线)
 *          移植自旧工程 flight_hal/interface/my_mpu6050.c：
 *          手表±1000°/s、±4g；初始化100Hz，sensor按使用状态切换50/25Hz六轴或5Hz加速度低功耗。
 *          数据就绪中断: 器件 INT_PIN_CFG=0x20 (高有效/推挽/锁存, 读 0x3A 清除),
 *          INT_ENABLE=0x01 (DATA_RDY)；PB12 EXTI只记事件，任务轮询最新寄存器。
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

static uint32_t mpu_sample_ms; /* 最近有效帧的读取开始时刻，非硬件采样时间戳。 */

static void mpu6050_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免tick边界缩短硬件最小等待；等待期间让出CPU。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1);
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
static uint32_t s_mpu_period_ms = MPU6050_SAMPLE_PERIOD_MS;

/* 零偏由sensor任务的校准/恢复接口维护；读取方通过get_offset取得副本。 */
static int16_t offset_accel_x;
static int16_t offset_accel_y;
static int16_t offset_accel_z;

static int16_t offset_gyro_x;
static int16_t offset_gyro_y;
static int16_t offset_gyro_z;

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
/* 0=成功，1=失败/冷却，2=推进中；每次step至多一笔I2C事务。 */
enum
{
    MPU_RECOVER_PROBE,
    MPU_RECOVER_RESET,
    MPU_RECOVER_WAIT_RESET,
    MPU_RECOVER_CONFIG,
    MPU_RECOVER_CLEAR,
    MPU_RECOVER_FAILED,
    MPU_RECOVER_DONE
};
static struct
{
    uint32_t phase_ms;
    uint32_t fault_ms;
    uint32_t sample_ms;
    uint32_t sleep_ms;
    uint8_t phase, index, probes, faults, no_data, sleeping;
} mpu_recovery;

void my_mpu6050_recover_begin(void)
{
    mpu6050_ready = 0;
    s_mpu_period_ms = MPU6050_SAMPLE_PERIOD_MS;
    mpu_recovery.no_data = 0;
    mpu_recovery.sleeping = 0;
    mpu_recovery.phase = MPU_RECOVER_PROBE;
    mpu_recovery.index = mpu_recovery.probes = mpu_recovery.faults = 0;
    mpu_recovery.phase_ms = HAL_GetTick();
}

static uint8_t mpu6050_recover_failed(void)
{
    mpu6050_ready = 0;
    mpu_recovery.phase = MPU_RECOVER_FAILED;
    mpu_recovery.phase_ms = HAL_GetTick();
    return 1;
}

static void mpu6050_read_fault(void)
{
    if (mpu_recovery.faults < 3 && ++mpu_recovery.faults == 3)
    {
        mpu_recovery.fault_ms = HAL_GetTick();
    }
}

uint8_t my_mpu6050_recover_step(uint32_t now_ms)
{
    static const uint8_t config[][2] = {
        {MPU6050_REG_PWR_MGMT_1, 0x00},
        {MPU6050_GYRO_CONFIG, MPU6050_GYRO_RANGE},
        {MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE},
        {MPU6050_INT_PIN_CFG, 0x20},
        {MPU6050_INT_ENABLE, 0x01},
        {MPU6050_FIFO_EN, 0x00},
        {MPU6050_USER_CTRL, 0x00},
        {MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_PERIOD_MS - 1},
        {MPU6050_CONFIG, MPU6050_DLPF_CFG},
        {MPU6050_PWR_MGMT_1, 0x01},
        {MPU6050_PWR_MGMT_2, 0x00}};
    uint8_t data;
    if (mpu6050_ready != 0)
    {
        /* 连续1s无新帧也重配，覆盖器件复位后仍ACK但停采样的情况。 */
        if ((mpu_recovery.no_data != 0 && (uint32_t)(now_ms - mpu_recovery.sample_ms) >= 1000) ||
            (mpu_recovery.faults >= 3 && (uint32_t)(now_ms - mpu_recovery.fault_ms) >= 1000))
        {
            my_mpu6050_recover_begin();
            return 2;
        }
        return 0;
    }
    switch (mpu_recovery.phase)
    {
    case MPU_RECOVER_FAILED:
        if ((uint32_t)(now_ms - mpu_recovery.phase_ms) >= 1000)
        {
            my_mpu6050_recover_begin();
            return 2;
        }
        return 1;
    case MPU_RECOVER_PROBE:
        if (mpu_recovery.probes != 0 && (uint32_t)(now_ms - mpu_recovery.phase_ms) < 50)
        {
            return 2;
        }
        if (my_mpu6050_read_reg(MPU6050_WHO_AM_I, &data) == 0)
        {
            mpu_recovery.phase = MPU_RECOVER_RESET;
        }
        else if (++mpu_recovery.probes >= 20)
        {
            return mpu6050_recover_failed();
        }
        mpu_recovery.phase_ms = HAL_GetTick();
        return 2;
    case MPU_RECOVER_RESET:
        if (my_mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80) != 0)
        {
            return mpu6050_recover_failed();
        }
        mpu_recovery.phase = MPU_RECOVER_WAIT_RESET;
        mpu_recovery.phase_ms = HAL_GetTick();
        return 2;
    case MPU_RECOVER_WAIT_RESET:
        if ((uint32_t)(now_ms - mpu_recovery.phase_ms) >= 1000)
        {
            return mpu6050_recover_failed();
        }
        /* 复位期间NACK/未回到0x40均等下一拍，不在本拍忙等。 */
        if (my_mpu6050_read_reg(MPU6050_REG_PWR_MGMT_1, &data) == 0 && data == 0x40)
        {
            mpu_recovery.phase = MPU_RECOVER_CONFIG;
        }
        return 2;
    case MPU_RECOVER_CONFIG:
        if (my_mpu6050_write_reg(config[mpu_recovery.index][0], config[mpu_recovery.index][1]) != 0)
        {
            return mpu6050_recover_failed();
        }
        if (++mpu_recovery.index >= sizeof(config) / sizeof(config[0]))
        {
            mpu_recovery.phase = MPU_RECOVER_CLEAR;
        }
        return 2;
    case MPU_RECOVER_CLEAR:
        if (my_mpu6050_read_reg(MPU6050_INT_STATUS, &data) != 0)
        {
            return mpu6050_recover_failed();
        }
        mpu_recovery.phase = MPU_RECOVER_DONE;
        mpu_recovery.sample_ms = HAL_GetTick();
        mpu6050_ready = 1;
        return 0;
    default:
        return 1;
    }
}

/* 仅保留启动/旧调用方的阻塞入口；运行期只调用recover_step。 */
uint8_t my_mpu6050_init(void)
{
    uint8_t result;
    my_mpu6050_recover_begin();
    while ((result = my_mpu6050_recover_step(HAL_GetTick())) == 2)
    {
        if (mpu_recovery.phase == MPU_RECOVER_PROBE)
        {
            mpu6050_delay_ms(50);
        }
        else if (mpu_recovery.phase == MPU_RECOVER_WAIT_RESET)
        {
            mpu6050_delay_ms(1);
        }
    }
    return result;
}

uint8_t my_mpu6050_set_sleep(uint8_t sleeping)
{
    uint8_t config = 0x01;
    uint32_t now_ms, elapsed_ms;
    if (sleeping != 0)
    {
        config = 0x41; /* 保留PLL时钟选择，置SLEEP，不复位零偏及采样配置。 */
    }
    if (my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, config) != 0)
    {
        my_mpu6050_recover_begin();
        return 1;
    }
    now_ms = HAL_GetTick();
    if (sleeping != 0 && mpu_recovery.sleeping == 0)
    {
        mpu_recovery.sleep_ms = now_ms;
        mpu_recovery.sleeping = 1;
    }
    else if (sleeping == 0 && mpu_recovery.sleeping != 0)
    {
        /* 正常睡眠不计入故障超时；保留各次唤醒采样失败的累计，直到真正读到新帧。 */
        elapsed_ms = now_ms - mpu_recovery.sleep_ms;
        mpu_recovery.sample_ms += elapsed_ms;
        if (mpu_recovery.faults >= 3)
        {
            mpu_recovery.fault_ms += elapsed_ms;
        }
        mpu_recovery.sleeping = 0;
    }
    return 0;
}

uint8_t my_mpu6050_set_sample_period(uint32_t period_ms)
{
    uint8_t filter;
    if (period_ms != 10 && period_ms != 20 && period_ms != 40 && period_ms != 200)
    {
        return 1;
    }
    if (mpu6050_ready == 0)
    {
        return 1;
    }
    if (period_ms == s_mpu_period_ms)
    {
        return 0;
    }
    if (period_ms == 200 || s_mpu_period_ms == 200)
    {
        /* 先退出循环并使用内部时钟，再启停陀螺仪，最后接通目标模式。 */
        uint8_t power_2 = 0x00;
        uint8_t power_1 = 0x01;
        if (period_ms == 200)
        {
            power_2 = 0x47; /* LP_WAKE_CTRL=1(5Hz)，三轴陀螺仪待机，加速度保持启用。 */
            power_1 = 0x28; /* CYCLE=1、SLEEP=0、TEMP_DIS=1，内部8MHz时钟。 */
        }
        if (my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x08) != 0 ||
            my_mpu6050_write_reg(MPU6050_PWR_MGMT_2, power_2) != 0 ||
            my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, power_1) != 0)
        {
            my_mpu6050_recover_begin();
            return 1;
        }
        mpu_recovery.sample_ms = HAL_GetTick();
        mpu_recovery.no_data = 0;
        if (period_ms == 200)
        {
            s_mpu_period_ms = period_ms;
            return 0;
        }
    }
    filter = 4; /* 50Hz采样：陀螺仪20Hz、加速度21Hz带宽。 */
    if (period_ms == 10)
    {
        filter = MPU6050_DLPF_CFG; /* 运动模式恢复100Hz/42Hz带宽。 */
    }
    else if (period_ms == 40)
    {
        filter = 5; /* 25Hz采样：10Hz带宽。 */
    }
    if (my_mpu6050_write_reg(MPU6050_CONFIG, filter) != 0 ||
        my_mpu6050_write_reg(MPU6050_SMPLRT_DIV, (uint8_t)(period_ms - 1)) != 0)
    {
        my_mpu6050_recover_begin();
        return 1;
    }
    s_mpu_period_ms = period_ms;
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
    uint32_t started_ms;
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
    s_cal.started_ms = now_ms;
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

static mpu6050_cal_state mpu6050_zero_offset_update(Accel_Gyro_struct *raw, uint8_t valid, uint32_t now_ms, uint32_t wall_ms)
{
    uint32_t timeout_ms;
    uint8_t still;
    if (s_cal.state != MPU6050_CAL_WAIT_STILL && s_cal.state != MPU6050_CAL_AVERAGING)
    {
        return s_cal.state;
    }
    timeout_ms = 3000 * s_mpu_period_ms / MPU6050_SAMPLE_PERIOD_MS;
    if (s_cal.state == MPU6050_CAL_AVERAGING)
    {
        timeout_ms = 2000 * s_mpu_period_ms / MPU6050_SAMPLE_PERIOD_MS;
    }
    if ((uint32_t)(wall_ms - s_cal.phase_ms) >= timeout_ms)
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
    /* 请求前取得的样本不累计；等待和平均阶段均按墙钟判断超时。 */
    if (valid != 0 && (int32_t)(now_ms - s_cal.started_ms) < 0)
    {
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
        if (s_cal.stable_count >= 100)
        {
            s_cal.phase_ms = wall_ms;
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
    if (++s_cal.average_count >= 100)
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

mpu6050_cal_state my_mpu6050_zero_offset_step(Accel_Gyro_struct *raw, uint8_t valid, uint32_t now_ms)
{
    return mpu6050_zero_offset_update(raw, valid, now_ms, now_ms);
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
            vTaskDelay(pdMS_TO_TICKS(s_mpu_period_ms));
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

/* 一笔读状态与六轴；5Hz循环只读0x3A..0x40加速度，无新帧不覆盖输出。 */
uint8_t my_mpu6050_get_data(Accel_Gyro_struct *data)
{
    uint8_t packet[15];
    const uint8_t *buf = packet + 1;
    uint32_t started_ms;
    uint16_t packet_size = sizeof(packet);

    if (s_mpu_period_ms == 200)
    {
        packet_size = 7; /* 低功耗只读状态和加速度，不读取停用的温度/陀螺仪寄存器。 */
    }
    if (data == NULL || mpu6050_ready == 0)
    {
        return 1;
    }
    started_ms = HAL_GetTick();
    if (mpu6050_bus_read(MPU6050_INT_STATUS, packet, packet_size) != 0)
    {
        mpu6050_read_fault();
        return 1;
    }
    mpu6050_int_status = packet[0];
    mpu_recovery.faults = 0;
    if ((packet[0] & 0x01) == 0)
    {
        mpu_recovery.no_data = 1;
        return MPU6050_DATA_NOT_READY;
    }

    /* 跳过0x41/0x42温度；完整读成功且DATA_RDY有效后才发布。 */
    data->Accel.Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - offset_accel_x);
    data->Accel.Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - offset_accel_y);
    data->Accel.Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - offset_accel_z);
    if (s_mpu_period_ms == 200)
    {
        memset(&data->Gyro, 0, sizeof(data->Gyro));
    }
    else
    {
        data->Gyro.Gyro_x = (int16_t)(((buf[8] << 8) | buf[9]) - offset_gyro_x);
        data->Gyro.Gyro_y = (int16_t)(((buf[10] << 8) | buf[11]) - offset_gyro_y);
        data->Gyro.Gyro_z = (int16_t)(((buf[12] << 8) | buf[13]) - offset_gyro_z);
    }
    mpu_sample_ms = started_ms;
    mpu_recovery.sample_ms = HAL_GetTick();
    mpu_recovery.no_data = 0;
    return 0;
}

uint32_t my_mpu6050_sample_ms(void)
{
    return mpu_sample_ms;
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
 * @brief 取任务读回的INT_STATUS快照：bit0=DATA_RDY、bit3=I2C_MST_INT、bit4=FIFO_OFLOW。
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
