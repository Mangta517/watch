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
#include "task.h"   /* vTaskDelay: 校准循环中的任务级延时 */
#include <stdlib.h> /* abs */

/* MPU6050 挂在软件 I2C 的传感器总线上 */
#define MPU6050_I2C_BUS MY_I2C_BUS_SENSOR

static volatile uint8_t mpu6050_int_flag;
static volatile uint8_t mpu6050_int_status; /* sensor 任务读回的状态 */
static volatile uint16_t mpu6050_int_edges;

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
 * @return 0=成功, 1=复位等待超时或总线无应答
 * @note  调用前需先执行 my_i2c_init(MY_I2C_BUS_SENSOR)
 */
uint8_t my_mpu6050_init(void)
{
    uint8_t data = 0;
    uint8_t id = 0;
    uint8_t retry;
    uint16_t guard;

    /* 上电就绪门: MPU6050 POR 后 I2C 接口需要 ~100ms 才能应答。
     * 本工程 init 在调度器启动前就被调用(复位后仅几 ms), 若不等就绪
     * 首写 NACK 直接返回, 器件将停留在默认 SLEEP 态 → 输出寄存器全 0、
     * INT 永远不触发, 表现为"陀螺仪没有数据"。
     * 以 WHO_AM_I 是否获得 ACK 为准 (不比较数值, 兼容 0x68/0x98/0x69 克隆片);
     * 调度器未启动, HAL_Delay 走 SysTick 1ms 可用。最多等 20×50ms = 1s。 */
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
        HAL_Delay(50);
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
    my_mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    /* 量程同时供公共层换算、1g判断和零偏校准引用。 */
    my_mpu6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_RANGE);
    my_mpu6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE);
    /* 中断引脚配置 (0x37) = 0x20: INT 高有效、推挽输出、中断锁存,
     * 仅读 INT_STATUS(0x3A) 后清除锁存释放 INT —— 配合 PB12 上升沿 EXTI */
    my_mpu6050_write_reg(MPU6050_INT_PIN_CFG, 0x20);
    /* 使能数据就绪中断 (0x38 bit0 DATA_RDY): 采样数据更新时 INT 拉高 */
    my_mpu6050_write_reg(MPU6050_INT_ENABLE, 0x01);
    /* 用户控制寄存器：不用 FIFO 和外设 I2C 主机 */
    my_mpu6050_write_reg(MPU6050_USER_CTRL, 0x00);
    /* 100Hz输出与sensor任务同步；42/44Hz低通保留转腕响应并抑制高频噪声。 */
    my_mpu6050_write_reg(MPU6050_SMPLRT_DIV, MPU6050_SAMPLE_PERIOD_MS - 1U);
    my_mpu6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_CFG);
    /* 时钟源选内部 X 轴陀螺仪 */
    my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);
    /* 使能加速度计和陀螺仪 (PWR_MGMT_2 全部轴不关断) */
    my_mpu6050_write_reg(MPU6050_PWR_MGMT_2, 0x00);

    /* 配置过程中可能已产生锁存的 DATA_RDY: 读一次 0x3A 清除锁存，
     * 确保 INT 初始为低，之后每个数据就绪都产生干净的上升沿 */
    my_mpu6050_read_reg(MPU6050_INT_STATUS, &data);

    /* 注意: 上电初始化不做零偏校准(校准阻塞且要求静止)，
     * 改由应用任务在确认设备静止后调用 my_mpu6050_zero_offset() 触发 */
    return 0;
}

/* ---------------------------------------------------------------- 零偏校准 */
/**
 * @brief 静止水平状态下的零偏校准 (阻塞，须在任务上下文调用)
 *
 * 静止判定: 相邻帧加速度变化小于0.0183g，连续100帧；水平容差约±0.7°。
 * 门限按当前1g的LSB计算，量程变化不改变物理容差。
 * 等待稳定最多3s，之后按100Hz采集100帧平均约1s；失败保留原偏移。
 *
 * @return 0=校准完成, 1=超时放弃
 */
uint8_t my_mpu6050_zero_offset(void)
{
    Accel_struct current_accel = {0};
    Accel_struct last_accel = {0};
    uint8_t count = 0;
    uint32_t attempts = 0;
    const uint32_t CALIB_TIMEOUT = 3000U / MPU6050_SAMPLE_PERIOD_MS;
    const int32_t still_lsb = (int32_t)(0.0183f * MPU6050_ACCEL_LSB_PER_G);
    const int32_t level_lsb = (int32_t)(0.0122f * MPU6050_ACCEL_LSB_PER_G);

    if (my_mpu6050_get_accel(&last_accel) != 0)
    {
        return 1;
    }
    while (count < 100)
    {
        vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_PERIOD_MS));
        if (my_mpu6050_get_accel(&current_accel) != 0)
        {
            return 1;
        }
        /* 如果误差判断的值过小，则无法进行零偏校准 */
        if (abs(current_accel.Accel_x - last_accel.Accel_x) < still_lsb && abs(current_accel.Accel_y - last_accel.Accel_y) < still_lsb &&
            abs(current_accel.Accel_z - last_accel.Accel_z) < still_lsb && abs(current_accel.Accel_x) < level_lsb &&
            abs(current_accel.Accel_y) < level_lsb && abs(current_accel.Accel_z - MPU6050_ACCEL_LSB_PER_G) < level_lsb)
        {
            count++;
        }
        else
        {
            count = 0;
        }
        last_accel = current_accel;
        if (++attempts >= CALIB_TIMEOUT)
        {
            /* TODO: 旧工程此处 debug_printf("MPU6050校准失败: 请将设备放平保持静止...")，
             *       当前工程串口调试模块未移植，失败状态由返回值携带 */
            return 1; /* 放弃本次校准，偏移量保持原值 */
        }
    }

    {
        Accel_Gyro_struct accel_gyro_data = {0};
        int32_t offset_sum_x = 0;
        int32_t offset_sum_y = 0;
        int32_t offset_sum_z = 0;
        int32_t offset_sum_x_g = 0;
        int32_t offset_sum_y_g = 0;
        int32_t offset_sum_z_g = 0;

        for (uint8_t i = 0; i < 100; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(MPU6050_SAMPLE_PERIOD_MS));
            /* 一帧同时取加速度和陀螺仪，避免100Hz输出下反复平均同一帧。 */
            if (my_mpu6050_get_data(&accel_gyro_data) != 0)
            {
                return 1;
            }
            /* 平均阶段也保持静止水平；运动时放弃，不能把动作写成零偏。 */
            current_accel = accel_gyro_data.Accel;
            if (abs(current_accel.Accel_x - last_accel.Accel_x) >= still_lsb ||
                abs(current_accel.Accel_y - last_accel.Accel_y) >= still_lsb ||
                abs(current_accel.Accel_z - last_accel.Accel_z) >= still_lsb || abs(current_accel.Accel_x) >= level_lsb ||
                abs(current_accel.Accel_y) >= level_lsb || abs(current_accel.Accel_z - MPU6050_ACCEL_LSB_PER_G) >= level_lsb)
            {
                return 1;
            }
            last_accel = current_accel;
            offset_sum_x += accel_gyro_data.Accel.Accel_x;
            offset_sum_y += accel_gyro_data.Accel.Accel_y;
            /* Z轴保留1g，只有其残差计入零偏。 */
            offset_sum_z += (accel_gyro_data.Accel.Accel_z - MPU6050_ACCEL_LSB_PER_G);
            offset_sum_x_g += accel_gyro_data.Gyro.Gyro_x;
            offset_sum_y_g += accel_gyro_data.Gyro.Gyro_y;
            offset_sum_z_g += accel_gyro_data.Gyro.Gyro_z;
        }

        /* get_data已减去旧零偏；平均值是残差，重校准时应累加而非覆盖。 */
        offset_accel_x += (int16_t)(offset_sum_x / 100);
        offset_accel_y += (int16_t)(offset_sum_y / 100);
        offset_accel_z += (int16_t)(offset_sum_z / 100);

        offset_gyro_x += (int16_t)(offset_sum_x_g / 100);
        offset_gyro_y += (int16_t)(offset_sum_y_g / 100);
        offset_gyro_z += (int16_t)(offset_sum_z_g / 100);
    }

    return 0;
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
 * @return 0=成功, 1=总线失败(数据未更新)
 */
uint8_t my_mpu6050_get_data(Accel_Gyro_struct *data)
{
    uint8_t packet[15];
    const uint8_t *buf = packet + 1;

    if (data == NULL)
    {
        return 1;
    }

    /* 0x3A..0x48 同一次事务：先读状态释放 INT 锁存，再取完整六轴帧。 */
    if (mpu6050_bus_read(MPU6050_INT_STATUS, packet, 15) != 0)
    {
        return 1;
    }

    mpu6050_int_status = packet[0];
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
