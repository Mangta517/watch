/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 驱动实现 —— 软件 I2C (common/my_i2c.c, SENSOR 总线)
 *          移植自旧工程 flight_hal/interface/my_mpu6050.c：
 *          寄存器配置与校准逻辑保持一致；HAL 硬件 I2C 调用替换为 my_i2c 封装。
 *          数据就绪中断: 器件 INT_PIN_CFG=0x20 (高有效/推挽/锁存, 读 0x3A 清除),
 *          INT_ENABLE=0x01 (DATA_RDY)；PB12 EXTI 上升沿，ISR 读 0x3A 释放锁存。
 ******************************************************************************
 */
#include "mpu6050.h"
#include "my_i2c.h"

#include "FreeRTOS.h"
#include "task.h"       /* vTaskDelay: 校准循环中的任务级延时 */
#include <stdlib.h>    /* abs */

/* MPU6050 挂在软件 I2C 的传感器总线上 */
#define MPU6050_I2C_BUS   MY_I2C_BUS_SENSOR

Gyro_struct  gyro_data  = {0};
Accel_struct accel_data = {0};

int16_t offset_accel_x;
int16_t offset_accel_y;
int16_t offset_accel_z;

int16_t offset_gyro_x;
int16_t offset_gyro_y;
int16_t offset_gyro_z;

/* ---------------------------------------------------------- 总线独占保护 */
/* EXTI ISR (HAL_GPIO_EXTI_Callback) 也会走 SENSOR 总线读 0x3A。若中断恰好
 * 落在任务层一次突发读写的中间, 两边时序都会被打断。因此器件的所有总线
 * 事务用 PRIMASK 临界区包住 (保存-恢复式, 任务/ISR 上下文均可安全调用):
 * 单次事务软件 I2C @~200kHz 最长约 2ms, 仅推迟同优先级以下的中断响应。 */
static uint8_t mpu6050_bus_write(uint8_t reg, const uint8_t *dat, uint16_t len)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t  ret;

    __disable_irq();
    ret = my_i2c_write_bytes(MPU6050_I2C_BUS, MPU6050_ADDR, reg, dat, len);
    __set_PRIMASK(primask);
    return ret;
}

static uint8_t mpu6050_bus_read(uint8_t reg, uint8_t *dat, uint16_t len)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t  ret;

    __disable_irq();
    ret = my_i2c_read_bytes(MPU6050_I2C_BUS, MPU6050_ADDR, reg, dat, len);
    __set_PRIMASK(primask);
    return ret;
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
    uint8_t  data = 0;
    uint8_t  id   = 0;
    uint8_t  retry;
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
            break;                       /* 器件已在总线上应答 */
        }
        if (++retry >= 20)
        {
            return 1;                   /* 1s 无应答: 未接/供电异常 */
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
        return 1;   /* 等待复位完成超时 */
    }

    /* 唤醒 MPU6050 */
    my_mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    /* 配置陀螺仪角速度量程 ±2000°/s */
    my_mpu6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_RANGE_2000DPS);
    /* 配置加速度计量程 ±2g */
    my_mpu6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_2G);
    /* 中断引脚配置 (0x37) = 0x20: INT 高有效、推挽输出、中断锁存,
     * 仅读 INT_STATUS(0x3A) 后清除锁存释放 INT —— 配合 PB12 上升沿 EXTI */
    my_mpu6050_write_reg(MPU6050_INT_PIN_CFG, 0x20);
    /* 使能数据就绪中断 (0x38 bit0 DATA_RDY): 采样数据更新时 INT 拉高 */
    my_mpu6050_write_reg(MPU6050_INT_ENABLE, 0x01);
    /* 用户控制寄存器：不用 FIFO 和外设 I2C 主机 */
    my_mpu6050_write_reg(MPU6050_USER_CTRL, 0x00);
    /* 采样率 = 1kHz / (1 + 1) = 500Hz */
    my_mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x01);
    /* DLPF 低通滤波：陀螺仪 184Hz / 加速度计 188Hz */
    my_mpu6050_write_reg(MPU6050_CONFIG, 0x01);
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
 * 静止判定: 两次加速度值相减小于 300，连续 100 次
 * 水平判定: |Accel_x|<200 且 |Accel_y|<200 且 |Accel_z-16384|<200 (约±0.7°)
 *           (原阈值 800≈±2.8° 太宽，1~2° 微坡会被当水平基准，导致起飞后倾斜前漂)
 * 超时保护: 约 3s 仍未达标则放弃本次校准，保留原偏移量，避免任务被永久阻塞
 *
 * @return 0=校准完成, 1=超时放弃
 */
uint8_t my_mpu6050_zero_offset(void)
{
    Accel_struct current_accel = {0};
    Accel_struct last_accel = {0};
    uint8_t  count = 0;
    uint32_t attempts = 0;
    const uint32_t CALIB_TIMEOUT = 500;   /* 500 次 × ~6ms ≈ 3s */

    my_mpu6050_get_accel(&last_accel);
    while (count < 100)
    {
        my_mpu6050_get_accel(&current_accel);
        /* 如果误差判断的值过小，则无法进行零偏校准 */
        if (abs(current_accel.Accel_x - last_accel.Accel_x) < 300 &&
            abs(current_accel.Accel_y - last_accel.Accel_y) < 300 &&
            abs(current_accel.Accel_z - last_accel.Accel_z) < 300 &&
            abs(current_accel.Accel_x) < 200 &&
            abs(current_accel.Accel_y) < 200 &&
            abs(current_accel.Accel_z - 16384) < 200)
        {
            count++;
        }
        else
        {
            count = 0;
        }
        last_accel = current_accel;
        vTaskDelay(6);
        if (++attempts >= CALIB_TIMEOUT)
        {
            /* TODO: 旧工程此处 debug_printf("MPU6050校准失败: 请将设备放平保持静止...")，
             *       当前工程串口调试模块未移植，失败状态由返回值携带 */
            return 1;   /* 放弃本次校准，偏移量保持原值 */
        }
    }

    {
        Accel_Gyro_struct accel_gyro_data = {0};
        int32_t offset_sum_x     = 0;
        int32_t offset_sum_y     = 0;
        int32_t offset_sum_z     = 0;
        int32_t offset_sum_x_g   = 0;
        int32_t offset_sum_y_g   = 0;
        int32_t offset_sum_z_g   = 0;

        for (uint8_t i = 0; i < 100; i++)
        {
            /* 重新读取加速度 */
            my_mpu6050_get_accel(&accel_gyro_data.Accel);
            offset_sum_x += accel_gyro_data.Accel.Accel_x;
            offset_sum_y += accel_gyro_data.Accel.Accel_y;
            /* Z 轴原本有 1G 重力，减去 16384 即为重力分量 */
            offset_sum_z += (accel_gyro_data.Accel.Accel_z - 16384);

            my_mpu6050_get_gyro(&accel_gyro_data.Gyro);
            offset_sum_x_g += accel_gyro_data.Gyro.Gyro_x;
            offset_sum_y_g += accel_gyro_data.Gyro.Gyro_y;
            offset_sum_z_g += accel_gyro_data.Gyro.Gyro_z;
        }

        offset_accel_x = (int16_t)(offset_sum_x / 100);
        offset_accel_y = (int16_t)(offset_sum_y / 100);
        offset_accel_z = (int16_t)(offset_sum_z / 100);

        offset_gyro_x  = (int16_t)(offset_sum_x_g / 100);
        offset_gyro_y  = (int16_t)(offset_sum_y_g / 100);
        offset_gyro_z  = (int16_t)(offset_sum_z_g / 100);
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
 * @brief 一次突发读取 14 字节: 加速度 6 + 温度 2 + 陀螺仪 6 (0x3B~0x48)
 * @return 0=成功, 1=总线失败(数据未更新)
 */
uint8_t my_mpu6050_get_data(Accel_Gyro_struct *data)
{
    uint8_t buf[14];

    if (data == NULL)
    {
        return 1;
    }

    if (mpu6050_bus_read(MPU6050_ACCEL_XOUT_H, buf, 14) != 0)
    {
        return 1;
    }

    data->Accel.Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - offset_accel_x);
    data->Accel.Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - offset_accel_y);
    data->Accel.Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - offset_accel_z);
    data->Gyro.Gyro_x   = (int16_t)(((buf[8] << 8) | buf[9])  - offset_gyro_x);
    data->Gyro.Gyro_y   = (int16_t)(((buf[10] << 8) | buf[11]) - offset_gyro_y);
    data->Gyro.Gyro_z   = (int16_t)(((buf[12] << 8) | buf[13]) - offset_gyro_z);
    return 0;
}

/* ------------------------------------------------------------ 数据就绪中断 */
/* PB12 (MPU6050_INT) EXTI 上升沿 -> HAL_GPIO_EXTI_IRQHandler -> 本回调。
 * 两级标志都要清:
 *   1) STM32 侧 EXTI PR: HAL_GPIO_EXTI_IRQHandler 内部已写 PR 清除;
 *   2) 器件侧锁存: 读 INT_STATUS(0x3A) 释放 latch, INT 回到低电平,
 *      下一次 DATA_RDY 才能再次产生上升沿。
 * 约束: EXTI15_10 抢占优先级 0, 高于 FreeRTOS configMAX_SYSCALL(11),
 *       回调内只允许位翻 I2C + 置标志, 禁止调用任何 FromISR API;
 *       若将来要用信号量同步任务, 需在 CubeMX 把 EXTI 优先级降到 >=11。
 * 总线约束: ISR 与任务层的器件事务由 mpu6050_bus_read/write 的 PRIMASK
 *       临界区互斥, 不会互相打断 (SENSOR 总线上的第三方器件若共用,
 *       其 my_i2c 直调不受此保护, 目前该总线仅 MPU6050 使用)。 */
static volatile uint8_t  mpu6050_int_flag = 0;     /* 1=有新的数据就绪中断 */
static volatile uint8_t  mpu6050_int_status = 0;   /* ISR 读回的 0x3A 快照  */
static volatile uint16_t mpu6050_int_count = 0;    /* 边沿计数器: 标志只有 1 位,
                                                     * 高频中断会被合并, 精确计频
                                                     * 必须用计数差分 (见 UI 页) */

/**
 * @brief HAL GPIO EXTI 回调 (强符号覆盖 hal_gpio.c 中的 __weak 版本)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INT_Pin)
    {
        uint8_t status = 0;

        /* 读 0x3A: 获取中断源并释放器件锁存 (INT 回低, 允许下次上升沿)。
         * mpu6050_bus_read 是 PRIMASK 保存-恢复式, ISR 内调用只是空操作级别。 */
        if (mpu6050_bus_read(MPU6050_INT_STATUS, &status, 1) == 0)
        {
            mpu6050_int_status = status;
        }
        mpu6050_int_count++;     /* 每个真实边沿都计, 16bit 回绕由差分法天然吸收 */
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
    cnt = mpu6050_int_count;
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
    uint8_t  flag;

    __disable_irq();
    flag = mpu6050_int_flag;
    mpu6050_int_flag = 0;
    __set_PRIMASK(primask);
    return flag;
}

/**
 * @brief 取 ISR 读回的 INT_STATUS(0x3A) 快照 (bit0=DATA_RDY, bit1=FIFO_OFLOW,
 *        bit2=I2C_MST_INT, bit7=DMP_INT)
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
