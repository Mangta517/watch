/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   MPU6050 驱动 —— 走软件 I2C SENSOR 总线 (PB14=SCL, PB13=SDA)
 *          移植自旧工程 flight_hal/interface/my_mpu6050.c (硬件 I2C1 版)，
 *          函数名/寄存器配置/校准逻辑保持一致，便于旧上层代码直接复用。
 *
 * 中断说明：数据就绪中断已使能 —— 器件 INT_PIN_CFG(0x37)=0x20
 *          (高有效/推挽/锁存, 读 0x3A 清除), INT_ENABLE(0x38)=0x01(DATA_RDY)；
 *          INT 接 PB12, ISR 只记事件，sensor 任务读 0x3A 释放锁存。
 ******************************************************************************
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

/* ------------------------------------------------------------ 器件 I2C 地址 */
/* 7 位地址 (ADDR 接地)。软件 I2C 层内部自动拼接读写位，
 * 不再像旧工程硬件 I2C 那样使用 0xD0/0xD1 移位地址 */
#define MPU6050_ADDR 0x68

/* ------------------------------------------------------------ 寄存器映射 */
#define MPU6050_SMPLRT_DIV   0x19 /* 采样率分频寄存器       */
#define MPU6050_CONFIG       0x1A /* 配置寄存器 (DLPF)      */
#define MPU6050_GYRO_CONFIG  0x1B /* 陀螺仪配置寄存器       */
#define MPU6050_ACCEL_CONFIG 0x1C /* 加速度计配置寄存器     */
#define MPU6050_MOT_THR      0x1F /* 运动检测阈值           */
#define MPU6050_FIFO_EN      0x23 /* FIFO 使能              */
#define MPU6050_I2C_MST_CTRL 0x24 /* I2C 主机控制           */
#define MPU6050_INT_PIN_CFG  0x37 /* 中断引脚配置           */
#define MPU6050_INT_ENABLE   0x38 /* 中断使能               */
#define MPU6050_INT_STATUS   0x3A /* 中断状态               */

#define MPU6050_ACCEL_XOUT_H 0x3B /* 加速度计输出寄存器     */
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_TEMP_OUT_H 0x41 /* 温度输出寄存器         */
#define MPU6050_TEMP_OUT_L 0x42

#define MPU6050_GYRO_XOUT_H 0x43 /* 陀螺仪输出寄存器       */
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define MPU6050_SIGNAL_PATH_RESET 0x68 /* 信号路径复位           */
#define MPU6050_USER_CTRL         0x6A /* 用户控制               */
#define MPU6050_PWR_MGMT_1        0x6B /* 电源管理 1             */
#define MPU6050_PWR_MGMT_2        0x6C /* 电源管理 2             */

#define MPU6050_FIFO_COUNTH 0x72 /* FIFO 计数寄存器        */
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W    0x74 /* FIFO 读写寄存器        */
#define MPU6050_WHO_AM_I    0x75 /* 设备 ID 寄存器 (默认 0x68) */

/* 兼容旧代码的常用寄存器别名 */
#define MPU6050_REG_PWR_MGMT_1 MPU6050_PWR_MGMT_1

/* ------------------------------------------------- 常用配置宏（量程选择） */
/* 陀螺仪量程 */
#define MPU6050_GYRO_RANGE_250DPS  0x00
#define MPU6050_GYRO_RANGE_500DPS  0x08
#define MPU6050_GYRO_RANGE_1000DPS 0x10
#define MPU6050_GYRO_RANGE_2000DPS 0x18

/* 加速度计量程 */
#define MPU6050_ACCEL_RANGE_2G  0x00
#define MPU6050_ACCEL_RANGE_4G  0x08
#define MPU6050_ACCEL_RANGE_8G  0x10
#define MPU6050_ACCEL_RANGE_16G 0x18

/* 手表运动档：寄存器配置与解算/校准共同引用，改量程时须同步灵敏度。 */
#define MPU6050_GYRO_RANGE       MPU6050_GYRO_RANGE_1000DPS
#define MPU6050_ACCEL_RANGE      MPU6050_ACCEL_RANGE_4G
#define MPU6050_GYRO_LSB_PER_DPS 32.8f
#define MPU6050_ACCEL_LSB_PER_G  8192
#define MPU6050_SAMPLE_PERIOD_MS 10   /* DLPF启用时1kHz/(9+1)=100Hz */
#define MPU6050_DLPF_CFG         0x03 /* 陀螺仪42Hz、加速度44Hz */

    /* ------------------------------------------------------------ 数据结构 */
    typedef struct
    {
        int16_t Gyro_x; /* 向左飞为正，向右飞为负，表示横滚角 */
        int16_t Gyro_y; /* 向后飞为正，向前飞为负，表示俯仰角 */
        int16_t Gyro_z; /* 逆时针为正，顺时针为负，表示偏航角 */
    } Gyro_struct;

    typedef struct
    {
        int16_t Accel_x; /* 向前为负 */
        int16_t Accel_y; /* 向右为正 */
        int16_t Accel_z; /* 朝上飞加速度为正 */
    } Accel_struct;

    typedef struct
    {
        Gyro_struct Gyro;
        Accel_struct Accel;
    } Accel_Gyro_struct;

    /* ------------------------------------------------------------ 接口函数 */
    /* 返回值约定：0=成功，1=失败（总线 NACK/超时）                             */

    /* 初始化 MPU6050 (调用前需先 my_i2c_init(MY_I2C_BUS_SENSOR)) */
    uint8_t my_mpu6050_init(void);
    /* 仅sensor任务：10/20/40ms为六轴100/50/25Hz，200ms为加速度5Hz低功耗循环。 */
    uint8_t my_mpu6050_set_sample_period(uint32_t period_ms);
    /* accel_reference=1连续采加速度，陀螺仪/温度待机；200ms仍使用原5Hz循环。 */
    uint8_t my_mpu6050_set_sample_mode(uint32_t period_ms, uint8_t accel_reference);
    uint8_t my_mpu6050_set_sleep(uint8_t sleeping);
    /* 仅sensor任务：begin使数据无效，step每拍至多一笔事务；0=就绪，1=失败冷却，2=推进中。 */
    void my_mpu6050_recover_begin(void);
    uint8_t my_mpu6050_recover_step(uint32_t now_ms);

    typedef enum
    {
        MPU6050_CAL_IDLE,
        MPU6050_CAL_WAIT_STILL,
        MPU6050_CAL_AVERAGING,
        MPU6050_CAL_DONE,
        MPU6050_CAL_FAILED,
        MPU6050_CAL_RESTORED
    } mpu6050_cal_state;

    typedef struct
    {
        int16_t accel[3]; /* 原始LSB的零偏，Z轴不包含重力1g。 */
        int16_t gyro[3];
    } mpu6050_offset_struct;

    /* 仅sensor任务使用；恢复后不启动静止校准，原读取路径继续减去这些偏移。 */
    void my_mpu6050_get_offset(mpu6050_offset_struct *offset);
    void my_mpu6050_restore_offset(const mpu6050_offset_struct *offset);

    /* sensor任务用正常读取的同一帧逐步校准，不额外读I2C或等待。
     * 保留连续100帧静止判断和100帧平均；等待静止3s、平均阶段2s真实时间超时。 */
    void my_mpu6050_zero_offset_begin(uint32_t now_ms);
    mpu6050_cal_state my_mpu6050_zero_offset_step(Accel_Gyro_struct *raw, uint8_t valid, uint32_t now_ms);
    mpu6050_cal_state my_mpu6050_zero_offset_state(void);
    void my_mpu6050_zero_offset_progress(uint16_t *stable, uint16_t *average);
    enum
    {
        MPU_CAL_DIAG_AX,
        MPU_CAL_DIAG_AY,
        MPU_CAL_DIAG_AZ,
        MPU_CAL_DIAG_DX,
        MPU_CAL_DIAG_DY,
        MPU_CAL_DIAG_DZ,
        MPU_CAL_DIAG_LEVEL_LIMIT,
        MPU_CAL_DIAG_STILL_LIMIT,
        MPU_CAL_DIAG_REJECT,
        MPU_CAL_DIAG_REJECT_SEEN,
        MPU_CAL_DIAG_ACCEPTED,
        MPU_CAL_DIAG_REJECTED,
        MPU_CAL_DIAG_STABLE_PEAK,
        MPU_CAL_DIAG_INTERVAL_MS,
        MPU_CAL_DIAG_CHECKED,
        MPU_CAL_DIAG_STATE,
        MPU_CAL_DIAG_FIELD_COUNT
    };
    enum
    {
        MPU_CAL_REJECT_DX = 1,
        MPU_CAL_REJECT_DY = 2,
        MPU_CAL_REJECT_DZ = 4,
        MPU_CAL_REJECT_AX = 8,
        MPU_CAL_REJECT_AY = 16,
        MPU_CAL_REJECT_AZ = 32,
        MPU_CAL_REJECT_READ = 64,
        MPU_CAL_REJECT_WAIT_TIMEOUT = 128,
        MPU_CAL_REJECT_AVERAGE_TIMEOUT = 256
    };
    /* 仅sensor任务读取；固定16字段用于诊断type=6，不增加采样或改变校准判定。 */
    const uint32_t *my_mpu6050_zero_offset_diag(void);
    /* 兼容旧调用方的阻塞入口，返回0成功/1失败；手表启动不用此接口。 */
    uint8_t my_mpu6050_zero_offset(void);

    /* 静止时增量微调陀螺仪零偏：把当前角速度读数当作残余零偏累加进 offset，
 * 用于消除 yaw 积分零漂(偏航无绝对参考，只能靠陀螺仪零点准)。
 * 仅在确认静止时调用，每次只累加当前残余，多次收敛                        */
    void my_mpu6050_gyro_trim(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);

    uint8_t my_mpu6050_write_reg(uint8_t reg, uint8_t data);
    uint8_t my_mpu6050_read_reg(uint8_t reg, uint8_t *data);

    /* 读取三轴角速度(已减零偏) */
    uint8_t my_mpu6050_get_gyro(Gyro_struct *gyro);
    /* 读取三轴加速度(已减零偏，抖动较大，必要时上层再滤波) */
    uint8_t my_mpu6050_get_accel(Accel_struct *accel);
/* 0=新帧，1=通信/配置失败，2=未就绪；失败或无新帧不覆盖输出。 */
#define MPU6050_DATA_NOT_READY 2
    /* 常规轮询状态+六轴，低功耗只读加速度并将角速度置零；DATA_RDY有效才发布。 */
    uint8_t my_mpu6050_get_data(Accel_Gyro_struct *accel_gyro);
    uint32_t my_mpu6050_sample_ms(void); /* 最近有效帧的HAL读取开始时刻，非硬件时间戳。 */
    /* 读片内温度传感器, 输出 0.01℃ 定点 (T = raw/340 + 36.53) */
    uint8_t my_mpu6050_get_temp(int16_t *temp_01c);

    /* ---------------------------------------- 数据就绪中断 (PB12 EXTI 上升沿) */
    /* ISR 只置标志/计边沿；器件锁存在 get_data 的任务事务中释放。 */

    /* 任务上下文查询并清除"数据就绪"标志: 1=发生过(已清), 0=无。
 * 注意: 标志只有 1 位, 同一时刻窗口内多个中断会合并成一个, 不能用来计频! */
    uint8_t my_mpu6050_int_check_clear(void);
    /* ISR 边沿计数快照；INT 锁存期间 DATA_RDY 合并，不能代表器件采样率 */
    uint16_t my_mpu6050_int_count(void);
    /* 取 sensor 任务读回的 INT_STATUS 快照 (bit0=DATA_RDY) */
    uint8_t my_mpu6050_get_int_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
