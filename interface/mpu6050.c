/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 驱动实现 —— 软件 I2C (common/my_i2c.c, SENSOR 总线)
 *          移植自旧工程 flight_hal/interface/my_mpu6050.c：
 *          手表±1000°/s、±4g；初始化100Hz，sensor按使用状态切换50/25Hz六轴或5Hz加速度低功耗。
 *          数据就绪中断: 器件 DATA_RDY 输出仍使能 —— INT_ENABLE(0x38)=0x01，
 *          器件 INT_PIN_CFG 仍为 0x20 (高有效/推挽/锁存, 读 0x3A 清除)；
 *          main 启动即持续屏蔽 PB12 EXTI（早于稍后的 sensor 初始化，避免倒置时序），
 *          sensor 任务按周期轮询 INT_STATUS 的 DATA_RDY 取最新帧，读 0x3A 同时释放器件锁存。
 *          下方 ISR 边沿计数与标志 API 仅为兼容旧调用方保留，正常运行不再有 PB12 中断进入。
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

/* 毫秒级延时封装：ms 为请求等待的毫秒数，按调度器状态选择阻塞方式。 */
static void mpu6050_delay_ms(uint32_t ms)
{
    /* 先确认调度器已经运行，只有这时才能调用 FreeRTOS 的阻塞延时。 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免tick边界缩短硬件最小等待；等待期间让出CPU。 */
        /* 把毫秒换算成 tick 后额外加 1 tick，再让当前任务进入阻塞。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1);
    }
    else
    {
        /* 调度器尚未启动（如初始化早期），改用 HAL 的忙等延时。 */
        HAL_Delay(ms);
    }
}

static volatile uint8_t mpu6050_int_flag;
static volatile uint8_t mpu6050_int_status; /* sensor 任务读回的状态 */
static volatile uint16_t mpu6050_int_edges;
static uint8_t mpu6050_ready;
static uint32_t s_mpu_period_ms = MPU6050_SAMPLE_PERIOD_MS;
static uint8_t s_mpu_accel_reference;

/* 六轴零偏集中于模块私有结构体：accel[0..2]/gyro[0..2] 下标 0/1/2 依次对应 X/Y/Z 轴，
 * 数值为原始 LSB 零偏（Z 轴不包含重力 1g），仍由 sensor 任务的校准/恢复接口维护；
 * 读取方通过 get_offset 取得副本。 */
static mpu6050_offset_struct s_mpu_offset;

/* ---------------------------------------------------------- 总线独占保护 */
/* 运行时仅 sensor 任务访问 SENSOR 总线；EXTI 不做 I2C。
 * 无需整次事务关中断，GPIO 方向切换的短临界区仍在 my_i2c 内。 */
static uint8_t mpu6050_bus_write(uint8_t reg, const uint8_t *dat, uint16_t len)
{
    /* 直接转交软件 I2C 的“寄存器地址 + 连续数据”写事务，返回值沿用 0=成功/1=失败。 */
    return my_i2c_write_bytes(MPU6050_I2C_BUS, MPU6050_ADDR, reg, dat, len);
}

static uint8_t mpu6050_bus_read(uint8_t reg, uint8_t *dat, uint16_t len)
{
    /* 从 reg 起连续读 len 字节写入 dat，返回值沿用 0=成功/1=失败。 */
    return my_i2c_read_bytes(MPU6050_I2C_BUS, MPU6050_ADDR, reg, dat, len);
}

/* -------------------------------------------------------------- 单寄存器访问 */
/**
 * @brief 写寄存器
 * @return 0=成功, 1=总线失败
 */
uint8_t my_mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    /* 把单字节 data 的地址传给总线写：len=1 表示只写这一个寄存器。 */
    return mpu6050_bus_write(reg, &data, 1);
}

/**
 * @brief 读单个寄存器
 * @return 0=成功, 1=总线失败
 */
uint8_t my_mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    /* 只读 1 字节：数据写入调用方提供的 data 缓冲区。 */
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
    /* 先让数据接口失效：重配期间读到的帧不再对外发布。 */
    mpu6050_ready = 0;
    /* 采样周期和加速度参考模式回到上电默认的 100Hz 六轴。 */
    s_mpu_period_ms = MPU6050_SAMPLE_PERIOD_MS;
    s_mpu_accel_reference = 0;
    /* 清掉上一次运行留下的“无新帧”与“睡眠中”标记。 */
    mpu_recovery.no_data = 0;
    mpu_recovery.sleeping = 0;
    /* 状态机回到第一步：探测器件是否应答。 */
    mpu_recovery.phase = MPU_RECOVER_PROBE;
    /* 配置表下标与探测/故障计数一起归零。 */
    mpu_recovery.index = mpu_recovery.probes = mpu_recovery.faults = 0;
    /* 记录本轮阶段起始时刻，供 50ms 探测间隔和复位超时使用。 */
    mpu_recovery.phase_ms = HAL_GetTick();
}

/* 标记恢复失败：清数据有效标志、进入 FAILED 阶段并记下时刻；固定返回 1。 */
static uint8_t mpu6050_recover_failed(void)
{
    /* 失败后不再允许 get_data 发布数据。 */
    mpu6050_ready = 0;
    /* 进入失败阶段，由 recover_step 在 1s 后自动重新 begin。 */
    mpu_recovery.phase = MPU_RECOVER_FAILED;
    mpu_recovery.phase_ms = HAL_GetTick();
    return 1;
}

/* 累计一次读取失败；计数封顶 3，并在刚好达到 3 时记下故障起始时刻。 */
static void mpu6050_read_fault(void)
{
    /* 仅在计数小于 3 时自增，避免计数器回绕或无限增长。 */
    if (mpu_recovery.faults < 3 && ++mpu_recovery.faults == 3)
    {
        /* 第 3 次失败的时刻，作为“持续 1s 无成功读”的超时起点。 */
        mpu_recovery.fault_ms = HAL_GetTick();
    }
}

/* 非阻塞恢复状态机：每次调用最多发起一笔 I2C 事务。
 * now_ms 为调用方的当前毫秒时刻；返回 0=已就绪，1=失败冷却中，2=仍在推进。 */
uint8_t my_mpu6050_recover_step(uint32_t now_ms)
{
    /* 复位后按顺序写入的寄存器表：每项为 {寄存器地址, 目标值}。 */
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
    /* 暂存单次寄存器读出的值（WHO_AM_I、PWR_MGMT_1、INT_STATUS 共用）。 */
    uint8_t data;
    /* 已经就绪时不再重配，只在两种异常下重新开始恢复流程。 */
    if (mpu6050_ready != 0)
    {
        /* 连续1s无新帧也重配，覆盖器件复位后仍ACK但停采样的情况。 */
        /* 条件一：最近一次读取未就绪且距有效帧已满 1s；
         * 条件二：累计 3 次读失败且距首次满 3 次已满 1s。 */
        if ((mpu_recovery.no_data != 0 && (uint32_t)(now_ms - mpu_recovery.sample_ms) >= 1000) ||
            (mpu_recovery.faults >= 3 && (uint32_t)(now_ms - mpu_recovery.fault_ms) >= 1000))
        {
            /* 重新走完整恢复流程，本轮返回“推进中”。 */
            my_mpu6050_recover_begin();
            return 2;
        }
        /* 没有异常：器件仍在正常出数，返回就绪。 */
        return 0;
    }
    /* 未就绪：按当前阶段推进一步。 */
    switch (mpu_recovery.phase)
    {
    case MPU_RECOVER_FAILED:
        /* 失败后冷却 1s，期间保持返回 1，不发起总线事务。 */
        if ((uint32_t)(now_ms - mpu_recovery.phase_ms) >= 1000)
        {
            /* 冷却结束：重新开始一轮恢复。 */
            my_mpu6050_recover_begin();
            return 2;
        }
        return 1;
    case MPU_RECOVER_PROBE:
        /* 已探测过至少一次时，两次探测之间至少间隔 50ms。 */
        if (mpu_recovery.probes != 0 && (uint32_t)(now_ms - mpu_recovery.phase_ms) < 50)
        {
            return 2;
        }
        /* 读 WHO_AM_I，能应答就进入复位阶段。 */
        if (my_mpu6050_read_reg(MPU6050_WHO_AM_I, &data) == 0)
        {
            mpu_recovery.phase = MPU_RECOVER_RESET;
        }
        /* 不应答：累计探测次数，达到 20 次判定恢复失败。 */
        else if (++mpu_recovery.probes >= 20)
        {
            return mpu6050_recover_failed();
        }
        /* 探测成功、或失败但未满 20 次时，都刷新阶段时刻作为下一次 50ms 间隔的起点。 */
        mpu_recovery.phase_ms = HAL_GetTick();
        return 2;
    case MPU_RECOVER_RESET:
        /* 向 PWR_MGMT_1 写 DEVICE_RESET(0x80) 触发器件软复位。 */
        if (my_mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80) != 0)
        {
            return mpu6050_recover_failed();
        }
        /* 写成功：转入等待复位完成，并记下等待起点。 */
        mpu_recovery.phase = MPU_RECOVER_WAIT_RESET;
        mpu_recovery.phase_ms = HAL_GetTick();
        return 2;
    case MPU_RECOVER_WAIT_RESET:
        /* 复位等待超过 1s 视为失败。 */
        if ((uint32_t)(now_ms - mpu_recovery.phase_ms) >= 1000)
        {
            return mpu6050_recover_failed();
        }
        /* 复位期间NACK/未回到0x40均等下一拍，不在本拍忙等。 */
        /* 读回 PWR_MGMT_1，只有读成功且值为 0x40（复位完成默认态）才进入配置。 */
        if (my_mpu6050_read_reg(MPU6050_REG_PWR_MGMT_1, &data) == 0 && data == 0x40)
        {
            mpu_recovery.phase = MPU_RECOVER_CONFIG;
        }
        return 2;
    case MPU_RECOVER_CONFIG:
        /* 按 index 逐项写配置表，本拍只写一个寄存器。 */
        if (my_mpu6050_write_reg(config[mpu_recovery.index][0], config[mpu_recovery.index][1]) != 0)
        {
            return mpu6050_recover_failed();
        }
        /* 下标前进；写满整张表后转入清中断状态阶段。 */
        if (++mpu_recovery.index >= sizeof(config) / sizeof(config[0]))
        {
            mpu_recovery.phase = MPU_RECOVER_CLEAR;
        }
        return 2;
    case MPU_RECOVER_CLEAR:
        /* 读一次 INT_STATUS，释放器件锁存的中断并清掉上电残留状态。 */
        if (my_mpu6050_read_reg(MPU6050_INT_STATUS, &data) != 0)
        {
            return mpu6050_recover_failed();
        }
        /* 全部步骤完成：进入 DONE，记录有效帧基准时刻并允许数据接口输出。 */
        mpu_recovery.phase = MPU_RECOVER_DONE;
        mpu_recovery.sample_ms = HAL_GetTick();
        mpu6050_ready = 1;
        return 0;
    default:
        /* 未知/已完成阶段：按失败返回 1，等待上层再次 begin。 */
        return 1;
    }
}

/* 仅保留启动/旧调用方的阻塞入口；运行期只调用recover_step。 */
uint8_t my_mpu6050_init(void)
{
    /* 保存状态机每次推进的返回值，循环结束后作为本函数的返回值。 */
    uint8_t result;
    /* 从探测阶段开始一轮完整的恢复/初始化流程。 */
    my_mpu6050_recover_begin();
    /* 只要状态机返回 2（仍在推进）就继续调用，每轮重新取当前毫秒时刻。 */
    while ((result = my_mpu6050_recover_step(HAL_GetTick())) == 2)
    {
        /* 探测阶段两次尝试之间等待 50ms，与 recover_step 内的间隔判断对应。 */
        if (mpu_recovery.phase == MPU_RECOVER_PROBE)
        {
            mpu6050_delay_ms(50);
        }
        /* 复位等待阶段每 1ms 轮询一次复位是否完成。 */
        else if (mpu_recovery.phase == MPU_RECOVER_WAIT_RESET)
        {
            mpu6050_delay_ms(1);
        }
    }
    /* 返回 0 表示已就绪；返回 1 表示失败冷却（或异常阶段）。 */
    return result;
}

/* 切换器件睡眠/唤醒；sleeping 非 0 表示进入睡眠。返回 0=写寄存器成功，1=失败。 */
uint8_t my_mpu6050_set_sleep(uint8_t sleeping)
{
    /* 将要写入 PWR_MGMT_1 的值，先按当前采样模式取默认配置。 */
    uint8_t config = 0x01;
    /* now_ms 为本次操作时刻；elapsed_ms 为本次睡眠持续时长，用于补偿超时基准。 */
    uint32_t now_ms, elapsed_ms;
    /* 200ms 低功耗循环模式：CYCLE=1、TEMP_DIS=1、内部 8MHz 时钟。 */
    if (s_mpu_period_ms == 200)
    {
        config = 0x28;
    }
    /* 加速度参考模式：关闭温度并改用内部 8MHz 时钟。 */
    else if (s_mpu_accel_reference != 0)
    {
        config = 0x08;
    }
    /* 需要睡眠时，在保留时钟选择的前提下清 CYCLE 位并置 SLEEP 位。 */
    if (sleeping != 0)
    {
        config = (uint8_t)((config & (uint8_t)~0x20) | 0x40); /* 暂停循环，保留温度和时钟选择，唤醒恢复当前模式。 */
    }
    /* 写入电源管理寄存器；失败则重新走恢复流程并返回 1。 */
    if (my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, config) != 0)
    {
        my_mpu6050_recover_begin();
        return 1;
    }
    /* 记录写寄存器成功后的当前时刻，作为睡眠计时基准。 */
    now_ms = HAL_GetTick();
    /* 由唤醒转为睡眠：记下入睡时刻并置睡眠标记。 */
    if (sleeping != 0 && mpu_recovery.sleeping == 0)
    {
        mpu_recovery.sleep_ms = now_ms;
        mpu_recovery.sleeping = 1;
    }
    /* 由睡眠转为唤醒：把整段睡眠时间从各项超时基准中扣除。 */
    else if (sleeping == 0 && mpu_recovery.sleeping != 0)
    {
        /* 正常睡眠不计入故障超时；保留各次唤醒采样失败的累计，直到真正读到新帧。 */
        elapsed_ms = now_ms - mpu_recovery.sleep_ms;
        /* 平移“最近有效帧”基准，避免唤醒瞬间被误判为 1s 无新帧。 */
        mpu_recovery.sample_ms += elapsed_ms;
        /* 已经累计到 3 次失败时，同时平移故障计时起点。 */
        if (mpu_recovery.faults >= 3)
        {
            mpu_recovery.fault_ms += elapsed_ms;
        }
        /* 清除睡眠标记，后续按正常运行判断超时。 */
        mpu_recovery.sleeping = 0;
    }
    return 0;
}

uint8_t my_mpu6050_set_sample_period(uint32_t period_ms)
{
    /* 只是不带加速度参考模式的采样周期设置，转交通用接口。 */
    return my_mpu6050_set_sample_mode(period_ms, 0);
}

/* 设置采样周期与加速度参考模式；period_ms 单位为 ms，accel_reference 非 0 表示只连续采加速度。
 * 返回 0=设置成功或无需改动，1=参数非法/器件未就绪/寄存器写失败。 */
uint8_t my_mpu6050_set_sample_mode(uint32_t period_ms, uint8_t accel_reference)
{
    /* 将要写入 CONFIG 寄存器的 DLPF 配置值。 */
    uint8_t filter;
    /* 只接受 10/20/40/200ms 四档，其余周期直接判为参数错误。 */
    if (period_ms != 10 && period_ms != 20 && period_ms != 40 && period_ms != 200)
    {
        return 1;
    }
    /* 器件尚未完成初始化/恢复时不允许改配置。 */
    if (mpu6050_ready == 0)
    {
        return 1;
    }
    /* 加速度参考模式与 200ms 低功耗循环互斥，后者优先。 */
    accel_reference = accel_reference != 0 && period_ms != 200;
    /* 周期与参考模式都没变化时直接返回成功，不产生总线事务。 */
    if (period_ms == s_mpu_period_ms && accel_reference == s_mpu_accel_reference)
    {
        return 0;
    }
    /* 涉及进入/退出低功耗循环或参考模式切换时，需要先做一次电源状态过渡。 */
    if (period_ms == 200 || s_mpu_period_ms == 200 || accel_reference != s_mpu_accel_reference)
    {
        /* 先退出循环并使用内部时钟，再启停陀螺仪，最后接通目标模式。 */
        /* 默认保持六轴全开：PWR_MGMT_2=0x00、PWR_MGMT_1=0x01（使用陀螺仪 PLL）。 */
        uint8_t power_2 = 0x00;
        uint8_t power_1 = 0x01;
        /* 目标为 200ms：三轴陀螺仪待机，加速度按 5Hz 循环唤醒。 */
        if (period_ms == 200)
        {
            power_2 = 0x47; /* LP_WAKE_CTRL=1(5Hz)，三轴陀螺仪待机，加速度保持启用。 */
            power_1 = 0x28; /* CYCLE=1、SLEEP=0、TEMP_DIS=1，内部8MHz时钟。 */
        }
        /* 目标为加速度参考模式：只停陀螺仪，时钟改用内部振荡器。 */
        else if (accel_reference != 0)
        {
            power_2 = 0x07; /* 三轴陀螺仪待机，加速度连续转换，周期仍由SMPLRT_DIV设置。 */
            power_1 = 0x08; /* 温度关闭、内部8MHz时钟；不能选择已停用的陀螺仪PLL。 */
        }
        /* 三步写入：先切内部时钟，再设置各轴启停，最后写入目标电源模式。 */
        if (my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x08) != 0 ||
            my_mpu6050_write_reg(MPU6050_PWR_MGMT_2, power_2) != 0 ||
            my_mpu6050_write_reg(MPU6050_PWR_MGMT_1, power_1) != 0)
        {
            /* 任一步失败都重新开始恢复流程，并返回失败。 */
            my_mpu6050_recover_begin();
            return 1;
        }
        /* 模式切换后重置有效帧基准，避免旧的 sample_ms 立刻触发 1s 无帧重配。 */
        mpu_recovery.sample_ms = HAL_GetTick();
        mpu_recovery.no_data = 0;
        /* 200ms 低功耗循环不需要再配置 DLPF 和分频，直接记录并返回。 */
        if (period_ms == 200)
        {
            s_mpu_period_ms = period_ms;
            s_mpu_accel_reference = 0;
            return 0;
        }
    }
    /* 其余周期默认使用 DLPF 配置 4：50Hz 采样时陀螺仪 20Hz、加速度 21Hz 带宽。 */
    filter = 4; /* 50Hz采样：陀螺仪20Hz、加速度21Hz带宽。 */
    /* 10ms 对应 100Hz 六轴运动模式，恢复项目默认的 42Hz 带宽。 */
    if (period_ms == 10)
    {
        filter = MPU6050_DLPF_CFG; /* 运动模式恢复100Hz/42Hz带宽。 */
    }
    /* 40ms 对应 25Hz 采样，使用 10Hz 带宽。 */
    else if (period_ms == 40)
    {
        filter = 5; /* 25Hz采样：10Hz带宽。 */
    }
    /* 先写 DLPF，再写分频值（周期减 1，因为分频寄存器是 0 基）。 */
    if (my_mpu6050_write_reg(MPU6050_CONFIG, filter) != 0 ||
        my_mpu6050_write_reg(MPU6050_SMPLRT_DIV, (uint8_t)(period_ms - 1)) != 0)
    {
        /* 写失败：重新恢复，避免停留在半配置状态。 */
        my_mpu6050_recover_begin();
        return 1;
    }
    /* 两笔写都成功后，才把新周期与新参考模式记为当前生效配置。 */
    s_mpu_period_ms = period_ms;
    s_mpu_accel_reference = accel_reference;
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
    /* 把两个判定门限的 LSB 值写进诊断数组，便于上层观察实际使用值。 */
    s_cal.diag[MPU_CAL_DIAG_LEVEL_LIMIT] = (uint32_t)mpu_cal_level_lsb;
    s_cal.diag[MPU_CAL_DIAG_STILL_LIMIT] = (uint32_t)mpu_cal_still_lsb;
    /* 同步当前校准状态，然后返回诊断数组首地址（数据仍在模块内）。 */
    s_cal.diag[MPU_CAL_DIAG_STATE] = (uint32_t)s_cal.state;
    return s_cal.diag;
}

void my_mpu6050_get_offset(mpu6050_offset_struct *offset)
{
    /* 把模块私有的六轴零偏结构体整体复制到输出结构体，六个分量单位均为原始 LSB。 */
    *offset = s_mpu_offset;
}

void my_mpu6050_restore_offset(const mpu6050_offset_struct *offset)
{
    /* 用传入副本整体覆盖模块私有的六轴零偏结构体。 */
    s_mpu_offset = *offset;
    /* 清空校准过程数据（含累计和、诊断字段），避免残留旧状态。 */
    memset(&s_cal, 0, sizeof(s_cal));
    /* 标记为“已恢复”，表示零偏来自保存的副本而非本次校准。 */
    s_cal.state = MPU6050_CAL_RESTORED;
}

void my_mpu6050_zero_offset_begin(uint32_t now_ms)
{
    /* 整个校准结构体清零，清掉上一次的累计和、样本与诊断计数。 */
    memset(&s_cal, 0, sizeof(s_cal));
    /* 阶段起点和请求起点都记为 now_ms，用于超时判断与“请求前样本不计入”。 */
    s_cal.phase_ms = now_ms;
    s_cal.started_ms = now_ms;
    /* 进入等待静止阶段。 */
    s_cal.state = MPU6050_CAL_WAIT_STILL;
}

/* 判断当前帧相对上一帧是否“静止”，并把各轴读数与判定结果写入诊断数组。
 * current 为本次读取的加速度；返回 1=通过全部静止判据，0=至少一项越界。 */
static uint8_t mpu6050_cal_still(const Accel_struct *current)
{
    /* 位掩码：每个被拒绝的判据占一位，0 表示本帧全部通过。 */
    uint32_t reason = 0;
    /* 记录本帧三轴原始加速度（按有符号补码转成无符号存放，便于诊断显示）。 */
    s_cal.diag[MPU_CAL_DIAG_AX] = (uint32_t)(int32_t)current->Accel_x;
    s_cal.diag[MPU_CAL_DIAG_AY] = (uint32_t)(int32_t)current->Accel_y;
    s_cal.diag[MPU_CAL_DIAG_AZ] = (uint32_t)(int32_t)current->Accel_z;
    /* 记录三轴相对上一帧的变化量绝对值（LSB）。 */
    s_cal.diag[MPU_CAL_DIAG_DX] = (uint32_t)abs(current->Accel_x - s_cal.last_accel.Accel_x);
    s_cal.diag[MPU_CAL_DIAG_DY] = (uint32_t)abs(current->Accel_y - s_cal.last_accel.Accel_y);
    s_cal.diag[MPU_CAL_DIAG_DZ] = (uint32_t)abs(current->Accel_z - s_cal.last_accel.Accel_z);
    /* X 轴帧间变化达到静止门限，判定为抖动。 */
    if (s_cal.diag[MPU_CAL_DIAG_DX] >= (uint32_t)mpu_cal_still_lsb)
    {
        reason |= MPU_CAL_REJECT_DX;
    }
    /* Y 轴帧间变化超限。 */
    if (s_cal.diag[MPU_CAL_DIAG_DY] >= (uint32_t)mpu_cal_still_lsb)
    {
        reason |= MPU_CAL_REJECT_DY;
    }
    /* Z 轴帧间变化超限。 */
    if (s_cal.diag[MPU_CAL_DIAG_DZ] >= (uint32_t)mpu_cal_still_lsb)
    {
        reason |= MPU_CAL_REJECT_DZ;
    }
    /* 水平轴 X 的绝对值超过水平容差，说明器件明显倾斜。 */
    if (abs(current->Accel_x) >= mpu_cal_level_lsb)
    {
        reason |= MPU_CAL_REJECT_AX;
    }
    /* 水平轴 Y 的绝对值超过水平容差。 */
    if (abs(current->Accel_y) >= mpu_cal_level_lsb)
    {
        reason |= MPU_CAL_REJECT_AY;
    }
    /* Z 轴相对本次校准的重力基准偏差超过容差。 */
    if (abs(current->Accel_z - s_cal.gravity_z_lsb) >= mpu_cal_level_lsb)
    {
        reason |= MPU_CAL_REJECT_AZ;
    }
    /* 保存本帧拒绝原因，并累计“曾经出现过”的原因位。 */
    s_cal.diag[MPU_CAL_DIAG_REJECT] = reason;
    s_cal.diag[MPU_CAL_DIAG_REJECT_SEEN] |= reason;
    /* 统计本帧是否参与判定，以及通过/拒绝各一帧。 */
    ++s_cal.diag[MPU_CAL_DIAG_CHECKED];
    if (reason == 0)
    {
        ++s_cal.diag[MPU_CAL_DIAG_ACCEPTED];
    }
    else
    {
        ++s_cal.diag[MPU_CAL_DIAG_REJECTED];
    }
    /* 返回 1 仅表示本帧所有判据都通过。 */
    return reason == 0;
}

/* 校准状态机单步：用 sensor 任务已读到的一帧推进校准。
 * raw 为本次六轴数据（已减旧零偏；成功提交时对当前帧补扣本次新增的零偏增量）；valid 非 0 表示该帧有效；
 * now_ms 为帧时刻（用于样本间隔与“请求前样本”判断），wall_ms 为墙钟（用于超时）。
 * 返回推进后的校准状态。 */
static mpu6050_cal_state mpu6050_zero_offset_update(Accel_Gyro_struct *raw, uint8_t valid, uint32_t now_ms, uint32_t wall_ms)
{
    /* 当前阶段的超时门限，单位 ms，按实际采样周期缩放。 */
    uint32_t timeout_ms;
    /* 本帧是否满足静止判据（1=静止）。 */
    uint8_t still;
    /* 只有等待静止/平均两个进行中状态才需要处理样本，其余状态原样返回。 */
    if (s_cal.state != MPU6050_CAL_WAIT_STILL && s_cal.state != MPU6050_CAL_AVERAGING)
    {
        return s_cal.state;
    }
    /* 等待静止阶段按 3s 基准时间折算到当前采样周期。 */
    timeout_ms = 3000 * s_mpu_period_ms / MPU6050_SAMPLE_PERIOD_MS;
    /* 平均阶段按 2s 基准时间折算。 */
    if (s_cal.state == MPU6050_CAL_AVERAGING)
    {
        timeout_ms = 2000 * s_mpu_period_ms / MPU6050_SAMPLE_PERIOD_MS;
    }
    /* 用墙钟判断当前阶段是否超时；无符号减法可正确处理计时器回绕。 */
    if ((uint32_t)(wall_ms - s_cal.phase_ms) >= timeout_ms)
    {
        /* 按阶段记录不同的超时原因位。 */
        if (s_cal.state == MPU6050_CAL_WAIT_STILL)
        {
            s_cal.diag[MPU_CAL_DIAG_REJECT] = MPU_CAL_REJECT_WAIT_TIMEOUT;
        }
        else
        {
            s_cal.diag[MPU_CAL_DIAG_REJECT] = MPU_CAL_REJECT_AVERAGE_TIMEOUT;
        }
        /* 超时原因同样累计进“曾经出现”掩码，并判定本次校准失败。 */
        s_cal.diag[MPU_CAL_DIAG_REJECT_SEEN] |= s_cal.diag[MPU_CAL_DIAG_REJECT];
        s_cal.state = MPU6050_CAL_FAILED;
        return s_cal.state;
    }
    /* 请求前取得的样本不累计；等待和平均阶段均按墙钟判断超时。 */
    /* now_ms 早于请求时刻（有符号差为负）说明这帧来自 begin 之前，直接丢弃。 */
    if (valid != 0 && (int32_t)(now_ms - s_cal.started_ms) < 0)
    {
        return s_cal.state;
    }
    /* 读取失败或无数据：只记录原因，不推进样本计数，也不改动已生效零偏。 */
    if (valid == 0 || raw == NULL)
    {
        /* 丢帧不计入有效样本，原墙钟窗口仍继续计时，不覆盖已生效零偏。 */
        s_cal.diag[MPU_CAL_DIAG_REJECT] = MPU_CAL_REJECT_READ;
        s_cal.diag[MPU_CAL_DIAG_REJECT_SEEN] |= MPU_CAL_REJECT_READ;
        return s_cal.state;
    }
    /* 第一帧有效样本：只建立基准，不参与平均。 */
    if (s_cal.have_sample == 0)
    {
        /* 按首帧 Z 轴符号确定本次校准的 ±1g 重力基准，采样期间不再切换。 */
        s_cal.gravity_z_lsb = MPU6050_ACCEL_LSB_PER_G;
        if (raw->Accel.Accel_z < 0)
        {
            s_cal.gravity_z_lsb = -MPU6050_ACCEL_LSB_PER_G;
        }
        /* 把首帧三轴读数写入诊断，并保存为首帧参考。 */
        s_cal.diag[MPU_CAL_DIAG_AX] = (uint32_t)(int32_t)raw->Accel.Accel_x;
        s_cal.diag[MPU_CAL_DIAG_AY] = (uint32_t)(int32_t)raw->Accel.Accel_y;
        s_cal.diag[MPU_CAL_DIAG_AZ] = (uint32_t)(int32_t)raw->Accel.Accel_z;
        s_cal.last_accel = raw->Accel;
        s_cal.last_sample_ms = now_ms;
        s_cal.have_sample = 1;
        return s_cal.state;
    }
    /* 帧间隔不足一个标称采样周期时跳过，避免同一帧被重复累计。 */
    if ((uint32_t)(now_ms - s_cal.last_sample_ms) < MPU6050_SAMPLE_PERIOD_MS)
    {
        return s_cal.state;
    }
    /* 记录实际帧间隔供诊断，并更新上一帧时刻。 */
    s_cal.diag[MPU_CAL_DIAG_INTERVAL_MS] = now_ms - s_cal.last_sample_ms;
    s_cal.last_sample_ms = now_ms;
    /* 用本帧加速度做静止判定，随后把本帧存为下一次比较的参考。 */
    still = mpu6050_cal_still(&raw->Accel);
    s_cal.last_accel = raw->Accel;
    /* 等待静止阶段：连续通过判定的帧数达到 100 才进入平均阶段。 */
    if (s_cal.state == MPU6050_CAL_WAIT_STILL)
    {
        if (still != 0)
        {
            /* 通过一帧就累加连续静止计数，并刷新历史峰值。 */
            ++s_cal.stable_count;
            if (s_cal.stable_count > s_cal.diag[MPU_CAL_DIAG_STABLE_PEAK])
            {
                s_cal.diag[MPU_CAL_DIAG_STABLE_PEAK] = s_cal.stable_count;
            }
        }
        else
        {
            /* 任一帧不满足就清零，要求的是“连续”静止。 */
            s_cal.stable_count = 0;
        }
        /* 连续 100 帧静止：重置阶段计时并进入平均阶段。 */
        if (s_cal.stable_count >= 100)
        {
            s_cal.phase_ms = wall_ms;
            s_cal.state = MPU6050_CAL_AVERAGING;
        }
        return s_cal.state;
    }
    /* 平均阶段中只要出现一帧不静止，本次校准立即失败（不提交任何新零偏）。 */
    if (still == 0)
    {
        s_cal.state = MPU6050_CAL_FAILED;
        return s_cal.state;
    }
    /* 静止帧累计：加速度三轴，Z 轴先扣除 ±1g 重力基准。 */
    s_cal.sum[0] += raw->Accel.Accel_x;
    s_cal.sum[1] += raw->Accel.Accel_y;
    s_cal.sum[2] += raw->Accel.Accel_z - s_cal.gravity_z_lsb;
    /* 累计已减旧零偏的三轴陀螺仪计数(LSB)，作为残差均值输入。 */
    s_cal.sum[3] += raw->Gyro.Gyro_x;
    s_cal.sum[4] += raw->Gyro.Gyro_y;
    s_cal.sum[5] += raw->Gyro.Gyro_z;
    /* 满 100 帧后求平均并一次性提交。 */
    if (++s_cal.average_count >= 100)
    {
        /* 仍以旧偏移下的残差平均增量更新；只有整窗成功才一次提交。 */
        /* 六个残差均值累加进零偏结构体中的六个分量（整数除法向零截断）。 */
        s_mpu_offset.accel[0] += (int16_t)(s_cal.sum[0] / 100);
        s_mpu_offset.accel[1] += (int16_t)(s_cal.sum[1] / 100);
        s_mpu_offset.accel[2] += (int16_t)(s_cal.sum[2] / 100);
        s_mpu_offset.gyro[0] += (int16_t)(s_cal.sum[3] / 100);
        s_mpu_offset.gyro[1] += (int16_t)(s_cal.sum[4] / 100);
        s_mpu_offset.gyro[2] += (int16_t)(s_cal.sum[5] / 100);
        /* 成功提交时对当前帧补扣本次新增的零偏增量，使调用方拿到的这一帧也已修正。 */
        raw->Accel.Accel_x -= (int16_t)(s_cal.sum[0] / 100);
        raw->Accel.Accel_y -= (int16_t)(s_cal.sum[1] / 100);
        raw->Accel.Accel_z -= (int16_t)(s_cal.sum[2] / 100);
        raw->Gyro.Gyro_x -= (int16_t)(s_cal.sum[3] / 100);
        raw->Gyro.Gyro_y -= (int16_t)(s_cal.sum[4] / 100);
        raw->Gyro.Gyro_z -= (int16_t)(s_cal.sum[5] / 100);
        /* 整窗提交完成，状态置为成功。 */
        s_cal.state = MPU6050_CAL_DONE;
    }
    return s_cal.state;
}

mpu6050_cal_state my_mpu6050_zero_offset_step(Accel_Gyro_struct *raw, uint8_t valid, uint32_t now_ms)
{
    /* 对外接口只有统一时间基准：样本时刻与墙钟都使用调用方传入的 now_ms。 */
    return mpu6050_zero_offset_update(raw, valid, now_ms, now_ms);
}

mpu6050_cal_state my_mpu6050_zero_offset_state(void)
{
    /* 返回当前校准状态枚举，不做任何状态推进。 */
    return s_cal.state;
}

void my_mpu6050_zero_offset_progress(uint16_t *stable, uint16_t *average)
{
    /* 输出连续静止帧计数与平均阶段已累计帧数。 */
    *stable = s_cal.stable_count;
    *average = s_cal.average_count;
}

/* 兼容旧调用方；手表启动只使用 begin/step，不调用此阻塞接口。 */
uint8_t my_mpu6050_zero_offset(void)
{
    /* 本拍从总线读到的六轴数据，作为校准输入。 */
    Accel_Gyro_struct raw;
    /* 本拍读取是否成功：非 0 表示可交给状态机累计。 */
    uint8_t valid;
    /* 状态机推进后的校准状态。 */
    mpu6050_cal_state state;
    /* 以当前时刻开始一轮校准（结构体清零并进入等待静止）。 */
    my_mpu6050_zero_offset_begin(HAL_GetTick());
    do
    {
        /* 先读一帧：读成功才算有效样本，失败时 raw 内容不作为数据使用。 */
        valid = my_mpu6050_get_data(&raw) == 0;
        /* 用同一帧推进校准状态机，帧时刻取当前毫秒。 */
        state = my_mpu6050_zero_offset_step(&raw, valid, HAL_GetTick());
        /* 仍在校准过程中：按当前采样周期让出 CPU，等下一帧。 */
        if (state == MPU6050_CAL_WAIT_STILL || state == MPU6050_CAL_AVERAGING)
        {
            vTaskDelay(pdMS_TO_TICKS(s_mpu_period_ms));
        }
    }
    /* 直到状态机离开等待/平均阶段（成功、失败或已恢复）才退出循环。 */
    while (state == MPU6050_CAL_WAIT_STILL || state == MPU6050_CAL_AVERAGING);
    /* 校准成功返回 0。 */
    if (state == MPU6050_CAL_DONE)
    {
        return 0;
    }
    /* 其余状态（失败/恢复等）返回 1。 */
    return 1;
}

/**
 * @brief 静止时增量微调陀螺仪零偏 (见头文件注释)
 */
void my_mpu6050_gyro_trim(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z)
{
    /* 把传入的三轴残余角速度直接累加进陀螺仪零偏，下次读取即按新零偏扣除。 */
    s_mpu_offset.gyro[0] += gyro_x;
    s_mpu_offset.gyro[1] += gyro_y;
    s_mpu_offset.gyro[2] += gyro_z;
}

/* ---------------------------------------------------------------- 数据读取 */
/**
 * @brief 读取三轴角速度 (已减零偏)
 * @return 0=成功, 1=总线失败(数据未更新)
 */
uint8_t my_mpu6050_get_gyro(Gyro_struct *gyro)
{
    /* 突发读缓冲：6 字节对应 X/Y/Z 三轴各高低两字节。 */
    uint8_t buf[6];

    /* 输出指针为空时直接返回失败，不发起总线事务。 */
    if (gyro == NULL)
    {
        return 1;
    }

    /* 一次突发读 6 字节 (0x43~0x48) */
    /* 读失败时保持输出结构体不变，并返回 1。 */
    if (mpu6050_bus_read(MPU6050_GYRO_XOUT_H, buf, 6) != 0)
    {
        return 1;
    }

    /* 高字节左移 8 位与低字节拼成有符号 16 位，再减去对应轴零偏。 */
    gyro->Gyro_x = (int16_t)(((buf[0] << 8) | buf[1]) - s_mpu_offset.gyro[0]);
    gyro->Gyro_y = (int16_t)(((buf[2] << 8) | buf[3]) - s_mpu_offset.gyro[1]);
    gyro->Gyro_z = (int16_t)(((buf[4] << 8) | buf[5]) - s_mpu_offset.gyro[2]);
    /* 三轴都写入后返回成功。 */
    return 0;
}

/**
 * @brief 读取三轴加速度 (已减零偏；抖动较严重，必要时上层再滤波)
 * @return 0=成功, 1=总线失败(数据未更新)
 */
uint8_t my_mpu6050_get_accel(Accel_struct *accel)
{
    /* 突发读缓冲：6 字节对应 X/Y/Z 三轴各高低两字节。 */
    uint8_t buf[6];

    /* 输出指针为空时直接返回失败，不发起总线事务。 */
    if (accel == NULL)
    {
        return 1;
    }

    /* 一次突发读 6 字节 (0x3B~0x40) */
    /* 读失败时保持输出结构体不变，并返回 1。 */
    if (mpu6050_bus_read(MPU6050_ACCEL_XOUT_H, buf, 6) != 0)
    {
        return 1;
    }

    /* 高低字节拼接成有符号 16 位原始值，再减去对应轴零偏（Z 轴零偏不含重力）。 */
    accel->Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - s_mpu_offset.accel[0]);
    accel->Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - s_mpu_offset.accel[1]);
    accel->Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - s_mpu_offset.accel[2]);
    /* 三轴都写入后返回成功。 */
    return 0;
}

/* 一笔读状态与六轴；加速度模式只读0x3A..0x40，无新帧不覆盖输出。 */
uint8_t my_mpu6050_get_data(Accel_Gyro_struct *data)
{
    /* 原始读缓冲：首字节是 0x3A 中断状态，其后依次为加速度/温度/陀螺仪数据。 */
    uint8_t packet[15];
    /* 指向 packet[1]，即 0x3B 起的六轴数据，便于按轴偏移取值。 */
    const uint8_t *buf = packet + 1;
    /* 本次读取开始的毫秒时刻，作为“最近有效帧”的时间戳。 */
    uint32_t started_ms;
    /* 本次要读的字节数，默认读满 15 字节（状态+加速度+温度+陀螺仪）。 */
    uint16_t packet_size = sizeof(packet);

    /* 低功耗/加速度参考模式：陀螺仪与温度已停用，只读状态和加速度共 7 字节。 */
    if (s_mpu_period_ms == 200 || s_mpu_accel_reference != 0)
    {
        packet_size = 7; /* 低功耗只读状态和加速度，不读取停用的温度/陀螺仪寄存器。 */
    }
    /* 输出指针为空或器件未就绪时直接返回失败。 */
    if (data == NULL || mpu6050_ready == 0)
    {
        return 1;
    }
    /* 记录读取开始时刻，用于填充 sample_ms。 */
    started_ms = HAL_GetTick();
    /* 从 INT_STATUS 起一笔突发读；失败则累计故障计数并返回 1。 */
    if (mpu6050_bus_read(MPU6050_INT_STATUS, packet, packet_size) != 0)
    {
        mpu6050_read_fault();
        return 1;
    }
    /* 保存中断状态快照供上层查询，并把连续故障计数清零。 */
    mpu6050_int_status = packet[0];
    mpu_recovery.faults = 0;
    /* bit0 为 DATA_RDY；没有新数据时标记 no_data 并返回 2，输出保持不变。 */
    if ((packet[0] & 0x01) == 0)
    {
        mpu_recovery.no_data = 1;
        return MPU6050_DATA_NOT_READY;
    }

    /* 跳过0x41/0x42温度；完整读成功且DATA_RDY有效后才发布。 */
    /* 加速度三轴：buf[0..5] 为 0x3B..0x40，拼接后减去零偏。 */
    data->Accel.Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - s_mpu_offset.accel[0]);
    data->Accel.Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - s_mpu_offset.accel[1]);
    data->Accel.Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - s_mpu_offset.accel[2]);
    /* 低功耗模式没有陀螺仪数据：把输出角速度清零。 */
    if (s_mpu_period_ms == 200 || s_mpu_accel_reference != 0)
    {
        memset(&data->Gyro, 0, sizeof(data->Gyro));
    }
    /* 六轴模式：buf[8..13] 对应 0x43..0x48，跳过 buf[6..7] 的温度。 */
    else
    {
        data->Gyro.Gyro_x = (int16_t)(((buf[8] << 8) | buf[9]) - s_mpu_offset.gyro[0]);
        data->Gyro.Gyro_y = (int16_t)(((buf[10] << 8) | buf[11]) - s_mpu_offset.gyro[1]);
        data->Gyro.Gyro_z = (int16_t)(((buf[12] << 8) | buf[13]) - s_mpu_offset.gyro[2]);
    }
    /* 记录最近有效帧的读取开始时刻，并刷新恢复状态机的无帧基准。 */
    mpu_sample_ms = started_ms;
    mpu_recovery.sample_ms = HAL_GetTick();
    mpu_recovery.no_data = 0;
    /* 新帧已发布，返回成功。 */
    return 0;
}

uint32_t my_mpu6050_sample_ms(void)
{
    /* 返回最近一次成功发布帧的读取开始时刻（HAL tick 毫秒），不是器件采样时刻。 */
    return mpu_sample_ms;
}

/* ------------------------------------------------------------ 数据就绪中断 */
/* 器件 DATA_RDY 输出仍使能 (INT_ENABLE=0x01, INT_PIN_CFG=0x20)；main 启动即持续屏蔽 PB12 EXTI，
 * 正常运行不再进入本回调，以下 ISR 边沿计数与标志 API 仅为兼容旧调用方保留。
 * HAL 已清 STM32 EXTI PR；这里只记事件，绝不抢 SENSOR 总线。
 * 器件 INT 锁存在 sensor 任务 my_mpu6050_get_data() 轮询 INT_STATUS 时读取 0x3A 释放。
 * 锁存期间多次 DATA_RDY 会合并，边沿计数不是器件采样率。 */

/**
 * @brief HAL GPIO EXTI 回调 (强符号覆盖 hal_gpio.c 中的 __weak 版本)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* 只处理 MPU6050 的 INT 引脚，其它引脚事件不在此驱动内响应。 */
    if (GPIO_Pin == MPU6050_INT_Pin)
    {
        /* 边沿计数加一（16 位自然回绕），供计频用的相邻快照差分。 */
        mpu6050_int_edges++;
        /* 置“发生过数据就绪”标志，由任务侧原子查询并清除。 */
        mpu6050_int_flag = 1;
    }
}

/**
 * @brief ISR 边沿计数快照 (uint16, 计频用相邻两次快照做无符号差分)
 */
uint16_t my_mpu6050_int_count(void)
{
    /* 先保存进入时的中断屏蔽状态，退出时原样恢复。 */
    uint32_t primask = __get_PRIMASK();
    /* 边沿计数的快照。 */
    uint16_t cnt;

    /* 关中断，保证 16 位计数不会被 ISR 在读取中途更新。 */
    __disable_irq();
    cnt = mpu6050_int_edges;
    /* 恢复此前的中断屏蔽状态（不一定重新开中断）。 */
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
    /* 先保存当前中断屏蔽状态，稍后恢复。 */
    uint32_t primask = __get_PRIMASK();
    /* 保存读到的标志原值，作为返回值。 */
    uint8_t flag;

    /* 关中断，使“读标志 + 清标志”成为不可被 ISR 打断的原子操作。 */
    __disable_irq();
    flag = mpu6050_int_flag;
    mpu6050_int_flag = 0;
    /* 恢复进入前的屏蔽状态。 */
    __set_PRIMASK(primask);
    /* 1=自上次清除后发生过中断；0=没有。 */
    return flag;
}

/**
 * @brief 取任务读回的INT_STATUS快照：bit0=DATA_RDY、bit3=I2C_MST_INT、bit4=FIFO_OFLOW。
 */
uint8_t my_mpu6050_get_int_status(void)
{
    /* 直接返回最近一次 get_data 读到的 INT_STATUS 字节快照。 */
    return mpu6050_int_status;
}

/**
 * @brief 读取片内温度传感器 (0x41/0x42 突发读)
 * @param temp_01c 输出温度, 单位 0.01℃ (换算: T = raw/340 + 36.53)
 * @return 0=成功, 1=总线失败
 */
uint8_t my_mpu6050_get_temp(int16_t *temp_01c)
{
    /* 温度寄存器高低字节的突发读缓冲。 */
    uint8_t buf[2];
    /* 拼接后的有符号原始温度值。 */
    int32_t raw;

    /* 输出指针为空时直接返回失败，不发起总线事务。 */
    if (temp_01c == NULL)
    {
        return 1;
    }

    /* 从 TEMP_OUT_H 起读 2 字节；失败返回 1，输出保持不变。 */
    if (mpu6050_bus_read(MPU6050_TEMP_OUT_H, buf, 2) != 0)
    {
        return 1;
    }

    /* 高字节左移 8 位与低字节拼接，并解释为有符号 16 位。 */
    raw = (int16_t)((buf[0] << 8) | buf[1]);
    /* 先转 0.01℃ 定点 (raw*100/340 + 3653), 避免浮点运算 */
    /* 整数除法向零截断；3653 即 36.53℃ 的 0.01℃ 表示。 */
    *temp_01c = (int16_t)(raw * 100 / 340 + 3653);
    return 0;
}
