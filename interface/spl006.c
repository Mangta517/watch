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

/* 毫秒延时封装：调度器运行时让出 CPU，未运行时退回 HAL 忙等。 */
static void spl006_delay_ms(uint32_t ms)
{
    /* 查询调度器状态，判断当前能否使用 FreeRTOS 的阻塞延时。 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 多等一拍，避免tick边界缩短硬件最小等待；等待期间让出CPU。 */
        vTaskDelay(pdMS_TO_TICKS(ms) + 1);
    }
    else
    {
        /* 调度器未启动，不能用 vTaskDelay，只能由 HAL 轮询等待。 */
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
    /* 向 SPL006 写：先发寄存器号，再连续写 len 个字节；返回 0 表示成功。 */
    return my_i2c_write_bytes(SPL006_I2C_BUS, SPL006_ADDR, reg, dat, len);
}

static uint8_t spl006_bus_read(uint8_t reg, uint8_t *dat, uint16_t len)
{
    /* 从 SPL006 读：先发寄存器号，再连续读 len 个字节；返回 0 表示成功。 */
    return my_i2c_read_bytes(SPL006_I2C_BUS, SPL006_ADDR, reg, dat, len);
}

static uint8_t spl006_write_reg(uint8_t reg, uint8_t data)
{
    /* 单字节写：把局部数据变量的地址交给通用写接口。 */
    return spl006_bus_write(reg, &data, 1);
}

static uint8_t spl006_read_reg(uint8_t reg, uint8_t *data)
{
    /* 单字节读：长度固定为 1。 */
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
/* 分步初始化状态机的私有上下文：阶段计时、故障计数与系数缓存。 */
static struct
{
    /* 阶段起点、故障起点、最近有效帧、进入休眠时刻，单位 ms。 */
    uint32_t phase_ms, fault_ms, sample_ms, sleep_ms;
    /* 当前阶段、系数/配置下标、连续读失败数、休眠标志。 */
    uint8_t phase, index, faults, sleeping;
    /* 按寄存器顺序缓存的 16 个系数原始字节。 */
    uint8_t coef[16];
} spl_recovery;

void spl006_recover_begin(void)
{
    /* 重新配置期间系数不可用，先清就绪标志。 */
    spl006_coef_ready = 0;
    /* 从 ID 阶段开始，并把系数/配置下标与故障计数清零。 */
    spl_recovery.phase = SPL_RECOVER_ID;
    spl_recovery.index = spl_recovery.faults = 0;
    /* 记录阶段起点，供失败退避计时。 */
    spl_recovery.phase_ms = HAL_GetTick();
}

static uint8_t spl006_recover_failed(void)
{
    /* 任一步失败都撤销就绪标志，进入失败阶段并记录时刻。 */
    spl006_coef_ready = 0;
    spl_recovery.phase = SPL_RECOVER_FAILED;
    spl_recovery.phase_ms = HAL_GetTick();
    return 1;
}

static void spl006_read_fault(void)
{
    /* 连续读失败计数上限为 3，只在刚好累计到 3 时记录起始时刻。 */
    if (spl_recovery.faults < 3 && ++spl_recovery.faults == 3)
    {
        spl_recovery.fault_ms = HAL_GetTick();
    }
}

static void spl006_decode_coefficients(void)
{
    /* 指向 16 字节原始系数，便于按偏移取值。 */
    const uint8_t *c = spl_recovery.coef;
    /* C00 为 20 位：c0 全 8 位、c1 全 8 位、c2 高 4 位。 */
    int32_t raw = ((int32_t)c[0] << 12) | ((int32_t)c[1] << 4) | (c[2] >> 4);
    /* 20 位补码最高位为 1 时做符号扩展。 */
    if ((raw & 0x80000) != 0)
    {
        raw |= (int32_t)0xFFF00000;
    }
    spl006_c00 = raw;
    /* C10 为 20 位：c3 低 4 位为最高半字节，再拼 c4、c5。 */
    raw = ((int32_t)(c[3] & 0x0F) << 16) | ((int32_t)c[4] << 8) | c[5];
    /* 同样按 20 位补码做符号扩展。 */
    if ((raw & 0x80000) != 0)
    {
        raw |= (int32_t)0xFFF00000;
    }
    spl006_c10 = raw;
    /* 其余系数均为 16 位有符号数，高字节在前拼接。 */
    spl006_c01 = (int16_t)(((uint16_t)c[6] << 8) | c[7]);
    spl006_c11 = (int16_t)(((uint16_t)c[8] << 8) | c[9]);
    spl006_c20 = (int16_t)(((uint16_t)c[10] << 8) | c[11]);
    spl006_c21 = (int16_t)(((uint16_t)c[12] << 8) | c[13]);
    spl006_c30 = (int16_t)(((uint16_t)c[14] << 8) | c[15]);
}

uint8_t spl006_recover_step(uint32_t now_ms)
{
    /* 保留原逐寄存器顺序，0x15的共享字节也与旧路径一样读取两次。 */
    static const uint8_t coef_regs[] = {
        SPL06_C00_BYTE2, SPL06_C00_BYTE1, SPL06_C00_BYTE0,
        SPL06_C00_BYTE0, SPL06_C10_BYTE1, SPL06_C10_BYTE0,
        SPL06_C01_BYTE1, SPL06_C01_BYTE0, SPL06_C11_BYTE1, SPL06_C11_BYTE0,
        SPL06_C20_BYTE1, SPL06_C20_BYTE0, SPL06_C21_BYTE1, SPL06_C21_BYTE0,
        SPL06_C30_BYTE1, SPL06_C30_BYTE0};
    /* 压力、温度各按 16 倍过采样 1Hz 配置，CFG 寄存器按过采样倍率设置结果右移，最后启动背景测量。 */
    static const uint8_t config[][2] = {
        {SPL06_PRS_CFG, SPL06_PM_PRC_16X | SPL06_PM_RATE_1PS},
        {SPL06_TMP_CFG, SPL06_TMP_EXT | SPL06_PM_RATE_1PS | SPL06_TMP_PRC_16X},
        {SPL06_CFG_REG, SPL06_P_SHIFT | SPL06_T_SHIFT},
        {SPL06_MEAS_CFG, SPL06_MEAS_BG_PRSTMP}};
    /* 读取 MEAS_CFG 得到的状态字节。 */
    uint8_t status;
    /* 休眠期间完全冻结恢复流程。 */
    if (spl_recovery.sleeping != 0)
    {
        return 2; /* standby期间不推进恢复，避免重新启动背景转换。 */
    }
    /* 已就绪时只做健康检查。 */
    if (spl006_coef_ready != 0)
    {
        /* 数据过旧或连续读失败且已过 1s，都触发重新初始化。 */
        if ((uint32_t)(now_ms - spl_recovery.sample_ms) >= SPL006_SAMPLE_MAX_AGE_MS ||
            (spl_recovery.faults >= 3 && (uint32_t)(now_ms - spl_recovery.fault_ms) >= 1000))
        {
            spl006_recover_begin();
            return 2;
        }
        return 0;
    }
    /* 按阶段推进初始化流程。 */
    switch (spl_recovery.phase)
    {
    case SPL_RECOVER_FAILED:
        /* 失败后等 1s 再整轮重试，期间返回 1。 */
        if ((uint32_t)(now_ms - spl_recovery.phase_ms) >= 1000)
        {
            spl006_recover_begin();
            return 2;
        }
        return 1;
    case SPL_RECOVER_ID:
        /* 读器件 ID 寄存器，仅为确认总线可通信，失败即进入失败阶段。 */
        if (spl006_read_reg(SPL06_ID, &status) != 0)
        {
            return spl006_recover_failed();
        }
        spl_recovery.phase = SPL_RECOVER_RESET;
        return 2;
    case SPL_RECOVER_RESET:
        /* 写软复位命令，成功则进入复位等待阶段并记录起点。 */
        if (spl006_write_reg(SPL06_RESET, SPL06_SOFT_RESET) != 0)
        {
            return spl006_recover_failed();
        }
        spl_recovery.phase = SPL_RECOVER_SENSOR;
        spl_recovery.phase_ms = HAL_GetTick();
        return 2;
    case SPL_RECOVER_SENSOR:
    case SPL_RECOVER_COEF_READY:
        /* 两阶段各自以phase_ms计时，分别最多等待500ms。 */
        if ((uint32_t)(now_ms - spl_recovery.phase_ms) >= 500)
        {
            return spl006_recover_failed();
        }
        /* 传感器至少等 20ms、系数至少等 50ms，未到点就先返回“继续”。 */
        if ((spl_recovery.phase == SPL_RECOVER_SENSOR && (uint32_t)(now_ms - spl_recovery.phase_ms) < 20) ||
            (spl_recovery.phase == SPL_RECOVER_COEF_READY && (uint32_t)(now_ms - spl_recovery.phase_ms) < 50))
        {
            return 2;
        }
        /* 读 MEAS_CFG 状态，失败本轮继续等待，不计入失败。 */
        if (spl006_read_reg(SPL06_MEAS_CFG, &status) != 0)
        {
            return 2;
        }
        /* 传感器阶段等到 SENSOR_RDY 后转入系数等待阶段。 */
        if (spl_recovery.phase == SPL_RECOVER_SENSOR && (status & SPL06_SENSOR_RDY) != 0)
        {
            spl_recovery.phase = SPL_RECOVER_COEF_READY;
            spl_recovery.phase_ms = HAL_GetTick();
        }
        /* 系数阶段等到 COEF_RDY 后进入系数读取阶段。 */
        else if (spl_recovery.phase == SPL_RECOVER_COEF_READY && (status & SPL06_COEF_RDY) != 0)
        {
            spl_recovery.phase = SPL_RECOVER_COEFFICIENTS;
        }
        return 2;
    case SPL_RECOVER_COEFFICIENTS:
        /* 每次读一个系数寄存器，直接存入 coef 数组对应位置。 */
        if (spl006_read_reg(coef_regs[spl_recovery.index], &spl_recovery.coef[spl_recovery.index]) != 0)
        {
            return spl006_recover_failed();
        }
        /* 下标推进到表尾说明 16 字节已读完，解析系数并转入配置阶段。 */
        if (++spl_recovery.index >= sizeof(coef_regs))
        {
            spl006_decode_coefficients();
            spl_recovery.index = 0;
            spl_recovery.phase = SPL_RECOVER_CONFIG;
        }
        return 2;
    case SPL_RECOVER_CONFIG:
        /* 按表逐项写测量配置寄存器，失败即进入失败阶段。 */
        if (spl006_write_reg(config[spl_recovery.index][0], config[spl_recovery.index][1]) != 0)
        {
            return spl006_recover_failed();
        }
        /* 配置表全部写完：标记系数就绪、进入完成阶段并记录有效帧起点。 */
        if (++spl_recovery.index >= sizeof(config) / sizeof(config[0]))
        {
            spl006_coef_ready = 1;
            spl_recovery.phase = SPL_RECOVER_DONE;
            spl_recovery.sample_ms = HAL_GetTick();
            return 0;
        }
        return 2;
    default:
        /* 其余未知阶段按失败返回，不继续推进。 */
        return 1;
    }
}

uint8_t spl006_set_sleep(uint8_t sleeping)
{
    /* 默认写回背景压力/温度测量配置。 */
    uint8_t mode = SPL06_MEAS_BG_PRSTMP;
    /* 唤醒时丢弃的旧 P/T 数据缓冲区。 */
    uint8_t discard[6];
    /* 本次操作结果，0 成功、1 失败。 */
    uint8_t result = 0;
    /* 当前时刻、本次休眠持续时长，单位 ms。 */
    uint32_t now_ms, elapsed_ms;
    /* 请求休眠时改为写 standby（0），停止背景转换。 */
    if (sleeping != 0)
    {
        mode = 0; /* MEAS_CFG: standby，停止背景转换，保留出厂系数。 */
    }
    /* 唤醒且系数已就绪时，先读走旧数据，避免误用休眠前的就绪标志。 */
    else if (spl006_coef_ready != 0)
    {
        /* 清掉休眠前的P/T就绪标志，调用方等待新的1Hz转换后再读取。 */
        if (spl006_bus_read(SPL06_PRS_B2, discard, sizeof(discard)) != 0)
        {
            result = 1;
        }
    }
    /* 恢复尚未完成时，唤醒只解除暂停；背景转换由最后一个配置步骤启动。 */
    /* 只有前面步骤成功、且确需改测量模式时才写 MEAS_CFG。 */
    if (result == 0 && (sleeping != 0 || spl006_coef_ready != 0) && spl006_write_reg(SPL06_MEAS_CFG, mode) != 0)
    {
        result = 1;
    }
    now_ms = HAL_GetTick();
    /* 从工作进入休眠：记录休眠起点并置标志。 */
    if (sleeping != 0 && spl_recovery.sleeping == 0)
    {
        spl_recovery.sleep_ms = now_ms;
        spl_recovery.sleeping = 1;
    }
    /* 从休眠回到工作：把休眠时长补回各项超时基准。 */
    else if (sleeping == 0 && spl_recovery.sleeping != 0)
    {
        /* standby只暂停故障和恢复等待，跨窗口的未就绪/NACK必须继续累计。 */
        elapsed_ms = now_ms - spl_recovery.sleep_ms;
        spl_recovery.sample_ms += elapsed_ms;
        spl_recovery.phase_ms += elapsed_ms;
        /* 已累计到 3 次失败时，故障计时同样顺延。 */
        if (spl_recovery.faults >= 3)
        {
            spl_recovery.fault_ms += elapsed_ms;
        }
        spl_recovery.sleeping = 0;
    }
    /* 写配置或清数据失败时重启恢复流程，再返回失败。 */
    if (result != 0)
    {
        spl006_recover_begin();
    }
    return result;
}

uint8_t spl006_init(void)
{
    /* 每次 step 的返回值：0 成功、1 失败、2 继续。 */
    uint8_t result;
    /* 初始化会重新配置并启动测量，先清除休眠标志让状态机可以推进。 */
    spl_recovery.sleeping = 0;
    /* 清状态并从 ID 阶段开始。 */
    spl006_recover_begin();
    /* 持续推进状态机，直到不再返回“继续”。 */
    while ((result = spl006_recover_step(HAL_GetTick())) == 2)
    {
        /* 传感器等待阶段每次让出 20ms，对应最小上电稳定时间。 */
        if (spl_recovery.phase == SPL_RECOVER_SENSOR)
        {
            spl006_delay_ms(20);
        }
        /* 系数等待阶段每次让出 50ms，给器件准备系数的时间。 */
        else if (spl_recovery.phase == SPL_RECOVER_COEF_READY)
        {
            spl006_delay_ms(50);
        }
    }
    return result;
}

uint8_t spl006_read_pressure(int32_t *pressure_raw)
{
    /* MEAS_CFG 状态字节。 */
    uint8_t meas_cfg;
    /* 压力 3 字节原始值，高字节在前。 */
    uint8_t buf[3];

    /* 先读状态寄存器，总线失败返回 2。 */
    if (spl006_read_reg(SPL06_MEAS_CFG, &meas_cfg) != 0)
    {
        return 2;
    }
    /* 压力就绪位未置位时返回 1，表示暂无新数据。 */
    if ((meas_cfg & SPL06_PRS_RDY) == 0)
    {
        return 1;
    }

    /* 突发读 3 字节 (0x00~0x02), 替代 3 次单字节读 */
    if (spl006_bus_read(SPL06_PRS_B2, buf, 3) != 0)
    {
        return 2;
    }

    /* 高字节在前拼成 24 位原始压力值。 */
    *pressure_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    /* 最高位为 1 说明是负数，补足高 8 位完成 24 位补码符号扩展。 */
    if (*pressure_raw & 0x800000)
    {
        *pressure_raw |= (int32_t)0xFF000000; /* 24bit 补码符号扩展 */
    }
    return 0;
}

uint8_t spl006_read_temperature(int32_t *temperature_raw)
{
    /* MEAS_CFG 状态字节。 */
    uint8_t meas_cfg;
    /* 温度 3 字节原始值，高字节在前。 */
    uint8_t buf[3];

    /* 先读状态寄存器，总线失败返回 2。 */
    if (spl006_read_reg(SPL06_MEAS_CFG, &meas_cfg) != 0)
    {
        return 2;
    }
    /* 温度就绪位未置位时返回 1，表示暂无新数据。 */
    if ((meas_cfg & SPL06_TMP_RDY) == 0)
    {
        return 1;
    }

    /* 突发读 3 字节 (0x03~0x05) */
    if (spl006_bus_read(SPL06_TMP_B2, buf, 3) != 0)
    {
        return 2;
    }

    /* 高字节在前拼成 24 位原始温度值。 */
    *temperature_raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    /* 最高位为 1 说明是负数，补足高 8 位完成 24 位补码符号扩展。 */
    if (*temperature_raw & 0x800000)
    {
        *temperature_raw |= (int32_t)0xFF000000;
    }
    return 0;
}

uint16_t spl006_read_sample(int32_t *pressure_raw, int32_t *temperature_raw)
{
    /* MEAS_CFG 状态字节与 6 字节 P/T 原始数据。 */
    uint8_t status, data[6];
    /* 结果位图：bit0 压力未就绪，bit8 温度未就绪。 */
    uint16_t result = 0;
    /* 本帧解析出的压力、温度原始值。 */
    int32_t pressure, temperature;
    /* 系数尚未就绪时无法提供有效样本，返回 0x0202。 */
    if (spl006_coef_ready == 0)
    {
        return 0x0202;
    }
    /* 读状态寄存器失败，累计一次读故障并返回 0x0202。 */
    if (spl006_read_reg(SPL06_MEAS_CFG, &status) != 0)
    {
        spl006_read_fault();
        return 0x0202;
    }
    /* 压力未就绪时在结果里置 bit0。 */
    if ((status & SPL06_PRS_RDY) == 0)
    {
        result |= 1;
    }
    /* 温度未就绪时在结果里置 bit8。 */
    if ((status & SPL06_TMP_RDY) == 0)
    {
        result |= 0x0100;
    }
    /* 任一未就绪就不读数据，避免提前消耗单个就绪标志。 */
    if (result != 0)
    {
        spl_recovery.faults = 0;
        return result; /* 等整对数据就绪，不提前消耗单个就绪标志。 */
    }
    /* 一次突发读 6 字节，前 3 字节压力、后 3 字节温度。 */
    if (spl006_bus_read(SPL06_PRS_B2, data, sizeof(data)) != 0)
    {
        spl006_read_fault();
        return 0x0202;
    }
    /* 高字节在前拼成 24 位压力原始值。 */
    pressure = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    /* 高字节在前拼成 24 位温度原始值。 */
    temperature = ((int32_t)data[3] << 16) | ((int32_t)data[4] << 8) | data[5];
    /* 压力最高位为 1 时做 24 位补码符号扩展。 */
    if ((pressure & 0x800000) != 0)
    {
        pressure |= (int32_t)0xFF000000;
    }
    /* 温度最高位为 1 时做 24 位补码符号扩展。 */
    if ((temperature & 0x800000) != 0)
    {
        temperature |= (int32_t)0xFF000000;
    }
    /* 整对数据读取成功，清零故障计数并记录有效帧时刻。 */
    spl_recovery.faults = 0;
    spl_recovery.sample_ms = HAL_GetTick();
    /* 写回两路原始值，返回 0 表示成功。 */
    *pressure_raw = pressure;
    *temperature_raw = temperature;
    return 0;
}

/* ------------------------------------------------------------ 温度补偿 */
uint8_t spl006_process_pressure(int32_t pressure_raw, int32_t temperature_raw, int32_t *compensated_pressure)
{
    /* 按过采样倍率归一化后的压力、温度值。 */
    float p_sc, t_sc;

    /* 系数未就绪时补偿结果不可信，返回失败。 */
    if (spl006_coef_ready == 0)
    {
        return 1; /* 系数未就绪, 输出不可信 */
    }

    /* 原始值除以 16 倍过采样比例，换算成补偿公式所需的标度值。 */
    p_sc = (float)pressure_raw / SPL06_SCALE_FACTOR_16X;
    t_sc = (float)temperature_raw / SPL06_SCALE_FACTOR_16X;

    /* 按手册多项式计算：压力三次项 + 温度一次项 + 温压交叉项，最后取整。 */
    *compensated_pressure =
        (int32_t)((float)spl006_c00 + p_sc * ((float)spl006_c10 + p_sc * ((float)spl006_c20 + p_sc * (float)spl006_c30)) +
                  t_sc * (float)spl006_c01 + t_sc * p_sc * ((float)spl006_c11 + p_sc * (float)spl006_c21));
    return 0;
}
