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
    /* 以磁力计地址发起“先写寄存器号再连续读”的事务，返回值 0 表示成功。 */
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
/* 分步恢复状态机的私有上下文，记录各阶段计时与故障累计。 */
static struct
{
    /* 阶段起点、故障起点、最近有效帧、进入休眠时刻，单位 ms。 */
    uint32_t phase_ms, fault_ms, sample_ms, sleep_ms;
    /* 当前阶段、连续读失败数、无新数据标志、休眠标志。 */
    uint8_t phase, faults, no_data, sleeping;
} mag_recovery;

void lsm303dlhc_mag_recover_begin(void)
{
    /* 重新配置前先标记未就绪，读取接口会立即拒绝。 */
    s_ready = 0;
    /* 恢复默认 75Hz 速率记录，配置成功前不对外声称其他速率。 */
    s_rate_hz_x10 = 750;
    /* 从器件 ID 校验阶段重新开始。 */
    mag_recovery.phase = MAG_RECOVER_ID;
    mag_recovery.faults = 0;
    mag_recovery.no_data = 0;
    mag_recovery.sleeping = 0;
    /* 记录阶段起点，供失败后退避计时使用。 */
    mag_recovery.phase_ms = HAL_GetTick();
}

static uint8_t mag_recover_failed(void)
{
    /* 任一配置步骤失败都清就绪标志，进入失败阶段并记录时刻。 */
    s_ready = 0;
    mag_recovery.phase = MAG_RECOVER_FAILED;
    mag_recovery.phase_ms = HAL_GetTick();
    return 1;
}

static void mag_read_fault(void)
{
    /* 连续读失败计数上限为 3，只在刚好累计到 3 时记录起始时刻。 */
    if (mag_recovery.faults < 3 && ++mag_recovery.faults == 3)
    {
        mag_recovery.fault_ms = HAL_GetTick();
    }
}

uint8_t lsm303dlhc_mag_recover_step(uint32_t now_ms)
{
    /* 依次写入 CRA、CRB、MR_REG：75Hz、增益配置、连续转换模式。 */
    static const uint8_t config[] = {0x18, 0x20, 0x00};
    /* 与 config 一一对应的寄存器号，按 phase 顺序取用。 */
    static const uint8_t regs[] = {MAG_CRA, MAG_CRB, MAG_MODE};
    /* ID_A/ID_B/ID_C 三个识别字节。 */
    uint8_t id[3];
    /* 由阶段号换算出的配置数组下标。 */
    uint8_t index;
    /* 已就绪时只做健康检查，不再重复配置。 */
    if (s_ready != 0)
    {
        /* 长时间无新数据或连续读失败且已过 1s，都触发重新配置。 */
        if ((mag_recovery.no_data != 0 && (uint32_t)(now_ms - mag_recovery.sample_ms) >= MAG_SAMPLE_TIMEOUT_MS) ||
            (mag_recovery.faults >= 3 && (uint32_t)(now_ms - mag_recovery.fault_ms) >= 1000))
        {
            lsm303dlhc_mag_recover_begin();
            return 2;
        }
        return 0;
    }
    /* 处于失败阶段时等 1s 再自动重试，期间返回 1。 */
    if (mag_recovery.phase == MAG_RECOVER_FAILED)
    {
        /* 1s 到点后重新开始整轮恢复。 */
        if ((uint32_t)(now_ms - mag_recovery.phase_ms) >= 1000)
        {
            lsm303dlhc_mag_recover_begin();
            return 2;
        }
        return 1;
    }
    /* ID 阶段：读 3 个识别字节并与 0x48/0x34/0x33 比对。 */
    if (mag_recovery.phase == MAG_RECOVER_ID)
    {
        /* 读失败或任一识别字节不符，进入失败阶段。 */
        if (mag_read(MAG_ID_A, id, 3) != 0 || id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33)
        {
            return mag_recover_failed();
        }
        /* 身份确认，进入 CRA 配置阶段。 */
        mag_recovery.phase = MAG_RECOVER_CRA;
        return 2;
    }
    /* 阶段号减 1 正好映射到 config/regs 的下标。 */
    index = mag_recovery.phase - 1;
    /* 写当前阶段的配置字节，失败则进入失败阶段。 */
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, regs[index], &config[index], 1) != 0)
    {
        return mag_recover_failed();
    }
    /* 推进到下一阶段；自增后正好等于 FAILED 说明三个寄存器都已写完。 */
    if (++mag_recovery.phase == MAG_RECOVER_FAILED)
    {
        /* 配置全部完成：标记就绪、记录有效帧起点并返回 0。 */
        mag_recovery.phase = MAG_RECOVER_DONE;
        s_ready = 1;
        mag_recovery.sample_ms = HAL_GetTick();
        return 0;
    }
    return 2;
}

uint8_t lsm303dlhc_mag_init(void)
{
    /* 每次 step 的返回值：0 成功、1 失败、2 继续。 */
    uint8_t result;
    /* 清状态并从 ID 阶段开始。 */
    lsm303dlhc_mag_recover_begin();
    /* 循环调用 step，直到返回值不再是 2（继续）。 */
    while ((result = lsm303dlhc_mag_recover_step(HAL_GetTick())) == 2)
    {
    }
    return result;
}

uint8_t lsm303dlhc_mag_set_sleep(uint8_t sleeping)
{
    /* MR_REG_M 写入值：0 为连续转换，0x03 为休眠。 */
    uint8_t mode = 0;
    /* 当前时刻、本次休眠持续时长，单位 ms。 */
    uint32_t now_ms, elapsed_ms;
    /* 请求休眠时把模式值改为 0x03。 */
    if (sleeping != 0)
    {
        mode = 0x03; /* MR_REG_M: sleep；唤醒恢复连续转换。 */
    }
    /* 写模式寄存器失败时重新走恢复流程，并返回失败。 */
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, MAG_MODE, &mode, 1) != 0)
    {
        lsm303dlhc_mag_recover_begin();
        return 1;
    }
    now_ms = HAL_GetTick();
    /* 从唤醒进入休眠：记下休眠起点并置休眠标志。 */
    if (sleeping != 0 && mag_recovery.sleeping == 0)
    {
        mag_recovery.sleep_ms = now_ms;
        mag_recovery.sleeping = 1;
    }
    /* 从休眠回到工作：把休眠时长补回各项超时基准。 */
    else if (sleeping == 0 && mag_recovery.sleeping != 0)
    {
        /* 休眠只暂停超时，保留跨唤醒的读失败及无数据状态，直到真正取得新帧。 */
        elapsed_ms = now_ms - mag_recovery.sleep_ms;
        mag_recovery.sample_ms += elapsed_ms;
        /* 已累计到 3 次失败时，故障计时同样顺延。 */
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
    /* 待写入 CRA 的速率配置字节。 */
    uint8_t config;
    /* 只接受 75.0、30.0、7.5Hz 三档（单位 0.1Hz）。 */
    if (rate_hz_x10 != 750 && rate_hz_x10 != 300 && rate_hz_x10 != 75)
    {
        return 1;
    }
    /* 未就绪时不允许改速率。 */
    if (s_ready == 0)
    {
        return 1;
    }
    /* 与当前速率相同则无需写寄存器。 */
    if (s_rate_hz_x10 == rate_hz_x10)
    {
        return 0;
    }
    config = 0x14; /* 30Hz，保留温度关闭。 */
    /* 请求 75Hz 时改用运动模式配置。 */
    if (rate_hz_x10 == 750)
    {
        config = 0x18; /* 运动模式恢复75Hz。 */
    }
    /* 请求 7.5Hz 时用最低档配置。 */
    else if (rate_hz_x10 == 75)
    {
        config = 0x0C; /* 7.5Hz。 */
    }
    /* 写 CRA 失败则重新恢复，并返回失败。 */
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, LSM303DLHC_MAG_ADDR, MAG_CRA, &config, 1) != 0)
    {
        lsm303dlhc_mag_recover_begin();
        return 1;
    }
    /* 写成功后才更新记录的当前速率。 */
    s_rate_hz_x10 = rate_hz_x10;
    return 0;
}

lsm303dlhc_mag_result lsm303dlhc_mag_read(lsm303dlhc_mag_struct *mag)
{
    /* 状态字节与 6 字节原始输出。 */
    uint8_t status, buf[6];
    /* 先在本帧内组装，成功后再整体发布。 */
    lsm303dlhc_mag_struct sample;
    /* 三轴循环下标，0=X、1=Y、2=Z。 */
    uint8_t i;
    /* 当前轴的灵敏度，单位 LSB/gauss。 */
    float sensitivity;

    /* 未就绪或输出指针为空时直接返回错误。 */
    if (!s_ready || mag == NULL)
    {
        return LSM303DLHC_MAG_ERROR;
    }
    /* 先读状态寄存器，失败则累计读故障。 */
    if (mag_read(MAG_STATUS, &status, 1) != 0)
    {
        mag_read_fault();
        return LSM303DLHC_MAG_ERROR;
    }
    /* 默认先标记“尚未取得新帧”，成功发布时再清除。 */
    mag_recovery.no_data = 1;
    /* 状态低两位都为 0 表示没有新数据，清零连续故障计数并返回未就绪。 */
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
    /* 状态低两位不为0x01（DRDY=1、LOCK=0）时，本轮只排空、不发布样本。 */
    if ((status & 0x03) != 0x01)
    {
        /* LOCK表示上次只读了部分输出；排空解锁，不将残留帧重新计为新样本。 */
        return LSM303DLHC_MAG_NOT_READY;
    }

    /* 寄存器为X高/低、Z高/低、Y高/低；快照统一存成X/Y/Z。 */
    sample.raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    /* 第 3、4 字节是 Z 轴，存入下标 2。 */
    sample.raw[2] = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    /* 第 5、6 字节是 Y 轴，存入下标 1。 */
    sample.raw[1] = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    /* 逐轴做范围检查并换算成微特斯拉。 */
    for (i = 0; i < 3; ++i)
    {
        /* 正常输出 -2048..2047；包含 -4096 溢出码在内的异常值不能融合。 */
        if (sample.raw[i] < -2048 || sample.raw[i] > 2047)
        {
            return LSM303DLHC_MAG_ERROR;
        }
        /* Z 轴灵敏度与 X/Y 不同，按轴选择换算系数。 */
        if (i == 2)
        {
            sensitivity = 980.0f;
        }
        else
        {
            sensitivity = 1100.0f;
        }
        /* 原始值乘 100 再除灵敏度，得到以 uT 为单位的磁场值。 */
        sample.field_uT[i] = sample.raw[i] * 100.0f / sensitivity;
    }
    /* 只有整帧读取和转换成功才发布，失败时保留调用方的上一帧。 */
    mag_recovery.faults = 0;
    mag_recovery.no_data = 0;
    mag_recovery.sample_ms = HAL_GetTick();
    *mag = sample;
    return LSM303DLHC_MAG_OK;
}
