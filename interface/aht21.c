/**
 * @file aht21.c
 * @brief 奥松 AHT21 手册 V1.2，第 8～9 页：初始化、触发、直接读和 CRC8。
 */
#include "aht21.h"
#include "my_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

#define AHT21_STATUS_BUSY       0x80
#define AHT21_STATUS_CALIBRATED 0x08
/* 熄屏每100ms才推进一拍，五个恢复阶段及调度余量须落在1.6s环境窗口内。 */
#define AHT21_RECOVERY_TIMEOUT_MS 1000

/* 毫秒延时封装：调度器运行时让出 CPU，未运行时退回 HAL 忙等。 */
static void aht21_delay_ms(uint32_t ms)
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

/* 0x71 是读地址字节，不是寄存器号；不能使用先写寄存器的 read_bytes。 */
static aht21_result aht21_read_bytes(uint8_t *data, uint8_t size)
{
    /* 已读取的字节下标，最后一字节单独处理以发送 NACK。 */
    uint8_t i;
    /* 先准备 SENSOR 总线的 GPIO 方向，失败直接返回总线错误。 */
    if (my_i2c_prepare(MY_I2C_BUS_SENSOR) != 0)
    {
        return AHT21_I2C_ERROR;
    }
    /* 产生起始条件，开始一次读事务。 */
    my_i2c_start(MY_I2C_BUS_SENSOR);
    /* 发送 7 位地址左移后或上读位 1，即手册的 0x71 读地址。 */
    my_i2c_write_byte(MY_I2C_BUS_SENSOR, (uint8_t)((AHT21_ADDR << 1) | 1));
    /* 等待器件应答；无应答说明器件不在线或总线异常。 */
    if (my_i2c_wait_ack(MY_I2C_BUS_SENSOR) != 0)
    {
        /* 释放总线后再返回，避免后续调用者卡在未结束的事务里。 */
        my_i2c_stop(MY_I2C_BUS_SENSOR);
        return AHT21_I2C_ERROR;
    }
    /* 逐字节接收，最后 1 字节需回 NACK 通知器件结束发送。 */
    for (i = 0; i < size; ++i)
    {
        /* 判断是否已到最后一字节，决定主机回 ACK 还是 NACK。 */
        if (i == size - 1)
        {
            data[i] = my_i2c_read_byte(MY_I2C_BUS_SENSOR, 1);
        }
        else
        {
            data[i] = my_i2c_read_byte(MY_I2C_BUS_SENSOR, 0);
        }
    }
    /* 发送停止条件，结束本次读事务并释放总线。 */
    my_i2c_stop(MY_I2C_BUS_SENSOR);
    return AHT21_OK;
}

static uint8_t aht21_crc8(const uint8_t *data)
{
    /* CRC 初值按手册取 0xFF。 */
    uint8_t crc = 0xFF;
    /* i 为参与校验的字节下标，bit 为当前字节内的位下标。 */
    uint8_t i, bit;
    /* 依次把 6 个数据字节并入 CRC。 */
    for (i = 0; i < 6; ++i)
    {
        crc ^= data[i];
        /* 对每个字节逐位处理 8 次。 */
        for (bit = 0; bit < 8; ++bit)
        {
            /* 判断最高位，为 1 时左移后异或多项式 0x31。 */
            if ((crc & 0x80) != 0)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            }
            else
            {
                /* 最高位为 0 时只左移，不做异或。 */
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

aht21_result aht21_init(void)
{
    /* 校准使能命令 0xBE 后跟的两个字节：0x08 置校准位，0x00 为保留字节。 */
    static const uint8_t config[2] = {0x08, 0x00};
    /* 读回的状态字节，bit7 为忙、bit3 为已校准。 */
    uint8_t status;
    /* 上电后先等 40ms，让器件完成内部启动。 */
    aht21_delay_ms(40);
    /* 读取当前状态，读失败按总线错误返回。 */
    if (aht21_read_bytes(&status, 1) != AHT21_OK)
    {
        return AHT21_I2C_ERROR;
    }
    /* 检查忙标志，器件仍忙时本次初始化不能继续。 */
    if ((status & AHT21_STATUS_BUSY) != 0)
    {
        return AHT21_BUSY;
    }
    /* 只有状态显示尚未校准时才下发校准命令。 */
    if ((status & AHT21_STATUS_CALIBRATED) == 0)
    {
        /* 向 0xBE 命令寄存器写入 2 字节校准参数。 */
        if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, AHT21_ADDR, 0xBE, config, sizeof(config)) != 0)
        {
            return AHT21_I2C_ERROR;
        }
        /* 校准命令后等 10ms 再回读状态。 */
        aht21_delay_ms(10);
        /* 回读状态，失败仍按总线错误处理。 */
        if (aht21_read_bytes(&status, 1) != AHT21_OK)
        {
            return AHT21_I2C_ERROR;
        }
        /* 回读时器件仍忙，返回忙状态由调用方决定是否重试。 */
        if ((status & AHT21_STATUS_BUSY) != 0)
        {
            return AHT21_BUSY;
        }
        /* 忙标志已清但仍未置校准位，说明校准未成功。 */
        if ((status & AHT21_STATUS_CALIBRATED) == 0)
        {
            return AHT21_CALIBRATION_ERROR;
        }
    }
    return AHT21_OK;
}

enum
{
    AHT_RECOVER_WAIT_POWER,
    AHT_RECOVER_STATUS,
    AHT_RECOVER_CONFIG,
    AHT_RECOVER_VERIFY_WAIT,
    AHT_RECOVER_VERIFY,
    AHT_RECOVER_DONE
};
/* 分步恢复状态机的私有上下文，由 recover_begin/recover_step 的调用序列驱动。 */
static struct
{
    /* 本次恢复起点、当前阶段起点，单位均为 ms tick。 */
    uint32_t started_ms, phase_ms;
    /* 当前所处阶段，取 AHT_RECOVER_* 枚举值。 */
    uint8_t phase;
    /* 完成后保留的最终结果，DONE 阶段直接返回它。 */
    aht21_result result;
} aht_recovery;

void aht21_recover_begin(uint32_t now_ms)
{
    /* 从最前面的上电等待阶段重新开始。 */
    aht_recovery.phase = AHT_RECOVER_WAIT_POWER;
    /* 总超时与阶段超时共用同一时间基准。 */
    aht_recovery.started_ms = aht_recovery.phase_ms = now_ms;
    /* 未完成前对外报告忙，避免调用方把中途状态当成成功。 */
    aht_recovery.result = AHT21_BUSY;
}

aht21_result aht21_recover_step(uint32_t now_ms)
{
    /* 与 aht21_init 相同的校准使能命令：0x08 置校准位，0x00 为保留字节。 */
    static const uint8_t config[2] = {0x08, 0x00};
    /* 每次状态阶段读回的状态字节。 */
    uint8_t status;
    /* 本次步骤要记录并返回的失败原因。 */
    aht21_result result;
    /* 已到 DONE 阶段说明恢复结束，直接返回保存的结果。 */
    if (aht_recovery.phase == AHT_RECOVER_DONE)
    {
        return aht_recovery.result;
    }
    /* 总耗时达到上限，后续不再执行任何总线操作，统一按超时结束。 */
    if ((uint32_t)(now_ms - aht_recovery.started_ms) >= AHT21_RECOVERY_TIMEOUT_MS)
    {
        result = AHT21_TIMEOUT;
    }
    /* 第一阶段：上电等待，满 40ms 才允许访问器件。 */
    else if (aht_recovery.phase == AHT_RECOVER_WAIT_POWER)
    {
        /* 阶段计时达到 40ms 后推进到状态检查阶段。 */
        if ((uint32_t)(now_ms - aht_recovery.phase_ms) >= 40)
        {
            aht_recovery.phase = AHT_RECOVER_STATUS;
        }
        return AHT21_BUSY;
    }
    /* 配置阶段：写校准使能命令，成功后进入写后等待。 */
    else if (aht_recovery.phase == AHT_RECOVER_CONFIG)
    {
        /* 写 0xBE 命令及 2 字节参数，失败则记录总线错误。 */
        if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, AHT21_ADDR, 0xBE, config, sizeof(config)) != 0)
        {
            result = AHT21_I2C_ERROR;
        }
        else
        {
            /* 写成功，记录当前时刻并进入 10ms 验证等待。 */
            aht_recovery.phase_ms = HAL_GetTick();
            aht_recovery.phase = AHT_RECOVER_VERIFY_WAIT;
            return AHT21_BUSY;
        }
    }
    /* 写后等待阶段：等满 10ms 再做状态回读。 */
    else if (aht_recovery.phase == AHT_RECOVER_VERIFY_WAIT)
    {
        /* 10ms 到点后切换到验证阶段，本轮仍返回忙。 */
        if ((uint32_t)(now_ms - aht_recovery.phase_ms) >= 10)
        {
            aht_recovery.phase = AHT_RECOVER_VERIFY;
        }
        return AHT21_BUSY;
    }
    else
    {
        /* 其余情况（状态检查或验证）都执行一次状态字节读取。 */
        result = aht21_read_bytes(&status, 1);
        if (result == AHT21_OK)
        {
            /* 器件忙时保持当前阶段，等下次调用再试。 */
            if ((status & AHT21_STATUS_BUSY) != 0)
            {
                return AHT21_BUSY;
            }
            /* 已不忙但未校准：状态阶段需转去下发配置，验证阶段则判失败。 */
            if ((status & AHT21_STATUS_CALIBRATED) == 0)
            {
                /* 状态阶段尚未配置过，转入配置阶段并继续等待。 */
                if (aht_recovery.phase == AHT_RECOVER_STATUS)
                {
                    aht_recovery.phase = AHT_RECOVER_CONFIG;
                    return AHT21_BUSY;
                }
                result = AHT21_CALIBRATION_ERROR;
            }
        }
    }
    /* 走到这里表示恢复已有最终结果：锁定阶段、保存结果并返回。 */
    aht_recovery.phase = AHT_RECOVER_DONE;
    aht_recovery.result = result;
    return result;
}

aht21_result aht21_trigger(void)
{
    /* 触发测量命令 0xAC 后跟 2 字节参数：0x33、0x00。 */
    static const uint8_t params[2] = {0x33, 0x00};
    /* 写入触发命令，失败按总线错误返回。 */
    if (my_i2c_write_bytes(MY_I2C_BUS_SENSOR, AHT21_ADDR, 0xAC, params, sizeof(params)) != 0)
    {
        return AHT21_I2C_ERROR;
    }
    return AHT21_OK;
}

aht21_result aht21_read(aht21_struct *sample)
{
    /* 一次读出状态字节 + 5 字节原始数据 + 1 字节 CRC。 */
    uint8_t data[7];
    /* 输出结构体指针为空时无法写回结果，直接拒绝。 */
    if (sample == NULL)
    {
        return AHT21_INVALID_ARGUMENT;
    }
    /* 连续读 7 字节，失败按总线错误返回。 */
    if (aht21_read_bytes(data, sizeof(data)) != AHT21_OK)
    {
        return AHT21_I2C_ERROR;
    }
    /* 状态字节 bit7 为忙，忙时数据无效。 */
    if ((data[0] & AHT21_STATUS_BUSY) != 0)
    {
        return AHT21_BUSY;
    }
    /* 用前 6 字节算 CRC，与第 7 字节比较，不一致说明数据传输出错。 */
    if (aht21_crc8(data) != data[6])
    {
        return AHT21_CRC_ERROR;
    }
    /* 状态字节 bit3 表示已校准，未校准时数据不可用。 */
    if ((data[0] & AHT21_STATUS_CALIBRATED) == 0)
    {
        return AHT21_CALIBRATION_ERROR;
    }
    /* 湿度为 20 位：data1 全 8 位、data2 全 8 位、data3 高 4 位。 */
    sample->humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
    /* 温度为 20 位：data3 低 4 位为高半字节，再拼 data4、data5。 */
    sample->temperature_raw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];
    /* 湿度原始值满量程 2^20 对应 100%RH。 */
    sample->humidity_rh = (float)sample->humidity_raw * 100.0f / 1048576.0f;
    /* 温度原始值满量程 2^20 对应 200℃，再减去 50℃ 偏移。 */
    sample->temperature_c = (float)sample->temperature_raw * 200.0f / 1048576.0f - 50.0f;
    /* 保留原始状态字节，供调用方查看其余标志位。 */
    sample->status = data[0];
    return AHT21_OK;
}
