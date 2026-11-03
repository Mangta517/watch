/**
 * @file em7028.h
 * @brief EM7028 HRS1 连续模式；SENSOR 软件 I2C，PB15 控制外部心率照明。
 */
#ifndef __EM7028_H__
#define __EM7028_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define EM7028_ADDR             0x24 /* 7 位地址，线上写/读地址为 0x48/0x49 */
#define EM7028_PRODUCT_ID       0x36
#define EM7028_SAMPLE_PERIOD_MS 25 /* 手册第8页16位连续转换的典型时间；实际周期需实测 */

    typedef enum
    {
        EM7028_OK,
        EM7028_NOT_READY, /* 未启动、首帧等待中，或读取时跨过数据更新 */
        EM7028_ERROR      /* 参数、识别、总线或配置失败 */
    } em7028_result;

    typedef enum
    {
        EM7028_FAULT_NONE,
        EM7028_FAULT_READ,
        EM7028_FAULT_WRITE,
        EM7028_FAULT_PID,
        EM7028_FAULT_CONFIG,
        EM7028_FAULT_DATA_CHANGED
    } em7028_fault_kind;

    typedef struct
    {
        em7028_fault_kind kind;
        uint8_t reg;
        uint8_t value; /* PID/配置不符时的读回值；通信失败时无效 */
    } em7028_fault;

    /* MX_GPIO_Init 和 my_i2c_init(MY_I2C_BUS_SENSOR) 后调用。
 * PID 不匹配时最多读取三次；识别成功后关闭模式及中断，配置 1 倍增益/RANGE=8/最高时钟/16 位/HRS1。
 * 不做软复位或猜测其等待时间；成功后保持采样及外部 LED 关闭。
 * sensor 任务可串行重新初始化，UI/ISR 不得调用。 */
    em7028_result em7028_init(void);

    /* 调度器启动后，这些接口仅由 sensor 任务串行调用，禁止 UI/ISR 直接访问总线。
 * start 在每次重新测量前确认 PID 并配置，然后开启 HRS1 和外部照明。
 * stop 总会先关闭外部照明，再尝试关闭芯片采样。
 * stop 返回 ERROR 时芯片采样可能仍在运行，应重新尝试停止或初始化。 */
    em7028_result em7028_start(void);
    em7028_result em7028_stop(void);

    /* 读取主寄存器表中的 HRS1_DATA0（0x28 低字节、0x29 高字节）。
 * 高/低/高单字节读，跨高字节更新时返回 NOT_READY；失败时不改调用方输出。
 * start 后至少等 SAMPLE_PERIOD_MS；之后由调用方定时采样。没有数据就绪标志，
 * 重复读值不表示新样本；返回无符号原始光强值，不是 BPM。 */
    em7028_result em7028_read_raw(uint16_t *raw);

    /* 仅 sensor 任务取出故障并复制到 UI 快照；此接口不访问总线。 */
    void em7028_get_fault(em7028_fault *fault);

#ifdef __cplusplus
}
#endif

#endif
