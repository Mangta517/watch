/**
 * @file aht21.h
 * @brief AHT21 温湿度驱动；与现有传感器共用 PB14/PB13，由 sensor 任务独占访问。
 */
#ifndef __AHT21_H__
#define __AHT21_H__

#include <stdint.h>

#define AHT21_ADDR          0x38U
#define AHT21_CONVERSION_MS 80U

typedef enum
{
    AHT21_OK,
    AHT21_BUSY,
    AHT21_I2C_ERROR,
    AHT21_CALIBRATION_ERROR,
    AHT21_CRC_ERROR,
    AHT21_TIMEOUT,
    AHT21_INVALID_ARGUMENT
} aht21_result;

typedef struct
{
    uint32_t humidity_raw;    /* 20 位无符号原始值 */
    uint32_t temperature_raw; /* 20 位无符号原始值 */
    float humidity_rh;        /* %RH */
    float temperature_c;      /* ℃ */
    uint8_t status;
} aht21_struct;

/* 总线初始化后调用；仅启动阶段等待 40ms，必要时初始化再等待 10ms。 */
aht21_result aht21_init(void);
/* 仅发 AC 33 00，不等待转换；总线在 STOP 后可供其他传感器使用。 */
aht21_result aht21_trigger(void);
/* 触发至少 80ms 后调用；校验七字节帧，失败时不覆盖输出。 */
aht21_result aht21_read(aht21_struct *sample);

#endif
