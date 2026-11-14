#ifndef CLIMATE_TYPES_H
#define CLIMATE_TYPES_H

#include <stdint.h>

/* 温湿度驱动与应用快照共享的数据，不包含总线、寄存器或操作声明。 */
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
    uint32_t humidity_raw;    /* 20位无符号原始值。 */
    uint32_t temperature_raw; /* 20位无符号原始值。 */
    float humidity_rh;        /* %RH */
    float temperature_c;      /* ℃ */
    uint8_t status;
} aht21_struct;

#endif
