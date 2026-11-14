#ifndef HEART_TYPES_H
#define HEART_TYPES_H

#include <stdint.h>

/* 驱动故障快照的数据契约；UI读取此副本，不访问器件。 */
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
    uint8_t value; /* PID/配置不符时的读回值；通信失败时无效。 */
} em7028_fault;

#endif
