#ifndef TICKLESS_HOST_FREERTOS_H
#define TICKLESS_HOST_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define configTICK_RATE_HZ 1000U
#define configCPU_CLOCK_HZ 84000000U
#endif
