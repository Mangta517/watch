#ifndef SENSOR_HOST_FREERTOS_H
#define SENSOR_HOST_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define configTICK_RATE_HZ 1000U
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))
#define portMAX_DELAY      UINT32_MAX
#endif
