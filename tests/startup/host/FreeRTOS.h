#ifndef STARTUP_HOST_FREERTOS_H
#define STARTUP_HOST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdPASS            1
#define pdTRUE            1
#define pdFALSE           0
#define portYIELD_FROM_ISR(woken) startup_yield_from_isr(woken)
void startup_yield_from_isr(BaseType_t woken);
#define portMAX_DELAY     UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#endif
