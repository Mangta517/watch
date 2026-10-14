#ifndef SENSOR_HOST_TASK_H
#define SENSOR_HOST_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
void host_dma_yield(BaseType_t woken);
#define portYIELD_FROM_ISR(woken) host_dma_yield(woken)
TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken);
#define taskSCHEDULER_SUSPENDED   0
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING     2
int xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
#endif
