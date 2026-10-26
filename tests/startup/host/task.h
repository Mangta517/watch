#ifndef STARTUP_HOST_TASK_H
#define STARTUP_HOST_TASK_H
#include "FreeRTOS.h"
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack, void *arg, UBaseType_t priority, TaskHandle_t *handle);
void vTaskStartScheduler(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *last, TickType_t ticks);
BaseType_t xTaskNotifyGive(TaskHandle_t task);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
#endif
