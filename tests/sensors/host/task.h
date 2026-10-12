#ifndef SENSOR_HOST_TASK_H
#define SENSOR_HOST_TASK_H
#include "FreeRTOS.h"
#define taskSCHEDULER_SUSPENDED   0
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING     2
int xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
#endif
