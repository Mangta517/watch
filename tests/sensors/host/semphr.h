#ifndef SENSOR_HOST_SEMPHR_H
#define SENSOR_HOST_SEMPHR_H
#include "FreeRTOS.h"
typedef void *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
int xSemaphoreGiveFromISR(SemaphoreHandle_t semaphore, int *woken);
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait);
int xSemaphoreGive(SemaphoreHandle_t mutex);
#endif
