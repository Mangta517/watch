#ifndef TICKLESS_HOST_TASK_H
#define TICKLESS_HOST_TASK_H
#include "FreeRTOS.h"
typedef enum
{
    eAbortSleep,
    eStandardSleep
} eSleepModeStatus;
eSleepModeStatus eTaskConfirmSleepModeStatus(void);
void vTaskStepTick(TickType_t ticks);
#endif
