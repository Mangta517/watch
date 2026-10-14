#ifndef DIAG_HOST_TASK_H
#define DIAG_HOST_TASK_H
#include "FreeRTOS.h"
typedef struct
{
    const char *pcTaskName;
    uint32_t uxCurrentPriority, eCurrentState, ulRunTimeCounter, usStackHighWaterMark;
} TaskStatus_t;
UBaseType_t uxTaskGetSystemState(TaskStatus_t *tasks, UBaseType_t count, uint32_t *total);
void diag_enter(void);
void diag_exit(void);
#define taskENTER_CRITICAL() diag_enter()
#define taskEXIT_CRITICAL() diag_exit()
#endif
