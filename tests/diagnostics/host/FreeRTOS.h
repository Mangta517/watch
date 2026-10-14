#ifndef DIAG_HOST_FREERTOS_H
#define DIAG_HOST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t StackType_t;
typedef unsigned int UBaseType_t;
#define configMINIMAL_STACK_SIZE 128U
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
#endif
