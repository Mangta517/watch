#ifndef UI_HOST_FREERTOS_H
#define UI_HOST_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define configTICK_RATE_HZ 1000U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
void host_touch_enter_critical(void);
void host_touch_exit_critical(void);
#define taskENTER_CRITICAL() host_touch_enter_critical()
#define taskEXIT_CRITICAL() host_touch_exit_critical()
#endif
