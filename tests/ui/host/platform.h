/* Preincluded only in the desktop harness; firmware uses the real main.h. */
#ifndef UI_HOST_PLATFORM_H
#define UI_HOST_PLATFORM_H
#include <stdint.h>
#define __MAIN_H
#define KEY_1_GPIO_Port ((void *)0)
#define KEY_1_Pin 32U
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
#endif
