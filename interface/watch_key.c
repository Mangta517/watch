#include "watch_key.h"
#include "main.h"

uint8_t watch_key_is_back_pressed(void)
{
    /* PA5返回键低有效；消抖和长按仍由应用层处理。 */
    return HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == GPIO_PIN_RESET;
}

uint8_t watch_key_is_wake_pressed(void)
{
    /* PA4唤醒键高有效。 */
    return HAL_GPIO_ReadPin(WAKE_GPIO_Port, WAKE_Pin) == GPIO_PIN_SET;
}
