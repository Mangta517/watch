#ifndef BLE_HOST_MAIN_H
#define BLE_HOST_MAIN_H
#include <stdint.h>
typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
typedef enum
{
    GPIO_PIN_RESET,
    GPIO_PIN_SET
} GPIO_PinState;
typedef struct
{
    uint32_t BaudRate;
} UART_InitTypeDef;
typedef struct
{
    void *Instance;
    UART_InitTypeDef Init;
    uint32_t ErrorCode;
} UART_HandleTypeDef;

#define HAL_UART_ERROR_NONE 0
#define HAL_UART_ERROR_ORE  8
typedef struct
{
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;
#define GPIOA                          ((void *)1)
#define GPIOB                          ((void *)2)
#define USART1                         ((void *)3)
#define USART1_IRQn                    37
#define BLE_EN_GPIO_Port               GPIOA
#define BLE_EN_Pin                     (1U << 8)
#define BLE_TX_Pin                     (1U << 9)
#define BLE_RX_Pin                     (1U << 10)
#define HEART_LED_GPIO_Port              GPIOB
#define HEART_LED_Pin                    (1U << 15)
#define GPIO_MODE_INPUT                0U
#define GPIO_MODE_ANALOG               3U
#define GPIO_MODE_AF_PP                2U
#define GPIO_PULLDOWN                  2U
#define GPIO_NOPULL                    0U
#define GPIO_SPEED_FREQ_VERY_HIGH      3U
#define GPIO_AF7_USART1                7U
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *uart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);
#endif
