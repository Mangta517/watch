/* HAL types only; pin assignments come from the real Core/Inc/main.h. */
#ifndef SENSOR_HOST_HAL_H
#define SENSOR_HOST_HAL_H
#include <stdint.h>

typedef struct
{
    uint32_t unused;
} GPIO_TypeDef;
extern GPIO_TypeDef host_gpio_a, host_gpio_b;
#define GPIOA                (&host_gpio_a)
#define GPIOB                (&host_gpio_b)
#define GPIO_PIN_0           (1U << 0)
#define GPIO_PIN_1           (1U << 1)
#define GPIO_PIN_2           (1U << 2)
#define GPIO_PIN_3           (1U << 3)
#define GPIO_PIN_4           (1U << 4)
#define GPIO_PIN_5           (1U << 5)
#define GPIO_PIN_6           (1U << 6)
#define GPIO_PIN_7           (1U << 7)
#define GPIO_PIN_8           (1U << 8)
#define GPIO_PIN_9           (1U << 9)
#define GPIO_PIN_10          (1U << 10)
#define GPIO_PIN_11          (1U << 11)
#define GPIO_PIN_12          (1U << 12)
#define GPIO_PIN_13          (1U << 13)
#define GPIO_PIN_14          (1U << 14)
#define GPIO_PIN_15          (1U << 15)
#define GPIO_MODE_INPUT      0U
#define GPIO_MODE_OUTPUT_PP  1U
#define GPIO_NOPULL          0U
#define GPIO_SPEED_FREQ_LOW  0U
#define GPIO_PULLUP          1U
#define GPIO_SPEED_FREQ_HIGH 2U
typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;
typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
typedef struct
{
    uint32_t Pin, Mode, Pull, Speed;
} GPIO_InitTypeDef;
typedef struct
{
    uint32_t SR, DR, CR1;
} SPI_TypeDef;
extern SPI_TypeDef host_spi;
#define SPI1                (&host_spi)
#define SPI_SR_BSY          0x80U
#define READ_BIT(reg, bits) ((reg) & (bits))
typedef struct
{
    uint32_t CR;
} DMA_Stream_TypeDef;
typedef struct
{
    DMA_Stream_TypeDef *Instance;
} DMA_HandleTypeDef;
typedef struct
{
    SPI_TypeDef *Instance;
    DMA_HandleTypeDef *hdmatx;
} SPI_HandleTypeDef;
#define DMA_SxCR_EN 1U
#define __HAL_SPI_DISABLE(spi) ((spi)->Instance->CR1 = 0)
typedef struct
{
    uint32_t arr, compare;
} TIM_HandleTypeDef;
#define TIM_CHANNEL_3                                3U
#define DMA2_Stream2_IRQn                            2
#define SPI1_IRQn                                    35
#define __HAL_TIM_SET_AUTORELOAD(timer, value)       ((timer)->arr = (value))
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare = (value))

typedef struct
{
    uint32_t unused;
} ADC_HandleTypeDef;
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout_ms);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *adc);

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
