#ifndef TICKLESS_HOST_MAIN_H
#define TICKLESS_HOST_MAIN_H
#include <stdint.h>
#define __IO volatile
typedef struct
{
    uint32_t CTRL, LOAD, VAL;
} SysTick_Type;
typedef struct
{
    uint32_t ICSR, SCR;
} SCB_Type;
extern SysTick_Type host_systick;
extern SCB_Type host_scb;
typedef struct
{
    uint32_t CTRL, CYCCNT, DEMCR, DHCSR;
} Host_Cycle_Type;
extern Host_Cycle_Type host_cycles;
#define DWT (&host_cycles)
#define CoreDebug (&host_cycles)
#define CoreDebug_DHCSR_C_DEBUGEN_Msk 1
typedef struct
{
    uint32_t CR;
} Host_Debug_Type;
extern Host_Debug_Type host_debug;
#define DBGMCU (&host_debug)
#define DBGMCU_CR_DBG_SLEEP (1U << 0)
#define DBGMCU_CR_DBG_STOP (1U << 1)
#define DBGMCU_CR_DBG_STANDBY (1U << 2)
#define RCC_APB2ENR_ADC1EN (1U << 8)
#define __HAL_RCC_ADC1_IS_CLK_DISABLED() ((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define SysTick (&host_systick)
#define SCB (&host_scb)
#define SysTick_CTRL_ENABLE_Msk 1U
#define SysTick_CTRL_TICKINT_Msk 2U
#define SysTick_CTRL_CLKSOURCE_Msk 4U
#define SCB_ICSR_PENDSTSET_Msk (1UL << 26)
#define SCB_ICSR_PENDSTCLR_Msk (1UL << 25)
#define SCB_SCR_SLEEPDEEP_Msk (1UL << 2)
typedef struct
{
    uint32_t CCR3, CNT, CR2, SR, CR, CFGR, APB2ENR, ODR, IMR, PR, CSR;
} Host_Peripheral_Type;
extern Host_Peripheral_Type host_tim2, host_tim3, host_rcc, host_spi, host_dma, host_adc, host_gpio, host_exti;
#define TIM2 (&host_tim2)
#define TIM3 (&host_tim3)
#define htim3 host_tim3
#define TIM_CHANNEL_3 3
#define __HAL_TIM_GET_COMPARE(timer, channel) ((timer)->CCR3)
#define RCC (&host_rcc)
#define SPI1 (&host_spi)
#define DMA2_Stream2 (&host_dma)
#define ADC1 (&host_adc)
#define EXTI (&host_exti)
extern Host_Peripheral_Type host_pwr;
#define PWR (&host_pwr)
#define PWR_CSR_PVDO (1U << 2)
#define PWR_CSR_VOSRDY (1U << 14)
#define BLE_EN_GPIO_Port (&host_gpio)
#define BLE_EN_Pin (1U << 8)
#define RCC_APB2ENR_USART1EN (1U << 4)
#define SPI_SR_BSY (1U << 7)
#define DMA_SxCR_EN 1U
#define ADC_CR2_ADON 1U
#define RCC_CR_PLLON (1U << 24)
#define RCC_CR_PLLRDY (1U << 25)
#define RCC_CFGR_SW 3U
#define RCC_CFGR_SW_PLL 2U
#define RCC_CFGR_SWS 12U
#define RCC_CFGR_SWS_PLL 8U
#define HSI_VALUE 16000000U
#define PWR_LOWPOWERREGULATOR_ON 1U
#define PWR_STOPENTRY_WFI 1U
void host_modify_reg(uint32_t *reg, uint32_t mask, uint32_t value);
#define MODIFY_REG(reg, mask, value) host_modify_reg(&(reg), mask, value)
void HAL_PWR_EnterSTOPMode(uint32_t regulator, uint8_t entry);
void Error_Handler(void);
void host_disable_irq(void);
void host_enable_irq(void);
void host_sleep(void);
void host_barrier(void);
#define __disable_irq() host_disable_irq()
#define __enable_irq() host_enable_irq()
#define __WFI() host_sleep()
#define __DSB() host_barrier()
#define __ISB() host_barrier()
typedef Host_Peripheral_Type GPIO_TypeDef;
typedef struct
{
    uint32_t Pin, Mode, Pull, Speed;
} GPIO_InitTypeDef;
typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;
extern GPIO_TypeDef host_watchdog_gpio;
#define WDI_GPIO_Port (&host_watchdog_gpio)
#define WDI_Pin (1U << 2)
#define WDG_EN_GPIO_Port (&host_watchdog_gpio)
#define WDG_EN_Pin (1U << 1)
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
uint32_t host_get_primask(void);
void host_set_primask(uint32_t mask);
#define __get_PRIMASK() host_get_primask()
#define __set_PRIMASK(mask) host_set_primask(mask)
#define __NOP() ((void)0)
typedef int HAL_StatusTypeDef;
#endif
