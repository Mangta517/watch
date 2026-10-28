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
    uint32_t CTRL, CYCCNT, DEMCR;
} Host_Cycle_Type;
extern Host_Cycle_Type host_cycles;
#define DWT (&host_cycles)
#define CoreDebug (&host_cycles)
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
    uint32_t CCR3, CNT, CR2, SR, CR, CFGR, APB2ENR, ODR;
} Host_Peripheral_Type;
extern Host_Peripheral_Type host_tim2, host_tim3, host_rcc, host_spi, host_dma, host_adc, host_gpio;
#define TIM2 (&host_tim2)
#define TIM3 (&host_tim3)
#define RCC (&host_rcc)
#define SPI1 (&host_spi)
#define DMA2_Stream2 (&host_dma)
#define ADC1 (&host_adc)
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
typedef int HAL_StatusTypeDef;
#endif
