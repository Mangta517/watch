#ifndef RESET_HOST_MAIN_H
#define RESET_HOST_MAIN_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t APB1ENR, BDCR, CSR;
} RCC_TypeDef;

typedef struct
{
    volatile uint32_t CR;
} PWR_TypeDef;

typedef struct
{
    volatile uint32_t backup[20];
} RTC_TypeDef;

typedef struct
{
    volatile uint32_t CFSR, HFSR, MMFAR, BFAR;
} SCB_Type;

extern RCC_TypeDef host_rcc;
extern PWR_TypeDef host_pwr;
extern RTC_TypeDef host_rtc;
extern SCB_Type host_scb;

#define RCC (&host_rcc)
#define PWR (&host_pwr)
#define RTC (&host_rtc)
#define SCB (&host_scb)
#define BKP0R backup[0]
#define RCC_APB1ENR_PWREN (1U << 28)
#define RCC_BDCR_RTCEN (1U << 15)
#define PWR_CR_DBP (1U << 8)
#define SCB_CFSR_MUNSTKERR_Msk (1U << 3)
#define SCB_CFSR_MSTKERR_Msk (1U << 4)
#define SCB_CFSR_MLSPERR_Msk (1U << 5)
#define SCB_CFSR_UNSTKERR_Msk (1U << 11)
#define SCB_CFSR_STKERR_Msk (1U << 12)
#define SCB_CFSR_LSPERR_Msk (1U << 13)

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
void __DSB(void);
uint32_t HAL_GetTick(void);
void host_clear_reset_flags(void);
void host_reset_checksum_bit(void);
#define __HAL_RCC_CLEAR_RESET_FLAGS() host_clear_reset_flags()

#endif
