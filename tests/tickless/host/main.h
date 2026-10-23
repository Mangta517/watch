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
