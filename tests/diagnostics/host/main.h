#ifndef DIAG_HOST_MAIN_H
#define DIAG_HOST_MAIN_H
#include <stdint.h>
typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
typedef struct
{
    uint32_t CR1, PSC, ARR, EGR, SR, CNT;
} diag_timer_struct;
typedef struct
{
    uint32_t CFGR;
} diag_rcc_struct;
extern diag_timer_struct diag_timer;
extern diag_rcc_struct diag_rcc;
#define TIM2 (&diag_timer)
#define RCC (&diag_rcc)
#define RCC_CFGR_PPRE1 0x1C00U
#define TIM_EGR_UG 1U
#define TIM_CR1_CEN 1U
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_GetTick(void);
#endif
