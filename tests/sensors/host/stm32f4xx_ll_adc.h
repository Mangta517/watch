#ifndef HOST_LL_ADC_H
#define HOST_LL_ADC_H
#include <stdint.h>
extern uint16_t host_vrefint_cal;
#define VREFINT_CAL_ADDR (&host_vrefint_cal)
#define VREFINT_CAL_VREF 3300UL
#define LL_ADC_DELAY_VREFINT_STAB_US 10UL
#endif
