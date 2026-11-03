#ifndef CLOCK_HOST_MAIN_H
#define CLOCK_HOST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
typedef struct
{
    uint32_t PLLState;
} RCC_PLLInitTypeDef;
typedef struct
{
    uint32_t OscillatorType, LSEState;
    RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;
typedef struct
{
    uint32_t PeriphClockSelection, RTCClockSelection;
} RCC_PeriphCLKInitTypeDef;
typedef struct
{
    uint32_t HourFormat, AsynchPrediv, SynchPrediv, OutPut, OutPutPolarity, OutPutType;
} RTC_InitTypeDef;
typedef struct
{
    void *Instance;
    RTC_InitTypeDef Init;
    uint32_t State;
} RTC_HandleTypeDef;
typedef struct
{
    uint8_t Hours, Minutes, Seconds;
    uint32_t DayLightSaving, StoreOperation, SubSeconds;
} RTC_TimeTypeDef;
typedef struct
{
    uint8_t Year, Month, Date, WeekDay;
} RTC_DateTypeDef;
typedef struct
{
    uint32_t PRER, BKP0R, CR, ISR, TR, DR;
} RTC_TypeDef;
extern RTC_TypeDef host_rtc;
#define RTC (&host_rtc)
#define RTC_CR_WUTE (1UL << 10)
#define RTC_CR_WUTIE (1UL << 14)
#define RTC_ISR_WUTWF (1UL << 2)
#define RTC_ISR_RSF (1UL << 5)
#define RTC_ISR_INITF (1UL << 6)
#define RTC_ISR_INIT (1UL << 7)
#define RTC_CR_BYPSHAD (1UL << 5)
#define RTC_CR_BKP (1UL << 18)
#define HAL_RTC_STATE_READY 1
#define HAL_RTC_STATE_BUSY 2
#define HAL_RTC_STATE_ERROR 3
#define RTC_FLAG_WUTF (1UL << 10)
#define RTC_RSF_MASK (~(RTC_ISR_INIT | RTC_ISR_RSF))
#define RTC_ISR_RESERVED_MASK (RTC_ISR_WUTWF | RTC_FLAG_WUTF | RTC_ISR_RSF | RTC_ISR_INIT)
#define RTC_WAKEUPCLOCK_RTCCLK_DIV16 0
#define RTC_WKUP_IRQn 3
typedef struct
{
    uint32_t CTRL, CYCCNT, DEMCR;
} Host_Cycle_Type;
extern Host_Cycle_Type host_cycles;
extern uint32_t SystemCoreClock;
#define DWT (&host_cycles)
#define CoreDebug (&host_cycles)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1
void host_nop(void);
#define __NOP() host_nop()
#define __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(handle, flag) ((void)(handle), RTC->ISR &= ~(flag))
#define __HAL_RTC_WAKEUPTIMER_EXTI_DISABLE_IT() host_wakeup_exti_disable()
#define __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG() host_wakeup_exti_clear()
void host_wakeup_exti_disable(void);
void host_wakeup_exti_clear(void);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *rtc, uint32_t count, uint32_t clock);
void host_write_protection(RTC_HandleTypeDef *rtc, uint8_t enabled);
#define __HAL_RTC_WRITEPROTECTION_DISABLE(handle) host_write_protection(handle, 0)
#define __HAL_RTC_WRITEPROTECTION_ENABLE(handle) host_write_protection(handle, 1)
HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *rtc);
HAL_StatusTypeDef RTC_ExitInitMode(RTC_HandleTypeDef *rtc);
#define RCC_OSCILLATORTYPE_LSE     1U
#define RCC_LSE_ON                 1U
#define RCC_PLL_NONE               0U
#define RCC_PERIPHCLK_RTC          1U
#define RCC_RTCCLKSOURCE_LSE       1U
#define RTC_HOURFORMAT_24          0U
#define RTC_OUTPUT_DISABLE         0U
#define RTC_OUTPUT_POLARITY_HIGH   0U
#define RTC_OUTPUT_TYPE_OPENDRAIN  0U
#define RTC_MONTH_JANUARY          1U
#define RTC_WEEKDAY_SATURDAY       6U
#define RTC_DAYLIGHTSAVING_NONE    0U
#define RTC_STOREOPERATION_RESET   0U
#define RTC_FORMAT_BIN             0U
#define RTC_BKP_DR0                0U
#define RCC_FLAG_LSERDY 2U
#define LSE_STARTUP_TIMEOUT 5000U
#define __HAL_RCC_LSE_CONFIG(value) mock_lse_config(value)
#define __HAL_RCC_GET_FLAG(flag) mock_lse_flag(flag)
void mock_lse_config(uint32_t value);
uint32_t mock_lse_flag(uint32_t flag);
uint32_t HAL_GetTick(void);
#define __HAL_RCC_PWR_CLK_ENABLE() mock_power_enable()
#define __HAL_RCC_RTC_ENABLE()     mock_rtc_enable()
void mock_power_enable(void);
void mock_rtc_enable(void);
void HAL_PWR_EnableBkUpAccess(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *oscillator);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *clock);
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *rtc);
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *rtc, RTC_TimeTypeDef *time, uint32_t format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *rtc, RTC_DateTypeDef *date, uint32_t format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *rtc, RTC_TimeTypeDef *time, uint32_t format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *rtc, RTC_DateTypeDef *date, uint32_t format);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *rtc, uint32_t reg);
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *rtc, uint32_t reg, uint32_t value);
#endif
