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
    uint32_t PRER;
} RTC_TypeDef;
extern RTC_TypeDef host_rtc;
#define RTC (&host_rtc)
#define __HAL_RTC_WRITEPROTECTION_DISABLE(handle) ((void)(handle))
#define __HAL_RTC_WRITEPROTECTION_ENABLE(handle) ((void)(handle))
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
