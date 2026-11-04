/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_task.h"    /* 应用层任务管理: 全工程唯一的 FreeRTOS 任务创建/调度器启动入口 */
#include "app_reset.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 启动只配置原有 HSI/PLL；LSE/RTC 由 watch_rtc_init 初始化，失败时仍启动其他功能。 */
static void watch_system_clock_config(void)
{
    /* 入参: 无；返回: 无。只在 main 启动阶段调用一次，失败直接进入 Error_Handler。 */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};   /* 振荡器/PLL 配置，先全清零再逐项填写。 */
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};   /* 总线分频与系统时钟源配置，先全清零。 */

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();                                        /* 打开 PWR 时钟，才能写电压调节相关寄存器。 */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);     /* 选 Scale3 调压档。 */

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;         /* 只配置 HSI，不在此打开 LSE。 */
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                           /* 打开内部 16MHz HSI。 */
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; /* 使用出厂默认 HSI 校准值。 */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                       /* 打开主 PLL。 */
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;               /* PLL 输入选 HSI。 */
    RCC_OscInitStruct.PLL.PLLM = 8;                                    /* 输入分频 M=8，得到 2MHz 比较频率。 */
    RCC_OscInitStruct.PLL.PLLN = 128;                                  /* 倍频 N=128，VCO 为 256MHz。 */
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;                        /* P 分频 4，系统时钟 64MHz。 */
    RCC_OscInitStruct.PLL.PLLQ = 8;                                    /* Q 分频 8，PLLQ 输出 256/8=32MHz。 */
    /* 按上述参数启动 HSI 和 PLL；失败时不再继续切换时钟。 */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();                                               /* 振荡器或 PLL 未就绪，停机记录。 */
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    /* 一次性声明要改的四类时钟：HCLK、SYSCLK、PCLK1、PCLK2。 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;          /* 系统时钟切到 PLL 输出。 */
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                 /* AHB 不分频，HCLK=64MHz。 */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;                  /* APB1 二分频，PCLK1=32MHz。 */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;                  /* APB2 不分频，PCLK2=64MHz。 */

    /* 应用分频并设置 2 个 Flash 等待周期；失败说明时钟切换未完成。 */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();                                               /* 时钟配置失败，停机记录。 */
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    /* 入参: 无；返回: 正常不返回。先自锁供电，再初始化复位记录、HAL、
     * 时钟和片上外设，最后创建任务并启动调度器。 */
    /* 局部 GPIO 配置模板：只用于把 PA3 配成推挽输出，字段填完即交给 HAL_GPIO_Init。 */
    GPIO_InitTypeDef power_gpio = {0};

    /* SW2短按先提供电源；立即拉高PA3保持TPS_EN，松手后继续完成初始化。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();                                     /* 先开 GPIOA 时钟，PA3 才能输出。 */
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET); /* 先把输出锁存器置高，避免切输出瞬间掉电。 */
    power_gpio.Pin = POWER_EN_Pin;                                    /* 选中 PA3(TPS_EN)。 */
    power_gpio.Mode = GPIO_MODE_OUTPUT_PP;                            /* 推挽输出，能主动拉高保持使能。 */
    power_gpio.Pull = GPIO_NOPULL;                                    /* 不用内部上下拉。 */
    power_gpio.Speed = GPIO_SPEED_FREQ_LOW;                           /* 电源使能是慢信号，用低速档即可。 */
    HAL_GPIO_Init(POWER_EN_GPIO_Port, &power_gpio);                   /* 写入配置，PA3 开始驱动高电平。 */
    app_reset_init(); /* 在HAL和RTC初始化前保存复位源及上次备份记录。 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* HAL_Init 已在上方启用 Flash 指令/数据缓存和预取、设置 NVIC 分组、
   * 建立 HAL 1ms 时基并调用 HAL_MspInit；不在此设置 Flash 等待周期。 */
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
    watch_system_clock_config();   /* 把系统时钟从复位默认值切到 HSI+PLL 的 64MHz。 */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    /* 以上 MX_* 依次完成 GPIO、DMA、SPI1(LCD)、TIM3(背光 PWM)、
     * USART1(蓝牙) 和 ADC1(电池采样) 的片上外设初始化。 */
    /* STOP进一步降低Flash和低功耗稳压器功耗；唤醒时间由RTC补齐。 */
    HAL_PWREx_EnableFlashPowerDown();        /* 允许进入 STOP 时 Flash 进入掉电模式。 */
    HAL_PWREx_EnableLowRegulatorLowVoltage(); /* 允许低功耗稳压器工作在低电压档。 */
    /* sensor 任务轮询 MPU 数据，启动时持续屏蔽未使用的 PB12 DATA_RDY EXTI。
     * 只清该位，保留 PA4、其它 EXTI 线和共享 NVIC 组；STOP 唤醒恢复原掩码后仍保持关闭。 */
    EXTI->IMR &= ~MPU6050_INT_Pin;
    /* main 只做 CubeMX 初始化; 任务归口 app_task, 背光/屏幕/触摸由应用层接管 */
    App_Task_Create();   /* 仅创建 LVGL 任务并初始化看门狗; sensor/touch 任务由 LVGL 任务首帧后内部创建 */
    App_Task_Start();    /* 不返回: 启动调度器 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* 调度器启动成功后不会回到这里；若意外返回，本循环原地空转，
         * 本路径没有喂狗或主动复位动作；若外部看门狗已启用，则由它超时复位。 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* SystemClock_Config 是 CubeMX 生成区里的时钟配置函数：同时打开 HSI 与 LSE，
 * PLL 参数与 watch_system_clock_config 相同(M=8/N=128/P=4/Q=8)，
 * 但本工程当前没有调用点，启动实际走 watch_system_clock_config。 */
/* 本文件用户启动函数集中在原有 USER 区域，包含 watch_system_clock_config 等启动辅助函数。 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* 入参: 无；返回: 不返回。关可屏蔽中断后把停机类型和返回地址写入备份域，再原地空转；
   * 本路径没有喂狗或主动复位动作，若外部看门狗已启用，则由它超时复位。 */
    __disable_irq();   /* 仅屏蔽可屏蔽中断，保证下面的记录过程不被中断打断；NMI 和硬故障仍可进入。 */
#if defined(__CC_ARM)
    /* ARMCC：取当前函数的返回地址作为停机现场 PC。 */
    app_reset_capture_stop(APP_RESET_ERROR_HANDLER, (uint32_t)__return_address(), 0);
#else
    /* GCC/其他编译器：用内建函数取返回地址并转为 32 位值。 */
    app_reset_capture_stop(APP_RESET_ERROR_HANDLER, (uint32_t)(uintptr_t)__builtin_return_address(0), 0);
#endif
    /* 记录已提交；此处原地空转，没有喂狗或主动复位动作；
     * 若外部看门狗已启用，则由它超时复位。 */
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* 入参: file 为断言失败的文件名，line 为行号；当前实现为空，不做任何上报。 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
