/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "app_reset.h"
#include "app_task.h"
#include "watch_rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */
extern void xPortSysTickHandler(void);   /* FreeRTOS(CM4F port) 系统节拍处理函数 */
extern void vPortSVCHandler(void);       /* FreeRTOS(CM4F port) SVC 处理函数 */
extern void xPortPendSVHandler(void);    /* FreeRTOS(CM4F port) PendSV 处理函数 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 异常入口先取得硬件压栈地址和EXC_RETURN；记录过程不使用LVGL、I2C或UART。 */
void app_fault_stop(const uint32_t *stack, uint32_t exc_return, uint32_t kind)
{
    /* 入参: stack 为异常压栈帧地址，exc_return 为 EXC_RETURN，kind 为故障类型编号；
     * 返回: 不返回。记录故障类型、SP 和 EXC_RETURN，并在压栈帧可采信时提取 LR/PC/xPSR；
     * 完成后原地空转，本路径没有喂狗或主动复位动作，若外部看门狗已启用，则由它超时复位。 */
    __disable_irq();                                  /* 仅屏蔽可屏蔽中断，避免采集期间被中断打断。 */
    app_reset_capture_fault(kind, stack, exc_return); /* 把故障类型、SP 和 EXC_RETURN 写入备份域，并按需提取 LR/PC/xPSR。 */
    for (;;)
    {
        /* 现场已提交；此处原地空转，本路径没有喂狗或主动复位动作，
         * 若外部看门狗已启用，则由它超时复位。 */
    }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern SPI_HandleTypeDef hspi1;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    /* 本工程不区分 NMI 来源；进入后没有恢复路径，直接原地空转，
     * 本路径没有喂狗或主动复位动作，若外部看门狗已启用，则由它超时复位。 */
    while (1)
    {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__asm void HardFault_Handler(void)
{
    extern app_fault_stop;
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    MOV r1, lr
    MOVS r2, #1
    B app_fault_stop
}

/**
  * @brief This function handles Memory management fault.
  */
__asm void MemManage_Handler(void)
{
    extern app_fault_stop;
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    MOV r1, lr
    MOVS r2, #2
    B app_fault_stop
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__asm void BusFault_Handler(void)
{
    extern app_fault_stop;
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    MOV r1, lr
    MOVS r2, #3
    B app_fault_stop
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__asm void UsageFault_Handler(void)
{
    extern app_fault_stop;
    TST lr, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    MOV r1, lr
    MOVS r2, #4
    B app_fault_stop
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
    /* 入参: 无；返回: 无。SVC 异常由 FreeRTOS 用于启动第一个任务，
     * 这里只把控制权转给移植层的 vPortSVCHandler。 */
    vPortSVCHandler();    /* 交给 FreeRTOS: 首个任务启动入口 */
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
    /* 本工程未使用调试监视异常，函数体为空，进入后直接返回。 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
    /* 入参: 无；返回: 无。PendSV 是 FreeRTOS 的上下文切换入口，这里只转交移植层。 */
    xPortPendSVHandler(); /* 交给 FreeRTOS: 上下文切换 */
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
    /* 入参: 无；返回: 无。上面先把 HAL 的 uwTick 加 1，供 HAL_Delay 和超时判断使用；
     * 下面在调度器已启动时再把节拍交给 FreeRTOS。 */
    /* 心跳中断(与 Watch 工程一致的做法):
     * 运行时SysTick每1ms进入一次；Tickless休眠的跳过时间由app_tickless同时补齐HAL/RTOS。
     * FreeRTOS 调度器启动后, 再把节拍交给 xPortSysTickHandler 做任务时间片轮转。
     * 调度器未启动时不执行调度处理, 避免启动阶段冲突。 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();   /* 调度器已运行，交给 FreeRTOS 更新节拍并处理时间片。 */
    }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WAKE_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
    /* 入参: 无；返回: 无。上面清 PA4(WAKE，上升沿触发) 的挂起标志；
     * 下面只发任务通知，把按键事件交给 LVGL 任务处理，不在中断里消抖或翻页。 */
    app_task_wake_from_isr();
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
    /* 入参: 无；返回: 无。上面已由 HAL 按当前中断源分发到收发完成或错误处理；
     * 具体回调由蓝牙驱动实现，这里不再追加动作。 */
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MPU6050_INT_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    /* 入参: 无；返回: 无。上面清 PB12(MPU6050_INT，上升沿触发) 的挂起标志，
     * 并由 HAL 调用 MPU6050 驱动里的 EXTI 回调累计边沿与数据就绪标志。 */
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */
    /* 入参: 无；返回: 无。上面把 SPI1 发送 DMA 的完成/半完成/错误事件交给 HAL 处理，
     * HAL 会清标志并继续 SPI 发送完成流程。 */
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void RTC_WKUP_IRQHandler(void)
{
    /* 入参: 无；返回: 无。RTC 唤醒定时器到点后进入，由 watch_rtc 停止唤醒源并清标志。 */
    watch_rtc_stop_wakeup();
}

void SPI1_IRQHandler(void)
{
    /* 入参: 无；返回: 无。SPI1 的中断事件交给 HAL 状态机，
     * 供 LCD 的 DMA 刷屏完成/错误路径使用。 */
    HAL_SPI_IRQHandler(&hspi1);
}

/* USER CODE END 1 */
