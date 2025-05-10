/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_it.h"
#include "CircularQueue.h"
#include "uart_prot.h"
#include "led.h"
#include "ds3232.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef hlpuart1;
/* USER CODE BEGIN EV */

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
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

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

  /* USER CODE END PendSV_IRQn 1 */
}

uint8_t first_alarm = 0;
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	LEDG_ON;
	ds3232_irq();
	first_alarm++;
	if(first_alarm == 2)
	{
		LEDG_OFF;
	}
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	extern mcu_req_timeout mcu_timeout;

  /* USER CODE END SysTick_IRQn 0 */
 	 HAL_IncTick();
	if(mcu_timeout.mcu_req_bat_discharge_flag == 1 && uwTick >= mcu_timeout.mcu_req_bat_discharge_tick+ 1000)
	{
		if(mcu_timeout.soc_ack_bat_discharge_flag == 0)
		{
				mcu_timeout.timeout_bat_discharge_flag = 1;
		}
		
		mcu_timeout.mcu_req_bat_discharge_flag = 0;
	}

	if(mcu_timeout.mcu_req_get_bat_info_flag == 1 && uwTick >= mcu_timeout.mcu_req_get_bat_info_tick + 1000)
	{
		if(mcu_timeout.soc_ack_get_bat_info_flag == 0)
		{
			mcu_timeout.timeout_get_bat_info_flag = 1;
		}

		mcu_timeout.mcu_req_get_bat_info_flag = 0;
	}

	if(mcu_timeout.mcu_req_log_flag == 1 && uwTick >= mcu_timeout.mcu_req_log_tick + 1000)
	{
		if(mcu_timeout.soc_ack_log_flag == 0)
		{
			mcu_timeout.timeout_mcu_log_flag = 1;
		}

		mcu_timeout.mcu_req_log_flag = 0;
	}

	if(mcu_timeout.mcu_req_udp_ready_flag == 1 && uwTick >= mcu_timeout.mcu_req_udp_ready_tick + 1000)
	{
		if(mcu_timeout.soc_ack_udp_ready_flag == 0)
		{
			mcu_timeout.timeout_udp_ready_flag = 1;
		}

		mcu_timeout.mcu_req_udp_ready_flag = 0;
	}


	if(mcu_timeout.mcu_req_shutdown_soc_flag == 1 && uwTick >= mcu_timeout.mcu_req_shutdown_soc_tick + 1000)
	{
		if(mcu_timeout.soc_ack_shutdown_info_flag == 0)
		{
			mcu_timeout.timeout_shutdown_soc_flag = 1;
		}

		mcu_timeout.mcu_req_shutdown_soc_flag = 0;
	}
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */
  uint8_t res;
  extern CircularQueue_Str Uart_Recv_Queue;
  /* USER CODE END LPUART1_IRQn 0 */
  //HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */
	if(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE) != RESET)
	{
		res = hlpuart1.Instance->RDR;
		//HAL_UART_Receive(&hlpuart1, &res, 1, 1000);
		 PushData_CircularQueue(&Uart_Recv_Queue, &res);
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_RXNE);
	}
	else
	{
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_IDLE);
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_ORE);
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_NE);
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_FE);
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_PE);
	}
  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
