/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "led.h"
#include "gpio_i2c.h"
#include "ds3232.h"
#include "Moving_Average_Filter.h"
#include "uart_prot.h"
#include "ds2782e.h"
#include "sw6301.h"
#include "uart_prot.h"
#include "button.h"
#include "bsp_hwdg.h"
#include "interflash.h"
#include "interflash_test.h"
#include "app.h"
#include <string.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
#define MAC_FILTER_POINT_NUM	15
uint16_t ADC1_ARR[MAC_FILTER_POINT_NUM];
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
uint16_t ADC_ConvertedValue;
uint16_t ADC_FINAL;
uint16_t REAL_VALUE;
MOVING_AVERAGE_FILTERtyp adc1_filter = {0};

struct ds278x_info my_info = {0};
uint8_t key_value = 0;

//12V_Switch_EN ADC采样电源电压低于12V置高，高于20V置低
//5VSW_EN soc电源开关

void Init_Moving_Average_Filter(void)
{
	INIT_MOVING_AVERAGE_FILTER(&adc1_filter, ADC1_ARR, MAC_FILTER_POINT_NUM);
}
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Get_Adc1_Average_Value(void)
{
	adc1_filter.INPUT = ADC_ConvertedValue;
	MOVING_AVERAGE_FILTER(&adc1_filter);
}

void ADC_Smooth(void)
{
	ADC_FINAL = adc1_filter.OUTPUT;
	REAL_VALUE = ADC_FINAL * 8.864469;
}


void Early_Filter_Flow(void)
{
	Get_Adc1_Average_Value();
	ADC_Smooth();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int my_capacity = 0;
int my_ret1 = 0;
int my_ret2 = 0;
int my_ret3 = 0;
int my_ret4 = 0;
int my_ret5 = 0;
int my_ret6 = 0;
int my_ret10 = 0;
chip_version version = {0};
struct rtc_time rct_read = {0};
struct rtc_time rtc_set = {0};


uint8_t charge_mode = 0;
uint8_t c_no_load = 0;
uint8_t charge_or_nocharge_mode = 0xff;

system_status sys_stat = {0};
mode_status mode_stat = {0};
c_no_load_status load_value = {0};
uint8_t my_flag = 0;
uint8_t temp_value;
int sw6301_write_regs(uint8_t port, uint8_t reg, uint8_t *data, uint16_t data_size);

event_int_enable write_event_int = {0};
event_int_enable read_event_int = {0};

//int event_int_enable_func(event_int_enable int_value)
//{
//	int ret;
//	ret = i2c_sw6301_write(0x25, (uint8_t *)&int_value);
//	return ret;
//}

force_control force_value = {0};
output_ibat_value ibat_value = {0};
float charge_goal_vol = 8.3;
float read_ibat_curr = 0;
float read_vbus_vol = 0;
float read_vbat_vol = 0;
float read_ibus_curr = 0;
uint8_t port_curr = 0;
uint8_t battery_curr = 0;
uint8_t reg40_value = 0;
float real_port_curr = 0;
float real_battery_curr = 0;
uint8_t reg43_value = 0;
uint8_t reg44_value = 0;
extern charge_state chargestate;
extern mcu_req_timeout mcu_timeout;
extern uint8_t interflash_ret1;
int rtc_ret;

int main(void)
{

  uint8_t ret;
  uart_msg msg_inst;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

	
//	ibat电流1.5A
//	充电目标电压
//	8.4
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();

 // FLASH_If_Init();
  /* USER CODE BEGIN 2 */
	i2c_init(1);
	i2c_init(2);
	i2c_init(3);
	//HAL_Delay(2000);
	API_WatchDog_Enable(0);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_ConvertedValue, 1);
  /* USER CODE END 2 */
	Init_Moving_Average_Filter();
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);
	__HAL_UART_ENABLE(&hlpuart1);
	if(1 == API_UART_RxFifoEnable(Uart_Recv_Fifo, UART_MSG_LEN))
		return 1;
	Early_Filter_Flow();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //ds278x_get_status(&my_info);
	//get_chip_version(uint8_t *version)
	//rtc_set.tm_sec = 10;
	//rtc_set.tm_min = 20;
//ds3232_set_time(&rtc_set);
	//sw6301_write_enable();
//charge_or_nocharge_mode = 0xff;
	//write_event_int.button_event =1;//0x25
//	write_event_int.charge_abn_event = 1;//0x25
//	write_event_int.discharge_abn_event = 1;//0x25
	//low_power_disable_func();
	
	//discharge_config0 config_value = {0};
	//config_value.output_power = 0;
	//charge_config0 config_value1 = {0};
	//config_value1.input_power = 0;
	//discharge_config0_func(config_value);
	//charge_config0_func(config_value1);
	//force_value.force_i2c_output_power = 1;
	//force_value.force_ibat_current = 1;
	//force_value.force_contr_charge_vol = 1;
	//force_control_func(force_value);

	//ibat_value.dischg_ibat_limit = 15;
	//set_output_ibat_value(ibat_value);

	//set_charge_goal_vol(charge_goal_vol);
	//set_charge_ibus_curr_limit_value(10);
	
	//set_charge_ibat_current_limit_value(10);

	//set_discharge_ibat_current_limit_value(1000);
	//set_discharge_ibus_curr_limit_value(1000);
	
	//__disable_irq();
	interflash_ret1 =  erase_flash(0x8008000, 0x1000);
	//__enable_irq();
	interflash_test();
HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);
HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5_HIGH);

  rtc_ret = ds3232_probe();
  if(rtc_ret != 0)
  {
		LEDG_OFF;
  }
  else
  {
		LEDG_ON;
  }

  while (1)
  {
#if 0  
		//ds3232_read_time(&rct_read);
    /* USER CODE END WHILE */
		//Get_Adc1_Average_Value();
	//	ADC_Smooth();
		//ds278x_get_status(&my_info);
	//my_ret = ds2782_get_capacity(&my_capacity);
	//if(my_ret)
	//	return my_ret;		
    /* USER CODE BEGIN 3 */
		my_ret1 = get_chip_version(&version);
		my_ret3 = get_mode_status(&mode_stat);
  //charge_mode = 0;
		my_ret2 = get_c_no_load_status(&load_value);
		my_ret4 = get_system_status(&sys_stat);

		my_ret10 = ds278x_get_status(&my_info);

		//HAL_Delay(3000);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	//	HAL_Delay(3000);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		//HAL_Delay(3000);

		//event_int_enable_func(write_event_int);
		//my_ret6 = enable_write_low_register();
		//mode_set_value.output_switch = 1;
		//my_ret5 = mode_set_func(mode_set_value);
		//uint8_t my_value = 0x18;
		
		//my_ret5 = sw6301_write_regs(2, 0x28, &my_value, 1);

		my_ret5 =  sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_IBAT, &read_ibat_curr);
		my_ret5 =  sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_VBUS, &read_vbus_vol);
				my_ret5 =  sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_VBAT, &read_vbat_vol);
				sw6301_read_adc(SW6301_ADC_CHANNEL_IBUS, &read_ibus_curr);
				
		i2c_sw6301_read(0x10, (uint8_t *)&port_curr);
		real_port_curr = port_curr * 50;
		i2c_sw6301_read(0x11, (uint8_t *)&battery_curr);
		real_battery_curr = battery_curr * 100;
		i2c_sw6301_read(0x40, &reg40_value);
		i2c_sw6301_read(0x43, &reg43_value);
		i2c_sw6301_read(0x44, &reg44_value);
//read_ibus_curr
		my_flag++;
 #else

#if 0
	if(HAL_GetTick() - heartbeat_value.last_tick_value >= 9000)
	{
		//鏂數/涓婄數閲嶅惎寮�鍙戞澘
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);//鍏虫満
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);//寮�鏈?
	}
	
	uart_msg_proc_flow(0, NULL);
	ret = get_system_status(&sys_stat);
	if(ret == 0)
	{
		if(sys_stat.discharge_status == 1)
		{
			chargestate.charge_new_state = 1;
			//chargestate.charge_old_state = 1;
			if(chargestate.charge_new_state != chargestate.charge_old_state)
			{
				msg_inst.msg_id = CMD_REQ_BAT_DISCHARGE_STATE;
				uart_msg_proc_flow(1, &msg_inst);
				mcu_timeout.mcu_req_tick = HAL_GetTick();
				mcu_timeout.mcu_req_flag = 1;
				chargestate.charge_old_state = 1;
			}
		}
		else if(sys_stat.discharge_status == 0)
		{
			chargestate.charge_new_state = 0;
			if(chargestate.charge_new_state != chargestate.charge_old_state)
			{
				msg_inst.msg_id = CMD_REQ_BAT_DISCHARGE_STATE;
				uart_msg_proc_flow(1, &msg_inst);		
				mcu_timeout.mcu_req_tick = HAL_GetTick();
				mcu_timeout.mcu_req_flag = 1;
				chargestate.charge_old_state = 0;
			}
		}
	}

	if(REAL_VALUE < 18)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);//鍏虫満	
	}

	if(REAL_VALUE > 20)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
#endif
	key_value = HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN);
API_WatchDog_FeedDog();
	//if(Key_Scan() == 1)
	//{
		//LEDG_ON;
	//}
	//else
	//{
	//	LEDG_OFF;
	//}
	
	//HAL_Delay(10000);
	
	//while(1)
	//{
	//	if(Key_Scan() == 1)
	//	{
	//		API_EnteryStandby();
	//	}
	//}
	
	//API_EnteryStandby();
		
//LEDG_ON;
//HAL_Delay(1000);
//LEDG_OFF;
//HAL_Delay(1000);
#endif
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pins: PC13*/
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
