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
int quick_charge_ret = 0;

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
struct rtc_datetime rct_read = {0};
struct rtc_time rtc_set = {0};
quick_charge_sign quick_value = {0};


uint8_t charge_mode = 0;
uint8_t c_no_load = 0;
uint8_t charge_or_nocharge_mode = 0xff;

mode_status mode_stat = {0};
c_no_load_status load_value = {0};
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
extern mcu_req_timeout mcu_timeout;
extern uint8_t interflash_ret1;
int rtc_ret;

struct ds278x_info my_2782_info;
int my_temp_value;

struct rtc_time time_value = {.tm_sec = 10, .tm_min = 20, .tm_hour = 5, .tm_mday = 2, .tm_mon = 10, .tm_year = 2015};
struct rtc_wkalrm time_alarm_value1 = {.time.tm_sec = 20, .time.tm_min = 20, .time.tm_hour = 5, .time.tm_mday = 2};
struct rtc_wkalrm time_alarm_value2 = {.time.tm_sec = 50, .time.tm_min = 20, .time.tm_hour = 5, .time.tm_mday = 2};
struct rtc_time time_read_value = {0};
extern uint8_t first_alarm;
short my_test_value;
int my_test_size;
uint16_t get_rtc_beat;
uint16_t test_soc_system_beat;
struct rtc_datetime  my_rtc_time;
uint8_t rtc_len;
extern void bsp_update_jumptoapp_evt_cbk(void);
float ibat_curr_limt = 1000;
uart_msg uart_msg_inst;
extern __IO uint32_t uwTick;
int time_ret = 0;
uint8_t interflash_ret = 0;
system_status sys_stat;
extern uint8_t ab_system_arr[512]MEM_ALIGNED(8);
extern uint32_t flashdestination;

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
	__enable_irq();
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
	//检测到系统电后禁止typec充电功能
	//测试系统电开启能否唤醒mcu
	//检测到系统电低于18v将12V_Switch_En拉高，在拉高的过程中，检测到系统电高于20V将12V_Switch_En拉低
	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_LPUART1_UART_Init();
	MX_ADC1_Init();

	// FLASH_If_Init();
	/* USER CODE BEGIN 2 */
	i2c_init(DS3232_PORT);
	i2c_init(SW6301_PORT);
	i2c_init(DS2782_PORT);
	HAL_Delay(2000);
	turnon_soc_func();
	HAL_Delay(1000);	
	API_WatchDog_Enable(0);
	//HAL_Delay(5000);
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
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5_HIGH);
	rtc_ret = ds3232_probe();	
	low_power_func(1);
	FLASH_If_Read(AB_SYSTEM_FLAG_ADDRESS, ab_system_arr, 4);
	memcpy(&binfile.curr_partition, ab_system_arr, 1);
	
	flashdestination = B_SYSTEM_APPLICATION_ADDRESS;
	
	if(!(binfile.curr_partition == 0 || binfile.curr_partition == 1))
		return 1;

	while (1)
	{  
		//ds3232_read_time(&rct_read);
	/* USER CODE END WHILE */
		Get_Adc1_Average_Value();
		ADC_Smooth();
		//ds278x_get_status(&my_info);
	//my_ret = ds2782_get_capacity(&my_capacity);
	//if(my_ret)
	//	return my_ret;		
	/* USER CODE BEGIN 3 */
		my_ret1 = get_chip_version(&version);
#if 0//here
		my_ret3 = get_mode_status(&mode_stat);
	//charge_mode = 0;
		my_ret2 = get_c_no_load_status(&load_value);
		my_ret4 = get_system_status(&sys_stat);

		my_ret10 = ds278x_get_status(&my_info);
		//low_power_func(1);

		//event_int_enable_func(write_event_int);
		//my_ret6 = enable_write_low_register();
		//mode_set_value.output_switch = 1;
		//my_ret5 = mode_set_func(mode_set_value);
		//uint8_t my_value = 0x18;
#endif
#if 1
		my_ret5 =  sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_IBAT, &read_ibat_curr);
		my_ret5 =  sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_VBUS, &read_vbus_vol);
		my_ret5 =  sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_VBAT, &read_vbat_vol);
		sw6301_read_adc(SW6301_ADC_CHANNEL_IBUS, &read_ibus_curr);
		quick_charge_indication(&quick_value);
		//charge_switch_func(0);
		set_charge_ibat_current_limit_value(ibat_curr_limt);
		//i2c_sw6301_read(0x10, (uint8_t *)&port_curr);
		//real_port_curr = port_curr * 50;
		//i2c_sw6301_read(0x11, (uint8_t *)&battery_curr);
		//real_battery_curr = battery_curr * 100;
		//i2c_sw6301_read(0x40, &reg40_value);
		//i2c_sw6301_read(0x43, &reg43_value);
		//i2c_sw6301_read(0x44, &reg44_value);
	//read_ibus_curr
#endif

#if 1
		heartbeat_timeout_func();
		get_system_status(&sys_stat);
		no_charge_func();
		mcu_send_readymsg_func();
		power_manager_func();
		API_WatchDog_FeedDog();
#endif
		LEDG_ON;
		//HAL_Delay(5000);

		//while(1)
		//{
		//	if(Key_Scan() == 1)
		//	{
		//		API_EnteryStandby();
		//	}
		//}
		

		//HAL_Delay(100);
		//LEDG_OFF;
		//HAL_Delay(100);
		//API_EnteryStandby();
#if 0
	ds278x_get_status(&my_2782_info);
	ds278x_get_temp(&my_temp_value);
	ds2782_get_capacity(&my_capacity);
		my_ret4 = get_system_status(&sys_stat);
	my_test_size = sizeof(my_test_value);
	quick_charge_ret = quick_charge_indication(&quick_value);
	//my_ret5 =	sw6301_read_adc((uint8_t)SW6301_ADC_CHANNEL_IBAT, &read_ibat_curr);
#endif

#if 1
		uart_msg_proc_flow();
		get_battery_info_func();
		//self_check_pro_flow();
		shutdown_func_from_soc();
		//shutdown_func_from_button();
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
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
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
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pins: PC13*/
  //GPIO_InitStruct.Pin = GPIO_PIN_13;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
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
