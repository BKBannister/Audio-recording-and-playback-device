/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "sinewave.h"
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
	uint8_t startUp[10] = {127, 128, 50, 48, 56, 56 ,57, 51, 53, 54};
	uint8_t stop[10] = {127, 128, 83, 116, 111, 112, 95, 95, 95, 95};
	uint8_t record_1[10] = {127, 128, 82, 101, 99, 111, 114, 100, 95, 49};
	uint8_t record_2[10] = {127, 128, 82, 101, 99, 111, 114, 100, 95, 50};
	uint8_t record_3[10] = {127, 128, 82, 101, 99, 111, 114, 100, 95, 51};
	uint8_t playBack_3[10] = {127, 128, 80, 108, 97, 121, 95, 95, 95, 51};
	uint8_t playBack_2[10] = {127, 128, 80, 108, 97, 121, 95, 95, 95, 50};
	uint8_t playBack_1[10] = {127, 128, 80, 108, 97, 121, 95, 95, 95, 49};

	extern volatile uint8_t Record_1;
	extern volatile uint8_t Record_2;
	extern volatile uint8_t Record_3;
	extern volatile uint8_t Playback_1;
	extern volatile uint8_t Playback_2;
	extern volatile uint8_t Playback_3;
	extern volatile uint8_t Stop;
	extern volatile uint8_t start_recording;

	uint8_t interrupt_time = 0;
	uint8_t last_interrupt_time = 0;
	volatile uint8_t debounce = 0;

	volatile bool led_flash_1 = false;
	volatile bool led_flash_2 = false;
	volatile bool led_flash_3 = false;

	uint16_t dacbuffer[1098];

	uint8_t adcbuffer[1024];
	int32_t tempsample;
	int8_t outputbuf[1024];

	int32_t average = 128;
	int32_t accumulator;
	int32_t numavg;
	float smoothed_sample;

	volatile enum state{idle, track_1, track_2, track_3, recording_1, recording_2, recording_3, stoped}machine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  //if((debounce == 1)&&(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)||HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)||HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)||HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4))){
	  	  //Start reording audio 1
		  if((GPIO_Pin == GPIO_PIN_8)&&(start_recording == 1)){
			Record_1 = 1;
		  }
		  //Start reording audio 2
		  if((GPIO_Pin == GPIO_PIN_9)&&(start_recording == 1)){
			Record_2 = 1;
		  }
		  //Start reording audio 3
		  if((GPIO_Pin == GPIO_PIN_7)&&(start_recording == 1)){
			Record_3 = 1;
		  }
		  //Start playback of audio 1
		  if((GPIO_Pin == GPIO_PIN_8)&&(start_recording == 0)){
			Playback_1 = 1;
		  }
		  //Start playback of audio 3
		  if((GPIO_Pin == GPIO_PIN_9)&&(start_recording == 0)){
			Playback_2 = 1;
		  }
		  //Start playback of audio 3
		  if((GPIO_Pin == GPIO_PIN_7)&&(start_recording == 0)){
			Playback_3 = 1;
		  }
		  //Stop all actions
		  if(GPIO_Pin == GPIO_PIN_4){
			Stop = 1;
		  }
		  debounce = 0;
  }
//}

void Hal_DAC_ConvCpltcallbackCh1(DAC_HandleTypeDef* hdac){
	HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)dacbuffer, 1098, DAC_ALIGN_12B_R);
	if(machine == track_1){
			wave_fillbuffer(dacbuffer + 549, 1, 549);
		}
	if(machine == track_2){
			wave_fillbuffer(dacbuffer + 549, 2, 549);
		}
	if(machine == track_3){
			wave_fillbuffer(dacbuffer + 549, 3, 549);
		}

}
void Hal_DAC_ConHalfCpltcallbackCh1(DAC_HandleTypeDef* hdac){
	if(machine == track_1){
		wave_fillbuffer(dacbuffer, 1, 549);
	}
	if(machine == track_2){
		wave_fillbuffer(dacbuffer, 2, 549);
	}
	if(machine == track_3){
		wave_fillbuffer(dacbuffer, 3, 549);
	}
}
void Hal_ADC_ConvCpltcallbackCh1(DAC_HandleTypeDef* hadc){
	huart2.gState = HAL_UART_STATE_READY;
	for(int i = 512; i < 1024; i++){
		accumulator += adcbuffer[i];
		tempsample = (int32_t)adcbuffer[i] - average;
		smoothed_sample = 0.125f*tempsample + 0.875f*smoothed_sample;
		tempsample = (int32_t)smoothed_sample;
		if(tempsample > 127){
			tempsample = 127;
		}
		if(tempsample < -128){
			tempsample = -128;
		}
		outputbuf[i] = (int8_t)tempsample;

	}
	numavg += 512;
	if(numavg >= 20480){
		average = accumulator/20480;
		accumulator = 0;
		numavg = 0;
	}
  	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)outputbuf+512, 512);
}
void Hal_ADC_ConHalfCpltcallbackCh1(DAC_HandleTypeDef* hadc){
	huart2.gState = HAL_UART_STATE_READY;
	for(int i = 0; i < 512; i++){
			accumulator += adcbuffer[i];
			tempsample = (int32_t)adcbuffer[i] - average;
			smoothed_sample = 0.125f*tempsample + 0.875f*smoothed_sample;
			tempsample = (int32_t)smoothed_sample;
			if(tempsample > 127){
				tempsample = 127;
			}
			if(tempsample < -128){
				tempsample = -128;
			}
			outputbuf[i] = (int8_t)tempsample;

		}
		numavg += 512;
		if(numavg >= 20480){
			average = accumulator/20480;
			accumulator = 0;
			numavg = 0;
		}
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)outputbuf, 512);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	enum state machine = idle;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, startUp, 10, 10);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcbuffer, 1024);
  HAL_TIM_Base_Start_IT(&htim2);
  wave_init();

  //wave_fillbuffer(dacbuffer, 1, 1024);
  //HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacbuffer, 1024, DAC_ALIGN_12B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

			  if((Playback_3 == 1)&&(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))){
					machine = track_3;
					Playback_3 = 0;
					HAL_UART_Transmit(&huart2, playBack_3, 10, 10);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
					led_flash_3 = true;
					led_flash_2 = false;
					led_flash_1 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
					wave_fillbuffer(dacbuffer, 3, 1098);
			  }
			  if((Playback_2 == 1)&&(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9))){
					machine = track_2;
					Playback_2 = 0;
					HAL_UART_Transmit(&huart2, playBack_2, 10, 10);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
					led_flash_2 = true;
					led_flash_3 = false;
					led_flash_1 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
					wave_fillbuffer(dacbuffer, 2, 1098);
			  }
			  if((Playback_1 == 1)&&(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))){
					machine = track_1;
					Playback_1 = 0;
					HAL_UART_Transmit(&huart2, playBack_1, 10, 10);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					led_flash_1 = true;
					led_flash_2 = false;
					led_flash_3 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
					wave_fillbuffer(dacbuffer, 1, 1098);
			  }
			  if((Record_3 == 1)&&(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))){
					machine = recording_3;
					Record_3 = 0;
					HAL_UART_Transmit(&huart2, record_3, 10, 10);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
					led_flash_3 = true;
					led_flash_2 = false;
					led_flash_1 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
			  }
			  if((Record_2 == 1)&&(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9))){
					machine = recording_2;
					Record_2 = 0;
					HAL_UART_Transmit(&huart2, record_2, 10, 10);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
					led_flash_2 = true;
					led_flash_3 = false;
					led_flash_1 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
			  }
			  if((Record_1 == 1)&&(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))){
					machine = recording_1;
					Record_1 = 0;
					HAL_UART_Transmit(&huart2, record_1, 10, 10);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
					led_flash_1 = true;
					led_flash_2 = false;
					led_flash_3 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
			  }
			  if((Stop == 1)&&(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4))){
					machine = stoped;
					Stop = 0;
					HAL_UART_Transmit(&huart2, stop, 10, 10);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
					led_flash_1 = false;
					led_flash_2 = false;
					led_flash_3 = false;
					HAL_TIM_Base_Stop_IT(&htim3);
					HAL_DAC_Stop_DMA (&hdac, DAC_CHANNEL_1);
			  }
		interrupt_time = HAL_GetTick();
		if((interrupt_time-last_interrupt_time) > 10){
			last_interrupt_time = interrupt_time;
		}

	  if(machine == idle){
		  //do nothing
	  }
	  if((machine == track_3)){
		  HAL_TIM_Base_Start_IT(&htim3);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacbuffer, 1098, DAC_ALIGN_12B_R);
	  }
	  if((machine == track_2)){
		  HAL_TIM_Base_Start_IT(&htim3);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacbuffer, 1098, DAC_ALIGN_12B_R);
	  }
	  if((machine == track_1)){
		  HAL_TIM_Base_Start_IT(&htim3);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dacbuffer, 1098, DAC_ALIGN_12B_R);
	  }
	  if((machine == recording_3)){
		  HAL_TIM_Base_Start_IT(&htim3);
	  }
	  if((machine == recording_2)){
		  HAL_TIM_Base_Start_IT(&htim3);
	  }
	  if((machine == recording_1)){
		  HAL_TIM_Base_Start_IT(&htim3);
	  }
	  if((machine == stoped)){
	  	  machine = idle;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1905;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 335;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 500000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
