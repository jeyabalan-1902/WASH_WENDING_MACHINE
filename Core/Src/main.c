/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>

#include "TM1637.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SEG_BLANK 0x00
#define COLON_ON  0x80
#define COLON_OFF 0x00

#define DEBOUNCE_TIME_MS 10
#define RESET_DELAY_MS 2000

#define PULSE_TIMEOUT 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t initial_display_done = 0;
volatile uint8_t pulse_interrupt_Flag = 0;
uint16_t readPotvalue;
volatile uint32_t current_time_ms = 0;
volatile uint8_t pulseCount = 0;
volatile uint8_t pulseProcessed = 0;
volatile uint16_t timeoutCounter = 0;
static uint32_t pulse_start_time = 0;
uint32_t pulse_timeout = 500;
uint32_t last_pulse_time = 0;
volatile uint8_t coin_pulse = 0;

uint8_t state = 0;
uint32_t task_start_time = 0;

int countdown_seconds;

uint8_t digit_map[15] = {
        0x3F, // 0
        0x06, // 1
        0x5B, // 2
        0x4F, // 3
        0x66, // 4
        0x6D, // 5
        0x7D, // 6
        0x07, // 7
        0x7F, // 8
        0x6F,  // 9
		0x71,  //F  10
		0x5E,  //D  11
		0x40,   //dash 12
		0x6D,   //S 13
		0x71,   //C 14
    };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void DisplayDashes(void);
void Display_fifty(void);
void TM1637_Countdown_20Sec(void);
void TM1637_DisplayClear(void);
void processPulse();
void Relay_off_time(uint16_t potvalue);
void Display_01(void);
void Display_02(void);
void Display_03(void);
void Display_SC01(void);
void Display_SC02(void);
void Display_OFF(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc);
  printf("system start\n\r");
  state = 0;
  task_start_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //DisplayDashes();
	  switch (state)
	  {
	      case 0:
			  DisplayDashes();
			  processPulse();
			  break;

	      case 1:

			  if (!initial_display_done)
			  {
				  Display_fifty();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }

			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  if (HAL_GetTick() - task_start_time < 23000)
				  {
					  TM1637_Countdown_20Sec();
				  }
				  else
				  {
					  if (countdown_seconds == 0)
					  {
						  printf("20 sec countdown completed\n");
						  HAL_ADC_PollForConversion(&hadc, 1000);
						  readPotvalue = HAL_ADC_GetValue(&hadc);
						  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_SET);
					      Relay_off_time(readPotvalue);
					  }
				  }
			  }
			  break;

	      case 2:
			  if (!initial_display_done)
			  {
				  Display_01();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  state = 3;
			  }
			  break;

		  case 3:
			  if (HAL_GetTick() - task_start_time < 23000)
			  {
				  TM1637_Countdown_20Sec();
			  }
			  else
			  {
				  if(countdown_seconds == 0)
				  {
					  printf("GPIO 3 5 7 enabled\n\r");
					  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
					  task_start_time = HAL_GetTick();
					  state = 4;
				  }
			  }
			  break;

		  case 4:
			  if (HAL_GetTick() - task_start_time >= 30000)
			  {
				  printf("GPIO 3 5 7 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = 20;
			  }
			  break;

		  case 5:
			  if (!initial_display_done)
			  {
				  Display_02();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  state = 6;
			  }
			  break;

		  case 6:
			  if (HAL_GetTick() - task_start_time < 23000)
			  {
				  TM1637_Countdown_20Sec();
			  }
			  else
			  {
				  if(countdown_seconds == 0)
				  {
					  printf("GPIO 3 5 6 7 enabled\n\r");
					  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
					  task_start_time = HAL_GetTick();
					  state = 7;
				  }
			  }
			  break;

		  case 7:
			  if (HAL_GetTick() - task_start_time >= 30000)
			  {
				  printf("GPIO 3 5 6 7 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = 20;
			  }
			  break;

		  case 8:
			  if (!initial_display_done)
			  {
				  Display_03();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  state = 9;
			  }
			  break;

		  case 9:
			  if (HAL_GetTick() - task_start_time < 23000)
			  {
				  TM1637_Countdown_20Sec();
			  }
			  else
			  {
				  if(countdown_seconds == 0)
				  {
					  printf("GPIO 2 5 6 7 enabled\n\r");
					  HAL_GPIO_WritePin(SIGNAL_2_GPIO_Port, SIGNAL_2_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
					  task_start_time = HAL_GetTick();
					  state = 10;
				  }
			  }
			  break;

		  case 10:
			  if (HAL_GetTick() - task_start_time >= 30000)
			  {
				  printf("GPIO 2 5 6 7 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_2_GPIO_Port, SIGNAL_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = 20;
			  }
			  break;

		  case 11:
			  if (!initial_display_done)
			  {
				  Display_SC01();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  state = 12;
			  }
			  break;

		  case 12:
			  if (HAL_GetTick() - task_start_time < 23000)
			  {
				  TM1637_Countdown_20Sec();
			  }
			  else
			  {
				  if(countdown_seconds == 0)
				  {
					  printf("GPIO 4 5 enabled\n\r");
					  HAL_GPIO_WritePin(SIGNAL_4_GPIO_Port, SIGNAL_4_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
					  task_start_time = HAL_GetTick();
					  state = 13;
				  }
			  }
			  break;

		  case 13:
			  if (HAL_GetTick() - task_start_time >= 30000)
			  {
				  printf("GPIO 4 5 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_4_GPIO_Port, SIGNAL_4_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = 20;
			  }
			  break;

		  case 14:
			  if (!initial_display_done)
			  {
				  Display_SC02();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  state = 15;
			  }
			  break;

		  case 15:
			  if (HAL_GetTick() - task_start_time < 23000)
			  {
				  TM1637_Countdown_20Sec();
			  }
			  else
			  {
				  if(countdown_seconds == 0)
				  {
					  printf("GPIO 5 6 enabled\n\r");
					  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_SET);
					  task_start_time = HAL_GetTick();
					  state = 16;
				  }
			  }
			  break;

		  case 16:
			  if (HAL_GetTick() - task_start_time >= 30000)
			  {
				  printf("GPIO 5 6 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_RESET);
				  task_start_time = HAL_GetTick(); // Record the start time for the next state
				  state = 20;
			  }
			  break;

		  case 17:
			  if (!initial_display_done)
			  {
				  Display_OFF();
				  task_start_time = HAL_GetTick();
				  initial_display_done = 1;
				  countdown_seconds = 20;
			  }
			  if (HAL_GetTick() - task_start_time >= 2000)
			  {
				  state = 18;
			  }
			  break;

		  case 18:
			  if (HAL_GetTick() - task_start_time < 23000)
			  {
				  TM1637_Countdown_20Sec();
			  }
			  else
			  {
				  if(countdown_seconds == 0)
				  {
					  printf("GPIO 2 3 4 5 7 enabled\n\r");
					  HAL_GPIO_WritePin(SIGNAL_2_GPIO_Port, SIGNAL_2_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_4_GPIO_Port, SIGNAL_4_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_SET);
					  task_start_time = HAL_GetTick();
					  state = 19;
				  }
			  }
			  break;

		  case 19:
			  if (HAL_GetTick() - task_start_time >= 30000)
			  {
				  printf("GPIO 2 3 4 5 7 disabled\n\r");
				  HAL_GPIO_WritePin(SIGNAL_2_GPIO_Port, SIGNAL_2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_3_GPIO_Port, SIGNAL_3_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_4_GPIO_Port, SIGNAL_4_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(SIGNAL_7_GPIO_Port, SIGNAL_7_Pin, GPIO_PIN_RESET);
				  printf("return to IDLE\n\r");
				  state = 0;
				  coin_pulse = 0;
				  initial_display_done = 0;
			  }
			  break;

		  case 20:
			  printf("GPIO A & B enabled\n\r");
			  HAL_GPIO_WritePin(SIGNAL_A_GPIO_Port, SIGNAL_A_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(SIGNAL_B_GPIO_Port, SIGNAL_B_Pin, GPIO_PIN_SET);
			  HAL_Delay(5000);
			  printf("GPIO A & B disabled\n\r");
			  HAL_GPIO_WritePin(SIGNAL_A_GPIO_Port, SIGNAL_A_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SIGNAL_B_GPIO_Port, SIGNAL_B_Pin, GPIO_PIN_RESET);
			  task_start_time = HAL_GetTick();
			  state = 21;
			  break;

		  case 21:
			  TM1637_DisplayClear();
			  if (HAL_GetTick() - task_start_time >= 90000)
			  {
				  printf("return to IDLE\n\r");
				  state = 0;
				  coin_pulse = 0;
				  initial_display_done = 0;

			  }
			  break;

		  default:
			  state = 0;
			  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIGNAL_1_Pin|SIGNAL_2_Pin|SIGNAL_3_Pin|SIGNAL_4_Pin
                          |SIGNAL_5_Pin|SIGNAL_6_Pin|SIGNAL_7_Pin|SIGNAL_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SIGNAL_A_Pin|SIGNAL_B_Pin|REL_SIG_1_Pin|CLK_Pin
                          |DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIGNAL_1_Pin SIGNAL_2_Pin SIGNAL_3_Pin SIGNAL_4_Pin
                           SIGNAL_5_Pin SIGNAL_6_Pin SIGNAL_7_Pin SIGNAL_8_Pin */
  GPIO_InitStruct.Pin = SIGNAL_1_Pin|SIGNAL_2_Pin|SIGNAL_3_Pin|SIGNAL_4_Pin
                          |SIGNAL_5_Pin|SIGNAL_6_Pin|SIGNAL_7_Pin|SIGNAL_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SIGNAL_A_Pin SIGNAL_B_Pin REL_SIG_1_Pin */
  GPIO_InitStruct.Pin = SIGNAL_A_Pin|SIGNAL_B_Pin|REL_SIG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COIN_Pin */
  GPIO_InitStruct.Pin = COIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin DATA_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Relay_off_time(uint16_t potvalue)
{
	static uint32_t relay_start_time = 0;  // Static to retain value across function calls

	if (relay_start_time == 0)
	{
		relay_start_time = HAL_GetTick();  // Set the time only once
	}
	 if(potvalue <= 100)
	  {
		  if (HAL_GetTick() - relay_start_time >= 15000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
			  readPotvalue = 0;
		  }
	  }
	 else if(potvalue > 100 && potvalue < 200)
	 {
		  if (HAL_GetTick() - relay_start_time >= 20000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 200 && potvalue < 300)
	 {
		  if (HAL_GetTick() - relay_start_time >= 25000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 300 && potvalue < 400)
	 {
		  if (HAL_GetTick() - relay_start_time >= 30000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 400 && potvalue < 500)
	 {
		  if (HAL_GetTick() - relay_start_time >= 35000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 500 && potvalue < 600)
	 {
		  if (HAL_GetTick() - relay_start_time >= 40000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 600 && potvalue < 700)
	 {
		  if (HAL_GetTick() - relay_start_time >= 45000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 700 && potvalue < 800)
	 {
		  if (HAL_GetTick() - relay_start_time >= 50000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 800 && potvalue < 900)
	 {
		  if (HAL_GetTick() - relay_start_time >= 55000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
	 else if(potvalue > 900)
	 {
		  if (HAL_GetTick() - relay_start_time >= 60000)
		  {
			  HAL_GPIO_WritePin(REL_SIG_1_GPIO_Port, REL_SIG_1_Pin, GPIO_PIN_RESET);
			  printf("return to IDLE\n\r");
			  state = 0;
			  coin_pulse = 0;
			  initial_display_done = 0;
		  }
	 }
}

void processPulse()
{
	 if (pulse_interrupt_Flag)
	 {
		HAL_Delay(50);
//		if (pulse_start_time == 0)
//		{
//			pulse_start_time = HAL_GetTick();
//		}

		if ((HAL_GetTick() - pulse_start_time) >= pulse_timeout)
		{
			__disable_irq();
			switch (coin_pulse)
			{
				case 1:
					printf("1 pulse received\n\r");
					state = 1;
					break;
				case 2:
					printf("2 pulses received\n\r");
					state = 2;
					break;
				case 3:
					printf("2 pulses received\n\r");
					state = 2;
					break;
				case 4:
					printf("3 pulses received\n\r");
					state = 5;
					break;
				case 5:
					printf("3 pulses received\n\r");
					state = 5;
					break;
				case 6:
					printf("6 pulses received\n\r");
					state = 8;
					break;
				case 7:
					printf("7 pulses received\n\r");
					state = 11;
					break;
				case 8:
					printf("8 pulses received\n\r");
					state = 14;
					break;
				case 9:
					printf("9 pulses received\n\r");
					state = 17;
					break;
				default:
					printf("Invalid number of pulses\n\r");
					state = 0;
					break;
			}
		}
	 }
}

void TM1637_Countdown_20Sec(void)
{
	static bool colon_state = false;
	static uint32_t last_update_time = 0;
	uint8_t display_data[4] = {0x00, 0x00, 0x00, 0x00};
    if (HAL_GetTick() - last_update_time >= 1000)
    {
        last_update_time = HAL_GetTick();
        colon_state = !colon_state;

        display_data[0] = digit_map[0];
		display_data[1] = digit_map[0];
        display_data[2] = digit_map[(countdown_seconds / 10)];
        display_data[3] = digit_map[(countdown_seconds % 10)];

        if (colon_state)
        {
			display_data[1] |= 0x80;
		} else
		{
			display_data[1] |= 0x00;
		}
        TM1637_WriteData(0xC0, display_data, 4);
        printf("Countdown: %02d seconds\n", countdown_seconds);
        countdown_seconds--;
        if (countdown_seconds < 0) {
        	countdown_seconds = 0;
			printf("20 sec countdown completed\n");
        }
    }
}

void Display_fifty(void)
{
	uint8_t data[4] = {0x00, digit_map[5], digit_map[0], digit_map[10]};
	TM1637_WriteData(0xC0, data, 4);
	printf("Displayed 50 F\n\r");
}

void Display_01(void)
{
	uint8_t data[4] = {0x00, 0x00, digit_map[0], digit_map[1]};
	TM1637_WriteData(0xC0, data, 4);
	printf("displayed 1 \n\r");
}

void Display_02(void)
{
	uint8_t data[4] = {0x00, 0x00, digit_map[0], digit_map[2]};
	TM1637_WriteData(0xC0, data, 4);
	printf("displayed 02 \n\r");
}

void Display_03(void)
{
	uint8_t data[4] = {0x00, 0x00, digit_map[0], digit_map[3]};
	TM1637_WriteData(0xC0, data, 4);
	printf("displayed 3 \n\r");
}

void Display_SC01(void)
{
	uint8_t data[4] = {digit_map[13], digit_map[14], digit_map[0], digit_map[1]};
	TM1637_WriteData(0xC0, data, 4);
	printf("displayed SC01\n\r");
}

void Display_SC02(void)
{
	uint8_t data[4] = {digit_map[13], digit_map[14], digit_map[0], digit_map[2]};
	TM1637_WriteData(0xC0, data, 4);
	printf("displayed SC02\n\r");
}

void Display_OFF(void)
{
	uint8_t data[4] = {0x00, digit_map[0], digit_map[0], digit_map[10]};
	TM1637_WriteData(0xC0, data, 4);
	printf("displayed OFF\n\r");
}


void DisplayDashes(void)
{
    uint8_t data[4] = {digit_map[12],digit_map[12], digit_map[12], digit_map[12]};
    TM1637_WriteData(0xC0, data, 4);
    //printf("Display Dashes\n\r");
    HAL_GPIO_WritePin(SIGNAL_4_GPIO_Port, SIGNAL_4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIGNAL_5_GPIO_Port, SIGNAL_5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIGNAL_6_GPIO_Port, SIGNAL_6_Pin, GPIO_PIN_SET);
}

void TM1637_DisplayClear(void)
{
	uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
	TM1637_WriteData(0xC0, data, 4);
	printf("System going to IDLE state.....it takes 2 minutes\n\r");
}

#ifdef __GNUC__
#define UART_printf   int __io_putchar(int ch)
UART_printf
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == COIN_Pin)
	{
		uint32_t current_time = HAL_GetTick();

		if ((current_time - last_pulse_time) > 50)  // Debounce filter (50 ms)
		{
			last_pulse_time = current_time;
			if (pulse_interrupt_Flag == 0)
			{
				coin_pulse = 1;  // First pulse detected
			}
			else
			{
				coin_pulse++;  // Increment pulse count
			}
			pulse_interrupt_Flag = 1;
			pulse_start_time = HAL_GetTick();  // Restart timeout timer on each new pulse
		}
	}
}


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
