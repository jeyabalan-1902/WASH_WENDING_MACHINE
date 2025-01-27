/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SIGNAL_1_Pin GPIO_PIN_0
#define SIGNAL_1_GPIO_Port GPIOA
#define SIGNAL_2_Pin GPIO_PIN_1
#define SIGNAL_2_GPIO_Port GPIOA
#define SIGNAL_3_Pin GPIO_PIN_2
#define SIGNAL_3_GPIO_Port GPIOA
#define SIGNAL_4_Pin GPIO_PIN_3
#define SIGNAL_4_GPIO_Port GPIOA
#define SIGNAL_5_Pin GPIO_PIN_4
#define SIGNAL_5_GPIO_Port GPIOA
#define SIGNAL_6_Pin GPIO_PIN_5
#define SIGNAL_6_GPIO_Port GPIOA
#define SIGNAL_7_Pin GPIO_PIN_6
#define SIGNAL_7_GPIO_Port GPIOA
#define SIGNAL_8_Pin GPIO_PIN_7
#define SIGNAL_8_GPIO_Port GPIOA
#define ADC_ADJ_Pin GPIO_PIN_0
#define ADC_ADJ_GPIO_Port GPIOB
#define SIGNAL_A_Pin GPIO_PIN_1
#define SIGNAL_A_GPIO_Port GPIOB
#define SIGNAL_B_Pin GPIO_PIN_10
#define SIGNAL_B_GPIO_Port GPIOB
#define REL_SIG_1_Pin GPIO_PIN_11
#define REL_SIG_1_GPIO_Port GPIOB
#define CUTOFF_RELAY_Pin GPIO_PIN_12
#define CUTOFF_RELAY_GPIO_Port GPIOA
#define COIN_Pin GPIO_PIN_7
#define COIN_GPIO_Port GPIOB
#define COIN_EXTI_IRQn EXTI4_15_IRQn
#define CLK_Pin GPIO_PIN_8
#define CLK_GPIO_Port GPIOB
#define DATA_Pin GPIO_PIN_9
#define DATA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
