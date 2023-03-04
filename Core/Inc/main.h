/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define B_Temp_Pin GPIO_PIN_0
#define B_Temp_GPIO_Port GPIOA
#define B_Temp_EXTI_IRQn EXTI0_IRQn
#define B_Water_Pin GPIO_PIN_1
#define B_Water_GPIO_Port GPIOA
#define B_Water_EXTI_IRQn EXTI1_IRQn
#define NSS_Pin GPIO_PIN_0
#define NSS_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_10
#define RST_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_11
#define DIO0_GPIO_Port GPIOB
#define Blink_Pin GPIO_PIN_13
#define Blink_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_14
#define D7_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_15
#define D6_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_9
#define D4_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOA
#define RW_Pin GPIO_PIN_11
#define RW_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_12
#define RS_GPIO_Port GPIOA
#define Led_Water_Pin GPIO_PIN_5
#define Led_Water_GPIO_Port GPIOB
#define Led_Temp_Pin GPIO_PIN_6
#define Led_Temp_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
