/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define PUMP1_Pin GPIO_PIN_0
#define PUMP1_GPIO_Port GPIOC
#define PUMP2_Pin GPIO_PIN_1
#define PUMP2_GPIO_Port GPIOC
#define PUMP3_Pin GPIO_PIN_2
#define PUMP3_GPIO_Port GPIOC
#define PUMP4_Pin GPIO_PIN_3
#define PUMP4_GPIO_Port GPIOC
#define CP1_Pin GPIO_PIN_1
#define CP1_GPIO_Port GPIOA
#define CP2_Pin GPIO_PIN_7
#define CP2_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_0
#define EN1_GPIO_Port GPIOB
#define EN2_Pin GPIO_PIN_1
#define EN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_12
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_13
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_14
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_15
#define IN4_GPIO_Port GPIOB
#define IN5_Pin GPIO_PIN_6
#define IN5_GPIO_Port GPIOC
#define IN6_Pin GPIO_PIN_7
#define IN6_GPIO_Port GPIOC
#define IN7_Pin GPIO_PIN_8
#define IN7_GPIO_Port GPIOC
#define IN8_Pin GPIO_PIN_9
#define IN8_GPIO_Port GPIOC
#define CP3_Pin GPIO_PIN_8
#define CP3_GPIO_Port GPIOA
#define DIR4_Pin GPIO_PIN_10
#define DIR4_GPIO_Port GPIOC
#define EN3_Pin GPIO_PIN_11
#define EN3_GPIO_Port GPIOC
#define EN4_Pin GPIO_PIN_12
#define EN4_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_3
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_5
#define DIR3_GPIO_Port GPIOB
#define CP4_Pin GPIO_PIN_9
#define CP4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
