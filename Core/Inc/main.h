/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define MOTOR_ENABLE_Pin GPIO_PIN_2
#define MOTOR_ENABLE_GPIO_Port GPIOA
#define MOTOR_1_ENABLE_Pin GPIO_PIN_3
#define MOTOR_1_ENABLE_GPIO_Port GPIOA
#define ENCODER_1_B_Pin GPIO_PIN_5
#define ENCODER_1_B_GPIO_Port GPIOA
#define ENCODER_2_A_Pin GPIO_PIN_6
#define ENCODER_2_A_GPIO_Port GPIOA
#define ENCODER_2_B_Pin GPIO_PIN_7
#define ENCODER_2_B_GPIO_Port GPIOA
#define MOTOR_2_ENABLE_Pin GPIO_PIN_4
#define MOTOR_2_ENABLE_GPIO_Port GPIOC
#define key1_Pin GPIO_PIN_5
#define key1_GPIO_Port GPIOC
#define key2_Pin GPIO_PIN_0
#define key2_GPIO_Port GPIOB
#define key3_Pin GPIO_PIN_1
#define key3_GPIO_Port GPIOB
#define key4_Pin GPIO_PIN_7
#define key4_GPIO_Port GPIOE
#define MOTOR_1_PWM_Pin GPIO_PIN_9
#define MOTOR_1_PWM_GPIO_Port GPIOE
#define MOTOR_2_PWM_Pin GPIO_PIN_11
#define MOTOR_2_PWM_GPIO_Port GPIOE
#define ENCODER_1_BD12_Pin GPIO_PIN_12
#define ENCODER_1_BD12_GPIO_Port GPIOD
#define ENCODER_1_A_Pin GPIO_PIN_13
#define ENCODER_1_A_GPIO_Port GPIOD
#define CLK_Pin GPIO_PIN_0
#define CLK_GPIO_Port GPIOD
#define DAT_Pin GPIO_PIN_1
#define DAT_GPIO_Port GPIOD
#define ENCODER_1_AB3_Pin GPIO_PIN_3
#define ENCODER_1_AB3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
