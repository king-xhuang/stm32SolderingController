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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TIM3_PWM_Pin GPIO_PIN_6
#define TIM3_PWM_GPIO_Port GPIOA
#define OUT_17_Pin GPIO_PIN_7
#define OUT_17_GPIO_Port GPIOA
#define OUT_19_Pin GPIO_PIN_1
#define OUT_19_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_12
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_B_Pin GPIO_PIN_13
#define ENC_B_GPIO_Port GPIOB
#define ENC_SW_Pin GPIO_PIN_14
#define ENC_SW_GPIO_Port GPIOB
#define ENC_SW_EXTI_IRQn EXTI15_10_IRQn
#define Dock_Pin GPIO_PIN_15
#define Dock_GPIO_Port GPIOB
#define Dock_EXTI_IRQn EXTI15_10_IRQn
#define Power_Pin GPIO_PIN_8
#define Power_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_4
#define BEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
