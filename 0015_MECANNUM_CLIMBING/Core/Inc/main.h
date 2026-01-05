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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LR_LPWM_Pin GPIO_PIN_5
#define LR_LPWM_GPIO_Port GPIOE
#define LR_RPWM_Pin GPIO_PIN_6
#define LR_RPWM_GPIO_Port GPIOE
#define LiftRear_LPWM_Pin GPIO_PIN_6
#define LiftRear_LPWM_GPIO_Port GPIOA
#define LiftRear_RPWM_Pin GPIO_PIN_7
#define LiftRear_RPWM_GPIO_Port GPIOA
#define LF_LPWM_Pin GPIO_PIN_0
#define LF_LPWM_GPIO_Port GPIOB
#define LF_RPWM_Pin GPIO_PIN_1
#define LF_RPWM_GPIO_Port GPIOB
#define LiftFront_LPWM_Pin GPIO_PIN_14
#define LiftFront_LPWM_GPIO_Port GPIOB
#define LiftFront_RPWM_Pin GPIO_PIN_15
#define LiftFront_RPWM_GPIO_Port GPIOB
#define RF_LPWM_Pin GPIO_PIN_4
#define RF_LPWM_GPIO_Port GPIOB
#define RF_RPWM_Pin GPIO_PIN_5
#define RF_RPWM_GPIO_Port GPIOB
#define RR_LPWM_Pin GPIO_PIN_8
#define RR_LPWM_GPIO_Port GPIOB
#define RR_RPWM_Pin GPIO_PIN_9
#define RR_RPWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
