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
#define DIRX_Pin GPIO_PIN_5
#define DIRX_GPIO_Port GPIOC
#define DIRY_Pin GPIO_PIN_0
#define DIRY_GPIO_Port GPIOB
#define GRAB_OBJECT_Pin GPIO_PIN_15
#define GRAB_OBJECT_GPIO_Port GPIOB
#define STATE_CONVEYOR_Pin GPIO_PIN_7
#define STATE_CONVEYOR_GPIO_Port GPIOC
#define SIG_STOP_CONVEYOR_Pin GPIO_PIN_5
#define SIG_STOP_CONVEYOR_GPIO_Port GPIOB
#define SIG_STOP_CONVEYOR_EXTI_IRQn EXTI9_5_IRQn
#define ENABLE_Y_Pin GPIO_PIN_6
#define ENABLE_Y_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
