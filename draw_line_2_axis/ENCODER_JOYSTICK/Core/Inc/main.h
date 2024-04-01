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
#define DIRX_Pin GPIO_PIN_2
#define DIRX_GPIO_Port GPIOC
#define DIRY_Pin GPIO_PIN_3
#define DIRY_GPIO_Port GPIOC
#define TIM5_CH3_Pin GPIO_PIN_2
#define TIM5_CH3_GPIO_Port GPIOA
#define RETURN_HOME_Pin GPIO_PIN_3
#define RETURN_HOME_GPIO_Port GPIOA
#define RETURN_HOME_EXTI_IRQn EXTI3_IRQn
#define TIM3_CH1_Pin GPIO_PIN_6
#define TIM3_CH1_GPIO_Port GPIOA
#define SAVE_POS_Pin GPIO_PIN_14
#define SAVE_POS_GPIO_Port GPIOB
#define SAVE_POS_EXTI_IRQn EXTI15_10_IRQn
#define RUN_AFTER_SAVE_Pin GPIO_PIN_15
#define RUN_AFTER_SAVE_GPIO_Port GPIOB
#define RUN_AFTER_SAVE_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
