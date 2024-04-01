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
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_CHANNEL TIM_CHANNEL_2
#define A1_CHANNEL TIM_CHANNEL_2
#define A2_CHANNEL TIM_CHANNEL_4
#define A3_CHANNEL TIM_CHANNEL_4
#define A4_CHANNEL TIM_CHANNEL_1
#define length 48
#define MAX_VALUE 100000
#define VALUE_FINISH_TIMER 31
#define BIT_STOP_BASE 0
#define BIT_STOP_A1 1
#define BIT_STOP_A2 2
#define BIT_STOP_A3 3
#define BIT_STOP_A4 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int pulse_base = 0, pulse_a1 = 0, pulse_a2 = 0, pulse_a3 = 0, pulse_a4 = 0,
		count_dot = 0, count_finish_timer = 0;
bool flag_receive_uart = false, flag_dir_change_base = false,
		flag_dir_change_a1 = false, flag_dir_change_a2 = 0, flag_dir_change_a3 =
				0, flag_dir_change_a4 = 0;
unsigned char flag_finish_timer = 0;
uint32_t value_base = MAX_VALUE, value_a1 = MAX_VALUE, value_a2 = MAX_VALUE,
		value_a3 = MAX_VALUE, value_a4 = MAX_VALUE, step_base = 0, step_a1 = 0,
		step_a2 = 0, step_a3 = 0, step_a4 = 0;
char rxByte[length];
char Home[length] = "Home----------------------";
char Stop[length] = "Stop----------------------";
typedef enum {
	Home_State = 1, Stop_State = 2,
} State;
typedef enum {
	ClockWise = 0, CounterClockWise = 1,
} Direction;
typedef enum {
	On = 0, Off = 1,
} Switch;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if defined(__GNUC__)
int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
#elif defined(__ICCARM__)#include "LowLevelIOInterface.h"

size_t __write(int handle,
  const unsigned char * buffer, size_t size) {
  HAL_UART_Transmit( &huart1, (uint8_t * ) buffer, size, UART_Delay);
  return size;
}
#elif defined(__CC_ARM)
int fputc(int ch, FILE * f) {
  HAL_UART_Transmit( &huart1, (uint8_t * ) &ch, 1, UART_Delay);
  return ch;
}
#endif
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (flag_receive_uart) {
		if (htim->Instance == TIM2) {
			pulse_a4 =
					!(flag_finish_timer & (1 << BIT_STOP_A4)) ?
							pulse_a4 + 1 : pulse_a4;
			pulse_base =
					!(flag_finish_timer & (1 << BIT_STOP_BASE)) ?
							pulse_base + 1 : pulse_base;
			if (pulse_a4 >= step_a4) {
				HAL_TIM_PWM_Stop(&htim2, A4_CHANNEL);
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A2);
			}
			if (pulse_base >= step_base) {
				HAL_TIM_PWM_Stop(&htim2, BASE_CHANNEL);
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_BASE);
			}
		}
		if (htim->Instance == TIM3) {
			pulse_a1 =
					!(flag_finish_timer & (1 << BIT_STOP_A1)) ?
							pulse_a1 + 1 : pulse_a1;
			pulse_a3 =
					!(flag_finish_timer & (1 << BIT_STOP_A3)) ?
							pulse_a3 + 1 : pulse_a3;
			if (pulse_a1 >= step_a1) {
				HAL_TIM_PWM_Stop(&htim3, A1_CHANNEL);
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A1);

			}
			if (pulse_a3 >= step_a3) {
				HAL_TIM_PWM_Stop(&htim3, A3_CHANNEL);
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A3);
			}

		}
		if (htim->Instance == TIM4) {
			pulse_a2 =
					(flag_finish_timer & (1 << BIT_STOP_A2)) ?
							pulse_a2 + 1 : pulse_a2;
			if (pulse_a2 >= step_a2) {
				HAL_TIM_PWM_Stop(&htim4, A2_CHANNEL);
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A4);
			}

		}
		if (flag_finish_timer == VALUE_FINISH_TIMER) {
			flag_receive_uart = false;
			pulse_base = 0;
			pulse_a1 = 0;
			pulse_a2 = 0;
			pulse_a3 = 0;
			pulse_a4 = 0;
			flag_finish_timer = 0;

		}
	}

}

void reset_value_uart(void) {
	step_base = 0;
	step_a1 = 0;
	step_a2 = 0;
	step_a3 = 0;
	step_a4 = 0;
	value_base = MAX_VALUE;
	value_a1 = MAX_VALUE;
	value_a2 = MAX_VALUE;
	value_a3 = MAX_VALUE;
	value_a4 = MAX_VALUE;
	count_dot = 0;
	flag_dir_change_base = false;
	flag_dir_change_a1 = false;
	flag_dir_change_a2 = false;
	flag_dir_change_a3 = false;
	flag_dir_change_a4 = false;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		reset_value_uart();
		HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length);
		printf("Mode: XYZ\r\n");
		for (int i = 0; i < length; i++) {
			if ((rxByte[i] >= 'a' && rxByte[i] <= 'z')
					|| (rxByte[i] >= 'A' && rxByte[i] <= 'Z')
					|| rxByte[i] == '=') {
				continue;
			} else {
				if (rxByte[i] == ',') {
					count_dot++;
					continue;
				}
				switch (count_dot) {
				case 0:
					if (rxByte[i] == '-') {
						HAL_GPIO_WritePin(GPIOA, DIR_BASE_Pin,
								CounterClockWise);
//						HAL_GPIO_WritePin(PORT_DIR, DIR_BASE_Pin,
//								CounterClockWise);
						flag_dir_change_base = true;
						value_base /= 10;
						continue;
					}
					if (!flag_dir_change_base) {
						HAL_GPIO_WritePin(GPIOA, DIR_BASE_Pin, ClockWise);

					}
					step_base += ((int) rxByte[i] - '0') * value_base;
					value_base /= 10;
					break;
				case 1:
					if (rxByte[i] == '-') {
						HAL_GPIO_WritePin(GPIOA, DIR_A1_Pin, CounterClockWise);
						flag_dir_change_a1 = true;
						value_a1 /= 10;
						continue;
					}
					if (!flag_dir_change_a1) {
						HAL_GPIO_WritePin(GPIOA, DIR_A1_Pin, ClockWise);
					}
					step_a1 += ((int) rxByte[i] - '0') * value_a1;
					value_a1 /= 10;
					break;
				case 2:
					if (rxByte[i] == '-') {
						HAL_GPIO_WritePin(GPIOA, DIR_A2_Pin, CounterClockWise);
						value_a2 /= 10;
						flag_dir_change_a2 = true;
						continue;
					}
					if (!flag_dir_change_a2) {
						HAL_GPIO_WritePin(GPIOA, DIR_A2_Pin, ClockWise);
					}
					step_a2 += ((int) rxByte[i] - '0') * value_a2;
					value_a2 /= 10;
					break;
				case 3:
					if (rxByte[i] == '-') {
						HAL_GPIO_WritePin(GPIOA, DIR_A3_Pin, CounterClockWise);
						value_a3 /= 10;
						flag_dir_change_a3 = true;
						continue;
					}
					if (!flag_dir_change_a3) {
						HAL_GPIO_WritePin(GPIOA, DIR_A3_Pin, ClockWise);
					}
					step_a3 += ((int) rxByte[i] - '0') * value_a3;
					value_a3 /= 10;
					break;
				case 4:
					if (rxByte[i] == '-') {
						HAL_GPIO_WritePin(GPIOA, DIR_A4_Pin, CounterClockWise);
						value_a4 /= 10;
						flag_dir_change_a4 = true;
						continue;
					}
					if (!flag_dir_change_a4) {
						HAL_GPIO_WritePin(GPIOA, DIR_A4_Pin, ClockWise);
					}
					step_a4 += ((int) rxByte[i] - '0') * value_a4;
					value_a4 /= 10;
					break;
				}
			}
			flag_finish_timer = 0;
			if (step_base) {
				HAL_TIM_PWM_Start(&htim2, BASE_CHANNEL);
			} else {
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_BASE);
			}
			if (step_a1) {
				HAL_TIM_PWM_Start(&htim3, A1_CHANNEL);
			} else {
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A1);
			}
			if (step_a2) {
				HAL_TIM_PWM_Start(&htim4, A2_CHANNEL);
			} else {
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A2);
			}
			if (step_a3) {
				HAL_TIM_PWM_Start(&htim3, A3_CHANNEL);
			} else {
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A3);
			}
			if (step_a4) {
				HAL_TIM_PWM_Start(&htim2, A4_CHANNEL);
			} else {
				flag_finish_timer = flag_finish_timer | (1 << BIT_STOP_A4);
			}

		}
		flag_receive_uart = true;
	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 720 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 50 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 30;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 720 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 50;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 720 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 100 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 50 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	DIR_A4_Pin | DIR_BASE_Pin | DIR_A1_Pin | DIR_A2_Pin | DIR_A3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : DIR_A4_Pin DIR_BASE_Pin DIR_A1_Pin DIR_A2_Pin
	 DIR_A3_Pin */
	GPIO_InitStruct.Pin = DIR_A4_Pin | DIR_BASE_Pin | DIR_A1_Pin | DIR_A2_Pin
			| DIR_A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
