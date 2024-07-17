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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CCW 1
#define CW 0
#define X_AXIS TIM_CHANNEL_2
#define Y_AXIS TIM_CHANNEL_1
#define POSITION1 1350
#define POSITION2 2000
#define POSITION3 2350
#define POSITION4 1200
#define WAITING_DROP_P2 2500
#define WAITING_DROP_P3 1700
#define WAITING_VACCUM_P2 40
#define WAITING_VACCUM_P3 70
#define TIME_WAITING_Y 300
#define length_string 15
#define MODE_POSITION_1 0
#define MODE_POSITION_2 1
#define MODE_POSITION_3 2
#define MODE_POSITION_4 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int current_pulse_x, current_pulse_y, index_array, state_dir;
char *stop = "Stop conveyor--";
//char position1[length_string] = "Move position 1";
char *position2 = "Move position 2";
char *position3 = "Move position 3";
char *position4 = "Move position 4";
char *start = "Start conveyor-";
char rxByte[length_string] = { 0 };
char *grab_object = "Grab object----";
int flag_finish_timer = 0;
bool flag_at_position_y = false, flag_return_home_x = false, flag_start_timer =
false, flag_return_home_y = false, flag_at_position_x = false,
		flag_start_timer_x = false, flag_start_timer_y = false,
		flag_finish_task = true, flag_wait_finish = false, flag_capture_object =
		false, flag_receive_move_robot = false;
int arr_pulse[4] = { POSITION1, POSITION2, POSITION3, POSITION4 };
int pulse_to_grab_object = 560, count_wait_vaccum_object = 0;
int time_waiting_drop[2] = { WAITING_DROP_P2, WAITING_DROP_P3 };
int time_waiting_vaccum[2] = { WAITING_VACCUM_P2, WAITING_VACCUM_P3 };
int time_to_wait_axis_x = 1100, time_to_wait_axis_y = 30;
char *uart_command[100];
char *robot_move_command;
int index_uart_command = 0, index_current_command = 0;
volatile int count_wait_drop_object = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
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
	if (htim->Instance == TIM2 && flag_start_timer_x) {
		if (!flag_at_position_x) {
			current_pulse_x++;
			if (current_pulse_x >= arr_pulse[index_array]) {
				__HAL_TIM_SET_COMPARE(&htim2, X_AXIS, 0);
				flag_at_position_x = true;
				flag_return_home_x = true;
				state_dir = HAL_GPIO_ReadPin(DIRX_GPIO_Port, DIRX_Pin);
				HAL_GPIO_WritePin(DIRX_GPIO_Port, DIRX_Pin, !state_dir);
			}
		}
		if (flag_return_home_x) {
			if (count_wait_drop_object < time_to_wait_axis_x) {
				count_wait_drop_object++;
			} else if (count_wait_drop_object >= time_to_wait_axis_x) {
				__HAL_TIM_SET_COMPARE(&htim2, X_AXIS, 40);
				current_pulse_x--;
				if (current_pulse_x <= 0) {

					__HAL_TIM_SET_COMPARE(&htim2, X_AXIS, 0);
					flag_return_home_x = false;
					flag_start_timer_x = false;
					current_pulse_x = 0;
					index_current_command++;
					flag_finish_task = true;
					count_wait_drop_object = 0;
					flag_capture_object = false;
					HAL_GPIO_WritePin(STATE_CONVEYOR_GPIO_Port,
					STATE_CONVEYOR_Pin, GPIO_PIN_SET);
					printf("Drop object successfully\n");
				}
			}
		}
	}
	if (htim->Instance == TIM3 && flag_start_timer_y) {
		if (!flag_at_position_y) {
			current_pulse_y++;
			if (current_pulse_y >= pulse_to_grab_object) {
				__HAL_TIM_SET_COMPARE(&htim3, Y_AXIS, 0);
				flag_at_position_y = true;
				flag_return_home_y = true;
				state_dir = HAL_GPIO_ReadPin(DIRY_GPIO_Port, DIRY_Pin);
				HAL_GPIO_WritePin(DIRY_GPIO_Port, DIRY_Pin, !state_dir);
			}
		}
		if (flag_return_home_y) {
			if (count_wait_vaccum_object < time_to_wait_axis_y) {
				count_wait_vaccum_object++;
			} else if (count_wait_vaccum_object >= time_to_wait_axis_y) {
				__HAL_TIM_SET_COMPARE(&htim3, Y_AXIS, 90);
				current_pulse_y--;
				HAL_GPIO_WritePin(GRAB_OBJECT_GPIO_Port, GRAB_OBJECT_Pin,
						GPIO_PIN_RESET);
				if (current_pulse_y <= 0) {
					__HAL_TIM_SET_COMPARE(&htim3, Y_AXIS, 0);
					flag_return_home_y = false;
					current_pulse_y = 0;
					count_wait_vaccum_object = 0;
					flag_start_timer_y = false;
					uart_command[index_uart_command] = robot_move_command;
					index_uart_command++;
					index_current_command++;
					flag_receive_move_robot = false;
					flag_finish_task = true;
					printf("Grab object successfully\n");
				}
			}
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SIG_STOP_CONVEYOR_Pin && flag_capture_object == false
			&& flag_receive_move_robot == true) {
		HAL_Delay(300);
		uart_command[index_uart_command] = grab_object;
		//printf("uart_command: %s\n", uart_command[index_uart_command]);
		index_uart_command++;
		//printf("This is interrupt\n");
		HAL_Delay(600);
		HAL_GPIO_WritePin(STATE_CONVEYOR_GPIO_Port, STATE_CONVEYOR_Pin,
				GPIO_PIN_RESET);
		flag_capture_object = true;
		EXTI->PR = SIG_STOP_CONVEYOR_Pin;
		__HAL_GPIO_EXTI_CLEAR_IT(SIG_STOP_CONVEYOR_Pin);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length_string);
		printf("Receive %s\n", rxByte);
		if (!strncmp(rxByte, position2, length_string)
				|| !strncmp(rxByte, position3, length_string)) {
			robot_move_command = rxByte;
			flag_receive_move_robot = true;
			return;
		}
		uart_command[index_uart_command] = rxByte;
		index_uart_command++;
	}
}
void handle_command() {
	if (!strncmp(uart_command[index_current_command], stop,
	length_string)) {
		printf("Robot doing: Stop conveyor\n");
		HAL_GPIO_WritePin(STATE_CONVEYOR_GPIO_Port, STATE_CONVEYOR_Pin,
				GPIO_PIN_RESET);
		flag_finish_task = true;
		index_current_command++;
		return;
	} else if (!strncmp(uart_command[index_current_command], grab_object,
	length_string)) {
		printf("Robot doing: Grab object\n");
		flag_at_position_y = false;
		flag_return_home_y = false;
		flag_start_timer_y = true;
		HAL_GPIO_WritePin(STATE_CONVEYOR_GPIO_Port, STATE_CONVEYOR_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GRAB_OBJECT_GPIO_Port, GRAB_OBJECT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIRY_GPIO_Port, DIRY_Pin, CW);
		flag_finish_task = false;
		HAL_TIM_Base_Start_IT(&htim3);
		HAL_TIM_PWM_Start(&htim3, Y_AXIS);
		__HAL_TIM_SET_COMPARE(&htim3, Y_AXIS, 90);
	} else if (!strncmp(uart_command[index_current_command], start,
	length_string)) {
		printf("Robot doing: Start conveyor\n");
		HAL_GPIO_WritePin(STATE_CONVEYOR_GPIO_Port, STATE_CONVEYOR_Pin,
				GPIO_PIN_SET);
		flag_finish_task = true;
		index_current_command++;
		return;
	} else if (!strncmp(uart_command[index_current_command], position2,
	length_string)) {
		printf("Move position 2\n");
		index_array = MODE_POSITION_1;
		HAL_GPIO_WritePin(DIRX_GPIO_Port, DIRX_Pin, CCW);
		flag_at_position_x = false;
		flag_return_home_x = false;
		flag_start_timer_x = true;
		flag_finish_task = false;
		time_to_wait_axis_x = time_waiting_drop[MODE_POSITION_1];
		time_to_wait_axis_y = time_waiting_vaccum[MODE_POSITION_1];
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_PWM_Start(&htim2, X_AXIS);
		__HAL_TIM_SET_COMPARE(&htim2, X_AXIS, 40);
	} else if (!strncmp(uart_command[index_current_command], position3,
	length_string)) {
		printf("Move position 3\n");
		index_array = MODE_POSITION_3;
		HAL_GPIO_WritePin(DIRX_GPIO_Port, DIRX_Pin, CCW);
		flag_at_position_x = false;
		flag_return_home_x = false;
		flag_start_timer_x = true;
		flag_finish_task = false;
		time_to_wait_axis_x = time_waiting_drop[MODE_POSITION_2];
		time_to_wait_axis_y = time_waiting_vaccum[MODE_POSITION_2];
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_PWM_Start(&htim2, X_AXIS);
		__HAL_TIM_SET_COMPARE(&htim2, X_AXIS, 40);
	}

}
void control_robot() {
	if (flag_finish_task) {
		handle_command();
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
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length_string);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		control_robot();
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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
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
	htim2.Init.Prescaler = 80 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 50 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigOC.Pulse = 20 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
	htim3.Init.Prescaler = 80 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 200 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	huart1.Init.BaudRate = 115200;
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GRAB_OBJECT_Pin | STATE_CONVEYOR_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DIRY_Pin | DIRX_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : GRAB_OBJECT_Pin */
	GPIO_InitStruct.Pin = GRAB_OBJECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GRAB_OBJECT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIRY_Pin DIRX_Pin */
	GPIO_InitStruct.Pin = DIRY_Pin | DIRX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : STATE_CONVEYOR_Pin */
	GPIO_InitStruct.Pin = STATE_CONVEYOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(STATE_CONVEYOR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SIG_STOP_CONVEYOR_Pin */
	GPIO_InitStruct.Pin = SIG_STOP_CONVEYOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SIG_STOP_CONVEYOR_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
