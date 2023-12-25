/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X_CHANNEL TIM_CHANNEL_1
#define Y_CHANNEL TIM_CHANNEL_1
#define Z_CHANNEL TIM_CHANNEL_3
#define TURN_CHANNEL TIM_CHANNEL_2
#define PULSE_ONE_ROUND 3200
#define length 26
#define MAX_UART_VALUE 100000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int homeX = 0, homeY = 0, homeZ = 0, startUart = 0, pulX = 0, pulY = 0,
		pulZ = 0, count_dot = 0, doneX = 0, doneY = 0, doneZ = 0, pulT = 0,
		doneT = 0, startXYZ = 0, startCmd = 0, running_TIM2 = 0, running_TIM3 =
				0;
int mode, dir_changeX = 0, dir_changeY = 0, dir_changeZ = 0;
uint32_t valueX = MAX_UART_VALUE, valueY = MAX_UART_VALUE, valueZ =
MAX_UART_VALUE, X = 0, Y = 0, Z = 0, count_timer = 0;
char rxByte[length];
char Home[30] = "Home----------------------";
char Turn_Around[30] = "Turn Around---------------";
char Stop[30] = "Stop----------------------";
char Stop_Conveyor[30] = "Stop conveyor-------------";
char Start_Conveyor[30] = "Start conveyor------------";
char Start_Pick_Up[30] = "Start pick up-------------";
char Stop_Pick_Up[30] = "Stop pick up--------------";
typedef enum {

	ClockWise = 0,
	CounterClockWise = 1,
	On = 0,
	Off = 1,
	Home_State = 1,
	Turn_Around_State = 2,
	Stop_State = 3,
	Stop_Conveyor_State = 4,
	Start_Conveyor_State = 5,
	Start_Pick_Up_State = 6,
	Stop_Pick_Up_State = 7
} State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
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
	if (htim->Instance == TIM2 && startUart == 1) {
		if (startXYZ) {
			pulX = doneX == 0 ? pulX + 1 : pulX;
			pulZ = doneZ == 0 ? pulZ + 1 : pulZ;
			running_TIM2 = 1;

			if (pulX >= X && doneX == 0) {
				HAL_TIM_PWM_Stop_IT(&htim2, X_CHANNEL);
				count_timer++;
				doneX = 1;
			}
			if (pulZ >= Z && doneZ == 0) {
				HAL_TIM_PWM_Stop_IT(&htim2, Z_CHANNEL);
				doneZ = 1;
				count_timer++;
			}
		}
		if (startCmd) {
			running_TIM2 = 1;
			pulT = doneT == 0 ? pulT + 1 : pulT;
			if (pulT >= PULSE_ONE_ROUND && doneT == 0) {
				HAL_TIM_PWM_Stop_IT(&htim2, TURN_CHANNEL);
				doneT = 1;
				count_timer++;
				//printf("Done turn around\r\n");
			}
		}

	}
	if (htim->Instance == TIM3 && startUart == 1) {
		if (startXYZ) {
			pulY = doneY == 0 ? pulY + 1 : pulY;
			running_TIM3 = 1;
			if (pulY >= Y && doneY == 0) {
				HAL_TIM_PWM_Stop_IT(&htim3, Y_CHANNEL);
				doneY = 1;
				count_timer++;

			}
		}
	}
	if ((count_timer == 3 && startUart == 1)
			|| (count_timer == 1 && startCmd == 1)) {
		startXYZ = 0;
		pulX = 0;
		pulY = 0;
		pulZ = 0;
		pulT = 0;
		count_timer = 0;
		startCmd = 0;
		startUart = 0;
		running_TIM2 = 0;
		running_TIM3 = 0;
	}
}
void Uart_Receive_Init() {
	X = 0;
	Y = 0;
	Z = 0;
	valueX = MAX_UART_VALUE;
	valueY = MAX_UART_VALUE;
	valueZ = MAX_UART_VALUE;
	count_dot = 0;
	mode = 0;
	dir_changeZ = 0;
	dir_changeX = 0;
	dir_changeY = 0;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (running_TIM2 == 0 && running_TIM3 == 0) {
			Uart_Receive_Init(); // khoi tao lai cac gia tri cua UART
			HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length); // truyen du? bit
			if (!strncmp(rxByte, Home,length)) {
				mode = Home_State;
			} else if (!strncmp(rxByte, Stop,length)) {
				mode = Stop_State;
			} else if (!strncmp(rxByte, Turn_Around,length)) {
				mode = Turn_Around_State;
			} else if (!strncmp(rxByte, Stop_Conveyor,length)) {
				mode = Stop_Conveyor_State;
			} else if (!strncmp(rxByte, Start_Conveyor,length)) {
				mode = Start_Conveyor_State;
			} else if (!strncmp(rxByte, Start_Pick_Up,length)) {
				mode = Start_Pick_Up_State;
			} else if (!strncmp(rxByte, Stop_Pick_Up,length)) {
				mode = Stop_Pick_Up_State;
			}
			switch (mode) {
			case Home_State: // cai nay cao nhi
				HAL_GPIO_WritePin(GPIOB, DIRX_Pin | DIRY_Pin | DIRZ_Pin,
						CounterClockWise);
				HAL_TIM_PWM_Start_IT(&htim2, X_CHANNEL);
				HAL_TIM_PWM_Start_IT(&htim3, Y_CHANNEL);
				HAL_TIM_PWM_Start_IT(&htim2, Z_CHANNEL);
				doneX = 1;
				doneY = 1;
				doneZ = 1;
				printf("Mode: Home\r\n");
				break;
			case Stop_State: // thay doi do uu tien boi vi cai nay cao nhat
				HAL_TIM_PWM_Stop_IT(&htim2, X_CHANNEL);
				HAL_TIM_PWM_Stop_IT(&htim3, Y_CHANNEL);
				HAL_TIM_PWM_Stop_IT(&htim2, Z_CHANNEL);
				HAL_GPIO_WritePin(GPIOB, RELAY_CONVEYOR_Pin, Off);
				printf("Mode: Stop\r\n");
				break;
			case Turn_Around_State:
				HAL_GPIO_WritePin(GPIOB, RELAY_CONVEYOR_Pin, On);
				HAL_TIM_PWM_Stop_IT(&htim2, X_CHANNEL);
				HAL_TIM_PWM_Stop_IT(&htim3, Y_CHANNEL);
				HAL_TIM_PWM_Stop_IT(&htim2, Z_CHANNEL);
				HAL_TIM_PWM_Start_IT(&htim2, TURN_CHANNEL);
				printf("Mode: Turn Around\r\n");
				doneT=0;
				break;
			case Stop_Conveyor_State:
				HAL_GPIO_WritePin(GPIOB, RELAY_CONVEYOR_Pin, Off);
				printf("Mode: Stop Conveyor\r\n");
				break;
			case Start_Conveyor_State:
				HAL_GPIO_WritePin(GPIOB, RELAY_CONVEYOR_Pin, On);
				printf("Mode: Start Conveyor\r\n");
				break;
			case Start_Pick_Up_State:
				HAL_GPIO_WritePin(GPIOA, RELAY_ROBOT_Pin, On);
				printf("Mode: Start pick up \r\n");
				break;
			case Stop_Pick_Up_State:
				HAL_GPIO_WritePin(GPIOA, RELAY_ROBOT_Pin, Off);
				printf("Mode: Stop pick up \r\n");
				break;
			default:
				mode = 0;
				break;
			}
			if (mode == 0) {
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
								HAL_GPIO_WritePin(GPIOB, DIRX_Pin,
										CounterClockWise);
								dir_changeX = 1;
								valueX /= 10;
								continue;
							}
							if (!dir_changeX) {
								HAL_GPIO_WritePin(GPIOB, DIRX_Pin, ClockWise);

							}
							X += ((int) rxByte[i] - '0') * valueX;
							valueX /= 10;
							break;
						case 1:
							if (rxByte[i] == '-') {
								HAL_GPIO_WritePin(GPIOA, DIRY_Pin,
										CounterClockWise);
								dir_changeY = 1;
								valueY /= 10;
								continue;
							}
							if (!dir_changeY) {
								HAL_GPIO_WritePin(GPIOA, DIRY_Pin, ClockWise);
							}
							Y += ((int) rxByte[i] - '0') * valueY;
							valueY /= 10;
							break;
						case 2:
							if (rxByte[i] == '-') {
								HAL_GPIO_WritePin(GPIOB, DIRZ_Pin,
										CounterClockWise);
								valueZ /= 10;
								dir_changeZ = 1;
								continue;
							}
							if (!dir_changeZ) {
								HAL_GPIO_WritePin(GPIOB, DIRZ_Pin, ClockWise);
							}
							Z += ((int) rxByte[i] - '0') * valueZ;
							valueZ /= 10;
							break;

						}
					}
				}
				startXYZ = 1;
				doneX = 0;
				doneY = 0;
				doneZ = 0;
				if (X) {
					HAL_TIM_PWM_Start_IT(&htim2, X_CHANNEL);
				} else {
					doneX = 1;
					count_timer++;
				}
				if (Y) {
					HAL_TIM_PWM_Start_IT(&htim3, Y_CHANNEL);
				} else {
					doneY = 1;
					count_timer++;
				}
				if (Z) {
					HAL_TIM_PWM_Start_IT(&htim2, Z_CHANNEL);
				} else {
					doneZ = 1;
					count_timer++;
				}

			} else if (mode != 0) {
				startCmd = 1;
			}
			startUart = 1;
		} else {
			printf("Running.... Dont receive value\r\n");
			HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length);
		}

	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case SIGX_Pin:
		HAL_TIM_PWM_Stop_IT(&htim2, X_CHANNEL);
		homeX = 1;
		EXTI->PR = SIGX_Pin;
		__HAL_GPIO_EXTI_CLEAR_IT(SIGX_Pin);
		break;
	case SIGY_Pin:
		HAL_TIM_PWM_Stop_IT(&htim3, Y_CHANNEL);
		homeY = 1;
		EXTI->PR = SIGY_Pin;
		__HAL_GPIO_EXTI_CLEAR_IT(SIGY_Pin);
		break;
	case SIGZ_Pin:
		HAL_TIM_PWM_Stop_IT(&htim2, Z_CHANNEL);
		homeZ = 1;
		EXTI->PR = SIGZ_Pin;
		__HAL_GPIO_EXTI_CLEAR_IT(SIGZ_Pin);
		break;
	case EMSTOP_Pin:
		HAL_TIM_PWM_Stop_IT(&htim2, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim3, Y_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim2, Z_CHANNEL);
		EXTI->PR = SIGZ_Pin;
		__HAL_GPIO_EXTI_CLEAR_IT(SIGZ_Pin);
		break;
	default:
		break;
	}
}
void ResetAllValue() {
	homeY = 0;
	homeX = 0;
	homeZ = 0;
	pulX = 0;
	pulY = 0;
	pulZ = 0;
	doneX = 0;
	doneY = 0;
	doneZ = 0;
}
void CheckHome(void) {
	if (homeY == 1 && homeX == 1 && homeZ == 1) {
		printf("At home\r\n");
		ResetAllValue();
		HAL_GPIO_WritePin(GPIOB, RELAY_CONVEYOR_Pin, On); // bat bang chuyen
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  /* USER CODE BEGIN 2 */
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_UART_Receive_IT(&huart1, (uint8_t*) rxByte, length);
//	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

//	HAL_TIM_PWM_Start_IT(&htim2, Z_CHANNEL);
//HAL_UART_Transmit(&huart1, &c, 1, 10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		CheckHome();
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 125-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 160-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 125-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIRY_GPIO_Port, DIRY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_ROBOT_GPIO_Port, RELAY_ROBOT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIRX_Pin|DIRZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_ABSORB_Pin|RELAY_CONVEYOR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : EMSTOP_Pin */
  GPIO_InitStruct.Pin = EMSTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EMSTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIRY_Pin */
  GPIO_InitStruct.Pin = DIRY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIRY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_ROBOT_Pin */
  GPIO_InitStruct.Pin = RELAY_ROBOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_ROBOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIRX_Pin DIRZ_Pin */
  GPIO_InitStruct.Pin = DIRX_Pin|DIRZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SIGY_Pin SIGX_Pin SIGZ_Pin */
  GPIO_InitStruct.Pin = SIGY_Pin|SIGX_Pin|SIGZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_ABSORB_Pin RELAY_CONVEYOR_Pin */
  GPIO_InitStruct.Pin = RELAY_ABSORB_Pin|RELAY_CONVEYOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
void Error_Handler(void)
{
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
