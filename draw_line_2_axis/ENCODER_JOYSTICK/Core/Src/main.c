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
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Position {
	union {
		uint32_t step;
		_Bool direction;
	} axisX;
	union {
		uint32_t step;
		_Bool direction;
	} axisY;
} Pos;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _OPEN_SYS_ITOA_EXT
#define Y_CHANNEL TIM_CHANNEL_3
#define X_CHANNEL TIM_CHANNEL_1
#define mmX 30
#define mmY 20
#define MAX_ARR 39
#define FASTER_ARR 19
#define length 13
#define one_roundX 3996
#define one_roundY 3996
#define MIN_VALUE 39
#define MAX_VALUE 9999
#define MAX_TIMER_VALUE 65535
#define MIN_DOUBLE 0.01
#define MAX_DOUBLE 1
#define VALUE_AT_HOME 50
#define DIRECTION_PORT GPIOC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
enum {
	CounterClockWise = 1,
	ClockWise = 0,
	moveRobotState = 3,
	returnHomeState = 4,
	runAfterSaveState = 5,
	bluetoothControl = 6
} state;
_Bool true = 1, false = 0;
uint32_t Axis[2], drawX[2], drawY[2];
int timesOverflowX = 0, timesOverflowY = 0;
Pos pos[100];
uint32_t calcX, calcY, currentPulseX = 0, currentPulseY = 0, compensateX = 0,
		compensateY = 0;
uint32_t clkX, clkY, stepX = 0, stepY = 0;
int indexSaving = 0, indexRunAfterSave = 0;
int value_uart = 0, stateOfRobot = moveRobotState;
_Bool isRunning = 0, StateDirX = 0, StateDirY = 0, isAtHomeX = 0, isAtHomeY = 0,
		isAtPositionX = 1, isAtPositionY = 1, isFirstLoop = 0, isReturnHome = 0;
double ratio;
uint32_t distanceX = 0, distanceY = 0;
char value_encoder_Ox[5] = { 0 };
char value_encoder_Oy[5] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case SAVE_POS_Pin:
		HAL_Delay(200);

		pos[indexSaving].axisX.step = __HAL_TIM_GET_COUNTER(&htim4)
				+ MAX_TIMER_VALUE * timesOverflowX;
		pos[indexSaving].axisY.step = __HAL_TIM_GET_COUNTER(&htim2)
				+ MAX_TIMER_VALUE * timesOverflowY;
		pos[indexSaving].axisX.direction = HAL_GPIO_ReadPin(DIRECTION_PORT,
		DIRX_Pin);
		pos[indexSaving].axisY.direction = HAL_GPIO_ReadPin(DIRECTION_PORT,
		DIRY_Pin);
		indexSaving++;
		HAL_Delay(200);
		__HAL_GPIO_EXTI_CLEAR_IT(SAVE_POS_Pin);
		EXTI->PR = SAVE_POS_Pin;
		break;
	case RUN_AFTER_SAVE_Pin:
		HAL_Delay(100);
		isRunning = true;
		stateOfRobot = runAfterSaveState;
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		clkX = __HAL_TIM_GET_COUNTER(&htim4);
		clkY = __HAL_TIM_GET_COUNTER(&htim2);
		timesOverflowX = 0;
		timesOverflowY = 0;
		isAtPositionX = true;
		isAtPositionY = true;
		HAL_Delay(100);
		EXTI->PR = RUN_AFTER_SAVE_Pin;
		__HAL_GPIO_EXTI_CLEAR_IT(RUN_AFTER_SAVE_Pin);
		break;
	case RETURN_HOME_Pin:
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(100);
		stateOfRobot = returnHomeState;
		isReturnHome = 1;
		__HAL_TIM_SET_AUTORELOAD(&htim3, FASTER_ARR);
		__HAL_TIM_SET_AUTORELOAD(&htim5, FASTER_ARR);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, 0);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, 0);
		HAL_Delay(10);
		if (!(clkX + MAX_TIMER_VALUE * timesOverflowX)) {
			HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
		} else {
			isAtHomeX = true;
		}
		if (!(clkY + MAX_TIMER_VALUE * timesOverflowY)) {
			HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		} else {
			isAtHomeY = true;
		}
		EXTI->PR = RETURN_HOME_Pin;
		HAL_Delay(100);
		__HAL_GPIO_EXTI_CLEAR_IT(RETURN_HOME_Pin);
		break;
	default:
		break;
	}

}
void IncreaseIndexReference(void) {
	if (isAtPositionX && isAtPositionY && !isReturnHome) {
		indexRunAfterSave++;
	}
}
void drawOLED(void) {
	drawY[0] = (uint32_t) clkY / one_roundY + timesOverflowY * 16;
	drawX[0] = (uint32_t) clkX / one_roundX + timesOverflowX * 16;
	if (drawX[0] < drawX[1] || drawY[0] < drawY[1]) {
		SSD1306_Clear();
		SSD1306_UpdateScreen();
	}
	itoa(drawY[0], value_encoder_Oy, 10);
	itoa(drawX[0], value_encoder_Ox, 10); // decimal
	SSD1306_GotoXY(10, 12); // goto 10, 10
	SSD1306_Puts("POSITION", &Font_11x18, 1); // print POSITION
	SSD1306_GotoXY(10, 40);
	SSD1306_Puts("(", &Font_7x10, 1);
	SSD1306_GotoXY(20, 40);
	SSD1306_Puts(value_encoder_Ox, &Font_7x10, 1);
	SSD1306_GotoXY(60, 40);
	SSD1306_Puts(",", &Font_7x10, 1);
	SSD1306_GotoXY(70, 40);
	SSD1306_Puts(value_encoder_Oy, &Font_7x10, 1);
	SSD1306_GotoXY(105, 40);
	SSD1306_Puts(")", &Font_7x10, 1);
	SSD1306_UpdateScreen(); // update screen
	memset(value_encoder_Ox, 0, sizeof(value_encoder_Ox));
	memset(value_encoder_Oy, 0, sizeof(value_encoder_Oy));
	drawX[1] = drawX[0];
	drawY[1] = drawY[0];
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		if (StateDirX == ClockWise && timesOverflowX != 0) {
			timesOverflowX--;
		} else if (StateDirX == CounterClockWise) {
			timesOverflowX++;
		}
	}
	if (htim->Instance == TIM2) {
		if (StateDirY == ClockWise && timesOverflowY != 0) {
			timesOverflowY--;
		} else if (StateDirY == CounterClockWise) {
			timesOverflowY++;
		}
	}
}

double RoundingDouble() {
	double round_value_max = MAX_DOUBLE - ratio;
	if (round_value_max < 0.01) {
		ratio = MAX_DOUBLE;
	}
	return ratio;
}
void SettingRobot(int index) {
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, pos[index].axisX.direction);
	HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, pos[index].axisY.direction);
	if (distanceX > distanceY && distanceX != 0 && distanceY != 0) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, MAX_ARR);
		ratio = (double) distanceY / distanceX;
		ratio = RoundingDouble();
		calcX = MAX_ARR / ratio;
		__HAL_TIM_SET_AUTORELOAD(&htim5, (uint32_t )calcX);
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
		isAtPositionX = false;
		isAtPositionY = false;
	} else if (distanceX < distanceY && distanceX != 0 && distanceY != 0) {
		__HAL_TIM_SET_AUTORELOAD(&htim5, MAX_ARR);
		ratio = (double) distanceX / distanceY;
		ratio = RoundingDouble();
		calcY = MAX_ARR / ratio;
		__HAL_TIM_SET_AUTORELOAD(&htim3, (uint32_t )calcY);
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
		isAtPositionX = false;
		isAtPositionY = false;
	} else if (distanceX == distanceY) {
		__HAL_TIM_SET_AUTORELOAD(&htim5, MAX_ARR);
		__HAL_TIM_SET_AUTORELOAD(&htim3, MAX_ARR);
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
		isAtPositionX = false;
		isAtPositionY = false;
	} else if (!distanceX && distanceY) {
		__HAL_TIM_SET_AUTORELOAD(&htim5, FASTER_ARR);
		isAtPositionX = true;
		isAtPositionY = false;
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
	} else if (distanceX && !distanceY) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, FASTER_ARR);
		isAtPositionY = true;
		isAtPositionX = false;
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
	} else if (!distanceX && !distanceY) {
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		isAtPositionY = true;
		isAtPositionX = true;
	}

}

void RunAfterSave(void) {
	if (isAtPositionX && isAtPositionY && !isReturnHome) {
		if (!pos[indexRunAfterSave].axisX.step
				&& !pos[indexRunAfterSave].axisY.step && indexRunAfterSave
				&& indexSaving == indexRunAfterSave) {
			indexRunAfterSave = 0;
			isFirstLoop = false;
		}
		if (indexRunAfterSave) {
			stepX = pos[indexRunAfterSave].axisX.step;
			stepY = pos[indexRunAfterSave].axisY.step;
			distanceX = abs(stepX - pos[indexRunAfterSave - 1].axisX.step);

			distanceY = abs(stepY - pos[indexRunAfterSave - 1].axisY.step);
			distanceX =
					(abs(distanceX - distanceY)) < 100 ? distanceY : distanceX;
			SettingRobot(indexRunAfterSave);
		} else if (!indexRunAfterSave && isFirstLoop) {
			stepX = pos[indexRunAfterSave].axisX.step;
			stepY = pos[indexRunAfterSave].axisY.step;
			SettingRobot(indexRunAfterSave);
		} else if (!indexRunAfterSave && !isFirstLoop) {
			stepX = pos[indexRunAfterSave].axisX.step;
			distanceX = abs(stepX - pos[indexSaving - 1].axisX.step);
			stepY = pos[indexRunAfterSave].axisY.step;
			distanceY = abs(stepY - pos[indexSaving - 1].axisY.step);
			distanceX =
					(abs(distanceX - distanceY)) < 100 ? distanceY : distanceX;
			SettingRobot(indexSaving - 1);
		}

	}
}
_Bool isAtSavePositionX() {
	uint32_t currentPositionX = __HAL_TIM_GET_COUNTER(&htim4);
	if (currentPositionX != pos[indexRunAfterSave].axisX.step) {
		return false;
	}
	return true;
}
_Bool isAtSavePositionY() {
	uint32_t currentPositionY = __HAL_TIM_GET_COUNTER(&htim2);
	if (currentPositionY != pos[indexRunAfterSave].axisY.step) {
		return false;
	}
	return true;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (!isAtPositionX) {
		clkX = __HAL_TIM_GET_COUNTER(&htim4);
		StateDirX = HAL_GPIO_ReadPin(DIRECTION_PORT, DIRX_Pin);
	}
	if (!isAtPositionY) {
		clkY = __HAL_TIM_GET_COUNTER(&htim2);
		StateDirY = HAL_GPIO_ReadPin(DIRECTION_PORT, DIRY_Pin);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) //PWM xog 1 xung
{
	switch (stateOfRobot) {
	case runAfterSaveState:
		if (htim->Instance == TIM3) {
			if (!isAtPositionX) {
				currentPulseX++;
			}
			compensateX = currentPulseX - clkX;
			if (clkX + MAX_TIMER_VALUE * timesOverflowX >= stepX + compensateX
					&& StateDirX == CounterClockWise) {
				HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
				if (isAtSavePositionX()) {
					printf("\n X at pos %d", indexRunAfterSave + 1);
					__HAL_TIM_SET_AUTORELOAD(&htim3, MAX_ARR);
					isAtPositionX = true;
					IncreaseIndexReference();
				} else {
					HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin,
							!pos[indexRunAfterSave].axisX.direction);
					HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
				}

			} else if (clkX + MAX_TIMER_VALUE * timesOverflowX
					<= stepX + compenstateX && StateDirX == ClockWise) {
				HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
				if (isAtSavePositionX()) {
					printf("\n X at pos %d", indexRunAfterSave + 1);
					__HAL_TIM_SET_AUTORELOAD(&htim3, MAX_ARR);
					isAtPositionX = true;
					IncreaseIndexReference();
				} else {
					HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin,
							!pos[indexRunAfterSave].axisX.direction);
					HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
				}

			}
		}
		if (htim->Instance == TIM5) {
			if (!isAtPositionY) {
				currentPulseY++;
			}
			compensateY = currentPulseY - clkY;
			if (clkY + MAX_TIMER_VALUE * timesOverflowY >= stepY + compensateY
					&& StateDirY == CounterClockWise) {
				HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
				if (isAtSavePositionY()) {
					printf("\n Y at pos %d", indexRunAfterSave + 1);
					__HAL_TIM_SET_AUTORELOAD(&htim5, MAX_ARR);
					isAtPositionY = true;
					IncreaseIndexReference();
				} else {
					HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin,
							!pos[indexRunAfterSave].axisY.direction);
					HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
				}

			} else if (clkY + MAX_TIMER_VALUE * timesOverflowY
					<= stepY + compensateY && StateDirY == ClockWise) {
				HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
				if (isAtSavePositionY()) {
					printf("\n Y at pos %d", indexRunAfterSave + 1);
					__HAL_TIM_SET_AUTORELOAD(&htim5, MAX_ARR);
					isAtPositionY = true;
					IncreaseIndexReference();
				} else {
					HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin,
							!pos[indexRunAfterSave].axisY.direction);
					HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
				}
			}
		}
		break;
	case returnHomeState:
		if (htim->Instance == TIM4) {
			clkX = __HAL_TIM_GET_COUNTER(&htim4);
			if (isReturnHome) {
				if (clkX + MAX_TIMER_VALUE * timesOverflowX <= VALUE_AT_HOME) {
					HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
					isAtHomeX = true;
				}
			}

		}
		if (htim->Instance == TIM2) {
			clkY = __HAL_TIM_GET_COUNTER(&htim2);
			if (isReturnHome) {
				if (clkY + MAX_TIMER_VALUE * timesOverflowY <= VALUE_AT_HOME) {
					HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
					isAtHomeY = true;
				}
			}
		}
		break;
	default:
		break;
	}

}
void Clear_Pos(void) {
	for (int i = 0; i < indexSaving; i++) {
		pos[i].axisX.step = 0;
		pos[i].axisY.step = 0;
		pos[i].axisX.direction = 0;
		pos[i].axisY.direction = 0;
	}
	indexSaving = 0;
	indexRunAfterSave = 0;
}

void MoveRobot(void) {
	if ((Axis[1] >= 1000 && Axis[1] <= 3000) && Axis[0] <= 500) {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, ClockWise);
		// di qua phai
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);

	} else if (Axis[0] >= 3800 && (Axis[1] <= 3000 && Axis[1] >= 1000)) {
		// di trai
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, CounterClockWise);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
	} else if ((Axis[0] <= 3000 && Axis[0] >= 1000) && Axis[1] <= 500) {
		// di xuong
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, ClockWise);
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
	} else if (Axis[1] >= 3800 && (Axis[0] <= 3000 && Axis[0] >= 1000)) {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, CounterClockWise); // 1 la di len
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
	} else if (Axis[1] >= 4000 && Axis[0] >= 4000) {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, CounterClockWise); // 1 la di len
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, CounterClockWise); // 1 la di len
		HAL_Delay(10);
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);

	} else if (Axis[0] >= 4000 && Axis[1] <= 500) {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, ClockWise); // 1 la di len
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, CounterClockWise); // 1 la di len
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
	} else if (Axis[1] <= 500 && Axis[0] <= 500) {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, ClockWise); // 1 la di len
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, ClockWise); // 1 la di len
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
	} else if (Axis[1] >= 4000 && Axis[0] <= 500) {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
		HAL_Delay(10);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, CounterClockWise); // 1 la di len
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, ClockWise); // 1 la di len
		HAL_TIM_PWM_Start_IT(&htim5, Y_CHANNEL);
		HAL_TIM_PWM_Start_IT(&htim3, X_CHANNEL);
	} else {
		HAL_TIM_PWM_Stop_IT(&htim3, X_CHANNEL);
		HAL_TIM_PWM_Stop_IT(&htim5, Y_CHANNEL);
	}

}

void CheckHome(void) {
	if (isAtHomeX && isAtHomeY && isReturnHome) {
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRX_Pin, CounterClockWise);
		HAL_GPIO_WritePin(DIRECTION_PORT, DIRY_Pin, CounterClockWise);
		isAtHomeX = 0;
		isAtHomeY = 0;
		isReturnHome = 0;
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		clkX = __HAL_TIM_GET_COUNTER(&htim4);
		clkY = __HAL_TIM_GET_COUNTER(&htim2);
		timesOverflowX = 0;
		timesOverflowY = 0;
		if (isRunning == 1) {
			Clear_Pos();
			isRunning = 0;
		}
	}

}

void CheckDirectionState(void) {
	StateDirX = HAL_GPIO_ReadPin(DIRECTION_PORT, DIRX_Pin);
	StateDirY = HAL_GPIO_ReadPin(DIRECTION_PORT, DIRY_Pin);
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_I2C1_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */

	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	SSD1306_Init(); // initialize the display
	stateOfRobot = moveRobotState;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		switch (stateOfRobot) {
		case moveRobotState:
			HAL_ADC_Start_DMA(&hadc1, Axis, 2);
			MoveRobot();
			break;
		case runAfterSaveState:
			HAL_ADC_Stop_DMA(&hadc1);
			RunAfterSave();
			break;
		case returnHomeState:
			HAL_ADC_Stop_DMA(&hadc1);
			CheckHome();
			break;
		default:
			drawOLED();
			CheckDirectionState();
		}

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	htim3.Init.Period = 40 - 1;
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
	sConfigOC.Pulse = 15;
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 720 - 1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 40 - 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 15;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, DIRX_Pin | DIRY_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DIRX_Pin DIRY_Pin */
	GPIO_InitStruct.Pin = DIRX_Pin | DIRY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : RETURN_HOME_Pin */
	GPIO_InitStruct.Pin = RETURN_HOME_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(RETURN_HOME_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SAVE_POS_Pin RUN_AFTER_SAVE_Pin */
	GPIO_InitStruct.Pin = SAVE_POS_Pin | RUN_AFTER_SAVE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
