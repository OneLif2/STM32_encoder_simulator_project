/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdbool.h"

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
int direction_mode = 0;

static bool encoder_state = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static bool prev_val;
	currentMillis = HAL_GetTick();
	if (GPIO_Pin == B1_Pin && (currentMillis - previousMillis > 100)) {
		if (prev_val == false) {
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 1);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, 1);
			delay(410000);
			//delay 500000us = 0.5s
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 0);
			encoder_state = !encoder_state;
			prev_val = true;
		} else {
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 1);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, 0);
			delay(410000);
			//delay 500000us = 0.5s
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 0);
			encoder_state = !encoder_state;
			prev_val = false;
			direction_mode++;
			if (direction_mode > 1) {
				direction_mode = 0;
			}
		}
		previousMillis = currentMillis;
	}
}

void delay(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER(&htim3) < us)
		;
}

#define stepsperrev 1440

bool gled_state = 1; // g led state
bool bled_state = 0; // b led state
uint8_t button_val = 0;

void stepper_set_rpm(int rpm) // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000 / stepsperrev / rpm);
}

void stepper_half_drive(int step) {
	switch (step) {
	case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);   // IN2
		break;

	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);   // IN1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // IN2

		break;

	case 2:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // IN2

		break;

	case 3:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);   // IN2

		break;

	}
}

void stepper_step_angle(float angle, int direction, int rpm) {
	float anglepersequence = 1;  // 360 = 360 sequences
	int numberofsequences = (int) (angle / anglepersequence);

	for (int seq = 0; seq < numberofsequences; seq++) {
		if (direction == 0)  // for clockwise
				{
			for (int step = 3; step >= 0; step--) {
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
				{
			for (int step = 0; step < 4; step++) {
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}
	}
}

/*
 if (encoder_state == 1) {
 stepper_step_angle(90, direction_mode, 180);
 } else if (encoder_state == 0) {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);   // IN2
 }
 */

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
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */

		if (encoder_state == 1) {
			stepper_step_angle(90, direction_mode, 30);
		} else if (encoder_state == 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);   // IN2
			HAL_Delay(500);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
