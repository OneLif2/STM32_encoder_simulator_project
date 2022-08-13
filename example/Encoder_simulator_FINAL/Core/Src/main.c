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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "string.h"
#include <stdbool.h> // for activate bool type
#include <stdio.h>

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
int32_t delaystart = 0;
int32_t delayMillis = 0;
int direction_mode = 1;
int polestep = 0;
int desStep = 0;

static bool encoder_state = 0;

#define rxbuff_size 10 // 10 bytes
char txbuffer[50] = { 0 };
#define mainbuff_size 20
uint8_t rxdatasize;

uint8_t rxbuff[rxbuff_size]; // where DMA is going to copy data
uint8_t blankdata[mainbuff_size]; // remove previous data
uint8_t mainbuff[mainbuff_size]; // Data will be finally store here
_Bool uart_rx_int = 0;

void delay(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

#define stepsperrev 1440

bool gled_state = 1; // g led state
bool bled_state = 0; // b led state
uint8_t button_val = 0;

uint32_t counter = 0;
_Bool pause = 0;
int estep = 0;

void stepper_set_rpm(int rpm) // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000 / stepsperrev / rpm); //delay(input us)

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
			for (int step1 = 3; step1 >= 0; step1--) {
				polestep--;
				estep--;
				if (estep < 0) {
					estep = 3;
				}
				stepper_half_drive(estep);
				stepper_set_rpm(rpm);
			}
		}

		else if (direction == 1)  // for anti-clockwise
				{
			for (int step1 = 0; step1 < 4; step1++) {
				polestep++;
				estep++;
				if (estep > 3) {
					estep = 0;
				}
				stepper_half_drive(estep);
				stepper_set_rpm(rpm);
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static bool prev_val;
	currentMillis = HAL_GetTick();
	if (GPIO_Pin == B1_Pin && (currentMillis - previousMillis > 100)) {
		if (prev_val == false) {
			delaystart = HAL_GetTick();
			delayMillis = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 1);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
			HAL_Delay(500);
			//delay 500000us = 0.5s
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
			encoder_state = !encoder_state;
			prev_val = true;
		} else {
			delaystart = HAL_GetTick();
			delayMillis = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 1);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
			HAL_Delay(500);
			//delay 500000us = 0.5s
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
			encoder_state = !encoder_state;
			prev_val = false;
			direction_mode++;
			polestep = 0;
			if (direction_mode > 1) {
				direction_mode = 0;
			}
		}
		previousMillis = currentMillis;
	}
}

// Code for DMA data Rx and Tx
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {
		memcpy(mainbuff, blankdata, mainbuff_size);
		//remove the enter digit from Arduino serial monitor
		if (rxbuff[Size - 1] == 10) {
			Size = Size - 1;
		}
		memcpy(mainbuff, rxbuff, Size); // store value fm Uart2, need #include "string.h" //Size - 1
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxbuff, rxbuff_size); // rx stop after receive, restart it again
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		rxdatasize = Size;
		uart_rx_int = 1;
	}
}

// Create counter msg
void printpolestep( txbuffer, polestep) {
	sprintf(txbuffer, "counter = %03d\n\r", polestep);
	tx_msg((char*) txbuffer);
}

// Tx msg from Uart 2
void tx_msg( txbuffer) {
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) txbuffer, strlen(txbuffer));
	HAL_Delay(10);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxbuff, rxbuff_size); // call uart rx function, data store in rxbuff
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // hal dma it start by default, disable half tx IT, this IT trigger when  half data has been transfer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (uart_rx_int == 1) {
			uart_rx_int = 0;

			if (strncmp((char*) mainbuff, "resume", rxdatasize - 1) == 0
					&& strlen(mainbuff) == 6) {
				pause = 0;

				sprintf(txbuffer, "\n '%s' has been recieved \n", mainbuff);
				tx_msg((char*) txbuffer);

				sprintf(txbuffer, "program resume \n\r");
				tx_msg((char*) txbuffer);

			} else if (strncmp((char*) mainbuff, "stop", rxdatasize - 1) == 0
					&& strlen(mainbuff) == 4) {
				pause = 1;

				sprintf(txbuffer, "\n '%s' has been recieved \n", mainbuff);
				tx_msg((char*) txbuffer);

				sprintf(txbuffer, "program stop \n\r");
				tx_msg((char*) txbuffer);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);   // IN1
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);   // IN2

			} else if (strncmp((char*) mainbuff, "forward", rxdatasize - 1) == 0
					&& strlen(mainbuff) == 7) {

				sprintf(txbuffer, " '%s' has been recieved \n", mainbuff);
				tx_msg((char*) txbuffer);
				direction_mode = 1;

			} else if (strncmp((char*) mainbuff, "backward", rxdatasize - 1)
					== 0 && strlen(mainbuff) == 8) {

				sprintf(txbuffer, " '%s' has been recieved \n", mainbuff);
				tx_msg((char*) txbuffer);
				direction_mode = 0;
			}

			else {
				sprintf(txbuffer, " '%s' has been recieved \n", mainbuff);
				tx_msg((char*) txbuffer);
				sprintf(txbuffer, "===invalid input=== \n\r");
				tx_msg((char*) txbuffer);
			}
		}

		if (pause == 0) {
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, bled_state);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, gled_state);
			gled_state = !gled_state;
			bled_state = !bled_state;
			stepper_step_angle(1, direction_mode, 1);
			printpolestep((char*) txbuffer, polestep);
			if (polestep % 360 == 0) {
				HAL_Delay(5000);
			}

		}
		//HAL_Delay(200);

	} // while END
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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

