/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "Statechart.h" // Incluimos el .h principal de la máquina de estados
#include "Statechart_required.h" // Y el de las funciones requeridas (acciones)
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
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
Statechart seguidor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define VELOCIDAD_AVANCE 60 // Valor de PWM (0 a 65535), ajústalo según necesites
#define VELOCIDAD_GIRO   50 // Velocidad para las curvas
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	statechart_init(&seguidor);
	statechart_enter(&seguidor);

	// Iniciar PWM para ambos motores
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// 1. Leer sensor izquierdo y levantar el evento correspondiente
		if (HAL_GPIO_ReadPin(sensor_izq_GPIO_Port, sensor_izq_Pin) == GPIO_PIN_RESET) // Negro -> 0
		{
			statechart_sensores_raise_sensor_I_0(&seguidor);
			HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);//AZUL
		}
		else // Blanco -> 1
		{
			statechart_sensores_raise_sensor_I_1(&seguidor);
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);//ROJO
		}

		// 2. Leer sensor derecho y levantar el evento correspondiente
		if (HAL_GPIO_ReadPin(sensor_der_GPIO_Port, sensor_der_Pin) == GPIO_PIN_RESET) // Negro -> 0
		{
			statechart_sensores_raise_sensor_D_0(&seguidor);
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);//NARANJA
		}
		else // Blanco -> 1
		{
			statechart_sensores_raise_sensor_D_1(&seguidor);
			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);//VERDE
		}

		// 3. Ejecutar un ciclo de la máquina de estados para procesar los eventos
		//  statechart_trigger_without_event(&seguidor);

		// 4. Pequeña pausa para estabilizar el sistema
		HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |M0_izq_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_der_Pin|M1_izq_Pin|M1_der_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           M0_izq_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |M0_izq_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_izq_Pin sensor_der_Pin */
  GPIO_InitStruct.Pin = sensor_izq_Pin|sensor_der_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_der_Pin M1_izq_Pin M1_der_Pin */
  GPIO_InitStruct.Pin = M0_der_Pin|M1_izq_Pin|M1_der_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- ACCIONES DE LA MÁQUINA DE ESTADOS ---

// --- ESTADO AVANZAR ---
void statechart_acciones_avanzar(Statechart* handle) {
	// Motor Izquierdo: Avance (M1=0, M0=1)
	HAL_GPIO_WritePin(M1_izq_GPIO_Port, M1_izq_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M0_izq_GPIO_Port, M0_izq_Pin, GPIO_PIN_SET);

	// Motor Derecho: Avance (M1=0, M0=1)
	HAL_GPIO_WritePin(M1_der_GPIO_Port, M1_der_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M0_der_GPIO_Port, M0_der_Pin, GPIO_PIN_SET);

	// Establecer velocidad con PWM
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, VELOCIDAD_AVANCE);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, VELOCIDAD_AVANCE);
}

// --- ESTADO DOBLAR A LA IZQUIERDA ---
void statechart_acciones_doblar_izq(Statechart* handle) {
	// Motor Izquierdo: Retroceso (M1=1, M0=0)
	HAL_GPIO_WritePin(M1_izq_GPIO_Port, M1_izq_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M0_izq_GPIO_Port, M0_izq_Pin, GPIO_PIN_RESET);

	// Motor Derecho: Avance (M1=0, M0=1)
	HAL_GPIO_WritePin(M1_der_GPIO_Port, M1_der_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M0_der_GPIO_Port, M0_der_Pin, GPIO_PIN_SET);

	// Establecer velocidad de giro
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, VELOCIDAD_GIRO);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, VELOCIDAD_GIRO);
}

// --- ESTADO DOBLAR A LA DERECHA ---
void statechart_acciones_doblar_der(Statechart* handle) {
	// Motor Izquierdo: Avance (M1=0, M0=1)
	HAL_GPIO_WritePin(M1_izq_GPIO_Port, M1_izq_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M0_izq_GPIO_Port, M0_izq_Pin, GPIO_PIN_SET);

	// Motor Derecho: Retroceso (M1=1, M0=0)
	HAL_GPIO_WritePin(M1_der_GPIO_Port, M1_der_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M0_der_GPIO_Port, M0_der_Pin, GPIO_PIN_RESET);

	// Establecer velocidad de giro
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, VELOCIDAD_GIRO);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, VELOCIDAD_GIRO);
}

// Callback de interrupciones externas (no lo usamos, pero debe existir)
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
// Dejar vacío
//}

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
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
