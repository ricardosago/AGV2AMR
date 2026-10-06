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
#include <string.h>
#include <stdio.h>
void movimiento(char str, int speed);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ENCODER_PPR 2000 // Pulsos por revolución del encoder
#define TIMER_INTERVAL  10 // Intervalo del cálculo de velocidad (ms)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile int32_t encoder_count_m1 = 0, encoder_count_m2 = 0, encoder_count_m3 = 0, encoder_count_m4 = 0;
volatile int8_t motor_direction_m1 = 0, motor_direction_m2 = 0, motor_direction_m3 = 0, motor_direction_m4 = 0;
volatile int32_t pulses_in_interval_m1 = 0, pulses_in_interval_m2 = 0, pulses_in_interval_m3 = 0, pulses_in_interval_m4 = 0;
volatile int32_t motor_speed_rpm_m1 = 0, motor_speed_rpm_m2 = 0, motor_speed_rpm_m3 = 0, motor_speed_rpm_m4 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ISR para las señales del encoder de los 4 motores */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12 || GPIO_Pin == GPIO_PIN_13) {  // Motor 1
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
            encoder_count_m1++;
            pulses_in_interval_m1++;
            motor_direction_m1 = 1;
        } else {
            encoder_count_m1--;
            pulses_in_interval_m1++;
            motor_direction_m1 = -1;
        }
    } else if (GPIO_Pin == GPIO_PIN_14 || GPIO_Pin == GPIO_PIN_15) {  // Motor 2
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) {
            encoder_count_m2++;
            pulses_in_interval_m2++;
            motor_direction_m2 = 1;
        } else {
            encoder_count_m2--;
            pulses_in_interval_m2++;
            motor_direction_m2 = -1;
        }
    } else if (GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_11) {  // Motor 3
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)) {
            encoder_count_m3++;
            pulses_in_interval_m3++;
            motor_direction_m3 = 1;
        } else {
            encoder_count_m3--;
            pulses_in_interval_m3++;
            motor_direction_m3 = -1;
        }
    } else if (GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_1) {  // Motor 4
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            encoder_count_m4++;
            pulses_in_interval_m4++;
            motor_direction_m4 = 1;
        } else {
            encoder_count_m4--;
            pulses_in_interval_m4++;
            motor_direction_m4 = -1;
        }
    }
}

/* Temporizador para calcular la velocidad de los 4 motores */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        motor_speed_rpm_m1 = (pulses_in_interval_m1 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);
        motor_speed_rpm_m2 = (pulses_in_interval_m2 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);
        motor_speed_rpm_m3 = (pulses_in_interval_m3 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);
        motor_speed_rpm_m4 = (pulses_in_interval_m4 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);

        if (pulses_in_interval_m1 == 0) motor_speed_rpm_m1 = 0, motor_direction_m1 = 0;
        if (pulses_in_interval_m2 == 0) motor_speed_rpm_m2 = 0, motor_direction_m2 = 0;
        if (pulses_in_interval_m3 == 0) motor_speed_rpm_m3 = 0, motor_direction_m3 = 0;
        if (pulses_in_interval_m4 == 0) motor_speed_rpm_m4 = 0, motor_direction_m4 = 0;

        pulses_in_interval_m1 = pulses_in_interval_m2 = pulses_in_interval_m3 = pulses_in_interval_m4 = 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_RCC_GPIOD_CLK_ENABLE();
  char buffer[200];


  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);

  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      char rxData;

      // Recibir datos por UART6, si los hay
      if (HAL_UART_Receive(&huart6, &rxData, 1, 100) == HAL_OK) {
          movimiento(rxData, 2520);
      }

      // Mostrar solo la velocidad del motor por consola
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "%ld\n", motor_speed_rpm_m4);
      HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 50);

      HAL_Delay(10); // Ajusta el retraso según la frecuencia deseada de actualización

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void movimiento(char str, int speed) {
	// Motor 1
	 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

	 // Motor 2
	 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
	 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

	 // Motor 3
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

	 // Motor 4
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);

	 HAL_Delay(1000);

		// Stop
		if (str  == 'e' )
		{
			return;
		}

	// Adelante
	if (str == 'b' )
	{
		// Motor 1
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

		 // Motor 2
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, speed);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

		 // Motor 3
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speed);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

		 // Motor 4
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, speed);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
	}

	// Atras
	if (str == 'h' )
	{
		// Motor 1
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);

		 // Motor 2
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, speed);

		 // Motor 3
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speed);

		 // Motor 4
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, speed);

	}

	// Rotación en sentido horario
	if (str == 'j' )
	{
		// Motor 1
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

		 // Motor 2
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, speed);

		 // Motor 3
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speed);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

		 // Motor 4
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, speed);
	}

	// Rotación en sentido antihorario
	if (str == 'k' )
	{
		// Motor 1
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);

		 // Motor 2
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, speed);
		 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

		 // Motor 3
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speed);

		 // Motor 4
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, speed);
		 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
	}

	// Adelante 45° derecha
	if (str == 'c' )
	{
		// Motor 1
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

		// Motor 2
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

		 // Motor 3
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

		// Motor 4
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, speed);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
	}

	// Adelante 45° izquierda
	if (str == 'a' )
	{
		// Motor 1
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

		// Motor 2
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, speed);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

		 // Motor 3
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

		// Motor 4
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
	}

	// Atrás 45° izquierda
	if (str == 'g' )
	{
		// Motor 1
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);

		// Motor 2
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

		 // Motor 3
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

		// Motor 4
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, speed);
	}
	// Atrás 45° derecha
	if (str == 'i' )
	{
		// Motor 1
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

		// Motor 2
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, speed);

		 // Motor 3
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speed);

		// Motor 4
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
	}

	// Derecha
	if (str == 'f' )
	{
		// Motor 1
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);

		// Motor 2
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, speed);

		 // Motor 3
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speed);

		// Motor 4
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, speed);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
	}
	// Izquierda
	if (str == 'd' )
	{
		// Motor 1
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);

		// Motor 2
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, speed);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

		 // Motor 3
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);

		// Motor 4
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, speed);
	}
}
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
