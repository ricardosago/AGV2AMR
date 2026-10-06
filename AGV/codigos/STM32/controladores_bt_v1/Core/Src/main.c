/* USER CODE BEGIN Header */
/**

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

// Variables globales para cada motor
typedef struct {
    float kp; // Ganancia proporcional
    float ki; // Ganancia integral
    float prev_error; // Error anterior
    float integral; // Acumulador integral
} PIcontroller;


void controlarMotor(TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2,
                    PIcontroller *pid, float setpoint, float feedback, int direccion, float a, float b, int motor);


void movimiento(char str, float setpointM1, float setpointM2, float setpointM3, float setpointM4,
                float feedbackM1, float feedbackM2, float feedbackM3, float feedbackM4, float a_m1, float b_m1,
				float a_m2, float b_m2, float a_m3, float b_m3,float a_m4, float b_m4);

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ENCODER_PPR 2000 // Pulsos por revolución del encoder
#define TIMER_INTERVAL 10 // Intervalo del cálculo de velocidad (ms)
/* Variables globales para los encoders */
char rxData='e';
char datico='e';
int *velocidad_mostrar;

volatile int32_t encoder_count_m1 = 0, encoder_count_m2 = 0, encoder_count_m3 = 0, encoder_count_m4 = 0;
volatile int8_t motor_direction_m1 = 0, motor_direction_m2 = 0, motor_direction_m3 = 0, motor_direction_m4 = 0;
volatile int32_t pulses_in_interval_m1 = 0, pulses_in_interval_m2 = 0, pulses_in_interval_m3 = 0, pulses_in_interval_m4 = 0;
volatile int32_t motor_speed_rpm_m1 = 0, motor_speed_rpm_m2 = 0, motor_speed_rpm_m3 = 0, motor_speed_rpm_m4 = 0;

float a_m1 = 22.697, a_m2 = 23.993, a_m3 = 22.697, a_m4 = 23.119;
float b_m1 = 1200.62, b_m2 = 1200.31, b_m3 = 927.31, b_m4 = 874;

// Función para calcular la salida del controlador PID
float calcularPID(PIcontroller *pid, float setpoint, float feedback, float a, float b) {
    float error = 0.5*(setpoint - feedback);
    pid->integral += error; // Acumulador de la integral
    pid->prev_error = error; // Actualizar error anterior

    return ((pid->kp * error) + (pid->ki * pid->integral))*a + b;
}

PIcontroller controladorM1 = { .kp = 0.328945918270809, .ki = 1.051473493173661};
PIcontroller controladorM2 = { .kp = 0.349547642624270, .ki = 1.172268884423284};
PIcontroller controladorM3 = { .kp = 0.422191752414783, .ki = 1.071228759963182};
PIcontroller controladorM4 = { .kp = 0.400520366470992, .ki = 1.063884770111944};

float feedbackM1, feedbackM2, feedbackM3, feedbackM4 = 0;  // Declare the feedback variables



/* ISR para las señales del encoder */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Motor 1
    if (GPIO_Pin == GPIO_PIN_12 || GPIO_Pin == GPIO_PIN_13) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
            encoder_count_m1++;
            pulses_in_interval_m1++;
            motor_direction_m1 = 1;
        } else {
            encoder_count_m1--;
            pulses_in_interval_m1++;
            motor_direction_m1 = -1;
        }
    }
    // Motor 2
    else if (GPIO_Pin == GPIO_PIN_14 || GPIO_Pin == GPIO_PIN_15) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) {
            encoder_count_m2++;
            pulses_in_interval_m2++;
            motor_direction_m2 = 1;
        } else {
            encoder_count_m2--;
            pulses_in_interval_m2++;
            motor_direction_m2 = -1;
        }
    }
    // Motor 3
    else if (GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_11) {
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)) {
            encoder_count_m3++;
            pulses_in_interval_m3++;
            motor_direction_m3 = 1;
        } else {
            encoder_count_m3--;
            pulses_in_interval_m3++;
            motor_direction_m3 = -1;
        }
    }
    // Motor 4
    else if (GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_1) {
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        // Reactivar la recepción ANTES de procesar datos
        HAL_UART_Receive_IT(&huart6, &rxData, 1);

        datico = rxData;  // Guardar el dato recibido

        // Confirmar que la interrupción se está ejecutando
        char mensaje[] = "Interrupcion UART ejecutada\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)mensaje, strlen(mensaje), 100);

        // Reiniciar las integrales de cada controlador
        controladorM1.integral = 0;
        controladorM2.integral = 0;
        controladorM3.integral = 0;
        controladorM4.integral = 0;
    }
}

/* Temporizador para calcular velocidad */
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
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart6, &rxData, 1);

  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_RCC_GPIOD_CLK_ENABLE();



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

  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  movimiento(datico, 40.0, 40.0, 40.0, 40.0, motor_speed_rpm_m1, motor_speed_rpm_m2, motor_speed_rpm_m3, motor_speed_rpm_m4, a_m1, b_m1,a_m2, b_m2,
	 	 	 	 				 a_m3, b_m3,a_m4, b_m4);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(A7_GPIO_Port, A7_Pin);
	  HAL_Delay(1000);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  HAL_GPIO_WritePin(A7_GPIO_Port, A7_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A7_Pin */
  GPIO_InitStruct.Pin = A7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A7_GPIO_Port, &GPIO_InitStruct);

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
void controlarMotor(TIM_HandleTypeDef *htim, uint32_t channela, uint32_t channelb,
                    PIcontroller *pid, float setpoint, float feedback, int direccion, float a, float b, int motor) {
    // Calcular la salida del controlador
    float salidaControl = calcularPID(pid, setpoint, feedback, a, b);

    // Escalar la salida del controlador al rango de PWM
    int velocidad = (int)(salidaControl);
    if (velocidad < 0) velocidad = 0; // Velocidad mínima
    if (velocidad > 4000) velocidad = 4000; // Velocidad máxima
    if(motor == 4) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "%i\n", velocidad);
        HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 10);
    }

    // Controlar motor según la dirección
    if (direccion == 1) { // Adelante
        __HAL_TIM_SetCompare(htim, channela, velocidad);
        __HAL_TIM_SetCompare(htim, channelb, 0);
    } else if (direccion == -1) { // Atrás
        __HAL_TIM_SetCompare(htim, channela, 0);
        __HAL_TIM_SetCompare(htim, channelb, velocidad);
    } else { // Parar
        __HAL_TIM_SetCompare(htim, channela, 0);
        __HAL_TIM_SetCompare(htim, channelb, 0);
    }
}

void movimiento(char str, float setpointM1, float setpointM2, float setpointM3, float setpointM4,
                float feedbackM1, float feedbackM2, float feedbackM3, float feedbackM4, float a_m1, float b_m1,
				float a_m2, float b_m2, float a_m3, float b_m3,float a_m4, float b_m4) {
    int direccionM1 = 0, direccionM2 = 0, direccionM3 = 0, direccionM4 = 0;

    // Configuración de direcciones según el comando
    switch (str) {
        case 'b': // Adelante
            direccionM1 = direccionM2 = direccionM3 = direccionM4 = 1;
            break;
        case 'h': // Atrás
            direccionM1 = direccionM2 = direccionM3 = direccionM4 = -1;
            break;
        case 'j': // Rotación en sentido horario
            direccionM1 = 1; direccionM2 = -1; direccionM3 = 1; direccionM4 = -1;
            break;
        case 'k': // Rotación en sentido antihorario
            direccionM1 = -1; direccionM2 = 1; direccionM3 = -1; direccionM4 = 1;
            break;
        case 'c': // Adelante 45° derecha
            direccionM1 = 1; direccionM2 = 0; direccionM3 = 0; direccionM4 = 1;
            break;
        case 'a': // Adelante 45° izquierda
            direccionM1 = 0; direccionM2 = 1; direccionM3 = 1; direccionM4 = 0;
            break;
        case 'g': // Atrás 45° izquierda
        	direccionM1 = -1; direccionM2 = 0; direccionM3 = 0; direccionM4 = -1;
            break;
        case 'i': // Atrás 45° derecha
        	direccionM1 = 0; direccionM2 = -1; direccionM3 = -1; direccionM4 = 0;
            break;
        case 'f': // Movimiento lateral a la derecha
            direccionM1 = 1; direccionM2 = -1; direccionM3 = -1; direccionM4 = 1;
            break;
        case 'd': // Movimiento lateral a la izquierda
            direccionM1 = -1; direccionM2 = 1; direccionM3 = 1; direccionM4 = -1;
            break;
        case 'e': // Parar
        default:
            direccionM1 = direccionM2 = direccionM3 = direccionM4 = 0;
            break;
    }
    // Controlar cada motor con su controlador PID
    controlarMotor(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, &controladorM1, setpointM1, feedbackM1, direccionM1, a_m1, b_m1, 1);
    controlarMotor(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, &controladorM2, setpointM2, feedbackM2, direccionM2, a_m2, b_m2, 2);
    controlarMotor(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, &controladorM3, setpointM3, feedbackM3, direccionM3, a_m3, b_m3, 3);
    controlarMotor(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, &controladorM4, setpointM4, feedbackM4, direccionM4, a_m4, b_m4, 4);
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
