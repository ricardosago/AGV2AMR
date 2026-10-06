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
#include <stdio.h>
#include <string.h>

// Variables globales para cada motor
typedef struct {
    float q0;        // coeficiente PI discreto
    float q1;        // coeficiente PI discreto

    float e_prev;    // error anterior
    float u_prev;    // salida anterior (PWM)
} PIController;


void controlarMotorPI(TIM_HandleTypeDef *htim, uint32_t chA, uint32_t chB,
                      PIController *pi, float setpoint, float feedback, int dir);

void movimiento(char str,
                float sp1, float sp2, float sp3, float sp4,
                float fb1, float fb2, float fb3, float fb4);
// ===== Límite de seguridad y helpers PWM =====
#define MAX_DUTY_PCT  50.0f   // tope de PWM en %, ajusta a tu gusto

static inline uint32_t pwm_get_arr(TIM_HandleTypeDef *htim) {
    return __HAL_TIM_GET_AUTORELOAD(htim);  // = Period (ARR) del timer
}

static inline uint32_t pwm_pct_to_ccr(TIM_HandleTypeDef *htim, float pct) {
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    float arr = (float)pwm_get_arr(htim);
    return (uint32_t)((pct * arr) / 100.0f + 0.5f);
}


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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// === ADDED ===
volatile char g_cmd = 'e';          // comando actual (empieza en STOP)
volatile uint8_t control_tick = 0;  // flag para ejecutar control cada 10 ms
uint8_t rx3, rx5;                   // buffers de RX por IT para USART3 y UART5

#define ENCODER_PPR 500 // Pulsos por revolución del encoder
#define TIMER_INTERVAL 100 // Intervalo del cálculo de velocidad (ms)
/* Variables globales para los encoders */
char rxData='b';
char datico='b';
int *velocidad_mostrar;

volatile int32_t encoder_count_m1 = 0, encoder_count_m2 = 0, encoder_count_m3 = 0, encoder_count_m4 = 0;
volatile int8_t motor_direction_m1 = 0, motor_direction_m2 = 0, motor_direction_m3 = 0, motor_direction_m4 = 0;
volatile int32_t pulses_in_interval_m1 = 0, pulses_in_interval_m2 = 0, pulses_in_interval_m3 = 0, pulses_in_interval_m4 = 0;
volatile int32_t motor_speed_rpm_m1 = 0, motor_speed_rpm_m2 = 0, motor_speed_rpm_m3 = 0, motor_speed_rpm_m4 = 0;

float a_m1 = 22.697, a_m2 = 23.993, a_m3 = 22.697, a_m4 = 23.119;
float b_m1 = 1200.62, b_m2 = 1200.31, b_m3 = 927.31, b_m4 = 874;

float PI_Compute(PIController *pi, float setpoint, float feedback)
{
    float e = setpoint - feedback;

    // cálculo incremental sin límites
    float u = pi->u_prev +
              pi->q0 * e +
              pi->q1 * pi->e_prev;

    // ---- ANTI-WINDUP ----
    if (u > MAX_DUTY_PCT) {
        u = MAX_DUTY_PCT;
    }
    else if (u < 0) {
        u = 0;
    } else {
        // solo actualizamos pi->u_prev si NO está saturado
        pi->u_prev = u;
    }

    pi->e_prev = e;

    return u;
}




#define TS_GLOBAL 0.003f   // 3 ms (Tu TIMER_INTERVAL = 3 ms)

PIController piM1 = { .q0 = 0.0662576184809118f,
                      .q1 = -0.0288467565916015f,
                      .e_prev = 0,
                      .u_prev = 0 };

PIController piM2 = { .q0 = 0.0662576184809118f,
						.q1 = -0.0288467565916015f,
                      .e_prev = 0,
                      .u_prev = 0 };

PIController piM3 = { 	.q0 = 0.0662576184809118f,
        				.q1 = -0.0288467565916015f,
                      .e_prev = 0,
                      .u_prev = 0 };

PIController piM4 = { .q0 = 0.0662576184809118f,
        				.q1 = -0.0288467565916015f,
                      .e_prev = 0,
                      .u_prev = 0 };

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
    char c;
    if (huart->Instance == USART3) {
        c = (char)rx3;
        HAL_UART_Receive_IT(&huart3, &rx3, 1); // rearmar
    } else if (huart->Instance == UART5) {
        c = (char)rx5;
        HAL_UART_Receive_IT(&huart5, &rx5, 1); // rearmar
    } else {
        return;
    }

    // Si es un comando válido, actualizar g_cmd y resetear integrales (anti-windup)
    if (c=='a'||c=='b'||c=='c'||c=='d'||c=='e'||c=='f'||c=='g'||c=='h'||c=='i'||c=='j'||c=='k') {
        g_cmd = c;

        // (Opcional) eco por consola:
        // HAL_UART_Transmit(&huart3, (uint8_t*)&g_cmd, 1, 50);
    }
}


/* Temporizador para calcular velocidad */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
    	static float f1=0, f2=0, f3=0, f4=0;

    	float rpm1 = (pulses_in_interval_m1 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);
    	float rpm2 = (pulses_in_interval_m2 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);
    	float rpm3 = (pulses_in_interval_m3 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);
    	float rpm4 = (pulses_in_interval_m4 * 60 * 1000) / (ENCODER_PPR * TIMER_INTERVAL);

    	// filtro exponencial: 70% pasado – 30% nuevo
    	f1 = 0.7f * f1 + 0.3f * rpm1;
    	f2 = 0.7f * f2 + 0.3f * rpm2;
    	f3 = 0.7f * f3 + 0.3f * rpm3;
    	f4 = 0.7f * f4 + 0.3f * rpm4;

    	motor_speed_rpm_m1 = rpm1;
    	motor_speed_rpm_m2 = rpm2;
    	motor_speed_rpm_m3 = rpm3;
    	motor_speed_rpm_m4 = rpm4;


        if (pulses_in_interval_m1 == 0) motor_speed_rpm_m1 = 0, motor_direction_m1 = 0;
        if (pulses_in_interval_m2 == 0) motor_speed_rpm_m2 = 0, motor_direction_m2 = 0;
        if (pulses_in_interval_m3 == 0) motor_speed_rpm_m3 = 0, motor_direction_m3 = 0;
        if (pulses_in_interval_m4 == 0) motor_speed_rpm_m4 = 0, motor_direction_m4 = 0;

        pulses_in_interval_m1 = pulses_in_interval_m2 = pulses_in_interval_m3 = pulses_in_interval_m4 = 0;
        control_tick = 1;
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
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Arranca RX por interrupción en ambos UARTs
  HAL_UART_Receive_IT(&huart3, &rx3, 1);
  HAL_UART_Receive_IT(&huart5, &rx5, 1);

  // Timer de 10 ms para velocidades y tick de control
  HAL_TIM_Base_Start_IT(&htim3);

  // PWM: usar HAL_TIM_PWM_Start (no *_Start_IT)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // Inicializa duty en 0
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if (control_tick) {
	        control_tick = 0;

	        float sp_rpm = 60.0f;
	        movimiento(g_cmd,
	                   sp_rpm, sp_rpm, sp_rpm, sp_rpm,
	                   motor_speed_rpm_m1, motor_speed_rpm_m2,
	                   motor_speed_rpm_m3, motor_speed_rpm_m4);

	    }
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
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
  htim3.Init.Prescaler = 839;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
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
void controlarMotorPI(TIM_HandleTypeDef *htim, uint32_t chA, uint32_t chB,
                      PIController *pi, float setpoint, float feedback, int dir)
{
    float u_pct = PI_Compute(pi, setpoint, feedback);

    if (u_pct > MAX_DUTY_PCT) u_pct = MAX_DUTY_PCT;
    if (u_pct < 0) u_pct = 0;

    uint32_t ccr = pwm_pct_to_ccr(htim, u_pct);

    if (dir == 1) {
        __HAL_TIM_SetCompare(htim, chA, ccr);
        __HAL_TIM_SetCompare(htim, chB, 0);
    }
    else if (dir == -1) {
        __HAL_TIM_SetCompare(htim, chA, 0);
        __HAL_TIM_SetCompare(htim, chB, ccr);
    }
    else {
        __HAL_TIM_SetCompare(htim, chA, 0);
        __HAL_TIM_SetCompare(htim, chB, 0);
    }
}


void movimiento(char str,
                float sp1, float sp2, float sp3, float sp4,
                float fb1, float fb2, float fb3, float fb4)
{
    int dir1 = 0, dir2 = 0, dir3 = 0, dir4 = 0;

    switch (str) {
        case 'b': dir1=dir3=1; dir2=dir4=-1; break;   // Adelante
        case 'h': dir1=dir3=-1; dir2=dir4=1; break;   // Atrás
        case 'j': dir1=dir2=dir3=dir4=1; break;       // Giro horario
        case 'k': dir1=dir2=dir3=dir4=-1; break;      // Giro antihorario
        case 'c': dir1=1; dir4=-1; break;             // 45° adelante derecha
        case 'a': dir2=-1; dir3=1; break;             // 45° adelante izquierda
        case 'g': dir1=-1; dir4=1; break;
        case 'i': dir2=1; dir3=-1; break;
        case 'f': dir1=dir2=1; dir3=dir4=-1; break;   // Lateral derecha
        case 'd': dir1=dir2=-1; dir3=dir4=1; break;   // Lateral izquierda
        case 'e':
        default: dir1=dir2=dir3=dir4 = 0; break;      // Stop
    }

    controlarMotorPI(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                      &piM1, sp1, fb1, dir1);

    controlarMotorPI(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4,
                      &piM2, sp2, fb2, dir2);

    controlarMotorPI(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2,
                      &piM3, sp3, fb3, dir3);

    controlarMotorPI(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4,
                      &piM4, sp4, fb4, dir4);

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
