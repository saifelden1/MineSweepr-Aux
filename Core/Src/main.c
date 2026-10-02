/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32_Auxiliary
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Config & BSP Drivers */
#include "auxiliary_pin_config.h"
#include "status_beacon_driver.h"
#include "servo_gripper_driver.h"
#include "pi_detector_driver.h"

/* Interfaces */
#include "gripper_interface.h"
#include "detector_interface.h"

/* Application Tasks & Middleware */
#include "task_detector.h"
#include "task_gripper.h"
#include "task_microros.h"
#include "microros_client.h"
#include "microros_transport.h"

/* Peripheral Handles --------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* FreeRTOS Thread Definitions -----------------------------------------------*/
osThreadId_t Task_DetectorHandle;
const osThreadAttr_t Task_Detector_attributes = {
  .name = "Task_Detector",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Task_GripperHandle;
const osThreadAttr_t Task_Gripper_attributes = {
  .name = "Task_Gripper",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Task_MicroROSHandle;
const osThreadAttr_t Task_MicroROS_attributes = {
  .name = "Task_MicroROS",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

void StartTaskDetector(void *argument);
void StartTaskGripper(void *argument);
void StartTaskMicroROS(void *argument);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize concrete BSP drivers */
  Status_Beacon_Init();
  Servo_Gripper_Init();
  PI_Detector_Init();

  /* Initialize micro-ROS transport and client */
  MicroROS_Transport_Init(&huart1, &hdma_usart1_rx, &hdma_usart1_tx);
  MicroROS_Client_Init();

  /* Initialize application tasks */
  Task_Detector_Init();
  Task_Gripper_Init();
  Task_MicroROS_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create the thread(s) */
  Task_DetectorHandle = osThreadNew(StartTaskDetector, NULL, &Task_Detector_attributes);
  Task_GripperHandle  = osThreadNew(StartTaskGripper, NULL, &Task_Gripper_attributes);
  Task_MicroROSHandle = osThreadNew(StartTaskMicroROS, NULL, &Task_MicroROS_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function (32-bit hardware counter @ 1 MHz, 1 us/tick)
  */
static void MX_TIM2_Init(void)
{
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;  /* 96 MHz / 96 = 1 MHz */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function (50 Hz PWM on PB6/PB7 for Servo Gripper, ARR=19999, PSC=95)
  */
static void MX_TIM4_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = AUX_GRIPPER_TIMER_INSTANCE;
  htim4.Init.Prescaler = AUX_GRIPPER_PWM_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = AUX_GRIPPER_PWM_PERIOD_TICKS;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = AUX_GRIPPER_PULSE_NEUTRAL_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, AUX_GRIPPER_PWM1_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, AUX_GRIPPER_PWM2_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function (Pulse Induction Decay Tail on PA1)
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = AUX_DETECTOR_ADC_INSTANCE;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = AUX_DETECTOR_ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function (micro-ROS transport @ 921600 baud)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = AUX_MICROROS_UART_INSTANCE;
  huart1.Init.BaudRate = AUX_MICROROS_BAUDRATE;
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
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  AUX_HEARTBEAT_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  AUX_DETECTOR_ADC_CLK_ENABLE();
  AUX_DETECTOR_PULSE_CLK_ENABLE();

  /* Configure PC13 (Heartbeat LED) */
  HAL_GPIO_WritePin(AUX_HEARTBEAT_PORT, AUX_HEARTBEAT_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = AUX_HEARTBEAT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUX_HEARTBEAT_PORT, &GPIO_InitStruct);

  /* Configure PB0 (Metal Detector Pulse Excitation Output) */
  HAL_GPIO_WritePin(AUX_DETECTOR_PULSE_PORT, AUX_DETECTOR_PULSE_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = AUX_DETECTOR_PULSE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AUX_DETECTOR_PULSE_PORT, &GPIO_InitStruct);

  /* Configure PB12 (Alert Siren) and PB13 (Alert Strobe) */
  AUX_BEACON_BUZZER_CLK_ENABLE();
  AUX_BEACON_STROBE_CLK_ENABLE();
  HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUX_BEACON_STROBE_PORT, AUX_BEACON_STROBE_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = AUX_BEACON_BUZZER_PIN | AUX_BEACON_STROBE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUX_BEACON_BUZZER_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Period elapsed callback in non blocking mode (TIM11 SYS tick)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
