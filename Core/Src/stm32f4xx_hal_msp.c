/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes for STM32_Auxiliary.
  ******************************************************************************
  */

#include "main.h"
#include "auxiliary_pin_config.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

/**
  * @brief TIM Base MSP Initialization for TIM2 (32-bit 1 MHz Microsecond reference)
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if (htim_base->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if (htim_base->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
}

/**
  * @brief TIM PWM MSP Initialization for Servo Gripper (TIM4 CH1 on PB6, CH2 on PB7)
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if (htim_pwm->Instance == AUX_GRIPPER_TIMER_INSTANCE)
  {
    AUX_GRIPPER_TIM_CLK_ENABLE();
    AUX_GRIPPER_PWM1_CLK_ENABLE();

    /** TIM4 GPIO Configuration: PB6 -> TIM4_CH1, PB7 -> TIM4_CH2 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AUX_GRIPPER_PWM1_PIN | AUX_GRIPPER_PWM2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = AUX_GRIPPER_PWM1_AF;
    HAL_GPIO_Init(AUX_GRIPPER_PWM1_PORT, &GPIO_InitStruct);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if (htim_pwm->Instance == AUX_GRIPPER_TIMER_INSTANCE)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(AUX_GRIPPER_PWM1_PORT, AUX_GRIPPER_PWM1_PIN | AUX_GRIPPER_PWM2_PIN);
  }
}

/**
  * @brief ADC MSP Initialization for Pulse Induction Detector (ADC1 on PA1)
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == AUX_DETECTOR_ADC_INSTANCE)
  {
    AUX_DETECTOR_ADC_CLK_ENABLE_ADC();
    AUX_DETECTOR_ADC_CLK_ENABLE();

    /** ADC1 GPIO Configuration: PA1 -> ADC1_IN1 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AUX_DETECTOR_ADC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AUX_DETECTOR_ADC_PORT, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == AUX_DETECTOR_ADC_INSTANCE)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(AUX_DETECTOR_ADC_PORT, AUX_DETECTOR_ADC_PIN);
  }
}

extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @brief UART MSP Initialization for USART1 (micro-ROS transport)
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /** USART1 GPIO Configuration: PA9 -> USART1_TX, PA10 -> USART1_RX */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1_TX DMA Init (DMA2_Stream7, Channel 4) */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

    /* USART1_RX DMA Init (DMA2_Stream2, Channel 4) */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

    /* NVIC configuration for DMA and USART */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

    HAL_DMA_DeInit(huart->hdmatx);
    HAL_DMA_DeInit(huart->hdmarx);

    HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
}
