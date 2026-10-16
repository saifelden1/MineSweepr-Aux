/* STM32F103C8T6 Blue Pill: paired servos, detector and framed UART payload link. */
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "status_beacon_driver.h"
#include "servo_gripper_driver.h"
#include "pi_detector_driver.h"
#include "microros_transport.h"
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); MX_TIM2_Init(); MX_TIM4_Init();
    MX_USART1_UART_Init();
    Status_Beacon_Init();
    if (!Servo_Gripper_Init() || !PI_Detector_Init()) Error_Handler();
    if (osKernelInitialize() != osOK) Error_Handler();
    /* RTOS synchronization objects must be created after kernel initialization. */
    if (!MicroROS_Transport_Init(&huart1, &hdma_usart1_rx, &hdma_usart1_tx)) Error_Handler();
    MX_FREERTOS_Init();
    osKernelStart();
    Error_Handler();
}
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM3) HAL_IncTick();
}
void Error_Handler(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    __disable_irq();
    for (;;) {}
}
