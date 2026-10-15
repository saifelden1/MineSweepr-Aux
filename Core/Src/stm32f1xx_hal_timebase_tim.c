/* Separate HAL millisecond clock: SysTick belongs to FreeRTOS. */
#include "stm32f1xx_hal.h"
TIM_HandleTypeDef htim3;
HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    uint32_t clock=HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1)!=0) clock*=2;
    htim3.Instance=TIM3; htim3.Init.Prescaler=clock/1000000U-1;
    htim3.Init.Period=999; htim3.Init.CounterMode=TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim3)!=HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(TIM3_IRQn,priority,0); HAL_NVIC_EnableIRQ(TIM3_IRQn); uwTickPrio=priority;
    return HAL_TIM_Base_Start_IT(&htim3);
}
void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&htim3,TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE); }
