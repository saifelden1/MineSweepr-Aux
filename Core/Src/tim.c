#include "tim.h"
#include "auxiliary_pin_config.h"
TIM_HandleTypeDef htim2, htim4;
void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance=TIM2; htim2.Init.Prescaler=71; htim2.Init.Period=65535;
    htim2.Init.CounterMode=TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim2)!=HAL_OK) Error_Handler();
}
void MX_TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance=TIM4; htim4.Init.Prescaler=AUX_GRIPPER_PWM_PRESCALER;
    htim4.Init.Period=AUX_GRIPPER_PWM_PERIOD_TICKS; htim4.Init.CounterMode=TIM_COUNTERMODE_UP;
    if (HAL_TIM_PWM_Init(&htim4)!=HAL_OK) Error_Handler();
    TIM_OC_InitTypeDef ch={0}; ch.OCMode=TIM_OCMODE_PWM1; ch.Pulse=AUX_GRIPPER_PULSE_NEUTRAL_US;
    ch.OCPolarity=TIM_OCPOLARITY_HIGH; ch.OCFastMode=TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4,&ch,AUX_GRIPPER_PWM1_CHANNEL)!=HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4,&ch,AUX_GRIPPER_PWM2_CHANNEL)!=HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim4);
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance==TIM4) {
        GPIO_InitTypeDef pin={0};
        pin.Pin=AUX_GRIPPER_PWM1_PIN | AUX_GRIPPER_PWM2_PIN;
        pin.Mode=GPIO_MODE_AF_PP;
        pin.Speed=GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB,&pin);
    }
}
