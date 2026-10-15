#include "adc.h"
ADC_HandleTypeDef hadc1;
void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    GPIO_InitTypeDef pin={0}; pin.Pin=GPIO_PIN_1; pin.Mode=GPIO_MODE_ANALOG; HAL_GPIO_Init(GPIOA,&pin);
    hadc1.Instance=ADC1;
    hadc1.Init.ScanConvMode=ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode=DISABLE;
    hadc1.Init.DiscontinuousConvMode=DISABLE;
    hadc1.Init.ExternalTrigConv=ADC_SOFTWARE_START;
    hadc1.Init.DataAlign=ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion=1;
    if (HAL_ADC_Init(&hadc1)!=HAL_OK) Error_Handler();
    ADC_ChannelConfTypeDef ch={0}; ch.Channel=ADC_CHANNEL_1; ch.Rank=ADC_REGULAR_RANK_1;
    ch.SamplingTime=ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1,&ch)!=HAL_OK || HAL_ADCEx_Calibration_Start(&hadc1)!=HAL_OK) Error_Handler();
}
