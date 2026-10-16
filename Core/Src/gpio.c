#include "gpio.h"
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* Retain SWD programming pins. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    GPIO_InitTypeDef pin = {0};
    pin.Pin=GPIO_PIN_0;
    pin.Mode=GPIO_MODE_OUTPUT_PP; pin.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&pin);
    /* PB12 is initialized inactive by Status_Beacon_Init; old LEDs stay inputs. */
}
