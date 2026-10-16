#include "status_beacon_driver.h"

static bool s_initialized;
static bool s_active;

void Status_Beacon_Init(void)
{
    AUX_RELAY_CLK_ENABLE();
    /* Set the inactive output latch before enabling the GPIO driver. */
    HAL_GPIO_WritePin(AUX_RELAY_PORT, AUX_RELAY_PIN, AUX_RELAY_OFF_LEVEL);
    GPIO_InitTypeDef pin = {0};
    pin.Pin = AUX_RELAY_PIN;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AUX_RELAY_PORT, &pin);
    s_active = false;
    s_initialized = true;
}

void Status_Beacon_SetMineAlert(bool detected)
{
    if (!s_initialized) return;
    s_active = detected;
    HAL_GPIO_WritePin(AUX_RELAY_PORT, AUX_RELAY_PIN,
                      detected ? AUX_RELAY_ON_LEVEL : AUX_RELAY_OFF_LEVEL);
}

bool Status_Beacon_IsActive(void)
{
    return s_active;
}
