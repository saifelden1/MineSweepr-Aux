/**
 * @file status_beacon_driver.c
 * @brief Board Support Package Driver Implementation for Status Beacon.
 */

#include "status_beacon_driver.h"
#include <string.h>

#define BEACON_STROBE_PERIOD_MS (100U) /**< 10 Hz toggle rate (50 ms ON, 50 ms OFF) */

static status_beacon_state_t s_beacon_state;
static bool s_initialized = false;

static void update_hardware_outputs(void)
{
    if (!s_initialized)
    {
        return;
    }

    s_beacon_state.is_active = (s_beacon_state.local_mine_detected || s_beacon_state.remote_alert_active);

    if (s_beacon_state.is_active)
    {
        /* Sound the audible siren */
        HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN, GPIO_PIN_SET);

        /* Drive strobe LED based on oscillator state */
        HAL_GPIO_WritePin(AUX_BEACON_STROBE_PORT, AUX_BEACON_STROBE_PIN,
                          s_beacon_state.strobe_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else
    {
        /* De-assert both buzzer and strobe LED */
        HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AUX_BEACON_STROBE_PORT, AUX_BEACON_STROBE_PIN, GPIO_PIN_RESET);
        s_beacon_state.strobe_state = false;
        s_beacon_state.strobe_timer_ms = 0;
    }
}

void Status_Beacon_Init(void)
{
    AUX_BEACON_BUZZER_CLK_ENABLE();
    AUX_BEACON_STROBE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure PB12 (Buzzer) */
    HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = AUX_BEACON_BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AUX_BEACON_BUZZER_PORT, &GPIO_InitStruct);

    /* Configure PB13 (Strobe LED) */
    HAL_GPIO_WritePin(AUX_BEACON_STROBE_PORT, AUX_BEACON_STROBE_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = AUX_BEACON_STROBE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AUX_BEACON_STROBE_PORT, &GPIO_InitStruct);

    memset(&s_beacon_state, 0, sizeof(s_beacon_state));
    s_initialized = true;

    update_hardware_outputs();
}

void Status_Beacon_SetMineAlert(bool detected)
{
    s_beacon_state.local_mine_detected = detected;
    update_hardware_outputs();
}

void Status_Beacon_SetRemoteAlert(bool active)
{
    s_beacon_state.remote_alert_active = active;
    update_hardware_outputs();
}

void Status_Beacon_Update(uint32_t delta_ms)
{
    if (!s_initialized)
    {
        return;
    }

    s_beacon_state.is_active = (s_beacon_state.local_mine_detected || s_beacon_state.remote_alert_active);

    if (s_beacon_state.is_active)
    {
        s_beacon_state.strobe_timer_ms += delta_ms;
        if (s_beacon_state.strobe_timer_ms >= (BEACON_STROBE_PERIOD_MS / 2U))
        {
            s_beacon_state.strobe_timer_ms = 0;
            s_beacon_state.strobe_state = !s_beacon_state.strobe_state;
        }
        update_hardware_outputs();
    }
    else
    {
        update_hardware_outputs();
    }
}

bool Status_Beacon_IsActive(void)
{
    return s_beacon_state.is_active;
}

bool Status_Beacon_GetMineAlert(void)
{
    return s_beacon_state.local_mine_detected;
}

bool Status_Beacon_GetRemoteAlert(void)
{
    return s_beacon_state.remote_alert_active;
}

void Status_Beacon_GetState(status_beacon_state_t *out_state)
{
    if (out_state != NULL)
    {
        *out_state = s_beacon_state;
    }
}
