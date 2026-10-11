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
        s_beacon_state.buzzer_state = true;

        /* Drive LED 1 / Strobe LED based on oscillator state */
        HAL_GPIO_WritePin(AUX_LED1_PORT, AUX_LED1_PIN,
                          s_beacon_state.strobe_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        s_beacon_state.led1_state = s_beacon_state.strobe_state;
    }
    else
    {
        /* De-assert both buzzer and strobe LED unless explicitly turned on */
        HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN,
                          s_beacon_state.buzzer_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AUX_LED1_PORT, AUX_LED1_PIN,
                          s_beacon_state.led1_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        s_beacon_state.strobe_state = false;
        s_beacon_state.strobe_timer_ms = 0;
    }

    /* Drive LED 2 (PB14 - Gripper / Payload) */
    HAL_GPIO_WritePin(AUX_LED2_PORT, AUX_LED2_PIN,
                      s_beacon_state.led2_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Drive LED 3 (PB15 - System Ready) */
    HAL_GPIO_WritePin(AUX_LED3_PORT, AUX_LED3_PIN,
                      s_beacon_state.led3_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Status_Beacon_Init(void)
{
    AUX_BEACON_BUZZER_CLK_ENABLE();
    AUX_LED1_CLK_ENABLE();
    AUX_LED2_CLK_ENABLE();
    AUX_LED3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure PB12 (Buzzer) */
    HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = AUX_BEACON_BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AUX_BEACON_BUZZER_PORT, &GPIO_InitStruct);

    /* Configure 3 Status LEDs: PB13 (LED1), PB14 (LED2), PB15 (LED3) */
    HAL_GPIO_WritePin(AUX_LED1_PORT, AUX_LED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AUX_LED2_PORT, AUX_LED2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AUX_LED3_PORT, AUX_LED3_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = AUX_LED1_PIN | AUX_LED2_PIN | AUX_LED3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AUX_LED1_PORT, &GPIO_InitStruct);

    memset(&s_beacon_state, 0, sizeof(s_beacon_state));
    s_initialized = true;

    update_hardware_outputs();
}

void Status_Beacon_SetMineAlert(bool detected)
{
    s_beacon_state.local_mine_detected = detected;
    if (!detected)
    {
        s_beacon_state.buzzer_state = false;
        s_beacon_state.led1_state = false;
    }
    update_hardware_outputs();
}

void Status_Beacon_SetRemoteAlert(bool active)
{
    s_beacon_state.remote_alert_active = active;
    if (!active && !s_beacon_state.local_mine_detected)
    {
        s_beacon_state.buzzer_state = false;
        s_beacon_state.led1_state = false;
    }
    update_hardware_outputs();
}

void Status_Beacon_SetLed(uint8_t led_id, bool state)
{
    switch (led_id)
    {
        case 1:
            s_beacon_state.led1_state = state;
            break;
        case 2:
            s_beacon_state.led2_state = state;
            break;
        case 3:
            s_beacon_state.led3_state = state;
            break;
        default:
            return;
    }
    update_hardware_outputs();
}

void Status_Beacon_SetBuzzer(bool state)
{
    s_beacon_state.buzzer_state = state;
    update_hardware_outputs();
}

void Status_Beacon_SetPayloadActive(bool active)
{
    s_beacon_state.led2_state = active;
    update_hardware_outputs();
}

void Status_Beacon_SetSystemReady(bool ready)
{
    s_beacon_state.led3_state = ready;
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
