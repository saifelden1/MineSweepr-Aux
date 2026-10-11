/**
 * @file debug_indicators.c
 * @brief Board Support Package Driver Implementation for Debug Indicators.
 */

#include "debug_indicators.h"
#include <string.h>

static volatile uint32_t s_beep_end_tick = 0;
static volatile bool s_beep_active = false;
static bool s_indicators_initialized = false;

void Debug_Indicators_Init(void)
{
    /* Enable GPIO Peripheral Clocks */
    DEBUG_BUZZER_CLK_ENABLE();
    DEBUG_LED1_CLK_ENABLE();
    DEBUG_LED2_CLK_ENABLE();
    DEBUG_LED3_CLK_ENABLE();
    DEBUG_HEARTBEAT_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. Configure PB12 (Audible Buzzer) - Initial state OFF (LOW) */
    DEBUG_BUZZER_OFF();
    GPIO_InitStruct.Pin = DEBUG_BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_BUZZER_PORT, &GPIO_InitStruct);

    /* 2. Configure 3 Status LEDs: PB13 (Red), PB14 (Yellow), PB15 (Green) - Initial state OFF (LOW) */
    DEBUG_LED1_OFF();
    DEBUG_LED2_OFF();
    DEBUG_LED3_OFF();
    GPIO_InitStruct.Pin = DEBUG_LED1_PIN | DEBUG_LED2_PIN | DEBUG_LED3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_LED1_PORT, &GPIO_InitStruct);

    /* 3. Configure PC13 (Diagnostic Heartbeat LED - Active LOW) - Initial state OFF (HIGH) */
    DEBUG_HEARTBEAT_OFF();
    GPIO_InitStruct.Pin = DEBUG_HEARTBEAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_HEARTBEAT_PORT, &GPIO_InitStruct);

    s_beep_active = false;
    s_beep_end_tick = 0;
    s_indicators_initialized = true;
}

void Debug_LED_Set(uint8_t led, bool state)
{
    switch (led)
    {
        case DEBUG_LED_1:
            if (state)
            {
                DEBUG_LED1_ON();
            }
            else
            {
                DEBUG_LED1_OFF();
            }
            break;

        case DEBUG_LED_2:
            if (state)
            {
                DEBUG_LED2_ON();
            }
            else
            {
                DEBUG_LED2_OFF();
            }
            break;

        case DEBUG_LED_3:
            if (state)
            {
                DEBUG_LED3_ON();
            }
            else
            {
                DEBUG_LED3_OFF();
            }
            break;

        case DEBUG_LED_HB:
            /* Active-LOW LED: state=true -> ON (RESET), state=false -> OFF (SET) */
            if (state)
            {
                DEBUG_HEARTBEAT_ON();
            }
            else
            {
                DEBUG_HEARTBEAT_OFF();
            }
            break;

        default:
            break;
    }
}

void Debug_LED_Toggle(uint8_t led)
{
    switch (led)
    {
        case DEBUG_LED_1:
            DEBUG_LED1_TOGGLE();
            break;

        case DEBUG_LED_2:
            DEBUG_LED2_TOGGLE();
            break;

        case DEBUG_LED_3:
            DEBUG_LED3_TOGGLE();
            break;

        case DEBUG_LED_HB:
            DEBUG_HEARTBEAT_TOGGLE();
            break;

        default:
            break;
    }
}

void Debug_Buzzer_Set(bool state)
{
    if (state)
    {
        DEBUG_BUZZER_ON();
    }
    else
    {
        DEBUG_BUZZER_OFF();
    }
}

void Debug_Buzzer_Beep(uint32_t ms)
{
    if (ms == 0)
    {
        Debug_Buzzer_Set(false);
        s_beep_active = false;
        return;
    }

    Debug_Buzzer_Set(true);
    s_beep_end_tick = HAL_GetTick() + ms;
    s_beep_active = true;
}

void Debug_Indicators_Update(void)
{
    if (s_beep_active)
    {
        /* Check if target tick reached (safe against unsigned 32-bit wrap) */
        if ((int32_t)(HAL_GetTick() - s_beep_end_tick) >= 0)
        {
            Debug_Buzzer_Set(false);
            s_beep_active = false;
        }
    }
}

void Debug_Buzzer_Beep_Blocking(uint32_t ms)
{
    if (ms == 0)
    {
        return;
    }

    Debug_Buzzer_Set(true);
    HAL_Delay(ms);
    Debug_Buzzer_Set(false);
}
