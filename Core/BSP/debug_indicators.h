/**
 * @file debug_indicators.h
 * @brief Board Support Package Driver for Debug Indicators (LEDs, Heartbeat, Buzzer).
 *
 * Microcontroller: STM32F411CEU6 BlackPill (ARM Cortex-M4F @ 96 MHz)
 * Hardware Assignments:
 *   - PB12: DEBUG_BUZZER_PORT / DEBUG_BUZZER_PIN (Audible Alarm / Debug Tone)
 *   - PB13: DEBUG_LED1_PORT / DEBUG_LED1_PIN [Red - Error / Target Alert]
 *   - PB14: DEBUG_LED2_PORT / DEBUG_LED2_PIN [Yellow - Motion / Pulse Busy]
 *   - PB15: DEBUG_LED3_PORT / DEBUG_LED3_PIN [Green - ROS OK / System Ready]
 *   - PC13: DEBUG_HEARTBEAT_PORT / DEBUG_HEARTBEAT_PIN [Active-LOW User LED]
 */

#ifndef DEBUG_INDICATORS_H
#define DEBUG_INDICATORS_H

#include <stdint.h>
#include <stdbool.h>
#include "auxiliary_pin_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Logical identifiers for debug LEDs.
 */
typedef enum {
    DEBUG_LED_1         = 1, /**< PB13 - Red: Error / Target Alert */
    DEBUG_LED_2         = 2, /**< PB14 - Yellow: Motion / Pulse Busy */
    DEBUG_LED_3         = 3, /**< PB15 - Green: ROS OK / System Ready */
    DEBUG_LED_HB        = 4, /**< PC13 - Active-LOW Heartbeat */

    DEBUG_LED_RED       = 1, /**< Alias for LED 1 */
    DEBUG_LED_YELLOW    = 2, /**< Alias for LED 2 */
    DEBUG_LED_GREEN     = 3, /**< Alias for LED 3 */
    DEBUG_LED_HEARTBEAT = 4  /**< Alias for Heartbeat LED */
} debug_led_id_t;

/**
 * @brief Initialize GPIO peripherals for all debug indicators (PB12, PB13, PB14, PB15, PC13).
 */
void Debug_Indicators_Init(void);

/**
 * @brief Set output state of a specific debug LED.
 * @param led LED identifier: 1 (Red), 2 (Yellow), 3 (Green), 4 (Heartbeat).
 * @param state true = ON (illuminated), false = OFF.
 */
void Debug_LED_Set(uint8_t led, bool state);

/**
 * @brief Toggle state of a specific debug LED.
 * @param led LED identifier: 1 (Red), 2 (Yellow), 3 (Green), 4 (Heartbeat).
 */
void Debug_LED_Toggle(uint8_t led);

/**
 * @brief Set state of the audible debug buzzer.
 * @param state true = ON (sounding), false = OFF (silent).
 */
void Debug_Buzzer_Set(bool state);

/**
 * @brief Sound buzzer for a specified duration in milliseconds (non-blocking).
 * @param ms Duration in milliseconds. If ms == 0, buzzer is turned OFF immediately.
 */
void Debug_Buzzer_Beep(uint32_t ms);

/**
 * @brief Periodic update function for non-blocking buzzer duration management.
 * Evaluates elapsed time via HAL_GetTick() and silences buzzer when expired.
 */
void Debug_Indicators_Update(void);

/**
 * @brief Blocking helper to sound buzzer for a specified duration.
 * @param ms Duration in milliseconds.
 */
void Debug_Buzzer_Beep_Blocking(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_INDICATORS_H */
