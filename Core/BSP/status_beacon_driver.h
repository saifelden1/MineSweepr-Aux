/**
 * @file status_beacon_driver.h
 * @brief Board Support Package Driver for Status Beacon (Audible Buzzer & Visual Strobe LED).
 *
 * Implements dual-input alarm activation:
 *   AlarmActive = (local_mine_detected || remote_alert_active)
 * Controls PB12 (Buzzer) and PB13 (Strobe LED) based on centralized auxiliary_pin_config.h.
 */

#ifndef STATUS_BEACON_DRIVER_H
#define STATUS_BEACON_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "auxiliary_pin_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool local_mine_detected; /**< Mine detection flag from local pulse induction sensor */
    bool remote_alert_active; /**< Remote override flag from ROS 2 /beacon/alert topic */
    bool is_active;           /**< Effective alarm state: (local || remote) */
    uint32_t strobe_timer_ms; /**< Millisecond accumulator for strobe timing */
    bool strobe_state;        /**< Current physical output state of strobe LED */
    bool led1_state;          /**< State of LED 1 (PB13, Red / Mine Alert) */
    bool led2_state;          /**< State of LED 2 (PB14, Yellow / Gripper Active) */
    bool led3_state;          /**< State of LED 3 (PB15, Green / System Ready) */
    bool buzzer_state;        /**< State of Buzzer (PB12) */
} status_beacon_state_t;

/**
 * @brief Initialize GPIO peripherals for 1 buzzer (PB12) and 3 LEDs (PB13, PB14, PB15).
 */
void Status_Beacon_Init(void);

/**
 * @brief Set local pulse induction metal detection alarm input.
 * @param detected True if landmine is locally detected.
 */
void Status_Beacon_SetMineAlert(bool detected);

/**
 * @brief Set remote host alert override input from micro-ROS (/beacon/alert).
 * @param active True if remote host requested alarm activation.
 */
void Status_Beacon_SetRemoteAlert(bool active);

/**
 * @brief Control individual LED state directly.
 * @param led_id LED number: 1 (PB13), 2 (PB14), or 3 (PB15).
 * @param state True = ON (HIGH), False = OFF (LOW).
 */
void Status_Beacon_SetLed(uint8_t led_id, bool state);

/**
 * @brief Control audible buzzer directly.
 * @param state True = Sounding, False = Silent.
 */
void Status_Beacon_SetBuzzer(bool state);

/**
 * @brief Convenience function to signal payload/gripper activity on LED 2 (PB14).
 * @param active True if gripper is actively in motion or closed.
 */
void Status_Beacon_SetPayloadActive(bool active);

/**
 * @brief Convenience function to signal system ready & ROS connected on LED 3 (PB15).
 * @param ready True when system is initialized and communication is verified.
 */
void Status_Beacon_SetSystemReady(bool ready);

/**
 * @brief Periodic status beacon update handler for strobe oscillation.
 * @param delta_ms Elapsed time in milliseconds since last update call.
 */
void Status_Beacon_Update(uint32_t delta_ms);

/**
 * @brief Query current composite alarm activation state.
 * @return True if either local mine detected or remote alert is active.
 */
bool Status_Beacon_IsActive(void);

/**
 * @brief Query current local mine alert status.
 * @return True if local mine detected.
 */
bool Status_Beacon_GetMineAlert(void);

/**
 * @brief Query current remote alert status.
 * @return True if remote alert active.
 */
bool Status_Beacon_GetRemoteAlert(void);

/**
 * @brief Query entire driver state snapshot.
 * @param out_state Destination pointer for state struct.
 */
void Status_Beacon_GetState(status_beacon_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_BEACON_DRIVER_H */
