/**
 * @file gripper_interface.h
 * @brief Common Hardware Interface for RC Servo Gripper End-Effector.
 *
 * Provides a pure abstract interface for discrete states:
 *   - 0: Neutral (1500 us)
 *   - 1: Open    (1000 us)
 *   - 2: Grip    (2000 us)
 */

#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRIPPER_STATE_NEUTRAL = 0, /**< 1.5 ms pulse (1500 us) */
    GRIPPER_STATE_OPEN    = 1, /**< 1.0 ms pulse (1000 us) */
    GRIPPER_STATE_CLOSED  = 2, /**< 2.0 ms pulse (2000 us) */
    GRIPPER_STATE_ERROR   = 3  /**< Uninitialized or invalid state */
} gripper_state_t;

typedef struct gripper_interface {
    bool (*init)(void);
    bool (*open)(void);                   /**< Move to open position (1.0 ms pulse) */
    bool (*grip)(void);                   /**< Move to grip/closed position (2.0 ms pulse) */
    bool (*neutral)(void);                /**< Move to neutral position (1.5 ms pulse) */
    bool (*set_state)(gripper_state_t state); /**< Set discrete state 0, 1, or 2 */
    bool (*set_pulse_width_us)(uint16_t pulse_us); /**< Direct pulse width [1000 .. 2000 us] */
    gripper_state_t (*get_state)(void);
} gripper_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_INTERFACE_H */
