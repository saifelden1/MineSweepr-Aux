/**
 * @file motor_interface.h
 * @brief Common Hardware Interface for Differential Drive DC Motors.
 *
 * Provides a pure abstract interface for commanding motor speeds, PWM duty,
 * and emergency braking without coupling to timer or GPIO hardware.
 */

#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_ID_LEFT  = 0,
    MOTOR_ID_RIGHT = 1,
    MOTOR_ID_COUNT = 2
} motor_id_t;

typedef enum {
    MOTOR_DIR_REVERSE = -1,
    MOTOR_DIR_BRAKE   =  0,
    MOTOR_DIR_FORWARD =  1
} motor_dir_t;

typedef struct motor_interface {
    bool (*init)(void);
    bool (*set_speed_norm)(motor_id_t motor, float speed_norm); /**< Normalized speed: [-1.0f .. 1.0f] */
    bool (*set_pwm_duty)(motor_id_t motor, motor_dir_t dir, uint16_t duty_ticks);
    bool (*emergency_stop_all)(void);
} motor_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_INTERFACE_H */
