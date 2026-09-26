/**
 * @file servo_gripper_driver.h
 * @brief Concrete BSP Driver for RC Servo Gripper on STM32_Auxiliary.
 *
 * Implements gripper_interface_t using TIM4_CH1 (PB6) generating 50 Hz PWM
 * with 1.0 us tick resolution (PSC=95, ARR=19999 @ 96 MHz timer clock).
 */

#ifndef SERVO_GRIPPER_DRIVER_H
#define SERVO_GRIPPER_DRIVER_H

#include "interfaces/gripper_interface.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_PWM_ARR_TICKS         (19999U) /**< 20 ms period @ 1 MHz */
#define SERVO_PULSE_OPEN_US         (1000U)  /**< 1.0 ms pulse width */
#define SERVO_PULSE_NEUTRAL_US      (1500U)  /**< 1.5 ms pulse width */
#define SERVO_PULSE_GRIP_US         (2000U)  /**< 2.0 ms pulse width */

#define SERVO_PULSE_MIN_US          (1000U)
#define SERVO_PULSE_MAX_US          (2000U)

/**
 * @brief Initialize TIM4 CH1 PWM for servo control.
 * @return true on success.
 */
bool Servo_Gripper_Init(void);

/**
 * @brief Move gripper to Open position (1000 us pulse).
 * @return true on success.
 */
bool Servo_Gripper_Open(void);

/**
 * @brief Move gripper to Grip/Closed position (2000 us pulse).
 * @return true on success.
 */
bool Servo_Gripper_Grip(void);

/**
 * @brief Move gripper to Neutral position (1500 us pulse).
 * @return true on success.
 */
bool Servo_Gripper_Neutral(void);

/**
 * @brief Set explicit pulse width in microseconds [1000 .. 2000 us].
 * @param pulse_us Pulse duration in microseconds.
 * @return true on success, false if parameter out of range.
 */
bool Servo_Gripper_SetPulseWidthUs(uint16_t pulse_us);

/**
 * @brief Retrieve current gripper state enum.
 * @return GRIPPER_STATE_OPEN, GRIPPER_STATE_NEUTRAL, GRIPPER_STATE_CLOSED, or GRIPPER_STATE_ERROR.
 */
gripper_state_t Servo_Gripper_GetState(void);

/**
 * @brief Get the gripper_interface_t function pointer table.
 * @return Pointer to gripper_interface_t instance.
 */
const gripper_interface_t* Servo_Gripper_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_GRIPPER_DRIVER_H */
