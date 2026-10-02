/**
 * @file task_gripper.h
 * @brief RC Servo Gripper & Payload Control FreeRTOS Task for STM32_Auxiliary.
 */

#ifndef TASK_GRIPPER_H
#define TASK_GRIPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "interfaces/gripper_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRIPPER_CMD_TYPE_PRESET_NEUTRAL = 0,
    GRIPPER_CMD_TYPE_PRESET_OPEN    = 1,
    GRIPPER_CMD_TYPE_PRESET_GRIP    = 2,
    GRIPPER_CMD_TYPE_CUSTOM_PULSE   = 3,
} gripper_cmd_type_t;

typedef struct {
    gripper_cmd_type_t type;
    uint16_t pulse_us;
} gripper_command_msg_t;

/**
 * @brief Initialize the gripper task data structures, command queue, and mutex.
 */
void Task_Gripper_Init(void);

/**
 * @brief FreeRTOS thread entry point for event-driven gripper servo positioning.
 * @param argument Unused.
 */
void StartTaskGripper(void *argument);

/**
 * @brief Send a gripper command into the FreeRTOS event queue.
 * @param cmd Value representing discrete state (0=neutral, 1=open, 2=grip) or direct pulse in [1000..2000] us.
 * @return true if command was enqueued successfully.
 */
bool Task_Gripper_SendCommand(int32_t cmd);

/**
 * @brief Thread-safe query of current gripper state as int8_t for ROS 2 /gripper/status.
 * @return 0 = Neutral, 1 = Open, 2 = Grip.
 */
int8_t Task_Gripper_GetStatus(void);

/**
 * @brief Thread-safe query of current gripper mechanical state enum.
 * @return GRIPPER_STATE_OPEN, GRIPPER_STATE_NEUTRAL, GRIPPER_STATE_CLOSED, or GRIPPER_STATE_ERROR.
 */
gripper_state_t Task_Gripper_GetState(void);

/**
 * @brief Thread-safe query of current active servo PWM pulse width in microseconds.
 * @return Current pulse width in microseconds [1000 .. 2000].
 */
uint16_t Task_Gripper_GetPulseWidthUs(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_GRIPPER_H */
