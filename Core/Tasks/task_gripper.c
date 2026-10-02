/**
 * @file task_gripper.c
 * @brief RC Servo Gripper & Payload Control FreeRTOS Task implementation.
 */

#include "task_gripper.h"
#include "servo_gripper_driver.h"
#include "cmsis_os.h"
#include <string.h>

#define GRIPPER_QUEUE_CAPACITY (8U)

static osMessageQueueId_t s_gripper_queue = NULL;
static const osMessageQueueAttr_t s_gripper_queue_attr = {
    .name = "GripperCmdQueue"
};

static osMutexId_t s_gripper_mutex = NULL;
static const osMutexAttr_t s_gripper_mutex_attr = {
    .name = "GripperMutex"
};

static uint16_t s_current_pulse_us = SERVO_PULSE_NEUTRAL_US;
static gripper_state_t s_cached_state = GRIPPER_STATE_NEUTRAL;

void Task_Gripper_Init(void)
{
    if (s_gripper_mutex == NULL)
    {
        s_gripper_mutex = osMutexNew(&s_gripper_mutex_attr);
    }
    if (s_gripper_queue == NULL)
    {
        s_gripper_queue = osMessageQueueNew(GRIPPER_QUEUE_CAPACITY, sizeof(gripper_command_msg_t), &s_gripper_queue_attr);
    }

    s_current_pulse_us = SERVO_PULSE_NEUTRAL_US;
    s_cached_state = GRIPPER_STATE_NEUTRAL;
}

void StartTaskGripper(void *argument)
{
    (void)argument;

    Task_Gripper_Init();
    Servo_Gripper_Init();

    gripper_command_msg_t msg;

    for (;;)
    {
        /* Event-driven wait on command queue */
        if (osMessageQueueGet(s_gripper_queue, &msg, NULL, osWaitForever) == osOK)
        {
            if (s_gripper_mutex != NULL)
            {
                osMutexAcquire(s_gripper_mutex, osWaitForever);
            }

            switch (msg.type)
            {
                case GRIPPER_CMD_TYPE_PRESET_OPEN:
                    Servo_Gripper_Open();
                    s_current_pulse_us = SERVO_PULSE_OPEN_US;
                    s_cached_state = GRIPPER_STATE_OPEN;
                    break;

                case GRIPPER_CMD_TYPE_PRESET_GRIP:
                    Servo_Gripper_Grip();
                    s_current_pulse_us = SERVO_PULSE_GRIP_US;
                    s_cached_state = GRIPPER_STATE_CLOSED;
                    break;

                case GRIPPER_CMD_TYPE_PRESET_NEUTRAL:
                    Servo_Gripper_Neutral();
                    s_current_pulse_us = SERVO_PULSE_NEUTRAL_US;
                    s_cached_state = GRIPPER_STATE_NEUTRAL;
                    break;

                case GRIPPER_CMD_TYPE_CUSTOM_PULSE:
                    if (msg.pulse_us >= SERVO_PULSE_MIN_US && msg.pulse_us <= SERVO_PULSE_MAX_US)
                    {
                        Servo_Gripper_SetPulseWidthUs(msg.pulse_us);
                        s_current_pulse_us = msg.pulse_us;
                        s_cached_state = Servo_Gripper_GetState();
                    }
                    break;

                default:
                    break;
            }

            if (s_gripper_mutex != NULL)
            {
                osMutexRelease(s_gripper_mutex);
            }
        }
    }
}

bool Task_Gripper_SendCommand(int32_t cmd)
{
    if (s_gripper_queue == NULL)
    {
        return false;
    }

    gripper_command_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    if (cmd == 0 || cmd == (int32_t)GRIPPER_STATE_NEUTRAL)
    {
        msg.type = GRIPPER_CMD_TYPE_PRESET_NEUTRAL;
        msg.pulse_us = SERVO_PULSE_NEUTRAL_US;
    }
    else if (cmd == 1 || cmd == (int32_t)GRIPPER_STATE_OPEN)
    {
        msg.type = GRIPPER_CMD_TYPE_PRESET_OPEN;
        msg.pulse_us = SERVO_PULSE_OPEN_US;
    }
    else if (cmd == 2 || cmd == (int32_t)GRIPPER_STATE_CLOSED)
    {
        msg.type = GRIPPER_CMD_TYPE_PRESET_GRIP;
        msg.pulse_us = SERVO_PULSE_GRIP_US;
    }
    else if (cmd >= SERVO_PULSE_MIN_US && cmd <= SERVO_PULSE_MAX_US)
    {
        msg.type = GRIPPER_CMD_TYPE_CUSTOM_PULSE;
        msg.pulse_us = (uint16_t)cmd;
    }
    else
    {
        /* Invalid command value */
        return false;
    }

    return (osMessageQueuePut(s_gripper_queue, &msg, 0, 10) == osOK);
}

int8_t Task_Gripper_GetStatus(void)
{
    gripper_state_t state = Task_Gripper_GetState();
    switch (state)
    {
        case GRIPPER_STATE_NEUTRAL:
            return 0;
        case GRIPPER_STATE_OPEN:
            return 1;
        case GRIPPER_STATE_CLOSED:
            return 2;
        default:
            return 0;
    }
}

gripper_state_t Task_Gripper_GetState(void)
{
    gripper_state_t state = GRIPPER_STATE_NEUTRAL;
    if (s_gripper_mutex != NULL && osMutexAcquire(s_gripper_mutex, 10) == osOK)
    {
        state = s_cached_state;
        osMutexRelease(s_gripper_mutex);
    }
    else
    {
        state = Servo_Gripper_GetState();
    }
    return state;
}

uint16_t Task_Gripper_GetPulseWidthUs(void)
{
    uint16_t pulse = SERVO_PULSE_NEUTRAL_US;
    if (s_gripper_mutex != NULL && osMutexAcquire(s_gripper_mutex, 10) == osOK)
    {
        pulse = s_current_pulse_us;
        osMutexRelease(s_gripper_mutex);
    }
    else
    {
        pulse = s_current_pulse_us;
    }
    return pulse;
}
