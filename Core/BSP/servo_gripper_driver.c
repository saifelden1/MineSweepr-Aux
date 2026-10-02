/**
 * @file servo_gripper_driver.c
 * @brief Concrete BSP Driver implementation for RC Servo Gripper.
 */

#include "servo_gripper_driver.h"

extern TIM_HandleTypeDef htim4;

static gripper_state_t s_current_state = GRIPPER_STATE_ERROR;
static uint16_t s_current_pulse_us = SERVO_PULSE_NEUTRAL_US;
static bool s_initialized = false;

static void set_hardware_pwm(uint16_t pulse_us)
{
    /* Set TIM4 Channel 1 (PB6) and Channel 2 (PB7) */
    __HAL_TIM_SET_COMPARE(&htim4, AUX_GRIPPER_PWM1_CHANNEL, pulse_us);
    __HAL_TIM_SET_COMPARE(&htim4, AUX_GRIPPER_PWM2_CHANNEL, pulse_us);
    s_current_pulse_us = pulse_us;
}

bool Servo_Gripper_Init(void)
{
    /* Start PWM on TIM4 CH1 (PB6) and CH2 (PB7) */
    HAL_TIM_PWM_Start(&htim4, AUX_GRIPPER_PWM1_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, AUX_GRIPPER_PWM2_CHANNEL);

    /* Default to Neutral position (1500 us, State 0) */
    set_hardware_pwm(SERVO_PULSE_NEUTRAL_US);
    s_current_state = GRIPPER_STATE_NEUTRAL;
    s_initialized = true;

    return true;
}

bool Servo_Gripper_Open(void)
{
    if (!s_initialized)
    {
        return false;
    }

    set_hardware_pwm(SERVO_PULSE_OPEN_US);
    s_current_state = GRIPPER_STATE_OPEN;
    return true;
}

bool Servo_Gripper_Grip(void)
{
    if (!s_initialized)
    {
        return false;
    }

    set_hardware_pwm(SERVO_PULSE_GRIP_US);
    s_current_state = GRIPPER_STATE_CLOSED;
    return true;
}

bool Servo_Gripper_Neutral(void)
{
    if (!s_initialized)
    {
        return false;
    }

    set_hardware_pwm(SERVO_PULSE_NEUTRAL_US);
    s_current_state = GRIPPER_STATE_NEUTRAL;
    return true;
}

bool Servo_Gripper_SetState(gripper_state_t state)
{
    switch (state)
    {
        case GRIPPER_STATE_NEUTRAL:
            return Servo_Gripper_Neutral();
        case GRIPPER_STATE_OPEN:
            return Servo_Gripper_Open();
        case GRIPPER_STATE_CLOSED:
            return Servo_Gripper_Grip();
        default:
            return false;
    }
}

bool Servo_Gripper_SetPulseWidthUs(uint16_t pulse_us)
{
    if (!s_initialized)
    {
        return false;
    }

    if (pulse_us < SERVO_PULSE_MIN_US || pulse_us > SERVO_PULSE_MAX_US)
    {
        return false;
    }

    set_hardware_pwm(pulse_us);

    if (pulse_us == SERVO_PULSE_OPEN_US)
    {
        s_current_state = GRIPPER_STATE_OPEN;
    }
    else if (pulse_us == SERVO_PULSE_NEUTRAL_US)
    {
        s_current_state = GRIPPER_STATE_NEUTRAL;
    }
    else if (pulse_us == SERVO_PULSE_GRIP_US)
    {
        s_current_state = GRIPPER_STATE_CLOSED;
    }

    return true;
}

gripper_state_t Servo_Gripper_GetState(void)
{
    return s_current_state;
}

uint16_t Servo_Gripper_GetPulseWidthUs(void)
{
    return s_current_pulse_us;
}

static const gripper_interface_t s_gripper_interface = {
    .init               = Servo_Gripper_Init,
    .open               = Servo_Gripper_Open,
    .grip               = Servo_Gripper_Grip,
    .neutral            = Servo_Gripper_Neutral,
    .set_state          = Servo_Gripper_SetState,
    .set_pulse_width_us = Servo_Gripper_SetPulseWidthUs,
    .get_state          = Servo_Gripper_GetState,
};

const gripper_interface_t* Servo_Gripper_GetInterface(void)
{
    return &s_gripper_interface;
}
