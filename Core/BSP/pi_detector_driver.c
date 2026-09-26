/**
 * @file pi_detector_driver.c
 * @brief Concrete BSP Driver implementation for Pulse Induction Metal Detector.
 */

#include "pi_detector_driver.h"
#include <string.h>

extern TIM_HandleTypeDef htim2;

static detector_reading_t s_latest_reading;
static uint32_t s_pulse_end_us = 0;
static uint32_t s_baseline_decay_us = DETECTOR_DEFAULT_BASELINE_US;
static uint32_t s_threshold_us = DETECTOR_DEFAULT_THRESH_US;
static volatile bool s_waiting_edge = false;
static bool s_initialized = false;

static void delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
    {
        /* Busy wait on 1 MHz hardware counter */
    }
}

bool PI_Detector_Init(void)
{
    /* 1. Configure PB0 (Pulse Output) */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DETECTOR_PULSE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(DETECTOR_PULSE_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DETECTOR_PULSE_PORT, DETECTOR_PULSE_PIN, GPIO_PIN_RESET);

    /* 2. Configure PB1 (EXTI1 Input) */
    GPIO_InitStruct.Pin = DETECTOR_ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DETECTOR_ECHO_PORT, &GPIO_InitStruct);

    /* Enable EXTI1 Interrupt */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* 3. Start TIM2 free-running counter (32-bit @ 1 MHz) */
    HAL_TIM_Base_Start(&htim2);

    memset(&s_latest_reading, 0, sizeof(s_latest_reading));
    s_waiting_edge = false;
    s_initialized = true;

    return true;
}

bool PI_Detector_TriggerPulse(void)
{
    if (!s_initialized)
    {
        return false;
    }

    /* Assert excitation pulse HIGH */
    HAL_GPIO_WritePin(DETECTOR_PULSE_PORT, DETECTOR_PULSE_PIN, GPIO_PIN_SET);
    s_waiting_edge = true;

    /* Maintain excitation pulse for configured duration (e.g. 70 us) */
    delay_us(DETECTOR_EXCITATION_US);

    /* Drop excitation LOW -> Flyback decay commences */
    HAL_GPIO_WritePin(DETECTOR_PULSE_PORT, DETECTOR_PULSE_PIN, GPIO_PIN_RESET);
    s_pulse_end_us = __HAL_TIM_GET_COUNTER(&htim2);

    return true;
}

void PI_Detector_OnExtiEdgeCaptured(uint32_t tick_us)
{
    if (!s_waiting_edge)
    {
        return;
    }
    s_waiting_edge = false;

    uint32_t decay_time = tick_us - s_pulse_end_us;

    /* Compute normalized signal intensity [0.0f .. 1.0f] */
    float intensity = 0.0f;
    if (decay_time > s_baseline_decay_us)
    {
        uint32_t diff = decay_time - s_baseline_decay_us;
        uint32_t span = DETECTOR_MAX_DECAY_US - s_baseline_decay_us;
        if (span > 0)
        {
            intensity = (float)diff / (float)span;
            if (intensity > 1.0f) intensity = 1.0f;
            if (intensity < 0.0f) intensity = 0.0f;
        }
    }

    s_latest_reading.decay_time_us = decay_time;
    s_latest_reading.signal_intensity = intensity;
    s_latest_reading.target_detected = (decay_time >= (s_baseline_decay_us + s_threshold_us));
    s_latest_reading.sample_id++;
    s_latest_reading.timestamp_ms = HAL_GetTick();
}

bool PI_Detector_GetLatestReading(detector_reading_t *out_reading)
{
    if (out_reading == NULL)
    {
        return false;
    }

    memcpy(out_reading, &s_latest_reading, sizeof(detector_reading_t));
    return true;
}

void PI_Detector_CalibrateGroundBaseline(uint16_t samples)
{
    if (samples == 0)
    {
        samples = 32;
    }

    uint64_t sum_decay = 0;
    uint16_t valid_count = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        PI_Detector_TriggerPulse();
        HAL_Delay(10); // Wait between excitation pulses

        if (s_latest_reading.decay_time_us > 0)
        {
            sum_decay += s_latest_reading.decay_time_us;
            valid_count++;
        }
    }

    if (valid_count > 0)
    {
        s_baseline_decay_us = (uint32_t)(sum_decay / valid_count);
    }
}

void PI_Detector_SetThresholdUs(uint32_t thresh_us)
{
    s_threshold_us = thresh_us;
}

static const detector_interface_t s_detector_interface = {
    .init                     = PI_Detector_Init,
    .trigger_pulse            = PI_Detector_TriggerPulse,
    .on_exti_edge_captured    = PI_Detector_OnExtiEdgeCaptured,
    .get_latest_reading       = PI_Detector_GetLatestReading,
    .calibrate_ground_baseline= PI_Detector_CalibrateGroundBaseline,
};

const detector_interface_t* PI_Detector_GetInterface(void)
{
    return &s_detector_interface;
}
