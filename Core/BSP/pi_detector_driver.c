/**
 * @file pi_detector_driver.c
 * @brief Concrete BSP Driver implementation for Pulse Induction Metal Detector.
 */

#include "pi_detector_driver.h"
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

static detector_reading_t s_latest_reading;
static float s_baseline_float = 500.0f;
static uint16_t s_scale_adc = AUX_DETECTOR_DEFAULT_SCALE_ADC;
static uint16_t s_threshold_adc = AUX_DETECTOR_DEFAULT_THRESH_ADC;
static float s_alpha = AUX_DETECTOR_BASELINE_ALPHA;
static bool s_initialized = false;
static bool s_calibrated = false;

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
    /* 1. Configure PB0 (Pulse Output to IRF740 Totem-Pole Driver) */
    AUX_DETECTOR_PULSE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AUX_DETECTOR_PULSE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(AUX_DETECTOR_PULSE_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(AUX_DETECTOR_PULSE_PORT, AUX_DETECTOR_PULSE_PIN, GPIO_PIN_RESET);

    /* 2. Start TIM2 free-running counter (32-bit @ 1 MHz) if not started */
    HAL_TIM_Base_Start(&htim2);

    /* 3. Reset reading structures and filter state */
    memset(&s_latest_reading, 0, sizeof(s_latest_reading));
    s_baseline_float = 500.0f;
    s_scale_adc = AUX_DETECTOR_DEFAULT_SCALE_ADC;
    s_threshold_adc = AUX_DETECTOR_DEFAULT_THRESH_ADC;
    s_alpha = AUX_DETECTOR_BASELINE_ALPHA;
    s_calibrated = false;
    s_initialized = true;

    return true;
}

bool PI_Detector_TriggerPulseAndSample(void)
{
    if (!s_initialized)
    {
        return false;
    }

    /* Critical timing section for microsecond precision */
    __disable_irq();

    /* 1. Fire Gate Excitation Pulse (70 us) */
    HAL_GPIO_WritePin(AUX_DETECTOR_PULSE_PORT, AUX_DETECTOR_PULSE_PIN, GPIO_PIN_SET);
    delay_us(AUX_DETECTOR_PULSE_WIDTH_US);

    /* 2. Turn off MOSFET -> Flyback commence */
    HAL_GPIO_WritePin(AUX_DETECTOR_PULSE_PORT, AUX_DETECTOR_PULSE_PIN, GPIO_PIN_RESET);

    /* 3. Hardware Blanking Interval (20 us) to bypass flyback spike */
    delay_us(AUX_DETECTOR_BLANKING_US);

    /* 4. Rapid 8x ADC1 Oversampling on PA1 */
    uint32_t adc_sum = 0;
    for (uint8_t i = 0; i < AUX_DETECTOR_OVERSAMPLES; i++)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
        {
            adc_sum += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    __enable_irq();

    uint16_t sample_avg = (uint16_t)(adc_sum / AUX_DETECTOR_OVERSAMPLES);

    /* 5. Compute Delta Voltage above Ground Baseline */
    uint16_t baseline_u16 = (uint16_t)s_baseline_float;
    uint16_t delta_v = 0;
    if (sample_avg > baseline_u16)
    {
        delta_v = sample_avg - baseline_u16;
    }

    bool target_detected = (delta_v >= s_threshold_adc);

    /* 6. Baseline Filter: Only adapt baseline when no target is present to prevent signal absorption */
    if (!target_detected || !s_calibrated)
    {
        s_baseline_float = (s_alpha * (float)sample_avg) + ((1.0f - s_alpha) * s_baseline_float);
    }

    /* 7. Normalized Signal Intensity S in [0.0f .. 1.0f] */
    float intensity = 0.0f;
    if (s_scale_adc > 0)
    {
        intensity = (float)delta_v / (float)s_scale_adc;
        if (intensity > 1.0f) intensity = 1.0f;
        if (intensity < 0.0f) intensity = 0.0f;
    }

    /* 8. Record telemetry */
    s_latest_reading.adc_raw_sample = sample_avg;
    s_latest_reading.adc_baseline = (uint16_t)s_baseline_float;
    s_latest_reading.delta_v = delta_v;
    s_latest_reading.signal_intensity = intensity;
    s_latest_reading.target_detected = target_detected;
    s_latest_reading.sample_id++;
    s_latest_reading.timestamp_ms = HAL_GetTick();

    return true;
}

bool PI_Detector_TriggerPulse(void)
{
    return PI_Detector_TriggerPulseAndSample();
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
        samples = 16;
    }

    uint32_t sum = 0;
    uint16_t count = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (PI_Detector_TriggerPulseAndSample())
        {
            sum += s_latest_reading.adc_raw_sample;
            count++;
        }
        HAL_Delay(5); /* 5 ms rest between excitation pulses */
    }

    if (count > 0)
    {
        s_baseline_float = (float)(sum / count);
        s_calibrated = true;
    }
}

void PI_Detector_SetThreshold(uint16_t thresh_adc)
{
    s_threshold_adc = thresh_adc;
}

void PI_Detector_SetScale(uint16_t scale_adc)
{
    if (scale_adc > 0)
    {
        s_scale_adc = scale_adc;
    }
}

static const detector_interface_t s_detector_interface = {
    .init                      = PI_Detector_Init,
    .trigger_pulse_and_sample = PI_Detector_TriggerPulseAndSample,
    .get_latest_reading        = PI_Detector_GetLatestReading,
    .calibrate_ground_baseline = PI_Detector_CalibrateGroundBaseline,
    .set_threshold             = PI_Detector_SetThreshold,
    .set_scale                 = PI_Detector_SetScale,
};

const detector_interface_t* PI_Detector_GetInterface(void)
{
    return &s_detector_interface;
}
