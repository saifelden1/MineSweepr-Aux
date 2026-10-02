/**
 * @file task_detector.c
 * @brief Pulse Induction Metal Detector 20 Hz Task implementation.
 */

#include "task_detector.h"
#include "pi_detector_driver.h"
#include "status_beacon_driver.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <string.h>

static detector_reading_t s_safe_reading;
static osMutexId_t s_detector_mutex = NULL;
static const osMutexAttr_t s_detector_mutex_attr = {
    .name = "DetectorMutex"
};

static uint32_t s_heartbeat_counter = 0;

void Task_Detector_Init(void)
{
    if (s_detector_mutex == NULL)
    {
        s_detector_mutex = osMutexNew(&s_detector_mutex_attr);
    }

    memset(&s_safe_reading, 0, sizeof(s_safe_reading));

    /* Initialize diagnostic heartbeat pin PC13 */
    AUX_HEARTBEAT_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AUX_HEARTBEAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AUX_HEARTBEAT_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(AUX_HEARTBEAT_PORT, AUX_HEARTBEAT_PIN, GPIO_PIN_RESET);
}

void StartTaskDetector(void *argument)
{
    (void)argument;

    Task_Detector_Init();

    /* Optional initial baseline ground calibration (16 samples) */
    PI_Detector_CalibrateGroundBaseline(16);

    uint32_t last_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        /* 1. Fire 70 us excitation pulse on PB0, wait 20 us blanking, sample ADC1 PA1 */
        PI_Detector_TriggerPulseAndSample();

        /* 2. Retrieve latest reading */
        detector_reading_t current_sample = {0};
        if (PI_Detector_GetLatestReading(&current_sample))
        {
            if (s_detector_mutex != NULL && osMutexAcquire(s_detector_mutex, 10) == osOK)
            {
                memcpy(&s_safe_reading, &current_sample, sizeof(detector_reading_t));
                osMutexRelease(s_detector_mutex);
            }

            /* 3. Forward detection status to Status Beacon */
            Status_Beacon_SetMineAlert(current_sample.target_detected);
        }

        /* 4. Update Status Beacon strobe timing */
        Status_Beacon_Update(DETECTOR_TASK_PERIOD_MS);

        /* 5. Heartbeat indicator on PC13 every ~1 second (every 20 cycles) */
        if (++s_heartbeat_counter >= 20)
        {
            s_heartbeat_counter = 0;
            HAL_GPIO_TogglePin(AUX_HEARTBEAT_PORT, AUX_HEARTBEAT_PIN);
        }

        /* 6. Enforce deterministic 20 Hz (50 ms) Rate-Monotonic timing */
        last_wake_tick += DETECTOR_TASK_PERIOD_MS;
        uint32_t now = osKernelGetTickCount();
        if (now < last_wake_tick)
        {
            osDelay(last_wake_tick - now);
        }
        else
        {
            last_wake_tick = now;
            osDelay(1);
        }
    }
}

bool Task_Detector_GetLatestReading(detector_reading_t *out_reading)
{
    if (out_reading == NULL)
    {
        return false;
    }

    if (s_detector_mutex != NULL && osMutexAcquire(s_detector_mutex, 10) == osOK)
    {
        memcpy(out_reading, &s_safe_reading, sizeof(detector_reading_t));
        osMutexRelease(s_detector_mutex);
        return true;
    }

    /* Fallback if mutex unacquired */
    return PI_Detector_GetLatestReading(out_reading);
}

void Task_Detector_Calibrate(uint16_t samples)
{
    PI_Detector_CalibrateGroundBaseline(samples);
}

void Task_Detector_SetThreshold(uint16_t threshold_adc)
{
    PI_Detector_SetThreshold(threshold_adc);
}

bool Task_Detector_IsTargetDetected(void)
{
    bool detected = false;
    if (s_detector_mutex != NULL && osMutexAcquire(s_detector_mutex, 5) == osOK)
    {
        detected = s_safe_reading.target_detected;
        osMutexRelease(s_detector_mutex);
    }
    else
    {
        detected = s_safe_reading.target_detected;
    }
    return detected;
}

float Task_Detector_GetIntensity(void)
{
    float intensity = 0.0f;
    if (s_detector_mutex != NULL && osMutexAcquire(s_detector_mutex, 5) == osOK)
    {
        intensity = s_safe_reading.signal_intensity;
        osMutexRelease(s_detector_mutex);
    }
    else
    {
        intensity = s_safe_reading.signal_intensity;
    }
    return intensity;
}
