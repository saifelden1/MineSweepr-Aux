/**
 * @file task_detector.h
 * @brief Pulse Induction Metal Detector FreeRTOS Task (20 Hz) for STM32_Auxiliary.
 */

#ifndef TASK_DETECTOR_H
#define TASK_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "interfaces/detector_interface.h"
#include "auxiliary_pin_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DETECTOR_TASK_PERIOD_MS     (50U)   /**< 20 Hz periodic cycle */

/**
 * @brief Initialize metal detector task data, mutexes, and baseline.
 */
void Task_Detector_Init(void);

/**
 * @brief FreeRTOS thread entry point for 20 Hz metal detector excitation and sampling.
 * @param argument Unused.
 */
void StartTaskDetector(void *argument);

/**
 * @brief Thread-safe retrieval of latest metal detector telemetry reading.
 * @param out_reading Destination struct for detector reading.
 * @return true on success, false if invalid or mutex timeout.
 */
bool Task_Detector_GetLatestReading(detector_reading_t *out_reading);

/**
 * @brief Perform baseline ground calibration across N samples.
 * @param samples Sample count for averaging.
 */
void Task_Detector_Calibrate(uint16_t samples);

/**
 * @brief Set anomaly detection threshold in ADC counts above baseline.
 * @param threshold_adc ADC counts threshold.
 */
void Task_Detector_SetThreshold(uint16_t threshold_adc);

/**
 * @brief Query if target anomaly is currently detected.
 * @return true if target detected on latest reading.
 */
bool Task_Detector_IsTargetDetected(void);

/**
 * @brief Query current normalized signal intensity [0.0f .. 1.0f].
 * @return Signal intensity.
 */
float Task_Detector_GetIntensity(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_DETECTOR_H */
