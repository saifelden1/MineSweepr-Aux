/**
 * @file pi_detector_driver.h
 * @brief Concrete BSP Driver for Pulse Induction Metal Detector on STM32_Auxiliary.
 *
 * Implements detector_interface_t with:
 *   - 70 us excitation gate pulse on PB0 (driving IRF740 power MOSFET).
 *   - 20 us blanking window immediately following pulse turn-off.
 *   - 8x rapid oversampling of TL072 analog decay tail via ADC1 on PA1.
 *   - Exponential Moving Average (EMA) baseline filter tracking soil drift.
 *   - Normalized output intensity S in [0.0f .. 1.0f].
 */

#ifndef PI_DETECTOR_DRIVER_H
#define PI_DETECTOR_DRIVER_H

#include "interfaces/detector_interface.h"
#include "auxiliary_pin_config.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize pulse induction detector hardware (PB0 pulse output, TIM2 timebase).
 * @return true on success.
 */
bool PI_Detector_Init(void);

/**
 * @brief Execute a 70 us gate pulse, wait 20 us blanking window, and sample ADC1 over 8 conversions.
 * @return true on success.
 */
bool PI_Detector_TriggerPulseAndSample(void);

/**
 * @brief Alias for PI_Detector_TriggerPulseAndSample for interface compatibility.
 * @return true on success.
 */
bool PI_Detector_TriggerPulse(void);

/**
 * @brief Retrieve the most recent metal detector telemetry reading.
 * @param out_reading Destination struct for detector reading.
 * @return true on success.
 */
bool PI_Detector_GetLatestReading(detector_reading_t *out_reading);

/**
 * @brief Perform baseline ground calibration by acquiring and averaging N samples.
 * @param samples Number of calibration samples to average.
 */
void PI_Detector_CalibrateGroundBaseline(uint16_t samples);

/**
 * @brief Set detection threshold in ADC LSB counts above baseline.
 * @param thresh_adc ADC counts above baseline to flag mine.
 */
void PI_Detector_SetThreshold(uint16_t thresh_adc);

/**
 * @brief Set ADC normalization full-scale span.
 * @param scale_adc ADC counts representing 1.0f signal intensity.
 */
void PI_Detector_SetScale(uint16_t scale_adc);

/**
 * @brief Get the detector_interface_t function pointer table.
 * @return Pointer to detector_interface_t instance.
 */
const detector_interface_t* PI_Detector_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* PI_DETECTOR_DRIVER_H */
