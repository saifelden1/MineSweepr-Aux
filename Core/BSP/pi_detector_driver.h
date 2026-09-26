/**
 * @file pi_detector_driver.h
 * @brief Concrete BSP Driver for Pulse Induction Metal Detector on STM32_Auxiliary.
 *
 * Implements detector_interface_t with coil excitation pulsing on PB0 (50-100 us),
 * flyback decay capture via EXTI1 rising edge on PB1, and microsecond timebase via TIM2 (32-bit @ 1 MHz).
 */

#ifndef PI_DETECTOR_DRIVER_H
#define PI_DETECTOR_DRIVER_H

#include "interfaces/detector_interface.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DETECTOR_PULSE_PORT         GPIOB
#define DETECTOR_PULSE_PIN          GPIO_PIN_0
#define DETECTOR_ECHO_PORT          GPIOB
#define DETECTOR_ECHO_PIN           GPIO_PIN_1

#define DETECTOR_EXCITATION_US      (70U)    /**< Excitation pulse width in microseconds */
#define DETECTOR_DEFAULT_BASELINE_US (20U)   /**< Baseline decay time in clean air/ground */
#define DETECTOR_MAX_DECAY_US       (200U)   /**< Upper limit for normalization scale */
#define DETECTOR_DEFAULT_THRESH_US  (45U)    /**< Threshold above baseline to flag mine */

/**
 * @brief Initialize pulse induction detector hardware (PB0 pulse output, PB1 EXTI, TIM2 counter).
 * @return true on success.
 */
bool PI_Detector_Init(void);

/**
 * @brief Fire a 50-100 us excitation pulse on PB0 and arm EXTI decay capture.
 * @return true on success.
 */
bool PI_Detector_TriggerPulse(void);

/**
 * @brief Callback invoked by EXTI1 interrupt when comparator detects flyback threshold crossing.
 * @param tick_us 32-bit hardware microsecond timestamp at moment of edge detection.
 */
void PI_Detector_OnExtiEdgeCaptured(uint32_t tick_us);

/**
 * @brief Retrieve the most recent metal detector telemetry reading.
 * @param out_reading Destination struct for detector reading.
 * @return true on success.
 */
bool PI_Detector_GetLatestReading(detector_reading_t *out_reading);

/**
 * @brief Perform baseline ground calibration by averaging decay time over N samples.
 * @param samples Number of sample pulses to average.
 */
void PI_Detector_CalibrateGroundBaseline(uint16_t samples);

/**
 * @brief Set detection threshold in microseconds above baseline.
 * @param thresh_us Threshold in microseconds.
 */
void PI_Detector_SetThresholdUs(uint32_t thresh_us);

/**
 * @brief Get the detector_interface_t function pointer table.
 * @return Pointer to detector_interface_t instance.
 */
const detector_interface_t* PI_Detector_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* PI_DETECTOR_DRIVER_H */
