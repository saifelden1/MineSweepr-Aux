/**
 * @file detector_interface.h
 * @brief Common Hardware Interface for Inductive Pulse Induction Metal Detector.
 *
 * Provides an abstract interface for coil excitation pulsing,
 * analog decay tail sampling via ADC1, ground baseline calibration,
 * and reading telemetry.
 */

#ifndef DETECTOR_INTERFACE_H
#define DETECTOR_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    signal_intensity; /**< Normalized coil response intensity [0.0f .. 1.0f] */
    uint16_t adc_raw_sample;   /**< Measured oversampled ADC1 value (0..4095) */
    uint16_t adc_baseline;     /**< Running EMA ground baseline ADC value */
    uint16_t delta_v;          /**< Raw sample minus baseline (clamped >= 0) */
    bool     target_detected;  /**< True if threshold exceeded (metal target present) */
    uint32_t sample_id;        /**< Monotonically increasing sample sequence counter */
    uint32_t timestamp_ms;     /**< Timestamp of sample acquisition */
} detector_reading_t;

typedef struct detector_interface {
    bool (*init)(void);
    bool (*trigger_pulse_and_sample)(void);
    bool (*get_latest_reading)(detector_reading_t *out_reading);
    void (*calibrate_ground_baseline)(uint16_t samples);
    void (*set_threshold)(uint16_t thresh_adc);
    void (*set_scale)(uint16_t scale_adc);
} detector_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* DETECTOR_INTERFACE_H */
