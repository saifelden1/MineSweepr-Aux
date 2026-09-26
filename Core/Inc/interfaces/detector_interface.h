/**
 * @file detector_interface.h
 * @brief Common Hardware Interface for Inductive Pulse Induction Metal Detector.
 *
 * Provides a pure abstract interface for coil excitation pulsing,
 * EXTI edge timestamp capture, ground baseline calibration, and reading telemetry.
 */

#ifndef DETECTOR_INTERFACE_H
#define DETECTOR_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float signal_intensity; /**< Normalized coil response intensity [0.0f .. 1.0f] */
    uint32_t decay_time_us; /**< Measured flyback decay duration in microseconds */
    bool target_detected;   /**< True if threshold exceeded (metal target present) */
    uint32_t sample_id;     /**< Monotonically increasing sample sequence counter */
    uint32_t timestamp_ms;  /**< Timestamp of sample acquisition */
} detector_reading_t;

typedef struct detector_interface {
    bool (*init)(void);
    bool (*trigger_pulse)(void);
    void (*on_exti_edge_captured)(uint32_t tick_us);
    bool (*get_latest_reading)(detector_reading_t *out_reading);
    void (*calibrate_ground_baseline)(uint16_t samples);
} detector_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* DETECTOR_INTERFACE_H */
