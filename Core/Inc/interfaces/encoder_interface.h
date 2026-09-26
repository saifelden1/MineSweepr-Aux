/**
 * @file encoder_interface.h
 * @brief Common Hardware Interface for Dual Quadrature Wheel Encoders.
 *
 * Provides a pure abstract interface for reading accumulated ticks,
 * incremental deltas, rotational speed (RPM), and linear wheel velocity.
 */

#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_ID_LEFT  = 0,
    ENCODER_ID_RIGHT = 1,
    ENCODER_ID_COUNT = 2
} encoder_id_t;

typedef struct {
    int64_t total_ticks;        /**< Monotonically accumulated quadrature counts */
    int32_t delta_ticks;        /**< Ticks elapsed in previous sampling period */
    float speed_rpm;            /**< Rotational speed in revolutions per minute */
    float linear_velocity_m_s;  /**< Wheel perimeter linear velocity in m/s */
} encoder_data_t;

typedef struct encoder_interface {
    bool (*init)(void);
    void (*update)(uint32_t delta_time_us);
    int64_t (*get_total_ticks)(encoder_id_t id);
    int32_t (*get_delta_ticks)(encoder_id_t id);
    float (*get_linear_velocity)(encoder_id_t id);
    void (*reset)(encoder_id_t id);
} encoder_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_INTERFACE_H */
