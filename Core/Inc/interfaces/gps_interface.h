/**
 * @file gps_interface.h
 * @brief Common Hardware Interface for Global Positioning System (GPS).
 *
 * Provides a pure abstract interface for feeding raw serial data and
 * retrieving parsed geographic fix parameters (coordinates, altitude, speed).
 */

#ifndef GPS_INTERFACE_H
#define GPS_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double latitude_deg;        /**< Latitude in decimal degrees (+ North, - South) */
    double longitude_deg;       /**< Longitude in decimal degrees (+ East, - West) */
    float altitude_m;           /**< Altitude above mean sea level in meters */
    float ground_speed_m_s;     /**< Speed over ground in m/s */
    float course_deg;           /**< True track course in degrees [0.0 .. 360.0] */
    float hdop;                 /**< Horizontal Dilution of Precision */
    uint8_t satellites_visible; /**< Number of satellites tracked */
    bool fix_valid;             /**< True if valid 2D/3D fix acquired */
    uint32_t timestamp_ms;      /**< System timestamp of last valid update */
} gps_data_t;

typedef struct gps_interface {
    bool (*init)(void);
    void (*feed_buffer)(const uint8_t *buffer, uint16_t len);
    bool (*get_fix)(gps_data_t *out_data);
} gps_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* GPS_INTERFACE_H */
