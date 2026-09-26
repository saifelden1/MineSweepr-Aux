/**
 * @file imu_interface.h
 * @brief Common Hardware Interface for 6-DOF Inertial Measurement Unit (IMU).
 *
 * Provides a pure abstract interface for reading calibrated linear acceleration,
 * angular velocity, temperature, and fused orientation quaternion without coupling
 * to hardware-specific registers or HAL libraries.
 */

#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_x_m_s2;     /**< Linear acceleration along X axis in m/s^2 */
    float accel_y_m_s2;     /**< Linear acceleration along Y axis in m/s^2 */
    float accel_z_m_s2;     /**< Linear acceleration along Z axis in m/s^2 */
    float gyro_x_rad_s;     /**< Angular velocity about X axis in rad/s */
    float gyro_y_rad_s;     /**< Angular velocity about Y axis in rad/s */
    float gyro_z_rad_s;     /**< Angular velocity about Z axis in rad/s */
    float temp_deg_c;       /**< Temperature in degrees Celsius */
    float orientation_quat[4]; /**< Orientation quaternion [w, x, y, z] */
    uint32_t timestamp_ms;  /**< Sample timestamp in milliseconds */
} imu_data_t;

typedef struct {
    int16_t accel_x_raw;    /**< Raw ADC count for X acceleration */
    int16_t accel_y_raw;    /**< Raw ADC count for Y acceleration */
    int16_t accel_z_raw;    /**< Raw ADC count for Z acceleration */
    int16_t gyro_x_raw;     /**< Raw ADC count for X angular rate */
    int16_t gyro_y_raw;     /**< Raw ADC count for Y angular rate */
    int16_t gyro_z_raw;     /**< Raw ADC count for Z angular rate */
    int16_t temp_raw;       /**< Raw ADC count for internal temperature */
} imu_raw_data_t;

typedef struct imu_interface {
    bool (*init)(void);
    bool (*read_calibrated)(imu_data_t *data);
    bool (*calibrate_bias)(uint16_t samples);
} imu_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* IMU_INTERFACE_H */
