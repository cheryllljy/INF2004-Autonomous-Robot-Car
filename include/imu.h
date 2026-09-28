/**
 * @file imu.h
 * @brief LSM303DLHC accelerometer and magnetometer driver.
 *
 * The LSM303DLHC contains no gyroscope. Samples are returned as the
 * signed raw register values; calibration and unit conversion belong
 * to the application.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_vector_t;

typedef enum
{
    IMU_OK = 0,
    IMU_BAD_PARAM,
    IMU_NOT_READY,
    IMU_I2C_ERROR,
    IMU_ACCEL_NOT_FOUND,
    IMU_MAG_NOT_FOUND,
    IMU_BAD_ID
} imu_status_t;

/** Configure Grove 3 / I2C0 and identify both LSM303DLHC dies. */
imu_status_t imu_init(void);

/** Scan the I2C bus and print found addresses. Useful for debugging. */
void imu_scan_bus(void);

/** Read one XYZ accelerometer sample (raw 12-bit left-justified values). */
imu_status_t imu_read_accel(imu_vector_t *p_sample);

/** Read one XYZ magnetometer sample (raw signed register values). */
imu_status_t imu_read_mag(imu_vector_t *p_sample);

/** Return the detected accelerometer address, or 0 if not initialized. */
uint8_t imu_accel_address(void);

#endif /* IMU_H */