/**
 * @file imu.h
 * @brief LSM303D accelerometer and magnetometer driver for Grove 3.
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

/** Configure Grove 3 / I2C0 and identify the LSM303D. */
imu_status_t imu_init(void);

/** Calibrate the accelerometer by averaging stationary samples. */
void imu_calibrate(void);

/** Scan the I2C bus and print found addresses. Useful for debugging. */
void imu_scan_bus(void);

/** Read one XYZ accelerometer sample (calibrated). */
imu_status_t imu_read_accel(imu_vector_t *p_sample);

/** Read one XYZ magnetometer sample (raw). */
imu_status_t imu_read_mag(imu_vector_t *p_sample);

/** Return the detected accelerometer address, or 0 if not initialized. */
uint8_t imu_accel_address(void);

#endif /* IMU_H */