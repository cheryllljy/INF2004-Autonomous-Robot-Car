/** @file main.c @brief USB-serial LSM303D hardware bring-up demo. */
#include "imu.h"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include <stdio.h>

int main(void)
{
    imu_vector_t accel;
    imu_vector_t mag;
    imu_status_t status;

    stdio_init_all();
    while (!stdio_usb_connected())
    {
        sleep_ms(100U);
    }
    sleep_ms(100U);
    printf("Buddy 4 LSM303D test: Grove 3, I2C0 GP4/GP5\n");

    status = imu_init();
    if (status != IMU_OK)
    {
        printf("IMU Init failed with status %d. Scanning bus...\n", (int)status);
        imu_scan_bus();

        while (true)
        {
            printf("IMU init failed (status %d).\n", (int)status);
            sleep_ms(1000U);
        }
    }

    printf("Sensor found at 0x%02X.\n", (unsigned int)imu_accel_address());

    imu_calibrate();

    while (true)
    {
        const imu_status_t accel_status = imu_read_accel(&accel);
        const imu_status_t mag_status = imu_read_mag(&mag);

        if ((accel_status == IMU_OK) && (mag_status == IMU_OK))
        {
            printf("accel x=%d y=%d z=%d | mag x=%d y=%d z=%d\n",
                   (int)accel.x, (int)accel.y, (int)accel.z,
                   (int)mag.x, (int)mag.y, (int)mag.z);
        }
        else
        {
            static int error_count = 0;
            if (++error_count % 10 == 0)
            {
                printf("read error: accel=%d mag=%d\n",
                       (int)accel_status, (int)mag_status);
            }
        }
        sleep_ms(100U);
    }
}