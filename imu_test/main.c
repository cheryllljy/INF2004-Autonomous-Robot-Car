/** @file main.c @brief USB-serial LSM303DLHC hardware bring-up demo. */
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
    /* Do not lose the only startup message before the serial terminal
     * opens. USB CDC uses DTR to report an attached terminal. */
    while (!stdio_usb_connected())
    {
        sleep_ms(100U);
    }
    sleep_ms(100U);
    printf("Buddy 4 LSM303DLHC test: Grove 3, I2C0 GP4/GP5\n");
    
    // NEW: Initialize I2C and scan the bus BEFORE trying to init the driver
    // Note: imu_init() calls i2c_init(), so we need to do a quick init here 
    // just for the scanner, or call imu_init() first and then scan.
    // For simplicity, let's just call imu_init() and if it fails, scan.
    
    status = imu_init();
    if (status != IMU_OK)
    {
        printf("IMU Init failed with status %d. Scanning bus...\n", (int)status);
        imu_scan_bus(); // Run the scanner to help debug
        
        while (true)
        {
            if (status == IMU_ACCEL_NOT_FOUND)
            {
                printf("Accelerometer not found at 0x18 or 0x19. Check Grove 3 wiring and SA0.\n");
            }
            else if (status == IMU_MAG_NOT_FOUND)
            {
                printf("Magnetometer not found at 0x1E. Check module power and Grove 3 SDA/SCL.\n");
            }
            else
            {
                printf("IMU init failed (status %d). Check sensor wiring/identity.\n",
                       (int)status);
            }
            sleep_ms(1000U);
        }
    }

    printf("Accelerometer found at 0x%02X; magnetometer verified at 0x1E.\n",
           (unsigned int)imu_accel_address());
    while (true)
    {
        const imu_status_t accel_status = imu_read_accel(&accel);
        const imu_status_t mag_status = imu_read_mag(&mag);

        if ((accel_status == IMU_OK) && (mag_status == IMU_OK))
        {
            printf("accel raw x=%d y=%d z=%d | mag raw x=%d y=%d z=%d\n",
                   (int)accel.x, (int)accel.y, (int)accel.z,
                   (int)mag.x, (int)mag.y, (int)mag.z);
        }
        else
        {
            printf("read error: accel=%d mag=%d\n",
                   (int)accel_status, (int)mag_status);
        }
        sleep_ms(200U);
    }
}