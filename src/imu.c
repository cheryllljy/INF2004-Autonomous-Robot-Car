/** @file imu.c @brief LSM303D driver for Grove 3 on the Pico W. */
#include "imu.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <stddef.h>
#include <stdio.h>

#define IMU_I2C       i2c0
#define IMU_SDA_GPIO  4U
#define IMU_SCL_GPIO  5U
#define IMU_I2C_HZ    100000U

/* LSM303D I2C Address (shared by both accelerometer and magnetometer) */
#define LSM303D_ADDR    0x1DU

/* Device Identification */
#define REG_WHO_AM_I    0x0FU
#define LSM303D_ID      0x49U

/* Accelerometer Registers */
#define REG_CTRL1_A     0x20U
#define REG_CTRL2_A     0x21U
#define REG_CTRL3_A     0x22U
#define REG_CTRL4_A     0x23U
#define REG_CTRL5_A     0x24U
#define REG_CTRL6_A     0x25U
#define REG_CTRL7_A     0x26U
#define REG_OUT_X_L_A   0x28U

/* Magnetometer Registers */
#define REG_CTRL1_M     0x20U   /* Shared with accelerometer CTRL1 */
#define REG_CTRL5_M     0x24U   /* Shared with accelerometer CTRL5 */
#define REG_CTRL6_M     0x25U   /* Shared with accelerometer CTRL6 */
#define REG_CTRL7_M     0x26U   /* Shared with accelerometer CTRL7 */
#define REG_OUT_X_L_M   0x08U
#define REG_OUT_X_H_M   0x09U
#define REG_OUT_Y_L_M   0x0AU
#define REG_OUT_Y_H_M   0x0BU
#define REG_OUT_Z_L_M   0x0CU
#define REG_OUT_Z_H_M   0x0DU

static uint8_t g_accel_address;
static bool g_ready;

static bool imu_read_regs(uint8_t address, uint8_t reg, uint8_t *p_data,
                          size_t length, bool auto_increment)
{
    uint8_t register_address = reg;

    if (auto_increment)
    {
        register_address |= 0x80U;
    }
    if (i2c_write_blocking(IMU_I2C, address, &register_address, 1U, true) != 1)
    {
        return false;
    }
    return i2c_read_blocking(IMU_I2C, address, p_data, length, false) ==
           (int)length;
}

static bool imu_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t bytes[2] = { reg, value };
    return i2c_write_blocking(IMU_I2C, address, bytes, sizeof(bytes), false) ==
           (int)sizeof(bytes);
}

void imu_scan_bus(void)
{
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        uint8_t dummy;
        if (i2c_read_blocking(IMU_I2C, addr, &dummy, 1, false) >= 0)
        {
            printf("  Found device at 0x%02X\n", addr);
        }
    }
    printf("Scan complete.\n");
}

imu_status_t imu_init(void)
{
    uint8_t id = 0U;

    g_ready = false;
    g_accel_address = 0U;
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(IMU_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_GPIO);
    gpio_pull_up(IMU_SCL_GPIO);

    printf("I2C0 initialized on SDA=%d, SCL=%d\n", IMU_SDA_GPIO, IMU_SCL_GPIO);

    /* Verify the LSM303D is present */
    if (imu_read_regs(LSM303D_ADDR, REG_WHO_AM_I, &id, 1U, false))
    {
        if (id == LSM303D_ID)
        {
            g_accel_address = LSM303D_ADDR;
        }
        else
        {
            printf("Unexpected WHO_AM_I value: 0x%02X (expected 0x49)\n", id);
        }
    }
    
    if (g_accel_address == 0U)
    {
        return IMU_ACCEL_NOT_FOUND;
    }

    /* ---- Configure Accelerometer ---- */
    /* CTRL1_A (0x20): ODR = 100 Hz, all axes enabled  -> 0x57 */
    if (!imu_write_reg(g_accel_address, REG_CTRL1_A, 0x57U))
    {
        return IMU_I2C_ERROR;
    }

    /* CTRL2_A (0x21): High-pass filter disabled, no reference -> 0x00 */
    if (!imu_write_reg(g_accel_address, REG_CTRL2_A, 0x00U))
    {
        return IMU_I2C_ERROR;
    }

    /* CTRL4_A (0x23): BDU enabled, +/-2 g, high-resolution -> 0x88 */
    if (!imu_write_reg(g_accel_address, REG_CTRL4_A, 0x88U))
    {
        return IMU_I2C_ERROR;
    }

    /* ---- Configure Magnetometer ---- */
    /* CTRL5_M (0x24): Temperature enabled, high-resolution, ODR = 50 Hz -> 0x74 */
    if (!imu_write_reg(g_accel_address, REG_CTRL5_M, 0x74U))
    {
        return IMU_I2C_ERROR;
    }

    /* CTRL6_M (0x25): Full-scale = +/-4 gauss -> 0x20 */
    if (!imu_write_reg(g_accel_address, REG_CTRL6_M, 0x20U))
    {
        return IMU_I2C_ERROR;
    }

    /* CTRL7_M (0x26): Continuous-conversion mode, high-resolution -> 0x00 */
    if (!imu_write_reg(g_accel_address, REG_CTRL7_M, 0x00U))
    {
        return IMU_I2C_ERROR;
    }

    g_ready = true;
    return IMU_OK;
}

imu_status_t imu_read_accel(imu_vector_t *p_sample)
{
    uint8_t bytes[6];
    if (p_sample == NULL)
    {
        return IMU_BAD_PARAM;
    }
    if (!g_ready)
    {
        return IMU_NOT_READY;
    }
    if (!imu_read_regs(g_accel_address, REG_OUT_X_L_A, bytes, sizeof(bytes), true))
    {
        return IMU_I2C_ERROR;
    }
    p_sample->x = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
    p_sample->y = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8U));
    p_sample->z = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8U));
    return IMU_OK;
}

imu_status_t imu_read_mag(imu_vector_t *p_sample)
{
    uint8_t bytes[6];
    if (p_sample == NULL)
    {
        return IMU_BAD_PARAM;
    }
    if (!g_ready)
    {
        return IMU_NOT_READY;
    }
    /* Read from 0x08 (OUT_X_L_M) with auto-increment for 6 bytes */
    if (!imu_read_regs(g_accel_address, REG_OUT_X_L_M, bytes, sizeof(bytes), true))
    {
        return IMU_I2C_ERROR;
    }
    /* LSM303D magnetometer output is little-endian: X, Y, Z */
    p_sample->x = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
    p_sample->y = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8U));
    p_sample->z = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8U));
    return IMU_OK;
}

uint8_t imu_accel_address(void)
{
    return g_ready ? g_accel_address : 0U;
}