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

/* Calibration offsets (in raw LSB) */
static int32_t g_offset_x = 0;
static int32_t g_offset_y = 0;
static int32_t g_offset_z = 0;

/* LSM303D I2C Address (shared by both accelerometer and magnetometer) */
#define LSM303D_ADDR    0x1DU

/* Device Identification */
#define REG_WHO_AM_I    0x0FU
#define LSM303D_ID      0x49U

/* Accelerometer Registers */
#define REG_CTRL1_A     0x20U
#define REG_CTRL2_A     0x21U
#define REG_CTRL4_A     0x23U
#define REG_OUT_X_L_A   0x28U

/* Magnetometer Registers */
#define REG_CTRL5_M     0x24U
#define REG_CTRL6_M     0x25U
#define REG_CTRL7_M     0x26U
#define REG_OUT_X_L_M   0x08U

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

    for (int retry = 0; retry < 3; retry++)
    {
        if (i2c_write_blocking(IMU_I2C, address, &register_address, 1U, true) != 1)
        {
            sleep_ms(1);
            continue;
        }
        if (i2c_read_blocking(IMU_I2C, address, p_data, length, false) == (int)length)
        {
            return true;
        }
        sleep_ms(1);
    }
    return false;
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

    /* Accelerometer: 100 Hz, all axes, +/-2g, high-resolution, BDU */
    if (!imu_write_reg(g_accel_address, REG_CTRL1_A, 0x57U)) return IMU_I2C_ERROR;
    if (!imu_write_reg(g_accel_address, REG_CTRL2_A, 0x00U)) return IMU_I2C_ERROR;
    if (!imu_write_reg(g_accel_address, REG_CTRL4_A, 0x88U)) return IMU_I2C_ERROR;

    /* Magnetometer: 50 Hz, high-res, +/-4 gauss, continuous */
    if (!imu_write_reg(g_accel_address, REG_CTRL5_M, 0x74U)) return IMU_I2C_ERROR;
    if (!imu_write_reg(g_accel_address, REG_CTRL6_M, 0x20U)) return IMU_I2C_ERROR;
    if (!imu_write_reg(g_accel_address, REG_CTRL7_M, 0x00U)) return IMU_I2C_ERROR;

    g_ready = true;
    return IMU_OK;
}

void imu_calibrate(void)
{
    imu_vector_t sample;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_samples = 0;
    const int num_samples = 100;

    printf("Calibrating IMU... Keep the sensor still and flat.\n");
    sleep_ms(1000);

    for (int i = 0; i < num_samples; i++)
    {
        if (imu_read_accel(&sample) == IMU_OK)
        {
            sum_x += sample.x;
            sum_y += sample.y;
            sum_z += sample.z;
            valid_samples++;
        }
        sleep_ms(20);
    }

    if (valid_samples == 0)
    {
        printf("Calibration failed: no valid samples.\n");
        return;
    }

    g_offset_x = sum_x / valid_samples;
    g_offset_y = sum_y / valid_samples;
    g_offset_z = (sum_z / valid_samples) + 16384;

    printf("Calibration complete: %d/%d samples. Offsets: X=%ld Y=%ld Z=%ld\n",
           valid_samples, num_samples,
           (long)g_offset_x, (long)g_offset_y, (long)g_offset_z);
}

imu_status_t imu_read_accel(imu_vector_t *p_sample)
{
    uint8_t bytes[6];
    if (p_sample == NULL) return IMU_BAD_PARAM;
    if (!g_ready) return IMU_NOT_READY;
    if (!imu_read_regs(g_accel_address, REG_OUT_X_L_A, bytes, sizeof(bytes), true))
        return IMU_I2C_ERROR;

    int16_t raw_x = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
    int16_t raw_y = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8U));
    int16_t raw_z = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8U));

    p_sample->x = (int16_t)((int32_t)raw_x - g_offset_x);
    p_sample->y = (int16_t)((int32_t)raw_y - g_offset_y);
    p_sample->z = (int16_t)((int32_t)raw_z - g_offset_z);

    return IMU_OK;
}

imu_status_t imu_read_mag(imu_vector_t *p_sample)
{
    uint8_t bytes[6];
    if (p_sample == NULL) return IMU_BAD_PARAM;
    if (!g_ready) return IMU_NOT_READY;
    if (!imu_read_regs(g_accel_address, REG_OUT_X_L_M, bytes, sizeof(bytes), true))
        return IMU_I2C_ERROR;

    p_sample->x = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
    p_sample->y = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8U));
    p_sample->z = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8U));
    return IMU_OK;
}

uint8_t imu_accel_address(void)
{
    return g_ready ? g_accel_address : 0U;
}