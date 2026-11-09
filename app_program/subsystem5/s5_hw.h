/*
 *      s5_hw.h
 *      Subsystem 5 -- the hardware: HC-SR04P on TRIG GP16 / ECHO GP17 and
 *      the SG90 servo on GP15 (nyx, 2026-11-07).
 *
 *      Only the S5 task calls these (s5_sensor.c), so nothing here locks.
 *      Nothing here blocks either: a ping is fire now, read S5_ECHO_READ_MS
 *      later.
 *      The host tests link their own version of these functions (a
 *      simulated sensor on a simulated car), so everything above this
 *      layer runs unchanged on the PC.
 *
 *      Echo timing: GP17 is the B input of PWM slice 0 in level-gated
 *      mode, so the slice counts 1 per microsecond only while ECHO is
 *      high and then holds the pulse width. No interrupt, no busy wait.
 */

#ifndef S5_HW_H
#define S5_HW_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    S5_ECHO_OK = 0, /* a pulse: *p_us is its width */
    S5_ECHO_NONE,   /* the sensor answered and heard nothing: clear */
    S5_ECHO_DEAD,   /* ECHO never rose: no sensor, no 5 V, swapped pins */
    S5_ECHO_BUSY    /* ECHO was still high from before: not fired */
} s5_echo_t;

/* Pins, PWM slices, servo to straight ahead. */
bool s5_hw_init(void);

/* Free-running microseconds (RP2040 TIMER, wraps every 71 minutes: only
 * ever compare two readings by subtracting them). Any task may call it. */
uint32_t s5_hw_us(void);

/* Servo to a logical angle, 0.1 deg, + = left, 0 = straight ahead. Trim
 * and direction (s5_config.h) are applied here; clamped to the bracket. */
void s5_hw_servo(int32_t ddeg);

/* Fire one ping. False if ECHO is still high from the last one. */
bool s5_hw_fire(void);

/* The ping fired last, read at least S5_ECHO_READ_MS later. */
s5_echo_t s5_hw_echo(uint32_t *p_us);

/* nyx, 2026-11-08: Buddy 4's latest magnetometer sample (LSM303D, z up,
 * 0.16 mgauss per count), x and y. False if there is none from the last
 * S5_MAG_AGE_MS. Called from the VehicleController task and the
 * calibration; Buddy 4's mutex is held only to copy the sample. */
bool s5_hw_mag(int32_t *p_x, int32_t *p_y);

#endif /* S5_HW_H */
