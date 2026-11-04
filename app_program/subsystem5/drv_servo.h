/**
 * @file    drv_servo.h
 * @brief   SG90 servo on one RP2040 PWM channel (GP15, slice 7 B).
 *
 * Driver layer: knows the hardware, knows nothing about obstacles.
 * Never blocks, never allocates. Called only from the obstacle task,
 * the owner of PWM slice 7 (TEAM.md section 3).
 *
 * Angle convention:  0 deg = full right, 90 = ahead, 180 = full left.
 * If the horn is mounted the other way round, set DRV_SERVO_INVERT to 1
 * in drv_servo.c. Do not flip angles at the call sites.
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdbool.h>
#include <stdint.h>

/** Hard limits: every command is clamped here, so no code path can
 *  drive the bracket into the Pico board (v2.2: 60..120 = +/-30 deg,
 *  bench 2026-11-01). A stalled SG90 also draws ~700 mA and can brown
 *  out the 5 V servo rail the sonar shares. */
#define DRV_SERVO_MIN_DEG           (60)
#define DRV_SERVO_MAX_DEG           (120)
#define DRV_SERVO_CENTRE_DEG        (90)

/**
 * v3.0 correction, pure so the host tests run the same maths: logical
 * angle in 0.1 deg (900 = straight ahead) to the physical command. Adds
 * the trim, and leads the backlash in the direction of travel so the
 * horn stops where it was asked to. dir: +1 moving left (up), -1 right,
 * 0 never moved.
 */
static inline int32_t drv_servo_correct_ddeg(int32_t angle_ddeg, int8_t dir,
                                             int16_t trim_ddeg,
                                             int16_t backlash_ddeg)
{
    int32_t ddeg = angle_ddeg + (int32_t)trim_ddeg;

    if (dir > 0)
    {
        ddeg += (int32_t)backlash_ddeg;
    }
    else if (dir < 0)
    {
        ddeg -= (int32_t)backlash_ddeg;
    }
    else
    {
        /* Never moved yet: no lead. */
    }

    return ddeg;
}

/** Configure PWM slice 7 for 50 Hz, 1 us per count, centre the horn.
 *  @return false if the clock cannot give an exact 1 MHz count. */
bool drv_servo_init(void);

/** Command an absolute angle, clamped to the limits.
 *  v3.0: the calibrated trim and backlash lead (s5_calib.h) are applied
 *  here, so every caller's 90 deg really is straight ahead.
 *  @return false if it had to be clamped. */
bool drv_servo_set_angle(int16_t angle_deg);

/** Calibration only: command a PHYSICAL angle in 0.1 deg, no trim and
 *  no backlash lead, still clamped to the bracket limits. */
bool drv_servo_set_raw_ddeg(int16_t ddeg);

/** v3.0: logical angle in 0.1 deg (900 = straight ahead), with trim and
 *  backlash, for the fine edge probes. get_angle() then returns the
 *  nearest whole degree. */
bool drv_servo_set_angle_ddeg(int16_t ddeg);

/** Calibration only: replace the s5_calib.h trim and backlash at run
 *  time, so the calibration build can verify new values before they
 *  are pasted in. */
void drv_servo_set_cal(int16_t trim_ddeg, int16_t backlash_ddeg);

/** Last angle actually commanded. */
int16_t drv_servo_get_angle(void);

/** Worst-case travel plus ringing time, ms. Ranging earlier measures
 *  whatever the beam was sweeping past. */
uint32_t drv_servo_settle_ms(int16_t from_deg, int16_t to_deg);

/** Stop the pulses; the horn goes limp and draws no holding current. */
void drv_servo_release(void);

#endif /* DRV_SERVO_H */
