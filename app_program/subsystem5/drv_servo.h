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

/** Kept inside the end stops: a stalled SG90 draws ~700 mA and can
 *  brown out the 5 V servo rail the sonar shares. */
#define DRV_SERVO_MIN_DEG           (15)
#define DRV_SERVO_MAX_DEG           (165)
#define DRV_SERVO_CENTRE_DEG        (90)

/** Configure PWM slice 7 for 50 Hz, 1 us per count, centre the horn.
 *  @return false if the clock cannot give an exact 1 MHz count. */
bool drv_servo_init(void);

/** Command an absolute angle, clamped to the limits.
 *  @return false if it had to be clamped. */
bool drv_servo_set_angle(int16_t angle_deg);

/** Last angle actually commanded. */
int16_t drv_servo_get_angle(void);

/** Worst-case travel plus ringing time, ms. Ranging earlier measures
 *  whatever the beam was sweeping past. */
uint32_t drv_servo_settle_ms(int16_t from_deg, int16_t to_deg);

/** Stop the pulses; the horn goes limp and draws no holding current. */
void drv_servo_release(void);

#endif /* DRV_SERVO_H */
