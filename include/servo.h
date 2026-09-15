/**
 * @file    servo.h
 * @brief   Hobby servo driver (RC pulse on one RP2040 PWM channel).
 *
 * Layer   : driver. Knows about hardware, knows nothing about the RTOS
 *           or about obstacles. Never blocks, never allocates.
 *
 * Angle convention used everywhere in this subsystem:
 *
 *      0 deg .......... full right          (servo pulse minimum)
 *     90 deg .......... straight ahead
 *    180 deg .......... full left           (servo pulse maximum)
 *
 * If your horn is mounted the other way round, set SERVO_INVERT to 1
 * in servo.c. Do not "fix" it by flipping angles at the call sites.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

/** Mechanical travel limits. Kept inside the physical end stops so a
 *  command can never stall the gearbox (a stalled SG90 draws ~700 mA
 *  and will brown out the Pico). */
#define SERVO_ANGLE_MIN_DEG         (15)
#define SERVO_ANGLE_MAX_DEG         (165)
#define SERVO_ANGLE_CENTRE_DEG      (90)

/**
 * @brief   Configure the PWM slice and park the horn at centre.
 * @return  true on success, false if the PWM clock cannot be derived.
 */
bool servo_init(void);

/**
 * @brief   Command an absolute angle. Clamped to the travel limits.
 * @param   angle_deg  Requested angle, degrees.
 * @return  true if the angle was applied as requested,
 *          false if it had to be clamped (caller may log this).
 */
bool servo_set_angle(int16_t angle_deg);

/**
 * @brief   Last angle actually commanded (post clamping).
 */
int16_t servo_get_angle(void);

/**
 * @brief   Worst-case time for the horn to travel and stop ringing.
 * @param   from_deg  Angle the horn is at now.
 * @param   to_deg    Angle it is being sent to.
 * @return  Settling time in milliseconds, >= 1.
 *
 * Ranging before this has elapsed measures whatever the sensor was
 * sweeping past, which is the single most common cause of garbage
 * obstacle profiles.
 */
uint32_t servo_settle_ms(int16_t from_deg, int16_t to_deg);

/**
 * @brief   Stop sending pulses. The horn goes limp and stops drawing
 *          holding current. Call this when the scan is finished.
 */
void servo_release(void);

#endif /* SERVO_H */
