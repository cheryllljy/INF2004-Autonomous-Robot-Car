/**
 * @file    sim_hal.h
 * @brief   HOST TEST ONLY. Control surface for the simulated world and
 *          the fake car used by test_sim.c.
 */
#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t pings;
    uint32_t servo_moves;
    uint32_t max_quiet_violation_ms;    /* must stay 0 */
} sim_counters_t;

void sim_world_clear(void);

/** Flat obstacle face: lateral centre (+ = left), distance ahead,
 *  true width, all in mm. */
void sim_add_box(double centre_x_mm, double y_mm, double width_mm);

/** 0 healthy, 1 ECHO stuck high, 2 sensor never responds. */
void sim_set_fault(int mode);

void sim_set_noise_mm(int16_t amplitude_mm);

/** Fake odometry for WaitStopped. */
void sim_set_speed_mm_s(int32_t speed);
bool sim_odometry(int32_t *p_speed_mm_s, int32_t *p_distance_mm);

void sim_counters_reset(void);
void sim_counters_get(sim_counters_t *p_out);

/*---------------------------------------------------------------------*/
/* v3.0: a realistically WRONG servo and sensor, for the calibration   */
/* tests. Defaults are all perfect, so the v2 scenarios are unchanged. */
/*---------------------------------------------------------------------*/

/** Where the horn really points: physical straight ahead is at command
 *  900 + trim (0.1 deg), and the gears stop backlash short of every
 *  command, on the side they came from. */
void sim_set_servo_error(int16_t true_trim_ddeg, int16_t true_backlash_ddeg);

/** Beam half-angle of the simulated HC-SR04, degrees (default 7.5). */
void sim_set_beam_half_deg(double half_deg);

/** Range error of the simulated sensor: raw = true * (1 + scale) + off. */
void sim_set_range_error(double scale, double offset_mm);

/** Physical sensor bearing right now, degrees (+ = left of straight). */
double sim_sensor_bearing_deg(void);

/** Replace the flat-face world with a caller's ray caster: returns the
 *  distance (mm) along a ray at `bearing_deg` from the sensor, or 0 for
 *  no hit. Used by the 2-D car simulation. NULL restores the boxes. */
void sim_set_ray_fn(double (*fn)(double bearing_deg));

/** Odometry source for S5 (default: sim_odometry). */
void sim_set_distance_mm(int32_t d);

#endif /* SIM_HAL_H */
