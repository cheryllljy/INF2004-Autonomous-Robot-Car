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

#endif /* SIM_HAL_H */
