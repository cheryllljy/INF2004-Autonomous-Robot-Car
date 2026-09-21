/**
 * @file    sim_hal.h
 * @brief   HOST TEST CODE ONLY. Control surface for the simulated
 *          obstacle world used by test_sim.c.
 */
#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <stdint.h>

/** Counters the scenario can assert on. */
typedef struct
{
    uint32_t pings;
    uint32_t servo_moves;
} sim_counters_t;

/** Remove every object and clear the fault flag. */
void sim_world_clear(void);

/**
 * @brief   Add a flat obstacle face.
 * @param   centre_x_mm  Lateral position, positive is to the left of
 *                       the car's centreline.
 * @param   y_mm         Distance straight ahead.
 * @param   width_mm     True physical width.
 */
void sim_add_box(double centre_x_mm, double y_mm, double width_mm);

/** 0 = healthy, 1 = ECHO stuck high, 2 = sensor never responds
 *  (dead interrupt, swapped TRIG/ECHO, no power). */
void sim_set_fault(int on);

/** Add deterministic pseudo-random ranging noise, +/- amplitude. */
void sim_set_noise_mm(int16_t amplitude_mm);

void sim_counters_reset(void);
void sim_counters_get(sim_counters_t *p_out);

#endif /* SIM_HAL_H */
