/**
 * @file    motion_mock.h
 * @brief   HOST TEST ONLY. Control surface of the simulated car behind
 *          the motion_control.h mock (motion_mock.c).
 */
#ifndef MOTION_MOCK_H
#define MOTION_MOCK_H

/** Put the car at a pose (axle centre, mm; heading deg, + = left). */
void   mm_reset(double x, double y, double psi_deg);

/** Encoder scale error (0.02 = reads 2 % long) and track-width error
 *  (0.03 = the real track is 3 % wider than the firmware thinks). */
void   mm_set_errors(double encoder_scale, double track_error);
void   mm_set_dead_right(int dead);   /* v3.2: GP3-style dead encoder */

/** Advance the car 1 ms. */
void   mm_step_1ms(void);

void   mm_get_truth(double *p_x, double *p_y, double *p_psi);
double mm_speed_true(void);
/* nyx, 2026-11-08: the left encoder reads long by lr / 2, the right one
 * short by lr / 2 (dead reckoning drifts round to the right) */
void mm_set_lr_error(double lr);
/* nyx, 2026-11-06: smallest non-zero |wheel speed| asked with
 * motion_set_wheel_speeds() since the last restart (0x7FFF: none) */
int    mm_min_wheel_cmd(int restart);

/* nyx, 2026-11-07: Buddy 2's 20 ms goal sampling, setpoint ramp and a
 * wheel lag of lag_ms (0 = ideal wheels, the default); both wheels start
 * at v0 mm/s in velocity mode. mm_kicks(): breakaway kicks so far. */
void   mm_set_lag(int lag_ms, double v0);
void   mm_set_motor(double off_pm, double ratio); /* call before mm_set_lag */
int    mm_kicks(void);

#endif /* MOTION_MOCK_H */
