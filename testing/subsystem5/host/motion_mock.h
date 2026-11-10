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
void   mm_wheels_true(double *l, double *r);
/* the left encoder reads long by lr / 2, the right one
 * short by lr / 2 (dead reckoning drifts round to the right) */
void mm_set_lr_error(double lr);
/* smallest non-zero |wheel speed| asked with
 * motion_set_wheel_speeds() since the last restart (0x7FFF: none) */
int    mm_min_wheel_cmd(int restart);

/* Buddy 2's 20 ms goal sampling, setpoint ramp and a
 * wheel lag of lag_ms (0 = ideal wheels, the default); both wheels start
 * at v0 mm/s in velocity mode. mm_kicks(): breakaway kicks so far. */
void   mm_set_lag(int lag_ms, double v0);
void   mm_set_stiction(double brk);
void   mm_set_stribeck(double ks, double kk, double vs);
void   mm_set_motor(double off_pm, double ratio); /* call before mm_set_lag */
int    mm_kicks(void);
/* Buddy 2's measured wheel speeds (80 ms window) */
void   mm_wheels_meas(double *l, double *r);
/* turning load: the outer wheel needs pm more drive, the
 * inner one is pushed along by pm, in full from a wheel speed difference
 * of w mm/s (PI wheel model only) */
void   mm_set_yaw_load(double pm, double w);

#endif /* MOTION_MOCK_H */
