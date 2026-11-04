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

/** Advance the car 1 ms. */
void   mm_step_1ms(void);

void   mm_get_truth(double *p_x, double *p_y, double *p_psi);
double mm_speed_true(void);

#endif /* MOTION_MOCK_H */
