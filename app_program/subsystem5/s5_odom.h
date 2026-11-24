/*
 *      s5_odom.h
 *      Subsystem 5 -- where the car is, from Buddy 2's encoder ticks. Pure: no
 *      kernel, the host tests use the same code.
 *
 *      Dead reckoning from the two wheels: forward distance is the mean of
 *      the wheels, the turn is their difference over the wheel track. The
 *      track used is TRACK_WIDTH_MM x 100 / S5_TURN_PCT (calibration step
 *      4): the real car turns less than its wheels say (slip, scrub), and
 *      the same number is used to steer, so a turn asked for is a turn got.
 *
 *      Pose: the middle of the wheel axle, mm, and the heading, rad, + =
 *      left, in whatever frame s5_odom_set_pose() put it in.
 */

#ifndef S5_ODOM_H
#define S5_ODOM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float psi;
    float dist; /* path length driven, forward or back, mm */
    int32_t ticks_l;
    int32_t ticks_r;
    bool b_ticks; /* ticks_l/r hold a reading */
} s5_odom_t;

void s5_odom_init(s5_odom_t *p_o);
/* New tick counts (signed, forward +). */
void s5_odom_update(s5_odom_t *p_o, int32_t ticks_l, int32_t ticks_r);
void s5_odom_set_pose(s5_odom_t *p_o, float x, float y, float psi);
/* A point on the car, ahead of the axle and to its left, mm. */
void s5_odom_point(const s5_odom_t *p_o, float ahead, float left, float *p_x,
                   float *p_y);
/* The wheel track the turn is worked out with, mm. */
float s5_odom_track_mm(void);
/* The right/left wheel ratio in use, minus 1, and learning it after a
 * bypass (S5_ODOM_LEARN): err = the IRs' heading minus the encoders'
 * (rad) after dist mm. True if it was taken. */
float s5_odom_lr(void);
bool s5_odom_learn(float err, float dist);
/* the learnt ratio is used from here on (called as a bypass begins, so it
 * never changes in the middle of one) */
void s5_odom_lr_commit(void);
/* back to S5_ODOM_LR_PPM (the host tests, between runs) */
void s5_odom_lr_reset(void);

#endif /* S5_ODOM_H */
