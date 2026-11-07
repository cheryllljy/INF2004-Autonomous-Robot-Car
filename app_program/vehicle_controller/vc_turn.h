/*
 *      vc_turn.h
 *      VehicleController -- the turn at a junction (nyx, 2026-11-06).
 *
 *      Called only from vc_task, so nothing in here needs a lock. It drives
 *      through Buddy 2's motion_*() API, like vc_obstacle.c.
 *
 *      Buddy 3 reports a junction once both line sensors have been black
 *      for 8 mm, so the IR pair is on the crossing line. The wheel axle is
 *      VC_IR_AHEAD_MM behind it (125 mm on nyx's car). The old turn stopped
 *      there and pivoted, so it turned 125 mm before the junction: the car
 *      ended up beside the new line, not on it, and the stop + spin was
 *      abrupt.
 *
 *      LEFT / RIGHT: one smooth arc, no stop. The arc's radius is
 *      VC_IR_AHEAD_MM and its centre is beside the axle, so after a quarter
 *      circle the axle is on the crossing line, pointing along it, with the
 *      IR pair on the line 125 mm further on. Wheel speeds come from the
 *      wheel track (TRACK_WIDTH_MM) and the encoder heading keeps the car on
 *      the arc. The END is found with the IRs, not the encoders: from
 *      VC_CORNER_SEEK_DEG on, the arc ends once the line is centred between
 *      them (the inner IR has seen it and gone white again, or the outer IR
 *      sees it). A wheel track that differs from TRACK_WIDTH_MM or slipping
 *      tyres then cannot leave the car beside the line. Buddy 3 takes over
 *      while the car is still rolling.
 *
 *      U-TURN: the car eases down (VC_SOFT_DECEL_MMPS2) instead of
 *      braking, pivots left on the spot with wheel speeds that ramp up and
 *      down (VC_UTURN_RATE_DPS, VC_PIVOT_ACCEL_MMPS2), slows to
 *      VC_UTURN_SEEK_DPS for the last VC_UTURN_SEEK_DEG and stops turning
 *      when the line it came in on is centred between the IRs. It then rolls
 *      on slowly and the VehicleController speeds up gently
 *      (VC_SOFT_START_MS). The pivot is about the axle, so the IR pair ends
 *      on the line the car came in on, 2 x VC_IR_AHEAD_MM before the
 *      junction.
 *
 *      Slow wheels: Buddy 2 gives a wheel no drive at all at 5 mm/s or less
 *      (MC_ZERO_SPEED) and kicks a wheel that reads under 10 mm/s
 *      (MOTION_STOPPED_SPEED_MMPS) with MOTOR_BREAKAWAY_PM. So no wheel
 *      speed is asked for between 0 and VC_MIN_WHEEL_MM_S (the arc's inner
 *      wheel, already rolling, not below VC_ARC_MIN_WHEEL_MM_S); a ramp that
 *      started lower was a coast, then a kick.
 *
 *      Time: the VehicleController loop runs about every 20 ms
 *      (tk_dly_tsk(10) on the 10 ms tick), so it passes the measured time
 *      since the last pass, not 10 ms.
 *
 *      Either turn also ends, with a message, if the line is not found by
 *      VC_TURN_OVER_DEG past the nominal angle.
 */

#ifndef VC_TURN_H
#define VC_TURN_H

#include "../app.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/buddy3.h" /* b3_nav_command_t, B3_BASE_SPEED_MM_S */
#include "vc_obstacle.h"          /* VC_OBS_IR_AHEAD_MM */

/* IR pair ahead of the wheel axle (125 mm, measured by nyx on 2026-11-06),
 * one value for the turn and the obstacle bypass. */
#define VC_IR_AHEAD_MM VC_OBS_IR_AHEAD_MM

/* Arc for LEFT / RIGHT. The axle centre moves at the line-following
 * speed, capped so a fast line speed does not throw the car round. */
#define VC_CORNER_RADIUS_MM VC_IR_AHEAD_MM
#define VC_CORNER_MAX_MM_S  80
#define VC_CORNER_SPEED_MM_S                                        \
    ((B3_BASE_SPEED_MM_S < VC_CORNER_MAX_MM_S) ? B3_BASE_SPEED_MM_S \
                                               : VC_CORNER_MAX_MM_S)
#define VC_CORNER_DEG 90
/* heading error -> extra turn rate, 1/s (keeps the car on the arc when a
 * wheel lags) */
#define VC_CORNER_GAIN_PER_S 2
/* from this heading on, the IRs look for the new line */
#define VC_CORNER_SEEK_DEG 60
/* safety: the arc also ends once the outer wheel has driven this much of
 * its nominal quarter circle, whatever the encoder heading says */
#define VC_CORNER_CAP_PCT 150

/* U-turn */
#define VC_UTURN_RATE_DPS    45  /* pivot rate (was Buddy 2's default 90) */
#define VC_UTURN_SEEK_DPS    25  /* slower for the last part ... */
#define VC_UTURN_SEEK_DEG    30  /* ... the last 30 deg, IRs looking */
#define VC_PIVOT_ACCEL_MMPS2 150 /* wheel speed ramp of the pivot */
#define VC_SOFT_DECEL_MMPS2  150 /* easing down (Buddy 2 brakes at 400) */
#define VC_SOFT_START_MS     500 /* speeding up after the U-turn */
#define VC_SOFT_START_PCT    20  /* first speed of the soft start, %, ... */

/* ... but never a wheel speed Buddy 2 cannot hold smoothly (see above) */
#define VC_MIN_WHEEL_MM_S     15
#define VC_ARC_MIN_WHEEL_MM_S 12
/* the U-turn's easing ends and its pivot starts at this speed */
#define VC_SOFT_STOP_MM_S VC_MIN_WHEEL_MM_S

/* line not found this far past 90 / 180 deg: give up, Buddy 3 takes over */
#define VC_TURN_OVER_DEG 30

typedef enum
{
    VC_TURN_RUNNING = 0, /* stay in VC_STATE_TURNING */
    VC_TURN_DONE,        /* back to LINE_FOLLOW, still rolling */
    /* back to LINE_FOLLOW at VC_MIN_WHEEL_MM_S: soft start */
    VC_TURN_DONE_STOPPED
} vc_turn_result_t;

/* Start the turn. LEFT, RIGHT or UTURN; anything else returns false and
 * the car goes straight on. */
bool vc_turn_start(b3_nav_command_t turn, const motion_status_t *p_motion);

/* One VehicleController pass of the turn, dt_ms since the last one, with
 * Buddy 3's IR readings (true = black). */
vc_turn_result_t vc_turn_step(const motion_status_t *p_motion, bool left_black,
                              bool right_black, int32_t dt_ms);

/* Carry on with a turn that an obstacle check stopped (the car stood
 * still meanwhile). False if no turn was under way. */
bool vc_turn_resume(const motion_status_t *p_motion);

/* Wheel speeds for the soft start: (left, right) scaled by how far into
 * the soft start the car is (elapsed_ms of VC_SOFT_START_MS). A wheel is
 * never made slower than VC_MIN_WHEEL_MM_S (or its own speed, if that is
 * lower) and never 0, so Buddy 2 never sees a (0, 0) stop from it. */
void vc_turn_soft_start(int16_t *p_left, int16_t *p_right, int32_t elapsed_ms);

#endif /* VC_TURN_H */
