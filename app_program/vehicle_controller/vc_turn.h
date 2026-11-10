/*
 *      vc_turn.h
 *      VehicleController -- the turn at a junction.
 *
 *      Called only from vc_task, so nothing in here needs a lock. It drives
 *      through Buddy 2's motion_*() API, like subsystem 5's bypass.
 *
 *      Buddy 3 reports a junction once both line sensors have been black
 *      for 8 mm, so the IR pair is on the crossing line. The wheel axle is
 *      VC_IR_AHEAD_MM behind it (125 mm on our car). The old turn stopped
 *      there and pivoted, so it turned 125 mm before the junction: the car
 *      ended up beside the new line, not on it, and the stop + spin was
 *      abrupt.
 *
 *      LEFT / RIGHT now pivot on the inner
 *      wheel by default (VC_CORNER_PIVOT, below). The arc described next is
 *      still there with VC_CORNER_PIVOT 0.
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
 *      the arc runs at VC_CORNER_MAX_MM_S (40 mm/s, was
 *      80) and looks for the line from 25 deg (was 60), so a branch that
 *      curves is found too. A branch curving back towards the line the car
 *      came in on is not reached by the quarter circle at all, so past
 *      90 deg the arc tightens and keeps looking for VC_CORNER_SEEK_MORE_DEG.
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
 *      Either turn also ends, with a message, if the line is not found:
 *      the U-turn VC_UTURN_OVER_DEG past 180 deg, the arc
 *      VC_CORNER_SEEK_MORE_DEG past 90 deg.
 */

#ifndef VC_TURN_H
#define VC_TURN_H

#include "../app.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/buddy3.h" /* b3_nav_command_t, B3_BASE_SPEED_MM_S */

/* IR pair ahead of the wheel axle (125 mm, measured on 2026-11-06;
 * subsystem 5 has the same number as S5_IR_AHEAD_MM). */
#define VC_IR_AHEAD_MM 125

/* Arc for LEFT / RIGHT. The axle centre moves at the line-following
 * speed, capped so a fast line speed does not throw the car round. */
#define VC_CORNER_RADIUS_MM VC_IR_AHEAD_MM
#define VC_CORNER_MAX_MM_S  40 /* was 80, missed a curve */
/* the U-turn eases down from at most this speed */
#define VC_UTURN_EASE_MAX_MM_S 160
#define VC_CORNER_SPEED_MM_S                                        \
    ((B3_BASE_SPEED_MM_S < VC_CORNER_MAX_MM_S) ? B3_BASE_SPEED_MM_S \
                                               : VC_CORNER_MAX_MM_S)
#define VC_CORNER_DEG 90
/* heading error -> extra turn rate, 1/s (keeps the car on the arc when a
 * wheel lags) */
#define VC_CORNER_GAIN_PER_S 2
/* from this heading on, the IRs look for the new line */
/* was 60; a branch that curves towards the car is met
 * earlier in the arc */
#ifndef VC_CORNER_SEEK_DEG
#define VC_CORNER_SEEK_DEG 25
#endif
/* not found by 90 deg (a branch curving back towards
 * the line the car came in on): the arc tightens to the inner wheel's
 * VC_ARC_MIN_WHEEL_MM_S, so the IRs sweep in towards it, for up to
 * VC_CORNER_SEEK_MORE_DEG more */
#define VC_CORNER_SEEK_MORE_DEG 75
/* LEFT / RIGHT pivot on the inner wheel instead of the
 * 125 mm arc, which drove too far along the old line: a branch bending the
 * other way a few cm after the junction came before the turn was done.
 * The inner wheel stands still (0 = no drive, not a slow crawl), the outer
 * one drives at VC_CORNER_PIVOT_MM_S. From VC_CORNER_SEEK_DEG on, the turn
 * ends once the inner IR (IR_L for LEFT) has crossed the new line: black,
 * then white again with the line between the IRs (or the outer IR sees
 * it). The car then goes straight and Buddy 3 takes over. */
#ifndef VC_CORNER_PIVOT
#define VC_CORNER_PIVOT 1
#endif
#ifndef VC_CORNER_PIVOT_MM_S
#define VC_CORNER_PIVOT_MM_S 60 /* outer wheel: about 31 deg/s */
#endif
/* Drive straight on this far after the junction before pivoting. The IRs
 * are 125 mm ahead of the axle, so a pivot started at once meets the new
 * line at about 48 deg and Buddy 3 has to turn the rest; 20 mm more makes
 * it about 64 deg. Host sim (test_turn): with 0, a branch bending within
 * 100 mm of the junction was lost at line speeds of 80 mm/s or less; with
 * 20 every scenario passes at 25-100 mm/s.
 * 20 -> 22 with B3_JUNCTION_MIN_PATCH_UM 8 -> 6 mm: the
 * junction is reported 2 mm sooner after both IRs go black, so the pivot
 * starts where it did (28 mm into the patch). With 20, J29 (branch bends
 * R250 100 mm on) was lost at 100 mm/s. */
#ifndef VC_CORNER_PREDRIVE_MM
#define VC_CORNER_PREDRIVE_MM 22
#endif
/* safety: the arc also ends once the outer wheel has driven this much of
 * its nominal quarter circle, whatever the encoder heading says */
#define VC_CORNER_CAP_PCT 200 /* was 150 (seek past 90) */

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

/* U-turn: line not found this far past 180 deg: give up, Buddy 3 takes over
 * (the arc uses VC_CORNER_SEEK_MORE_DEG) */
#define VC_TURN_OVER_DEG 30
/* the U-turn keeps pivoting (slowly, IRs looking) until
 * this far past 180 deg by the encoders before giving up. The car's wheels
 * turn it less than the encoders say (up to ~1/1.6 measured on the floor),
 * so 30 gave up at about 150 deg real and the line was lost. */
#ifndef VC_UTURN_OVER_DEG
#define VC_UTURN_OVER_DEG 150
#endif

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

/* Wheel speeds for the soft start: (left, right) scaled by how far into
 * the soft start the car is (elapsed_ms of VC_SOFT_START_MS). A wheel is
 * never made slower than VC_MIN_WHEEL_MM_S (or its own speed, if that is
 * lower) and never 0, so Buddy 2 never sees a (0, 0) stop from it. */
void vc_turn_soft_start(int16_t *p_left, int16_t *p_right, int32_t elapsed_ms);

#endif /* VC_TURN_H */
