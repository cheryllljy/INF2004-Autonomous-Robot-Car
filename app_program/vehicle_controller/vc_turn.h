/*
 *      vc_turn.h
 *      VehicleController -- the turn at a junction.
 *
 *      Called only from vc_task, so nothing in here needs a lock. It drives
 *      through Buddy 2's motion_*() API, like subsystem 5's bypass.
 *
 *      Buddy 3 reports a junction once both line sensors have been black
 *      for 8 mm, so the IR pair is on the crossing line and the wheel axle
 *      is VC_IR_AHEAD_MM behind it.
 *
 *      LEFT / RIGHT: by default the car pivots on the inner wheel
 *      (VC_CORNER_PIVOT). With VC_CORNER_PIVOT 0 it drives one smooth arc of
 *      radius VC_IR_AHEAD_MM instead, centred beside the axle, so after a
 *      quarter circle the axle is on the crossing line. Either way the END
 *      is found with the IRs, not the encoders: from VC_CORNER_SEEK_DEG on,
 *      the turn ends once the line is centred between them, so track width
 *      errors or slipping tyres cannot leave the car beside the line. Past
 *      90 deg the arc tightens and keeps looking for VC_CORNER_SEEK_MORE_DEG.
 *
 *      U-TURN: the car eases down (VC_SOFT_DECEL_MMPS2), pivots left on the
 *      spot with ramped wheel speeds (VC_UTURN_RATE_DPS,
 *      VC_PIVOT_ACCEL_MMPS2), slows to VC_UTURN_SEEK_DPS for the last
 *      VC_UTURN_SEEK_DEG and stops when the line is centred between the
 *      IRs, then speeds up gently (VC_SOFT_START_MS).
 *
 *      Slow wheels: Buddy 2 gives a wheel no drive at 5 mm/s or less
 *      (MC_ZERO_SPEED) and kicks a wheel that reads under 10 mm/s, so no
 *      wheel speed is asked for between 0 and VC_MIN_WHEEL_MM_S (the arc's
 *      inner wheel, already rolling, not below VC_ARC_MIN_WHEEL_MM_S).
 *
 *      Time: the loop runs about every 20 ms, so it passes the measured
 *      time since the last pass, not 10 ms.
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

/* IR pair ahead of the wheel axle (125 mm, measured;
 * subsystem 5 has the same number as S5_IR_AHEAD_MM). */
#define VC_IR_AHEAD_MM 125

/* Arc for LEFT / RIGHT. The axle centre moves at the line-following
 * speed, capped so a fast line speed does not throw the car round. */
#define VC_CORNER_RADIUS_MM VC_IR_AHEAD_MM
#define VC_CORNER_MAX_MM_S  40
/* the U-turn eases down from at most this speed */
#define VC_UTURN_EASE_MAX_MM_S 160
#define VC_CORNER_SPEED_MM_S                                        \
    ((B3_BASE_SPEED_MM_S < VC_CORNER_MAX_MM_S) ? B3_BASE_SPEED_MM_S \
                                               : VC_CORNER_MAX_MM_S)
#define VC_CORNER_DEG 90
/* heading error -> extra turn rate, 1/s (keeps the car on the arc when a
 * wheel lags) */
#define VC_CORNER_GAIN_PER_S 2
/* from this heading on, the IRs look for the new line (low, so a branch
 * that curves towards the car is found too) */
#ifndef VC_CORNER_SEEK_DEG
#define VC_CORNER_SEEK_DEG 25
#endif
/* not found by 90 deg: the arc tightens to the inner wheel's
 * VC_ARC_MIN_WHEEL_MM_S so the IRs sweep inwards, for up to this much more */
#define VC_CORNER_SEEK_MORE_DEG 75
/* 1: LEFT / RIGHT pivot on the inner wheel instead of the arc, which
 * drives too far along the old line for a branch that bends soon after.
 * The inner wheel stands still (0 = no drive), the outer one drives at
 * VC_CORNER_PIVOT_MM_S. The turn ends once the inner IR has crossed the
 * new line (or the outer IR sees it); then Buddy 3 takes over. */
#ifndef VC_CORNER_PIVOT
#define VC_CORNER_PIVOT 1
#endif
#ifndef VC_CORNER_PIVOT_MM_S
#define VC_CORNER_PIVOT_MM_S 60 /* outer wheel: about 31 deg/s */
#endif
/* Drive straight on this far (mm) after the junction before pivoting, so
 * the pivot meets the new line at about 64 deg instead of 48; tuned with
 * B3_JUNCTION_MIN_PATCH_UM (pivot starts 28 mm into the patch). */
#ifndef VC_CORNER_PREDRIVE_MM
#define VC_CORNER_PREDRIVE_MM 22
#endif
/* safety: the arc also ends once the outer wheel has driven this much of
 * its nominal quarter circle, whatever the encoder heading says */
#define VC_CORNER_CAP_PCT 200

/* U-turn */
#define VC_UTURN_RATE_DPS    45  /* pivot rate */
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

/* line not found this far past 180 deg: give up, Buddy 3 takes over */
#define VC_TURN_OVER_DEG 30
/* the U-turn keeps pivoting (slowly, IRs looking) until this far past
 * 180 deg by the encoders: the car turns less than the encoders say (up to
 * ~1/1.6 measured on the floor). */
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
