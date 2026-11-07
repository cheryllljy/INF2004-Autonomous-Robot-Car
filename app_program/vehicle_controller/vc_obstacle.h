/*
 *      vc_obstacle.h
 *      VehicleController -- the obstacle part (Buddy 5 hook, v3.0).
 *
 *      Owner: Buddy 5 (2502410). Called only from vc_task, so nothing in
 *      here needs a lock. It never touches the servo or the sonar: it asks
 *      ObstacleScan (S5) through IF-S5-1/2/7 and drives through Buddy 2's
 *      motion_*() API, the same as the rest of vc_task.
 *
 *      What it does once S5 raises DETECTED (the car is already stopping):
 *
 *        WAIT_SCAN  S5 waits for speed 0 (WaitStopped), sweeps, publishes.
 *          CONTINUE  -> back to line following
 *          REVERSE   -> back off the distance S5 asks for, slowly, then
 *                       obstacle_rescan()                       (prof item 3)
 *          BYPASS_x  -> v3.1 (nyx, 2026-11-06): the FIRST bypass profile of
 *                       an episode always backs off slowly to about
 *                       VC_OBS_REMEASURE_MM from the face and rescans; the
 *                       manoeuvre below is planned from the second profile.
 *                       Both profiles' width and edges are printed.
 *          STOP/SLOW -> stay stopped, S5 rescans; 3 in a row -> fault
 *
 *        Bypass, side s (+1 left, -1 right). S5 picks s: the side whose
 *        swing is smaller, i.e. the clearer side (an obstacle more to the
 *        left -> pass on the right).
 *          TURN_OUT   turn s * a. v3.1: a is the smallest angle (20..70 deg)
 *                     whose OUT leg reaches the swing before the obstacle,
 *                     so the car stays as close to its lane as it can
 *          OUT        drive until the car is s * (swing + extra) to the side
 *          TURN_PAR   back to the original heading
 *          PASS       obstacle_watch_start(): S5 turns the sensor to the
 *                     obstacle and back on every look (prof item 2). Drive
 *                     until S5_EVT_PASSED. S5_EVT_SIDE_CLOSE -> stop, turn
 *                     10 deg away, carry on.
 *          IN         v3.1: one smooth arc back to the centre of the track
 *                     (wheel speeds, no pivot turns): the heading asked for
 *                     is proportional to the sideways offset, so the car
 *                     eases onto the line and is straight when it gets
 *                     there. Buddy 3's IR corrects the offset when the line
 *                     passes between the two sensors.
 *                     v3.4 (nyx, 2026-11-06): a line met while dead
 *                     reckoning still has the car far from its own line is
 *                     another lane's and is ignored; near the line the arc
 *                     slows down; once the line has been between the IRs,
 *                     the moment it is under one IR again Buddy 3 takes
 *                     over (it steers towards it); the arc gives up by
 *                     distance, not after a fixed time.
 *          (TURN_IN and ALIGN, the v3.0 pivot turns, are no longer used;
 *          the values stay so the step numbers in telemetry do not move.)
 *        -> DONE: vc_task goes back to LINE_FOLLOW while still rolling;
 *           Buddy 3 takes over.
 *
 *      The pose (x sideways +left, y forward, heading +left) is dead
 *      reckoned from Buddy 2's distance and heading, in the frame the car
 *      had when the obstacle was detected, so the rejoin error against the
 *      original line is measured, printed and kept (requirement: +/-50 mm).
 *
 *      HARD_STOP (under 120 mm) at any time: instant motor cut through
 *      Buddy 2's motion_estop_request(MOTION_ESTOP_OBSTACLE), released by
 *      this module once both wheels read stopped. It is NOT the latched
 *      system E-stop: no fault bit, no RESET needed (Tribunal 1: E-stop
 *      only on a sensor fault). The bypass is then re-planned with a rescan.
 */

#ifndef VC_OBSTACLE_H
#define VC_OBSTACLE_H

#include "../app.h"
#include "../subsystem2/motion_control.h"

typedef enum
{
    VC_OBS_RUNNING = 0, /* stay in VC_STATE_OBSTACLE */
    VC_OBS_DONE,        /* back to LINE_FOLLOW */
    VC_OBS_FAULT        /* raise VC_FAULT_OBSTACLE */
} vc_obs_result_t;

/* Steps, 4 bits (14 values), exported for the log and telemetry. */
typedef enum
{
    VC_OBS_IDLE = 0,
    VC_OBS_WAIT_SCAN,
    VC_OBS_REVERSE,
    VC_OBS_ALIGN_SCAN, /* square up before a rescan */
    VC_OBS_TURN_OUT,
    VC_OBS_OUT,
    VC_OBS_TURN_PAR,
    VC_OBS_PASS,
    VC_OBS_SIDESTEP,
    VC_OBS_TURN_IN,
    VC_OBS_IN,
    VC_OBS_ALIGN,
    VC_OBS_FINISHED,
    VC_OBS_FAILED,
    VC_OBS_STEP_COUNT
} vc_obs_step_t;

#define VC_OBS_STEP_BITS 4U

/* Tunables. TODO(measure) items need the real chassis. */
#define VC_OBS_SPEED_MM_S 150 /* bypass legs */
/* backing off (sensor faces forward).
 * nyx, 2026-11-06: was 100, too quick */
#define VC_OBS_REV_SPEED_MM_S 40
#define VC_OBS_TURN_DPS       90
#define VC_OBS_TURN_OUT_DEG   45 /* v3.0 fixed angle; v3.1 computes it */
#define VC_OBS_TURN_MIN_DEG \
    20                           /* nyx, 2026-11-06: shallowest out
                                           angle (smaller = longer out leg) */
#define VC_OBS_TURN_STEEP_DEG 70 /* steepest out angle */
#define VC_OBS_EXTRA_MM \
    10 /* added to S5's swing. nyx,
                                           2026-11-06: 20 -> 10, stay close
                                           to the lane */
#define VC_OBS_REMEASURE_MM \
    350 /* nyx, 2026-11-06: back off to about
                                           this from the face, then rescan */
#define VC_OBS_FRONT_MARGIN_MM \
    60 /* nearest - margin must cover the
                                           forward travel of the OUT leg */
#define VC_OBS_SIDESTEP_DEG  10
#define VC_OBS_MAX_SIDESTEPS 3
#define VC_OBS_MAX_DEPTH_MM  600 /* longest obstacle we will pass */
#define VC_OBS_PASS_TAIL_MM \
    300 /* + rear of the car + margin: the
                                           pass leg's hard cap is nearest +
                                           depth + this (S5 normally says
                                           PASSED long before) */
/* IR sensors ahead of the wheel axle. nyx, 2026-11-06: measured 125 mm
 * on the car (was a 60 mm guess); vc_turn.c uses the same value. */
#define VC_OBS_IR_AHEAD_MM 125
#define VC_OBS_IN_EXTRA_MM 100 /* v3.0 only (straight IN leg) */

/* nyx, 2026-11-06 (v3.1): the smooth arc back to the centre */
#define VC_OBS_ARC_SPEED_MM_S 100 /* forward speed on the arc */
#define VC_OBS_ARC_LOOKAHEAD_MM \
    200                       /* heading asked = -offset / this
                                           (rad): 50 mm off -> 14 deg */
#define VC_OBS_ARC_MAX_DEG 30 /* never cut back steeper than this */
#define VC_OBS_ARC_GAIN \
    2 /* wheel speed difference, mm/s per
                                           deg of heading error */
#define VC_OBS_ARC_DIFF_MM_S \
    40                         /* at most +/- this per wheel: turn
                                           radius >= ~140 mm at 100 mm/s */
#define VC_OBS_ARC_DONE_MM  15 /* on the centre within this...  */
#define VC_OBS_ARC_DONE_DEG 3  /* ...and this straight: done    */
#define VC_OBS_ARC_SEND_MS \
    20 /* wheel speed update period (Buddy
                                           2's 20 ms control loop, 4-deep
                                           command queue) */
/* arc never converged: fail. nyx, 2026-11-06 (v3.4): 12 -> 40 s, a cap
 * for a car that is not moving; the real limit is the distance
 * (VC_OBS_ARC_SLACK_MM), so a slower car is not cut off. */
#define VC_OBS_ARC_TIMEOUT_MS 40000
#define VC_OBS_ARC_SLACK_MM   600
/* nyx, 2026-11-06 (v3.4): within this of the line (dead reckoning, at
 * the IRs), after the first IR hit, and while searching, the arc slows
 * to VC_OBS_ARC_NEAR_SPEED_MM_S. */
#define VC_OBS_ARC_NEAR_MM         100
#define VC_OBS_ARC_NEAR_SPEED_MM_S 60
/* nyx, 2026-11-06 (v3.4): the course has lanes. On the way back, a line
 * met while dead reckoning has the car more than half the bypass offset
 * (at least this) from its own line is another lane's, and ignored. */
#define VC_OBS_LINE_WINDOW_MIN_MM 100
/* nyx, 2026-11-06 (v3.2): line search at the end of the arc. When dead
 * reckoning says the car is within VC_OBS_SEARCH_X_MM of the line (or
 * past it) and no IR has seen the line yet, keep closing in at
 * VC_OBS_SEARCH_DEG for at most VC_OBS_SEARCH_MM (205 mm sideways), then
 * stop and fail. */
#define VC_OBS_SEARCH_X_MM VC_OBS_ARC_DONE_MM
#define VC_OBS_SEARCH_DEG  20
#define VC_OBS_SEARCH_MM   600
/* nyx, 2026-11-06 (v3.2): steepest-at-least angle back to the line
 * until an IR has seen it. */
#define VC_OBS_APPROACH_MIN_DEG 15
/* nyx, 2026-11-06 (v3.2): one wheel turning VC_OBS_ENC_MOVED_MM while
 * the other's encoder reads under VC_OBS_ENC_DEAD_MM means a dead encoder
 * (GP3 on nyx's car): the heading and distance are wrong, so stop
 * instead of steering blind. */
#define VC_OBS_ENC_MOVED_MM 150
#define VC_OBS_ENC_DEAD_MM  5
#define VC_OBS_LEG_BLOCK_MM \
    250 /* front closer than this on a leg:
                                           stop, square up, rescan. Covers
                                           the look interval in WATCH (~45 mm
                                           at 150 mm/s), the ramp-down (~30
                                           mm), the 50 mm of car ahead of the
                                           sensor, and a margin (sim C10) */
/* ...but only within what is left of
 * the leg plus this (car nose + stop) */
#define VC_OBS_BLOCK_AHEAD_MM 150
#define VC_OBS_MAX_STOP_SCANS 3 /* STOP/SLOW profiles in a row */
#define VC_OBS_MAX_REPLANS    3 /* square up + rescan, per obstacle */
#define VC_OBS_GONE_MS \
    1500 /* no event after STOP/SLOW and the
                                           front is clear: obstacle gone */
/* nyx, 2026-11-06: 45 -> 60 s for the slower v3.2 scans. */
#define VC_OBS_TIMEOUT_MS    60000 /* whole episode */
#define VC_OBS_STILL_MM_S    10    /* both wheels below: stopped */
#define VC_OBS_STILL_MS      100   /* ...for this long: release a cut */
#define VC_OBS_REJOIN_TOL_MM 50    /* requirement */

/* vc_task interface */
extern void vc_obs_begin(const motion_status_t *p_motion);
extern void vc_obs_events(UINT s5_events);
extern vc_obs_result_t vc_obs_step(const motion_status_t *p_motion,
                                   bool ir_left_black, bool ir_right_black,
                                   INT period_ms);
extern void vc_obs_leave(void); /* left OBSTACLE early */

/* HARD_STOP motor cut, any VC state */
extern void vc_obs_hard_stop(const motion_status_t *p_motion);
extern void vc_obs_service_cut(const motion_status_t *p_motion, INT period_ms);
extern bool vc_obs_cut_active(void);

/* Diagnostics */
extern vc_obs_step_t vc_obs_get_step(void);
extern int32_t vc_obs_last_rejoin_mm(void); /* + = left of line */
/* nyx, 2026-11-06: this episode planned a bypass (the car left the line) */
extern bool vc_obs_bypassed(void);

#endif /* VC_OBSTACLE_H */
