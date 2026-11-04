/*
 *	vc_obstacle.h
 *	VehicleController -- the obstacle part (Buddy 5 hook, v3.0).
 *
 *	Owner: Buddy 5 (2502410). Called only from vc_task, so nothing in
 *	here needs a lock. It never touches the servo or the sonar: it asks
 *	ObstacleScan (S5) through IF-S5-1/2/7 and drives through Buddy 2's
 *	motion_*() API, the same as the rest of vc_task.
 *
 *	What it does once S5 raises DETECTED (the car is already stopping):
 *
 *	  WAIT_SCAN  S5 waits for speed 0 (WaitStopped), sweeps, publishes.
 *	    CONTINUE  -> back to line following
 *	    REVERSE   -> back off the distance S5 asks for, slowly, then
 *	                 obstacle_rescan()                       (prof item 3)
 *	    BYPASS_x  -> the manoeuvre below
 *	    STOP/SLOW -> stay stopped, S5 rescans; 3 in a row -> fault
 *
 *	  Bypass, side s (+1 left, -1 right):
 *	    TURN_OUT   turn s * a (45 deg, steeper if the obstacle is close)
 *	    OUT        drive until the car is s * (swing + extra) to the side
 *	    TURN_PAR   back to the original heading
 *	    PASS       obstacle_watch_start(): S5 turns the sensor to the
 *	               obstacle and back on every look (prof item 2). Drive
 *	               until S5_EVT_PASSED. S5_EVT_SIDE_CLOSE -> stop, turn
 *	               10 deg away, carry on.
 *	    TURN_IN    face back towards the line (-s * 45 deg)
 *	    IN         drive until Buddy 3's IR sees the line (or dead
 *	               reckoning says we are on it), plus the IR-to-axle
 *	               distance so the AXLE ends on the line
 *	    ALIGN      original heading again, obstacle_resume()
 *	  -> DONE: vc_task goes back to LINE_FOLLOW; Buddy 3 takes over.
 *
 *	The pose (x sideways +left, y forward, heading +left) is dead
 *	reckoned from Buddy 2's distance and heading, in the frame the car
 *	had when the obstacle was detected, so the rejoin error against the
 *	original line is measured, printed and kept (requirement: +/-50 mm).
 *
 *	HARD_STOP (under 120 mm) at any time: instant motor cut through
 *	Buddy 2's motion_estop_request(MOTION_ESTOP_OBSTACLE), released by
 *	this module once both wheels read stopped. It is NOT the latched
 *	system E-stop: no fault bit, no RESET needed (Tribunal 1: E-stop
 *	only on a sensor fault). The bypass is then re-planned with a rescan.
 */

#ifndef VC_OBSTACLE_H
#define VC_OBSTACLE_H

#include "../app.h"
#include "../subsystem2/motion_control.h"

typedef enum {
	VC_OBS_RUNNING = 0,	/* stay in VC_STATE_OBSTACLE */
	VC_OBS_DONE,		/* back to LINE_FOLLOW */
	VC_OBS_FAULT		/* raise VC_FAULT_OBSTACLE */
} vc_obs_result_t;

/* Steps, 4 bits (14 values), exported for the log and telemetry. */
typedef enum {
	VC_OBS_IDLE = 0,
	VC_OBS_WAIT_SCAN,
	VC_OBS_REVERSE,
	VC_OBS_ALIGN_SCAN,	/* square up before a rescan */
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

#define VC_OBS_STEP_BITS	4U

/* Tunables. TODO(measure) items need the real chassis. */
#define VC_OBS_SPEED_MM_S	150	/* bypass legs */
#define VC_OBS_REV_SPEED_MM_S	100	/* backing off (sensor faces forward) */
#define VC_OBS_TURN_DPS		90
#define VC_OBS_TURN_OUT_DEG	45
#define VC_OBS_TURN_STEEP_DEG	70	/* obstacle closer than the swing */
#define VC_OBS_EXTRA_MM		20	/* added to S5's swing */
#define VC_OBS_FRONT_MARGIN_MM	60	/* nearest - margin must cover the
					   forward travel of the OUT leg */
#define VC_OBS_SIDESTEP_DEG	10
#define VC_OBS_MAX_SIDESTEPS	3
#define VC_OBS_MAX_DEPTH_MM	600	/* longest obstacle we will pass */
#define VC_OBS_PASS_TAIL_MM	300	/* + rear of the car + margin: the
					   pass leg's hard cap is nearest +
					   depth + this (S5 normally says
					   PASSED long before) */
#define VC_OBS_IR_AHEAD_MM	60	/* TODO(measure): IR sensors ahead of
					   the wheel axle */
#define VC_OBS_IN_EXTRA_MM	100	/* drive this far past the expected
					   line before giving up on the IR */
#define VC_OBS_LEG_BLOCK_MM	250	/* front closer than this on a leg:
					   stop, square up, rescan. Covers
					   the look interval in WATCH (~45 mm
					   at 150 mm/s), the ramp-down (~30
					   mm), the 50 mm of car ahead of the
					   sensor, and a margin (sim C10) */
#define VC_OBS_BLOCK_AHEAD_MM	150	/* ...but only within what is left of
					   the leg plus this (car nose + stop) */
#define VC_OBS_MAX_STOP_SCANS	3	/* STOP/SLOW profiles in a row */
#define VC_OBS_MAX_REPLANS	3	/* square up + rescan, per obstacle */
#define VC_OBS_GONE_MS		1500	/* no event after STOP/SLOW and the
					   front is clear: obstacle gone */
#define VC_OBS_TIMEOUT_MS	45000	/* whole episode */
#define VC_OBS_STILL_MM_S	10	/* both wheels below: stopped */
#define VC_OBS_STILL_MS		100	/* ...for this long: release a cut */
#define VC_OBS_REJOIN_TOL_MM	50	/* requirement */

/* vc_task interface */
IMPORT void		vc_obs_begin(const motion_status_t *p_motion);
IMPORT void		vc_obs_events(UINT s5_events);
IMPORT vc_obs_result_t	vc_obs_step(const motion_status_t *p_motion,
				    bool ir_left_black, bool ir_right_black,
				    INT period_ms);
IMPORT void		vc_obs_leave(void);	/* left OBSTACLE early */

/* HARD_STOP motor cut, any VC state */
IMPORT void		vc_obs_hard_stop(const motion_status_t *p_motion);
IMPORT void		vc_obs_service_cut(const motion_status_t *p_motion,
					   INT period_ms);
IMPORT bool		vc_obs_cut_active(void);

/* Diagnostics */
IMPORT vc_obs_step_t	vc_obs_get_step(void);
IMPORT int32_t		vc_obs_last_rejoin_mm(void);	/* + = left of line */

#endif /* VC_OBSTACLE_H */
