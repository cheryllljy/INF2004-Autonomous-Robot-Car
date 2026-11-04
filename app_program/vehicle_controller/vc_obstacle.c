/*
 *	vc_obstacle.c
 *	VehicleController -- the obstacle part (Buddy 5 hook, v3.0).
 *	See vc_obstacle.h for the manoeuvre. Runs inside vc_task only.
 *
 *	Every motion is one Buddy 2 command (motion_turn / motion_move),
 *	issued when MotionControl is IDLE and finished when it is IDLE again
 *	(DONE, or STOPPED after we cut a leg short). A leg is cut short with
 *	motion_stop(), which ramps down; the HARD_STOP cut is separate and
 *	instant.
 */

#include "vc_obstacle.h"
#include "../subsystem5/subsystem5.h"

/* ------------------------------------------------------------------ */
/*  State (vc_task only)                                              */
/* ------------------------------------------------------------------ */

typedef enum {
	VO_CMD_NONE = 0,
	VO_CMD_TURN,
	VO_CMD_MOVE
} vo_cmd_t;

/* Why a leg was cut short (decides the next step). */
typedef enum {
	VO_CUT_NONE = 0,
	VO_CUT_PASSED,
	VO_CUT_SIDE_CLOSE,
	VO_CUT_BLOCKED,
	VO_CUT_LINE
} vo_cut_t;

typedef struct {
	vc_obs_step_t	step;
	int8_t		side;		/* +1 pass on the left, -1 right */

	/* one motion command in flight */
	vo_cmd_t	cmd;
	int32_t		p1;
	int32_t		p2;
	bool		issued;
	bool		seen_busy;
	bool		stopping;
	INT		cmd_ms;
	INT		cmd_limit_ms;
	int32_t		cmd_origin_mm;	/* distance when the move started */
	vo_cut_t	cut;

	/* latched S5 events, consumed by the steps */
	bool		ev_profiled;
	bool		ev_cleared;
	bool		ev_passed;
	bool		ev_side_close;
	bool		ev_fault;

	/* dead reckoning, 1/256 mm, frame = pose at detection */
	int32_t		h0_deg;
	int32_t		last_dist_mm;
	int32_t		x_q;
	int32_t		y_q;

	/* the plan */
	uint16_t	nearest_mm;
	uint16_t	swing_mm;
	int32_t		out_deg;
	int32_t		y_scan_q;	/* y when the profile was taken */
	int32_t		pass_end_y_q;	/* hard cap on the pass leg */
	int32_t		pass_fallback_y_q;
	int32_t		line_hit_dist_mm;
	bool		line_hit;
	bool		watching;
	bool		interrupted;	/* a HARD_STOP cut hit a leg */
	uint8_t		stop_scans;
	uint8_t		sidesteps;
	uint8_t		replans;
	INT		dwell_ms;

	INT		step_ms;
	INT		total_ms;
	INT		quiet_ms;	/* since the last S5 event */
	int32_t		rejoin_mm;
} vc_obs_t;

LOCAL vc_obs_t	vo;
LOCAL bool	g_cut;			/* HARD_STOP motor cut active */
LOCAL INT	g_still_ms;
LOCAL int32_t	g_last_rejoin_mm;

#define VO_Q		256		/* pose in 1/256 mm: no truncation
					   drift over a few thousand steps */
#define VO_DWELL_MS	150		/* after a turn, let S5 take a fresh
					   front ping before trusting it */
#define VO_CMD_START_MS	1000		/* MotionControl never went busy */

/* ------------------------------------------------------------------ */
/*  Small helpers                                                     */
/* ------------------------------------------------------------------ */

LOCAL int32_t vo_abs(int32_t v)
{
	return (v < 0) ? -v : v;
}

LOCAL int32_t vo_heading(const motion_status_t *m)
{
	return m->heading_deg - vo.h0_deg;
}

LOCAL int32_t vo_x_mm(void)
{
	return vo.x_q / VO_Q;
}

LOCAL void vo_goto(vc_obs_step_t next)
{
	if (next != vo.step) {
		tm_printf((UB*)"[VC-OBS] step %d -> %d  (x %d y %d mm)\n",
			  (INT)vo.step, (INT)next, (INT)vo_x_mm(),
			  (INT)(vo.y_q / VO_Q));
		vo.step    = next;
		vo.step_ms = 0;
	}
}

LOCAL void vo_pose_update(const motion_status_t *m)
{
	const int32_t d  = m->distance_total_mm;
	const int32_t ds = d - vo.last_dist_mm;
	const int32_t h  = vo_heading(m) * 10;	/* 0.1 deg */

	vo.last_dist_mm = d;

	/* x is +left: heading +h (left) moves the car to +x. */
	vo.x_q += (ds * VO_Q * obstacle_sin_q15_ddeg(h)) / 32768;
	vo.y_q += (ds * VO_Q * obstacle_sin_q15_ddeg(900 - h)) / 32768;
}

LOCAL void vo_cmd_set(vo_cmd_t cmd, int32_t p1, int32_t p2)
{
	vo.cmd       = cmd;
	vo.p1        = p1;
	vo.p2        = p2;
	vo.issued    = false;
	vo.seen_busy = false;
	vo.stopping  = false;
	vo.cmd_ms    = 0;
	vo.cut       = VO_CUT_NONE;

	if (cmd == VO_CMD_TURN) {
		vo.cmd_limit_ms = 2000 + ((vo_abs(p1) * 1000) /
					  VC_OBS_TURN_DPS) * 2;
	} else {
		vo.cmd_limit_ms = 2000 + ((vo_abs(p1) * 1000) /
					  ((p2 > 0) ? p2 : 1)) * 2;
	}
}

/* Turn to an absolute heading in the detection frame. Under 2 deg is
   within Buddy 2's turn tolerance (NFR13): nothing to do. */
LOCAL void vo_turn_to(const motion_status_t *m, int32_t heading_deg)
{
	const int32_t a = heading_deg - vo_heading(m);

	if (vo_abs(a) < 2) {
		vo_cmd_set(VO_CMD_NONE, 0, 0);
	} else {
		vo_cmd_set(VO_CMD_TURN, a, VC_OBS_TURN_DPS);
	}
}

/* Cut the current leg short; the step learns why from vo.cut. */
LOCAL void vo_cmd_cut(vo_cut_t why)
{
	if (!vo.stopping && (vo.cmd != VO_CMD_NONE)) {
		vo.stopping = true;
		vo.cut      = why;
		(void)motion_stop();
	}
}

/*
 * Drive the command in flight. 0 = still going, 1 = finished (target
 * reached, or stopped by us), -1 = refused / failed / timed out.
 */
LOCAL INT vo_cmd_run(const motion_status_t *m, INT period_ms)
{
	INT r = 0;

	if (vo.cmd == VO_CMD_NONE) {
		return 1;
	}

	vo.cmd_ms += period_ms;

	if (!vo.issued) {
		if (m->state == MOTION_STATE_IDLE) {
			const motion_err_t er = (vo.cmd == VO_CMD_TURN) ?
				motion_turn(vo.p1, vo.p2) :
				motion_move(vo.p1, vo.p2);

			if (er == MOTION_OK) {
				vo.issued        = true;
				vo.cmd_ms        = 0;
				vo.cmd_origin_mm = m->distance_total_mm;
			}
		}
		if (!vo.issued && (vo.cmd_ms > VO_CMD_START_MS)) {
			r = -1;
		}
	} else if (m->state != MOTION_STATE_IDLE) {
		vo.seen_busy = true;
		if (vo.cmd_ms > vo.cmd_limit_ms) {
			(void)motion_stop();
			r = -1;
		}
	} else if (vo.seen_busy || (vo.cmd_ms > VO_CMD_START_MS)) {
		/* Back to IDLE. A move so short it never showed busy still
		   ends DONE. */
		r = ((m->last_result == MOTION_RESULT_DONE) ||
		     (m->last_result == MOTION_RESULT_STOPPED)) ? 1 : -1;
	} else {
		/* Issued, MotionControl has not picked it up yet. */
	}

	if (r != 0) {
		if (r < 0) {
			tm_printf((UB*)"[VC-OBS] motion command %d (%d) failed,"
				  " result %d\n", (INT)vo.cmd, (INT)vo.p1,
				  (INT)m->last_result);
		}
		vo.cmd = VO_CMD_NONE;
	}

	return r;
}

LOCAL void vo_fail(const char *why)
{
	tm_printf((UB*)"[VC-OBS] FAIL: %s\n", why);
	if (vo.watching) {
		obstacle_watch_end();
		vo.watching = false;
	}
	(void)motion_stop();
	vo_goto(VC_OBS_FAILED);
}

/* Square up to the detection heading, then ask S5 for a fresh scan. */
LOCAL void vo_replan(const motion_status_t *m, const char *why)
{
	tm_printf((UB*)"[VC-OBS] replan: %s\n", why);
	if (vo.watching) {
		obstacle_watch_end();
		vo.watching = false;
	}
	vo.replans++;
	if (vo.replans > VC_OBS_MAX_REPLANS) {
		vo_fail("too many replans");
	} else {
		vo_turn_to(m, 0);
		vo_goto(VC_OBS_ALIGN_SCAN);
	}
}

/* Front range along the current heading (S5 keeps pinging ahead in
   ASSIST and on every second look in WATCH). Only something that lies
   on what is LEFT of this leg counts: on the way back to the line a
   wall beyond the line is in the beam but not in the way (sim C5). */
LOCAL bool vo_front_blocked(const motion_status_t *m, int32_t leg_left_mm)
{
	const uint16_t f = obstacle_get_front_mm();
	int32_t limit = VC_OBS_LEG_BLOCK_MM;
	int32_t left  = leg_left_mm;

	if ((left < 0) && (vo.cmd == VO_CMD_MOVE) && vo.issued) {
		left = vo_abs(vo.p1) -
		       vo_abs(m->distance_total_mm - vo.cmd_origin_mm);
	}
	if (left >= 0) {
		left = ((left > 0) ? left : 0) + VC_OBS_BLOCK_AHEAD_MM;
		if (left < limit) {
			limit = left;
		}
	}

	return (f != OBSTACLE_RANGE_CLEAR) && (f != OBSTACLE_RANGE_UNKNOWN) &&
	       ((int32_t)f < limit);
}

/* ------------------------------------------------------------------ */
/*  Plan from a profile                                               */
/* ------------------------------------------------------------------ */

LOCAL bool vo_plan_bypass(const motion_status_t *m,
			  const obstacle_profile_t *p)
{
	int32_t lateral;

	vo.side       = (p->action == OBSTACLE_ACT_BYPASS_LEFT) ? 1 : -1;
	vo.nearest_mm = p->nearest_mm;
	vo.swing_mm   = (vo.side > 0) ? p->swing_left_mm : p->swing_right_mm;
	vo.y_scan_q  = vo.y_q;
	vo.sidesteps  = 0U;

	lateral = (int32_t)vo.swing_mm + VC_OBS_EXTRA_MM;

	/* At 45 deg the car moves forward as much as sideways. If the
	   obstacle is nearer than that, turn out steeper. */
	vo.out_deg = (((int32_t)vo.nearest_mm - VC_OBS_FRONT_MARGIN_MM) >=
		      lateral) ? VC_OBS_TURN_OUT_DEG : VC_OBS_TURN_STEEP_DEG;

	/* Pass leg ends, at the latest, past the deepest obstacle we
	   accept; the fallback (no side data) is S5's v2 estimate. */
	vo.pass_end_y_q = vo.y_q + VO_Q *
		((int32_t)vo.nearest_mm + VC_OBS_MAX_DEPTH_MM +
		 VC_OBS_PASS_TAIL_MM);
	vo.pass_fallback_y_q = vo.y_q + VO_Q *
		(int32_t)obstacle_pass_distance_mm(p);

	tm_printf((UB*)"[VC-OBS] bypass %s: nearest %d mm, swing %d mm,"
		  " out %d deg\n", (vo.side > 0) ? "LEFT" : "RIGHT",
		  (INT)vo.nearest_mm, (INT)vo.swing_mm, (INT)vo.out_deg);

	vo_turn_to(m, vo.side * vo.out_deg);

	return (vo.swing_mm > 0U);
}

/* Lateral target of the OUT leg, as a distance along the out heading. */
LOCAL int32_t vo_out_distance(void)
{
	const int32_t target = vo.side *
		((int32_t)vo.swing_mm + VC_OBS_EXTRA_MM);
	const int32_t need = vo_abs(target - vo_x_mm());

	return (need * 32768) / obstacle_sin_q15_ddeg(vo.out_deg * 10);
}

LOCAL void vo_start_pass(const motion_status_t *m)
{
	int32_t cap = (vo.pass_end_y_q - vo.y_q) / VO_Q;

	(void)m;

	if (cap < 50) {
		cap = 50;
	}

	if (!vo.watching) {
		/* We pass on the left -> the obstacle is on our right. */
		obstacle_watch_start(vo.side < 0);
		vo.watching = true;
	}

	vo.ev_passed     = false;
	vo.ev_side_close = false;
	vo_cmd_set(VO_CMD_MOVE, cap, VC_OBS_SPEED_MM_S);
	vo_goto(VC_OBS_PASS);
}

LOCAL void vo_start_in(const motion_status_t *m)
{
	if (vo.watching) {
		obstacle_watch_end();
		vo.watching = false;
	}
	vo_turn_to(m, -vo.side * VC_OBS_TURN_OUT_DEG);
	vo_goto(VC_OBS_TURN_IN);
}

/* ------------------------------------------------------------------ */
/*  Public: vc_task hooks                                             */
/* ------------------------------------------------------------------ */

EXPORT void vc_obs_begin(const motion_status_t *p_motion)
{
	const bool cut = g_cut;

	vo = (vc_obs_t){0};
	vo.h0_deg       = p_motion->heading_deg;
	vo.last_dist_mm = p_motion->distance_total_mm;
	vo.step         = VC_OBS_IDLE;
	g_cut           = cut;		/* a cut may already be on */
	vo_goto(VC_OBS_WAIT_SCAN);
}

EXPORT void vc_obs_events(UINT s5_events)
{
	if (s5_events != 0U) {
		vo.quiet_ms = 0;
	}
	if (s5_events & S5_EVT_PROFILED)   { vo.ev_profiled   = true; }
	if (s5_events & S5_EVT_CLEARED)    { vo.ev_cleared    = true; }
	if (s5_events & S5_EVT_PASSED)     { vo.ev_passed     = true; }
	if (s5_events & S5_EVT_SIDE_CLOSE) { vo.ev_side_close = true; }
	if (s5_events & S5_EVT_FAULT)      { vo.ev_fault      = true; }
}

EXPORT void vc_obs_leave(void)
{
	if ((vo.step != VC_OBS_IDLE) && (vo.step != VC_OBS_FINISHED)) {
		if (vo.watching) {
			obstacle_watch_end();
		}
		obstacle_resume();	/* S5 back to GUARD */
	}
	vo.watching = false;
	vo.cmd      = VO_CMD_NONE;
	vo.step     = VC_OBS_IDLE;
}

EXPORT vc_obs_result_t vc_obs_step(const motion_status_t *p_motion,
				   bool ir_left_black, bool ir_right_black,
				   INT period_ms)
{
	const motion_status_t *m = p_motion;
	vc_obs_result_t res = VC_OBS_RUNNING;
	INT r;

	obstacle_assist_kick();		/* S5 watchdog: VC is alive */
	vo_pose_update(m);
	vo.step_ms  += period_ms;
	vo.total_ms += period_ms;
	vo.quiet_ms += period_ms;

	if (vo.step == VC_OBS_FAILED) {
		return VC_OBS_FAULT;
	}
	if (vo.step == VC_OBS_FINISHED) {
		return VC_OBS_DONE;
	}

	if (vo.total_ms > VC_OBS_TIMEOUT_MS) {
		vo_fail("obstacle episode timed out");
		return VC_OBS_FAULT;
	}
	if (vo.ev_fault) {
		/* S5 FAULT: its own handling (E-stop bit or latched mission
		   fault) is in read_obstacle(); the bypass is over. */
		/* Diagnostic (Buddy 3, 2026-11-04): which S5 fault, since
		   S5 itself prints nothing that reaches the MQTT log. */
		{
			obstacle_health_t h;
			static const char *const names[] = {
				"none", "SENSOR (no echo)",
				"STOP_TIMEOUT (car not still <= 5 mm/s within 1 s)",
				"REVERSE_LIMIT (third reverse)",
				"ASSIST_TIMEOUT (no VC request for 5 s)" };

			obstacle_get_health(&h);
			tm_printf((UB*)"[VC-OBS] S5 fault %u: %s, S5 state %u\n",
				  (UINT)h.fault,
				  (h.fault < (sizeof(names) / sizeof(names[0]))) ?
				  names[h.fault] : "?", (UINT)h.state);
		}
		vo_fail("ObstacleScan fault");
		return VC_OBS_FAULT;
	}

	if (g_cut) {
		/* HARD_STOP: motors cut. Wait for the release. */
		vo.interrupted = true;
		return res;
	}

	if (vo.interrupted) {
		vo.interrupted = false;
		vo.cmd = VO_CMD_NONE;
		if ((vo.step != VC_OBS_WAIT_SCAN) && (vo.step != VC_OBS_REVERSE) &&
		    (vo.step != VC_OBS_ALIGN_SCAN)) {
			vo_replan(m, "hard stop during a leg");
		}
	}

	switch (vo.step) {
	case VC_OBS_WAIT_SCAN:
		if (vo.ev_cleared) {
			vo.ev_cleared = false;
			obstacle_resume();
			tm_printf((UB*)"[VC-OBS] path clear\n");
			vo.rejoin_mm = vo_x_mm();
			g_last_rejoin_mm = vo.rejoin_mm;
			vo_goto(VC_OBS_FINISHED);
			res = VC_OBS_DONE;
		} else if (vo.ev_profiled) {
			obstacle_profile_t p;
			obstacle_watch_t   w;

			vo.ev_profiled = false;
			(void)obstacle_get_profile(&p);
			obstacle_get_watch(&w);

			switch (p.action) {
			case OBSTACLE_ACT_BYPASS_LEFT:
			case OBSTACLE_ACT_BYPASS_RIGHT:
				vo.stop_scans = 0U;
				if (vo_plan_bypass(m, &p)) {
					vo_goto(VC_OBS_TURN_OUT);
				} else {
					vo_fail("bypass with no swing");
				}
				break;
			case OBSTACLE_ACT_REVERSE:
				tm_printf((UB*)"[VC-OBS] reverse %d mm to "
					  "remeasure (nearest %d mm)\n",
					  (INT)w.reverse_mm, (INT)p.nearest_mm);
				vo_cmd_set(VO_CMD_MOVE,
					   -(int32_t)w.reverse_mm,
					   VC_OBS_REV_SPEED_MM_S);
				vo_goto(VC_OBS_REVERSE);
				break;
			case OBSTACLE_ACT_CONTINUE:
				obstacle_resume();
				vo_goto(VC_OBS_FINISHED);
				res = VC_OBS_DONE;
				break;
			default:
				/* STOP / SLOW: S5 will look again by itself. */
				vo.stop_scans++;
				tm_printf((UB*)"[VC-OBS] profile says %d, waiting"
					  " (%d)\n", (INT)p.action,
					  (INT)vo.stop_scans);
				if (vo.stop_scans >= VC_OBS_MAX_STOP_SCANS) {
					vo_fail("no usable profile");
				}
				break;
			}
		} else if ((vo.stop_scans > 0U) &&
			   (vo.quiet_ms > VC_OBS_GONE_MS)) {
			const uint16_t f = obstacle_get_front_mm();

			if ((f == OBSTACLE_RANGE_CLEAR) ||
			    ((f != OBSTACLE_RANGE_UNKNOWN) &&
			     (f > (uint16_t)OBSTACLE_TRIGGER_MM))) {
				tm_printf((UB*)"[VC-OBS] obstacle gone\n");
				obstacle_resume();
				vo_goto(VC_OBS_FINISHED);
				res = VC_OBS_DONE;
			}
		} else {
			/* S5 is waiting for the stop, or sweeping. */
		}
		break;

	case VC_OBS_REVERSE:
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			obstacle_rescan();
			vo_goto(VC_OBS_WAIT_SCAN);
		} else if (r < 0) {
			vo_fail("reverse refused");
		} else {
			/* Backing off. */
		}
		break;

	case VC_OBS_ALIGN_SCAN:
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			vo.ev_profiled = false;
			vo.ev_cleared  = false;
			obstacle_rescan();
			vo_goto(VC_OBS_WAIT_SCAN);
		} else if (r < 0) {
			vo_fail("could not square up");
		} else {
			/* Turning. */
		}
		break;

	case VC_OBS_TURN_OUT:
		r = (vo.cmd == VO_CMD_NONE) ? 1 : vo_cmd_run(m, period_ms);
		if (r > 0) {
			vo.dwell_ms += period_ms;
		}
		if ((r > 0) && (vo.dwell_ms >= VO_DWELL_MS)) {
			const int32_t d = vo_out_distance();
			const uint16_t f = obstacle_get_front_mm();

			vo.dwell_ms = 0;

			/* The sensor now looks along the out leg. */
			if ((f != OBSTACLE_RANGE_CLEAR) &&
			    (f != OBSTACLE_RANGE_UNKNOWN) &&
			    ((int32_t)f < (d + VC_OBS_FRONT_MARGIN_MM))) {
				if (vo.out_deg != VC_OBS_TURN_STEEP_DEG) {
					/* Clipping the near corner: steeper. */
					tm_printf((UB*)"[VC-OBS] out leg blocked "
						  "at %d mm, turning out steeper\n",
						  (INT)f);
					vo.out_deg = VC_OBS_TURN_STEEP_DEG;
					vo_turn_to(m, vo.side * vo.out_deg);
				} else {
					vo_replan(m, "out leg blocked");
				}
			} else {
				vo_cmd_set(VO_CMD_MOVE, d, VC_OBS_SPEED_MM_S);
				vo_goto(VC_OBS_OUT);
			}
		} else if (r < 0) {
			vo_fail("turn out refused");
		} else {
			/* Turning. */
		}
		break;

	case VC_OBS_OUT:
		if (vo_front_blocked(m, -1)) {
			vo_cmd_cut(VO_CUT_BLOCKED);
		}
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			if (vo.cut == VO_CUT_BLOCKED) {
				vo_replan(m, "something ahead on the out leg");
			} else {
				vo_turn_to(m, 0);
				vo_goto(VC_OBS_TURN_PAR);
			}
		} else if (r < 0) {
			vo_fail("out leg refused");
		} else {
			/* Driving out. */
		}
		break;

	case VC_OBS_TURN_PAR:
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			vo_start_pass(m);
		} else if (r < 0) {
			vo_fail("turn parallel refused");
		} else {
			/* Turning. */
		}
		break;

	case VC_OBS_PASS:
		if (vo.ev_passed) {
			vo.ev_passed = false;
			vo.watching  = false;	/* S5 left WATCH by itself */
			vo_cmd_cut(VO_CUT_PASSED);
		} else if (vo.ev_side_close) {
			vo.ev_side_close = false;
			vo_cmd_cut(VO_CUT_SIDE_CLOSE);
		} else if (vo_front_blocked(m, -1)) {
			vo_cmd_cut(VO_CUT_BLOCKED);
		} else {
			/* Driving past. */
		}

		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			obstacle_watch_t w;

			obstacle_get_watch(&w);

			if (vo.cut == VO_CUT_PASSED) {
				tm_printf((UB*)"[VC-OBS] passed (closest face"
					  " %d mm from the sensor axis)\n",
					  (INT)w.lateral_mm);
				vo_start_in(m);
			} else if (vo.cut == VO_CUT_SIDE_CLOSE) {
				vo.sidesteps++;
				if (vo.sidesteps > VC_OBS_MAX_SIDESTEPS) {
					vo_fail("kept drifting onto the obstacle");
				} else {
					tm_printf((UB*)"[VC-OBS] too close to the"
						  " side (%d mm), steering away\n",
						  (INT)w.lateral_mm);
					vo_cmd_set(VO_CMD_TURN,
						   vo.side * VC_OBS_SIDESTEP_DEG,
						   VC_OBS_TURN_DPS);
					vo_goto(VC_OBS_SIDESTEP);
				}
			} else if (vo.cut == VO_CUT_BLOCKED) {
				vo_replan(m, "something ahead on the pass leg");
			} else if ((w.phase == S5_WATCH_BLIND) &&
				   (vo.y_q >= vo.pass_fallback_y_q)) {
				/* Never saw it beside us: trust the profile's
				   estimate, as v2 did. */
				tm_printf((UB*)"[VC-OBS] no side data, using the"
					  " profile pass distance\n");
				vo_start_in(m);
			} else {
				vo_fail("pass leg ended, obstacle still beside us");
			}
		} else if (r < 0) {
			vo_fail("pass leg refused");
		} else {
			/* Driving past. */
		}
		break;

	case VC_OBS_SIDESTEP:
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			vo_start_pass(m);
		} else if (r < 0) {
			vo_fail("side step refused");
		} else {
			/* Turning away. */
		}
		break;

	case VC_OBS_TURN_IN:
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			/* Distance to the line along -s * 45 deg, plus a
			   margin in case dead reckoning is short. */
			const int32_t d = ((vo_abs(vo_x_mm()) * 32768) /
					   obstacle_sin_q15_ddeg(
						   VC_OBS_TURN_OUT_DEG * 10)) +
					  VC_OBS_IN_EXTRA_MM;

			vo.line_hit = false;
			vo_cmd_set(VO_CMD_MOVE, d, VC_OBS_SPEED_MM_S);
			vo_goto(VC_OBS_IN);
		} else if (r < 0) {
			vo_fail("turn in refused");
		} else {
			/* Turning. */
		}
		break;

	case VC_OBS_IN:
		if (!vo.line_hit && (ir_left_black || ir_right_black)) {
			vo.line_hit         = true;
			vo.line_hit_dist_mm = m->distance_total_mm;
			tm_printf((UB*)"[VC-OBS] line found (x %d mm)\n",
				  (INT)vo_x_mm());
		}
		if (vo.line_hit &&
		    ((m->distance_total_mm - vo.line_hit_dist_mm) >=
		     VC_OBS_IR_AHEAD_MM)) {
			/* The IR is ahead of the axle: carry on until the
			   AXLE is on the line, then square up. */
			vo_cmd_cut(VO_CUT_LINE);
		} else if (vo_front_blocked(m, vo.line_hit ?
			   (VC_OBS_IR_AHEAD_MM - (m->distance_total_mm -
						  vo.line_hit_dist_mm)) :
			   ((vo_abs(vo_x_mm()) * 32768) /
			    obstacle_sin_q15_ddeg(VC_OBS_TURN_OUT_DEG * 10)))) {
			/* Only what lies before the line counts: the car
			   turns to the original heading there. */
			vo_cmd_cut(VO_CUT_BLOCKED);
		} else {
			/* Heading back. */
		}

		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			if (vo.cut == VO_CUT_BLOCKED) {
				vo_replan(m, "something ahead on the way back");
			} else {
				if (!vo.line_hit) {
					tm_printf((UB*)"[VC-OBS] IR never saw "
						  "the line; dead reckoning\n");
				}
				vo_turn_to(m, 0);
				vo_goto(VC_OBS_ALIGN);
			}
		} else if (r < 0) {
			vo_fail("way back refused");
		} else {
			/* Heading back. */
		}
		break;

	case VC_OBS_ALIGN:
		r = vo_cmd_run(m, period_ms);
		if (r > 0) {
			vo.rejoin_mm     = vo_x_mm();
			g_last_rejoin_mm = vo.rejoin_mm;
			tm_printf((UB*)"[VC-OBS] rejoined: %d mm from the line"
				  " (%s, tolerance %d), %d ms\n",
				  (INT)vo.rejoin_mm,
				  (vo_abs(vo.rejoin_mm) <= VC_OBS_REJOIN_TOL_MM) ?
				  "PASS" : "FAIL", (INT)VC_OBS_REJOIN_TOL_MM,
				  (INT)vo.total_ms);
			obstacle_resume();
			vo_goto(VC_OBS_FINISHED);
			res = VC_OBS_DONE;
		} else if (r < 0) {
			vo_fail("align refused");
		} else {
			/* Turning. */
		}
		break;

	case VC_OBS_IDLE:
	case VC_OBS_FINISHED:
	case VC_OBS_FAILED:
	default:
		break;
	}

	if (vo.step == VC_OBS_FAILED) {
		res = VC_OBS_FAULT;
	}

	return res;
}

/* ------------------------------------------------------------------ */
/*  HARD_STOP motor cut                                               */
/* ------------------------------------------------------------------ */

EXPORT void vc_obs_hard_stop(const motion_status_t *p_motion)
{
	/* Only a car driving INTO the obstacle needs the cut. Backing away
	   from it (REVERSE) or turning on the spot next to it, S5 keeps
	   reporting "under 120 mm" on every ping, and cutting then would
	   pin the car in place. */
	const int32_t v = ((int32_t)p_motion->speed_l_mmps +
			   (int32_t)p_motion->speed_r_mmps) / 2;

	if (!g_cut && (v > VC_OBS_STILL_MM_S)) {
		tm_printf((UB*)"[VC-OBS] HARD STOP: motors cut (%d mm/s)\n",
			  (INT)v);
		motion_estop_request(MOTION_ESTOP_OBSTACLE);
		g_cut      = true;
		g_still_ms = 0;
	}
}

EXPORT void vc_obs_service_cut(const motion_status_t *p_motion,
			       INT period_ms)
{
	if (g_cut) {
		if ((vo_abs(p_motion->speed_l_mmps) < VC_OBS_STILL_MM_S) &&
		    (vo_abs(p_motion->speed_r_mmps) < VC_OBS_STILL_MM_S)) {
			g_still_ms += period_ms;
		} else {
			g_still_ms = 0;
		}

		if (g_still_ms >= VC_OBS_STILL_MS) {
			/* Stopped: release our bit only. MotionControl
			   refuses the reset while any other cause is set. */
			motion_estop_clear(MOTION_ESTOP_OBSTACLE);
			(void)motion_reset();
			g_cut = false;
			tm_printf((UB*)"[VC-OBS] hard stop released\n");
		}
	}
}

EXPORT bool vc_obs_cut_active(void)
{
	return g_cut;
}

EXPORT vc_obs_step_t vc_obs_get_step(void)
{
	return vo.step;
}

EXPORT int32_t vc_obs_last_rejoin_mm(void)
{
	return g_last_rejoin_mm;
}
