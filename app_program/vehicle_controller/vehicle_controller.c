/*
 *	vehicle_controller.c
 *	VehicleController task: one 10 ms loop that reads every subsystem,
 *	decides, and commands Buddy 2.  General structure for the team; each
 *	"OWNER" hook is filled in by that buddy.
 *
 *	Each pass:
 *	  1. faults    any cause bit in flg_vc        -> ESTOP (latched)
 *	  2. commands  START / STOP / RESET requests   -> state change
 *	  3. inputs    Buddy 2 status, Buddy 4 IMU, Buddy 5 events
 *	  4. Buddy 3   hand over context, read line + barcode
 *	  5. act       the current state's motor command
 *	  6. publish   one 32-bit status word
 */

#include "vehicle_controller.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/b3_port.h"
#include "../subsystem5/subsystem5.h"

EXPORT ID	flg_vc = 0;

LOCAL volatile uint32_t	g_status_word;	/* written only by vc_task */

#define VC_TURN_ANGLE_DEG	90
#define VC_TURN_TIMEOUT_MS	5000

typedef struct {
	vc_state_t	state;
	UINT		faults;		/* latched causes seen this pass */
	motion_status_t	motion;
	b3_snapshot_t	line;
	b3_nav_command_t last_nav;
	char		last_symbol;
	uint32_t	last_seq;
	INT		state_ms;	/* time in the current state */
	INT		since_send_ms;
	int16_t		sent_l;
	int16_t		sent_r;
} vc_t;

LOCAL vc_t	vc;

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

EXPORT void vc_raise_fault(UINT causes)
{
	causes &= VC_FAULT_MASK;
	if ((flg_vc > 0) && (causes != 0U)) {
		(void)tk_set_flg(flg_vc, causes);	/* atomic in the kernel */
	}
}

EXPORT void vc_post_command(vc_cmd_t cmd)
{
	if ((flg_vc > 0) && (cmd > VC_CMD_NONE) && (cmd < VC_CMD_COUNT)) {
		(void)tk_set_flg(flg_vc, VC_FLG_CMD(cmd));
	}
}

EXPORT uint32_t vc_get_status_word(void)
{
	return g_status_word;			/* one aligned 32-bit load */
}

/* OWNER: Buddy 1 -- fill wifi/mqtt and the metric, then b1_telemetry_pack() */
EXPORT void vc_fill_telemetry(B1_Telemetry *p_out)
{
	uint32_t w = g_status_word;

	p_out->controller_state = (uint8_t)VC_SW_STATE(w);
	p_out->fault_flags      = VC_SW_FAULTS(w);
	p_out->wifi_connected   = 0U;		/* TODO Buddy 1 */
	p_out->mqtt_connected   = 0U;		/* TODO Buddy 1 */
	p_out->metric_0_100     = 0U;		/* TODO team: which measurement */
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

LOCAL void enter(vc_state_t next)
{
	if (next != vc.state) {
		tm_printf((UB*)"[VC] state %d -> %d\n", vc.state, next);
		vc.state = next;
		vc.state_ms = 0;
	}
}

LOCAL void stop_motors(void)
{
	(void)motion_stop();
	vc.sent_l = 0;
	vc.sent_r = 0;
}

/* Send Buddy 3's wheel speeds on change, and refresh them periodically */
LOCAL void drive(int16_t left, int16_t right)
{
	if ((left != vc.sent_l) || (right != vc.sent_r) ||
	    (vc.since_send_ms >= VC_SPEED_REFRESH_MS)) {
		if (motion_set_wheel_speeds(left, right) == MOTION_OK) {
			vc.sent_l = left;
			vc.sent_r = right;
			vc.since_send_ms = 0;
		}
	}
}

/* ------------------------------------------------------------------ */
/*  1-2. Faults and commands                                          */
/* ------------------------------------------------------------------ */

LOCAL void handle_flags(void)
{
	T_RFLG	rflg;
	UINT	cmds;

	if (tk_ref_flg(flg_vc, &rflg) < E_OK) {
		return;
	}
	vc.faults = rflg.flgptn & VC_FAULT_MASK;
	cmds = rflg.flgptn & VC_FLG_CMD_MASK;
	if (cmds != 0U) {
		(void)tk_clr_flg(flg_vc, ~cmds);	/* consume the requests */
	}

	if (cmds & VC_FLG_CMD(VC_CMD_RESET)) {
		/* Only the VehicleController clears fault causes. A source
		   that is still faulty sets its bit again next pass. */
		(void)tk_clr_flg(flg_vc, ~(UINT)VC_FAULT_MASK);
		vc.faults = 0U;
		(void)motion_reset();
		if (flg_obstacle > 0) {
			obstacle_reset_fault();
		}
		enter(VC_STATE_IDLE);
		return;
	}
	if (vc.faults != 0U) {
		if (vc.state != VC_STATE_ESTOP) {
			stop_motors();
			enter(VC_STATE_ESTOP);
		}
		return;
	}
	if (cmds & VC_FLG_CMD(VC_CMD_STOP)) {
		stop_motors();
		enter(VC_STATE_STOPPED);
	} else if ((cmds & VC_FLG_CMD(VC_CMD_START)) &&
		   ((vc.state == VC_STATE_IDLE) ||
		    (vc.state == VC_STATE_STOPPED))) {
		enter(VC_STATE_LINE_FOLLOW);
	}
}

/* ------------------------------------------------------------------ */
/*  3. Inputs from the other subsystems                               */
/* ------------------------------------------------------------------ */

/* OWNER: Buddy 2 -- motion status and motion faults */
LOCAL void read_motion(void)
{
	motion_get_status(&vc.motion);
	if (vc.motion.state == MOTION_STATE_FAULT) {
		vc_raise_fault((vc.motion.fault == MOTION_FAULT_ENCODER_TIMEOUT) ?
			       VC_FAULT_ENCODER : VC_FAULT_MOTOR_STALL);
	}
}

/* OWNER: Buddy 4 -- IMU yaw rate / terrain for Buddy 3, IMU fault.
   Fill the fields and call vc_raise_fault(VC_FAULT_IMU) on a fault. */
LOCAL void read_imu(b3_context_t *p_ctx)
{
	p_ctx->imu_valid        = false;	/* TODO Buddy 4 */
	p_ctx->yaw_rate_mdeg_s  = 0;
	p_ctx->terrain_unstable = false;
}

/* OWNER: Buddy 5 -- obstacle events (IF-S5-1). Skipped until S5 runs. */
LOCAL void read_obstacle(void)
{
	UINT	ptn = 0U;

	if (flg_obstacle <= 0) {
		return;
	}
	if (tk_wai_flg(flg_obstacle, S5_EVT_ALL, TWF_ORW | TWF_BITCLR,
		       &ptn, TMO_POL) < E_OK) {
		return;				/* nothing new */
	}
	if ((ptn & (S5_EVT_DETECTED | S5_EVT_HARD_STOP)) &&
	    ((vc.state == VC_STATE_LINE_FOLLOW) ||
	     (vc.state == VC_STATE_TURNING))) {
		stop_motors();			/* normal stop, not an E-stop */
		enter(VC_STATE_OBSTACLE);
	}
	if (ptn & S5_EVT_PROFILED) {
		/* TODO Buddy 5 + Buddy 2: bypass using obstacle_get_profile() */
	}
	if ((ptn & S5_EVT_CLEARED) && (vc.state == VC_STATE_OBSTACLE)) {
		obstacle_resume();
		enter(VC_STATE_LINE_FOLLOW);
	}
	/* S5_EVT_FAULT: S5 sets VC_FAULT_NO_ECHO itself (registered in
	   vc_init), so the fault path above handles it. */
}

/* ------------------------------------------------------------------ */
/*  4. Buddy 3: line following and barcodes                           */
/* ------------------------------------------------------------------ */

LOCAL void exchange_line(void)
{
	b3_context_t	ctx = {0};
	b3_nav_event_t	ev;

	ctx.time_us            = b3_port_time_us();
	ctx.follow_enabled     = (vc.state == VC_STATE_LINE_FOLLOW);
	ctx.scan_enabled       = (vc.state == VC_STATE_LINE_FOLLOW);
	ctx.stop_requested     = (vc.state == VC_STATE_ESTOP) ||
				 (vc.state == VC_STATE_STOPPED);
	ctx.motion_valid       = (vc.motion.state != MOTION_STATE_FAULT);
	ctx.left_measured_mm_s = vc.motion.speed_l_mmps;
	ctx.right_measured_mm_s = vc.motion.speed_r_mmps;
	read_imu(&ctx);
	buddy3_set_context(&ctx);
	buddy3_get_snapshot(&vc.line);

	/* Serious Buddy 3 faults latch the E-stop. A plain stop_required
	   (stale context, follow off) just means "do not drive". */
	if ((vc.line.task_error != 0) || (vc.line.dropped_events != 0U)) {
		vc_raise_fault(VC_FAULT_LINE);
	}

	while (buddy3_pop_navigation(&ev)) {
		vc.last_nav    = ev.command;
		vc.last_symbol = ev.barcode.symbol;
		vc.last_seq    = ev.sequence;
		tm_printf((UB*)"[VC] barcode '%c' -> command %d%s\n",
			  ev.barcode.symbol, ev.command,
			  ev.navigation_allowed ? "" : " (not allowed)");
		if (!ev.navigation_allowed || (vc.state != VC_STATE_LINE_FOLLOW)) {
			continue;
		}
		switch (ev.command) {
		case B3_NAV_LEFT:
			stop_motors();
			(void)motion_turn_left(VC_TURN_ANGLE_DEG);
			enter(VC_STATE_TURNING);
			break;
		case B3_NAV_RIGHT:
			stop_motors();
			(void)motion_turn_right(VC_TURN_ANGLE_DEG);
			enter(VC_STATE_TURNING);
			break;
		case B3_NAV_UTURN:
			stop_motors();
			(void)motion_u_turn();
			enter(VC_STATE_TURNING);
			break;
		case B3_NAV_STRAIGHT:		/* keep following */
		default:
			break;
		}
	}
}

/* ------------------------------------------------------------------ */
/*  5. Act on the current state                                       */
/* ------------------------------------------------------------------ */

LOCAL void act(void)
{
	switch (vc.state) {
	case VC_STATE_LINE_FOLLOW:
		if (vc.line.stop_required || !vc.line.control_active) {
			drive(0, 0);
		} else {
			drive(vc.line.line.left_mm_s, vc.line.line.right_mm_s);
		}
		break;
	case VC_STATE_TURNING:
		/* Buddy 2 finishes the turn on its own; back to the line
		   once it is idle again. */
		if ((vc.state_ms >= 2 * VC_PERIOD_MS) &&
		    (vc.motion.state == MOTION_STATE_IDLE)) {
			enter(VC_STATE_LINE_FOLLOW);
		} else if (vc.state_ms >= VC_TURN_TIMEOUT_MS) {
			vc_raise_fault(VC_FAULT_TASK);
		}
		break;
	case VC_STATE_IDLE:
	case VC_STATE_OBSTACLE:
	case VC_STATE_STOPPED:
	case VC_STATE_ESTOP:
	default:
		break;				/* motors already stopped */
	}
}

/* ------------------------------------------------------------------ */
/*  Task                                                              */
/* ------------------------------------------------------------------ */

LOCAL void vc_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	for (;;) {
		handle_flags();
		read_motion();
		read_obstacle();
		exchange_line();
		act();

		/* 6. one aligned store: readers see all or nothing */
		g_status_word = vc_status_pack(vc.state, vc.faults, vc.last_nav,
					       vc.last_symbol, vc.last_seq);

		vc.state_ms += VC_PERIOD_MS;
		vc.since_send_ms += VC_PERIOD_MS;
		tk_dly_tsk(VC_PERIOD_MS);
	}
}

EXPORT ER vc_init(void)
{
	T_CFLG	cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_vc = tk_cre_flg(&cflg);
	if (flg_vc < E_OK) {
		return flg_vc;
	}
	vc = (vc_t){0};
	vc.state = VC_STATE_IDLE;
	g_status_word = vc_status_pack(VC_STATE_IDLE, 0U, B3_NAV_NONE, 0, 0U);

	/* IF-S5-5: Buddy 5 sets its own cause bit on a sensor fault */
	obstacle_register_estop(flg_vc, VC_FAULT_NO_ECHO);
	/* TODO Buddy 2 + Buddy 5: obstacle_register_odometry() */
	return E_OK;
}

EXPORT ER vc_start(void)
{
	ID tid = app_start_task(vc_task, VC_PRI, 0, "vc_task");

	return (tid < E_OK) ? tid : E_OK;
}
