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
#include "vc_obstacle.h"		/* Buddy 5 hook (v3.0) */
#include "../subsystem2/motion_control.h"
#include "../subsystem3/b3_port.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem1/buddy1_fr2.h"
#include "../subsystem1/buddy1_mapping.h"
#if defined(APP_MQTT) && APP_MQTT
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#endif

EXPORT ID	flg_vc = 0;

LOCAL volatile uint32_t	g_status_word;	/* written only by vc_task */

/* Odometry for Buddy 5 (IF-S5-4). motion_get_status() locks a mutex, which
   S5's callback must not do, so vc_task copies speed and distance here and
   S5 reads them lock-free. Two words, so a sequence count guards against a
   torn read: odd = write in progress (single writer, so no RMW race). */
LOCAL volatile uint32_t	g_odo_seq;
LOCAL volatile int32_t	g_odo_speed_mm_s;
LOCAL volatile int32_t	g_odo_dist_mm;

#define VC_TURN_ANGLE_DEG	90
#define VC_TURN_TIMEOUT_MS	5000

typedef struct {
	vc_state_t	state;
	UINT		faults;		/* latched causes seen this pass */
	motion_status_t	motion;
	b3_snapshot_t	line;
	b3_nav_command_t pending_turn;	/* from MQTT, done at next junction */
	char		last_symbol;
	uint32_t	last_seq;
	uint32_t	junctions;	/* Buddy 3 junction_sequence seen */
	uint32_t	rejected;	/* Buddy 3 rejected_scans seen */
	bool		scanning;	/* Buddy 3 scan_active seen */
	INT		state_ms;	/* time in the current state */
	INT		since_send_ms;
	int16_t		sent_l;
	int16_t		sent_r;
} vc_t;

LOCAL vc_t	vc;
#if defined(APP_MQTT) && APP_MQTT
LOCAL bool b1_reset_pending = false;
#endif

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
		/* Buddy 5: leaving OBSTACLE any way but "bypass done" (STOP,
		   E-stop, RESET) ends the manoeuvre and frees S5. */
		if ((vc.state == VC_STATE_OBSTACLE) &&
		    (next != VC_STATE_LINE_FOLLOW)) {
			vc_obs_leave();
		}
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

/*
 * Buddy 1 integration.
 *
 * Only VehicleController consumes FR2 commands.
 * MQTT callbacks never call Buddy 2 motors directly.
 */
LOCAL void poll_buddy1(void)
{
#if defined(APP_MQTT) && APP_MQTT
    B1_FR3_Status link = {0};
    B1_FR2_Command command;
    bool status_valid;

    status_valid = (b1_fr3_get_status(&link) == 0);

    /*
     * Communication safety:
     * Never continue driving without healthy MQTT and Wi-Fi.
     */
    if (!status_valid || link.safety_stop_required) {

        if (vc.state == VC_STATE_LINE_FOLLOW ||
            vc.state == VC_STATE_TURNING ||
            vc.state == VC_STATE_OBSTACLE) {

            /* Diagnostic (Buddy 3, 2026-11-04): say which link failed */
            tm_printf((UB*)"[VC] link lost while driving: %s\n",
                      !status_valid ? "no status" :
                      (link.loss_reason == B1_FR3_REASON_WIFI) ?
                      "Wi-Fi down" : "MQTT broker not answering (3 s)");
            motion_estop_request(MOTION_ESTOP_VEHICLE);
            stop_motors();
            vc_raise_fault(VC_FAULT_WIFI_LOSS);
        }

        /*
         * FR3 latch persists even after communication
         * recovers, until an authorized RESET.
         */
        if (status_valid && link.link_lost_latched) {
            vc_raise_fault(VC_FAULT_WIFI_LOSS);
        }
    }

    /*
     * Complete a previously requested FR3 reset only
     * after the network task has cleared its latch.
     */
    if (b1_reset_pending &&
        status_valid &&
        link.healthy &&
        !link.link_lost_latched) {

        b1_reset_pending = false;
        vc_post_command(VC_CMD_RESET);

        tm_printf((UB*)
            "[VC] Communication reset complete\n");
    }

    /*
     * Consume one command per 10 ms loop.
     * Buddy 1's priority STOP is returned first.
     */
    if (b1_fr2_pop(&command) != 0) {
        return;
    }

    switch (command.command) {

    case B1_FR2_START:

        if (status_valid &&
            link.healthy &&
            !link.link_lost_latched &&
            !b1_reset_pending) {

            vc_post_command(VC_CMD_START);

            tm_printf((UB*)
                "[VC] MQTT START received seq=%u\n",
                (unsigned)command.sequence);
        } else {
            tm_printf((UB*)
                "[VC] MQTT START rejected: "
                "communication not ready\n");
        }

        break;

    case B1_FR2_STOP:

        /*
         * Stop immediately; do not wait for the
         * normal command event to be processed.
         */
        stop_motors();
        vc_post_command(VC_CMD_STOP);

        tm_printf((UB*)
            "[VC] MQTT STOP received seq=%u\n",
            (unsigned)command.sequence);

        break;

    case B1_FR2_RESET:

        if (!status_valid || !link.healthy) {

            tm_printf((UB*)
                "[VC] MQTT RESET rejected: "
                "communication unhealthy\n");

            break;
        }

        if (link.link_lost_latched) {

            if (b1_fr3_controller_reset() == 0) {

                b1_reset_pending = true;

                tm_printf((UB*)
                    "[VC] MQTT RESET requested "
                    "for FR3 safety latch\n");
            }

        } else {

            vc_post_command(VC_CMD_RESET);

            tm_printf((UB*)
                "[VC] MQTT RESET received seq=%u\n",
                (unsigned)command.sequence);
        }

        break;

    default:

        tm_printf((UB*)
            "[VC] Unknown MQTT command=%u\n",
            (unsigned)command.command);

        break;
    }
#endif /* APP_MQTT: without MQTT=1 there is no Buddy 1 link to poll
	  (edit by Buddy 3, 2026-11-04, so motion + IR tests build
	  without Wi-Fi/MQTT) */
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
		motion_estop_clear(MOTION_ESTOP_VEHICLE);
		(void)motion_reset();
		vc.pending_turn = B3_NAV_NONE;
		if (flg_obstacle > 0) {
			obstacle_reset_fault();
		}
		enter(VC_STATE_IDLE);
		return;
	}
	if (vc.faults != 0U) {
		if (vc.state != VC_STATE_ESTOP) {
			stop_motors();
			tm_printf((UB*)"[VC] E-stop, fault bits 0x%02x\n", vc.faults);
			enter(VC_STATE_ESTOP);
		}
		return;
	}
	if (cmds & VC_FLG_CMD(VC_CMD_STOP)) {
		stop_motors();
		 vc.pending_turn = B3_NAV_NONE;
		enter(VC_STATE_STOPPED);
	} else if ((cmds & VC_FLG_CMD(VC_CMD_START)) &&
		   ((vc.state == VC_STATE_IDLE) ||
		    (vc.state == VC_STATE_STOPPED))) {
		enter(VC_STATE_LINE_FOLLOW);
	}

	/* Turn commands come from the MQTT side after it mapped the barcode
	   letter. Keep the latest one for the next junction. */
	if (cmds & VC_FLG_CMD(VC_CMD_TURN_LEFT)) {
		vc.pending_turn = B3_NAV_LEFT;
	} else if (cmds & VC_FLG_CMD(VC_CMD_TURN_RIGHT)) {
		vc.pending_turn = B3_NAV_RIGHT;
	} else if (cmds & VC_FLG_CMD(VC_CMD_U_TURN)) {
		vc.pending_turn = B3_NAV_UTURN;
	} else if (cmds & VC_FLG_CMD(VC_CMD_GO_STRAIGHT)) {
		vc.pending_turn = B3_NAV_STRAIGHT;
	}
}

/* ------------------------------------------------------------------ */
/*  3. Inputs from the other subsystems                               */
/* ------------------------------------------------------------------ */

LOCAL int32_t abs_i32(int32_t v)
{
	return (v < 0) ? -v : v;
}

/* IF-S5-4: called from Buddy 5's task; never blocks */
LOCAL bool vc_odometry(int32_t *p_speed_mm_s, int32_t *p_distance_mm)
{
	uint32_t	seq;
	INT		tries;

	for (tries = 0; tries < 3; tries++) {
		seq = g_odo_seq;
		__sync_synchronize();
		*p_speed_mm_s  = g_odo_speed_mm_s;
		*p_distance_mm = g_odo_dist_mm;
		__sync_synchronize();
		if (((seq & 1U) == 0U) && (seq != 0U) && (seq == g_odo_seq)) {
			return true;
		}
	}
	return false;			/* no data yet, or kept colliding */
}

/* OWNER: Buddy 2 -- motion status and motion faults */
LOCAL void read_motion(void)
{
	motion_get_status(&vc.motion);

	g_odo_seq++;			/* odd: writing */
	__sync_synchronize();
	/* Buddy 5 (v3.0): the FASTER wheel, not the mean. Turning on the
	   spot the mean is ~0, and S5's WaitStopped would sweep while the
	   car is still spinning. */
	g_odo_speed_mm_s = (abs_i32(vc.motion.speed_l_mmps) >
			    abs_i32(vc.motion.speed_r_mmps)) ?
			   vc.motion.speed_l_mmps : vc.motion.speed_r_mmps;
	g_odo_dist_mm    = vc.motion.distance_total_mm;
	__sync_synchronize();
	g_odo_seq++;			/* even: done */

	/* Diagnostic (Buddy 3, 2026-11-04): say which Buddy 2 fault, once */
	static motion_fault_t reported = MOTION_FAULT_NONE;
	if (vc.motion.state != MOTION_STATE_FAULT) {
		reported = MOTION_FAULT_NONE;
	}
	if (vc.motion.state == MOTION_STATE_FAULT) {
		if (vc.motion.fault != reported) {
			static const char *const names[] = {
				"none", "STALL_L (left wheel: high PWM, no encoder ticks 300 ms)",
				"STALL_R (right wheel: high PWM, no encoder ticks 300 ms)",
				"ENCODER_TIMEOUT (full PWM, wheel far slower than asked)",
				"LINE_STALE",
			};
			unsigned f = (unsigned)vc.motion.fault;
			reported = vc.motion.fault;
			tm_printf((UB*)"[VC] Buddy 2 fault %u: %s, speed L %d R %d mm/s, "
				  "ticks L %d R %d\n", f,
				  (f < sizeof names / sizeof names[0]) ? names[f] : "?",
				  (int)vc.motion.speed_l_mmps, (int)vc.motion.speed_r_mmps,
				  (int)vc.motion.ticks_l, (int)vc.motion.ticks_r);
		}
		vc_raise_fault((vc.motion.fault == MOTION_FAULT_ENCODER_TIMEOUT) ?
			       VC_FAULT_ENCODER : VC_FAULT_MOTOR_STALL);
	}
}

/* Use Subsystem 4's cached sample; the controller never accesses I2C. */
LOCAL void read_imu(b3_context_t *p_ctx)
{
	s4_imu_snapshot_t snapshot;
	SYSTIM now;

	p_ctx->accel_valid      = false;
	p_ctx->accel_x_lsb      = 0;
	p_ctx->accel_y_lsb      = 0;
	p_ctx->accel_z_lsb      = 0;
	/* LSM303D has no gyroscope, so yaw rate is not available. */
	p_ctx->imu_valid        = false;
	p_ctx->yaw_rate_mdeg_s  = 0;
	/* Terrain classification is a separate, not-yet-implemented step. */
	p_ctx->terrain_unstable = false;

	if (subsystem4_get_imu_snapshot(&snapshot) < E_OK) {
		vc_raise_fault(VC_FAULT_IMU);
		return;
	}
	if (!snapshot.has_sample) {
		return;
	}
	if ((tk_get_tim(&now) < E_OK) ||
	    ((uint32_t)(now.lo - snapshot.timestamp_ms) > S4_IMU_MAX_AGE_MS) ||
	    (snapshot.accel_status != IMU_OK)) {
		vc_raise_fault(VC_FAULT_IMU);
		return;
	}

	p_ctx->accel_valid = true;
	p_ctx->accel_x_lsb = snapshot.accel.x;
	p_ctx->accel_y_lsb = snapshot.accel.y;
	p_ctx->accel_z_lsb = snapshot.accel.z;
}

/* OWNER: Buddy 5 -- obstacle events (IF-S5-1, IF-S5-7). The manoeuvre
   itself is in vc_obstacle.c; this only routes the events. */
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

	/* Under 120 mm while moving: cut the motors now (Buddy 2's
	   obstacle cause bit, released by vc_obstacle once stopped). Not
	   the latched E-stop. */
	if ((ptn & S5_EVT_HARD_STOP) &&
	    ((vc.state == VC_STATE_LINE_FOLLOW) ||
	     (vc.state == VC_STATE_TURNING) ||
	     (vc.state == VC_STATE_OBSTACLE))) {
		vc_obs_hard_stop(&vc.motion);
	}

	if ((ptn & (S5_EVT_DETECTED | S5_EVT_HARD_STOP)) &&
	    ((vc.state == VC_STATE_LINE_FOLLOW) ||
	     (vc.state == VC_STATE_TURNING))) {
		stop_motors();			/* normal stop, not an E-stop */
		enter(VC_STATE_OBSTACLE);
		vc_obs_begin(&vc.motion);
	}

	if (vc.state == VC_STATE_OBSTACLE) {
		vc_obs_events(ptn);
	} else if (ptn & S5_EVT_PROFILED) {
		/* Parked (IDLE / STOPPED / E-stop): someone's hand in front
		   of the sensor. Nothing to drive round; S5 back to GUARD. */
		obstacle_resume();
	}

	/* S5_EVT_FAULT: a SENSOR fault sets VC_FAULT_NO_ECHO itself
	   (registered in vc_init). A mission fault (stop timeout, reverse
	   limit, VC silent) latches S5, so it no longer looks for
	   obstacles: while driving the car must not go on. Parked, there
	   is nothing to protect: clear it so S5 keeps guarding. */
	if (ptn & S5_EVT_FAULT) {
		obstacle_health_t h;

		obstacle_get_health(&h);
		if ((h.fault != OBSTACLE_FAULT_NONE) &&
		    (h.fault != OBSTACLE_FAULT_SENSOR)) {
			if ((vc.state == VC_STATE_LINE_FOLLOW) ||
			    (vc.state == VC_STATE_TURNING) ||
			    (vc.state == VC_STATE_OBSTACLE)) {
				vc_raise_fault(VC_FAULT_OBSTACLE);
			} else {
				tm_printf((UB*)"[VC] S5 fault %d while parked,"
					  " cleared\n", (INT)h.fault);
				obstacle_reset_fault();
			}
		}
	}
}

/* ------------------------------------------------------------------ */
/*  4. Buddy 3: line following and barcodes                           */
/* ------------------------------------------------------------------ */

/* OWNER: Buddy 1 -- publish the barcode letter over MQTT. The MQTT side
   maps it to a command and calls vc_post_command(VC_CMD_TURN_*). */
LOCAL void on_barcode(char symbol, uint32_t sequence)
{
    B1_Movement movement;

    if (b1_mapping_get_movement(symbol, &movement) != 0) {
        tm_printf((UB*)
            "[VC] Barcode '%c' has no MQTT mapping\n",
            symbol);
        return;
    }

    switch (movement) {

    case B1_MOVE_FORWARD:
        vc_post_command(VC_CMD_GO_STRAIGHT);
        break;

    case B1_MOVE_LEFT:
        vc_post_command(VC_CMD_TURN_LEFT);
        break;

    case B1_MOVE_RIGHT:
        vc_post_command(VC_CMD_TURN_RIGHT);
        break;

    case B1_MOVE_UTURN:
        vc_post_command(VC_CMD_U_TURN);
        break;

    default:
        return;
    }

    tm_printf((UB*)
        "[VC] Barcode '%c' mapped to movement=%u seq=%u\n",
        symbol,
        (unsigned)movement,
        (unsigned)sequence);
}

/* OWNER: Buddy 1 -- publish "junction reached" if the dashboard wants it */
LOCAL void on_junction(uint32_t count)
{
	(void)count;				/* TODO Buddy 1 */
}

/* Do the pending turn at the junction. None pending: keep going straight. */
LOCAL void take_junction(void)
{
	b3_nav_command_t turn = vc.pending_turn;

	vc.pending_turn = B3_NAV_NONE;
	switch (turn) {
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
	case B3_NAV_STRAIGHT:
	case B3_NAV_NONE:
	default:
		break;				/* Buddy 3 drives over the crossing */
	}
}

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

	/* Barcode: Buddy 3 gives only the letter. Hand it to the MQTT side,
	   which maps it to a command and sends it back (VC_CMD_TURN_*). */
	while (buddy3_pop_navigation(&ev)) {
		vc.last_symbol = ev.barcode.symbol;
		vc.last_seq    = ev.sequence;
		tm_printf((UB*)"[VC] barcode '%c' (#%u)\n",
			  ev.barcode.symbol, (unsigned)ev.sequence);
		on_barcode(ev.barcode.symbol, ev.sequence);
	}

	/* Diagnostic (Buddy 3, 2026-11-04): IR_B started reading a barcode */
	if (vc.line.scan_active != vc.scanning) {
		vc.scanning = vc.line.scan_active;
		if (vc.scanning) {
			tm_printf((UB*)"[VC] barcode scan started\n");
		}
	}

	/* Diagnostic: a barcode was seen but failed to decode */
	if (vc.line.rejected_scans != vc.rejected) {
		vc.rejected = vc.line.rejected_scans;
		/* Buddy 3 diagnostics (nyx): elements measured and why. */
		static const char *const reject_names[] = {
			"none", "same colour twice (missed edge)",
			"element too short (noise)", "element too long",
			"element much wider than the narrowest",
			"whole scan too long", "not a valid Code 39 pattern",
			"white before barcode too short",
			"white after barcode too short",
		};
		unsigned why = vc.line.reject_reason;
		tm_printf((UB*)"[VC] barcode rejected (total %u): %u/%u elements, "
			  "last element %u us, reason %u: %s\n",
			  (unsigned)vc.rejected, (unsigned)vc.line.reject_edges,
			  (unsigned)B3_BARCODE_RUNS,
			  (unsigned)vc.line.reject_element_us, why,
			  (why < sizeof reject_names / sizeof reject_names[0]) ?
				reject_names[why] : "?");
		if (why == 6U) {	/* not Code 39: show the widths, B = bar, w = gap */
			for (unsigned c = 0U; c < 3U; ++c) {
				char line[120];
				unsigned n = 0U;
				for (unsigned k = c * 10U;
				     (k < (c + 1U) * 10U) && (k < B3_BARCODE_RUNS); ++k) {
					unsigned ms10 = (unsigned)((vc.line.reject_widths[k] + 50U) / 100U);
					char digits[8];
					unsigned d = 0U;
					if (ms10 > 99999U) ms10 = 99999U;
					digits[d++] = (char)('0' + ms10 % 10U);	/* tenths */
					digits[d++] = '.';
					ms10 /= 10U;
					do { digits[d++] = (char)('0' + ms10 % 10U); ms10 /= 10U; } while (ms10 > 0U);
					if (n + d + 2U >= sizeof line) break;
					line[n++] = ' ';
					line[n++] = (k & 1U) ? 'w' : 'B';
					while (d > 0U) line[n++] = digits[--d];
				}
				line[n] = '\0';
				tm_printf((UB*)"[VC]   widths ms %u-%u:%s\n", c * 10U,
					  (c * 10U + 9U < B3_BARCODE_RUNS) ? c * 10U + 9U :
					  B3_BARCODE_RUNS - 1U, line);
			}
		}
	}

	/* Junction: Buddy 3 counts each one, so a new count = just reached */
	if (vc.line.junction_sequence != vc.junctions) {
		vc.junctions = vc.line.junction_sequence;
		tm_printf((UB*)"[VC] junction #%u reached, turn %d\n",
			  (unsigned)vc.junctions, vc.pending_turn);
		on_junction(vc.junctions);
		if (vc.state == VC_STATE_LINE_FOLLOW) {
			take_junction();
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
	case VC_STATE_OBSTACLE:
		/* Buddy 5: scan, reverse, bypass, rejoin (vc_obstacle.c) */
		switch (vc_obs_step(&vc.motion, vc.line.line.left_black,
				    vc.line.line.right_black, VC_PERIOD_MS)) {
		case VC_OBS_DONE:
			enter(VC_STATE_LINE_FOLLOW);
			break;
		case VC_OBS_FAULT:
			vc_raise_fault(VC_FAULT_OBSTACLE);
			break;
		case VC_OBS_RUNNING:
		default:
			break;
		}
		break;
	case VC_STATE_IDLE:
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

#if VC_BENCH
	{
		/* repeat, so a serial monitor opened after boot still sees it */
		INT n;

		for (n = VC_BENCH_START_S; n > 0; n--) {
			tm_printf((UB*)"[VC] bench mode: START in %d s\n", n);
			tk_dly_tsk(1000);
		}
	}
	vc_post_command(VC_CMD_START);
#endif

	for (;;) {
		poll_buddy1();
		handle_flags();
		read_motion();
		vc_obs_service_cut(&vc.motion, VC_PERIOD_MS);	/* Buddy 5 */
		read_obstacle();
		exchange_line();
		act();

		/* 6. one aligned store: readers see all or nothing */
		g_status_word = vc_status_pack(vc.state, vc.faults,
					       vc.pending_turn, vc.last_symbol,
					       vc.last_seq, vc.junctions);

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
	g_status_word = vc_status_pack(VC_STATE_IDLE, 0U, B3_NAV_NONE, 0, 0U, 0U);

	/* IF-S5-5: Buddy 5 sets its own cause bit on a sensor fault */
	obstacle_register_estop(flg_vc, VC_FAULT_NO_ECHO);
	/* IF-S5-4: Buddy 2's speed and distance, via vc_task's copy */
	obstacle_register_odometry(vc_odometry);
	return E_OK;
}

EXPORT ER vc_start(void)
{
	ID tid = app_start_task(vc_task, VC_PRI, 0, "vc_task");

	return (tid < E_OK) ? tid : E_OK;
}
