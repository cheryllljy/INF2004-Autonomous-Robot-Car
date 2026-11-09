/*
 *      vehicle_controller.c
 *      VehicleController task: one 10 ms loop that reads every subsystem,
 *      decides, and commands Buddy 2.  General structure for the team; each
 *      "OWNER" hook is filled in by that buddy.
 *
 *      Each pass:
 *        1. faults    any cause bit in flg_vc        -> ESTOP (latched)
 *        2. commands  START / STOP / RESET requests   -> state change
 *        3. inputs    Buddy 2 status, Buddy 4 IMU, Buddy 5 (obstacle)
 *        4. Buddy 3   hand over context, read line + barcode
 *        5. act       the current state's motor command
 *        6. publish   one 32-bit status word
 */

#include "vehicle_controller.h"
#include "vc_turn.h"     /* nyx, 2026-11-06: turns at a junction */
#include "vc_barcode.h"  /* nyx, 2026-11-06: back up, read barcodes slowly */
#include "../subsystem2/motion_control.h"
#include "../subsystem3/b3_port.h"
#include "../subsystem5/subsystem5.h" /* nyx, 2026-11-07: s5_avoid_*() */
#include "../subsystem4/subsystem4.h"
#include "../subsystem1/buddy1_fr2.h"
#include "../subsystem1/buddy1_mapping.h"
#if defined(APP_MQTT) && APP_MQTT
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#endif

ID flg_vc = 0;

static volatile uint32_t g_status_word; /* written only by vc_task */

/* nyx, 2026-11-06: the arc takes about 4 s at 50 mm/s (8 s at 25) and the
 * U-turn about 5 s (vc_turn.c), so 5 s was too short. */
#define VC_TURN_TIMEOUT_MS 15000

typedef struct
{
    vc_state_t state;
    UINT faults; /* latched causes seen this pass */
    motion_status_t motion;
    b3_snapshot_t line;
    b3_nav_command_t pending_turn; /* from MQTT, done at next junction */
    b3_nav_command_t turn;         /* nyx, 2026-11-06: turn being done */
    INT soft_ms; /* nyx, 2026-11-06: time into the soft start after a U-turn */
    /* nyx, 2026-11-06: measured pass time (the loop runs about every 20 ms,
     * not VC_PERIOD_MS) for vc_turn's ramps and the soft start */
    INT dt_ms;
    uint32_t pass_us;
    /* nyx, 2026-11-06: this pass Buddy 3 decoded / rejected a barcode */
    bool bar_decoded;
    bool bar_rejected;
    char last_symbol;
    uint32_t last_seq;
    uint32_t junctions; /* Buddy 3 junction_sequence seen */
    uint32_t rejected;  /* Buddy 3 rejected_scans seen */
    bool scanning;      /* Buddy 3 scan_active seen */
    INT state_ms;       /* time in the current state */
    INT since_send_ms;
    int16_t sent_l;
    int16_t sent_r;
    /* nyx, 2026-11-08: the next START came as SEARCH (robot.ps1 search):
     * look for a line first (s5_search_begin), then follow it */
    bool b_search_req;
    /* nyx, 2026-11-09: curve log (curve_log()): sums over the last
     * VC_CURVE_LOG_MM of a curve */
    int32_t cv_um;     /* driven, um */
    int32_t cv_n;      /* passes */
    int32_t cv_ask_l;  /* Buddy 3's wheel speeds, mm/s */
    int32_t cv_ask_r;
    int32_t cv_got_l;  /* Buddy 2's measured speeds, mm/s */
    int32_t cv_got_r;
    int32_t cv_curv;   /* Buddy 3's curve, 1e6 / radius mm */
} vc_t;

static vc_t vc;
#if defined(APP_MQTT) && APP_MQTT
static bool b1_reset_pending = false;
#endif

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

void vc_raise_fault(UINT causes)
{
    causes &= VC_FAULT_MASK;
    if ((flg_vc > 0) && (0U != causes))
    {
        /* atomic in the kernel */
        (void)tk_set_flg(flg_vc, causes);
    }
}

void vc_post_command(vc_cmd_t cmd)
{
    if ((flg_vc > 0) && (cmd > VC_CMD_NONE) && (cmd < VC_CMD_COUNT))
    {
        (void)tk_set_flg(flg_vc, VC_FLG_CMD(cmd));
    }
}

uint32_t vc_get_status_word(void)
{
    return g_status_word; /* one aligned 32-bit load */
}

/* OWNER: Buddy 1 -- fill wifi/mqtt and the metric, then b1_telemetry_pack() */
void vc_fill_telemetry(B1_Telemetry *p_out)
{
    uint32_t w = g_status_word;

    p_out->controller_state = (uint8_t)VC_SW_STATE(w);
    p_out->fault_flags = VC_SW_FAULTS(w);
    p_out->wifi_connected = 0U; /* TODO Buddy 1 */
    p_out->mqtt_connected = 0U; /* TODO Buddy 1 */
    /* TODO team: which measurement */
    p_out->metric_0_100 = 0U;
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

static void enter(vc_state_t next)
{
    if (next != vc.state)
    {
        /* Buddy 5 (nyx, 2026-11-07): leaving OBSTACLE any way but
                   "back on the line" (STOP, E-stop, RESET) ends the bypass
                   and puts the sensor back on guard. */
        if ((VC_STATE_OBSTACLE == vc.state) && (VC_STATE_LINE_FOLLOW != next))
        {
            s5_avoid_abort();
        }
        tm_printf((UB *)"[VC] state %d -> %d\n", vc.state, next);
        vc.state = next;
        vc.state_ms = 0;
        if (VC_STATE_LINE_FOLLOW == next)
        {
            vc_bar_reset(&vc.motion); /* nyx, 2026-11-06: fast again */
        }
    }
}

/* nyx, 2026-11-06: time since the last pass, measured. tk_dly_tsk(10)
 * waits for the tick after 10 ms, so a pass is about 20 ms, not
 * VC_PERIOD_MS. Clamped so a long first pass or a stall cannot jump a
 * ramp. */
static void pass_time(void)
{
    const uint32_t now = b3_port_time_us();
    uint32_t ms = (now - vc.pass_us + 500U) / 1000U;

    if ((0U == vc.pass_us) || (ms > 100U))
    {
        ms = VC_PERIOD_MS;
    }
    if (0U == ms)
    {
        ms = 1U;
    }
    vc.pass_us = now;
    vc.dt_ms = (INT)ms;
}

static void stop_motors(void)
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
static void poll_buddy1(void)
{
#if defined(APP_MQTT) && APP_MQTT
    B1_FR3_Status link = { 0 };
    B1_FR2_Command command;
    bool status_valid;

    status_valid = (b1_fr3_get_status(&link) == 0);

    /*
     * Communication safety:
     * Never continue driving without healthy MQTT and Wi-Fi.
     */
    if (!status_valid || link.safety_stop_required)
    {
        if (VC_STATE_LINE_FOLLOW == vc.state || VC_STATE_TURNING == vc.state ||
            vc.state == VC_STATE_OBSTACLE)
        {
            /* Diagnostic (Buddy 3, 2026-11-04): say which link failed */
            /* nyx, 2026-11-07: say how to drive again */
            tm_printf((UB *)"[VC] link lost while driving: %s%d s); once "
                            "it is back, robot.ps1 go\n",
                      !status_valid ? "no status ("
                      : (B1_FR3_REASON_WIFI == link.loss_reason)
                          ? "Wi-Fi down ("
                          : "MQTT broker not answering (",
                      (INT)(B1_FR3_TIMEOUT_MS / 1000u));
            motion_estop_request(MOTION_ESTOP_VEHICLE);
            stop_motors();
            vc_raise_fault(VC_FAULT_WIFI_LOSS);
        }

        /*
         * FR3 latch persists even after communication
         * recovers, until an authorized RESET.
         */
        if (status_valid && link.link_lost_latched)
        {
            vc_raise_fault(VC_FAULT_WIFI_LOSS);
        }
    }

    /*
     * Complete a previously requested FR3 reset only
     * after the network task has cleared its latch.
     */
    if (b1_reset_pending && status_valid && link.healthy &&
        !link.link_lost_latched)
    {
        b1_reset_pending = false;
        vc_post_command(VC_CMD_RESET);

        tm_printf((UB *)"[VC] Communication reset complete\n");
    }

    /*
     * Consume one command per 10 ms loop.
     * Buddy 1's priority STOP is returned first.
     */
    if (b1_fr2_pop(&command) != 0)
    {
        return;
    }

    switch (command.command)
    {
        case B1_FR2_START:

            if (status_valid && link.healthy && !link.link_lost_latched &&
                !b1_reset_pending)
            {
                vc_post_command(VC_CMD_START);

                tm_printf((UB *)"[VC] MQTT START received seq=%u\n",
                          (uint32_t)command.sequence);
            }
            else
            {
                tm_printf((UB *)"[VC] MQTT START rejected: "
                                "communication not ready\n");
            }

            break;

        case B1_FR2_SEARCH:

            /* nyx, 2026-11-08: START, but drive straight until the IRs
             * meet a line and line up along it first (s5_avoid.c) */
            if (status_valid && link.healthy && !link.link_lost_latched &&
                !b1_reset_pending)
            {
                vc.b_search_req = true;
                vc_post_command(VC_CMD_START);

                tm_printf((UB *)"[VC] MQTT SEARCH received seq=%u: "
                                "line search, then line following\n",
                          (uint32_t)command.sequence);
            }
            else
            {
                tm_printf((UB *)"[VC] MQTT SEARCH rejected: "
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

            tm_printf((UB *)"[VC] MQTT STOP received seq=%u\n",
                      (uint32_t)command.sequence);

            break;

        case B1_FR2_RESET:

            if (!status_valid || !link.healthy)
            {
                tm_printf((UB *)"[VC] MQTT RESET rejected: "
                                "communication unhealthy\n");

                break;
            }

            if (link.link_lost_latched)
            {
                if (b1_fr3_controller_reset() == 0)
                {
                    b1_reset_pending = true;

                    tm_printf((UB *)"[VC] MQTT RESET requested "
                                    "for FR3 safety latch\n");
                }
            }
            else
            {
                vc_post_command(VC_CMD_RESET);

                tm_printf((UB *)"[VC] MQTT RESET received seq=%u\n",
                          (uint32_t)command.sequence);
            }

            break;

        default:

            tm_printf((UB *)"[VC] Unknown MQTT command=%u\n",
                      (uint32_t)command.command);

            break;
    }
#endif /* APP_MQTT: without MQTT=1 there is no Buddy 1 link to poll
          (edit by Buddy 3, 2026-11-04, so motion + IR tests build
          without Wi-Fi/MQTT) */
}

/* Send Buddy 3's wheel speeds on change, and refresh them periodically */
static void drive(int16_t left, int16_t right)
{
    if ((left != vc.sent_l) || (right != vc.sent_r) ||
        (vc.since_send_ms >= VC_SPEED_REFRESH_MS))
    {
        if (motion_set_wheel_speeds(left, right) == MOTION_OK)
        {
            vc.sent_l = left;
            vc.sent_r = right;
            vc.since_send_ms = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  1-2. Faults and commands                                          */
/* ------------------------------------------------------------------ */

static void handle_flags(void)
{
    T_RFLG rflg;
    UINT cmds;
    bool b_search = false;

    if (tk_ref_flg(flg_vc, &rflg) < E_OK)
    {
        return;
    }
    vc.faults = rflg.flgptn & VC_FAULT_MASK;
    cmds = rflg.flgptn & VC_FLG_CMD_MASK;
    /* nyx, 2026-11-08: a SEARCH counts for this START only, also when the
     * START is refused (E-stop, already driving) */
    if (0U != (cmds & VC_FLG_CMD(VC_CMD_START)))
    {
        b_search = vc.b_search_req;
        vc.b_search_req = false;
    }
    if (0U != cmds)
    {
        /* consume the requests */
        (void)tk_clr_flg(flg_vc, ~cmds);
    }

    if (cmds & VC_FLG_CMD(VC_CMD_RESET))
    {
        /* Only the VehicleController clears fault causes. A source
                   that is still faulty sets its bit again next pass. */
        (void)tk_clr_flg(flg_vc, ~(UINT)VC_FAULT_MASK);
        vc.faults = 0U;
        motion_estop_clear(MOTION_ESTOP_VEHICLE);
        (void)motion_reset();
        /* nyx, 2026-11-06: Buddy 2's RESET does not stop velocity mode, so
         * a RESET while turning (or line following) left the wheels running
         * with the VehicleController IDLE and Buddy 5 disarmed. */
        stop_motors();
        vc.pending_turn = B3_NAV_NONE;
        buddy3_clear_held_symbol(); /* nyx, 2026-11-06 */
        enter(VC_STATE_IDLE);
        return;
    }
    if (0U != vc.faults)
    {
        if (VC_STATE_ESTOP != vc.state)
        {
            stop_motors();
            tm_printf((UB *)"[VC] E-stop, fault bits 0x%02x\n", vc.faults);
            enter(VC_STATE_ESTOP);
        }
        return;
    }
    if (cmds & VC_FLG_CMD(VC_CMD_STOP))
    {
        stop_motors();
        vc.pending_turn = B3_NAV_NONE;
        buddy3_clear_held_symbol(); /* nyx, 2026-11-06 */
        enter(VC_STATE_STOPPED);
    }
    else if ((cmds & VC_FLG_CMD(VC_CMD_START)) &&
             ((VC_STATE_IDLE == vc.state) || (VC_STATE_STOPPED == vc.state)))
    {
        if (b_search)
        {
            /* nyx, 2026-11-08: S5 drives (OBSTACLE state) until the car is
             * on a line, then hands over like after a bypass */
            stop_motors();
            enter(VC_STATE_OBSTACLE);
            s5_search_begin(&vc.motion);
        }
        else
        {
            enter(VC_STATE_LINE_FOLLOW);
        }
    }

    /* Turn commands come from the MQTT side after it mapped the barcode
           letter. Keep the latest one for the next junction. */
    if (cmds & VC_FLG_CMD(VC_CMD_TURN_LEFT))
    {
        vc.pending_turn = B3_NAV_LEFT;
    }
    else if (cmds & VC_FLG_CMD(VC_CMD_TURN_RIGHT))
    {
        vc.pending_turn = B3_NAV_RIGHT;
    }
    else if (cmds & VC_FLG_CMD(VC_CMD_U_TURN))
    {
        vc.pending_turn = B3_NAV_UTURN;
    }
    else if (cmds & VC_FLG_CMD(VC_CMD_GO_STRAIGHT))
    {
        vc.pending_turn = B3_NAV_STRAIGHT;
    }
}

/* ------------------------------------------------------------------ */
/*  3. Inputs from the other subsystems                               */
/* ------------------------------------------------------------------ */

/* OWNER: Buddy 2 -- motion status and motion faults */
static void read_motion(void)
{
    motion_get_status(&vc.motion);

    /* Diagnostic (Buddy 3, 2026-11-04): say which Buddy 2 fault, once */
    static motion_fault_t reported = MOTION_FAULT_NONE;
    if (MOTION_STATE_FAULT != vc.motion.state)
    {
        reported = MOTION_FAULT_NONE;
    }
    if (MOTION_STATE_FAULT == vc.motion.state)
    {
        if (vc.motion.fault != reported)
        {
            static const char *const names[] = {
                "none",
                "STALL_L (left wheel: high PWM, no encoder ticks 300 ms)",
                "STALL_R (right wheel: high PWM, no encoder ticks 300 ms)",
                "ENCODER_TIMEOUT (full PWM, wheel far slower than asked)",
                "LINE_STALE",
            };
            uint32_t f = (uint32_t)vc.motion.fault;
            reported = vc.motion.fault;
            tm_printf((UB *)"[VC] Buddy 2 fault %u: %s, speed L %d R %d mm/s, "
                            "ticks L %d R %d\n",
                      f, (f < sizeof names / sizeof names[0]) ? names[f] : "?",
                      (int32_t)vc.motion.speed_l_mmps,
                      (int32_t)vc.motion.speed_r_mmps,
                      (int32_t)vc.motion.ticks_l, (int32_t)vc.motion.ticks_r);
        }
        vc_raise_fault((MOTION_FAULT_ENCODER_TIMEOUT == vc.motion.fault)
                           ? VC_FAULT_ENCODER
                           : VC_FAULT_MOTOR_STALL);
    }
}

/* Use Subsystem 4's cached sample; the controller never accesses I2C.
 *
 * Buddy 3 change (2026-11-05): the IMU fault is no longer raised on the
 * first bad pass. Subsystem 4 publishes nothing while it calibrates
 * (about 0.4 s after boot and after every imu_reset), and it publishes
 * a failed status if the first imu_init try fails, so the old check
 * latched an E-stop 0x04 at every start. Now the sample has to be bad
 * or stale for VC_IMU_FAULT_MS in a row before VC_FAULT_IMU is raised.
 * The reason is printed once per bad spell.
 */
#define VC_IMU_FAULT_MS 1000

static void read_imu(b3_context_t *p_ctx)
{
    static UINT bad_ms; /* how long the IMU has been bad */
    static BOOL reported;
    s4_imu_snapshot_t snapshot;
    SYSTIM now;
    INT reason = 0; /* 0 = good sample */
    uint32_t age = 0U;

    p_ctx->accel_valid = false;
    p_ctx->accel_x_lsb = 0;
    p_ctx->accel_y_lsb = 0;
    p_ctx->accel_z_lsb = 0;
    /* LSM303D has no gyroscope, so yaw rate is not available. */
    p_ctx->imu_valid = false;
    p_ctx->yaw_rate_mdeg_s = 0;
    /* Terrain classification is a separate, not-yet-implemented step. */
    p_ctx->terrain_unstable = false;

    if (subsystem4_get_imu_snapshot(&snapshot) < E_OK)
    {
        reason = 1;
    }
    else if (!snapshot.has_sample)
    {
        reason = 2; /* still starting up */
    }
    else if (tk_get_tim(&now) < E_OK)
    {
        reason = 3;
    }
    else if ((age = (uint32_t)(now.lo - snapshot.timestamp_ms)) >
             S4_IMU_MAX_AGE_MS)
    {
        /* stale (calibrating or stuck) */
        reason = 4;
    }
    else if (IMU_OK != snapshot.accel_status)
    {
        reason = 5;
    }

    if (0 != reason)
    {
        bad_ms += VC_PERIOD_MS;
        if (bad_ms >= VC_IMU_FAULT_MS)
        {
            if (!reported)
            {
                reported = TRUE;
                tm_printf((UB *)"[VC] IMU fault: %s (status %d, age %u ms)\n",
                          (1 == reason)   ? "snapshot call failed"
                          : (2 == reason) ? "no sample yet"
                          : (3 == reason) ? "no clock"
                          : (4 == reason) ? "sample too old"
                                          : "accel read failed",
                          (INT)snapshot.accel_status, (UINT)age);
            }
            vc_raise_fault(VC_FAULT_IMU);
        }
        return;
    }
    bad_ms = 0U;
    reported = FALSE;

    p_ctx->accel_valid = true;
    p_ctx->accel_x_lsb = snapshot.accel.x;
    p_ctx->accel_y_lsb = snapshot.accel.y;
    p_ctx->accel_z_lsb = snapshot.accel.z;
}

/* OWNER: Buddy 5 -- obstacles (nyx, 2026-11-07, subsystem 5 rewritten).
   The S5 task only works the sensor; the bypass runs here, in
   s5_avoid.c, so only this task drives Buddy 2. */
static void read_obstacle(void)
{
    const bool b_driving = (VC_STATE_LINE_FOLLOW == vc.state) ||
                           (VC_STATE_TURNING == vc.state) ||
                           (VC_STATE_OBSTACLE == vc.state);

    /* FR25: a sensor that gives no echo at all, or no pings, while the
     * car drives is a fault (latched E-stop, like before) */
    if (b_driving && s5_avoid_sensor_fault())
    {
        vc_raise_fault(VC_FAULT_NO_ECHO);
        return;
    }

    /* Something within 300 mm ahead while following the line: stop and
     * hand over to the bypass. Not while backing up for a barcode, and not
     * during a junction turn (the turn ends on the line first). */
    if ((VC_STATE_LINE_FOLLOW == vc.state) &&
        s5_avoid_check(vc.line.control_active && !vc_bar_busy()))
    {
        stop_motors(); /* a normal stop, not an E-stop */
        enter(VC_STATE_OBSTACLE);
        s5_avoid_begin(&vc.motion);
    }
}

/* ------------------------------------------------------------------ */
/*  4. Buddy 3: line following and barcodes                           */
/* ------------------------------------------------------------------ */

/* OWNER: Buddy 1 -- a barcode letter was decoded. Buddy 3 holds it until
   the next junction (nyx, 2026-11-06), so nothing is mapped here any more;
   Buddy 1 may publish the letter to the dashboard from this hook. */
static void on_barcode(char symbol, uint32_t sequence)
{
    tm_printf((UB *)"[VC] barcode '%c' (#%u) held for the next junction\n",
              symbol, (uint32_t)sequence);
}

/* OWNER: Buddy 1 -- map a barcode letter to a turn with the mapping that
   came over MQTT. Moved here from on_barcode() by Buddy 3 (nyx,
   2026-11-06): the letter is now mapped when the junction is reached, so a
   mapping that arrives after the barcode was read still applies. */
static bool symbol_to_turn(char symbol, b3_nav_command_t *p_turn)
{
    B1_Movement movement;
    bool mapped = true;

    if (b1_mapping_get_movement(symbol, &movement) != 0)
    {
        tm_printf((UB *)"[VC] barcode '%c' has no MQTT mapping, "
                        "going straight\n",
                  symbol);
        return false;
    }

    switch (movement)
    {
        case B1_MOVE_FORWARD:
            *p_turn = B3_NAV_STRAIGHT;
            break;
        case B1_MOVE_LEFT:
            *p_turn = B3_NAV_LEFT;
            break;
        case B1_MOVE_RIGHT:
            *p_turn = B3_NAV_RIGHT;
            break;
        case B1_MOVE_UTURN:
            *p_turn = B3_NAV_UTURN;
            break;
        default:
            mapped = false;
            break;
    }

    return mapped;
}

/* OWNER: Buddy 1 -- publish "junction reached" if the dashboard wants it */
static void on_junction(uint32_t count)
{
    (void)count; /* TODO Buddy 1 */
}

/* nyx, 2026-11-06: say once why Buddy 3 stopped the car. */
#define VC_LINE_STALE_US    100000U /* 10 Buddy 3 periods */
#define VC_LINE_FAULT_STALE ((uint8_t)B3_FAULT_COUNT)

static void report_line_fault(uint8_t source)
{
    static bool reported = false;
    static const char *const names[] = {
        "queue or task error",    "navigation event queue overflow",
        "sensor task RTOS error", "message buffer send failed",
        "mailbox receive failed", "snapshot not updated for 100 ms"
    };

    if (!reported)
    {
        reported = true;
        tm_printf((UB *)"[VC] Buddy 3 fault: %s (error %d), E-stop\n",
                  (source < (sizeof names / sizeof names[0])) ? names[source]
                                                              : "?",
                  (INT)vc.line.task_error);
    }
}

static const char *turn_name(b3_nav_command_t turn)
{
    switch (turn)
    {
        case B3_NAV_LEFT:
            return "LEFT";
        case B3_NAV_RIGHT:
            return "RIGHT";
        case B3_NAV_UTURN:
            return "U-TURN";
        case B3_NAV_STRAIGHT:
            return "STRAIGHT";
        case B3_NAV_NONE:
        default:
            return "STRAIGHT (none)";
    }
}

/* Do the pending turn at the junction. None pending: keep going straight.
 * nyx, 2026-11-06: the turn is vc_turn.c's: a smooth arc for LEFT/RIGHT
 * (no stop) and an eased stop + gentle pivot for the U-turn, both ending
 * when the IRs have the new line centred. The old stop + 90 deg pivot
 * turned where the IRs met the junction, 125 mm before the wheels got
 * there, and left the car beside the new line. */
static void take_junction(void)
{
    b3_nav_command_t turn = vc.pending_turn;

    vc.pending_turn = B3_NAV_NONE;
    if (vc_turn_start(turn, &vc.motion))
    {
        vc.turn = turn;
        enter(VC_STATE_TURNING);
    }
    else
    {
        /* STRAIGHT or none: Buddy 3 drives over the crossing */
    }
}

static void exchange_line(void)
{
    b3_context_t ctx = { 0 };
    b3_nav_event_t ev;

    ctx.time_us = b3_port_time_us();
    /* nyx, 2026-11-06: Buddy 3 keeps following while vc_turn or the
     * obstacle bypass drives (their wheel speeds win in act()). Its
     * snapshot is a pass behind this context: with follow off until the
     * turn ended, the first LINE_FOLLOW pass read control_active false and
     * sent (0, 0), so Buddy 2 braked to a stop at the end of every arc and
     * refused the next speeds while STOPPING. */
    ctx.follow_enabled = (VC_STATE_LINE_FOLLOW == vc.state) ||
                         (VC_STATE_TURNING == vc.state) ||
                         (VC_STATE_OBSTACLE == vc.state);
    /* nyx, 2026-11-06: no scanning while stopping and backing up for a
     * barcode (vc_barcode.c), which also sets the line speed */
    ctx.scan_enabled =
        (VC_STATE_LINE_FOLLOW == vc.state) && vc_bar_scan_allowed();
    ctx.base_speed_mm_s = vc_bar_speed_mm_s();
    /* nyx, 2026-11-07: Buddy 3's curve memory restarts from straight when
     * it gets the wheels back */
    ctx.steer_hold = (VC_STATE_TURNING == vc.state) ||
                     (VC_STATE_OBSTACLE == vc.state) || vc_bar_busy();
    ctx.stop_requested =
        (VC_STATE_ESTOP == vc.state) || (VC_STATE_STOPPED == vc.state);
    ctx.motion_valid = (MOTION_STATE_FAULT != vc.motion.state);
    ctx.left_measured_mm_s = vc.motion.speed_l_mmps;
    ctx.right_measured_mm_s = vc.motion.speed_r_mmps;
    read_imu(&ctx);
    buddy3_set_context(&ctx);
    buddy3_get_snapshot(&vc.line);

    /* Serious Buddy 3 faults latch the E-stop. A plain stop_required
           (stale context, follow off) just means "do not drive". */
    if ((0 != vc.line.task_error) || (0U != vc.line.dropped_events) ||
        (vc.line.fault_source != (uint8_t)B3_FAULT_NONE))
    {
        report_line_fault(vc.line.fault_source);
        vc_raise_fault(VC_FAULT_LINE);
    }

    /* nyx, 2026-11-06: Buddy 3's task must refresh its snapshot every
           10 ms. If it stops (task ended, never started) the old wheel
           speeds would keep driving the car, so while the car is meant to
           move, a snapshot older than VC_LINE_STALE_US is a fault too. */
    if (((VC_STATE_LINE_FOLLOW == vc.state) ||
         (VC_STATE_TURNING == vc.state)) &&
        ((int32_t)(ctx.time_us - vc.line.time_us) >
         /* signed: a newer snapshot is fine */
         (int32_t)VC_LINE_STALE_US))
    {
        report_line_fault(VC_LINE_FAULT_STALE);
        vc_raise_fault(VC_FAULT_LINE);
    }

    /* Barcode: Buddy 3 gives only the letter. Hand it to the MQTT side,
           which maps it to a command and sends it back (VC_CMD_TURN_*). */
    vc.bar_decoded = false;
    vc.bar_rejected = false;
    while (buddy3_pop_navigation(&ev))
    {
        vc.bar_decoded = true; /* nyx, 2026-11-06 */
        vc.last_symbol = ev.barcode.symbol;
        vc.last_seq = ev.sequence;
        on_barcode(ev.barcode.symbol, ev.sequence);
    }

    /* Diagnostic (Buddy 3, 2026-11-04): IR_B started reading a barcode */
    if (vc.line.scan_active != vc.scanning)
    {
        vc.scanning = vc.line.scan_active;
        if (vc.scanning)
        {
            tm_printf((UB *)"[VC] barcode scan started\n");
        }
    }

    /* Diagnostic: a barcode was seen but failed to decode */
    if (vc.line.rejected_scans != vc.rejected)
    {
        vc.rejected = vc.line.rejected_scans;
        vc.bar_rejected = true; /* nyx, 2026-11-06 */
        /* Buddy 3 diagnostics (nyx): elements measured and why. */
        static const char *const reject_names[] = {
            "none",
            "same colour twice (missed edge)",
            "element too short (noise)",
            "element too long",
            "element much wider than the narrowest",
            "whole scan too long",
            "not a valid Code 39 pattern",
            "white before barcode too short",
            "white after barcode too short",
        };
        uint32_t why = vc.line.reject_reason;
        tm_printf((UB *)"[VC] barcode rejected (total %u): %u/%u elements, "
                        "last element %u us, reason %u: %s\n",
                  (uint32_t)vc.rejected, (uint32_t)vc.line.reject_edges,
                  (uint32_t)B3_BARCODE_RUNS,
                  (uint32_t)vc.line.reject_element_us, why,
                  (why < sizeof reject_names / sizeof reject_names[0])
                      ? reject_names[why]
                      : "?");
        /* nyx, 2026-11-06: white before barcode too short: how much white
         * was measured before the first bar and how much was needed. */
        if (7U == why)
        {
            if (vc.line.reject_encoder)
            {
                /* ticks are the mean of both wheels; x10 gives 0.1 mm */
                uint32_t lead = (vc.line.reject_lead > 1000000U)
                                    ? 1000000U
                                    : vc.line.reject_lead;
                uint32_t need = (vc.line.reject_need > 1000000U)
                                    ? 1000000U
                                    : vc.line.reject_need;
                int32_t lead10 = motion_ticks_to_mm((int32_t)(lead * 10U));
                int32_t need10 = motion_ticks_to_mm((int32_t)(need * 10U));

                tm_printf((UB *)"[VC]   white before the first bar %d.%d mm "
                                "(%u ticks), needs %d.%d mm (%u x narrow "
                                "bar)\n",
                          (INT)(lead10 / 10), (INT)(lead10 % 10), lead,
                          (INT)(need10 / 10), (INT)(need10 % 10),
                          (uint32_t)B3_QUIET_NARROW_MULTIPLE);
            }
            else
            {
                tm_printf((UB *)"[VC]   white before the first bar %u ms, "
                                "needs %u ms (%u x narrow bar, timed: no "
                                "encoder distance)\n",
                          vc.line.reject_lead / 1000U,
                          vc.line.reject_need / 1000U,
                          (uint32_t)B3_QUIET_NARROW_MULTIPLE);
            }
        }
        /* not Code 39: show the widths, B = bar, w = gap */
        if (6U == why)
        {
            for (uint32_t c = 0U; c < 3U; ++c)
            {
                char line[120];
                uint32_t n = 0U;
                for (uint32_t k = c * 10U;
                     (k < (c + 1U) * 10U) && (k < B3_BARCODE_RUNS); ++k)
                {
                    uint32_t ms10 =
                        (uint32_t)((vc.line.reject_widths[k] + 50U) / 100U);
                    char digits[8];
                    uint32_t d = 0U;
                    if (ms10 > 99999U)
                    {
                        ms10 = 99999U;
                    }
                    /* tenths */
                    digits[d++] = (char)('0' + ms10 % 10U);
                    digits[d++] = '.';
                    ms10 /= 10U;
                    do
                    {
                        digits[d++] = (char)('0' + ms10 % 10U);
                        ms10 /= 10U;
                    } while (ms10 > 0U);
                    if (n + d + 2U >= sizeof line)
                    {
                        break;
                    }
                    line[n++] = ' ';
                    line[n++] = (k & 1U) ? 'w' : 'B';
                    while (d > 0U)
                    {
                        line[n++] = digits[--d];
                    }
                }
                line[n] = '\0';
                tm_printf((UB *)"[VC]   widths ms %u-%u:%s\n", c * 10U,
                          (c * 10U + 9U < B3_BARCODE_RUNS)
                              ? c * 10U + 9U
                              : B3_BARCODE_RUNS - 1U,
                          line);
            }
        }
    }

    /* Junction: Buddy 3 counts each one, so a new count = just reached.
           nyx, 2026-11-06: Buddy 3 hands over the letter it held since the
           barcode; map it now and do that turn here. A turn command sent
           straight over MQTT (VC_CMD_TURN_*) is used when no letter came. */
    /* nyx, 2026-11-06: both line sensors went black but Buddy 3 did not
     * count a junction: print the patch length and the reason once. */
    {
        static uint32_t patch_misses;
        static const char *const why[] = {
            "junction", "a sensor's run too short", "too slow on average",
            "car not driving (stopped, no motion data, stale)",
            "barcode scan in progress under IR_B"
        };

        if (vc.line.patch_misses != patch_misses)
        {
            patch_misses = vc.line.patch_misses;
            /* nyx, 2026-11-08: each sensor's own run, and the part with
             * both black together */
            tm_printf((UB *)"[VC] line sensors black L %u / R %u mm, "
                            "%u mm together, not a junction: %s (needs %u "
                            "mm each at %d mm/s)\n",
                      (UINT)vc.line.patch_miss_l_mm,
                      (UINT)vc.line.patch_miss_r_mm,
                      (UINT)vc.line.patch_miss_mm,
                      (vc.line.patch_miss_why <
                       (uint8_t)(sizeof(why) / sizeof(why[0])))
                          ? why[vc.line.patch_miss_why]
                          : "?",
                      (UINT)(B3_JUNCTION_MIN_PATCH_UM / 1000U),
                      (INT)B3_JUNCTION_MIN_SPEED_MM_S);
        }
    }

    if (vc.line.junction_sequence != vc.junctions)
    {
        b3_nav_command_t turn = vc.pending_turn;

        vc.junctions = vc.line.junction_sequence;
        if ((vc.line.junction_symbol != '\0') &&
            !symbol_to_turn(vc.line.junction_symbol, &turn))
        {
            turn = B3_NAV_STRAIGHT;
        }
        vc.pending_turn = turn;
        tm_printf((UB *)"[VC] junction #%u reached, letter '%c', "
                        "turn %s\n",
                  (uint32_t)vc.junctions,
                  (vc.line.junction_symbol != '\0') ? vc.line.junction_symbol
                                                    : '-',
                  turn_name(turn));
        on_junction(vc.junctions);
        if ((VC_STATE_LINE_FOLLOW == vc.state) &&
            !vc_bar_busy()) /* nyx, 2026-11-06: not while backing up */
        {
            take_junction();
        }
        else if (VC_STATE_OBSTACLE == vc.state)
        {
            /* nyx, 2026-11-07: the bypass crossed a line; keep the turn
             * for the real junction after it */
        }
        else
        {
            vc.pending_turn = B3_NAV_NONE;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  5. Act on the current state                                       */
/* ------------------------------------------------------------------ */

/* nyx, 2026-11-09: in a curve, every VC_CURVE_LOG_MM of driving, what
 * Buddy 3 asked for and what the wheels did (means over that stretch):
 *   [VC] curve: asked R<r> (L <l> R <r> mm/s), wheels L <l> R <r> mm/s
 *   = R<r>, IRs <L><R>
 * R is the turn radius in mm, + = left, 0 = straight. The wheels' R much
 * larger than the asked one: the car turns less than Buddy 3 wants (a
 * slow inner wheel that Buddy 2 cannot hold, or slip). */
#define VC_CURVE_LOG_MM   100
#define VC_CURVE_LOG_CURV 1000 /* tighter than R1000 counts as a curve */

static int32_t curve_radius(int32_t l, int32_t r)
{
    /* 1e6 / R = 1e6 x (r - l) / (track x (l + r) / 2) */
    const int32_t sum = l + r;
    int32_t radius = 0;

    if ((r != l) && (sum > 0))
    {
        radius = (int32_t)(((int64_t)B3_WHEEL_TRACK_MM * sum) /
                           (2 * (int64_t)(r - l)));
    }
    return radius;
}

static void curve_log(int16_t ask_l, int16_t ask_r)
{
    const int32_t curv = vc.line.line.curvature;
    const int32_t got_l = vc.motion.speed_l_mmps;
    const int32_t got_r = vc.motion.speed_r_mmps;
    int32_t v = (got_l + got_r) / 2;

    if ((curv < VC_CURVE_LOG_CURV) && (curv > -VC_CURVE_LOG_CURV))
    {
        vc.cv_um = 0;
        vc.cv_n = 0;
        vc.cv_ask_l = 0;
        vc.cv_ask_r = 0;
        vc.cv_got_l = 0;
        vc.cv_got_r = 0;
        vc.cv_curv = 0;
        return;
    }
    v = (v < 0) ? -v : v;
    vc.cv_um += v * vc.dt_ms;
    vc.cv_n++;
    vc.cv_ask_l += ask_l;
    vc.cv_ask_r += ask_r;
    vc.cv_got_l += got_l;
    vc.cv_got_r += got_r;
    vc.cv_curv += curv;
    if (vc.cv_um >= (VC_CURVE_LOG_MM * 1000))
    {
        const int32_t n = vc.cv_n;
        const int32_t al = vc.cv_ask_l / n;
        const int32_t ar = vc.cv_ask_r / n;
        const int32_t gl = vc.cv_got_l / n;
        const int32_t gr = vc.cv_got_r / n;
        const int32_t c = vc.cv_curv / n;

        tm_printf((UB *)"[VC] curve: asked R%d (L %d R %d mm/s), wheels "
                        "L %d R %d mm/s = R%d, IRs %c%c\n",
                  (INT)((0 != c) ? (1000000 / c) : 0), (INT)al, (INT)ar,
                  (INT)gl, (INT)gr, (INT)curve_radius(gl, gr),
                  vc.line.line.left_black ? 'B' : 'w',
                  vc.line.line.right_black ? 'B' : 'w');
        vc.cv_um = 0;
        vc.cv_n = 0;
        vc.cv_ask_l = 0;
        vc.cv_ask_r = 0;
        vc.cv_got_l = 0;
        vc.cv_got_r = 0;
        vc.cv_curv = 0;
    }
}

static void act(void)
{
    /* Buddy 5 (nyx, 2026-11-07): while Buddy 3 follows the line, S5
     * dead reckons the car and keeps the IR pair's track (the line a
     * bypass is planned against); any other pass starts the track again.
     * In OBSTACLE, s5_avoid_step() does it. */
    if (VC_STATE_OBSTACLE != vc.state)
    {
        s5_avoid_track(&vc.motion, (VC_STATE_LINE_FOLLOW == vc.state) &&
                                       vc.line.control_active &&
                                       !vc.line.stop_required &&
                                       !vc_bar_busy());
    }
    switch (vc.state)
    {
        case VC_STATE_LINE_FOLLOW:
            if (vc.line.stop_required)
            {
                drive(0, 0);
            }
            else if (vc_bar_step(&vc.motion, &vc.line, vc.bar_decoded,
                                 vc.bar_rejected, vc.dt_ms))
            {
                /* nyx, 2026-11-06: stopping or backing up for a barcode;
                 * Buddy 3's speeds are sent again afterwards */
                vc.sent_l = INT16_MIN;
                vc.sent_r = INT16_MIN;
            }
            else if (!vc.line.control_active)
            {
                drive(0, 0);
            }
            else
            {
                int16_t left = vc.line.line.left_mm_s;
                int16_t right = vc.line.line.right_mm_s;

                /* nyx, 2026-11-06: pull away gently after a U-turn */
                vc_turn_soft_start(&left, &right, vc.soft_ms);
                if (vc.soft_ms < VC_SOFT_START_MS)
                {
                    vc.soft_ms += vc.dt_ms;
                }
                drive(left, right);
                curve_log(left, right); /* nyx, 2026-11-09 */
            }
            break;
        case VC_STATE_TURNING:
        {
            /* nyx, 2026-11-06: the arc or the U-turn (vc_turn.c), ended by
                   the IRs; Buddy 3 takes over afterwards. */
            const vc_turn_result_t r =
                vc_turn_step(&vc.motion, vc.line.line.left_black,
                             vc.line.line.right_black, vc.dt_ms);

            if (VC_TURN_RUNNING != r)
            {
                /* vc_turn sent its own wheel speeds: resend Buddy 3's */
                vc.sent_l = INT16_MIN;
                vc.sent_r = INT16_MIN;
                vc.soft_ms = (VC_TURN_DONE_STOPPED == r) ? 0 : VC_SOFT_START_MS;
                enter(VC_STATE_LINE_FOLLOW);
            }
            else if (vc.state_ms >= VC_TURN_TIMEOUT_MS)
            {
                vc_raise_fault(VC_FAULT_TASK);
            }
            break;
        }
        case VC_STATE_OBSTACLE:
        {
            /* Buddy 5 (nyx, 2026-11-07): stop, sweep, go round on the
             * shorter side, find the same line again (s5_avoid.c) */
            int16_t left = 0;
            int16_t right = 0;

            switch (s5_avoid_step(&vc.motion, vc.line.line.left_black,
                                  vc.line.line.right_black, vc.dt_ms, &left,
                                  &right))
            {
                case S5_AVOID_DONE:
                    /* on the line again, rolling: Buddy 3's speeds next */
                    vc.sent_l = INT16_MIN;
                    vc.sent_r = INT16_MIN;
                    vc.soft_ms = VC_SOFT_START_MS;
                    enter(VC_STATE_LINE_FOLLOW);
                    break;
                case S5_AVOID_FAULT:
                    stop_motors();
                    vc_raise_fault(VC_FAULT_OBSTACLE);
                    break;
                case S5_AVOID_RUNNING:
                default:
                    drive(left, right);
                    break;
            }
            break;
        }
        case VC_STATE_IDLE:
        case VC_STATE_STOPPED:
        case VC_STATE_ESTOP:
        default:
            break; /* motors already stopped */
    }
}

/* ------------------------------------------------------------------ */
/*  Task                                                              */
/* ------------------------------------------------------------------ */

static void vc_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

#if VC_BENCH
    {
        /* repeat, so a serial monitor opened after boot still sees it */
        INT n;

        for (n = VC_BENCH_START_S; n > 0; n--)
        {
            tm_printf((UB *)"[VC] bench mode: START in %d s\n", n);
            tk_dly_tsk(1000);
        }
    }
    vc_post_command(VC_CMD_START);
#endif

    for (;;)
    {
        poll_buddy1();
        handle_flags();
        read_motion();
        read_obstacle();
        exchange_line();
        pass_time();
        act();

        /* 6. one aligned store: readers see all or nothing. nyx,
         * 2026-11-07: no way round an obstacle shows as its fault bit
         * while the car waits, without the E-stop. */
        g_status_word = vc_status_pack(
            vc.state,
            vc.faults | (s5_avoid_blocked() ? VC_FAULT_OBSTACLE : 0U),
            vc.pending_turn, vc.last_symbol, vc.last_seq, vc.junctions);

        vc.state_ms += VC_PERIOD_MS;
        vc.since_send_ms += VC_PERIOD_MS;
        tk_dly_tsk(VC_PERIOD_MS);
    }
}

ER vc_init(void)
{
    T_CFLG cflg = { 0 };

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    flg_vc = tk_cre_flg(&cflg);
    if (flg_vc < E_OK)
    {
        return flg_vc;
    }
    vc = (vc_t){ 0 };
    vc.state = VC_STATE_IDLE;
    vc.soft_ms = VC_SOFT_START_MS; /* nyx, 2026-11-06: no soft start yet */
    vc.dt_ms = VC_PERIOD_MS;
    g_status_word = vc_status_pack(VC_STATE_IDLE, 0U, B3_NAV_NONE, 0, 0U, 0U);
    return E_OK;
}

ER vc_start(void)
{
    ID tid = app_start_task(vc_task, VC_PRI, 0, "vc_task");

    return (tid < E_OK) ? tid : E_OK;
}
