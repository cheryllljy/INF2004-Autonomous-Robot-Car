/*
 * motion_control.c - Buddy 2 MotionControl task (v1).
 *
 * One task woken every MOTION_CONTROL_PERIOD_MS by a cyclic handler. Each cycle:
 *   1. poll the estop flag and cut the motors at once (NFR21)
 *   2. read the encoders: odometry and wheel speeds
 *   3. stall check
 *   4. read commands from the motion_cmd message buffer (VehicleController only)
 *   5. state logic: moves, turns, stopping -> wheel speed goals
 *   6. ramp the goals, run the speed controller, write the PWM
 *   7. publish the status snapshot and raise events
 *
 * Implemented: MOVE, TURN, SET_SPEED, STOP, RESET, RESET_TRIP, E-stop, stall fault.
 * Not yet: LINE_FOLLOW (needs the LineSense interface) - it is answered with REJECTED.
 *
 * NOT compiled against the real kernel: check the micro T-Kernel calls and the
 * T_C??? structure fields against tkernel.h.
 */
#include <stdint.h>
#include <stdbool.h>
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hw.h"

#include <tk/tkernel.h>

#define MC_SIDES            (2U)
#define MC_DT_NOMINAL_S     ((float)MOTION_CONTROL_PERIOD_MS / 1000.0f)
#define MC_HIST_LEN         (SPEED_WINDOW_PERIODS + 1U)
#define MC_ESTOP_ALL        (MOTION_ESTOP_IMU | MOTION_ESTOP_OBSTACLE | MOTION_ESTOP_VEHICLE)
#define MC_INTEG_MAX_PM     (300.0f)
#define MC_ZERO_SPEED       (5.0f)      /* mm/s: below this a goal counts as zero */
#define MC_STILL_CYCLES     (3U)        /* cycles at rest before STOPPING is finished */
#define MC_RAD_PER_DEG      (MC_PI / 180.0f)

/* ---------- Kernel objects ---------- */
static ID s_task_id;
static ID s_cyc_id;
static ID s_cmd_mbf;
static ID s_estop_flg;
static ID s_status_mtx;
static ID s_evt_flg;                    /* VehicleController's vehicle_evt, if attached */
static uint32_t s_evt_shift;

/* ---------- Task-owned state ---------- */
static motion_state_t  s_state = MOTION_STATE_INIT;
static motion_fault_t  s_fault = MOTION_FAULT_NONE;
static motion_result_t s_result = MOTION_RESULT_NONE;
static motion_result_t s_pending_result = MOTION_RESULT_NONE;
static uint32_t s_estop_src;
static uint32_t s_pending_evt;
static uint32_t s_cycle_count;
static uint32_t s_overrun_count;

static uint32_t s_raw_last[MC_SIDES];
static uint32_t s_delta[MC_SIDES];
static int32_t  s_pos_ticks[MC_SIDES];  /* signed odometry ticks, forward = + */
static int32_t  s_dir[MC_SIDES] = { 1, 1 };
static int32_t  s_hist[MC_SIDES][MC_HIST_LEN];
static uint32_t s_hist_idx;
static uint32_t s_hist_t_ms[MC_HIST_LEN];   /* time stamp of every history sample using monotonic kernel time */
static uint32_t s_t_last_ms;
static uint32_t s_edge_clock_last_raw;
static bool     s_edge_clock_valid;          /* compare is timings valid */
static float    s_dt_s = MC_DT_NOMINAL_S;   /* ACTUAL time since the last cycle, from the hardware timer */
static uint32_t s_dt_ms = MOTION_CONTROL_PERIOD_MS;
static float    s_speed[MC_SIDES];      /* measured mm/s, signed */
static float    s_goal[MC_SIDES];       /* wanted mm/s */
static float    s_set[MC_SIDES];        /* ramped setpoint mm/s */
static float    s_integ[MC_SIDES];
static int32_t  s_duty[MC_SIDES];       /* last duty written, permille */
static uint32_t s_stall_ms[MC_SIDES];
static uint32_t s_runaway_ms[MC_SIDES];
static uint32_t s_still;

static float    s_target_mm;            /* MOVE: distance. TURN: arc per wheel (+ = left turn) */
static float    s_orig_mm[MC_SIDES];     /* wheel positions when the command started */
static float    s_cruise;               /* mm/s magnitude for MOVE / TURN */
static bool     s_vel_mode;
static int32_t  s_trip_origin_ticks;

static motion_status_t s_shared;        /* published copy, guarded by s_status_mtx */

/* ---------- Small helpers ---------- */

static float mc_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float mc_clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    return (x > hi) ? hi : x;
}

/* Newton iteration; avoids needing libm. Plenty accurate for the speed profile. */
static float mc_sqrt(float x)
{
    float r;
    uint32_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }
    r = (x > 1.0f) ? (0.5f * x) : 1.0f;
    for (i = 0U; i < 24U; i++)
    {
        r = 0.5f * (r + (x / r));
    }
    return r;
}

static float mc_ramp(float cur, float goal, float step)
{
    if (goal > (cur + step))
    {
        return cur + step;
    }
    if (goal < (cur - step))
    {
        return cur - step;
    }
    return goal;
}

static float mc_dist_mm(uint32_t side)
{
    return (float)s_pos_ticks[side] * MM_PER_TICK;
}

/* Everything a command measures is relative to where it started. */
static float mc_rel_mm(uint32_t side)
{
    return mc_dist_mm(side) - s_orig_mm[side];
}

static float mc_move_progress_mm(void)
{
    return 0.5f * (mc_rel_mm(0U) + mc_rel_mm(1U));
}

static float mc_turn_progress_mm(void)
{
    return 0.5f * (mc_rel_mm(1U) - mc_rel_mm(0U));
}

static float mc_speed_avg(void)
{
    return 0.5f * (mc_absf(s_speed[0]) + mc_absf(s_speed[1]));
}

static bool mc_latched(void)
{
    return (s_state == MOTION_STATE_ESTOP) || (s_state == MOTION_STATE_FAULT);
}

static uint32_t mc_read_estop(void)
{
    T_RFLG rflg;

    if ((s_estop_flg > 0) && (tk_ref_flg(s_estop_flg, &rflg) == E_OK))
    {
        return (uint32_t)rflg.flgptn & MC_ESTOP_ALL;
    }
    return 0U;
}

static void mc_notify(uint32_t bits)
{
    if ((bits != 0U) && (s_evt_flg > 0))
    {
        (void)tk_set_flg(s_evt_flg, bits << s_evt_shift);
    }
}

/* ---------- Outputs, latching ---------- */

static void mc_cut_outputs(void)
{
    uint32_t side;

    for (side = 0U; side < MC_SIDES; side++)
    {
        motion_hw_motor_set(side, 0);
        s_goal[side] = 0.0f;
        s_set[side] = 0.0f;
        s_integ[side] = 0.0f;
        s_duty[side] = 0;
        s_stall_ms[side] = 0U;
        s_runaway_ms[side] = 0U;
    }
}

static void mc_enter_estop(uint32_t bits)
{
    mc_cut_outputs();                           /* first thing: no ramp (NFR21) */
    s_estop_src |= bits;
    if (s_state != MOTION_STATE_ESTOP)
    {
        s_state = MOTION_STATE_ESTOP;
        s_result = MOTION_RESULT_ESTOP;
        s_pending_evt |= MOTION_EVT_ESTOP;
    }
}

static void mc_enter_fault(motion_fault_t fault)
{
    mc_cut_outputs();
    s_fault = fault;
    s_state = MOTION_STATE_FAULT;
    s_result = MOTION_RESULT_FAULT;
    s_pending_evt |= MOTION_EVT_FAULT;
}

/* ---------- Odometry and speed ---------- */

/* Measure the real time since the last cycle. The kernel tick may not be exactly
 * 20 ms (clock set-up differences), so speed and the PID use this, not 20 ms. */
static void mc_update_time(void)
{
    SYSTIM now_tim = {0};
    uint32_t    now;
    uint32_t    dt_us;
    uint32_t    raw_now;    

    /* The RP2040 raw timer is used by the encoder ISR, but its rate is not
     * trusted for control timing: the power-cycle log showed it advancing
     * 23 ms during a 1000 ms kernel delay. Use the kernel's monotonic clock,
     * which also paces the cyclic handler, for speed and PID calculations. */
    if (tk_get_otm(&now_tim) >= E_OK)
    {
        now = now_tim.lo;
    }
    else
    {
        now = s_t_last_ms + MOTION_CONTROL_PERIOD_MS;
    }

    dt_us = (s_t_last_ms == 0U) ? (MOTION_CONTROL_PERIOD_MS * 1000U)
                                : ((now - s_t_last_ms) * 1000U);

    // const uint32_t now = motion_hw_time_us();
    // uint32_t dt_us = (s_t_last_us == 0U) ? (MOTION_CONTROL_PERIOD_MS * 1000U) : (now - s_t_last_us);

    s_t_last_ms = now;
    if (dt_us > ((MOTION_CONTROL_PERIOD_MS + 2U) * 1000U) || (dt_us + 2000U) < (MOTION_CONTROL_PERIOD_MS * 1000U))
    {
        s_overrun_count++;                      /* outside the +/-2 ms jitter allowed by NFR5 */
    }
    if (dt_us < 1000U)
    {
        dt_us = 1000U;
    }
    else if (dt_us > 100000U)
    {
        dt_us = 100000U;
    }
    s_dt_s = (float)dt_us / 1000000.0f;
    s_dt_ms = (dt_us + 500U) / 1000U;

    /* Keep the encoder's physical 100 us debounce interval when TIMERAWL's
     * rate differs from 1 MHz. The ISR only reads this single aligned word. */
    raw_now = motion_hw_time_us();
    if (s_edge_clock_valid && (s_dt_ms != 0U))
    {
        const uint32_t raw_elapsed = raw_now - s_edge_clock_last_raw;
        uint32_t min_edge_ticks = (ENCODER_MIN_EDGE_US * raw_elapsed) / (s_dt_ms * 1000U);

        if (min_edge_ticks == 0U)
        {
            min_edge_ticks = 1U;
        }
        motion_hw_set_encoder_min_edge_ticks(min_edge_ticks);
    }
    s_edge_clock_last_raw = raw_now;
    s_edge_clock_valid = true;
}

static void mc_update_odometry(void)
{
    uint32_t side;
    uint32_t newest;
    uint32_t oldest;

    for (side = 0U; side < MC_SIDES; side++)
    {
        const uint32_t raw = motion_hw_ticks(side);

        s_delta[side] = raw - s_raw_last[side];
        s_raw_last[side] = raw;
        s_pos_ticks[side] += s_dir[side] * (int32_t)s_delta[side];
    }

    s_hist_idx = (s_hist_idx + 1U) % MC_HIST_LEN;
    newest = s_hist_idx;
    oldest = (s_hist_idx + 1U) % MC_HIST_LEN;
    s_hist_t_ms[newest] = s_t_last_ms;
    for (side = 0U; side < MC_SIDES; side++)
    {
        /* Buddy 3 fix (2026-10-28): the history times are kernel ms since the
         * switch away from TIMERAWL, but this was still used as us. An 80 ms
         * window read as "80 us" (< 1000), so every wheel speed was 0: the PI
         * integrator wound up and the car drove faster than asked (~170 mm/s
         * for 100), and the stop-lag braking used speed 0 (300 mm move -> 362 mm). */
        const uint32_t window_us = (s_t_last_ms - s_hist_t_ms[oldest]) * 1000U;

        s_hist[side][newest] = s_pos_ticks[side];
        s_speed[side] = (window_us < 1000U) ? 0.0f
                      : (((float)(s_pos_ticks[side] - s_hist[side][oldest]) * MM_PER_TICK) /
                         ((float)window_us / 1000000.0f));
    }
}

static void mc_check_stall(void)
{
    uint32_t side;

    for (side = 0U; side < MC_SIDES; side++)
    {
        const int32_t mag = (s_duty[side] < 0) ? -s_duty[side] : s_duty[side];

        if ((mag >= MOTION_STALL_MIN_PWM_PM) && (s_delta[side] == 0U))
        {
            s_stall_ms[side] += s_dt_ms;
        }
        else
        {
            s_stall_ms[side] = 0U;
        }
        if (s_stall_ms[side] >= MOTION_STALL_TIMEOUT_MS)
        {
            mc_enter_fault((side == 0U) ? MOTION_FAULT_STALL_L : MOTION_FAULT_STALL_R);
            return;
        }

        /* Feedback sanity: full allowed duty but the wheel is far slower than wanted.
         * Means the encoder is not telling the truth (or the wheel is jammed). */
        if ((mag >= (MOTION_DUTY_MAX_PM - 5)) && (mc_absf(s_speed[side]) < (0.5f * mc_absf(s_set[side]))))
        {
            s_runaway_ms[side] += s_dt_ms;
        }
        else
        {
            s_runaway_ms[side] = 0U;
        }
        if (s_runaway_ms[side] >= MOTION_RUNAWAY_MS)
        {
            mc_enter_fault(MOTION_FAULT_ENCODER_TIMEOUT);
            return;
        }
    }
}

/* ---------- Speed controller: feed-forward + PI, one per wheel ---------- */

static int32_t mc_speed_control(uint32_t side)
{
    const float set = s_set[side];
    const float mag_set = mc_absf(set);
    const float sgn = (set >= 0.0f) ? 1.0f : -1.0f;
    const float gain = (side == 0U) ? MOTOR_L_GAIN : MOTOR_R_GAIN;
    float err;
    float integ;
    float u;

    if (mag_set < MC_ZERO_SPEED)
    {
        s_integ[side] = 0.0f;
        return 0;
    }

    err = mag_set - (s_speed[side] * sgn);
    integ = mc_clampf(s_integ[side] + (MOTION_SPEED_KI * err * s_dt_s),
                      -MC_INTEG_MAX_PM, MC_INTEG_MAX_PM);
    u = (float)MOTOR_FF_OFFSET_PM + (mag_set / (MM_PER_TICK * MOTOR_FF_TICKS_PER_PM)) +
        (MOTION_SPEED_KP * err) + integ;

    if (!(((u > (float)MOTION_DUTY_MAX_PM) && (err > 0.0f)) || ((u < 0.0f) && (err < 0.0f))))
    {
        s_integ[side] = integ;                  /* hold the integrator while saturated */
    }
    if ((mc_absf(s_speed[side]) < (float)MOTION_STOPPED_SPEED_MMPS) && (u < (float)MOTOR_BREAKAWAY_PM))
    {
        u = (float)MOTOR_BREAKAWAY_PM;          /* kick a stopped wheel past static friction */
    }
    u = mc_clampf(u * gain, 0.0f, (float)MOTION_DUTY_MAX_PM);
    return (int32_t)(sgn * u);
}

static void mc_apply_outputs(void)
{
    uint32_t side;

    for (side = 0U; side < MC_SIDES; side++)
    {
        int32_t want = 0;

        if (s_set[side] > MC_ZERO_SPEED)
        {
            want = 1;
        }
        else if (s_set[side] < -MC_ZERO_SPEED)
        {
            want = -1;
        }

        /* Single-channel encoders cannot sense direction: only flip the counting
         * direction while the wheel is (nearly) stopped, never mid-reversal. */
        if ((want != 0) && (want != s_dir[side]))
        {
            if (mc_absf(s_speed[side]) > (float)MOTION_STOPPED_SPEED_MMPS)
            {
                want = 0;
                s_integ[side] = 0.0f;
            }
            else
            {
                s_dir[side] = want;
            }
        }

        s_duty[side] = (want == 0) ? 0 : mc_speed_control(side);
        motion_hw_motor_set(side, s_duty[side]);
    }
}

/* ---------- Motions ---------- */

static void mc_begin_stop(motion_result_t result)
{
    s_state = MOTION_STATE_STOPPING;
    s_pending_result = result;
    s_goal[0] = 0.0f;
    s_goal[1] = 0.0f;
    s_still = 0U;
}

static void mc_finish_now(motion_result_t result)
{
    s_result = result;
    s_pending_evt |= MOTION_EVT_DONE;
}

static void mc_start_move(int32_t distance_mm, int32_t speed_mmps)
{
    if (distance_mm == 0)
    {
        mc_finish_now(MOTION_RESULT_DONE);
        return;
    }
    s_target_mm = (float)distance_mm;
    s_orig_mm[0] = mc_dist_mm(0U);
    s_orig_mm[1] = mc_dist_mm(1U);
    s_cruise = mc_clampf((float)speed_mmps, (float)MOTION_MIN_SPEED_MMPS, (float)MOTION_MAX_SPEED_MMPS);
    s_vel_mode = false;
    s_integ[0] = 0.0f;
    s_integ[1] = 0.0f;
    s_state = MOTION_STATE_DRIVING;
    s_result = MOTION_RESULT_NONE;
}

static void mc_start_turn(int32_t angle_deg, int32_t rate_dps)
{
    const float half_track = TRACK_WIDTH_MM * 0.5f;

    if (angle_deg == 0)
    {
        mc_finish_now(MOTION_RESULT_DONE);
        return;
    }
    s_target_mm = (float)angle_deg * MC_RAD_PER_DEG * half_track;
    s_orig_mm[0] = mc_dist_mm(0U);
    s_orig_mm[1] = mc_dist_mm(1U);
    s_cruise = mc_clampf((float)rate_dps * MC_RAD_PER_DEG * half_track,
                         (float)MOTION_MIN_SPEED_MMPS, (float)MOTION_MAX_SPEED_MMPS);
    s_vel_mode = false;
    s_integ[0] = 0.0f;
    s_integ[1] = 0.0f;
    s_state = MOTION_STATE_TURNING;
    s_result = MOTION_RESULT_NONE;
}

static void mc_start_velocity(int32_t left_mmps, int32_t right_mmps)
{
    if ((left_mmps == 0) && (right_mmps == 0))
    {
        mc_begin_stop(MOTION_RESULT_STOPPED);
        return;
    }
    s_goal[0] = mc_clampf((float)left_mmps, -(float)MOTION_MAX_SPEED_MMPS, (float)MOTION_MAX_SPEED_MMPS);
    s_goal[1] = mc_clampf((float)right_mmps, -(float)MOTION_MAX_SPEED_MMPS, (float)MOTION_MAX_SPEED_MMPS);
    s_vel_mode = true;
    s_state = MOTION_STATE_DRIVING;
    s_result = MOTION_RESULT_NONE;
}

static void mc_plan_move(void)
{
    const float rem = s_target_mm - mc_move_progress_mm();
    const float sgn = (s_target_mm >= 0.0f) ? 1.0f : -1.0f;
    float room;
    float v;
    float trim;

    if ((mc_absf(rem) <= (float)MOTION_DIST_TOL_MM) || ((rem * sgn) < 0.0f))
    {
        mc_begin_stop(MOTION_RESULT_DONE);      /* arrived (or passed it) */
        return;
    }
    room = mc_absf(rem) - (mc_speed_avg() * MOTION_STOP_LAG_S);     /* the wheels need time to slow */
    v = mc_sqrt(2.0f * (float)MOTION_ACCEL_MMPS2 * ((room > 0.0f) ? room : 0.0f));
    if (v > s_cruise)
    {
        v = s_cruise;
    }
    if (v < (float)MOTION_MIN_SPEED_MMPS)
    {
        v = (float)MOTION_MIN_SPEED_MMPS;
    }
    trim = MOTION_HEADING_KP * (mc_rel_mm(0U) - mc_rel_mm(1U));     /* keep both wheels level */
    s_goal[0] = (sgn * v) - trim;
    s_goal[1] = (sgn * v) + trim;
}

static void mc_plan_turn(void)
{
    const float rem = s_target_mm - mc_turn_progress_mm();
    const float sgn = (s_target_mm >= 0.0f) ? 1.0f : -1.0f;
    const float tol = (float)MOTION_TURN_TOL_DEG * MC_RAD_PER_DEG * (TRACK_WIDTH_MM * 0.5f);
    float room;
    float v;
    float trim;

    if ((mc_absf(rem) <= tol) || ((rem * sgn) < 0.0f))
    {
        mc_begin_stop(MOTION_RESULT_DONE);
        return;
    }
    room = mc_absf(rem) - (mc_speed_avg() * MOTION_STOP_LAG_S);
    v = mc_sqrt(2.0f * (float)MOTION_ACCEL_MMPS2 * ((room > 0.0f) ? room : 0.0f));
    if (v > s_cruise)
    {
        v = s_cruise;
    }
    if (v < (float)MOTION_MIN_SPEED_MMPS)
    {
        v = (float)MOTION_MIN_SPEED_MMPS;
    }
    trim = MOTION_HEADING_KP * (mc_rel_mm(0U) + mc_rel_mm(1U));     /* wheels should mirror each other */
    s_goal[0] = (-sgn * v) - trim;
    s_goal[1] = (sgn * v) - trim;
}

static void mc_run_state(void)
{
    const float step = (float)MOTION_ACCEL_MMPS2 * s_dt_s;
    uint32_t side;

    switch (s_state)
    {
    case MOTION_STATE_DRIVING:
        if (!s_vel_mode)
        {
            mc_plan_move();
        }
        break;
    case MOTION_STATE_TURNING:
        mc_plan_turn();
        break;
    case MOTION_STATE_STOPPING:
        s_goal[0] = 0.0f;
        s_goal[1] = 0.0f;
        if ((mc_absf(s_speed[0]) < (float)MOTION_STOPPED_SPEED_MMPS) &&
            (mc_absf(s_speed[1]) < (float)MOTION_STOPPED_SPEED_MMPS) &&
            (mc_absf(s_set[0]) < MC_ZERO_SPEED) && (mc_absf(s_set[1]) < MC_ZERO_SPEED))
        {
            s_still++;
        }
        else
        {
            s_still = 0U;
        }
        if (s_still >= MC_STILL_CYCLES)
        {
            s_state = MOTION_STATE_IDLE;
            s_result = s_pending_result;
            s_pending_evt |= MOTION_EVT_DONE;
        }
        break;
    case MOTION_STATE_IDLE:
        s_goal[0] = 0.0f;
        s_goal[1] = 0.0f;
        break;
    default:
        break;
    }

    if (!mc_latched())
    {
        for (side = 0U; side < MC_SIDES; side++)
        {
            s_set[side] = mc_ramp(s_set[side], s_goal[side], step);
        }
    }
}

/* ---------- Commands ---------- */

static void mc_reject(void)
{
    s_result = MOTION_RESULT_REJECTED;
    s_pending_evt |= MOTION_EVT_REJECTED;
}

static void mc_flush_commands(void)
{
    motion_cmd_msg_t dump;

    while (tk_rcv_mbf(s_cmd_mbf, &dump, TMO_POL) > 0)
    {
        /* drop stale commands so none runs after a reset */
    }
}

static void mc_handle_cmd(const motion_cmd_msg_t *m)
{
    if (m->type == MOTION_CMD_RESET_TRIP)
    {
        s_trip_origin_ticks = (s_pos_ticks[0] + s_pos_ticks[1]) / 2;
        return;
    }

    if (m->type == MOTION_CMD_RESET)
    {
        if (mc_latched())
        {
            if (mc_read_estop() != 0U)
            {
                mc_reject();                    /* VehicleController must clear the flag first */
                return;
            }
            mc_flush_commands();
            mc_cut_outputs();
            s_state = MOTION_STATE_IDLE;
            s_fault = MOTION_FAULT_NONE;
            s_estop_src = 0U;
        }
        mc_finish_now(MOTION_RESULT_DONE);
        return;
    }

    if (mc_latched())
    {
        mc_reject();
        return;
    }

    switch (m->type)
    {
    case MOTION_CMD_MOVE:
        if (s_state == MOTION_STATE_IDLE)
        {
            mc_start_move(m->p1, m->p2);
        }
        else
        {
            mc_reject();
        }
        break;
    case MOTION_CMD_TURN:
        if (s_state == MOTION_STATE_IDLE)
        {
            mc_start_turn(m->p1, m->p2);
        }
        else
        {
            mc_reject();
        }
        break;
    case MOTION_CMD_SET_SPEED:
        if ((s_state == MOTION_STATE_IDLE) || (s_state == MOTION_STATE_DRIVING))
        {
            mc_start_velocity(m->p1, m->p2);
        }
        else
        {
            mc_reject();
        }
        break;
    case MOTION_CMD_STOP:
        if (s_state == MOTION_STATE_IDLE)
        {
            mc_finish_now(MOTION_RESULT_STOPPED);
        }
        else if (s_state != MOTION_STATE_STOPPING)
        {
            mc_begin_stop(MOTION_RESULT_STOPPED);
        }
        else
        {
            /* already stopping: nothing to add */
        }
        break;
    case MOTION_CMD_LINE_FOLLOW:                /* TODO: needs the LineSense interface */
    default:
        mc_reject();
        break;
    }
}

static void mc_drain_commands(void)
{
    motion_cmd_msg_t msg;
    uint32_t n;

    for (n = 0U; n < 4U; n++)
    {
        const INT size = tk_rcv_mbf(s_cmd_mbf, &msg, TMO_POL);

        if (size != (INT)sizeof(msg))
        {
            break;
        }
        mc_handle_cmd(&msg);
    }
}

/* ---------- Status ---------- */

static void mc_publish(void)
{
    motion_status_t st;
    const float track = TRACK_WIDTH_MM;

    st.state = s_state;
    st.fault = s_fault;
    st.last_result = s_result;
    st.estop_sources = s_estop_src;
    // st.timestamp_ms = motion_hw_time_us() / 1000U;
    st.timestamp_ms = s_t_last_ms;
    st.cycle_count = s_cycle_count;
    st.overrun_count = s_overrun_count;
    st.speed_l_mmps = (int16_t)s_speed[0];
    st.speed_r_mmps = (int16_t)s_speed[1];
    st.ticks_l = s_pos_ticks[0];
    st.ticks_r = s_pos_ticks[1];
    st.dist_l_mm = (int32_t)mc_dist_mm(0U);
    st.dist_r_mm = (int32_t)mc_dist_mm(1U);
    st.distance_total_mm = (int32_t)(0.5f * (mc_dist_mm(0U) + mc_dist_mm(1U)));
    st.distance_trip_mm = (int32_t)((float)(((s_pos_ticks[0] + s_pos_ticks[1]) / 2) - s_trip_origin_ticks) * MM_PER_TICK);
    st.heading_deg = (int32_t)(((mc_dist_mm(1U) - mc_dist_mm(0U)) / track) / MC_RAD_PER_DEG);

    if (tk_loc_mtx(s_status_mtx, TMO_POL) == E_OK)  /* never wait: skip this cycle if busy */
    {
        s_shared = st;
        (void)tk_unl_mtx(s_status_mtx);
    }
}

void motion_get_status(motion_status_t *out)
{
    if ((s_status_mtx > 0) && (tk_loc_mtx(s_status_mtx, TMO_FEVR) == E_OK))
    {
        *out = s_shared;
        (void)tk_unl_mtx(s_status_mtx);
    }
    else
    {
        out->state = MOTION_STATE_INIT;
    }
}

/* ---------- The control cycle and the task ---------- */

static void mc_control_cycle(void)
{
    const uint32_t estop = mc_read_estop();

    if (estop != 0U)
    {
        mc_enter_estop(estop);                  /* before anything else */
    }

    mc_update_time();
    mc_update_odometry();
    if (!mc_latched())
    {
        mc_check_stall();
    }
    mc_drain_commands();
    mc_run_state();
    if (!mc_latched())
    {
        mc_apply_outputs();
    }

    s_cycle_count++;
    mc_publish();                               /* status first, then the event bits */
    mc_notify(s_pending_evt);
    s_pending_evt = 0U;
}

static void mc_task(INT stacd, void *exinf)
{
    uint32_t side;
    uint32_t i;

    (void)stacd;
    (void)exinf;

    for (side = 0U; side < MC_SIDES; side++)
    {
        s_raw_last[side] = motion_hw_ticks(side);
        for (i = 0U; i < MC_HIST_LEN; i++)
        {
            s_hist[side][i] = 0;
            // s_hist_t_us[i] = motion_hw_time_us();
            SYSTIM now_tim = {0};
            if (tk_get_otm(&now_tim) >= E_OK)
            {
                s_hist_t_ms[i] = now_tim.lo;
            }
            else
            {
                s_hist_t_ms[i] = 0U;
            }
        }
    }
    s_state = MOTION_STATE_IDLE;

    for (;;)
    {
        INT late;

        (void)tk_slp_tsk(TMO_FEVR);             /* woken by the cyclic handler */
        late = tk_can_wup(TSK_SELF);            /* wake-ups that piled up = we were late */
        if (late > 0)
        {
            s_overrun_count += (uint32_t)late;
        }
        mc_control_cycle();
    }
}

static void mc_cyc_handler(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(s_task_id);
}

/* ---------- Lifecycle ---------- */

bool motion_init(void)
{
    T_CTSK ctsk = {0};
    T_CCYC ccyc = {0};
    T_CMTX cmtx = {0};
    T_CMBF cmbf = {0};
    T_CFLG cflg = {0};

    if (!motion_hw_init())
    {
        return false;
    }

    cmtx.mtxatr = TA_INHERIT;
    s_status_mtx = tk_cre_mtx(&cmtx);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)(MOTION_CMD_QUEUE_DEPTH * (sizeof(motion_cmd_msg_t) + 8U));
    cmbf.maxmsz = (INT)sizeof(motion_cmd_msg_t);
    s_cmd_mbf = tk_cre_mbf(&cmbf);

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    s_estop_flg = tk_cre_flg(&cflg);

    ctsk.tskatr  = TA_HLNG | TA_RNG0;
    ctsk.task    = mc_task;
    ctsk.itskpri = MOTION_TASK_PRIORITY;
    ctsk.stksz   = MOTION_TASK_STACK_BYTES;
    s_task_id = tk_cre_tsk(&ctsk);

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = mc_cyc_handler;
    ccyc.cyctim = MOTION_CONTROL_PERIOD_MS;
    ccyc.cycphs = 0U;
    s_cyc_id = tk_cre_cyc(&ccyc);

    return (s_status_mtx > 0) && (s_cmd_mbf > 0) && (s_estop_flg > 0) &&
           (s_task_id > 0) && (s_cyc_id > 0);
}

bool motion_start(void)
{
    if ((s_task_id <= 0) || (s_cyc_id <= 0))
    {
        return false;
    }
    if (tk_sta_tsk(s_task_id, 0) < E_OK)
    {
        return false;
    }
    return (tk_sta_cyc(s_cyc_id) >= E_OK);
}

void motion_attach_vehicle_evt(int32_t flgid, uint32_t bit_shift)
{
    s_evt_flg = (ID)flgid;
    s_evt_shift = bit_shift;
}

/* ---------- Public command API (VehicleController only) ---------- */

static motion_err_t mc_post(motion_cmd_type_t type, int32_t p1, int32_t p2)
{
    motion_cmd_msg_t msg;
    ER er;

    if (s_cmd_mbf <= 0)
    {
        return MOTION_ERR_INVALID;
    }
    msg.type = type;
    msg.p1 = p1;
    msg.p2 = p2;
    er = tk_snd_mbf(s_cmd_mbf, &msg, (INT)sizeof(msg), TMO_POL);
    if (er >= E_OK)
    {
        return MOTION_OK;
    }
    return (er == E_TMOUT) ? MOTION_ERR_QUEUE_FULL : MOTION_ERR_INVALID;
}

motion_err_t motion_move(int32_t distance_mm, int32_t speed_mmps)
{
    return (speed_mmps > 0) ? mc_post(MOTION_CMD_MOVE, distance_mm, speed_mmps) : MOTION_ERR_INVALID;
}

motion_err_t motion_turn(int32_t angle_deg, int32_t rate_dps)
{
    return (rate_dps > 0) ? mc_post(MOTION_CMD_TURN, angle_deg, rate_dps) : MOTION_ERR_INVALID;
}

motion_err_t motion_line_follow(int32_t speed_mmps)
{
    return (speed_mmps > 0) ? mc_post(MOTION_CMD_LINE_FOLLOW, speed_mmps, 0) : MOTION_ERR_INVALID;
}

motion_err_t motion_set_wheel_speeds(int32_t left_mmps, int32_t right_mmps)
{
    return mc_post(MOTION_CMD_SET_SPEED, left_mmps, right_mmps);
}

motion_err_t motion_stop(void)
{
    return mc_post(MOTION_CMD_STOP, 0, 0);
}

motion_err_t motion_reset(void)
{
    return mc_post(MOTION_CMD_RESET, 0, 0);
}

motion_err_t motion_reset_trip(void)
{
    return mc_post(MOTION_CMD_RESET_TRIP, 0, 0);
}

motion_err_t motion_move_forward(int32_t distance_mm)
{
    return motion_move(distance_mm, MOTION_DEFAULT_SPEED_MMPS);
}

motion_err_t motion_move_backward(int32_t distance_mm)
{
    return motion_move(-distance_mm, MOTION_DEFAULT_SPEED_MMPS);
}

motion_err_t motion_turn_left(int32_t angle_deg)
{
    return motion_turn(angle_deg, MOTION_DEFAULT_TURN_RATE_DPS);
}

motion_err_t motion_turn_right(int32_t angle_deg)
{
    return motion_turn(-angle_deg, MOTION_DEFAULT_TURN_RATE_DPS);
}

motion_err_t motion_u_turn(void)
{
    return motion_turn(180, MOTION_DEFAULT_TURN_RATE_DPS);
}

/* ---------- E-stop ---------- */

void motion_estop_request(uint32_t source_bits)
{
    if (s_estop_flg > 0)
    {
        (void)tk_set_flg(s_estop_flg, source_bits);
    }
}

void motion_estop_clear(uint32_t source_bits)
{
    if (s_estop_flg > 0)
    {
        (void)tk_clr_flg(s_estop_flg, ~source_bits);    /* keeps the bits that are 1 in the mask */
    }
}
