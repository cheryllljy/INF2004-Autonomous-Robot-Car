/**
 * @file    motion_mock.c
 * @brief   HOST TEST ONLY. Buddy 2's motion_control.h API on a simulated
 *          differential-drive car, for the 2-D bypass simulation.
 *
 * Same contract as the real MotionControl as far as subsystem 5's
 * bypass (s5_avoid.c) and vc_turn.c can see it: distance / turn / velocity commands only from IDLE (velocity
 * also while DRIVING), a refused command shows as last_result REJECTED,
 * a finished one goes back to IDLE with DONE (or STOPPED after
 * motion_stop()), the e-stop cuts the motors at once and only
 * motion_reset() after the cause is cleared leaves it.
 *
 * Kinematics on a 1 ms step: per-wheel speed ramps at 400 mm/s^2 (the
 * real MOTION_ACCEL_MMPS2), in-place turns, a minimum crawl speed at
 * the end of a move. The encoders can be given a scale error and the
 * track width a calibration error, so dead reckoning drifts like the
 * real car's.
 */

#include "motion_control.h"
#include "motion_mock.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MM_PI           (3.14159265358979323846)
#define MM_ACCEL        (400.0)     /* mm/s^2, MOTION_ACCEL_MMPS2 */
#define MM_CUT_DECEL    (3000.0)    /* e-stop: PWM off, the car coasts */
#define MM_MIN_SPEED    (40.0)      /* MOTION_MIN_SPEED_MMPS */
#define MM_MIN_RATE     (20.0)      /* deg/s at the end of a turn */
#define MM_STILL        (10.0)      /* MOTION_STOPPED_SPEED_MMPS */
#define MM_STILL_MS     (60)        /* 3 control cycles */
#define MM_TRACK_CFG    (112.0)     /* TRACK_WIDTH_MM the firmware uses */
#define MM_TICK_CFG     (65.0 * MM_PI / 1271.7) /* MM_PER_TICK, mm */

typedef enum { MD_IDLE, MD_MOVE, MD_TURN, MD_VEL, MD_STOPPING } mm_mode_t;

typedef struct
{
    /* truth */
    double x, y, psi;               /* axle centre, mm; heading deg +left */
    double vl, vr;                  /* actual wheel speeds */
    double track_true;
    /* controller */
    mm_mode_t mode;
    motion_state_t state;
    motion_result_t last;
    motion_result_t pending;
    double goal_l, goal_r;
    double target;                  /* move: mm; turn: deg (measured) */
    double origin;                  /* measured distance / heading at start */
    double speed;                   /* move speed or turn rate */
    int    still_ms;
    uint32_t estop;
    /* measured */
    double dl, dr;                  /* encoder distances */
    double scale;                   /* encoder scale error */
    double lr;                      /* nyx, 2026-11-08: left reads long by
                                     * lr / 2, right short by lr / 2 */
    int    dead_r;                  /* v3.2: right encoder reads 0 */
} mm_t;

static mm_t g;
/* nyx, 2026-11-07: optional Buddy 2 timing for the line-following
 * simulation (mm_set_lag): the goal is taken every 20 ms
 * (MOTION_CONTROL_PERIOD_MS), the setpoint ramps at MOTION_ACCEL_MMPS2
 * and the wheel follows the setpoint with a first-order lag (motor, PI
 * on an 80 ms speed window). A setpoint at or under 5 mm/s
 * (MC_ZERO_SPEED) gives no drive; restarting a wheel that reads under
 * 10 mm/s counts as a breakaway kick. 0 = the old ideal wheels. */
static int g_lag_ms;
/* nyx, 2026-11-07: Buddy 2's real wheel loop instead of a first-order lag
 * (mm_set_motor): every 20 ms the speed over the last 80 ms, feed-forward
 * 50 pm + 1.245 pm per mm/s, PI 0.5 / 3.0, duty 0..600, breakaway 120;
 * the motor needs g_m_off pm to move and g_m_ratio x the duty Buddy 2's
 * feed-forward assumes per mm/s, with a 50 ms mechanical lag. */
static int    g_motor;
static double g_m_off = 50.0, g_m_ratio = 1.0;
static double g_m_pos[2], g_m_hist[2][5], g_m_integ[2], g_m_duty[2];
static int    g_m_n;
static double g_set_l, g_set_r;   /* Buddy 2's ramped setpoints */
static double g_take_l, g_take_r; /* goal as taken at the last 20 ms tick */
static int g_tick_ms;
static int g_kicks;
static int g_kick_l, g_kick_r;
static int g_min_cmd = 0x7FFF; /* nyx, 2026-11-06: smallest |wheel speed| asked */

void mm_reset(double x, double y, double psi_deg)
{
    memset(&g, 0, sizeof(g));
    g.x = x;
    g.y = y;
    g.psi = psi_deg;
    g.track_true = MM_TRACK_CFG;
    g.state = MOTION_STATE_IDLE;
    g.last = MOTION_RESULT_NONE;
    g.mode = MD_IDLE;
}

void mm_set_motor(double off_pm, double ratio)
{
    g_motor = 1;
    g_m_off = off_pm;
    g_m_ratio = ratio;
}

static double m_duty_to_v(double duty)
{
    return (duty > g_m_off) ? ((duty - g_m_off) / (1.245 * g_m_ratio)) : 0.0;
}

static void m_control(void)
{
    const double set[2] = { g_set_l, g_set_r };
    double *v[2] = { &g.vl, &g.vr };
    int s;
    int k;

    for (s = 0; s < 2; s++)
    {
        for (k = 4; k > 0; k--) { g_m_hist[s][k] = g_m_hist[s][k - 1]; }
        g_m_hist[s][0] = g_m_pos[s];
    }
    if (g_m_n < 5) { g_m_n++; }
    for (s = 0; s < 2; s++)
    {
        const double mag = fabs(set[s]);
        const double meas = (g_m_n >= 5)
                                ? ((g_m_hist[s][0] - g_m_hist[s][4]) / 0.08)
                                : *v[s];
        double err, integ, u;

        if (mag < 5.0) { g_m_integ[s] = 0.0; g_m_duty[s] = 0.0; continue; }
        err = mag - fabs(meas);
        integ = g_m_integ[s] + (3.0 * err * 0.02);
        if (integ > 300.0) { integ = 300.0; }
        if (integ < -300.0) { integ = -300.0; }
        u = 50.0 + (mag * 1.245) + (0.5 * err) + integ;
        if (!(((u > 600.0) && (err > 0.0)) || ((u < 0.0) && (err < 0.0))))
        {
            g_m_integ[s] = integ;
        }
        if ((fabs(meas) < 10.0) && (u < 120.0)) { u = 120.0; }
        if (u > 600.0) { u = 600.0; }
        if (u < 0.0) { u = 0.0; }
        g_m_duty[s] = (set[s] >= 0.0) ? u : -u;
    }
}

void mm_set_lag(int lag_ms, double v0)
{
    g_lag_ms = lag_ms;
    g_set_l = v0;
    g_set_r = v0;
    g_take_l = v0;
    g_take_r = v0;
    g.goal_l = v0;
    g.goal_r = v0;
    g.vl = v0;
    g.vr = v0;
    if (v0 != 0.0)
    {
        g.mode = MD_VEL;
        g.state = MOTION_STATE_DRIVING;
    }
    g_tick_ms = 0;
    g_kicks = 0;
    g_kick_l = 0;
    g_kick_r = 0;
    {
        /* the wheel loop already settled at v0 */
        const double duty = (v0 > 0.0) ? (g_m_off + (v0 * 1.245 * g_m_ratio)) : 0.0;
        int s;
        int k;

        for (s = 0; s < 2; s++)
        {
            g_m_pos[s] = 0.0;
            for (k = 0; k < 5; k++) { g_m_hist[s][k] = -(double)k * 0.02 * v0; }
            g_m_duty[s] = duty;
            g_m_integ[s] = (v0 > 0.0) ? (duty - (50.0 + (v0 * 1.245))) : 0.0;
        }
        g_m_n = 5;
    }
}

int mm_kicks(void)
{
    return g_kicks;
}

void mm_set_dead_right(int dead)
{
    g.dead_r = dead;
}

void mm_set_errors(double encoder_scale, double track_error)
{
    g.scale = encoder_scale;
    g.track_true = MM_TRACK_CFG * (1.0 + track_error);
    g.lr = 0.0;
}

void mm_set_lr_error(double lr)
{
    g.lr = lr;
}

void mm_get_truth(double *p_x, double *p_y, double *p_psi)
{
    *p_x = g.x;
    *p_y = g.y;
    *p_psi = g.psi;
}

double mm_speed_true(void)
{
    return (g.vl + g.vr) / 2.0;
}

static double meas_dist(void)
{
    return (g.dl + g.dr) / 2.0;
}

static double meas_heading(void)
{
    return ((g.dr - g.dl) / MM_TRACK_CFG) * 180.0 / MM_PI;
}

static double ramp(double v, double goal, double step)
{
    if (v < goal - step) { return v + step; }
    if (v > goal + step) { return v - step; }
    return goal;
}

static void finish(motion_result_t r)
{
    g.goal_l = 0.0;
    g.goal_r = 0.0;
    g.pending = r;
    g.mode = MD_STOPPING;
    g.state = MOTION_STATE_STOPPING;
    g.still_ms = 0;
}

void mm_step_1ms(void)
{
    const double dt = 0.001;
    double step = MM_ACCEL * dt;

    /* ---- controller ---- */
    if (g.state == MOTION_STATE_ESTOP)
    {
        g.goal_l = 0.0;
        g.goal_r = 0.0;
        step = MM_CUT_DECEL * dt;
    }
    else if (g.mode == MD_MOVE)
    {
        const double done = (meas_dist() - g.origin) * ((g.target < 0) ? -1.0 : 1.0);
        const double left = fabs(g.target) - done;
        const double v = fabs((g.vl + g.vr) / 2.0);
        const double brake = (v * v) / (2.0 * MM_ACCEL) + v * 0.02;
        double s = g.speed;

        if (left <= 0.5)
        {
            finish(MOTION_RESULT_DONE);
        }
        else
        {
            if (left <= brake) { s = MM_MIN_SPEED; }
            if (g.target < 0) { s = -s; }
            g.goal_l = s;
            g.goal_r = s;
        }
    }
    else if (g.mode == MD_TURN)
    {
        const double done = (meas_heading() - g.origin) * ((g.target < 0) ? -1.0 : 1.0);
        const double left = fabs(g.target) - done;
        const double wheel = fabs((g.vr - g.vl) / 2.0);
        const double rate_now = wheel * 2.0 / MM_TRACK_CFG * 180.0 / MM_PI;
        const double brake = (rate_now * rate_now) /
                             (2.0 * (MM_ACCEL * 2.0 / MM_TRACK_CFG * 180.0 / MM_PI)) +
                             rate_now * 0.02;
        double rate = g.speed;
        double w;

        if (left <= 0.3)
        {
            finish(MOTION_RESULT_DONE);
        }
        else
        {
            if (left <= brake) { rate = MM_MIN_RATE; }
            w = rate * MM_PI / 180.0 * MM_TRACK_CFG / 2.0;
            if (g.target < 0) { w = -w; }
            g.goal_l = -w;
            g.goal_r = w;
        }
    }
    else if (g.mode == MD_STOPPING)
    {
        g.goal_l = 0.0;
        g.goal_r = 0.0;
        if ((fabs(g.vl) < MM_STILL) && (fabs(g.vr) < MM_STILL) &&
            (fabs(g.vl) < 1e-9) && (fabs(g.vr) < 1e-9))
        {
            g.still_ms++;
        }
        else
        {
            g.still_ms = 0;
        }
        if (g.still_ms >= MM_STILL_MS)
        {
            g.mode = MD_IDLE;
            g.state = MOTION_STATE_IDLE;
            g.last = g.pending;
        }
    }
    else
    {
        /* IDLE / VEL keep their goals */
    }

    if (g_lag_ms > 0)
    {
        if (++g_tick_ms >= 20)
        {
            g_tick_ms = 0;
            g_take_l = g.goal_l;
            g_take_r = g.goal_r;
        }
        g_set_l = ramp(g_set_l, g_take_l, step);
        g_set_r = ramp(g_set_r, g_take_r, step);
        {
            const int drive_l = fabs(g_set_l) > 5.0;
            const int drive_r = fabs(g_set_r) > 5.0;

            const int kick_l = drive_l && (fabs(g.vl) < 10.0);
            const int kick_r = drive_r && (fabs(g.vr) < 10.0);

            if (kick_l && !g_kick_l) { g_kicks++; }
            if (kick_r && !g_kick_r) { g_kicks++; }
            g_kick_l = kick_l;
            g_kick_r = kick_r;
            if (g_motor)
            {
                if (0 == g_tick_ms) { m_control(); }
                {
                    const double ssl = (g_m_duty[0] >= 0.0)
                                           ? m_duty_to_v(g_m_duty[0])
                                           : -m_duty_to_v(-g_m_duty[0]);
                    const double ssr = (g_m_duty[1] >= 0.0)
                                           ? m_duty_to_v(g_m_duty[1])
                                           : -m_duty_to_v(-g_m_duty[1]);

                    g.vl += (ssl - g.vl) * dt * 1000.0 / 50.0;
                    g.vr += (ssr - g.vr) * dt * 1000.0 / 50.0;
                    g_m_pos[0] += fabs(g.vl) * dt;
                    g_m_pos[1] += fabs(g.vr) * dt;
                }
            }
            else
            {
                g.vl += ((drive_l ? g_set_l : 0.0) - g.vl) * dt * 1000.0 / g_lag_ms;
                g.vr += ((drive_r ? g_set_r : 0.0) - g.vr) * dt * 1000.0 / g_lag_ms;
            }
        }
    }
    else
    {
        g.vl = ramp(g.vl, g.goal_l, step);
        g.vr = ramp(g.vr, g.goal_r, step);
    }

    /* ---- truth ---- */
    {
        const double v = (g.vl + g.vr) / 2.0;
        const double w = (g.vr - g.vl) / g.track_true;  /* rad/s, +left */
        const double mid = (g.psi * MM_PI / 180.0) + (w * dt / 2.0);

        g.x += v * sin(mid) * dt;
        g.y += v * cos(mid) * dt;
        g.psi += w * dt * 180.0 / MM_PI;
    }

    /* ---- encoders ---- */
    g.dl += g.vl * dt * (1.0 + g.scale + (g.lr / 2.0));
    if (!g.dead_r) { g.dr += g.vr * dt * (1.0 + g.scale - (g.lr / 2.0)); }
}

/*=====================================================================*/
/* motion_control.h                                                    */
/*=====================================================================*/

static motion_err_t refuse(void)
{
    g.last = MOTION_RESULT_REJECTED;
    return MOTION_OK;           /* queued; MotionTask refuses it later */
}

motion_err_t motion_move(int32_t distance_mm, int32_t speed_mmps)
{
    if ((g.state != MOTION_STATE_IDLE) || (speed_mmps <= 0))
    {
        return refuse();
    }
    g.mode = MD_MOVE;
    g.state = MOTION_STATE_DRIVING;
    g.target = (double)distance_mm;
    g.origin = meas_dist();
    g.speed = (double)speed_mmps;
    if (distance_mm == 0)
    {
        finish(MOTION_RESULT_DONE);
    }
    return MOTION_OK;
}

motion_err_t motion_turn(int32_t angle_deg, int32_t rate_dps)
{
    if ((g.state != MOTION_STATE_IDLE) || (rate_dps <= 0))
    {
        return refuse();
    }
    g.mode = MD_TURN;
    g.state = MOTION_STATE_TURNING;
    g.target = (double)angle_deg;
    g.origin = meas_heading();
    g.speed = (double)rate_dps;
    return MOTION_OK;
}

motion_err_t motion_set_wheel_speeds(int32_t l, int32_t r)
{
    if ((g.state != MOTION_STATE_IDLE) &&
        !((g.state == MOTION_STATE_DRIVING) && (g.mode == MD_VEL)))
    {
        return refuse();
    }
    /* nyx, 2026-11-06: like Buddy 2's mc_start_velocity, (0, 0) is a stop,
     * and further speeds are refused until it is IDLE again */
    if ((0 == l) && (0 == r))
    {
        if (g.state == MOTION_STATE_IDLE)
        {
            g.last = MOTION_RESULT_STOPPED;
        }
        else
        {
            finish(MOTION_RESULT_STOPPED);
        }
        return MOTION_OK;
    }
    if ((0 != l) && (abs((int)l) < g_min_cmd))
    {
        g_min_cmd = abs((int)l);
    }
    if ((0 != r) && (abs((int)r) < g_min_cmd))
    {
        g_min_cmd = abs((int)r);
    }
    g.mode = MD_VEL;
    g.state = MOTION_STATE_DRIVING;
    g.goal_l = (double)l;
    g.goal_r = (double)r;
    return MOTION_OK;
}

int mm_min_wheel_cmd(int restart)
{
    const int m = g_min_cmd;

    if (restart)
    {
        g_min_cmd = 0x7FFF;
    }
    return m;
}

motion_err_t motion_stop(void)
{
    if (g.state == MOTION_STATE_ESTOP)
    {
        return refuse();
    }
    if (g.state == MOTION_STATE_IDLE)
    {
        g.last = MOTION_RESULT_STOPPED;
    }
    else if (g.mode != MD_STOPPING)
    {
        finish(MOTION_RESULT_STOPPED);
    }
    else
    {
        /* already stopping */
    }
    return MOTION_OK;
}

motion_err_t motion_reset(void)
{
    if (g.state == MOTION_STATE_ESTOP)
    {
        if (g.estop != 0U)
        {
            return refuse();
        }
        g.vl = 0.0;
        g.vr = 0.0;
    }
    g.mode = MD_IDLE;
    g.state = MOTION_STATE_IDLE;
    g.goal_l = 0.0;
    g.goal_r = 0.0;
    g.last = MOTION_RESULT_DONE;
    return MOTION_OK;
}

void motion_estop_request(uint32_t source_bits)
{
    g.estop |= source_bits;
    g.mode = MD_IDLE;
    g.state = MOTION_STATE_ESTOP;
    g.last = MOTION_RESULT_ESTOP;
}

void motion_estop_clear(uint32_t source_bits)
{
    g.estop &= ~source_bits;
}

void motion_get_status(motion_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->state = g.state;
    out->last_result = g.last;
    out->estop_sources = g.estop;
    out->speed_l_mmps = (int16_t)lround(g.vl * (1.0 + g.scale));
    out->speed_r_mmps = g.dead_r ? 0 : (int16_t)lround(g.vr * (1.0 + g.scale));
    out->dist_l_mm = (int32_t)lround(g.dl);
    out->dist_r_mm = (int32_t)lround(g.dr);
    /* the encoder ticks (s5_odom.c dead reckons from them) */
    out->ticks_l = (int32_t)lround(g.dl / MM_TICK_CFG);
    out->ticks_r = (int32_t)lround(g.dr / MM_TICK_CFG);
    out->distance_total_mm = (int32_t)lround(meas_dist());
    out->heading_deg = (int32_t)lround(meas_heading());
}
