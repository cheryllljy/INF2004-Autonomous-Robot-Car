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
/* Buddy 5, 2026-11-20: Buddy 2's fix (subsystem2 v4.7). 1 = model it:
 * in wheel-speed mode the mean ramps at MM_ACCEL but the curve shape at
 * MM_STEER_ACCEL; SET_SPEED is taken while STOPPING; speed over 40 ms.
 * 0 = Buddy 2 before the fix. */
#ifndef MM_S2_V2
#define MM_S2_V2 1
#endif
#define MM_STEER_ACCEL  (2000.0)    /* MOTION_STEER_ACCEL_MMPS2 */
#if MM_S2_V2
#define MM_WIN 2                    /* SPEED_WINDOW_PERIODS */
#else
#define MM_WIN 4
#endif
#define MM_CUT_DECEL    (3000.0)    /* e-stop: PWM off, the car coasts */
#define MM_MIN_SPEED    (40.0)      /* MOTION_MIN_SPEED_MMPS */
#define MM_MIN_RATE     (20.0)      /* deg/s at the end of a turn */
#define MM_STILL        (10.0)      /* MOTION_STOPPED_SPEED_MMPS */
#define MM_STILL_MS     (60)        /* 3 control cycles */
#define MM_TRACK_CFG    (112.0)     /* TRACK_WIDTH_MM the firmware uses */
#define MM_TICK_CFG     (65.0 * MM_PI / 1271.7) /* MM_PER_TICK, mm */
/* the motor's mechanical lag, ms (host tests try others) */
#ifndef MM_MOTOR_MS
#define MM_MOTOR_MS (50.0)
#endif

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
    double lr;                      /* left reads long by
                                     * lr / 2, right short by lr / 2 */
    int    dead_r;                  /* v3.2: right encoder reads 0 */
} mm_t;

static mm_t g;
/* optional Buddy 2 timing for the line-following
 * simulation (mm_set_lag): the goal is taken every 20 ms
 * (MOTION_CONTROL_PERIOD_MS), the setpoint ramps at MOTION_ACCEL_MMPS2
 * and the wheel follows the setpoint with a first-order lag (motor, PI
 * on an 80 ms speed window). A setpoint at or under 5 mm/s
 * (MC_ZERO_SPEED) gives no drive; restarting a wheel that reads under
 * 10 mm/s counts as a breakaway kick. 0 = the old ideal wheels. */
static int g_lag_ms;
/* Buddy 2's real wheel loop instead of a first-order lag
 * (mm_set_motor): every 20 ms the speed over the last 80 ms, feed-forward
 * 50 pm + 1.245 pm per mm/s, PI 0.5 / 3.0, duty 0..600, breakaway 120;
 * the motor needs g_m_off pm to move and g_m_ratio x the duty Buddy 2's
 * feed-forward assumes per mm/s, with a 50 ms mechanical lag. */
static int    g_motor;
static double g_m_off = 50.0, g_m_ratio = 1.0;
static double g_m_brk = 0.0;
void mm_set_stiction(double brk) { g_m_brk = brk; }
/* Stribeck friction (experiment): a stopped wheel needs g_sb_ks pm; moving,
 * friction falls from g_sb_ks to g_sb_kk with speed (scale g_sb_vs mm/s);
 * speed per pm above friction 1 / (1.245 g_m_ratio) */
static double g_sb_ks, g_sb_kk, g_sb_vs;
static int g_sb_on;
void mm_set_stribeck(double ks, double kk, double vs) { g_sb_ks = ks; g_sb_kk = kk; g_sb_vs = vs; g_sb_on = 1; }
static double sb_step(double v, double duty, double load, double dt)
{
    const double u = (fabs(duty) > 0.0) ? (fabs(duty) - load) : 0.0;
    const double sg = (duty >= 0.0) ? 1.0 : -1.0;
    double a = fabs(v);
    if ((a < 0.5) && (u < g_sb_ks)) { return 0.0; }
    {
        const double f = g_sb_kk + (g_sb_ks - g_sb_kk) * exp(-a / g_sb_vs);
        const double tgt = (u - f) / (1.245 * g_m_ratio);
        a += (tgt - a) * dt * 1000.0 / MM_MOTOR_MS;
        if (a < 0.0) { a = 0.0; }
    }
    return sg * a;
}
static double g_m_pos[2], g_m_hist[2][5], g_m_integ[2], g_m_duty[2];
static int    g_m_n;
static double g_set_l, g_set_r;   /* Buddy 2's ramped setpoints */
static double g_take_l, g_take_r; /* goal as taken at the last 20 ms tick */
static int g_tick_ms;
static int g_kicks;
static int g_kick_l, g_kick_r;
static int g_min_cmd = 0x7FFF; /* smallest |wheel speed| asked */
/* Buddy 2's measured wheel speeds (encoder distance over
 * the last 80 ms, updated every 20 ms), as the VehicleController and its
 * curve log see them (motion_status_t speed_l/r_mmps) */
static double g_ms_hist[2][5];
static double g_ms_v[2];
static int    g_ms_tick;
/* turning load (mm_set_yaw_load). Turning the car takes a
 * yaw torque (caster, tyres), so the outer wheel needs g_y_pm more drive
 * and the inner wheel is pushed along by g_y_pm; in full from a wheel
 * speed difference of g_y_w mm/s. 0 = off (the old model). */
static double g_y_pm, g_y_w = 1.0;
void mm_set_yaw_load(double pm, double w)
{
    g_y_pm = pm;
    g_y_w = (w > 0.1) ? w : 0.1;
}
/* load on wheel side (0 L, 1 R) against its own direction of drive, pm */
static double yaw_load(int side, double duty)
{
    double f = (g.vr - g.vl) / g_y_w;
    const double sg = (duty >= 0.0) ? 1.0 : -1.0;

    if (f > 1.0) { f = 1.0; }
    if (f < -1.0) { f = -1.0; }
    return (0 == side) ? (-g_y_pm * f * sg) : (g_y_pm * f * sg);
}
void mm_wheels_meas(double *l, double *r) { *l = g_ms_v[0]; *r = g_ms_v[1]; }

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

/* Buddy 2's MOTION_SPEED_KP / KI (motion_config.h), so
 * host tests can try other gains. KP 1.6 as in motion_config.h since
 * 2026-11-09 (was 0.5); -DMM_KP=0.5 is Buddy 2 as it was. */
#ifndef MM_KP
#define MM_KP (1.6)
#endif
#ifndef MM_KI
#define MM_KI (3.0)
#endif
/* experiment: 1 = when the setpoint changes, the
 * integrator is rescaled as if it were a gain error (integ x new / old
 * setpoint), so what it learnt at 100 mm/s does not overdrive a wheel
 * just slowed to 20 mm/s */
#ifndef MM_ISCALE
#define MM_ISCALE 0
#endif
static double g_m_prev_set[2];
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
        const double meas = (g_m_n >= (MM_WIN + 1))
                                ? ((g_m_hist[s][0] - g_m_hist[s][MM_WIN]) /
                                   (0.02 * MM_WIN))
                                : *v[s];
        double err, integ, u;

        if (mag < 5.0)
        {
            g_m_integ[s] = 0.0;
            g_m_duty[s] = 0.0;
            g_m_prev_set[s] = 0.0;
            continue;
        }
#if MM_ISCALE
        if ((g_m_prev_set[s] >= 5.0) && (mag < g_m_prev_set[s]))
        {
            g_m_integ[s] *= mag / g_m_prev_set[s];
        }
#endif
        g_m_prev_set[s] = mag;
        err = mag - fabs(meas);
        integ = g_m_integ[s] + (MM_KI * err * 0.02);
        if (integ > 300.0) { integ = 300.0; }
        if (integ < -300.0) { integ = -300.0; }
        u = 50.0 + (mag * 1.245) + (MM_KP * err) + integ;
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
    {
        int k;

        g_ms_tick = 0;
        for (k = 0; k < 5; k++)
        {
            g_ms_hist[0][k] = g.dl - ((double)k * 0.02 * v0);
            g_ms_hist[1][k] = g.dr - ((double)k * 0.02 * v0);
        }
        g_ms_v[0] = v0;
        g_ms_v[1] = v0;
    }
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
            g_m_prev_set[s] = v0;
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

void mm_wheels_true(double *l, double *r) { *l = g.vl; *r = g.vr; }
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

/* Buddy 5, 2026-11-20: Buddy 2's mc_ramp_velocity() (MM_S2_V2). Returns 0
 * when it does not apply and the per-wheel ramp should run. */
static int g_c_on;
static double g_c_mean, g_c_ratio;
static int curve_ramp(double *pl, double *pr, double gl, double gr,
                      double step, double dt)
{
    const double mg = 0.5 * (gl + gr);
    const double mn = 0.5 * (*pl + *pr);
    double rg, rs, den;
    double *p[2];
    double gg[2];
    int s;

    if (!MM_S2_V2 || (g.mode != MD_VEL) ||
        (g.state != MOTION_STATE_DRIVING) || (fabs(mg) < 5.0) ||
        (gl * gr < 0.0) || (*pl * gl < 0.0) || (*pr * gr < 0.0))
    {
        g_c_on = 0;
        return 0;
    }
    rg = (gr - gl) / (2.0 * mg);
    if (!g_c_on)
    {
        g_c_mean = mn;
        g_c_ratio = (fabs(mn) >= 5.0) ? ((*pr - *pl) / (2.0 * mn)) : rg;
        g_c_on = 1;
    }
    den = fabs(g_c_mean);
    if (den < 50.0) { den = 50.0; }
    rs = MM_STEER_ACCEL * dt / den;
    g_c_mean = ramp(g_c_mean, mg, step);
    g_c_ratio = ramp(g_c_ratio, rg, rs);
    p[0] = pl; p[1] = pr; gg[0] = gl; gg[1] = gr;
    for (s = 0; s < 2; s++)
    {
        const double old = *p[s];
        const double lo = (old < gg[s]) ? old : gg[s];
        const double hi = (old < gg[s]) ? gg[s] : old;
        const double plain = ramp(old, gg[s], step);
        double c = g_c_mean * ((0 == s) ? (1.0 - g_c_ratio) : (1.0 + g_c_ratio));

        if (c < lo) { c = lo; }
        if (c > hi) { c = hi; }
        *p[s] = (fabs(gg[s] - c) < fabs(gg[s] - plain)) ? c : plain;
    }
    return 1;
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
        /* as Buddy 2 (MC_ZERO_SPEED 5): its ramped
         * setpoints at zero, the wheels under MOTION_STOPPED_SPEED_MMPS.
         * Was the wheels at exactly 0, which with the wheel lag took
         * 1.2 s, not Buddy 2's 3 cycles */
        const double set_l = (g_lag_ms > 0) ? g_set_l : g.vl;
        const double set_r = (g_lag_ms > 0) ? g_set_r : g.vr;

        if ((fabs(g.vl) < MM_STILL) && (fabs(g.vr) < MM_STILL) &&
            (fabs(set_l) < 5.0) && (fabs(set_r) < 5.0))
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
        if (!curve_ramp(&g_set_l, &g_set_r, g_take_l, g_take_r, step, dt))
        {
            g_set_l = ramp(g_set_l, g_take_l, step);
            g_set_r = ramp(g_set_r, g_take_r, step);
        }
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
                    const double ll = yaw_load(0, g_m_duty[0]);
                    const double lr = yaw_load(1, g_m_duty[1]);
                    const double ssl = (0.0 == g_m_duty[0]) ? 0.0
                                       : ((g_m_duty[0] > 0.0)
                                           ? m_duty_to_v(g_m_duty[0] - ll)
                                           : -m_duty_to_v(-g_m_duty[0] - ll));
                    const double ssr = (0.0 == g_m_duty[1]) ? 0.0
                                       : ((g_m_duty[1] > 0.0)
                                           ? m_duty_to_v(g_m_duty[1] - lr)
                                           : -m_duty_to_v(-g_m_duty[1] - lr));

                    if (g_sb_on)
                    {
                        g.vl = sb_step(g.vl, g_m_duty[0], ll, dt);
                        g.vr = sb_step(g.vr, g_m_duty[1], lr, dt);
                        g_m_pos[0] += fabs(g.vl) * dt;
                        g_m_pos[1] += fabs(g.vr) * dt;
                    }
                    else
                    {
                    double sl = ssl, sr = ssr;
                    if ((g_m_brk > 0.0) && (fabs(g.vl) < 3.0) && (fabs(g_m_duty[0]) < g_m_brk)) { sl = 0.0; }
                    if ((g_m_brk > 0.0) && (fabs(g.vr) < 3.0) && (fabs(g_m_duty[1]) < g_m_brk)) { sr = 0.0; }
                    g.vl += (sl - g.vl) * dt * 1000.0 / MM_MOTOR_MS;
                    g.vr += (sr - g.vr) * dt * 1000.0 / MM_MOTOR_MS;
                    g_m_pos[0] += fabs(g.vl) * dt;
                    g_m_pos[1] += fabs(g.vr) * dt;
                    }
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
        if (!curve_ramp(&g.vl, &g.vr, g.goal_l, g.goal_r, step, dt))
        {
            g.vl = ramp(g.vl, g.goal_l, step);
            g.vr = ramp(g.vr, g.goal_r, step);
        }
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

    /* Buddy 2's speed window (MM_WIN x 20 ms), every 20 ms */
    if (++g_ms_tick >= 20)
    {
        int k;

        g_ms_tick = 0;
        for (k = 4; k > 0; k--)
        {
            g_ms_hist[0][k] = g_ms_hist[0][k - 1];
            g_ms_hist[1][k] = g_ms_hist[1][k - 1];
        }
        g_ms_hist[0][0] = g.dl;
        g_ms_hist[1][0] = g.dr;
        g_ms_v[0] = (g_ms_hist[0][0] - g_ms_hist[0][MM_WIN]) / (0.02 * MM_WIN);
        g_ms_v[1] = (g_ms_hist[1][0] - g_ms_hist[1][MM_WIN]) / (0.02 * MM_WIN);
    }
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
        !((g.state == MOTION_STATE_DRIVING) && (g.mode == MD_VEL)) &&
        !(MM_S2_V2 && (g.state == MOTION_STATE_STOPPING)))
    {
        return refuse();
    }
    /* Buddy 2 v4.7: (0, 0) while already stopping changes nothing */
    if (MM_S2_V2 && (g.state == MOTION_STATE_STOPPING) && (0 == l) && (0 == r))
    {
        return MOTION_OK;
    }
    /* like Buddy 2's mc_start_velocity, (0, 0) is a stop,
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
