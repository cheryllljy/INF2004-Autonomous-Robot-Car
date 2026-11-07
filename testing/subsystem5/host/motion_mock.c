/**
 * @file    motion_mock.c
 * @brief   HOST TEST ONLY. Buddy 2's motion_control.h API on a simulated
 *          differential-drive car, for the 2-D bypass simulation.
 *
 * Same contract as the real MotionControl as far as vc_obstacle.c can
 * see it: distance / turn / velocity commands only from IDLE (velocity
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
    int    dead_r;                  /* v3.2: right encoder reads 0 */
} mm_t;

static mm_t g;
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

void mm_set_dead_right(int dead)
{
    g.dead_r = dead;
}

void mm_set_errors(double encoder_scale, double track_error)
{
    g.scale = encoder_scale;
    g.track_true = MM_TRACK_CFG * (1.0 + track_error);
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

    g.vl = ramp(g.vl, g.goal_l, step);
    g.vr = ramp(g.vr, g.goal_r, step);

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
    g.dl += g.vl * dt * (1.0 + g.scale);
    if (!g.dead_r) { g.dr += g.vr * dt * (1.0 + g.scale); }
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
    out->distance_total_mm = (int32_t)lround(meas_dist());
    out->heading_deg = (int32_t)lround(meas_heading());
}
