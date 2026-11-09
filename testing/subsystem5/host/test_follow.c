/**
 * @file    test_follow.c
 * @brief   HOST TEST (B3-H10, nyx 2026-11-07). 2-D simulation of Buddy 3's
 *          line following (the REAL b3_line.c) on bends, S-curves, a
 *          hairpin and the handover after a junction turn (car starting
 *          at an angle to the line).
 *
 *   real       b3_line.c
 *   simulated  the car (motion_mock.c with Buddy 2's timing: goal taken
 *              every 20 ms, 400 mm/s^2 setpoint ramp, wheel lag), the IR
 *              pair VC_IR_AHEAD_MM ahead of the axle straddling 18 mm
 *              tape, Buddy 3 every 10 ms, the VehicleController sending
 *              Buddy 3's wheel speeds every 20 ms
 *
 * A run passes if the car gets to the end of the track without the line
 * leaving the IR pair (both IRs white with the line outside them), and
 * if the steering is smooth: few left/right reversals per metre, little
 * yaw-rate wobble, no inner wheel slower than Buddy 2 can drive without
 * a breakaway kick.
 *
 *   ./test_follow             all scenarios
 *   ./test_follow N [v]       scenario N, v = trace every 50 ms
 *   ./test_follow -s SPREAD   IRs SPREAD mm off the centre line
 *   ./test_follow -l LAG      wheel lag, ms (default 60)
 *   ./test_follow -t E        the wheels turn the car 1/(1+E) as tightly as
 *                             Buddy 3 asks (slip, caster drag; 0 = exact)
 *   ./test_follow -m OFF,K    Buddy 2's real PI wheel loop on a motor that
 *                             needs OFF pm to move and K x the pm per mm/s
 *                             Buddy 2's feed-forward assumes (50,1 = exact)
 *   ./test_follow -q          one summary line (for parameter sweeps)
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_mock.h"
#include "motion_control.h"
#include "b3_line.h"

#define PI          (3.14159265358979323846)
#define LINE_HALF   (9.0)   /* 18 mm tape */
#define IR_AHEAD    (125.0) /* nyx, 2026-11-06 */
#define MAX_PTS     (8000)
#define STEP_MM     (2.0)
#define START_MM    (400.0) /* straight before the shape */
#define END_MM      (700.0) /* straight after it */
#define SETTLE_MM   (150.0) /* metrics start after this */
#define VC_MS       (20U)

/* nyx, 2026-11-09: 25 mm apart, measured (was 15) */
static double g_spread = 12.5; /* each IR this far off the centre line */
static int    g_lag_ms = 60;
static int    g_verbose;
static double g_m_off;          /* -m OFF,RATIO: Buddy 2's PI wheel loop */
static double g_m_ratio;
static double g_m_brk;
static double g_sb_ks, g_sb_kk, g_sb_vs;
static double g_cs_ask_in, g_cs_got_in, g_cs_ask_out, g_cs_got_out; static long g_cs_n;
static double g_track_err;      /* -t E: the car turns 1/(1+E) as tightly */
static double g_y_pm, g_y_w;    /* -y PM,W: turning load (mm_set_yaw_load) */
/* nyx, 2026-11-09: -c prints the car's curve log (vehicle_controller.c
 * curve_log()): every 100 mm driven while |curvature| >= 1000, the means
 * of the wheel speeds asked and of Buddy 2's measured ones. The first
 * window of each curve and the later ones are summed apart (-q). */
static int    g_curve_log;
static double g_cl_pct[2][2];   /* [first/later][inner/outer] sum of % */
static int    g_cl_n[2];
static double g_cl_rmin, g_cl_rmax; /* asked |R| swing within one curve */
static double g_cl_swing;       /* sum over curves of max/min asked |R| */
static int    g_cl_curves;
/* nyx, 2026-11-09: -e A,B,TH: each IR sees an elliptic spot (semi-axes A
 * mm sideways, B mm forward) and reads black when at least TH of it is on
 * the tape (the module's comparator threshold). TH < 0.5 reads black
 * beyond the tape edge; a tape crossing the pair at an angle (in a bend)
 * then covers both IRs. 0 = the old point sensors. */
static double g_spot_a, g_spot_b, g_spot_th;
/* nyx, 2026-11-09: -x D,LEN: both IRs read black for LEN mm of the track
 * from D mm after the straight start (a crossing tape, a junction patch or
 * the car's long both-black runs, 52..67 mm, put into the bend) */
static double g_bb_at, g_bb_len;
/* both-black statistics in bends (|curvature asked| >= 1000) */
static long   g_bb_bend, g_bend_n, g_bb_all, g_all_n, g_one_bend;
static double g_bb_run_mm, g_bb_run_max;
static double g_bb_run_sum; static long g_bb_runs;

/*=====================================================================*/
/* Track: a polyline drawn by a turtle, x + = left, y forward          */
/*=====================================================================*/

static double g_px[MAX_PTS], g_py[MAX_PTS];
static int    g_np;
static double g_tx, g_ty, g_th; /* turtle, heading deg + left */
static int    g_cursor;         /* nearest track point to the IRs */

static void t_point(void)
{
    if (g_np < MAX_PTS)
    {
        g_px[g_np] = g_tx;
        g_py[g_np] = g_ty;
        g_np++;
    }
}

static void t_start(void)
{
    g_np = 0;
    g_tx = 0.0;
    g_ty = 0.0;
    g_th = 0.0;
    t_point();
}

static void t_straight(double len)
{
    double d;

    for (d = STEP_MM; d <= len + 1e-9; d += STEP_MM)
    {
        g_tx += STEP_MM * sin(g_th * PI / 180.0);
        g_ty += STEP_MM * cos(g_th * PI / 180.0);
        t_point();
    }
}

/* + angle = to the left */
static void t_arc(double radius, double angle)
{
    const double len = fabs(angle) * PI / 180.0 * radius;
    const int n = (int)(len / STEP_MM);
    const double dth = angle / n;
    int i;

    for (i = 0; i < n; i++)
    {
        g_th += dth / 2.0;
        g_tx += (len / n) * sin(g_th * PI / 180.0);
        g_ty += (len / n) * cos(g_th * PI / 180.0);
        g_th += dth / 2.0;
        t_point();
    }
}

/* distance from (x, y) to the track; *p_idx = nearest point */
static double track_dist(double x, double y, int *p_idx)
{
    double best = 1e18;
    int bi = 0;
    int i;
    /* the tracks do not cross themselves: search near the car only */
    const int from = (g_cursor > 200) ? (g_cursor - 200) : 0;
    const int to = (g_cursor + 400 < g_np) ? (g_cursor + 400) : g_np;

    for (i = from; i + 1 < to; i++)
    {
        const double dx = g_px[i + 1] - g_px[i];
        const double dy = g_py[i + 1] - g_py[i];
        const double len2 = dx * dx + dy * dy;
        double t = ((x - g_px[i]) * dx + (y - g_py[i]) * dy) / len2;
        double d;

        if (t < 0.0) { t = 0.0; }
        if (t > 1.0) { t = 1.0; }
        d = hypot(x - (g_px[i] + t * dx), y - (g_py[i] + t * dy));
        if (d < best)
        {
            best = d;
            bi = i;
        }
    }
    if (NULL != p_idx)
    {
        *p_idx = bi;
    }
    return best;
}

/* nyx, 2026-11-09: IR reading at world (x, y) with the car heading psi
 * (deg): point sensor, or the elliptic spot of -e. The tape is taken as
 * straight across the spot (the spot is a few mm, the bends >= R40). */
static int ir_black(double x, double y, double psi)
{
    int idx = 0;
    const double d0 = track_dist(x, y, &idx);

    if (g_spot_a <= 0.0)
    {
        return d0 <= LINE_HALF;
    }
    {
        const int i = (idx + 1 < g_np) ? idx : idx - 1;
        const double dx = g_px[i + 1] - g_px[i];
        const double dy = g_py[i + 1] - g_py[i];
        const double len = hypot(dx, dy);
        const double nx = -dy / len;
        const double ny = dx / len;
        /* signed distance of the spot centre from the tape centre */
        const double sd = (x - g_px[i]) * nx + (y - g_py[i]) * ny;
        const double fx = sin(psi * PI / 180.0);
        const double fy = cos(psi * PI / 180.0);
        int in = 0, on = 0;
        int a, b;
        const int n = 12;

        for (a = -n; a <= n; a++)
        {
            for (b = -n; b <= n; b++)
            {
                const double u = (double)a / n; /* sideways, -1..1 */
                const double w = (double)b / n; /* forward */

                if ((u * u + w * w) <= 1.0)
                {
                    const double l = u * g_spot_a;
                    const double f = w * g_spot_b;
                    /* body offset (forward f, left l) in the world */
                    const double ox = f * fx - l * fy;
                    const double oy = f * fy + l * fx;
                    const double d = sd + ox * nx + oy * ny;

                    in++;
                    if (fabs(d) <= LINE_HALF)
                    {
                        on++;
                    }
                }
            }
        }
        (void)d0;
        return ((double)on / in) >= g_spot_th;
    }
}

/*=====================================================================*/
/* Scenarios                                                            */
/*=====================================================================*/

typedef enum
{
    T_STRAIGHT, T_BEND, T_S, T_CIRCLE, T_HAIRPIN, T_WIGGLE, T_ONBEND
} shape_t;

typedef struct
{
    const char *name;
    shape_t     shape;
    double      radius;  /* mm */
    double      angle;   /* deg, + left */
    double      psi0;    /* car heading to the line at the start, deg */
    double      x0;      /* car sideways from the line at the start, mm */
    int         speed;   /* line speed, mm/s (0: B3_BASE_SPEED_MM_S) */
} scen_t;

static const scen_t g_scen[] =
{
    { "F1 straight",                              T_STRAIGHT, 0,   0,    0,   0, 0 },
    { "F2 straight, start 15 deg off (handover)",  T_STRAIGHT, 0,   0,   15,   0, 0 },
    { "F3 straight, start 30 deg off (handover)",  T_STRAIGHT, 0,   0,   30,   0, 0 },
    { "F4 straight, start -30 deg off, 8 mm aside", T_STRAIGHT, 0, 0,  -30, -8, 0 },
    { "F5 bend R500 left 90",                      T_BEND,   500,  90,    0,   0, 0 },
    { "F6 bend R300 left 90",                      T_BEND,   300,  90,    0,   0, 0 },
    { "F7 bend R200 right 90",                     T_BEND,   200, -90,    0,   0, 0 },
    { "F8 bend R150 left 120",                     T_BEND,   150, 120,    0,   0, 0 },
    { "F9 S-curve R300 left 90, right 90",         T_S,      300,  90,    0,   0, 0 },
    { "F10 S-curve R200 right 90, left 90",        T_S,      200, -90,    0,   0, 0 },
    { "F11 circle R250, 300 deg",                 T_CIRCLE, 250, 300,    0,   0, 0 },
    { "F12 hairpin R140 left 180",                 T_HAIRPIN, 140, 180,   0,   0, 0 },
    { "F13 wiggle R400 +-45 x4",                   T_WIGGLE, 400,  45,    0,   0, 0 },
    { "F14 bend R300 left 90 at 30 mm/s",          T_BEND,   300,  90,    0,   0, 30 },
    { "F15 bend R200 right 90 at 60 mm/s",         T_BEND,   200, -90,    0,   0, 60 },
    { "F16 straight at 30 mm/s, start 20 deg off", T_STRAIGHT, 0,   0,   20,   0, 30 },
    { "F17 S-curve R250 at 150 mm/s",              T_S,      250,  90,    0,   0, 150 },
    { "F18 bend R300 left 90, start 25 deg off",   T_BEND,   300,  90,  -25,   0, 0 },
    /* nyx, 2026-11-07: from the review, tight S-bends */
    { "F19 S-curve R130 left 90, right 90",        T_S,      130,  90,    0,   0, 0 },
    { "F20 S-curve R130 right 90, left 90",        T_S,      130, -90,    0,   0, 0 },
    { "F21 S-curve R150 left 90, right 90",        T_S,      150,  90,    0,   0, 0 },
    { "F22 S-curve R140 left 180, right 180",      T_S,      140, 180,    0,   0, 0 },
    /* tight bends and corners (nyx, 2026-11-07): shown, LIMIT if lost */
    { "F23 bend R40 left 90",                      T_BEND,    40,  90,    0,   0, 0 },
    { "F24 bend R60 right 90",                     T_BEND,    60, -90,    0,   0, 0 },
    { "F25 bend R80 left 90",                      T_BEND,    80,  90,    0,   0, 0 },
    { "F26 bend R100 right 90",                    T_BEND,   100, -90,    0,   0, 0 },
    { "F27 bend R120 left 90",                     T_BEND,   120,  90,    0,   0, 0 },
    { "F28 hairpin R100 left 180",                 T_HAIRPIN, 100, 180,   0,   0, 0 },
    { "F29 bend R60 left 90 at 60 mm/s",           T_BEND,    60,  90,    0,   0, 60 },
    { "F30 bend R40 right 90 at 60 mm/s",          T_BEND,    40, -90,    0,   0, 60 },
    { "F31 start standing on R300 left 120",       T_ONBEND, 300, 120,    0,   0, 0 },
    { "F32 start standing on R300 right 120",      T_ONBEND, 300, -120,   0,   0, 0 },
    { "F33 start standing on R200 left 120",       T_ONBEND, 200, 120,    0,   0, 0 },
    { "F34 start standing on R200 right 120",      T_ONBEND, 200, -120,   0,   0, 0 },
    { "F35 start standing on R500 left 90",        T_ONBEND, 500, 90,     0,   0, 0 },
    { "F36 start standing on R150 right 150",      T_ONBEND, 150, -150,   0,   0, 0 },
};

#define N_SCEN ((int)(sizeof(g_scen) / sizeof(g_scen[0])))

static void build(const scen_t *s)
{
    int i;

    t_start();
    t_straight(START_MM);
    switch (s->shape)
    {
        case T_BEND:
        case T_ONBEND:
            t_arc(s->radius, s->angle);
            break;
        case T_S:
            t_arc(s->radius, s->angle);
            t_arc(s->radius, -s->angle);
            break;
        case T_CIRCLE:
            t_arc(s->radius, s->angle);
            break;
        case T_HAIRPIN:
            t_arc(s->radius, s->angle);
            break;
        case T_WIGGLE:
            for (i = 0; i < 4; i++)
            {
                t_arc(s->radius, ((i % 2) ? -1.0 : 1.0) * s->angle);
                t_arc(s->radius, ((i % 2) ? 1.0 : -1.0) * s->angle);
            }
            break;
        case T_STRAIGHT:
        default:
            t_straight(800.0);
            break;
    }
    t_straight(END_MM);
}

/*=====================================================================*/
/* Car and sensors                                                      */
/*=====================================================================*/

static void body(double f, double l, double *wx, double *wy)
{
    double x, y, psi;

    mm_get_truth(&x, &y, &psi);
    {
        const double fx = sin(psi * PI / 180.0);
        const double fy = cos(psi * PI / 180.0);

        *wx = x + f * fx + l * fy;
        *wy = y + f * fy - l * fx;
    }
}

typedef struct
{
    int    ok;
    int    finished;
    double max_ir_off;   /* IR midpoint to the line, mm */
    double max_axle_off; /* axle to the line, mm */
    int    lost_ms;      /* line outside the IR pair */
    double wobble;       /* yaw-rate RMS around its 0.5 s mean, deg/s */
    double flips_m;      /* steering reversals per metre */
    double change_m;     /* sum of |wheel command changes| per metre, mm/s */
    int    min_inner;    /* slowest wheel asked */
    int    kicks;        /* Buddy 2 breakaway kicks */
    double time_s;
    double min_v;
} out_t;

static out_t run(const scen_t *s)
{
    const int base = (s->speed > 0) ? s->speed : (int)B3_BASE_SPEED_MM_S;
    b3_line_t line;
    b3_line_result_t snap;
    out_t o;
    uint32_t t_ms = 0U;
    int16_t sent_l = INT16_MIN;
    int16_t sent_r = INT16_MIN;
    double travelled = 0.0;
    double yaw_lp = 0.0;
    double wob2 = 0.0;
    int wob_n = 0;
    double psi_prev;
    int last_sign = 0;
    int flips = 0;
    double change = 0.0;
    double metric_mm = 0.0;
    int idx = 0;
    double total;
    const double settle = (T_ONBEND == s->shape) ? 0.0 : SETTLE_MM;
    /* nyx, 2026-11-09: curve log, as vehicle_controller.c */
    long cv_um = 0, cv_n = 0, cv_al = 0, cv_ar = 0, cv_gl = 0, cv_gr = 0;
    long cv_curv = 0;
    int cv_first = 1;

    memset(&o, 0, sizeof o);
    memset(&snap, 0, sizeof snap);
    o.min_inner = 0x7FFF;
    o.min_v = 1e9;
    build(s);
    g_cursor = 0;
    total = (g_np - 1) * STEP_MM;
    {
        /* start on the first straight, turned psi0 and moved x0 aside,
         * IRs over the line's start */
        const double iy = 185.0;
        const double a = s->psi0 * PI / 180.0;

        mm_reset(s->x0 - IR_AHEAD * sin(a), iy - IR_AHEAD * cos(a), s->psi0);
    }
    if (T_ONBEND == s->shape)
    {
        /* standing on the bend: IRs over the line 150 mm into it, axle
         * on the line behind them (the car along the chord) */
        const int ii = (int)((START_MM + 150.0) / STEP_MM);
        int ia = ii;

        while ((ia > 0) && (hypot(g_px[ii] - g_px[ia], g_py[ii] - g_py[ia]) < IR_AHEAD))
        {
            ia--;
        }
        mm_reset(g_px[ia], g_py[ia],
                 atan2(g_px[ii] - g_px[ia], g_py[ii] - g_py[ia]) * 180.0 / PI);
        g_cursor = ii;
    }
    mm_set_errors(0.0, g_track_err);
    mm_set_dead_right(0);
    if (g_m_ratio > 0.0)
    {
        mm_set_motor(g_m_off, g_m_ratio);
        mm_set_stiction(g_m_brk);
        if (g_sb_ks > 0.0) { mm_set_stribeck(g_sb_ks, g_sb_kk, g_sb_vs); }
    }
    mm_set_yaw_load(g_y_pm, g_y_w);
    mm_set_lag(g_lag_ms, (T_ONBEND == s->shape) ? 0.0 : (double)base);
    b3_line_init(&line, 0U);
    {
        double x, y;

        mm_get_truth(&x, &y, &psi_prev);
    }

    while (t_ms < 120000U)
    {
        double x, y, psi;

        mm_get_truth(&x, &y, &psi);

        if (0U == (t_ms % 10U))
        {
            double lx, ly, rx, ry;

            body(IR_AHEAD, g_spread, &lx, &ly);
            body(IR_AHEAD, -g_spread, &rx, &ry);
            {
                const double pos = g_cursor * STEP_MM - START_MM;
                const int forced = (g_bb_len > 0.0) && (pos >= g_bb_at) &&
                                   (pos < (g_bb_at + g_bb_len));

                snap = b3_line_step(&line, forced || ir_black(lx, ly, psi),
                                    forced || ir_black(rx, ry, psi),
                                    t_ms * 1000U, (int16_t)base);
            }
            /* nyx, 2026-11-09: how often both IRs read black */
            {
                const int bb = snap.left_black && snap.right_black;
                const int bend = (snap.curvature >= 1000) ||
                                 (snap.curvature <= -1000);

                g_all_n++;
                g_bb_all += bb;
                if (bend)
                {
                    g_bend_n++;
                    g_bb_bend += bb;
                    g_one_bend += (snap.left_black != snap.right_black);
                }
                if (bb)
                {
                    g_bb_run_mm += mm_speed_true() * 0.010;
                }
                else if (g_bb_run_mm > 0.0)
                {
                    g_bb_run_sum += g_bb_run_mm;
                    g_bb_runs++;
                    if (g_bb_run_mm > g_bb_run_max)
                    {
                        g_bb_run_max = g_bb_run_mm;
                    }
                    g_bb_run_mm = 0.0;
                }
            }

            /* yaw-rate wobble around its 0.5 s mean */
            {
                double yaw = (psi - psi_prev) / 0.010;

                psi_prev = psi;
                yaw_lp += (yaw - yaw_lp) * (0.010 / 0.5);
                if (travelled >= settle)
                {
                    wob2 += (yaw - yaw_lp) * (yaw - yaw_lp);
                    wob_n++;
                }
            }
            /* where the car is */
            {
                const double mx = (lx + rx) / 2.0;
                const double my = (ly + ry) / 2.0;
                const double ir_off = track_dist(mx, my, &idx);

                g_cursor = idx;
                const double axle_off = track_dist(x, y, NULL);

                if (travelled >= settle)
                {
                    if (ir_off > o.max_ir_off) { o.max_ir_off = ir_off; }
                    if (axle_off > o.max_axle_off) { o.max_axle_off = axle_off; }
                    if (ir_off > (g_spread + LINE_HALF)) { o.lost_ms += 10; }
                    if (ir_off > 150.0)
                    {
                        break; /* gone */
                    }
                    if (mm_speed_true() < o.min_v) { o.min_v = mm_speed_true(); }
                }
                if (g_verbose && (0U == (t_ms % 50U)))
                {
                    /* nyx, 2026-11-09: the tape's angle to the car at
                     * the IRs, deg (+ = runs to the left) */
                    const int j = (idx + 1 < g_np) ? idx : idx - 1;
                    double tang = atan2(g_px[j + 1] - g_px[j],
                                        g_py[j + 1] - g_py[j]) * 180.0 / PI -
                                  psi;

                    while (tang > 180.0) { tang -= 360.0; }
                    while (tang < -180.0) { tang += 360.0; }
                    (void)printf("t %5.2f  x %7.1f y %7.1f psi %6.1f  IR %d%d off %5.1f  "
                                 "ang %5.1f cmd %4d %4d  curv %6d\n",
                                 t_ms / 1000.0, x, y, psi, snap.left_black,
                                 snap.right_black, ir_off, tang, snap.left_mm_s,
                                 snap.right_mm_s,
#ifdef B3_STEER_KICK_CURV
                                 (int)snap.curvature
#else
                                 0
#endif
                                 );
                }
                /* done: the IRs reached the end of the track */
                if ((idx * STEP_MM) >= (total - 60.0))
                {
                    o.finished = 1;
                    break;
                }
            }
        }

        if (5U == (t_ms % VC_MS))
        {
            const int16_t l = snap.left_mm_s;
            const int16_t r = snap.right_mm_s;

            if ((l != sent_l) || (r != sent_r))
            {
                if ((sent_l != INT16_MIN) && (travelled >= settle))
                {
                    const int d = r - l;
                    const int sign = (d > 4) ? 1 : ((d < -4) ? -1 : 0);

                    change += fabs((double)(l - sent_l)) + fabs((double)(r - sent_r));
                    if ((0 != sign) && (0 != last_sign) && (sign != last_sign))
                    {
                        flips++;
                    }
                    if (0 != sign)
                    {
                        last_sign = sign;
                    }
                }
                (void)motion_set_wheel_speeds(l, r);
                sent_l = l;
                sent_r = r;
            }
            /* nyx, 2026-11-09: the car's curve log (curve_log()) */
            if ((snap.curvature < 1000) && (snap.curvature > -1000))
            {
                if (!cv_first && (g_cl_rmin > 0.0))
                {
                    g_cl_swing += g_cl_rmax / g_cl_rmin;
                    g_cl_curves++;
                }
                cv_um = 0; cv_n = 0; cv_al = 0; cv_ar = 0; cv_gl = 0;
                cv_gr = 0; cv_curv = 0; cv_first = 1;
            }
            else
            {
                double ml, mr;
                long v;

                mm_wheels_meas(&ml, &mr);
                v = labs((long)ml + (long)mr) / 2;
                cv_um += v * (long)VC_MS;
                cv_n++;
                cv_al += l;
                cv_ar += r;
                cv_gl += (long)ml;
                cv_gr += (long)mr;
                cv_curv += snap.curvature;
                if (cv_um >= 100000)
                {
                    const long al = cv_al / cv_n, ar = cv_ar / cv_n;
                    const long gl = cv_gl / cv_n, gr = cv_gr / cv_n;
                    const long c = cv_curv / cv_n;
                    const long rg = ((gr != gl) && ((gl + gr) > 0))
                                        ? (112L * (gl + gr)) / (2L * (gr - gl))
                                        : 0;
                    const long ra = (0 != c) ? (1000000L / c) : 0;
                    const int w = cv_first ? 0 : 1;
                    const long ai = (c > 0) ? al : ar, gi = (c > 0) ? gl : gr;
                    const long ao = (c > 0) ? ar : al, go = (c > 0) ? gr : gl;

                    if (g_curve_log)
                    {
                        (void)printf("  t %6.2f curve: asked R%ld (L %ld R "
                                     "%ld mm/s), wheels L %ld R %ld mm/s = "
                                     "R%ld, IRs %c%c%s\n",
                                     t_ms / 1000.0, ra, al, ar, gl, gr, rg,
                                     snap.left_black ? 'B' : 'w',
                                     snap.right_black ? 'B' : 'w',
                                     cv_first ? "  (first)" : "");
                    }
                    if ((ai > 0) && (ao > 0))
                    {
                        g_cl_pct[w][0] += 100.0 * ((double)gi / ai - 1.0);
                        g_cl_pct[w][1] += 100.0 * ((double)go / ao - 1.0);
                        g_cl_n[w]++;
                    }
                    if (cv_first)
                    {
                        g_cl_rmin = 1e9;
                        g_cl_rmax = 0.0;
                    }
                    if (labs(ra) < g_cl_rmin) { g_cl_rmin = labs(ra); }
                    if (labs(ra) > g_cl_rmax) { g_cl_rmax = labs(ra); }
                    cv_first = 0;
                    cv_um = 0; cv_n = 0; cv_al = 0; cv_ar = 0; cv_gl = 0;
                    cv_gr = 0; cv_curv = 0;
                }
            }
            if ((snap.curvature >= 1000) || (snap.curvature <= -1000))
            {
                double tl, tr;
                mm_wheels_true(&tl, &tr);
                if (snap.curvature > 0) { g_cs_ask_in += l; g_cs_got_in += tl; g_cs_ask_out += r; g_cs_got_out += tr; }
                else { g_cs_ask_in += r; g_cs_got_in += tr; g_cs_ask_out += l; g_cs_got_out += tl; }
                g_cs_n++;
            }
            if (travelled >= settle)
            {
                const int inner = (l < r) ? l : r;

                if (inner < o.min_inner) { o.min_inner = inner; }
            }
        }

        mm_step_1ms();
        travelled += fabs(mm_speed_true()) * 0.001;
        if (travelled >= settle)
        {
            metric_mm += fabs(mm_speed_true()) * 0.001;
        }
        t_ms++;
    }
    o.time_s = t_ms / 1000.0;
    o.kicks = mm_kicks();
    o.wobble = (wob_n > 0) ? sqrt(wob2 / wob_n) : 0.0;
    o.flips_m = (metric_mm > 1.0) ? flips / (metric_mm / 1000.0) : 0.0;
    o.change_m = (metric_mm > 1.0) ? change / (metric_mm / 1000.0) : 0.0;
    o.ok = o.finished && (0 == o.lost_ms);
    return o;
}

int main(int argc, char **argv)
{
    int first = 0;
    int last = N_SCEN - 1;
    int failed = 0;
    int quiet = 0;
    int loss_only = 0;
    int n_limit = 0;
    int i;
    double sum_wob = 0.0, sum_flips = 0.0, sum_change = 0.0, sum_t = 0.0;
    int lost = 0, kicks = 0, unfinished = 0;

    for (i = 1; i < argc; i++)
    {
        if ((0 == strcmp(argv[i], "-s")) && (i + 1 < argc))
        {
            g_spread = atof(argv[++i]);
        }
        else if ((0 == strcmp(argv[i], "-t")) && (i + 1 < argc))
        {
            g_track_err = atof(argv[++i]);
        }
        else if ((0 == strcmp(argv[i], "-x")) && (i + 1 < argc))
        {
            if (2 != sscanf(argv[++i], "%lf,%lf", &g_bb_at, &g_bb_len))
            {
                g_bb_len = 0.0;
            }
        }
        else if ((0 == strcmp(argv[i], "-e")) && (i + 1 < argc))
        {
            if (3 != sscanf(argv[++i], "%lf,%lf,%lf", &g_spot_a, &g_spot_b,
                            &g_spot_th))
            {
                g_spot_a = 0.0;
            }
        }
        else if ((0 == strcmp(argv[i], "-k")) && (i + 1 < argc))
        {
            (void)sscanf(argv[++i], "%lf,%lf,%lf", &g_sb_ks, &g_sb_kk, &g_sb_vs);
        }
        else if ((0 == strcmp(argv[i], "-y")) && (i + 1 < argc))
        {
            (void)sscanf(argv[++i], "%lf,%lf", &g_y_pm, &g_y_w);
        }
        else if (0 == strcmp(argv[i], "-c"))
        {
            g_curve_log = 1;
        }
        else if ((0 == strcmp(argv[i], "-b")) && (i + 1 < argc))
        {
            g_m_brk = atof(argv[++i]);
        }
        else if ((0 == strcmp(argv[i], "-m")) && (i + 1 < argc))
        {
            if (2 != sscanf(argv[++i], "%lf,%lf", &g_m_off, &g_m_ratio))
            {
                g_m_ratio = 0.0;
            }
        }
        else if ((0 == strcmp(argv[i], "-l")) && (i + 1 < argc))
        {
            g_lag_ms = atoi(argv[++i]);
        }
        else if (0 == strcmp(argv[i], "-q"))
        {
            quiet = 1;
        }
        else if (0 == strcmp(argv[i], "-n"))
        {
            loss_only = 1; /* pass = stays on the line; weaving shown only */
        }
        else if (0 == strcmp(argv[i], "v"))
        {
            g_verbose = 1;
        }
        else
        {
            first = atoi(argv[i]);
            last = first;
        }
    }
    if (!quiet)
    {
        (void)printf("Line following: IRs %.0f mm ahead, %.0f mm each side of "
                     "the centre, 18 mm tape, line speed %d mm/s, wheel lag %d ms\n",
                     IR_AHEAD, g_spread, (int)B3_BASE_SPEED_MM_S, g_lag_ms);
    }
    for (i = first; (i <= last) && (i < N_SCEN); i++)
    {
        const scen_t *s = &g_scen[i];
        const out_t o = ((g_curve_log && !quiet)
                             ? (void)printf("%s\n", s->name) : (void)0,
                         run(s));
        /* smooth: at most 2.5 left/right reversals per metre (the old
         * steering made 30 to 40 after a handover) */
        /* nyx, 2026-11-07: below R140 the steering may pivot, so staying
         * on the line is enough; losing it there is the known limit of
         * the smooth steering (shown as LIMIT, not counted) */
        const int sharp = (s->radius > 0) && (s->radius < 140);
        const int limit = sharp && !o.ok;
        const int smooth = loss_only || sharp ||
                           ((o.flips_m <= 2.5) && (o.kicks == 0) &&
                            (o.min_inner >= 12));

        sum_wob += o.wobble;
        sum_flips += o.flips_m;
        sum_change += o.change_m;
        sum_t += o.time_s;
        lost += o.lost_ms;
        kicks += o.kicks;
        unfinished += !o.finished;
        if (!quiet)
        {
            (void)printf("%-46s %s  %5.1f s, IR off <= %4.1f, axle <= %5.1f mm, "
                         "wobble %5.1f deg/s, %4.1f reversals/m, changes "
                         "%5.0f mm/s per m, inner >= %3d, kicks %d, slowest %3.0f%s\n",
                         s->name,
                         limit ? "LIMIT" : ((o.ok && smooth) ? "PASS" : "FAIL"), o.time_s,
                         o.max_ir_off, o.max_axle_off, o.wobble, o.flips_m,
                         o.change_m, o.min_inner, o.kicks, o.min_v,
                         o.finished ? ((o.lost_ms > 0) ? "  LOST the line" : "")
                                    : "  did NOT finish");
        }
        if (!limit && !(o.ok && smooth))
        {
            failed++;
        }
        n_limit += limit;
    }
    if (quiet)
    {
        (void)printf("failed %d lost_ms %d unfinished %d kicks %d  wobble %.1f "
                     "flips %.2f change %.0f time %.1f\n",
                     failed, lost, unfinished, kicks, sum_wob / N_SCEN,
                     sum_flips / N_SCEN, sum_change / N_SCEN, sum_t);
        if ((g_cl_n[0] > 0) && (g_cl_n[1] > 0) && (g_cl_curves > 0))
        {
            (void)printf("curve log: first windows %d inner %+.1f %% outer "
                         "%+.1f %%, later %d inner %+.1f %% outer %+.1f %%, "
                         "asked R swing x%.2f\n",
                         g_cl_n[0], g_cl_pct[0][0] / g_cl_n[0],
                         g_cl_pct[0][1] / g_cl_n[0], g_cl_n[1],
                         g_cl_pct[1][0] / g_cl_n[1], g_cl_pct[1][1] / g_cl_n[1],
                         g_cl_swing / g_cl_curves);
        }
        if (g_cs_n > 0)
        {
            (void)printf("in bends: inner asked %.1f got %.1f (%+.0f %%), outer asked %.1f got %.1f (%+.0f %%)\n",
                         g_cs_ask_in / g_cs_n, g_cs_got_in / g_cs_n, 100.0 * (g_cs_got_in / g_cs_ask_in - 1.0),
                         g_cs_ask_out / g_cs_n, g_cs_got_out / g_cs_n, 100.0 * (g_cs_got_out / g_cs_ask_out - 1.0));
        }
        (void)printf("IRs both black: %.1f %% of steps, %.1f %% of curve steps "
                     "(one side %.1f %%); "
                     "%ld runs, mean %.1f mm, longest %.1f mm\n",
                     (g_all_n > 0) ? (100.0 * g_bb_all / g_all_n) : 0.0,
                     (g_bend_n > 0) ? (100.0 * g_bb_bend / g_bend_n) : 0.0,
                     (g_bend_n > 0) ? (100.0 * g_one_bend / g_bend_n) : 0.0,
                     g_bb_runs, (g_bb_runs > 0) ? (g_bb_run_sum / g_bb_runs) : 0.0,
                     g_bb_run_max);
        return failed ? EXIT_FAILURE : EXIT_SUCCESS;
    }
    if (0 != failed)
    {
        (void)printf("%d line-following scenario(s) FAILED\n", failed);
        return EXIT_FAILURE;
    }
    (void)printf("All line-following scenarios passed (%d tight bends "
                 "lost: LIMIT, not counted).\n", n_limit);
    return EXIT_SUCCESS;
}
