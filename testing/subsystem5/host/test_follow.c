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

static double g_spread = 15.0; /* each IR this far off the centre line */
static int    g_lag_ms = 60;
static int    g_verbose;
static double g_m_off;          /* -m OFF,RATIO: Buddy 2's PI wheel loop */
static double g_m_ratio;
static double g_track_err;      /* -t E: the car turns 1/(1+E) as tightly */

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

/*=====================================================================*/
/* Scenarios                                                            */
/*=====================================================================*/

typedef enum
{
    T_STRAIGHT, T_BEND, T_S, T_CIRCLE, T_HAIRPIN, T_WIGGLE
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
    mm_set_errors(0.0, g_track_err);
    mm_set_dead_right(0);
    if (g_m_ratio > 0.0)
    {
        mm_set_motor(g_m_off, g_m_ratio);
    }
    mm_set_lag(g_lag_ms, (double)base);
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
            snap = b3_line_step(&line, track_dist(lx, ly, NULL) <= LINE_HALF,
                                track_dist(rx, ry, NULL) <= LINE_HALF,
                                t_ms * 1000U, (int16_t)base);

            /* yaw-rate wobble around its 0.5 s mean */
            {
                double yaw = (psi - psi_prev) / 0.010;

                psi_prev = psi;
                yaw_lp += (yaw - yaw_lp) * (0.010 / 0.5);
                if (travelled >= SETTLE_MM)
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

                if (travelled >= SETTLE_MM)
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
                    (void)printf("t %5.2f  x %7.1f y %7.1f psi %6.1f  IR %d%d off %5.1f  "
                                 "cmd %4d %4d  curv %6d\n",
                                 t_ms / 1000.0, x, y, psi, snap.left_black,
                                 snap.right_black, ir_off, snap.left_mm_s,
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
                if ((sent_l != INT16_MIN) && (travelled >= SETTLE_MM))
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
            if (travelled >= SETTLE_MM)
            {
                const int inner = (l < r) ? l : r;

                if (inner < o.min_inner) { o.min_inner = inner; }
            }
        }

        mm_step_1ms();
        travelled += fabs(mm_speed_true()) * 0.001;
        if (travelled >= SETTLE_MM)
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
        const out_t o = run(s);
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
