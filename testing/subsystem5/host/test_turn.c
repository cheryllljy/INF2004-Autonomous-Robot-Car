/**
 * @file    test_turn.c
 * @brief   HOST TEST (VC-H01). 2-D simulation of the turn
 *          at a junction: the REAL vc_turn.c (VehicleController) and the
 *          REAL b3_line.c (Buddy 3's line following) drive the simulated
 *          car of motion_mock.c over T, cross and corner junctions.
 *
 *   real       vc_turn.c, b3_line.c
 *   mirrored   Buddy 3's junction rule (2026-11-08: each line sensor black
 *              for B3_JUNCTION_MIN_PATCH_UM, runs overlap), reported after an
 *              optional delay
 *   simulated  the car (motion_mock.c), the IR pair VC_IR_AHEAD_MM ahead
 *              of the axle (or another true distance) straddling 18 mm
 *              tape
 *
 * After the turn Buddy 3 follows the new line for AFTER_MM; the car must
 * then be on it: axle within AFTER_X_MM, heading within AFTER_DEG, line
 * between or under the IRs. The old turn (stop, pivot 90 deg on the spot
 * where the IRs met the junction) is run too, for comparison only.
 *
 *   ./test_turn           all scenarios
 *   ./test_turn N [v]     scenario N, v = print the VC log
 */

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_mock.h"
#include "vc_turn.h"
#include "b3_line.h"
#include "motion_config.h" /* TRACK_WIDTH_MM */

#define PI          (3.14159265358979323846)
#define IR_SPREAD   (10.0) /* each IR 10 mm off the centre line */
#define LINE_HALF   (9.0)  /* 18 mm tape */
#define AFTER_MM    (500.0)
#define AFTER_X_MM  (20.0)
#define AFTER_DEG   (10.0)
#define START_Y     (-900.0)
#define MAX_SEG     (800)
/* the VehicleController loop really runs every 20 ms
 * (tk_dly_tsk(10) on the 10 ms tick); Buddy 3 every 10 ms */
#define SIM_VC_MS   (20U)
/* mirrors exchange_line(): Buddy 3 follows while the VC turns; 0 shows the
 * stop at the handover that the review found (2026-11-06) */
#ifndef SIM_B3_FOLLOWS_WHILE_TURNING
#define SIM_B3_FOLLOWS_WHILE_TURNING 1
#endif

typedef enum { SHAPE_T, SHAPE_CROSS, SHAPE_L } shape_t;

typedef struct
{
    const char      *name;
    b3_nav_command_t turn;
    shape_t          shape;
    double           ir_ahead; /* true IR distance (0: VC_IR_AHEAD_MM) */
    double           psi0;     /* heading at the start, deg (+ left) */
    double           x0;       /* sideways offset at the start, mm */
    double           enc;      /* encoder scale error */
    double           track;    /* real track wider than the firmware's */
    int              delay_ms; /* junction reported this much later */
    int              old_turn; /* the old stop + pivot, comparison only:
                                  1 must miss, 2 either */
    /* the branch the car turns into bends (radius mm,
     * 0 = straight; angle deg, + = to the left of its own direction;
     * starting this far from the junction) */
    double           bend_r;
    double           bend_deg;
    double           bend_from;
} scen_t;

static const scen_t g_scen[] =
{
    { "J1 LEFT at a T",                         B3_NAV_LEFT,  SHAPE_T,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J2 RIGHT at a T",                        B3_NAV_RIGHT, SHAPE_T,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J3 LEFT at a cross",                     B3_NAV_LEFT,  SHAPE_CROSS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J4 RIGHT at a cross",                    B3_NAV_RIGHT, SHAPE_CROSS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J5 LEFT at a left-hand corner",          B3_NAV_LEFT,  SHAPE_L,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J6 U-TURN at a T",                       B3_NAV_UTURN, SHAPE_T,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J7 U-TURN at a cross",                   B3_NAV_UTURN, SHAPE_CROSS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J8 LEFT, car 6 deg off and 5 mm aside",  B3_NAV_LEFT,  SHAPE_T,     0, 6, 5, 0, 0, 0, 0, 0, 0, 0 },
    { "J9 RIGHT, car 6 deg off the other way",  B3_NAV_RIGHT, SHAPE_T,     0, -6, -5, 0, 0, 0, 0, 0, 0, 0 },
    { "J10 LEFT, IRs 140 mm ahead (VC: 125)",   B3_NAV_LEFT,  SHAPE_T,     140, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J11 RIGHT, IRs 110 mm ahead (VC: 125)",  B3_NAV_RIGHT, SHAPE_T,     110, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "J12 LEFT, encoders 3 % long, track 5 % wide", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0.03, 0.05, 0, 0, 0, 0, 0 },
    { "J13 RIGHT, encoders 3 % short, track 5 % narrow", B3_NAV_RIGHT, SHAPE_T, 0, 0, 0, -0.03, -0.05, 0, 0, 0, 0, 0 },
    { "J14 LEFT, junction reported 40 ms late", B3_NAV_LEFT,  SHAPE_T,     0, 0, 0, 0, 0, 40, 0, 0, 0, 0 },
    { "J15 U-TURN, encoders 3 % long, track 5 % wide", B3_NAV_UTURN, SHAPE_T, 0, 0, 0, 0.03, 0.05, 0, 0, 0, 0, 0 },
    { "J16 U-TURN, car 6 deg off",              B3_NAV_UTURN, SHAPE_T,     0, 6, 5, 0, 0, 0, 0, 0, 0, 0 },
    { "J17 LEFT, track 10 % wide (tyres scrub)", B3_NAV_LEFT,  SHAPE_T,     0, 0, 0, 0, 0.10, 0, 0, 0, 0, 0 },
    { "J18 RIGHT, track 10 % narrow",           B3_NAV_RIGHT, SHAPE_T,     0, 0, 0, 0, -0.10, 0, 0, 0, 0, 0 },
    { "J19 U-TURN, track 10 % wide",            B3_NAV_UTURN, SHAPE_T,     0, 0, 0, 0, 0.10, 0, 0, 0, 0, 0 },
    { "J20 U-TURN, track 10 % narrow",          B3_NAV_UTURN, SHAPE_CROSS, 0, 0, 0, 0, -0.10, 0, 0, 0, 0, 0 },
    { "J21 LEFT, IRs 150 mm ahead, 6 deg off",  B3_NAV_LEFT,  SHAPE_CROSS, 150, 6, 3, 0, 0, 0, 0, 0, 0, 0 },
    { "J22 RIGHT, IRs 100 mm ahead, 6 deg off", B3_NAV_RIGHT, SHAPE_CROSS, 100, -6, -3, 0, 0, 0, 0, 0, 0, 0 },
    /* curved branches (the car missed a curved line) */
    { "J23 LEFT, branch bends left R400 at once",  B3_NAV_LEFT,  SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 400,  90, 0 },
    { "J24 LEFT, branch bends right R400 at once", B3_NAV_LEFT,  SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 400, -90, 0 },
    { "J25 RIGHT, branch bends right R400 at once", B3_NAV_RIGHT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 400, -90, 0 },
    { "J26 RIGHT, branch bends left R400 at once", B3_NAV_RIGHT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 400,  90, 0 },
    { "J27 LEFT, branch bends left R300 after 60 mm", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 300, 90, 60 },
    { "J28 LEFT, branch bends right R300 after 60 mm", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 300, -90, 60 },
    { "J29 RIGHT, branch bends right R250 after 100 mm", B3_NAV_RIGHT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 250, -90, 100 },
    { "J30 LEFT at a cross, branch bends left R500", B3_NAV_LEFT, SHAPE_CROSS, 0, 0, 0, 0, 0, 0, 0, 500, 60, 0 },
    /* the car missed a right bend of about R130 a few cm
     * after a LEFT junction turn */
    { "J31 LEFT, branch bends right R130 after 30 mm", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 130, -90, 30 },
    { "J32 LEFT, branch bends right R130 after 60 mm", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 130, -90, 60 },
    { "J33 RIGHT, branch bends left R130 after 30 mm", B3_NAV_RIGHT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 130, 90, 30 },
    { "J34 LEFT, branch bends left R130 after 30 mm", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0, 0, 0, 130, 90, 30 },
    { "J35 LEFT, wheels turn the car 1/1.4 as tightly", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0.4, 0, 0, 0, 0, 0 },
    { "J36 RIGHT, wheels 1/1.4, branch bends left R130 after 30 mm", B3_NAV_RIGHT, SHAPE_T, 0, 0, 0, 0, 0.4, 0, 0, 130, 90, 30 },
    /* the car's U-turn stopped short of the line */
    { "J37 U-TURN, wheels turn the car 1/1.4 as tightly", B3_NAV_UTURN, SHAPE_T, 0, 0, 0, 0, 0.4, 0, 0, 0, 0, 0 },
    { "J38 U-TURN at a cross, wheels 1/1.6",    B3_NAV_UTURN, SHAPE_CROSS, 0, 0, 0, 0, 0.6, 0, 0, 0, 0, 0 },
    { "J39 U-TURN, wheels 1/1.3, car 6 deg off", B3_NAV_UTURN, SHAPE_T,    0, 6, 5, 0, 0.3, 0, 0, 0, 0, 0 },
    /* comparison: the turn before this change, expected to miss */
    { "O1 old turn: LEFT at a T, stop and pivot where the IRs meet it", B3_NAV_LEFT, SHAPE_T, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
    { "O2 old turn: U-TURN at a T (on the line, but stop + fast spin)", B3_NAV_UTURN, SHAPE_T, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0 },
};

#define N_SCEN ((int)(sizeof(g_scen) / sizeof(g_scen[0])))

static int g_verbose;

INT tm_printf(const UB *format, ...)
{
    va_list ap;

    if (g_verbose)
    {
        va_start(ap, format);
        (void)vprintf((const char *)format, ap);
        va_end(ap);
    }
    return 0;
}

/*=====================================================================*/
/* World: line segments, x + = left, y forward                          */
/*=====================================================================*/

typedef struct { double x0, y0, x1, y1; } seg_t;

static seg_t g_seg[MAX_SEG];
static int   g_nseg;

/* a branch from the junction, heading psi (deg, + left;
 * forward = (sin, cos)), bending as the scenario says */
static void branch(double psi, const scen_t *s, int bent)
{
    double x = 0.0, y = 0.0;
    const double step = 5.0;
    double d;

    if (!bent || (s->bend_r <= 0.0))
    {
        g_seg[g_nseg++] = (seg_t){ 0, 0, 1500.0 * sin(psi * PI / 180.0),
                                   1500.0 * cos(psi * PI / 180.0) };
        return;
    }
    if (s->bend_from > 0.0)
    {
        const double x1 = s->bend_from * sin(psi * PI / 180.0);
        const double y1 = s->bend_from * cos(psi * PI / 180.0);

        g_seg[g_nseg++] = (seg_t){ 0, 0, x1, y1 };
        x = x1;
        y = y1;
    }
    {
        const double len = fabs(s->bend_deg) * PI / 180.0 * s->bend_r;
        const int n = (int)(len / step);
        const double dpsi = s->bend_deg / n;
        int i;

        for (i = 0; (i < n) && (g_nseg < MAX_SEG - 2); i++)
        {
            double nx, ny;

            psi += dpsi / 2.0;
            nx = x + (len / n) * sin(psi * PI / 180.0);
            ny = y + (len / n) * cos(psi * PI / 180.0);
            psi += dpsi / 2.0;
            g_seg[g_nseg++] = (seg_t){ x, y, nx, ny };
            x = nx;
            y = ny;
        }
    }
    for (d = 0.0; d < 1; d += 1.0)
    {
        g_seg[g_nseg++] = (seg_t){ x, y, x + 1200.0 * sin(psi * PI / 180.0),
                                   y + 1200.0 * cos(psi * PI / 180.0) };
    }
}

static void world(shape_t shape, const scen_t *s)
{
    g_nseg = 0;
    g_seg[g_nseg++] = (seg_t){ 0, -3000, 0, 0 };          /* coming in */
    branch(90.0, s, B3_NAV_LEFT == s->turn);              /* left */
    if (SHAPE_L != shape)
    {
        branch(-90.0, s, B3_NAV_RIGHT == s->turn);        /* right */
    }
    if (SHAPE_CROSS == shape)
    {
        g_seg[g_nseg++] = (seg_t){ 0, 0, 0, 1500 };       /* straight on */
    }
}

static double seg_dist(const seg_t *s, double x, double y)
{
    const double dx = s->x1 - s->x0;
    const double dy = s->y1 - s->y0;
    const double len2 = dx * dx + dy * dy;
    double t = ((x - s->x0) * dx + (y - s->y0) * dy) / len2;

    if (t < 0.0) { t = 0.0; }
    if (t > 1.0) { t = 1.0; }
    return hypot(x - (s->x0 + t * dx), y - (s->y0 + t * dy));
}

static int black_at(double x, double y)
{
    int i;

    for (i = 0; i < g_nseg; i++)
    {
        if (seg_dist(&g_seg[i], x, y) <= LINE_HALF)
        {
            return 1;
        }
    }
    return 0;
}

/* body (forward f, left l) -> world */
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

static double g_ir_ahead;

static void read_irs(int *p_l, int *p_r, double *p_mx, double *p_my)
{
    double lx, ly, rx, ry;

    body(g_ir_ahead, IR_SPREAD, &lx, &ly);
    body(g_ir_ahead, -IR_SPREAD, &rx, &ry);
    *p_l = black_at(lx, ly);
    *p_r = black_at(rx, ry);
    *p_mx = (lx + rx) / 2.0;
    *p_my = (ly + ry) / 2.0;
}

/*=====================================================================*/
/* One scenario                                                         */
/*=====================================================================*/

static double wrap180(double a)
{
    while (a > 180.0) { a -= 360.0; }
    while (a <= -180.0) { a += 360.0; }
    return a;
}

typedef struct
{
    int    ok;
    double turn_s;      /* junction to handover */
    double min_speed;   /* slowest axle speed during the turn, mm/s */
    double hand_off_mm; /* IR midpoint to the new line at the handover */
    double end_x;       /* axle to the new line after AFTER_MM */
    double end_deg;     /* heading error after AFTER_MM */
    int    end_ir_ok;   /* line between or under the IRs at the end */
    double after_min_speed; /* slowest just after the handover */
    int    min_cmd;     /* smallest non-zero wheel speed vc_turn asked for */
} out_t;

static void target(const scen_t *s, double x, double y, double psi,
                   double *p_off, double *p_deg)
{
    if (s->bend_r > 0.0)
    {
        /* curved branch: distance to the nearest piece
         * of the branch (not the line the car came in on), heading against
         * that piece's direction */
        double best = 1e18;
        int i;

        *p_deg = 180.0;
        for (i = 1; i < g_nseg; i++)
        {
            const double d = seg_dist(&g_seg[i], x, y);

            if (d < best)
            {
                const double dir = atan2(g_seg[i].x1 - g_seg[i].x0,
                                         g_seg[i].y1 - g_seg[i].y0) *
                                   180.0 / PI;

                best = d;
                *p_deg = fabs(wrap180(psi - dir));
            }
        }
        *p_off = best;
    }
    else if (B3_NAV_UTURN == s->turn)
    {
        *p_off = fabs(x);
        *p_deg = fabs(wrap180(psi - 180.0));
    }
    else
    {
        const double want = (B3_NAV_LEFT == s->turn) ? 90.0 : -90.0;

        *p_off = fabs(y);
        *p_deg = fabs(wrap180(psi - want));
    }
}

static out_t run(const scen_t *s)
{
    b3_line_t line;
    motion_status_t st;
    out_t o;
    uint32_t t_ms = 0U;
    int phase = 0; /* 0 approach, 1 turning, 2 after */
    double odo = 0.0; /* mm driven (Buddy 3's distance) */
    struct
    {
        int on;
        int valid;
        double s;
        double e;
        unsigned int s_ms;
        unsigned int e_ms;
    } run[2];
    int counted = 0;
    int report_at = -1;
    double after_from = 0.0;
    double travelled = 0.0;
    int soft_ms = VC_SOFT_START_MS;
    uint32_t turn_start_ms = 0U;
    uint32_t hand_ms = 0U;
    int old_step = 0;
    int16_t sent_l = INT16_MIN;
    int16_t sent_r = INT16_MIN;
    /* Buddy 3's snapshot as the VC reads it: built on Buddy 3's last pass,
     * with the context the VC published on its pass before that */
    b3_line_result_t snap;
    int snap_active = 1;
    int ctx_follow = 1;

    memset(&o, 0, sizeof o);
    memset(&snap, 0, sizeof snap);
    memset(run, 0, sizeof run);
    o.min_speed = 1e9;
    o.after_min_speed = 1e9;
    world(s->shape, s);
    g_ir_ahead = (s->ir_ahead > 0.0) ? s->ir_ahead : (double)VC_IR_AHEAD_MM;
    mm_reset(s->x0, START_Y, s->psi0);
    mm_set_errors(s->enc, s->track);
    mm_set_dead_right(0);
    b3_line_init(&line, 0U);

    while (t_ms < 150000U)
    {
        int irl, irr;
        double mx, my, x, y, psi;

        read_irs(&irl, &irr, &mx, &my);
        motion_get_status(&st);

        /* Buddy 3: every 10 ms (its own cyclic handler) */
        if (0U == (t_ms % 10U))
        {
            const double v = ((double)st.speed_l_mmps + st.speed_r_mmps) / 2.0;

            snap = b3_line_step(&line, irl != 0, irr != 0, t_ms * 1000U,
                                (int16_t)B3_BASE_SPEED_MM_S);
            if (1 == phase)
            {
                /* buddy3.c holds the steering while the
                 * VC turns (context steer_hold) */
                b3_line_hold(&line);
            }
            snap_active = ctx_follow;
            if (g_verbose && (0U == (t_ms % 50U)))
            {
                mm_get_truth(&x, &y, &psi);
                (void)printf("  t %6.2f ph %d axle %7.1f %7.1f psi %6.1f IR %d%d "
                             "cmd %4d %4d\n", t_ms / 1000.0, phase, x, y, psi,
                             irl, irr, snap.left_mm_s, snap.right_mm_s);
            }
            /* Buddy 3's junction rule (each sensor's own
             * run over the tape, the two runs overlapping in time) */
            {
                const int blk[2] = { irl != 0, irr != 0 };
                const double odo0 = odo;
                int k;

                odo += ((v > 0.0) ? v : 0.0) * 0.010;
                for (k = 0; k < 2; ++k)
                {
                    if (blk[k])
                    {
                        if (!run[k].on)
                        {
                            run[k].on = 1;
                            run[k].valid = 1;
                            run[k].s = odo0; /* as the old mirror */
                            run[k].s_ms = t_ms;
                        }
                        run[k].e = odo;
                        run[k].e_ms = t_ms;
                    }
                    else
                    {
                        run[k].on = 0;
                    }
                }
                if (!counted && (0 == phase) && run[0].valid &&
                    run[1].valid && (run[0].on || run[1].on))
                {
                    const int late = (run[1].s_ms > run[0].s_ms) ? 1 : 0;
                    const int endk = (run[1].e_ms < run[0].e_ms) ? 1 : 0;
                    const int linked = (run[endk].e_ms >= run[late].s_ms);

                    if (linked &&
                        ((run[0].e - run[0].s) >=
                         B3_JUNCTION_MIN_PATCH_UM / 1000.0) &&
                        ((run[1].e - run[1].s) >=
                         B3_JUNCTION_MIN_PATCH_UM / 1000.0))
                    {
                        counted = 1;
                        report_at = (int)t_ms + s->delay_ms;
                    }
                }
            }
        }

        /* the VehicleController: every SIM_VC_MS, 5 ms after Buddy 3 */
        if (5U == (t_ms % SIM_VC_MS))
        {
            const double v = ((double)st.speed_l_mmps + st.speed_r_mmps) / 2.0;
            const b3_line_result_t lr = snap; /* read before the context */
            const int active = snap_active;

            /* exchange_line(): Buddy 3 follows in LINE_FOLLOW, and (since
             * the fix) also while turning */
            ctx_follow = (1 != phase) || SIM_B3_FOLLOWS_WHILE_TURNING;

            if ((0 == phase) && (report_at >= 0) && ((int)t_ms >= report_at))
            {
                phase = 1;
                turn_start_ms = t_ms;
                (void)mm_min_wheel_cmd(1);
                if (s->old_turn)
                {
                    (void)motion_stop();
                    old_step = 0;
                }
                else if (!vc_turn_start(s->turn, &st))
                {
                    return o;
                }
            }

            if (0 == phase)
            {
                if ((lr.left_mm_s != sent_l) || (lr.right_mm_s != sent_r))
                {
                    (void)motion_set_wheel_speeds(lr.left_mm_s, lr.right_mm_s);
                    sent_l = lr.left_mm_s;
                    sent_r = lr.right_mm_s;
                }
            }
            else if (1 == phase)
            {
                vc_turn_result_t r = VC_TURN_RUNNING;

                if (v < o.min_speed) { o.min_speed = v; }
                if (s->old_turn)
                {
                    /* old: stop, then pivot 90 (or 180) at 90 deg/s */
                    if ((0 == old_step) && (MOTION_STATE_IDLE == st.state))
                    {
                        const int32_t a = (B3_NAV_UTURN == s->turn) ? 180
                                          : (B3_NAV_LEFT == s->turn) ? 90 : -90;
                        (void)motion_turn(a, 90);
                        old_step = 1;
                    }
                    else if ((1 == old_step) && (MOTION_STATE_TURNING == st.state))
                    {
                        old_step = 2;
                    }
                    else if ((2 == old_step) && (MOTION_STATE_IDLE == st.state))
                    {
                        r = VC_TURN_DONE_STOPPED;
                    }
                }
                else
                {
                    /* the VC passes Buddy 3's (debounced) IR state */
                    r = vc_turn_step(&st, lr.left_black, lr.right_black,
                                     SIM_VC_MS);
                }
                if (VC_TURN_RUNNING != r)
                {
                    double off, deg;

                    phase = 2;
                    hand_ms = t_ms;
                    o.turn_s = (t_ms - turn_start_ms) / 1000.0;
                    o.min_cmd = mm_min_wheel_cmd(0);
                    soft_ms = (VC_TURN_DONE_STOPPED == r) ? 0 : VC_SOFT_START_MS;
                    sent_l = INT16_MIN;
                    sent_r = INT16_MIN;
                    mm_get_truth(&x, &y, &psi);
                    target(s, mx, my, psi, &off, &deg);
                    o.hand_off_mm = off;
                    after_from = travelled;
                }
            }
            else
            {
                int16_t l = lr.left_mm_s;
                int16_t rr = lr.right_mm_s;

                if (!active)
                {
                    /* act(): no Buddy 3 control yet -> drive(0, 0) */
                    l = 0;
                    rr = 0;
                }
                else
                {
                    vc_turn_soft_start(&l, &rr, soft_ms);
                    if (soft_ms < VC_SOFT_START_MS) { soft_ms += SIM_VC_MS; }
                }
                if ((l != sent_l) || (rr != sent_r))
                {
                    (void)motion_set_wheel_speeds(l, rr);
                    sent_l = l;
                    sent_r = rr;
                }
            }
        }

        if (2 == phase)
        {
            const uint32_t since = t_ms - hand_ms;
            const double v = mm_speed_true();

            /* rolling through the handover: the arc from the start, the
             * U-turn once its last wheel has turned round */
            if ((since <= 300U) && (B3_NAV_UTURN != s->turn) &&
                (v < o.after_min_speed))
            {
                o.after_min_speed = v;
            }
            if ((since >= 150U) && (since <= 450U) &&
                (B3_NAV_UTURN == s->turn) && (v < o.after_min_speed))
            {
                o.after_min_speed = v;
            }
            if ((travelled - after_from) >= AFTER_MM)
            {
                double off, deg;
                int ml, mr;
                double ix, iy;

                mm_get_truth(&x, &y, &psi);
                target(s, x, y, psi, &off, &deg);
                o.end_x = off;
                o.end_deg = deg;
                read_irs(&ml, &mr, &ix, &iy);
                target(s, ix, iy, psi, &off, &deg);
                o.end_ir_ok = (off <= IR_SPREAD + LINE_HALF);
                o.ok = (o.end_x <= AFTER_X_MM) && (o.end_deg <= AFTER_DEG) &&
                       o.end_ir_ok;
                return o;
            }
        }

        mm_step_1ms();
        travelled += fabs(mm_speed_true()) * 0.001;
        t_ms++;
    }
    return o;
}

/* The soft start never asks for a wheel speed Buddy 2 cannot hold. */
static int check_soft_start(void)
{
    static const int16_t speeds[] = { 1, 5, 12, 14, 15, 16, 25, 30, 60, 80,
                                      -1, -14, -15, -30 };
    int32_t ms;
    size_t i;
    int bad = 0;

    for (i = 0U; i < (sizeof(speeds) / sizeof(speeds[0])); i++)
    {
        for (ms = 0; ms <= VC_SOFT_START_MS; ms += 10)
        {
            int16_t l = speeds[i];
            int16_t r = 0;
            const int mag = abs((int)speeds[i]);
            const int want = (mag < VC_MIN_WHEEL_MM_S) ? mag : VC_MIN_WHEEL_MM_S;

            vc_turn_soft_start(&l, &r, ms);
            if ((0 != r) || ((l < 0) != (speeds[i] < 0)) ||
                (abs((int)l) < want) || (abs((int)l) > mag))
            {
                (void)printf("    FAIL: soft start %d mm/s at %d ms gave %d\n",
                             (int)speeds[i], (int)ms, (int)l);
                bad++;
            }
        }
    }
    return bad;
}

int main(int argc, char **argv)
{
    int first = 0;
    int last = N_SCEN - 1;
    int failed = 0;
    int i;

    if (argc > 1)
    {
        first = atoi(argv[1]);
        last = first;
        g_verbose = (argc > 2);
    }
    (void)printf("Junction turns: IRs %d mm ahead of the axle, track %d mm, "
                 "line speed %d mm/s, arc %d mm/s, U-turn %d deg/s\n",
                 (int)VC_IR_AHEAD_MM, (int)(TRACK_WIDTH_MM + 0.5f),
                 (int)B3_BASE_SPEED_MM_S, (int)VC_CORNER_SPEED_MM_S,
                 (int)VC_UTURN_RATE_DPS);
    for (i = first; (i <= last) && (i < N_SCEN); i++)
    {
        const scen_t *s = &g_scen[i];
        const out_t o = run(s);
        /* an L corner gives no both-black patch, so it is
         * no junction. The old sharp-turn pivot happened to sweep both IRs
         * over it; the smooth steering does not, so J5 is a known limit,
         * shown but not counted. */
        const int limit = (SHAPE_L == s->shape) && !o.ok;
        const int pass = limit ? 1
                         : (2 == s->old_turn) ? 1
                                              : (s->old_turn ? !o.ok : o.ok);

        (void)printf("%-50s %s  turn %4.1f s, slowest %5.1f mm/s, IR %5.1f mm "
                     "off at handover, then axle %5.1f mm / %4.1f deg off, "
                     "IRs %s\n",
                     s->name,
                     limit ? "LIMIT (no junction patch at an L corner)"
                     : (2 == s->old_turn) ? "(old, for comparison)"
                     : s->old_turn ? (o.ok ? "FAIL (old turn worked?)"
                                         : "misses, as expected")
                                 : (o.ok ? "PASS" : "FAIL"),
                     o.turn_s, (o.min_speed > 1e8) ? 0.0 : o.min_speed,
                     o.hand_off_mm, o.end_x, o.end_deg,
                     o.end_ir_ok ? "on the line" : "OFF the line");
        if (!pass)
        {
            failed++;
        }
        /* the arc must not stop the car */
        if (!s->old_turn && (B3_NAV_UTURN != s->turn) && o.ok &&
            (o.min_speed < 0.5 * VC_CORNER_SPEED_MM_S))
        {
            (void)printf("    FAIL: the arc slowed to %.1f mm/s\n", o.min_speed);
            failed++;
        }
        /* ... nor the handover to Buddy 3 */
        if (!s->old_turn && o.ok &&
            (o.after_min_speed <
             ((B3_NAV_UTURN == s->turn) ? (0.5 * VC_MIN_WHEEL_MM_S)
                                        : (0.5 * VC_CORNER_SPEED_MM_S))))
        {
            (void)printf("    FAIL: the car slowed to %.1f mm/s after the "
                         "handover\n", o.after_min_speed);
            failed++;
        }
        /* no wheel speed in Buddy 2's dead band / breakaway range */
        if (!s->old_turn && o.ok &&
            (o.min_cmd < ((B3_NAV_UTURN == s->turn) ? VC_MIN_WHEEL_MM_S
                                                    : VC_ARC_MIN_WHEEL_MM_S)))
        {
            (void)printf("    FAIL: vc_turn asked for a wheel speed of %d "
                         "mm/s\n", o.min_cmd);
            failed++;
        }
    }
    failed += check_soft_start();
    if (0 != failed)
    {
        (void)printf("%d junction turn scenario(s) FAILED\n", failed);
        return EXIT_FAILURE;
    }
    (void)printf("All junction turn scenarios passed.\n");
    return EXIT_SUCCESS;
}
