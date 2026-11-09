/**
 * @file    test_search.c
 * @brief   HOST TEST (S5-H22, nyx 2026-11-08). The line search
 *          (robot.ps1 search) on the simulated car: the car starts off
 *          the line, s5_search_begin(), s5_avoid_step() until it hands
 *          over, then Buddy 3 follows the line.
 *
 *   real       s5_avoid.c (s5_search_begin and the pick-up), s5_sensor.c,
 *              s5_scan.c, s5_path.c, s5_odom.c, s5_math.c; b3_line.c
 *   simulated  the world, the sensor and the IR pair (s5_sim.c), Buddy 2
 *              (motion_mock.c), the VehicleController's OBSTACLE state
 *              (act(), drive()), the S5 task every 10 ms, Buddy 3 every
 *              10 ms, the VehicleController every 20 ms
 *
 * Our line runs along X (the sim's straight track, 6 m). A scenario puts
 * the car off it (or on it), turned so many degrees, and says which way
 * the car must end up facing along the line (+X, -X or either). Passes
 * when the search hands over, and from AFTER_FROM_MM to AFTER_MM of
 * Buddy 3's line following the IR pair stays within AFTER_OFF_MM of the
 * line and the car faces that way within AFTER_DEG. A "none" scenario
 * must stop with the fault instead.
 *
 *   ./test_search            all scenarios, 5 seeds each
 *   ./test_search ID [v]     one scenario; v = log, vv = trace
 *   ./test_search -s N       N seeds per scenario
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "b3_line.h"
#include "motion_control.h"
#include "motion_mock.h"
#include "s5_avoid.h"
#include "s5_config.h"
#include "s5_hw.h"
#include "s5_sensor.h"
#include "s5_sim.h"

#define PI            (3.14159265358979323846)
#define VC_MS         (20U)
#define REFRESH       (100U) /* VC_SPEED_REFRESH_MS */
#define AFTER_FROM_MM (300.0)
#define AFTER_MM      (700.0)
#define AFTER_OFF_MM  (15.0)
#define AFTER_DEG     (12.0)
#define LIMIT_MS      (90000U)

typedef struct
{
    const char *id;
    const char *name;
    double x;       /* car's axle, track frame */
    double y;
    double th;      /* heading, deg (0 = along +X) */
    int want;       /* end facing +1 (+X), -1 (-X), 0 either; 9 = fault */
    double enc_err; /* encoders read long by this fraction */
    double lr_err;  /* left long by this / 2, right short by this / 2 */
    double bend_r;  /* > 0: our line bends left by bend_deg from bend_x */
    double bend_deg;
    double bend_x;
    double box_y;   /* != 0: a 21x16 cm box centred here (x = car's x) */
    double box_gone_ms; /* ... taken away this long after the start */
    double track_err;   /* the real track is wider: wheels turn the car
                         * 1 / (1 + this) as far as asked */
    double ir_half;     /* real IRs this far off the centre (0: 12.5) */
} scen_t;

static const scen_t g_scen[] =
{
    { "Q1", "line straight across, 40 cm ahead: turns LEFT (S5_SEARCH_TURN)",
      3000, -400, 90, -1 },
    { "Q2", "straight across from the other side: LEFT again",
      3000, 400, -90, 1 },
    { "Q3", "straight across, 1.2 m ahead", 3000, -1200, 90, -1 },
    { "Q4", "85 deg (within 8 of square): LEFT", 3000, -400, 85, -1 },
    { "Q5", "95 deg (within 8 of square): LEFT", 3000, -400, 95, -1 },
    { "Q6", "80 deg: the shorter way, RIGHT", 3000, -400, 80, 1 },
    { "Q7", "100 deg: the shorter way, LEFT", 3000, -400, 100, -1 },
    { "Q8", "60 deg", 3000, -400, 60, 1 },
    { "Q9", "120 deg", 3000, -400, 120, -1 },
    { "Q10", "45 deg", 3000, -400, 45, 1 },
    { "Q11", "135 deg", 3000, -400, 135, -1 },
    { "Q12", "30 deg", 3000, -300, 30, 1 },
    { "Q13", "35 deg", 3000, -300, 35, 1 },
    { "Q14", "25 deg", 2500, -300, 25, 1 },
    { "Q15", "20 deg (Buddy 3 takes it from one IR)", 2500, -300, 20, 1 },
    { "Q16", "15 deg (Buddy 3 takes it)", 2500, -200, 15, 1 },
    { "Q17", "10 deg (Buddy 3 takes it)", 2500, -150, 10, 1 },
    { "Q18", "-30 deg, coming from the left of the line", 3000, 300, -30, 1 },
    { "Q19", "started on the line, 3 deg off: Buddy 3 takes it",
      500, 0, 3, 1 },
    { "Q20", "started on the line, 2 deg off the other way", 500, 0, -2, 1 },
    { "Q21", "started on the line, a R400 left bend 60 deg 40 cm ahead",
      3000, 0, 0, 0, 0.0, 0.0, 400.0, 60.0, 3400.0 },
    { "Q22", "started on the line in that bend, IR_L on the tape",
      3000, 0, 0, 0, 0.0, 0.0, 400.0, 60.0, 3000.0 },
    { "Q23", "45 deg, encoders 2 % long and L/R 0.5 % apart",
      3000, -400, 45, 1, 0.02, 0.005 },
    { "Q24", "straight across, encoders 2 % long and L/R 0.5 % apart",
      3000, -400, 90, -1, 0.02, 0.005 },
    { "Q25", "a box in the way, taken away after 4 s, then the line",
      3000, -1100, 90, -1, 0.0, 0.0, 0.0, 0.0, 0.0, -500.0, 4000.0 },
    { "Q26", "straight across, real track 10 % wider (turns 10 % less)",
      3000, -400, 90, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10 },
    { "Q27", "straight across, real track 10 % narrower (turns more)",
      3000, -400, 90, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.10 },
    { "Q28", "60 deg, real track 10 % wider",
      3000, -400, 60, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10 },
    { "Q29", "30 deg, real track 10 % narrower",
      3000, -300, 30, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.10 },
    { "Q30", "straight across, real IRs 12 mm off the centre (not 12.5)",
      3000, -400, 90, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.0 },
    { "Q31", "45 deg, real IRs 12 mm off the centre (not 12.5)",
      3000, -400, 45, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.0 },
    /* nyx, 2026-11-09: the line at the start is a bend (R300 left 90
     * from x 2800) */
    { "Q32", "across a R300 left bend, 48 deg to it, from outside",
      3000, -300, 90, 0, 0.0, 0.0, 300.0, 90.0, 2800.0 },
    { "Q33", "across a R300 left bend from inside, 60 deg to it",
      2950, 250, -90, 0, 0.0, 0.0, 300.0, 90.0, 2800.0 },
    { "Q34", "near a R300 left bend, outside, IRs 6 cm off, 40 deg in",
      2937, -130, 70, 0, 0.0, 0.0, 300.0, 90.0, 2800.0 },
    { "Q35", "beside a R300 left bend, inside, IRs 6 cm off, on tangent",
      2812, 30, 30, 0, 0.0, 0.0, 300.0, 90.0, 2800.0 },
    { "Q36", "across a R300 left bend from outside, wheels turn 30 % less",
      3000, -300, 90, 0, 0.0, 0.0, 300.0, 90.0, 2800.0, 0.0, 0.0, 0.30 },
    { "Q37", "square across a R300 left bend from inside: LEFT, along it",
      2950, 250, -60, 0, 0.0, 0.0, 300.0, 90.0, 2800.0 },
    { "Q38", "across a R300 RIGHT bend, 48 deg to it, from outside",
      3000, 300, -90, 0, 0.0, 0.0, 300.0, -90.0, 2800.0 },
    { "Q39", "near a R300 RIGHT bend, outside, IRs 6 cm off, 40 deg in",
      2937, 130, -70, 0, 0.0, 0.0, 300.0, -90.0, 2800.0 },
    { "Q40", "across a R200 left bend, from outside",
      2950, -300, 90, 0, 0.0, 0.0, 200.0, 90.0, 2800.0 },
    { "Q41", "across a R200 RIGHT bend, from outside",
      2950, 300, -90, 0, 0.0, 0.0, 200.0, -90.0, 2800.0 },
    { "Q42", "square across a R200 left bend from inside: LEFT, along it",
      2839, 154, -50, 0, 0.0, 0.0, 200.0, 90.0, 2800.0 },
    { "Q43", "across a R200 left bend from outside, wheels turn 20 % less",
      2950, -300, 90, 0, 0.0, 0.0, 200.0, 90.0, 2800.0, 0.0, 0.0, 0.20 },
    { "Q44", "square across a R200 left bend from inside, turn 20 % less",
      2839, 154, -50, 0, 0.0, 0.0, 200.0, 90.0, 2800.0, 0.0, 0.0, 0.20 },
    { "Q45", "across a R300 RIGHT bend from outside, IRs 12 mm off centre",
      3000, 300, -90, 0, 0.0, 0.0, 300.0, -90.0, 2800.0, 0.0, 0.0, 0.0,
      12.0 },
    { "Q46", "across a R200 RIGHT bend, encoders 2 % long, L/R 0.5 %",
      2950, 300, -90, 0, 0.02, 0.005, 200.0, -90.0, 2800.0 },
    { "Q47", "started on a straight line 8 mm right of it, IR_L on the tape",
      500, -8, 0, 1 },
    { "Q48", "started on the line in a R400 RIGHT bend, IR_R on the tape",
      3000, 0, 0, 0, 0.0, 0.0, 400.0, -60.0, 3000.0 },
    { "none", "facing away from the line: stops with the fault",
      3000, -300, -90, 9 },
};
#define N_SCEN ((int)(sizeof(g_scen) / sizeof(g_scen[0])))

typedef struct
{
    int searching;
    int16_t sent_l;
    int16_t sent_r;
    uint32_t since_send_ms;
    int fault;
    b3_line_t line;
    b3_line_result_t snap;
} vc_t;

static vc_t g_vc;
static int g_verbose;

static void drive(int16_t l, int16_t r)
{
    if ((l != g_vc.sent_l) || (r != g_vc.sent_r) ||
        (g_vc.since_send_ms >= REFRESH))
    {
        if (MOTION_OK == motion_set_wheel_speeds(l, r))
        {
            g_vc.sent_l = l;
            g_vc.sent_r = r;
            g_vc.since_send_ms = 0U;
        }
    }
}

/* one vc_task pass: 1 = the search just handed over */
static int vc_pass(void)
{
    motion_status_t m;
    int done = 0;

    motion_get_status(&m);
    g_vc.since_send_ms += VC_MS;
    if (0 != g_vc.fault)
    {
        return 0;
    }
    if (!g_vc.searching)
    {
        s5_avoid_track(&m, true);
        drive(g_vc.snap.left_mm_s, g_vc.snap.right_mm_s);
    }
    else
    {
        int16_t l = 0;
        int16_t r = 0;

        switch (s5_avoid_step(&m, g_vc.snap.left_black,
                              g_vc.snap.right_black, (int32_t)VC_MS, &l,
                              &r))
        {
            case S5_AVOID_DONE:
                g_vc.sent_l = INT16_MIN;
                g_vc.sent_r = INT16_MIN;
                g_vc.searching = 0;
                done = 1;
                break;
            case S5_AVOID_FAULT:
                (void)motion_stop();
                g_vc.fault = 1;
                break;
            case S5_AVOID_RUNNING:
            default:
                drive(l, r);
                break;
        }
    }
    return done;
}

typedef struct
{
    int handed;
    int fault;
    double max_off;  /* |IR centre to the line|, AFTER_FROM_MM .. AFTER_MM */
    double end_deg;  /* heading at the end */
    double after;    /* followed this far */
    double search_s;
    int longest;
} res_t;

static res_t run(const scen_t *s, uint32_t seed)
{
    res_t o;
    uint32_t t = 0U;
    double after = -1.0;
    int first = 1;

    memset(&o, 0, sizeof o);
    sim_reset(seed);
    sim_log_verbose(g_verbose);
    if (s->bend_r > 0.0)
    {
        sim_track_start();
        sim_track_straight(s->bend_x + 400.0); /* the track starts at -400 */
        sim_track_arc(s->bend_r, s->bend_deg);
        sim_track_straight(2000.0);
    }
    if (0.0 != s->box_y)
    {
        g_sim.box[0].cx = s->x;
        g_sim.box[0].cy = s->box_y;
        g_sim.box[0].w = 160.0;
        g_sim.box[0].d = 210.0;
        g_sim.box[0].rot_deg = 0.0;
        g_sim.box[0].on = 1;
        g_sim.n_box = 1;
    }
    sim_car_place(s->x, s->y, s->th);
    if (s->ir_half > 0.0)
    {
        g_sim.ir_half = s->ir_half;
    }
    mm_set_errors(s->enc_err, s->track_err);
    mm_set_lr_error(s->lr_err);
    mm_set_dead_right(0);
    mm_set_lag(60, (double)B3_BASE_SPEED_MM_S);
    (void)mm_min_wheel_cmd(1);

    memset(&g_vc, 0, sizeof g_vc);
    g_vc.sent_l = INT16_MIN;
    g_vc.sent_r = INT16_MIN;
    b3_line_init(&g_vc.line, sim_now_us());
    (void)s5_hw_init();
    s5_sensor_init();
    s5_avoid_init();

    while (t < LIMIT_MS)
    {
        double x;
        double y;
        double th;

        if ((0.0 != s->box_y) && g_sim.box[0].on &&
            (t >= (uint32_t)s->box_gone_ms))
        {
            g_sim.box[0].on = 0;
            if (g_verbose)
            {
                (void)printf("    %7.3f  [SIM] box taken away\n", t / 1000.0);
            }
        }
        if (3U == (t % S5_POLL_MS))
        {
            s5_sensor_poll();
        }
        if (0U == (t % 10U))
        {
            g_vc.snap = b3_line_step(&g_vc.line, sim_ir_black(true),
                                     sim_ir_black(false), sim_now_us(),
                                     (int16_t)B3_BASE_SPEED_MM_S);
            if (g_vc.searching)
            {
                b3_line_hold(&g_vc.line); /* ctx.steer_hold */
            }
        }
        if (5U == (t % VC_MS))
        {
            /* Buddy 3 runs all the time, so at START its filtered IRs
             * are settled: begin after a few of its passes */
            if (first && (t >= 45U))
            {
                /* START after RESET with the search asked for */
                motion_status_t m;

                first = 0;
                motion_get_status(&m);
                s5_search_begin(&m);
                g_vc.searching = 1;
            }
            if (vc_pass())
            {
                o.handed = 1;
                o.search_s = t / 1000.0;
                after = 0.0;
            }
        }
        mm_step_1ms();
        sim_step_1ms();
        t++;

        sim_car_pose(&x, &y, &th);
        if (after >= 0.0)
        {
            after += fabs(mm_speed_true()) * 0.001;
            if (after >= AFTER_FROM_MM)
            {
                double off;

                (void)sim_ir_lane(&off);
                if (fabs(off) > o.max_off)
                {
                    o.max_off = fabs(off);
                }
            }
            if (after >= AFTER_MM)
            {
                o.after = after;
                o.end_deg = th * 180.0 / PI;
                break;
            }
        }
        if (0 != g_vc.fault)
        {
            o.fault = 1;
            break;
        }
        if ((g_verbose >= 2) && (0U == (t % 100U)))
        {
            (void)printf("    %7.3f  car x %7.1f y %7.1f th %6.1f  cmd %4d "
                         "%4d  ir %d%d %s\n",
                         t / 1000.0, x, y, th * 180.0 / PI, g_vc.sent_l,
                         g_vc.sent_r, sim_ir_black(true), sim_ir_black(false),
                         g_vc.searching ? "search" : "follow");
        }
    }
    o.longest = sim_log_longest();
    return o;
}

static double wrap_deg(double a)
{
    while (a > 180.0)
    {
        a -= 360.0;
    }
    while (a <= -180.0)
    {
        a += 360.0;
    }
    return a;
}

static int judge(const scen_t *s, const res_t *o, char *p_why, size_t n)
{
    p_why[0] = '\0';
    if (o->longest > 159)
    {
        (void)snprintf(p_why, n, "a log line is %d chars", o->longest);
        return 0;
    }
    if (9 == s->want)
    {
        if (!o->fault || (sim_log_find("no line within", 0) < 0))
        {
            (void)snprintf(p_why, n, "expected the no-line fault");
            return 0;
        }
        return 1;
    }
    if (o->fault)
    {
        (void)snprintf(p_why, n, "fault");
        return 0;
    }
    if (!o->handed || (o->after < AFTER_MM))
    {
        (void)snprintf(p_why, n, "never handed over / followed");
        return 0;
    }
    if (o->max_off > AFTER_OFF_MM)
    {
        (void)snprintf(p_why, n, "IRs %.0f mm off the line while following",
                       o->max_off);
        return 0;
    }
    if (0 != s->want)
    {
        const double want = (s->want > 0) ? 0.0 : 180.0;
        const double d = wrap_deg(o->end_deg - want);

        if (fabs(d) > AFTER_DEG)
        {
            (void)snprintf(p_why, n, "faces %.0f deg, wanted %.0f",
                           o->end_deg, want);
            return 0;
        }
    }
    return 1;
}

int main(int argc, char **argv)
{
    const char *only = NULL;
    int seeds = 5;
    int fails = 0;
    int runs = 0;
    int a;
    int i;

    for (a = 1; a < argc; a++)
    {
        if ((0 == strcmp(argv[a], "-s")) && (a + 1 < argc))
        {
            seeds = atoi(argv[++a]);
        }
        else if (0 == strcmp(argv[a], "v"))
        {
            g_verbose = 1;
        }
        else if (0 == strcmp(argv[a], "vv"))
        {
            g_verbose = 2;
        }
        else
        {
            only = argv[a];
        }
    }
    (void)printf("Line search: IRs %d mm ahead, %d mm apart; search %d mm/s, "
                 "line speed %d mm/s\n",
                 (int)S5_IR_AHEAD_MM, (int)(2.0f * S5_IR_HALF_GAP_MM),
                 (int)S5_SEARCH_MM_S, (int)B3_BASE_SPEED_MM_S);
    for (i = 0; i < N_SCEN; i++)
    {
        const scen_t *s = &g_scen[i];
        int k;
        int ok_n = 0;
        double worst = 0.0;
        double t_max = 0.0;
        char why[160] = "";

        if ((NULL != only) && (0 != strcmp(only, s->id)))
        {
            continue;
        }
        for (k = 0; k < seeds; k++)
        {
            const res_t o = run(s, (uint32_t)(1000 + (17 * k)));
            char w[96];

            runs++;
            if (judge(s, &o, w, sizeof w))
            {
                ok_n++;
            }
            else
            {
                fails++;
                (void)snprintf(why, sizeof why, "seed %d: %s", k, w);
            }
            worst = (o.max_off > worst) ? o.max_off : worst;
            t_max = (o.search_s > t_max) ? o.search_s : t_max;
        }
        (void)printf("%-5s %-62s %s  %d/%d  worst %4.1f mm off, handover "
                     "by %5.1f s  %s\n",
                     s->id, s->name, (ok_n == seeds) ? "PASS" : "FAIL", ok_n,
                     seeds, worst, t_max, why);
    }
    if (0 != fails)
    {
        (void)printf("%d of %d line search run(s) FAILED\n", fails, runs);
        return 1;
    }
    (void)printf("All %d line search runs passed.\n", runs);
    return 0;
}
