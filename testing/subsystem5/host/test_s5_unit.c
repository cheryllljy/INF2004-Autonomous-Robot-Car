/**
 * @file    test_s5_unit.c
 * @brief   HOST TEST (S5-H11..H20; H01..H10 belonged
 *          to the old subsystem 5 and are not reused). Subsystem 5's parts
 *          one at a time: the maths, the sweep and the width (the real
 *          S5 task code on the simulated sensor, s5_sim.c), the room
 *          beside the object, the path, the footprint check, the steering,
 *          the odometry and the sensor's timing.
 *
 *   ./test_s5_unit        all, one line per check, exit 1 if any failed
 *   ./test_s5_unit -v     also the numbers behind each check
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_config.h"
#include "motion_mock.h"
#include "s5_config.h"
#include "s5_hw.h"
#include "s5_math.h"
#include "s5_odom.h"
#include "s5_path.h"
#include "s5_scan.h"
#include "s5_sensor.h"
#include "s5_sim.h"

#define PI (3.14159265358979323846)

static int g_fail;
static int g_verbose;
static uint32_t g_poll_ms = S5_POLL_MS; /* -p 20: no cyclic handler */

static void check(const char *p_id, int ok, const char *p_what)
{
    (void)printf("[%s] %-7s %s\n", ok ? "PASS" : "FAIL", p_id, p_what);
    if (!ok)
    {
        g_fail++;
    }
}

/*=====================================================================*/
/* The simulated sensor, car standing still                              */
/*=====================================================================*/

/* Run the S5 task for ms milliseconds (poll every g_poll_ms). */
static void run_ms(uint32_t ms)
{
    uint32_t t;

    for (t = 0U; t < ms; t++)
    {
        if (3U == (sim_ms() % g_poll_ms))
        {
            s5_sensor_poll();
        }
        mm_step_1ms();
        sim_step_1ms();
    }
}

static void world(uint32_t seed)
{
    sim_reset(seed);
    sim_car_place(0.0, 0.0, 0.0);
    mm_set_errors(0.0, 0.0);
    (void)s5_hw_init();
    s5_sensor_init();
    run_ms(500U);
}

static void add_box(double face, double y, double w, double d)
{
    sim_box_t *b = &g_sim.box[g_sim.n_box];

    b->cx = 70.0 + face + d / 2.0; /* face measured from the sensor */
    b->cy = y;
    b->w = w;
    b->d = d;
    b->rot_deg = 0.0;
    b->on = 1;
    g_sim.n_box++;
}

/* One sweep with the real S5 task code; the profile from where the car
 * stands (line frame = track frame, sensor 70 mm ahead). */
static int sweep(s5_profile_t *p_pr, uint32_t *p_ms)
{
    s5_sweep_t sw;
    const uint32_t seq = s5_sensor_scan();
    const uint32_t t0 = sim_ms();
    s5_view_t view = { 70.0f, 0.0f, 0.0f };

    while (!s5_sensor_scan_get(seq, &sw))
    {
        run_ms(10U);
        if ((sim_ms() - t0) > (uint32_t)S5_SCAN_TIMEOUT_MS)
        {
            return 0;
        }
    }
    *p_ms = sim_ms() - t0;
    s5_sensor_guard();
    s5_scan_profile(sw.s, sw.n, &view, p_pr);
    if (g_verbose >= 2)
    {
        uint32_t i;

        for (i = 0U; i < sw.n; i++)
        {
            (void)printf("          %5d %4u\n", sw.s[i].ddeg, sw.s[i].mm);
        }
    }
    return 1;
}

/*=====================================================================*/
/* H01 maths                                                             */
/*=====================================================================*/

static void h01(void)
{
    double e_sin = 0.0;
    double e_cos = 0.0;
    double e_tan = 0.0;
    double e_atan2 = 0.0;
    double e_sqrt = 0.0;
    double a;
    double y;

    for (a = -7.0; a <= 7.0; a += 0.001)
    {
        e_sin = fmax(e_sin, fabs(s5_sinf((float)a) - sin(a)));
        e_cos = fmax(e_cos, fabs(s5_cosf((float)a) - cos(a)));
    }
    for (a = -1.45; a <= 1.45; a += 0.001)
    {
        e_tan = fmax(e_tan, fabs(s5_tanf((float)a) - tan(a)) / fmax(1.0, fabs(tan(a))));
    }
    for (y = -500.0; y <= 500.0; y += 7.0)
    {
        double x;

        for (x = -500.0; x <= 500.0; x += 7.0)
        {
            e_atan2 = fmax(e_atan2,
                           fabs(remainder(s5_atan2f((float)y, (float)x) - atan2(y, x),
                                          2.0 * PI)));
        }
    }
    for (a = 0.01; a <= 4.0e6; a *= 1.01)
    {
        e_sqrt = fmax(e_sqrt, fabs(s5_sqrtf((float)a) - sqrt(a)) / sqrt(a));
    }
    if (g_verbose)
    {
        (void)printf("        sin %.1e cos %.1e tan %.1e atan2 %.1e sqrt %.1e\n",
                     e_sin, e_cos, e_tan, e_atan2, e_sqrt);
    }
    check("S5-H11",
          (e_sin < 2e-5) && (e_cos < 2e-5) && (e_tan < 1e-4) &&
              (e_atan2 < 2e-5) && (e_sqrt < 1e-5) &&
              (5 == s5_roundf(4.5f)) && (-5 == s5_roundf(-4.5f)) &&
              (fabs(s5_wrapf(7.0f) - (7.0 - 2.0 * PI)) < 1e-5),
          "sin, cos, tan, atan2, sqrt, round, wrap without libm");
}

/*=====================================================================*/
/* H02 width from the real sweep                                          */
/*=====================================================================*/

static void h02(void)
{
    static const double widths[] = { 80, 140, 210, 260, 320 };
    static const double faces[] = { 200, 250, 300, 380 };
    static const double offs[] = { -80, 0, 60 };
    double worst_w = 0.0;
    double worst_e = 0.0;
    uint32_t worst_ms = 0U;
    int bad = 0;
    int n = 0;
    size_t i;
    size_t j;
    size_t k;

    for (i = 0; i < sizeof widths / sizeof widths[0]; i++)
    {
        for (j = 0; j < sizeof faces / sizeof faces[0]; j++)
        {
            for (k = 0; k < sizeof offs / sizeof offs[0]; k++)
            {
                s5_profile_t pr;
                uint32_t ms = 0U;
                const double w = widths[i];
                const double y = offs[k];

                world((uint32_t)(i * 100 + j * 10 + k + 1));
                add_box(faces[j], y, w, 160.0);
                if (!sweep(&pr, &ms) || !pr.b_found || !pr.b_left_seen ||
                    !pr.b_right_seen)
                {
                    /* a wide box near: an edge outside the sweep is right
                     * (the sweep goes less far to the right) */
                    const double beam = 22.0 * PI / 180.0;
                    const double l_lim =
                        (double)S5_SERVO_LIMIT_DDEG * PI / 1800.0;
                    const double r_lim =
                        (double)S5_SERVO_LIMIT_R_DDEG * PI / 1800.0;

                    if (((atan2(y + w / 2.0, faces[j]) + beam) < l_lim) &&
                        ((atan2(w / 2.0 - y, faces[j]) + beam) < r_lim))
                    {
                        bad++;
                        if (g_verbose)
                        {
                            (void)printf("        w %.0f face %.0f y %.0f: no width\n",
                                         w, faces[j], y);
                        }
                    }
                    continue;
                }
                n++;
                worst_ms = (ms > worst_ms) ? ms : worst_ms;
                {
                    const double ew = fabs(pr.width - w);
                    const double el = fabs(pr.y_left - (y + w / 2.0));
                    const double er = fabs(pr.y_right - (y - w / 2.0));

                    worst_w = fmax(worst_w, ew);
                    worst_e = fmax(worst_e, fmax(el, er));
                    if (g_verbose && ((ew > 15.0) || (el > 12.0) || (er > 12.0)))
                    {
                        (void)printf("        w %.0f face %.0f y %.0f: %.0f wide, "
                                     "edges %.0f / %.0f\n",
                                     w, faces[j], y, pr.width, pr.y_left,
                                     pr.y_right);
                    }
                }
            }
        }
    }
    if (g_verbose)
    {
        (void)printf("        %d sweeps: width within %.1f mm, each edge within "
                     "%.1f mm, slowest sweep %u ms\n",
                     n, worst_w, worst_e, worst_ms);
    }
    check("S5-H12", (0 == bad) && (worst_w <= 20.0) && (worst_e <= 15.0) &&
                        (worst_ms <= 2000U),
          "real sweep: boxes 8..32 cm wide at 20..38 cm, width within 20 mm, "
          "each edge within 15 mm, under 2 s from the servo at 0 deg "
          "(NFR11 from detection: test_s5_car)");
}

/* H03: missed echoes (30 %) must not pull an edge in */
static void h03(void)
{
    double worst = 0.0;
    int s;

    for (s = 1; s <= 30; s++)
    {
        s5_profile_t pr;
        uint32_t ms;

        world((uint32_t)(1000 + s));
        g_sim.drop = 0.15;
        add_box(260.0, 0.0, 210.0, 160.0);
        if (sweep(&pr, &ms) && pr.b_found)
        {
            worst = fmax(worst, fabs(pr.width - 210.0));
            if (g_verbose && (fabs(pr.width - 210.0) > 35.0))
            {
                (void)printf("        seed %d: %.0f wide (%.0f / %.0f), seen %d %d\n",
                             s, pr.width, pr.y_left, pr.y_right,
                             pr.b_left_seen, pr.b_right_seen);
            }
        }
        else
        {
            worst = 1e9;
        }
    }
    if (g_verbose)
    {
        (void)printf("        30 sweeps with 15 %% dropouts: width within %.1f mm\n",
                     worst);
    }
    check("S5-H13", worst <= 25.0,
          "15 % missed echoes: the 21 cm box still reads 21 cm +- 25 mm");
}

/* H04: room beside the object */
static void h04(void)
{
    s5_profile_t pr;
    uint32_t ms;
    int ok = 1;

    int r1;
    int r2;
    int r3;
    int r4;

    world(7U);
    add_box(260.0, 0.0, 210.0, 160.0);
    r1 = sweep(&pr, &ms) && !pr.b_left_blocked && !pr.b_right_blocked;
    world(8U);
    add_box(260.0, 0.0, 210.0, 160.0);
    add_box(320.0, 300.0, 160.0, 160.0); /* where the car passes left */
    r2 = sweep(&pr, &ms) && pr.b_left_blocked && !pr.b_right_blocked;
    world(9U);
    add_box(260.0, 0.0, 210.0, 160.0);
    add_box(200.0, -330.0, 200.0, 400.0); /* a wall on the right */
    /* its echoes run on from the box's: the right edge is not found, or
     * the right side is blocked; either way not that way */
    r3 = sweep(&pr, &ms) && !pr.b_left_blocked && pr.b_left_seen &&
         (pr.b_right_blocked || !pr.b_right_seen);
    world(10U);
    add_box(260.0, 0.0, 210.0, 160.0);
    add_box(700.0, 450.0, 160.0, 160.0); /* well out of the way */
    r4 = sweep(&pr, &ms) && !pr.b_left_blocked && !pr.b_right_blocked;
    ok = r1 && r2 && r3 && r4;
    if (g_verbose)
    {
        (void)printf("        alone %d, box left %d, wall right %d, far box %d\n",
                     r1, r2, r3, r4);
    }
    check("S5-H14", ok,
          "another box or a wall where the car would pass rules that side "
          "out, and only that side; one well out of the way does not");
}

/*=====================================================================*/
/* H05 ramps, H06 footprint, H07 steering                                */
/*=====================================================================*/

static void h05(void)
{
    double worst_slope = 0.0;
    double worst_curv = 0.0;
    double worst_jump = 0.0;
    double dy;

    for (dy = -400.0; dy <= 400.0; dy += 10.0)
    {
        const float len = s5_path_ramp_len((float)dy);
        s5_path_t p = { 0.0f, len, 0.0f, (float)dy, 100000.0f, len, 0.0f };
        s5_ref_t prev;
        float x;

        s5_path_eval(&p, -1.0f, &prev);
        for (x = 0.0f; x <= len + 2.0f; x += 1.0f)
        {
            s5_ref_t r;

            s5_path_eval(&p, x, &r);
            worst_slope = fmax(worst_slope, fabs(r.slope));
            worst_curv = fmax(worst_curv, fabs(r.curv));
            worst_jump = fmax(worst_jump, fabs(r.slope - prev.slope));
            prev = r;
        }
        if (fabs(prev.y - dy) > 0.01)
        {
            worst_jump = 1e9;
        }
    }
    if (g_verbose)
    {
        (void)printf("        steepest %.1f deg, tightest R%.0f, biggest slope step "
                     "%.4f per mm\n",
                     atan(worst_slope) * 180.0 / PI, 1.0 / worst_curv, worst_jump);
    }
    check("S5-H15",
          (atan(worst_slope) * 180.0 / PI <= S5_MAX_HEADING_DEG + 0.5) &&
              (1.0 / worst_curv >= S5_MIN_RADIUS_MM - 3.0) && (worst_jump < 0.01),
          "ramps of 1..400 mm: no steeper than 40 deg, no tighter than R150, "
          "smooth, end where asked");
}

/* distance from the car outline (axle at x,y heading psi) to a box,
 * double precision, dense */
static double outline_gap(double x, double y, double psi, const s5_box_t *p_b)
{
    double best = 1e9;
    double f;
    double l;
    const double c = cos(psi);
    const double s = sin(psi);

    for (f = -S5_CAR_REAR_MM; f <= S5_CAR_FRONT_MM; f += 2.0)
    {
        for (l = -S5_CAR_HALF_WIDTH_MM; l <= S5_CAR_HALF_WIDTH_MM; l += 2.0)
        {
            const double px = x + f * c - l * s;
            const double py = y + f * s + l * c;
            const double dx = fmax(fmax(p_b->x_near - px, px - p_b->x_far), 0.0);
            const double dyy = fmax(fmax(p_b->y_right - py, py - p_b->y_left), 0.0);
            const double d = ((dx == 0.0) && (dyy == 0.0)) ? -1.0 : hypot(dx, dyy);

            best = fmin(best, d);
        }
    }
    return best;
}

static void h06(void)
{
    int wrong = 0;
    int clear_n = 0;
    int it;

    srand(12345);
    for (it = 0; it < 400; it++)
    {
        const double a = 100.0 + (rand() % 250);
        const float len = s5_path_ramp_len((float)a);
        const float x0 = (float)(-(rand() % 500));
        s5_path_t p = { x0, len, 0.0f, (float)a, 100000.0f, len, 0.0f };
        const s5_box_t b = { 300.0f, 460.0f, -105.0f, (float)(a - 65.0 - (rand() % 80)) };
        const int clear = s5_path_clear(&p, &b, 30.0f, x0, x0 + len + 600.0f);
        double gap = 1e9;
        double x;

        for (x = x0; x <= x0 + len + 600.0; x += 2.0)
        {
            s5_ref_t r;

            s5_path_eval(&p, (float)x, &r);
            gap = fmin(gap, outline_gap(x, r.y, atan(r.slope), &b));
        }
        if (clear)
        {
            clear_n++;
            if (gap < 27.0)
            {
                wrong++;
                if (g_verbose)
                {
                    (void)printf("        clear but %.1f mm (a %.0f)\n", gap, a);
                }
            }
        }
        else if (gap > 36.0)
        {
            wrong++;
            if (g_verbose)
            {
                (void)printf("        not clear but %.1f mm (a %.0f)\n", gap, a);
            }
        }
    }
    if (g_verbose)
    {
        (void)printf("        400 random ramps (%d clear): %d disagree with a dense "
                     "check\n",
                     clear_n, wrong);
    }
    check("S5-H16", 0 == wrong,
          "footprint check agrees with a dense outline-to-box distance (30 mm "
          "margin, -3 / +6 mm)");
}

/* the steering law on an ideal unicycle along a bypass path */
static void h07(void)
{
    const float len = s5_path_ramp_len(215.0f);
    s5_path_t p = { 0.0f, len, 0.0f, 215.0f, len + 200.0f, len, 0.0f };
    double x = -100.0;
    double y = 12.0; /* starts off the path */
    double psi = 5.0 * PI / 180.0;
    double worst = 0.0;
    double worst_k = 0.0;

    while (x < s5_path_end(&p) + 200.0)
    {
        s5_ref_t r;
        float k;

        s5_path_eval(&p, (float)x, &r);
        k = s5_path_steer(&r, (float)y, (float)psi, (float)S5_TRACK_DIST_MM);
        worst_k = fmax(worst_k, fabs(k));
        x += cos(psi);
        y += sin(psi);
        psi += k;
        if (x > 150.0)
        {
            s5_path_eval(&p, (float)x, &r);
            worst = fmax(worst, fabs(y - r.y));
        }
    }
    if (g_verbose)
    {
        (void)printf("        off the path by at most %.1f mm after 250 mm, end %.1f "
                     "mm, tightest R%.0f\n",
                     worst, y, 1.0 / worst_k);
    }
    check("S5-H17", (worst < 6.0) && (fabs(y) < 1.0),
          "steering: starts 12 mm / 5 deg off, follows a 215 mm bypass within "
          "6 mm, ends on the line");
}

/*=====================================================================*/
/* H08 odometry                                                          */
/*=====================================================================*/

static void h08(void)
{
    s5_odom_t o;
    const double r = 300.0;
    const double half = 0.5 * s5_odom_track_mm();
    double tl = 0.0;
    double tr = 0.0;
    int i;

    s5_odom_init(&o);
    s5_odom_update(&o, 0, 0);
    /* a quarter circle to the left, radius 300, in 1 mm steps */
    for (i = 0; i < (int)(PI / 2.0 * r); i++)
    {
        tl += (1.0 - half / r) / MM_PER_TICK;
        tr += (1.0 + half / r) / MM_PER_TICK;
        s5_odom_update(&o, (int32_t)lround(tl), (int32_t)lround(tr));
    }
    if (g_verbose)
    {
        (void)printf("        quarter circle R300: x %.1f y %.1f psi %.2f deg\n",
                     o.x, o.y, o.psi * 180.0 / PI);
    }
    check("S5-H18",
          (fabs(o.x - r) < 3.0) && (fabs(o.y - r) < 3.0) &&
              (fabs(o.psi * 180.0 / PI - 90.0) < 0.5),
          "dead reckoning: quarter circle R300 to the left ends at (300, 300), "
          "90 deg");
}

/*=====================================================================*/
/* H09 sensor timing, H10 dead sensor                                     */
/*=====================================================================*/

static void h09(void)
{
    s5_ping_t p0;
    s5_ping_t p1;
    uint32_t guard_rate;
    uint32_t slow_rate;
    int ok = 1;

    world(21U);
    add_box(250.0, 0.0, 210.0, 160.0);
    (void)s5_sensor_ping(&p0);
    run_ms(2000U);
    (void)s5_sensor_ping(&p1);
    guard_rate = (p1.seq - p0.seq) / 2U;
    ok &= p1.b_valid && p1.b_close && (abs((int)p1.mm - 250) <= 8);
    if (g_verbose)
    {
        (void)printf("        box: valid %d close %d mm %d\n", p1.b_valid,
                     p1.b_close, p1.mm);
    }

    /* nothing there, sensor holding ECHO 150 ms when it hears nothing */
    world(22U);
    g_sim.no_echo_ms = 150.0;
    (void)s5_sensor_ping(&p0);
    run_ms(3000U);
    (void)s5_sensor_ping(&p1);
    slow_rate = (p1.seq - p0.seq) / 3U;
    ok &= !s5_sensor_dead() && p1.b_valid && (0U == p1.mm) && !p1.b_close;
    if (g_verbose)
    {
        (void)printf("        nothing: dead %d valid %d mm %d close %d\n",
                     s5_sensor_dead(), p1.b_valid, p1.mm, p1.b_close);
    }

    /* LOOK follows a creeping angle and keeps pinging: slowly (10 deg/s)
     * and fast (60 deg/s: each step used to restart the settle
     * before the ping could go) */
    world(23U);
    {
        int32_t a;
        uint32_t before;
        uint32_t fast;
        int32_t worst = 0;

        (void)s5_sensor_ping(&p0);
        before = p0.seq;
        for (a = 0; a <= 300; a += 2)
        {
            s5_sensor_look(a);
            run_ms(20U);
        }
        (void)s5_sensor_ping(&p1);
        ok &= ((p1.seq - before) >= 30U) && (abs((int)p1.ddeg - 300) <= 15);
        if (g_verbose)
        {
            (void)printf("        look: %u pings, last at %d\n",
                         p1.seq - before, p1.ddeg);
        }
        before = p1.seq;
        for (a = 300; a >= -400; a -= 12)
        {
            s5_ping_t p;

            s5_sensor_look(a);
            run_ms(20U);
            if (s5_sensor_ping(&p) && (p.seq != p1.seq))
            {
                /* how far behind the asked angle the pings are */
                const int32_t lag = abs((int)p.ddeg - (int)a);

                worst = (lag > worst) ? lag : worst;
                p1 = p;
            }
        }
        fast = p1.seq - before;
        ok &= (fast >= 16U) && (worst <= 100);
        if (g_verbose)
        {
            (void)printf("        look at 60 deg/s: %u pings in 1.2 s, up to "
                         "%d.%d deg behind\n",
                         fast, (int)(worst / 10), (int)(worst % 10));
        }
    }
    if (g_verbose)
    {
        (void)printf("        %u pings/s on guard, %u/s when nothing answers "
                     "(150 ms pulses)\n",
                     guard_rate, slow_rate);
    }
    check("S5-H19", ok && (guard_rate >= 14U) && (slow_rate >= 5U),
          "pings: 14+ per second, a box at 250 mm reads 250 +- 8 and is close, "
          "a slow no-echo pulse is not a dead sensor, LOOK keeps pinging");
}

static void h10(void)
{
    s5_ping_t p;
    int ok;

    world(31U);
    g_sim.dead = 1;
    run_ms(1000U);
    ok = s5_sensor_dead() && (!s5_sensor_ping(&p) || !p.b_valid);
    world(32U);
    run_ms(1000U);
    ok = ok && !s5_sensor_dead();
    check("S5-H20", ok,
          "no ECHO pulse at all: dead within 1 s (FR25); a working sensor is "
          "not");
}

int main(int argc, char **argv)
{
    int i;

    for (i = 1; i < argc; i++)
    {
        if (0 == strcmp(argv[i], "-v"))
        {
            g_verbose = 1;
        }
        else if (0 == strcmp(argv[i], "-vv"))
        {
            g_verbose = 2;
        }
        else if ((0 == strcmp(argv[i], "-p")) && ((i + 1) < argc))
        {
            g_poll_ms = (uint32_t)atoi(argv[++i]);
            g_poll_ms = (g_poll_ms < 5U) ? 5U : g_poll_ms;
        }
        else
        {
            /* unknown: ignored */
        }
    }
    (void)printf("S5 task polls every %u ms\n", (unsigned)g_poll_ms);
    h01();
    h02();
    h03();
    h04();
    h05();
    h06();
    h07();
    h08();
    h09();
    h10();
    (void)printf("\nsubsystem 5 unit tests: %d failed\n", g_fail);
    return (0 == g_fail) ? 0 : 1;
}
