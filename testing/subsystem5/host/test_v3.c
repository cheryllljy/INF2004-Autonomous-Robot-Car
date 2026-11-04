/**
 * @file    test_v3.c
 * @brief   HOST TEST (S5-H05). v3.0 pure maths: calibrated refinement
 *          (the "centre point" fix), reverse distance, side-watch
 *          geometry, the shared sine table.
 *
 * Sweeps are generated here by a small ray caster with a beam of known
 * width, so every check compares S5's answer with the true object.
 *
 *   V1  sine table in 0.1 deg against libm
 *   V2  centre point: posts and boxes at several bearings, v2 (raw)
 *       against v3 (refined) centre and width error
 *   V3  unseen edge: a wall wider than the +/-30 deg view -> REVERSE,
 *       never a bypass through an unseen side
 *   V4  reverse distance rules
 *   V5  side-watch lateral and remaining-travel formulas against libm
 *   V6  refine leaves CLEAR / invalid / NULL profiles alone
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obstacle_model.h"
#include "s5_config.h"

#define PI (3.14159265358979323846)

static int g_checks;
static int g_failures;

static void check(int cond, const char *what)
{
    g_checks++;
    if (!cond)
    {
        g_failures++;
        printf("  FAIL: %s\n", what);
    }
}

/*=====================================================================*/
/* A tiny world: flat faces square to the sensor axis                  */
/*=====================================================================*/

typedef struct
{
    double x_min;
    double x_max;
    double y;
} face_t;

static face_t g_faces[4];
static int    g_nfaces;

static double cast(double bearing_deg)
{
    const double s = sin(bearing_deg * PI / 180.0);
    const double c = cos(bearing_deg * PI / 180.0);
    double best = 0.0;
    int i;

    for (i = 0; i < g_nfaces; i++)
    {
        const double t = g_faces[i].y / c;
        const double x = t * s;

        if ((c > 0.05) && (x >= g_faces[i].x_min) && (x <= g_faces[i].x_max) &&
            ((best == 0.0) || (t < best)))
        {
            best = t;
        }
    }

    return best;
}

/** Shortest return in the cone, as the HC-SR04 reports. */
static uint16_t beam(double bearing_deg, double half_deg)
{
    double best = 0.0;
    int i;

    for (i = 0; i <= 40; i++)
    {
        const double r = cast(bearing_deg - half_deg + (2.0 * half_deg * i / 40.0));

        if ((r > 0.0) && ((best == 0.0) || (r < best)))
        {
            best = r;
        }
    }

    return (best > 0.0) ? (uint16_t)(best + 0.5) : OBSTACLE_RANGE_CLEAR;
}

/** The S5 v2.2 pattern: coarse -30,-20,0,20,30, then fine every 10 deg
 *  round the in-path target, all inside +/-30: a 10 deg grid. */
static uint8_t sweep(s5_sample_t *p, double half_deg)
{
    static const int16_t b[7] = { -30, -20, -10, 0, 10, 20, 30 };
    uint8_t i;

    for (i = 0U; i < 7U; i++)
    {
        p[i] = s5_sample_pack(b[i], beam((double)b[i], half_deg));
    }

    return 7U;
}

/* The same bisection obstacle_task.c runs (S5_EDGE_PROBES pings per
 * seen edge), against the same beam. */
static s5_edge_probes_t g_pr;

static void probe_one(s5_edge_probe_t *e, int16_t edge_deg, int towards,
                      uint16_t nearest, double half)
{
    int32_t hit = edge_deg * 10;
    int32_t miss = hit + towards * 100;
    unsigned k;

    for (k = 0U; k < S5_EDGE_PROBES; k++)
    {
        const int32_t mid = (hit + miss) / 2;
        const uint16_t r = beam((double)mid / 10.0, half);

        if ((r != OBSTACLE_RANGE_CLEAR) && (r <= nearest + S5_PROBE_DEPTH_MM))
        {
            hit = mid;
        }
        else
        {
            miss = mid;
        }
    }
    e->valid = true;
    e->hit_ddeg = (int16_t)hit;
    e->miss_ddeg = (int16_t)miss;
}

static void probe(const obstacle_profile_t *p, double half)
{
    memset(&g_pr, 0, sizeof(g_pr));
    if (p->valid && (p->nearest_mm < 0xFFFEU))
    {
        if (p->left_edge_deg < 30)
        {
            probe_one(&g_pr.left, p->left_edge_deg, 1, p->nearest_mm, half);
        }
        if (p->right_edge_deg > -30)
        {
            probe_one(&g_pr.right, p->right_edge_deg, -1, p->nearest_mm, half);
        }
    }
}

static void world_box(double centre_x, double y, double width)
{
    g_nfaces = 1;
    g_faces[0].x_min = centre_x - (width / 2.0);
    g_faces[0].x_max = centre_x + (width / 2.0);
    g_faces[0].y     = y;
}

/*=====================================================================*/
/* V1                                                                  */
/*=====================================================================*/

static void v1_sine(void)
{
    int32_t d;
    double  worst = 0.0;

    printf("V1 sine table, 0.1 deg steps, -360..+360 deg\n");

    for (d = -3600; d <= 3600; d += 7)
    {
        const double got  = (double)obstacle_sin_q15_ddeg(d) / 32768.0;
        const double want = sin((double)d * PI / 1800.0);
        const double e    = fabs(got - want);

        if (e > worst)
        {
            worst = e;
        }
    }

    printf("  worst error %.5f\n", worst);
    check(worst < 0.002, "sine within 0.002 everywhere");
}

/*=====================================================================*/
/* V2: the centre point                                                */
/*=====================================================================*/

static void v2_one(const char *name, double centre_x, double y, double width,
                   double beam_half, double *p_raw_err, double *p_ref_err,
                   double *p_raw_w, double *p_ref_w, int *p_seen)
{
    s5_sample_t        s[8];
    obstacle_profile_t p;
    const s5_geom_cal_t cal = { (int16_t)(beam_half * 10.0 + 0.5), 10, 30, 300U };
    const double true_b = atan2(centre_x, y) * 180.0 / PI;
    double raw_b;
    uint16_t raw_w;
    uint8_t n;

    world_box(centre_x, y, width);
    n = sweep(s, beam_half);
    obstacle_profile_build(s, n, &p);
    raw_b = (double)p.bearing_deg;
    raw_w = p.width_mm;
    *p_seen = (p.left_edge_deg < 30) && (p.right_edge_deg > -30);
    probe(&p, beam_half);
    obstacle_profile_refine(&p, &cal, &g_pr);

    *p_raw_err = fabs(raw_b - true_b);
    *p_ref_err = fabs((double)p.bearing_deg - true_b);
    *p_raw_w   = fabs((double)raw_w - width);
    *p_ref_w   = fabs((double)p.width_mm - width);

    printf("  %-22s true %+5.1f deg %4.0f mm | v2 %+4d deg %4u mm | v3 %+4d deg %4u mm %s %s\n",
           name, true_b, width, (int)raw_b, (unsigned)raw_w, (int)p.bearing_deg,
           (unsigned)p.width_mm, *p_seen ? "  " : "E!",
           (p.action == OBSTACLE_ACT_BYPASS_LEFT) ? "BYPASS_L" :
           (p.action == OBSTACLE_ACT_BYPASS_RIGHT) ? "BYPASS_R" :
           (p.action == OBSTACLE_ACT_REVERSE) ? "REVERSE" : "other");
}

static void v2_centre(void)
{
    static const struct { const char *name; double cx; double y; double w; } k[] =
    {
        { "post 20 mm, centred",   0.0, 300.0, 20.0 },
        { "post 20 mm, 40 mm L",  40.0, 300.0, 20.0 },
        { "post 20 mm, 60 mm R", -60.0, 280.0, 20.0 },
        { "box 80 mm, centred",    0.0, 280.0, 80.0 },
        { "box 80 mm, 30 mm L",   30.0, 260.0, 80.0 },
        { "box 120 mm, 50 mm R", -50.0, 300.0, 120.0 },
    };
    const double beams[2] = { 7.5, 12.0 };
    double raw_sum = 0.0, ref_sum = 0.0, raw_ws = 0.0, ref_ws = 0.0;
    double ref_max = 0.0;
    int nb, i, n = 0;

    printf("V2 centre point and width, v2 (raw) vs v3 (refined)\n");

    for (nb = 0; nb < 2; nb++)
    {
        printf(" beam half-angle %.1f deg (calibrated to the same)\n", beams[nb]);
        for (i = 0; i < (int)(sizeof(k) / sizeof(k[0])); i++)
        {
            double re, fe, rw, fw;
            int seen;

            v2_one(k[i].name, k[i].cx, k[i].y, k[i].w, beams[nb], &re, &fe, &rw, &fw,
                   &seen);
            if (seen)
            {
                raw_sum += re; ref_sum += fe; raw_ws += rw; ref_ws += fw;
                if (fe > ref_max) { ref_max = fe; }
                n++;
            }
        }
    }

    printf("  (E! = an edge outside the +/-30 deg view: not counted, the\n"
           "   planner treats that side as unknown)\n");
    printf("  mean centre error  v2 %.1f deg  ->  v3 %.1f deg (worst %.1f)\n",
           raw_sum / n, ref_sum / n, ref_max);
    printf("  mean width error   v2 %.0f mm  ->  v3 %.0f mm\n",
           raw_ws / n, ref_ws / n);

    check(ref_sum < raw_sum, "v3 centre error smaller than v2 overall");
    check(ref_max <= 2.0, "v3 centre within 2 deg when both edges are seen");
    check(ref_ws < raw_ws, "v3 width error smaller than v2 overall");
}

/*=====================================================================*/
/* V3 / V4: unseen edges and reverse distance                          */
/*=====================================================================*/

static void v3_unseen(void)
{
    s5_sample_t        s[8];
    obstacle_profile_t p;
    const s5_geom_cal_t cal = { 75, 10, 30, 300U };
    uint8_t n;
    obstacle_action_t raw_action;
    uint16_t raw_gl, raw_gr;

    printf("V3 wall wider than the +/-30 deg view at 220 mm\n");

    world_box(0.0, 220.0, 600.0);
    n = sweep(s, 7.5);
    obstacle_profile_build(s, n, &p);
    raw_action = p.action;
    raw_gl = p.gap_left_mm;
    raw_gr = p.gap_right_mm;
    obstacle_profile_refine(&p, &cal, NULL);

    printf("  v2 action %d (gaps L %u / R %u), v3 action %d (gaps %u / %u), reverse %u mm\n",
           (int)raw_action, (unsigned)raw_gl, (unsigned)raw_gr, (int)p.action,
           (unsigned)p.gap_left_mm, (unsigned)p.gap_right_mm,
           (unsigned)obstacle_reverse_mm(&p));

    check((p.gap_left_mm == 0U) && (p.gap_right_mm == 0U),
          "both edges unseen -> both gaps unknown (0)");
    check(p.action == OBSTACLE_ACT_REVERSE, "-> REVERSE to see the edges");
    check(obstacle_reverse_mm(&p) == (uint16_t)(S5_REMEASURE_FAR_MM - 220U),
          "backs off to S5_REMEASURE_FAR_MM");

    /* Same wall seen from 400 mm with one edge inside the view. */
    g_nfaces = 1;
    g_faces[0].x_min = -60.0; g_faces[0].x_max = 600.0; g_faces[0].y = 400.0;
    n = sweep(s, 7.5);
    obstacle_profile_build(s, n, &p);
    obstacle_profile_refine(&p, &cal, NULL);
    printf("  from 400 mm, right edge at -60 mm: action %d, gap R %u\n",
           (int)p.action, (unsigned)p.gap_right_mm);
    check(p.gap_left_mm == 0U, "left edge still unseen -> no left bypass");
}

static void v4_reverse(void)
{
    obstacle_profile_t p;

    printf("V4 reverse distance\n");

    memset(&p, 0, sizeof(p));
    p.valid = true;
    p.action = OBSTACLE_ACT_REVERSE;

    p.nearest_mm = 90U;
    check(obstacle_reverse_mm(&p) == (uint16_t)(S5_REMEASURE_NEAR_MM - 90U),
          "inside the hard stop: back to 250 mm");
    p.nearest_mm = 40U;
    check(obstacle_reverse_mm(&p) == (uint16_t)(S5_REMEASURE_NEAR_MM - 40U),
          "40 mm: back 210 mm");
    p.nearest_mm = 200U;
    check(obstacle_reverse_mm(&p) == (uint16_t)(S5_REMEASURE_FAR_MM - 200U),
          "edge unseen at 200 mm: back to 400 mm");
    p.nearest_mm = 380U;
    check(obstacle_reverse_mm(&p) == (uint16_t)S5_REVERSE_MIN_MM,
          "never less than the minimum");
    p.nearest_mm = OBSTACLE_RANGE_UNKNOWN;
    check(obstacle_reverse_mm(&p) == (uint16_t)S5_REVERSE_MAX_MM,
          "unknown range: the maximum, never 0");
    p.action = OBSTACLE_ACT_BYPASS_LEFT;
    check(obstacle_reverse_mm(&p) == 0U, "not a REVERSE: 0");
    check(obstacle_reverse_mm(NULL) == 0U, "NULL: 0");
}

/*=====================================================================*/
/* V5: side watch                                                      */
/*=====================================================================*/

static void v5_watch(void)
{
    int16_t w;
    double worst_lat = 0.0, worst_rem = 0.0;

    printf("V5 side-watch geometry against libm\n");

    for (w = 20; w <= 90; w += 10)
    {
        int16_t h;

        for (h = 50; h <= 150; h += 25)
        {
            uint16_t r;

            for (r = 150U; r <= 600U; r = (uint16_t)(r + 75U))
            {
                const double a = (w * 10.0 + h) / 10.0;
                const double want = (a >= 90.0) ? r : r * sin(a * PI / 180.0);
                const double got = obstacle_watch_lateral_mm(r, w, h);
                const double e = fabs(got - want);

                if (e > worst_lat) { worst_lat = e; }
            }

            {
                const uint16_t lat = 130U;
                double ia = (w * 10.0 + h) / 10.0;
                double want;
                double e;

                if (ia < 5.0) { ia = 5.0; }
                if (ia > 90.0) { ia = 90.0; }
                want = ((ia >= 90.0) ? 0.0 : lat / tan(ia * PI / 180.0)) +
                       S5_REAR_FROM_SENSOR_MM + S5_PASS_MARGIN_MM;
                e = fabs(obstacle_watch_remaining_mm(lat, w, h) - want);
                if (e > worst_rem) { worst_rem = e; }
            }
        }
    }

    printf("  worst lateral error %.1f mm, worst remaining error %.1f mm\n",
           worst_lat, worst_rem);
    printf("  example: watch 30 deg, beam 7.5, face 135 mm beside -> %u mm more after clear\n",
           (unsigned)obstacle_watch_remaining_mm(135U, 30, 75));
    check(worst_lat <= 2.0, "lateral within 2 mm");
    check(worst_rem <= 3.0, "remaining travel within 3 mm");
    check(obstacle_watch_lateral_mm(OBSTACLE_RANGE_CLEAR, 30, 75) == 0xFFFFU,
          "CLEAR side ping is never 'beside us'");
}

/*=====================================================================*/
/* V6                                                                  */
/*=====================================================================*/

static void v6_safe(void)
{
    obstacle_profile_t p;
    obstacle_profile_t q;
    const s5_geom_cal_t cal = { 120, 10, 30, 300U };

    printf("V6 refine leaves non-obstacle profiles alone\n");

    obstacle_profile_build(NULL, 0U, &p);
    q = p;
    obstacle_profile_refine(&p, &cal, NULL);
    check(memcmp(&p, &q, sizeof(p)) == 0, "invalid profile unchanged");

    memset(&p, 0, sizeof(p));
    p.valid = true; p.nearest_mm = OBSTACLE_RANGE_CLEAR; p.confidence_pct = 100U;
    p.action = OBSTACLE_ACT_CONTINUE;
    q = p;
    obstacle_profile_refine(&p, &cal, NULL);
    check(memcmp(&p, &q, sizeof(p)) == 0, "CLEAR profile unchanged");

    obstacle_profile_refine(NULL, &cal, NULL);
    obstacle_profile_refine(&p, NULL, NULL);
    check(1, "NULL arguments do not crash");
}

int main(void)
{
    v1_sine();
    v2_centre();
    v3_unseen();
    v4_reverse();
    v5_watch();
    v6_safe();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
