/*
 *      subsystem5_test.c
 *      Subsystem 5 -- on-target tests, make TEST=1.
 *      IDs as in testing/subsystem5/TEST_LOG.md. The robot can stand
 *      anywhere: S5-T01/T02 only need the sensor wired and powered; the
 *      rest check the maths on the RP2040 itself (soft float, no libm).
 */

#include "subsystem5.h"

#if APP_TEST

#include "../subsystem2/motion_config.h"
#include "../test_report.h"
#include "s5_config.h"
#include "s5_hw.h"
#include "s5_math.h"
#include "s5_odom.h"
#include "s5_path.h"
#include "s5_scan.h"
#include "s5_sensor.h"

static s5_sample_t g_s[S5_SCAN_MAX];

static bool s5_t_near(float a, float b, float tol)
{
    return s5_absf(a - b) <= tol;
}

/* A box with its face square to the sensor, face mm ahead, from y_right
 * to y_left; ex > 0: something else, a corner at (ex, ey). */
typedef struct
{
    float face;
    float y_right;
    float y_left;
    float ex;
    float ey;
} s5_t_scene_t;

/* What the sensor hears looking th (rad) off straight ahead: the nearest
 * point of the face (corners included), or the other corner, whose
 * bearing is within the beam of th. Unlike a real face this has no gap
 * between the face's echo and a corner's, so the result does not depend
 * on the calibrated beam. */
static uint16_t s5_t_hear(const s5_t_scene_t *p_sc, float th, float beam)
{
    float best = 0.0f;
    float lo = th - beam;
    float hi = th + beam;
    const float b_lo = s5_atan2f(p_sc->y_right, p_sc->face);
    const float b_hi = s5_atan2f(p_sc->y_left, p_sc->face);

    lo = (lo < b_lo) ? b_lo : lo;
    hi = (hi > b_hi) ? b_hi : hi;
    if (lo <= hi)
    {
        /* the face point nearest the sensor: nearest bearing 0 */
        const float b = (lo > 0.0f) ? lo : ((hi < 0.0f) ? hi : 0.0f);

        best = p_sc->face / s5_cosf(b);
    }
    if (p_sc->ex > 0.0f)
    {
        const float b = s5_atan2f(p_sc->ey, p_sc->ex);
        const float r = s5_sqrtf((p_sc->ex * p_sc->ex) + (p_sc->ey * p_sc->ey));

        if ((s5_absf(th - b) <= beam) && ((0.0f == best) || (r < best)))
        {
            best = r;
        }
    }
    return (uint16_t)s5_roundf(best);
}

static uint32_t s5_t_add(uint32_t n, int32_t ddeg, const s5_t_scene_t *p_sc,
                         float beam)
{
    if (n >= S5_SCAN_MAX)
    {
        return n;
    }
    g_s[n].ddeg = (int16_t)ddeg;
    g_s[n].mm = s5_t_hear(p_sc, (float)ddeg * (S5_DEG2RAD_F / 10.0f), beam);
    return n + 1U;
}

/* The sweep as s5_sensor.c does it: S5_SCAN_STEP_DDEG steps, then
 * S5_SCAN_FINE halvings at each edge. */
static uint32_t s5_t_sweep(const s5_t_scene_t *p_sc)
{
    const float beam =
        (float)s5_scan_beam_ddeg((uint16_t)s5_roundf(p_sc->face)) *
        (S5_DEG2RAD_F / 10.0f);
    uint32_t n = 0U;
    int32_t a;
    uint16_t face;

    for (a = S5_SERVO_LIMIT_DDEG; a >= -S5_SERVO_LIMIT_DDEG;
         a -= S5_SCAN_STEP_DDEG)
    {
        n = s5_t_add(n, a, p_sc, beam);
    }
    s5_scan_sort(g_s, n);
    face = s5_scan_face(g_s, n);
    if (0U != face)
    {
        int16_t obj[2];
        int16_t clr[2];
        bool b_seen[2];
        uint32_t e;

        s5_scan_edges(g_s, n, face, &obj[0], &clr[0], &b_seen[0], &obj[1],
                      &clr[1], &b_seen[1]);
        for (e = 0U; e < 2U; e++)
        {
            uint32_t level;

            for (level = 0U; b_seen[e] && (level < S5_SCAN_FINE); level++)
            {
                const int32_t probe = ((int32_t)obj[e] + (int32_t)clr[e]) / 2;
                const uint32_t k = n;

                n = s5_t_add(n, probe, p_sc, beam);
                if ((n > k) && s5_scan_is_object(g_s[k].mm, face))
                {
                    obj[e] = (int16_t)probe;
                }
                else
                {
                    clr[e] = (int16_t)probe;
                }
            }
        }
        s5_scan_sort(g_s, n);
    }
    return n;
}

void subsystem5_test(void)
{
    TEST_INIT();
    s5_ping_t p0 = { 0 };
    s5_ping_t p1 = { 0 };
    const s5_view_t view = { 0.0f, 0.0f, 0.0f };
    s5_profile_t pr;
    uint32_t n;

    tm_printf((UB *)"[TEST] --- subsystem 5 (ultrasonic + servo) ---\n");

    /* S5-T01/T02: the S5 task and the sensor */
    s5_sensor_guard();
    tk_dly_tsk(500);
    (void)s5_sensor_ping(&p0);
    tk_dly_tsk(1000);
    (void)s5_sensor_ping(&p1);
    TEST_CHECK("S5-T01", (p1.seq - p0.seq) >= 12U,
               "S5 task pings about every 60 ms (NFR10)");
    TEST_CHECK("S5-T02", !s5_sensor_dead() && p1.b_valid,
               "HC-SR04P answers on TRIG GP16 / ECHO GP17 (FR25)");
    tm_printf((UB *)"[TEST] S5 range straight ahead: %d mm (0 = nothing "
                    "within 2 m), %d pings in 1 s\n",
              (INT)p1.mm, (INT)(p1.seq - p0.seq));

    /* S5-T03: the maths without libm */
    TEST_CHECK("S5-T03",
               s5_t_near(s5_sinf(0.5f), 0.4794255f, 1e-4f) &&
                   s5_t_near(s5_cosf(2.0f), -0.4161468f, 1e-4f) &&
                   s5_t_near(s5_atan2f(-1.0f, -2.0f), -2.6779451f, 1e-4f) &&
                   s5_t_near(s5_sqrtf(2.0e4f), 141.42136f, 1e-2f) &&
                   s5_t_near(s5_tanf(0.7f), 0.8422884f, 1e-4f),
               "sin, cos, atan2, sqrt, tan within 1e-4");

    /* S5-T04: width of the 21 cm box at 250 mm */
    {
        const s5_t_scene_t sc = { 250.0f, -105.0f, 105.0f, 0.0f, 0.0f };

        n = s5_t_sweep(&sc);
    }
    s5_scan_profile(g_s, n, &view, &pr);
    TEST_CHECK("S5-T04",
               pr.b_found && pr.b_left_seen && pr.b_right_seen &&
                   s5_t_near(pr.width, 210.0f, 15.0f),
               "a 210 mm box at 250 mm reads 210 +- 15 mm wide (the "
               "calibrated beam, coarse and fine sweep)");
    tm_printf((UB *)"[TEST] S5 synthetic box: %d mm wide, face %d mm\n",
              (INT)s5_roundf(pr.width), (INT)pr.face_mm);

    /* S5-T05: off to the right, so the left side is the short way */
    {
        const s5_t_scene_t sc = { 250.0f, -185.0f, 25.0f, 0.0f, 0.0f };

        n = s5_t_sweep(&sc);
    }
    s5_scan_profile(g_s, n, &view, &pr);
    TEST_CHECK("S5-T05",
               pr.b_found && (pr.y_left < -pr.y_right) && !pr.b_left_blocked,
               "box 80 mm right of the line: left is the shorter side");

    /* S5-T06: a second box where the car would pass on the left */
    {
        const s5_t_scene_t sc = { 250.0f, -105.0f, 105.0f, 330.0f, 190.0f };

        n = s5_t_sweep(&sc);
    }
    s5_scan_profile(g_s, n, &view, &pr);
    TEST_CHECK("S5-T06", pr.b_found && pr.b_left_blocked && !pr.b_right_blocked,
               "something beside the box blocks that side only");

    /* S5-T07: the ramp shape stays within the limits */
    {
        const float len = s5_path_ramp_len(215.0f);
        s5_path_t path = { 0.0f, len, 0.0f, 215.0f, 10000.0f, len, 0.0f };
        float worst_slope = 0.0f;
        float worst_curv = 0.0f;
        float x;

        for (x = 0.0f; x <= len; x += 2.0f)
        {
            s5_ref_t r;

            s5_path_eval(&path, x, &r);
            worst_slope = (s5_absf(r.slope) > worst_slope) ? s5_absf(r.slope)
                                                           : worst_slope;
            worst_curv =
                (s5_absf(r.curv) > worst_curv) ? s5_absf(r.curv) : worst_curv;
        }
        TEST_CHECK(
            "S5-T07",
            (worst_slope <=
             (s5_tanf((float)S5_MAX_HEADING_DEG * S5_DEG2RAD_F) + 0.01f)) &&
                (worst_curv <= (1.0f / ((float)S5_MIN_RADIUS_MM - 5.0f))),
            "bypass ramp: no steeper than 40 deg, no tighter than "
            "R150");
    }

    /* S5-T08: the footprint check sees a ramp that cuts the corner */
    {
        const float len = s5_path_ramp_len(215.0f);
        const s5_box_t box = { 320.0f, 480.0f, -105.0f, 105.0f };
        s5_path_t late = { 300.0f, len, 0.0f, 215.0f, 10000.0f, len, 0.0f };
        s5_path_t early = { -400.0f, len, 0.0f, 215.0f, 10000.0f, len, 0.0f };

        TEST_CHECK("S5-T08",
                   !s5_path_clear(&late, &box, 30.0f, 300.0f, 300.0f + len) &&
                       s5_path_clear(&early, &box, 30.0f, -400.0f,
                                     -400.0f + len + 300.0f),
                   "car outline vs the box: too late a ramp hits it");
    }

    /* S5-T09: dead reckoning from ticks */
    {
        s5_odom_t o;
        const int32_t rev = 1272;

        s5_odom_init(&o);
        s5_odom_update(&o, 0, 0);
        s5_odom_update(&o, rev / 4, rev / 4);
        s5_odom_update(&o, rev / 2, rev / 2);
        TEST_CHECK(
            "S5-T09",
            s5_t_near(o.x, 2.0f * (float)(rev / 4) * MM_PER_TICK, 0.5f) &&
                s5_t_near(o.y, 0.0f, 0.1f),
            "encoder ticks to distance (straight)");
    }

    TEST_SUMMARY("subsystem5");
}

#else
typedef int s5_test_not_built_t; /* ISO C: no empty file */
#endif /* APP_TEST */
