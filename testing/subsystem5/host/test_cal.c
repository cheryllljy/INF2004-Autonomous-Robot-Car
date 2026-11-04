/**
 * @file    test_cal.c
 * @brief   HOST TEST (S5-H06). The calibration, end to end, against a
 *          deliberately wrong servo and sensor.
 *
 * The simulated robot has the faults the prof saw on the real one:
 *   servo  physical straight ahead at command 86.5 deg (trim -3.5 deg),
 *          1.5 deg of backlash
 *   beam   +/-12 deg instead of the datasheet +/-7.5
 *   range  reads 3 % long and 8 mm short (raw = 1.03 * true - 8)
 *
 *   ./test_cal cal       run the REAL s5_calibrate.c; the world follows
 *                        its prompts (post / board at 100..500 mm). The
 *                        values it finds must match the faults above.
 *   ./test_cal centre    run the REAL obstacle task on the same faulty
 *                        robot and report where it thinks a box is.
 *                        Built twice by the Makefile: test_cal (no
 *                        calibration, = v2.2) and test_cal_fixed (the
 *                        calibration values compiled in, = v3.0).
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "subsystem5.h"
#include "s5_calib.h"
#include "s5_config.h"
#include "sim_hal.h"

#define TRUE_TRIM_DDEG      (-35)
#define TRUE_BACKLASH_DDEG  (15)
#define TRUE_BEAM_DEG       (12.0)
#define TRUE_SCALE          (0.03)
#define TRUE_OFFSET_MM      (-8.0)

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

static void faulty_robot(void)
{
    sim_counters_reset();
    sim_world_clear();
    sim_set_servo_error(TRUE_TRIM_DDEG, TRUE_BACKLASH_DDEG);
    sim_set_beam_half_deg(TRUE_BEAM_DEG);
    sim_set_range_error(TRUE_SCALE, TRUE_OFFSET_MM);
    sim_set_noise_mm(2);
}

/*=====================================================================*/
/* cal: the world follows the calibration prompts                      */
/*=====================================================================*/

static void cal_prompt(const char *text)
{
    const char *p;

    if (strstr(text, "STEP 1/3") != NULL || strstr(text, "STEP 3/3") != NULL)
    {
        /* A 10 mm pen on the centre line, 300 mm from the sensor face:
         * x = 0 is where the horn REALLY points straight, not where
         * the servo thinks 90 deg is. */
        sim_world_clear();
        sim_add_box(0.0, 300.0, (double)S5_CAL_POST_DIAMETER_MM);
    }
    else if ((p = strstr(text, "mm from the sensor face")) != NULL &&
             strstr(text, "STEP 2/3") != NULL)
    {
        /* "... square to the sensor, NNN mm from the sensor face" */
        const char *q = p;
        int mm;

        while ((q > text) && (q[-1] == ' ')) { q--; }
        while ((q > text) && (q[-1] >= '0') && (q[-1] <= '9')) { q--; }
        mm = atoi(q);
        sim_world_clear();
        sim_add_box(0.0, (double)mm, 600.0);
    }
    else
    {
        /* Not a prompt. */
    }
}

static void stop_when_done(void)
{
    s5_cal_result_t r;

    s5_cal_get_result(&r);
    if (r.finished)
    {
        tkshim_stop();
    }
}

static int run_cal(void)
{
    s5_cal_result_t r;
    const double want_scale = 4096.0 / (1.0 + TRUE_SCALE);
    const double want_off   = -TRUE_OFFSET_MM / (1.0 + TRUE_SCALE);

    printf("H06a calibration against a faulty robot\n");
    printf("  truth: trim %d, backlash %d (0.1 deg), beam %.1f deg,"
           " range x%.2f %+.0f mm\n", TRUE_TRIM_DDEG, TRUE_BACKLASH_DDEG,
           TRUE_BEAM_DEG, 1.0 + TRUE_SCALE, TRUE_OFFSET_MM);

    tkshim_reset();
    faulty_robot();
    tkshim_set_quiet(1);
    tkshim_set_print_hook(cal_prompt);
    tkshim_set_idle_hook(stop_when_done);
    tkshim_set_time_budget_ms(600000U);
    tkshim_run(s5_calibrate_task);

    s5_cal_get_result(&r);
    printf("  found: trim %d, backlash %d, beam %d (0.1 deg), scale %d/4096"
           " (want %.0f), offset %d mm (want %.1f)\n", r.trim_ddeg,
           r.backlash_ddeg, r.beam_half_ddeg, (int)r.scale_q12, want_scale,
           (int)r.offset_mm, want_off);
    printf("  steps: %s / %s / %s, verify centre %d deg, took %u s\n",
           r.step1_ok ? "PASS" : "FAIL", r.step2_ok ? "PASS" : "FAIL",
           r.step3_ok ? "PASS" : "FAIL", r.verify_bearing_deg,
           (unsigned)(tkshim_now_ms() / 1000U));

    check(r.finished, "calibration finished");
    check(r.step1_ok && r.step2_ok && r.step3_ok, "all three steps pass");
    check(abs(r.trim_ddeg - TRUE_TRIM_DDEG) <= 10, "trim within 1 deg");
    check(abs(r.backlash_ddeg - TRUE_BACKLASH_DDEG) <= 10,
          "backlash within 1 deg");
    check(abs(r.beam_half_ddeg - (int)(TRUE_BEAM_DEG * 10.0)) <= 20,
          "beam half-angle within 2 deg");
    check(abs((int)r.scale_q12 - (int)(want_scale + 0.5)) <= 41,
          "range scale within 1 %");
    check(abs((int)r.offset_mm - (int)(want_off + 0.5)) <= 5,
          "range offset within 5 mm");
    check(abs(r.verify_bearing_deg) <= 2, "post found straight ahead");

    printf("  %d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/*=====================================================================*/
/* centre: where does the obstacle task put a box?                     */
/*=====================================================================*/

static obstacle_profile_t g_prof;
static int                g_have;

static void watch_profile(void)
{
    if ((tkshim_flag_pattern(flg_obstacle) & S5_EVT_PROFILED) != 0U)
    {
        (void)obstacle_get_profile(&g_prof);
        g_have = 1;
        tkshim_stop();
    }
}

static int run_centre(void)
{
    static const struct { double x; double y; double w; } k[3] =
    {
        {   0.0, 280.0,  80.0 },
        {  40.0, 260.0,  60.0 },
        { -45.0, 270.0, 100.0 },
    };
    double sum_b = 0.0, sum_w = 0.0, sum_r = 0.0;
    int i;

    printf("H06b %s: obstacle task on the faulty robot\n",
           (S5_CAL_DONE != 0) ? "CALIBRATED (v3.0)" : "UNCALIBRATED (v2.2 behaviour)");

    for (i = 0; i < 3; i++)
    {
        const double true_b = atan2(k[i].x, k[i].y) * 57.29578;

        tkshim_reset();
        faulty_robot();
        tkshim_set_quiet(1);
        sim_add_box(k[i].x, k[i].y, k[i].w);
        g_have = 0;
        (void)obstacle_create_objects();
        tkshim_set_idle_hook(watch_profile);
        tkshim_set_time_budget_ms(10000U);
        tkshim_run(obstacle_task);

        check(g_have, "a profile was published");
        printf("  box %3.0f mm at x %+4.0f y %3.0f: centre %+3d deg (true %+5.1f),"
               " width %3u mm, nearest %3u mm\n", k[i].w, k[i].x, k[i].y,
               g_prof.bearing_deg, true_b, (unsigned)g_prof.width_mm,
               (unsigned)g_prof.nearest_mm);
        sum_b += fabs((double)g_prof.bearing_deg - true_b);
        sum_w += fabs((double)g_prof.width_mm - k[i].w);
        sum_r += fabs((double)g_prof.nearest_mm - k[i].y);
    }

    printf("  MEAN ERROR  centre %.1f deg, width %.0f mm, range %.0f mm\n",
           sum_b / 3.0, sum_w / 3.0, sum_r / 3.0);

    if (S5_CAL_DONE != 0)
    {
        check(sum_b / 3.0 <= 2.0, "calibrated: centre within 2 deg");
        check(sum_r / 3.0 <= 10.0, "calibrated: range within NFR15 10 mm");
    }

    printf("  %d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main(int argc, char **argv)
{
    if ((argc > 1) && (strcmp(argv[1], "cal") == 0))
    {
        return run_cal();
    }

    return run_centre();
}
