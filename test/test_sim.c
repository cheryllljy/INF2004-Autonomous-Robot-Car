/**
 * @file    test_sim.c
 * @brief   HOST TEST. Runs the real obstacle task and the real
 *          geometry against a simulated sensor on a virtual clock.
 *
 * What this catches that the geometry tests cannot:
 *   - state machine transitions and the order they happen in
 *   - the guard period against the sensor's real quiet time, which is
 *     where a scan rate that looks fine on paper turns into a missed
 *     deadline
 *   - the wall time of a complete scan, measured rather than estimated
 *   - fault handling when the sensor stops answering
 *
 * Each scenario runs in its own process so the module's static state
 * always starts clean:
 *
 *   ./build/test_sim 1     clear track
 *   ./build/test_sim 2     box on the line
 *   ./build/test_sim 3     box with a wall close on the left
 *   ./build/test_sim 4     sensor unplugged
 *   ./build/test_sim 5     narrow post with ranging noise
 *   ./build/test_sim 6 W D width calibration, box W mm at D mm
 *   ./build/test_sim 7     sensor never responds (dead IRQ / swap)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tk/tkernel.h>

#include "obstacle.h"
#include "sim_hal.h"

/*=====================================================================*/
/* Tiny assertion harness                                              */
/*=====================================================================*/

static int g_checks   = 0;
static int g_failures = 0;

static void check(int condition, const char *what)
{
    g_checks++;

    if (!condition)
    {
        g_failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_le(uint32_t got, uint32_t limit, const char *what)
{
    g_checks++;

    if (got > limit)
    {
        g_failures++;
        printf("  FAIL: %s (%lu > limit %lu)\n",
               what, (unsigned long)got, (unsigned long)limit);
    }
}

static const char *action_name(obstacle_action_t a)
{
    const char *name;

    switch (a)
    {
        case OBSTACLE_ACT_CONTINUE:     name = "CONTINUE";     break;
        case OBSTACLE_ACT_SLOW:         name = "SLOW";         break;
        case OBSTACLE_ACT_STOP:         name = "STOP";         break;
        case OBSTACLE_ACT_BYPASS_LEFT:  name = "BYPASS_LEFT";  break;
        case OBSTACLE_ACT_BYPASS_RIGHT: name = "BYPASS_RIGHT"; break;
        default:                        name = "REVERSE";      break;
    }

    return name;
}

static const char *state_name(obstacle_state_t s)
{
    const char *name;

    switch (s)
    {
        case OBSTACLE_ST_INIT:    name = "INIT";    break;
        case OBSTACLE_ST_GUARD:   name = "GUARD";   break;
        case OBSTACLE_ST_CONFIRM: name = "CONFIRM"; break;
        case OBSTACLE_ST_COARSE:  name = "COARSE";  break;
        case OBSTACLE_ST_FINE:    name = "FINE";    break;
        case OBSTACLE_ST_PUBLISH: name = "PUBLISH"; break;
        case OBSTACLE_ST_ASSIST:  name = "ASSIST";  break;
        default:                  name = "FAULT";   break;
    }

    return name;
}

/*=====================================================================*/
/* Scenario state, driven from the shim idle hook                      */
/*=====================================================================*/

typedef struct
{
    uint32_t         scan_start_ms;
    uint32_t         scan_end_ms;
    uint32_t         assist_entry_ms;
    uint32_t         first_detect_ms;
    uint32_t         box_appears_ms;
    int              resumed;
    int              saw_clear;
    int              saw_unknown;
    obstacle_state_t last_state;
    uint32_t         stop_at_ms;
    int              scenario;
    double           cal_width_mm;
    double           cal_dist_mm;
} scen_t;

static scen_t g_s;

/**
 * @brief   Stands in for the vehicle controller and the test observer.
 *
 * Called once per simulated millisecond. It must not call anything
 * that advances the virtual clock, or time would run away.
 */
static void scenario_tick(void)
{
    const uint32_t    now = tkshim_now_ms();
    obstacle_health_t h;
    uint16_t          front;

    obstacle_get_health(&h);
    front = obstacle_get_front_mm();

    if (h.state != g_s.last_state)
    {
        printf("  %6lu ms  %-8s -> %-8s\n",
               (unsigned long)now,
               state_name(g_s.last_state), state_name(h.state));
        g_s.last_state = h.state;
    }

    if (front == OBSTACLE_RANGE_CLEAR)
    {
        g_s.saw_clear = 1;
    }
    else if (front == OBSTACLE_RANGE_UNKNOWN)
    {
        g_s.saw_unknown = 1;
    }
    else if (g_s.first_detect_ms == 0U)
    {
        g_s.first_detect_ms = now;
    }
    else
    {
        /* Already recorded. */
    }

    if ((h.state == OBSTACLE_ST_COARSE) && (g_s.scan_start_ms == 0U))
    {
        g_s.scan_start_ms = now;
    }

    if ((h.scans_completed > 0U) && (g_s.scan_end_ms == 0U))
    {
        g_s.scan_end_ms = now;
    }

    /* Play the vehicle controller: hold in ASSIST for 800 ms, as if
     * steering round the object, then report the line reacquired. */
    if (h.state == OBSTACLE_ST_ASSIST)
    {
        if (g_s.assist_entry_ms == 0U)
        {
            g_s.assist_entry_ms = now;
        }
        else if (!g_s.resumed && ((now - g_s.assist_entry_ms) >= 800U))
        {
            sim_world_clear();      /* the car is now past the object */
            obstacle_resume();
            g_s.resumed = 1;
        }
        else
        {
            /* Still manoeuvring. */
        }
    }

    /* Scenario 2 and 3 drop the object in after half a second so the
     * guard state gets exercised on an empty track first. */
    if ((g_s.box_appears_ms != 0U) && (now == g_s.box_appears_ms))
    {
        if (g_s.scenario == 2)
        {
            sim_add_box(0.0, 250.0, 150.0);
        }
        else if (g_s.scenario == 3)
        {
            sim_add_box(0.0, 250.0, 150.0);
            sim_add_box(330.0, 480.0, 400.0);   /* wall on the left */
        }
        else if (g_s.scenario == 5)
        {
            sim_add_box(0.0, 260.0, 45.0);      /* narrow post */
        }
        else if (g_s.scenario == 6)
        {
            sim_add_box(0.0, g_s.cal_dist_mm, g_s.cal_width_mm);
        }
        else
        {
            /* Nothing to add. */
        }
    }

    if (now >= g_s.stop_at_ms)
    {
        tkshim_stop();
    }
}

/*=====================================================================*/
/* Scenarios                                                           */
/*=====================================================================*/

static void report_profile(void)
{
    obstacle_profile_t p;

    if (obstacle_get_profile(&p) && p.valid)
    {
        printf("  profile: nearest %u mm, bearing %+d deg, width %u mm,"
               " gapL %u gapR %u, swingL %u swingR %u, conf %u%%\n",
               (unsigned)p.nearest_mm, p.bearing_deg,
               (unsigned)p.width_mm,
               (unsigned)p.gap_left_mm, (unsigned)p.gap_right_mm,
               (unsigned)p.swing_left_mm, (unsigned)p.swing_right_mm,
               (unsigned)p.confidence_pct);
        printf("  action : %s\n", action_name(p.action));
    }
    else
    {
        puts("  profile: none published");
    }
}

static void scenario_1_clear(void)
{
    obstacle_health_t h;

    puts("S1 clear track, nothing in front for 1.5 s");

    sim_world_clear();
    g_s.stop_at_ms = 1500U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    obstacle_get_health(&h);

    printf("  guard pings drove %lu scans, %lu overruns\n",
           (unsigned long)h.scans_completed,
           (unsigned long)h.deadline_overruns);

    check(g_s.saw_clear, "front reported CLEAR on an empty track");
    check(h.scans_completed == 0U, "no sweep launched with nothing there");
    check(h.state == OBSTACLE_ST_GUARD, "stayed in GUARD");
    check(h.deadline_overruns == 0U,
          "guard period survives the sensor quiet time plus timeout");
}

static void scenario_2_box(void)
{
    obstacle_health_t h;
    uint32_t          scan_ms;

    puts("S2 150 mm box appears on the line at 250 mm");

    sim_world_clear();
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    obstacle_get_health(&h);
    scan_ms = g_s.scan_end_ms - g_s.scan_start_ms;

    printf("  detected at %lu ms, scan took %lu ms\n",
           (unsigned long)g_s.first_detect_ms, (unsigned long)scan_ms);
    report_profile();

    check(h.scans_completed >= 1U, "a sweep completed");
    check(g_s.resumed, "reached ASSIST and the controller resumed it");
    check_le(g_s.first_detect_ms - g_s.box_appears_ms, 200U,
             "detection latency inside 200 ms");
    check_le(scan_ms, 2000U, "scan wall time inside the 2 s budget");

    {
        obstacle_profile_t p;

        check(obstacle_get_profile(&p) && p.valid, "profile is valid");
        check((p.action == OBSTACLE_ACT_BYPASS_LEFT) ||
              (p.action == OBSTACLE_ACT_BYPASS_RIGHT),
              "chose a side to pass on");
    }
}

static void scenario_3_wall(void)
{
    obstacle_profile_t p;

    puts("S3 box on the line with a wall close on the left");

    sim_world_clear();
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    report_profile();

    check(obstacle_get_profile(&p) && p.valid, "profile is valid");
    check(p.gap_left_mm < p.gap_right_mm, "left corridor is tighter");
    check(p.action == OBSTACLE_ACT_BYPASS_RIGHT, "goes right");
}

static void scenario_4_fault(void)
{
    obstacle_health_t h;

    puts("S4 sensor unplugged from the start");

    sim_world_clear();
    sim_set_fault(1);
    g_s.stop_at_ms = 4000U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    obstacle_get_health(&h);

    printf("  ended in %s after %lu fault reports\n",
           state_name(h.state), (unsigned long)h.sensor_faults);

    check(g_s.saw_unknown, "front reported UNKNOWN, never CLEAR");
    check(!g_s.saw_clear, "a dead sensor is never read as a clear path");
    check(h.state == OBSTACLE_ST_FAULT, "entered the fault state");
    check(h.scans_completed == 0U, "no sweep attempted on bad data");
}

static void scenario_5_noise(void)
{
    obstacle_profile_t p;
    obstacle_health_t  h;

    puts("S5 narrow post at 260 mm with +/- 8 mm of ranging noise");

    sim_world_clear();
    sim_set_noise_mm(8);
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    obstacle_get_health(&h);
    report_profile();

    check(h.scans_completed >= 1U, "noise did not prevent a sweep");
    check(obstacle_get_profile(&p) && p.valid, "profile is valid");
    check(p.nearest_mm > 200U && p.nearest_mm < 320U,
          "nearest range survived the noise");
}

/*=====================================================================*/

/*---------------------------------------------------------------------
 * S6: width calibration sweep. Run one box of known size and print
 *     what the profiler reports, plus the beam-correction constant
 *     that would have made it exact.
 *
 * This is bench test G3 done in simulation. Run the same rows against
 * real boxes later and compare; if the two disagree, trust the bench
 * and retune OBS_BEAM_CORR_Q10 in src/obstacle_geometry.c.
 *-------------------------------------------------------------------*/
static void scenario_6_calibrate(void)
{
    obstacle_profile_t p;

    printf("S6 calibration: %4.0f mm box at %4.0f mm\n",
           g_s.cal_width_mm, g_s.cal_dist_mm);

    sim_world_clear();
    g_s.box_appears_ms = 200U;
    g_s.stop_at_ms     = 6000U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    if (obstacle_get_profile(&p) && p.valid &&
        (p.nearest_mm < 3000U))
    {
        /* reported = raw - (nearest * Q10 / 1024), so recover raw and
         * solve for the Q10 that would give the true width. */
        const long raw =
            (long)p.width_mm + (((long)p.nearest_mm * 270L) / 1024L);
        const long ideal_q10 =
            ((raw - (long)g_s.cal_width_mm) * 1024L) / (long)p.nearest_mm;

        printf("  nearest %u mm, reported width %u mm, true %4.0f mm,"
               " error %+ld mm\n",
               (unsigned)p.nearest_mm, (unsigned)p.width_mm,
               g_s.cal_width_mm,
               (long)p.width_mm - (long)g_s.cal_width_mm);
        printf("  raw extent %ld mm -> ideal OBS_BEAM_CORR_Q10 = %ld"
               " (current 270)\n", raw, ideal_q10);

        check(p.width_mm > 0U, "a width was produced");
    }
    else
    {
        puts("  no scan was launched: the box must sit inside"
             " OBSTACLE_TRIGGER_MM");
        check(0, "profile produced");
    }
}

/*---------------------------------------------------------------------
 * S7: the sensor never responds. ECHO stays low, exactly as it did on
 *     the bench when the bank interrupt was not enabled, or as it
 *     would with TRIG/ECHO swapped. A silent sensor must end up as a
 *     FAULT with the range UNKNOWN, never as a clear track.
 *-------------------------------------------------------------------*/
static void scenario_7_silent(void)
{
    obstacle_health_t h;

    puts("S7 sensor never responds (dead interrupt or swapped wires)");

    sim_world_clear();
    sim_set_fault(2);
    g_s.stop_at_ms = 4000U;

    check(obstacle_init(), "obstacle_init succeeded");
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run();

    obstacle_get_health(&h);

    printf("  ended in %s after %lu fault reports\n",
           state_name(h.state), (unsigned long)h.sensor_faults);

    check(!g_s.saw_clear, "silence is never reported as a clear path");
    check(g_s.saw_unknown, "front reported UNKNOWN");
    check(h.state == OBSTACLE_ST_FAULT, "entered the fault state");
    check(h.scans_completed == 0U, "no sweep on a silent sensor");
}

int main(int argc, char **argv)
{
    const int which = (argc > 1) ? atoi(argv[1]) : 1;

    memset(&g_s, 0, sizeof(g_s));
    g_s.last_state = OBSTACLE_ST_INIT;
    g_s.scenario   = which;
    g_s.stop_at_ms   = 5000U;
    g_s.cal_width_mm = (argc > 2) ? atof(argv[2]) : 150.0;
    g_s.cal_dist_mm  = (argc > 3) ? atof(argv[3]) : 250.0;

    sim_counters_reset();
    tkshim_set_time_budget_ms(20000U);

    switch (which)
    {
        case 1:  scenario_1_clear(); break;
        case 2:  scenario_2_box();   break;
        case 3:  scenario_3_wall();  break;
        case 4:  scenario_4_fault(); break;
        case 5:  scenario_5_noise(); break;
        case 6:  scenario_6_calibrate(); break;
        case 7:  scenario_7_silent(); break;
        default:
            puts("usage: test_sim <1..5|7> | 6 <width_mm> <dist_mm>");
            return EXIT_FAILURE;
    }

    {
        sim_counters_t c;

        sim_counters_get(&c);
        printf("  %lu pings, %lu servo moves, %lu ms simulated\n",
               (unsigned long)c.pings, (unsigned long)c.servo_moves,
               (unsigned long)tkshim_now_ms());
    }

    printf("  %d checks, %d failures\n\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
