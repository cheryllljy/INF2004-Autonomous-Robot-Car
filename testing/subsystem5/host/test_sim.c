/**
 * @file    test_sim.c
 * @brief   HOST TEST. Runs the real obstacle_task.c and the real
 *          geometry against a simulated sensor and a fake car, on a
 *          virtual clock.
 *
 * The idle hook plays VehicleController: it stops the car on DETECTED,
 * drives round the object on PROFILED, and calls obstacle_resume().
 * Each scenario runs in its own process so static state starts clean.
 *
 *   1  clear track                     7  sensor never responds
 *   2  box on the line                 8  car never stops -> STOP_TIMEOUT
 *   3  box + wall close on the left    9  boxed in -> 3rd REVERSE = FAULT
 *   4  ECHO stuck high                10  no resume -> ASSIST_TIMEOUT
 *   5  narrow post, +/-8 mm noise     11  no odometry registered
 *   6  W D  width calibration         12  sensor fault, then recovers
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "subsystem5.h"
#include "s5_config.h"
#include "sim_hal.h"

/*=====================================================================*/
/* Harness                                                             */
/*=====================================================================*/

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

static void check_le(uint32_t got, uint32_t limit, const char *what)
{
    g_checks++;
    if (got > limit)
    {
        g_failures++;
        printf("  FAIL: %s (%lu > %lu)\n", what, (unsigned long)got,
               (unsigned long)limit);
    }
}

static const char *state_name(obstacle_state_t s)
{
    static const char *names[OBSTACLE_ST_COUNT] =
    {
        "INIT", "GUARD", "CONFIRM", "WAIT_STOP", "COARSE", "FINE",
        "PUBLISH", "ASSIST", "FAULT"
    };

    return ((unsigned)s < OBSTACLE_ST_COUNT) ? names[s] : "?";
}

static const char *action_name(obstacle_action_t a)
{
    static const char *names[OBSTACLE_ACT_COUNT] =
    {
        "CONTINUE", "SLOW", "STOP", "BYPASS_LEFT", "BYPASS_RIGHT", "REVERSE"
    };

    return ((unsigned)a < OBSTACLE_ACT_COUNT) ? names[a] : "?";
}

/*=====================================================================*/
/* Scenario state                                                      */
/*=====================================================================*/

#define CRUISE_MM_S         (300)
#define STOPPING_MS         (200U)  /* fake car: DETECTED -> speed 0 */
#define BYPASS_MS           (800U)  /* fake car: time to get past it */
#define ESTOP_BIT           (0x0004U)

typedef struct
{
    int              scenario;
    uint32_t         stop_at_ms;
    uint32_t         box_appears_ms;
    double           cal_w;
    double           cal_d;

    obstacle_state_t last_state;
    uint32_t         first_detect_ms;   /* first real range seen       */
    uint32_t         detected_evt_ms;
    uint32_t         car_stopped_ms;
    uint32_t         first_move_ms;     /* first servo move after DETECTED */
    uint32_t         coarse_ms;
    uint32_t         scan_end_ms;
    uint32_t         profiled_ms;
    uint32_t         fault_ms;
    obstacle_action_t last_action;
    int              profiles;
    int              resumes;
    int              never_resume;
    int              never_stop;
    int              keep_box;
    int              saw_clear;
    int              saw_unknown;
    int              estop_cleared_by_s5;
    uint32_t         moves_at_detect;
    uint32_t         fault_until_ms;
    uint32_t         m_detect;          /* first cycle, kept after resume */
    uint32_t         m_stopped;
    uint32_t         m_move;
    uint32_t         recovered_ms;
    ID               estop_flg;
} scen_t;

static scen_t g_s;

/** The fake VehicleController + observer. Called every simulated ms.
 *  Must never call anything that advances the clock. */
static void scenario_tick(void)
{
    const uint32_t    now = tkshim_now_ms();
    obstacle_health_t h;
    sim_counters_t    c;
    uint16_t          front;
    UINT              ptn;

    obstacle_get_health(&h);
    sim_counters_get(&c);
    front = obstacle_get_front_mm();

    if (h.state != g_s.last_state)
    {
        printf("  %6lu ms  %-9s -> %-9s\n", (unsigned long)now,
               state_name(g_s.last_state), state_name(h.state));
        g_s.last_state = h.state;

        if ((h.state == OBSTACLE_ST_COARSE) && (g_s.coarse_ms == 0U))
        {
            g_s.coarse_ms = now;
        }
        if ((h.state == OBSTACLE_ST_FAULT) && (g_s.fault_ms == 0U))
        {
            g_s.fault_ms = now;
        }
    }

    if (front == OBSTACLE_RANGE_CLEAR)          { g_s.saw_clear = 1; }
    else if (front == OBSTACLE_RANGE_UNKNOWN)   { g_s.saw_unknown = 1; }
    else if (g_s.first_detect_ms == 0U)         { g_s.first_detect_ms = now; }
    else                                        { /* recorded */ }

    if ((g_s.detected_evt_ms != 0U) && (g_s.first_move_ms == 0U) &&
        (c.servo_moves > g_s.moves_at_detect))
    {
        g_s.first_move_ms = now;
    }

    if ((g_s.estop_flg > 0) && (g_s.fault_ms != 0U) &&
        ((tkshim_flag_pattern(g_s.estop_flg) & ESTOP_BIT) == 0U) &&
        (h.fault == OBSTACLE_FAULT_SENSOR))
    {
        g_s.estop_cleared_by_s5 = 1;
    }

    /* --- consume S5 events like VehicleController would ---------- */
    if (tk_wai_flg(flg_obstacle, S5_EVT_ALL, TWF_ORW | TWF_BITCLR,
                   &ptn, TMO_POL) == E_OK)
    {
        if ((ptn & S5_EVT_DETECTED) != 0U)
        {
            g_s.detected_evt_ms = now;
            g_s.moves_at_detect = c.servo_moves;
        }

        if ((ptn & (S5_EVT_PROFILED | S5_EVT_CLEARED)) != 0U)
        {
            obstacle_profile_t p;

            if (obstacle_get_profile(&p))
            {
                if (g_s.profiles == 0)
                {
                    g_s.m_detect  = g_s.detected_evt_ms;
                    g_s.m_stopped = g_s.car_stopped_ms;
                    g_s.m_move    = g_s.first_move_ms;
                }
                g_s.profiles++;
                g_s.last_action = p.action;
                g_s.profiled_ms = now;
                if (g_s.scan_end_ms == 0U) { g_s.scan_end_ms = now; }
            }
        }
    }

    /* --- the car ---------------------------------------------------- */
    if ((g_s.detected_evt_ms != 0U) && !g_s.never_stop &&
        (g_s.car_stopped_ms == 0U) &&
        ((now - g_s.detected_evt_ms) >= STOPPING_MS))
    {
        sim_set_speed_mm_s(0);
        g_s.car_stopped_ms = now;
    }

    if ((h.state == OBSTACLE_ST_ASSIST) && !g_s.never_resume &&
        (g_s.profiled_ms != 0U) && ((now - g_s.profiled_ms) == BYPASS_MS))
    {
        if (!g_s.keep_box)
        {
            sim_world_clear();          /* driven past it */
        }
        obstacle_resume();
        g_s.resumes++;
        g_s.detected_evt_ms = 0U;       /* ready for the next one */
        g_s.car_stopped_ms  = 0U;
        g_s.first_move_ms   = 0U;
        sim_set_speed_mm_s(CRUISE_MM_S);
    }

    /* --- the world -------------------------------------------------- */
    if ((g_s.box_appears_ms != 0U) && (now == g_s.box_appears_ms))
    {
        switch (g_s.scenario)
        {
            case 3:
                sim_add_box(0.0, 250.0, 150.0);
                sim_add_box(330.0, 480.0, 400.0);       /* wall, left */
                break;
            case 5:
                sim_add_box(0.0, 260.0, 45.0);
                break;
            case 6:
                sim_add_box(0.0, g_s.cal_d, g_s.cal_w);
                break;
            case 9:
                sim_add_box(0.0, 90.0, 300.0);          /* boxed in */
                break;
            default:
                sim_add_box(0.0, 250.0, 150.0);
                break;
        }
    }

    if ((g_s.fault_until_ms != 0U) && (now == g_s.fault_until_ms))
    {
        sim_set_fault(0);
    }

    if ((g_s.fault_ms != 0U) && (g_s.recovered_ms == 0U) &&
        (h.state == OBSTACLE_ST_GUARD))
    {
        g_s.recovered_ms = now;
    }

    if (now >= g_s.stop_at_ms)
    {
        tkshim_stop();
    }
}

static void run(void)
{
    tkshim_set_idle_hook(scenario_tick);
    tkshim_run(obstacle_task);
}

static void report_profile(void)
{
    obstacle_profile_t p;

    if (obstacle_get_profile(&p) && p.valid)
    {
        printf("  profile: nearest %u mm, bearing %+d, width %u mm,"
               " gapL %u gapR %u, swingL %u swingR %u, conf %u%%,"
               " pass after %u mm\n",
               (unsigned)p.nearest_mm, p.bearing_deg, (unsigned)p.width_mm,
               (unsigned)p.gap_left_mm, (unsigned)p.gap_right_mm,
               (unsigned)p.swing_left_mm, (unsigned)p.swing_right_mm,
               (unsigned)p.confidence_pct,
               (unsigned)obstacle_pass_distance_mm(&p));
        printf("  action : %s\n", action_name(p.action));
    }
    else
    {
        puts("  profile: none valid");
    }
}

/*=====================================================================*/
/* Scenarios                                                           */
/*=====================================================================*/

static void s1_clear(void)
{
    obstacle_health_t h;

    puts("S1 clear track for 1.5 s");
    g_s.stop_at_ms = 1500U;
    run();
    obstacle_get_health(&h);

    check(g_s.saw_clear, "front reported CLEAR on an empty track");
    check(h.scans_completed == 0U, "no sweep with nothing there");
    check(h.state == OBSTACLE_ST_GUARD, "stayed in GUARD");
    check(h.deadline_overruns == 0U,
          "60 ms guard period holds with the 20 ms read window");
}

static void s2_box(void)
{
    obstacle_health_t h;
    obstacle_profile_t p;

    puts("S2 150 mm box appears 250 mm ahead at 500 ms");
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;
    run();
    obstacle_get_health(&h);
    report_profile();

    printf("  detect %lu ms after the box, DETECTED->stopped %lu ms,"
           " stopped->first servo move %lu ms, scan %lu ms\n",
           (unsigned long)(g_s.first_detect_ms - g_s.box_appears_ms),
           (unsigned long)(g_s.m_stopped - g_s.m_detect),
           (unsigned long)(g_s.m_move - g_s.m_stopped),
           (unsigned long)(g_s.scan_end_ms - g_s.coarse_ms));

    check_le(g_s.first_detect_ms - g_s.box_appears_ms, 200U,
             "detection latency under 200 ms");
    check((g_s.m_stopped != 0U) &&
          (g_s.m_move >= g_s.m_stopped + S5_STOPPED_HOLD_MS),
          "servo does not move until the car has been still 100 ms");
    check_le(g_s.scan_end_ms - g_s.coarse_ms, 1800U,
             "scan inside NFR11 1.8 s");
    check(h.scans_completed >= 1U, "a sweep completed");
    check(h.odometry_ok, "WaitStopped used odometry");
    check(g_s.resumes >= 1, "controller resumed after the bypass");
    check(obstacle_get_profile(&p) && p.valid, "profile is valid");
    check((g_s.last_action == OBSTACLE_ACT_BYPASS_LEFT) ||
          (g_s.last_action == OBSTACLE_ACT_BYPASS_RIGHT), "chose a side");
    check(h.state == OBSTACLE_ST_GUARD, "back in GUARD after resume");
}

static void s3_wall(void)
{
    obstacle_profile_t p;

    puts("S3 box on the line, wall close on the left");
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;
    run();
    report_profile();

    check(obstacle_get_profile(&p) && p.valid, "profile is valid");
    check(p.gap_left_mm < p.gap_right_mm, "left corridor is tighter");
    check(g_s.last_action == OBSTACLE_ACT_BYPASS_RIGHT, "goes right");
}

static void s4_stuck(void)
{
    obstacle_health_t h;

    puts("S4 ECHO stuck high from the start");
    sim_set_fault(1);
    g_s.stop_at_ms = 4000U;
    run();
    obstacle_get_health(&h);

    check(!g_s.saw_clear, "a stuck sensor is never read as clear");
    check(h.state == OBSTACLE_ST_FAULT, "FAULT");
    check(h.fault == OBSTACLE_FAULT_SENSOR, "cause SENSOR");
    check((tkshim_flag_pattern(g_s.estop_flg) & ESTOP_BIT) != 0U,
          "E-stop cause bit set");
    check(h.scans_completed == 0U, "no sweep on bad data");
}

static void s5_noise(void)
{
    obstacle_profile_t p;

    puts("S5 narrow post at 260 mm with +/-8 mm noise");
    sim_set_noise_mm(8);
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;
    run();
    report_profile();

    check(obstacle_get_profile(&p) && p.valid, "profile is valid");
    check((p.nearest_mm > 200U) && (p.nearest_mm < 320U),
          "nearest range survived the noise");
}

static void s6_calibrate(void)
{
    obstacle_profile_t p;

    printf("S6 calibration: %4.0f mm box at %4.0f mm\n", g_s.cal_w, g_s.cal_d);
    g_s.box_appears_ms = 200U;
    g_s.stop_at_ms     = 6000U;
    run();

    if (obstacle_get_profile(&p) && p.valid && (p.nearest_mm < 3000U))
    {
        const long raw =
            (long)p.width_mm + (((long)p.nearest_mm * 270L) / 1024L);
        const long ideal = ((raw - (long)g_s.cal_w) * 1024L) /
                           (long)p.nearest_mm;

        printf("  nearest %u mm, width %u mm (true %4.0f), error %+ld mm,"
               " ideal OBS_BEAM_CORR_Q10 %ld (current 270)\n",
               (unsigned)p.nearest_mm, (unsigned)p.width_mm, g_s.cal_w,
               (long)p.width_mm - (long)g_s.cal_w, ideal);
        check(p.width_mm > 0U, "a width was produced");
    }
    else
    {
        puts("  no scan: the box must sit inside OBSTACLE_TRIGGER_MM");
        check(0, "profile produced");
    }
}

static void s7_silent(void)
{
    obstacle_health_t h;

    puts("S7 sensor never responds");
    sim_set_fault(2);
    g_s.stop_at_ms = 4000U;
    run();
    obstacle_get_health(&h);

    printf("  FAULT after %lu ms\n", (unsigned long)g_s.fault_ms);
    check(!g_s.saw_clear, "silence is never a clear path");
    check(g_s.saw_unknown, "front UNKNOWN");
    check(h.fault == OBSTACLE_FAULT_SENSOR, "SENSOR fault");
    check_le(g_s.fault_ms, 1000U, "fault within 1 s (10 x 60 ms + start)");
    check((tkshim_flag_pattern(g_s.estop_flg) & ESTOP_BIT) != 0U,
          "E-stop cause bit set");
}

static void s8_never_stops(void)
{
    obstacle_health_t h;

    puts("S8 car never stops after DETECTED");
    g_s.never_stop     = 1;
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 4000U;
    run();
    obstacle_get_health(&h);

    printf("  DETECTED %lu ms, FAULT %lu ms\n",
           (unsigned long)g_s.detected_evt_ms, (unsigned long)g_s.fault_ms);
    check(h.fault == OBSTACLE_FAULT_STOP_TIMEOUT, "STOP_TIMEOUT fault");
    check_le(g_s.fault_ms - g_s.detected_evt_ms, 1100U, "within ~1 s");
    check(g_s.first_move_ms == 0U, "servo never swept while moving");
    check(h.scans_completed == 0U, "no scan while moving");
    check((tkshim_flag_pattern(g_s.estop_flg) & ESTOP_BIT) == 0U,
          "mission fault does NOT use the E-stop");
}

static void s9_boxed_in(void)
{
    obstacle_health_t h;

    puts("S9 object at 90 mm that never goes away");
    g_s.keep_box       = 1;
    g_s.box_appears_ms = 300U;
    g_s.stop_at_ms     = 12000U;
    run();
    obstacle_get_health(&h);

    printf("  %d profiles, %d resumes, last action %s\n", g_s.profiles,
           g_s.resumes, action_name(g_s.last_action));
    check(g_s.resumes == 2, "two REVERSE manoeuvres allowed");
    check(h.fault == OBSTACLE_FAULT_REVERSE_LIMIT, "third one -> FAULT");
    {
        obstacle_profile_t p;

        check(obstacle_get_profile(&p) && (p.action == OBSTACLE_ACT_STOP),
              "third profile says STOP, not a third REVERSE");
    }
}

static void s10_no_resume(void)
{
    obstacle_health_t h;

    puts("S10 controller never calls obstacle_resume()");
    g_s.never_resume   = 1;
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 10000U;
    run();
    obstacle_get_health(&h);

    printf("  PROFILED %lu ms, FAULT %lu ms\n",
           (unsigned long)g_s.profiled_ms, (unsigned long)g_s.fault_ms);
    check(h.fault == OBSTACLE_FAULT_ASSIST_TIMEOUT, "ASSIST_TIMEOUT");
    check_le(g_s.fault_ms - g_s.profiled_ms, 5200U, "after about 5 s");
}

static void s11_no_odometry(void)
{
    obstacle_health_t h;

    puts("S11 no odometry registered (bench)");
    obstacle_register_odometry(NULL);
    g_s.box_appears_ms = 500U;
    g_s.stop_at_ms     = 6000U;
    run();
    obstacle_get_health(&h);

    check(h.scans_completed >= 1U, "still scans after the fixed wait");
    check(!h.odometry_ok, "health shows the stop was not verified");
}

static void s12_recovers(void)
{
    obstacle_health_t h;

    puts("S12 sensor silent until 1.5 s, then healthy");
    sim_set_fault(2);
    g_s.fault_until_ms = 1500U;
    g_s.stop_at_ms     = 5000U;
    run();
    obstacle_get_health(&h);

    printf("  FAULT %lu ms, back in GUARD %lu ms\n",
           (unsigned long)g_s.fault_ms, (unsigned long)g_s.recovered_ms);
    check(g_s.fault_ms != 0U, "faulted while silent");
    check(g_s.recovered_ms > g_s.fault_until_ms,
          "back to GUARD after 3 good pings");
    check(h.state == OBSTACLE_ST_GUARD, "guarding again");
    check(obstacle_get_front_mm() == OBSTACLE_RANGE_CLEAR,
          "front range trusted again");
    check((tkshim_flag_pattern(g_s.estop_flg) & ESTOP_BIT) != 0U,
          "E-stop stays latched for VehicleController to clear");
    check(!g_s.estop_cleared_by_s5, "S5 never clears the E-stop itself");
}

/*=====================================================================*/

int main(int argc, char **argv)
{
    const int which = (argc > 1) ? atoi(argv[1]) : 1;
    T_CFLG    cflg  = { NULL, TA_WMUL, 0U };

    memset(&g_s, 0, sizeof(g_s));
    g_s.scenario   = which;
    g_s.last_state = OBSTACLE_ST_INIT;
    g_s.cal_w      = (argc > 2) ? atof(argv[2]) : 150.0;
    g_s.cal_d      = (argc > 3) ? atof(argv[3]) : 250.0;

    tkshim_reset();
    tkshim_set_time_budget_ms(30000U);
    sim_world_clear();
    sim_counters_reset();
    sim_set_speed_mm_s(CRUISE_MM_S);

    /* What other subsystems do in their _init(): E-stop owner and
     * odometry provider register with S5. */
    g_s.estop_flg = tk_cre_flg(&cflg);
    obstacle_register_estop(g_s.estop_flg, ESTOP_BIT);
    obstacle_register_odometry(sim_odometry);

    if (obstacle_create_objects() != E_OK)
    {
        puts("object creation failed");
        return EXIT_FAILURE;
    }

    switch (which)
    {
        case 1:  s1_clear();        break;
        case 2:  s2_box();          break;
        case 3:  s3_wall();         break;
        case 4:  s4_stuck();        break;
        case 5:  s5_noise();        break;
        case 6:  s6_calibrate();    break;
        case 7:  s7_silent();       break;
        case 8:  s8_never_stops();  break;
        case 9:  s9_boxed_in();     break;
        case 10: s10_no_resume();   break;
        case 11: s11_no_odometry(); break;
        case 12: s12_recovers();    break;
        default:
            puts("usage: test_sim <1..12> | 6 <width_mm> <dist_mm>");
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
