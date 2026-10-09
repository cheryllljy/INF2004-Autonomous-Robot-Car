/**
 * @file    main.c
 * @brief   Bench bring-up firmware for the Buddy 5 obstacle subsystem.
 *
 * Two roles, chosen from the USB serial menu at runtime:
 *
 *   SELF TEST  drive the servo and the ranger directly and print
 *              numbers. Every command maps to a test ID in the
 *              module README, so the output goes straight into the
 *              report as evidence.
 *
 *   MISSION    run obstacle_init() and let the real state machine
 *              drive, printing the published range and health while
 *              it works.
 *
 * printf over USB CDC can block for tens of milliseconds. That is
 * acceptable here because nothing else is running, and it is exactly
 * why the mission build for the graded demonstration must define
 * BRINGUP_SILENT and let Buddy 1 publish telemetry over MQTT instead.
 */

#include <stdio.h>
#include <string.h>

#include <tk/tkernel.h>

#include "pico/stdlib.h"

#include "board_config.h"
#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"
#include "wifi.h"
#include "mqtt.h"

/*=====================================================================*/
/* Local helpers                                                       */
/*=====================================================================*/

#define SAMPLE_BURST        (50U)
#define SWEEP_POINTS        (11U)
#define SWEEP_STEP_DEG      (15)

/*=====================================================================*/
/* Buddy 1 - WiFi bench test configuration                             */
/*=====================================================================*/

#define WIFI_TEST_SSID      "Chloeee"
#define WIFI_TEST_PASSWORD  "Chloeee27~21"

static bool g_drivers_up = false;
static bool g_mission_up = false;

/**
 * @brief   Integer square root, for standard deviation without a FPU.
 */
static uint32_t isqrt_u32(uint32_t value)
{
    uint32_t rem = 0U;
    uint32_t root = 0U;
    int      i;

    for (i = 0; i < 16; i++)
    {
        root <<= 1;
        rem = (rem << 2) | (value >> 30);
        value <<= 2;

        if (root < rem)
        {
            root++;
            rem -= root;
            root++;
        }
    }

    return root >> 1;
}

/*=====================================================================*/
/* Buddy 1 - MQTT bench test                                           */
/*=====================================================================*/

static void test_mqtt_connection(void)
{
    int result;
    int wait_count = 0;

    puts("");
    puts("[Buddy 1] Starting MQTT test");

    result = mqtt_init();

    if (result != 0)
    {
        printf(
            "[Buddy 1] MQTT initialisation FAILED: %d\n",
            result
        );

        return;
    }

    /*
     * MQTT and DNS operate asynchronously.
     * Wait up to approximately 15 seconds for the result.
     */
    while (!mqtt_connection_complete() && wait_count < 150)
    {
        sleep_ms(100);
        wait_count++;
    }

    if (mqtt_is_connected())
    {
        puts("[Buddy 1] MQTT connection PASSED");
    }
    else
    {
        puts("[Buddy 1] MQTT connection FAILED");
    }
}

static const char *status_name(ultrasonic_status_t s)
{
    const char *name;

    switch (s)
    {
        case ULTRASONIC_OK:         name = "OK";         break;
        case ULTRASONIC_NO_ECHO:    name = "NO_ECHO";    break;
        case ULTRASONIC_TOO_CLOSE:  name = "TOO_CLOSE";  break;
        case ULTRASONIC_TOO_FAR:    name = "TOO_FAR";    break;
        case ULTRASONIC_ECHO_STUCK: name = "ECHO_STUCK"; break;
        case ULTRASONIC_NOT_READY:  name = "NOT_READY";  break;
        default:                    name = "BAD_PARAM";  break;
    }

    return name;
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

/**
 * @brief   Bring the two drivers up once.
 */
static bool drivers_up(void)
{
    if (!g_drivers_up)
    {
        const bool servo_ok = servo_init();
        const bool sonar_ok = ultrasonic_init();

        printf("servo_init  : %s\n", servo_ok ? "ok" : "FAILED");
        printf("sonar_init  : %s\n", sonar_ok ? "ok" : "FAILED");

        g_drivers_up = servo_ok && sonar_ok;
    }

    return g_drivers_up;
}

/*=====================================================================*/
/* Test S1 - servo angle accuracy                                      */
/*=====================================================================*/

static void test_servo_angles(void)
{
    static const int16_t points[5] = { 15, 45, 90, 135, 165 };
    uint8_t i;

    puts("\n[S1] Servo angle accuracy.");
    puts("Measure the horn with a protractor at each step and record");
    puts("the error. Press any key between steps.");

    for (i = 0U; i < 5U; i++)
    {
        const int16_t from = servo_get_angle();

        (void)servo_set_angle(points[i]);
        printf("  commanded %3d deg, settle %lu ms ... ",
               points[i], (unsigned long)servo_settle_ms(from, points[i]));
        (void)tk_dly_tsk((RELTIM)servo_settle_ms(from, points[i]));
        printf("measure now, press a key\n");
        (void)getchar();
    }

    (void)servo_set_angle(SERVO_ANGLE_CENTRE_DEG);
    puts("[S1] done, horn returned to centre.");
}

/*=====================================================================*/
/* Test S2 - servo travel rate                                         */
/*=====================================================================*/

static void test_servo_travel(void)
{
    uint8_t  i;
    uint32_t t0;

    puts("\n[S2] Servo travel rate.");
    puts("Ten 60 degree hops. Watch or film the horn: if it is still");
    puts("moving when the tick prints, raise SERVO_TRAVEL_MS_PER_60DEG");
    puts("in servo.c until it is not.");

    for (i = 0U; i < 10U; i++)
    {
        const int16_t target = ((i % 2U) == 0U) ? 30 : 150;
        const int16_t from   = servo_get_angle();
        const uint32_t settle = servo_settle_ms(from, target);

        t0 = tkshim_now_ms();
        (void)servo_set_angle(target);
        (void)tk_dly_tsk((RELTIM)settle);
        printf("  hop %2u: %3d -> %3d, allowed %lu ms, elapsed %lu ms\n",
               (unsigned)i, from, target,
               (unsigned long)settle,
               (unsigned long)(tkshim_now_ms() - t0));
    }

    (void)servo_set_angle(SERVO_ANGLE_CENTRE_DEG);
    puts("[S2] done.");
}

/*=====================================================================*/
/* Test U1 - ranging accuracy and repeatability                        */
/*=====================================================================*/

static void test_range_burst(void)
{
    uint32_t sum   = 0U;
    uint32_t sumsq = 0U;
    uint16_t lo    = 0xFFFFU;
    uint16_t hi    = 0U;
    uint16_t n     = 0U;
    uint16_t fails = 0U;
    uint16_t i;
    uint32_t t0;

    puts("\n[U1] Ranging burst, 50 samples straight ahead.");
    puts("Put a flat target at a measured distance first.");

    (void)servo_set_angle(SERVO_ANGLE_CENTRE_DEG);
    (void)tk_dly_tsk(300U);

    t0 = tkshim_now_ms();

    for (i = 0U; i < (uint16_t)SAMPLE_BURST; i++)
    {
        ultrasonic_result_t r;

        if (ultrasonic_ping(&r) == ULTRASONIC_OK)
        {
            sum   += r.distance_mm;
            sumsq += ((uint32_t)r.distance_mm * r.distance_mm);
            if (r.distance_mm < lo) { lo = r.distance_mm; }
            if (r.distance_mm > hi) { hi = r.distance_mm; }
            n++;
        }
        else
        {
            fails++;
        }
    }

    printf("  elapsed     : %lu ms for %u pings (%lu ms/ping)\n",
           (unsigned long)(tkshim_now_ms() - t0),
           (unsigned)SAMPLE_BURST,
           (unsigned long)((tkshim_now_ms() - t0) / SAMPLE_BURST));
    printf("  valid       : %u of %u (%u failed)\n",
           (unsigned)n, (unsigned)SAMPLE_BURST, (unsigned)fails);

    if (n >= 2U)
    {
        const uint32_t mean = sum / n;
        const uint32_t var  = (sumsq / n) - (mean * mean);

        printf("  mean        : %lu mm\n", (unsigned long)mean);
        printf("  min / max   : %u / %u mm (spread %u)\n",
               (unsigned)lo, (unsigned)hi, (unsigned)(hi - lo));
        printf("  std dev     : %lu mm\n", (unsigned long)isqrt_u32(var));
    }

    puts("[U1] done.");
}

/*=====================================================================*/
/* Test U3 / U4 - live stream, blind zone and unplug behaviour         */
/*=====================================================================*/

static void test_live_stream(void)
{
    puts("\n[U3/U4] Live stream. Walk a target in from 100 mm to");
    puts("touching to find the real blind zone, then unplug the sensor");
    puts("and confirm every ping still returns and reports a fault.");
    puts("Press any key to stop.\n");

    (void)servo_set_angle(SERVO_ANGLE_CENTRE_DEG);
    (void)tk_dly_tsk(300U);

    while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT)
    {
        ultrasonic_result_t r;
        const uint32_t      t0 = tkshim_now_ms();
        const ultrasonic_status_t s = ultrasonic_ping(&r);

        printf("  %-10s  %5u mm  echo %6lu us  call took %lu ms\n",
               status_name(s),
               (unsigned)r.distance_mm,
               (unsigned long)r.echo_us,
               (unsigned long)(tkshim_now_ms() - t0));
    }

    puts("[U3/U4] done.");
}

/*=====================================================================*/
/* Sweep and profile - exercises the geometry with real sensor data    */
/*=====================================================================*/

static void test_sweep_profile(void)
{
    static obstacle_sample_t samples[SWEEP_POINTS];
    obstacle_profile_t       profile;
    uint8_t                  i;
    uint32_t                 t0;

    puts("\n[E1] Sweep and profile, 11 points at 15 degrees.");

    t0 = tkshim_now_ms();

    for (i = 0U; i < (uint8_t)SWEEP_POINTS; i++)
    {
        const int16_t bearing =
            (int16_t)(-75 + ((int16_t)i * SWEEP_STEP_DEG));
        const int16_t target  =
            (int16_t)(SERVO_ANGLE_CENTRE_DEG + bearing);
        const int16_t from    = servo_get_angle();
        ultrasonic_result_t r;
        ultrasonic_status_t s;

        (void)servo_set_angle(target);
        (void)tk_dly_tsk((RELTIM)servo_settle_ms(from, target));

        s = ultrasonic_ping(&r);

        samples[i].bearing_deg = bearing;

        if (s == ULTRASONIC_OK)
        {
            samples[i].range_mm = r.distance_mm;
        }
        else if (s == ULTRASONIC_TOO_CLOSE)
        {
            samples[i].range_mm = (uint16_t)ULTRASONIC_MIN_VALID_MM;
        }
        else if ((s == ULTRASONIC_NO_ECHO) || (s == ULTRASONIC_TOO_FAR))
        {
            samples[i].range_mm = OBSTACLE_RANGE_CLEAR;
        }
        else
        {
            samples[i].range_mm = OBSTACLE_RANGE_UNKNOWN;
        }

        printf("  %+4d deg : %-10s %5u mm\n",
               bearing, status_name(s), (unsigned)samples[i].range_mm);
    }

    printf("  sweep wall time: %lu ms\n",
           (unsigned long)(tkshim_now_ms() - t0));

    (void)servo_set_angle(SERVO_ANGLE_CENTRE_DEG);

    obstacle_profile_build(samples, (uint8_t)SWEEP_POINTS, &profile);

    puts("  --- profile ---");
    printf("  valid        : %s\n", profile.valid ? "yes" : "no");
    printf("  nearest      : %u mm\n", (unsigned)profile.nearest_mm);
    printf("  bearing      : %+d deg\n", profile.bearing_deg);
    printf("  width        : %u mm\n", (unsigned)profile.width_mm);
    printf("  edges        : %+d .. %+d deg\n",
           profile.right_edge_deg, profile.left_edge_deg);
    printf("  gap L / R    : %u / %u mm\n",
           (unsigned)profile.gap_left_mm, (unsigned)profile.gap_right_mm);
    printf("  swing L / R  : %u / %u mm\n",
           (unsigned)profile.swing_left_mm,
           (unsigned)profile.swing_right_mm);
    printf("  confidence   : %u %%\n", (unsigned)profile.confidence_pct);
    printf("  action       : %s\n", action_name(profile.action));

    puts("[E1] done.");
}

/*=====================================================================*/
/* Health dump                                                         */
/*=====================================================================*/

static void test_health(void)
{
    ultrasonic_stats_t stats;

    ultrasonic_get_stats(&stats);

    puts("\n[health] ultrasonic driver");
    printf("  pings        : %lu\n", (unsigned long)stats.ping_count);
    printf("  timeouts     : %lu\n", (unsigned long)stats.timeout_count);
    printf("  rejects      : %lu\n", (unsigned long)stats.reject_count);
    printf("  spurious edge: %lu\n", (unsigned long)stats.spurious_edges);

    if (g_mission_up)
    {
        obstacle_health_t h;

        obstacle_get_health(&h);
        puts("[health] obstacle task");
        printf("  state        : %d\n", (int)h.state);
        printf("  scans        : %lu\n", (unsigned long)h.scans_completed);
        printf("  overruns     : %lu\n",
               (unsigned long)h.deadline_overruns);
        printf("  faults       : %lu\n", (unsigned long)h.sensor_faults);
    }
}

/*=====================================================================*/
/* Mission mode                                                        */
/*=====================================================================*/

/*=====================================================================*/
/* Buddy 1 - MQTT robot telemetry                                      */
/*=====================================================================*/

static void publish_robot_telemetry(
    uint16_t front_mm,
    const obstacle_health_t *health)
{
    char message[32];

    if (!wifi_is_connected() || !mqtt_is_connected())
    {
        return;
    }

    /* Publish front ultrasonic distance. */
    if (front_mm == OBSTACLE_RANGE_CLEAR)
    {
        (void)mqtt_publish_message(
            "inf2004/robot/distance",
            "CLEAR"
        );
    }
    else if (front_mm == OBSTACLE_RANGE_UNKNOWN)
    {
        (void)mqtt_publish_message(
            "inf2004/robot/distance",
            "UNKNOWN"
        );
    }
    else
    {
        snprintf(
            message,
            sizeof(message),
            "%u",
            (unsigned)front_mm
        );

        (void)mqtt_publish_message(
            "inf2004/robot/distance",
            message
        );
    }

    /* Publish obstacle state. */
    snprintf(
        message,
        sizeof(message),
        "%d",
        (int)health->state
    );

    (void)mqtt_publish_message(
        "inf2004/robot/obstacle_state",
        message
    );

    /* Publish number of completed scans. */
    snprintf(
        message,
        sizeof(message),
        "%lu",
        (unsigned long)health->scans_completed
    );

    (void)mqtt_publish_message(
        "inf2004/robot/scans",
        message
    );
}

/*=====================================================================*/
/* Buddy 1 - Connection recovery                                       */
/*=====================================================================*/

static void buddy1_connection_recovery(void)
{
    static uint32_t last_recovery_ms = 0U;

    const uint32_t now = tkshim_now_ms();

    /*
     * Check connection health every 5 seconds.
     * This prevents constant reconnection attempts.
     */
    if ((now - last_recovery_ms) < 5000U)
    {
        return;
    }

    last_recovery_ms = now;

    /*
     * WiFi must be available before MQTT can reconnect.
     */
    if (!wifi_is_connected())
    {
        printf("[Buddy 1] WiFi connection lost. Reconnecting...\n");

        if (wifi_connect(WIFI_TEST_SSID, WIFI_TEST_PASSWORD) != 0)
        {
            printf("[Buddy 1] WiFi reconnection failed.\n");
            return;
        }

        printf("[Buddy 1] WiFi reconnected successfully!\n");
    }

    /*
     * WiFi is available. Restore MQTT if necessary.
     */
    if (!mqtt_is_connected())
    {
        printf("[Buddy 1] MQTT connection lost. Reconnecting...\n");

        if (mqtt_reconnect() != 0)
        {
            printf("[Buddy 1] MQTT reconnection attempt failed.\n");
        }
    }
}

/*=====================================================================*/
/* Buddy 1 - MQTT heartbeat                                             */
/*=====================================================================*/

static void publish_heartbeat(void)
{
    /*
     * Only send the heartbeat when both WiFi and MQTT
     * are currently connected.
     */
    if (!wifi_is_connected() || !mqtt_is_connected())
    {
        return;
    }

    (void)mqtt_publish_message(
        "inf2004/robot/heartbeat",
        "alive"
    );
}

/**
 * @brief   Idle hook, called from inside the shim's wait loops.
 *
 * This is how the monitor prints without a second task. Under the
 * real kernel this whole function disappears; a separate low priority
 * telemetry task does the job instead.
 */
static void mission_monitor(void)
{
    static uint32_t last_ms = 0U;
    static uint32_t last_mqtt_ms = 0U;
    static uint32_t last_heartbeat_ms = 0U;

    const uint32_t now = tkshim_now_ms();
    buddy1_connection_recovery();
    
    
    /* Buddy 1 - report heartbeat every 5 seconds. */
    if ((now - last_heartbeat_ms) >= 5000U)
    {
        last_heartbeat_ms = now;
        publish_heartbeat();
    }

    if ((now - last_ms) >= 250U)
    {
        const uint16_t front = obstacle_get_front_mm();
        obstacle_health_t h;

        last_ms = now;
        obstacle_get_health(&h);

        /* Buddy 1 - publish telemetry once every second. */
        if ((now - last_mqtt_ms) >= 1000U)
        {
            last_mqtt_ms = now;
            publish_robot_telemetry(front, &h);
        }

        if (front == OBSTACLE_RANGE_CLEAR)
        {
            printf("[%6lu] front CLEAR        state %d scans %lu\n",
                   (unsigned long)now, (int)h.state,
                   (unsigned long)h.scans_completed);
        }
        else if (front == OBSTACLE_RANGE_UNKNOWN)
        {
            printf("[%6lu] front UNKNOWN      state %d scans %lu\n",
                   (unsigned long)now, (int)h.state,
                   (unsigned long)h.scans_completed);
        }
        else
        {
            printf("[%6lu] front %5u mm     state %d scans %lu\n",
                   (unsigned long)now, (unsigned)front, (int)h.state,
                   (unsigned long)h.scans_completed);
        }
    }
}

static void run_mission(void)
{
    puts("\n[mission] starting obstacle subsystem.");
    puts("The task now owns the servo and the ranger. Put something in");
    puts("front of the car to trigger a scan. Reset the board to stop.");

    if (!obstacle_init())
    {
        puts("[mission] obstacle_init FAILED, not safe to drive.");
    }
    else
    {
        g_mission_up = true;
        tkshim_set_idle_hook(mission_monitor);
        tkshim_run();   /* does not return on the target */
    }
}

/*=====================================================================*/
/* Menu                                                                */
/*=====================================================================*/

static void show_menu(void)
{
    puts("");
    puts("=== Buddy 5 obstacle subsystem, bench bring-up ===");
    puts("  1  [S1]    servo angle accuracy, stepped");
    puts("  2  [S2]    servo travel rate, ten hops");
    puts("  3  [U1]    single ping");
    puts("  4  [U1]    50 ping burst with statistics");
    puts("  5  [U3/U4] live stream, blind zone and unplug test");
    puts("  6  [E1]    sweep 11 points and build a profile");
    puts("  7          driver health counters");
    puts("  9          run mission mode (needs a reset to leave)");
    puts("  ?          this menu");
    puts("");
}

/*=====================================================================*/
/* Buddy 1 - WiFi bench test                                           */
/*=====================================================================*/

static void test_wifi_connection(void)
{
    int result;

    puts("");
    puts("[Buddy 1] WiFi connection test");

    result = wifi_init();

    if (result != 0)
    {
        printf("[Buddy 1] WiFi initialisation FAILED: %d\n", result);
        return;
    }

    result = wifi_connect(WIFI_TEST_SSID, WIFI_TEST_PASSWORD);

    if (result != 0)
    {
        printf("[Buddy 1] WiFi connection FAILED: %d\n", result);
        return;
    }

    puts("[Buddy 1] WiFi connection PASSED");
}

/*=====================================================================*/
/* Buddy 1 - MQTT publish bench test                                   */
/*=====================================================================*/

static void test_mqtt_publish(void)
{
    int result;

    puts("");
    puts("[Buddy 1] Starting MQTT publish test");

    if (!mqtt_is_connected())
    {
        puts("[Buddy 1] MQTT publish test SKIPPED: MQTT not connected");
        return;
    }

    result = mqtt_publish_message(
        "inf2004/robot/status",
        "Hello from pico W"
    );

    if (result == 0)
    {
        puts("[Buddy 1] MQTT publish request accepted");
    }
    else
    {
        printf(
            "[Buddy 1] MQTT publish test FAILED: %d\n",
            result
        );
    }

    /*
     * Give the asynchronous MQTT callback time to complete
     * before Buddy 5's interactive bench test starts.
     */
    sleep_ms(1000);
}

int main(void)
{
    stdio_init_all();

    /* Give the USB CDC link a moment so the banner is not lost. */
    sleep_ms(2000);

    /* Buddy 1 - WiFi bench test. */
    test_wifi_connection();
    test_mqtt_connection();
    test_mqtt_publish();

    puts("");
    puts("Buddy 5: adaptive ultrasonic scanning and obstacle profiling");
    printf("build      : %s %s\n", __DATE__, __TIME__);
    printf("servo pin  : GP%u\n", (unsigned)BOARD_SERVO_GPIO);
    printf("sonar trig : GP%u\n", (unsigned)BOARD_SONAR_TRIG_GPIO);
    printf("sonar echo : GP%u  (3V3 ONLY - check the level shift)\n",
           (unsigned)BOARD_SONAR_ECHO_GPIO);

    show_menu();

    for (;;)
    {
        const int key = getchar();

        if ((key == '9') && !g_drivers_up)
        {
            run_mission();
        }
        else if (key == '9')
        {
            puts("Reset the board before entering mission mode, so the");
            puts("drivers are initialised exactly once.");
        }
        else if (key == '?')
        {
            show_menu();
        }
        else if (drivers_up())
        {
            switch (key)
            {
                case '1': test_servo_angles();  break;
                case '2': test_servo_travel();  break;
                case '3':
                {
                    ultrasonic_result_t r;
                    const ultrasonic_status_t s = ultrasonic_ping(&r);

                    printf("\n  %-10s %5u mm  echo %lu us\n",
                           status_name(s), (unsigned)r.distance_mm,
                           (unsigned long)r.echo_us);
                    break;
                }
                case '4': test_range_burst();   break;
                case '5': test_live_stream();   break;
                case '6': test_sweep_profile(); break;
                case '7': test_health();        break;
                default:                        break;
            }
        }
        else
        {
            puts("Drivers failed to initialise, check the wiring.");
        }
    }
}
