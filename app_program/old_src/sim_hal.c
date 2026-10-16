/**
 * @file    sim_hal.c
 * @brief   HOST TEST CODE ONLY. Fake servo and fake HC-SR04 that
 *          implement servo.h and ultrasonic.h against a simulated
 *          world, so src/obstacle.c can be run on a laptop.
 *
 * This file is never compiled into the firmware. It replaces
 * src/servo.c and src/ultrasonic.c in the host simulation build, so
 * the thing under test is the real state machine and the real
 * geometry, driven by a sensor whose answers we control exactly.
 *
 * Floating point is used freely here. That is fine: this runs on a
 * PC, not on the Cortex-M0+. Nothing in src/ does the same.
 *
 * The simulated sensor charges the same time penalties the real one
 * does (60 ms quiet period, flight time, servo settling), and the
 * shim's virtual clock records them. That is what makes the scan
 * wall time printed by test_sim.c a meaningful check of the timing
 * budget rather than a guess.
 */

#include "sim_hal.h"

#include <tk/tkernel.h>

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "servo.h"
#include "ultrasonic.h"

#define SIM_MAX_BOXES       (4)
#define SIM_BEAM_HALF_DEG   (7.5)
#define SIM_BEAM_RAYS       (5)
#define SIM_PI              (3.14159265358979323846)
#define SIM_QUIET_MS        (60U)
#define SIM_SPEED_MM_PER_MS (343.0)
#define SIM_TRAVEL_MS_60    (140U)
#define SIM_DAMPING_MS      (25U)

typedef struct
{
    double x_min;
    double x_max;
    double y;
    int    used;
} sim_box_t;

static sim_box_t          g_boxes[SIM_MAX_BOXES];
static int                g_fault      = 0;
static int16_t            g_angle      = SERVO_ANGLE_CENTRE_DEG;
static uint32_t           g_last_trig  = 0U;
static uint32_t           g_noise_seed = 1U;
static int16_t            g_noise_mm   = 0;
static ultrasonic_stats_t g_stats;
static sim_counters_t     g_counters;

/*=====================================================================*/
/* World control                                                       */
/*=====================================================================*/

void sim_world_clear(void)
{
    memset(g_boxes, 0, sizeof(g_boxes));
    g_fault = 0;
}

void sim_add_box(double centre_x_mm, double y_mm, double width_mm)
{
    int i;

    for (i = 0; i < SIM_MAX_BOXES; i++)
    {
        if (!g_boxes[i].used)
        {
            g_boxes[i].used  = 1;
            g_boxes[i].x_min = centre_x_mm - (width_mm / 2.0);
            g_boxes[i].x_max = centre_x_mm + (width_mm / 2.0);
            g_boxes[i].y     = y_mm;
            break;
        }
    }
}

void sim_set_fault(int on)
{
    g_fault = on;
}

void sim_set_noise_mm(int16_t amplitude_mm)
{
    g_noise_mm = amplitude_mm;
}

void sim_counters_reset(void)
{
    memset(&g_counters, 0, sizeof(g_counters));
    memset(&g_stats, 0, sizeof(g_stats));
}

void sim_counters_get(sim_counters_t *p_out)
{
    if (p_out != NULL)
    {
        *p_out = g_counters;
    }
}

/*=====================================================================*/
/* Ray casting                                                         */
/*=====================================================================*/

/**
 * @brief   Distance to the nearest box along one ray.
 * @return  Range in mm, or 0 for no hit.
 */
static double sim_cast_ray(double bearing_deg)
{
    const double rad  = bearing_deg * SIM_PI / 180.0;
    const double sinb = sin(rad);
    const double cosb = cos(rad);
    double       best = 0.0;
    int          i;

    if (cosb > 0.05)
    {
        for (i = 0; i < SIM_MAX_BOXES; i++)
        {
            if (g_boxes[i].used)
            {
                const double t = g_boxes[i].y / cosb;
                const double x = t * sinb;

                if ((x >= g_boxes[i].x_min) && (x <= g_boxes[i].x_max))
                {
                    if ((best == 0.0) || (t < best))
                    {
                        best = t;
                    }
                }
            }
        }
    }

    return best;
}

/**
 * @brief   Shortest return inside the 15 degree beam, which is what a
 *          real HC-SR04 reports.
 */
static double sim_beam_range(double bearing_deg)
{
    double best = 0.0;
    int    i;

    for (i = 0; i < SIM_BEAM_RAYS; i++)
    {
        const double offset =
            -SIM_BEAM_HALF_DEG +
            ((2.0 * SIM_BEAM_HALF_DEG * i) / (SIM_BEAM_RAYS - 1));
        const double r = sim_cast_ray(bearing_deg + offset);

        if ((r > 0.0) && ((best == 0.0) || (r < best)))
        {
            best = r;
        }
    }

    return best;
}

static int16_t sim_noise(void)
{
    int16_t n = 0;

    if (g_noise_mm > 0)
    {
        g_noise_seed = (g_noise_seed * 1103515245U) + 12345U;
        n = (int16_t)(((g_noise_seed >> 16) %
                       (uint32_t)((2 * g_noise_mm) + 1)) - g_noise_mm);
    }

    return n;
}

/*=====================================================================*/
/* servo.h implementation                                              */
/*=====================================================================*/

bool servo_init(void)
{
    g_angle = SERVO_ANGLE_CENTRE_DEG;

    return true;
}

bool servo_set_angle(int16_t angle_deg)
{
    int16_t clamped = angle_deg;

    if (clamped < SERVO_ANGLE_MIN_DEG) { clamped = SERVO_ANGLE_MIN_DEG; }
    if (clamped > SERVO_ANGLE_MAX_DEG) { clamped = SERVO_ANGLE_MAX_DEG; }

    g_angle = clamped;
    g_counters.servo_moves++;

    return (clamped == angle_deg);
}

int16_t servo_get_angle(void)
{
    return g_angle;
}

uint32_t servo_settle_ms(int16_t from_deg, int16_t to_deg)
{
    int32_t delta = (int32_t)to_deg - (int32_t)from_deg;

    if (delta < 0)
    {
        delta = -delta;
    }

    return (((uint32_t)delta * SIM_TRAVEL_MS_60) / 60U) + SIM_DAMPING_MS;
}

void servo_release(void)
{
}

/*=====================================================================*/
/* ultrasonic.h implementation                                         */
/*=====================================================================*/

bool ultrasonic_init(void)
{
    g_last_trig = tkshim_now_ms();

    return true;
}

ultrasonic_status_t ultrasonic_ping(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t status = ULTRASONIC_BAD_PARAM;

    if (p_result != NULL)
    {
        const uint32_t since = tkshim_now_ms() - g_last_trig;

        /* Same quiet period the real driver enforces, so the virtual
         * clock charges the scan the same cost the bench will. */
        if (since < SIM_QUIET_MS)
        {
            (void)tk_dly_tsk((RELTIM)(SIM_QUIET_MS - since));
        }

        g_last_trig = tkshim_now_ms();
        g_counters.pings++;
        g_stats.ping_count++;

        p_result->distance_mm = ULTRASONIC_RANGE_NONE;
        p_result->echo_us     = 0U;

        if (g_fault == 1)
        {
            g_stats.reject_count++;
            status = ULTRASONIC_ECHO_STUCK;
        }
        else if (g_fault == 2)
        {
            /* Trigger sent, ECHO never rises: the bug this build was
             * written to catch. Charges the full timeout, as the real
             * driver does. */
            (void)tk_dly_tsk(25U);
            g_stats.no_response_count++;
            status = ULTRASONIC_NO_RESPONSE;
        }
        else
        {
            const double bearing =
                (double)(g_angle - SERVO_ANGLE_CENTRE_DEG);
            const double range = sim_beam_range(bearing);

            if (range <= 0.0)
            {
                /* Flight out to the timeout, then nothing. */
                (void)tk_dly_tsk(25U);
                g_stats.timeout_count++;
                status = ULTRASONIC_NO_ECHO;
            }
            else
            {
                const double flight_ms =
                    (2.0 * range) / SIM_SPEED_MM_PER_MS;
                int32_t mm = (int32_t)(range + 0.5) + sim_noise();

                if (flight_ms >= 1.0)
                {
                    (void)tk_dly_tsk((RELTIM)flight_ms);
                }

                p_result->echo_us = (uint32_t)(flight_ms * 1000.0);

                if (mm < (int32_t)ULTRASONIC_MIN_VALID_MM)
                {
                    p_result->distance_mm =
                        (uint16_t)ULTRASONIC_MIN_VALID_MM;
                    status = ULTRASONIC_TOO_CLOSE;
                }
                else if (mm > (int32_t)ULTRASONIC_MAX_VALID_MM)
                {
                    status = ULTRASONIC_TOO_FAR;
                }
                else
                {
                    p_result->distance_mm = (uint16_t)mm;
                    status = ULTRASONIC_OK;
                }
            }
        }

        p_result->status = status;
    }

    return status;
}

ultrasonic_status_t ultrasonic_ping_median(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t status = ULTRASONIC_BAD_PARAM;

    if (p_result != NULL)
    {
        uint16_t valid[ULTRASONIC_MEDIAN_SAMPLES];
        uint8_t  n = 0U;
        uint8_t  i;

        for (i = 0U; i < (uint8_t)ULTRASONIC_MEDIAN_SAMPLES; i++)
        {
            ultrasonic_result_t one;

            if (ultrasonic_ping(&one) == ULTRASONIC_OK)
            {
                valid[n] = one.distance_mm;
                n++;
            }
        }

        if (n >= 2U)
        {
            uint8_t a;

            for (a = 1U; a < n; a++)
            {
                const uint16_t key = valid[a];
                uint8_t        b   = a;

                while ((b > 0U) && (valid[b - 1U] > key))
                {
                    valid[b] = valid[b - 1U];
                    b--;
                }
                valid[b] = key;
            }

            p_result->distance_mm = valid[n / 2U];
            status                = ULTRASONIC_OK;
        }
        else
        {
            p_result->distance_mm = ULTRASONIC_RANGE_NONE;
            status                = ULTRASONIC_NO_ECHO;
        }

        p_result->echo_us = 0U;
        p_result->status  = status;
    }

    return status;
}

void ultrasonic_get_stats(ultrasonic_stats_t *p_stats)
{
    if (p_stats != NULL)
    {
        *p_stats = g_stats;
    }
}
