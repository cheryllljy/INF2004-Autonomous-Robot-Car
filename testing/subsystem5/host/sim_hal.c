/**
 * @file    sim_hal.c
 * @brief   HOST TEST ONLY. Fake servo and fake HC-SR04 implementing
 *          drv_servo.h and drv_ultrasonic.h against a simulated world,
 *          so the real obstacle_task.c and geometry run on a PC.
 *
 * The fake sensor charges the same time the real v2 driver does: the
 * 60 ms quiet period, then the fixed 30 ms read window (the real driver
 * reads the PWM counter after the window, not at the falling edge).
 * The shim's virtual clock records it, so the scan times printed by
 * test_sim.c are a real check of the timing budget.
 *
 * Floating point is fine here; it never runs on the robot.
 */

#include "sim_hal.h"

#include <tk/tkernel.h>

#include <math.h>
#include <string.h>

#include "drv_servo.h"
#include "drv_ultrasonic.h"

#define SIM_MAX_BOXES       (4)
#define SIM_BEAM_HALF_DEG   (7.5)
#define SIM_BEAM_RAYS       (5)
#define SIM_PI              (3.14159265358979323846)
#define SIM_QUIET_MS        (60U)   /* real: 60 ms of kernel time */
#define SIM_WINDOW_MS       (30U)   /* real: 30 ms of kernel time (v2.1) */
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
static int                g_fault;
static int16_t            g_angle = DRV_SERVO_CENTRE_DEG;
static uint32_t           g_last_trig;
static int                g_fired;
static uint32_t           g_noise_seed = 1U;
static int16_t            g_noise_mm;
static int32_t            g_speed;
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

void sim_set_fault(int mode)          { g_fault = mode; }
void sim_set_noise_mm(int16_t a)      { g_noise_mm = a; }
void sim_set_speed_mm_s(int32_t s)    { g_speed = s; }

bool sim_odometry(int32_t *p_speed_mm_s, int32_t *p_distance_mm)
{
    *p_speed_mm_s  = g_speed;
    *p_distance_mm = 0;

    return true;
}

void sim_counters_reset(void)
{
    memset(&g_counters, 0, sizeof(g_counters));
    memset(&g_stats, 0, sizeof(g_stats));
    g_fired = 0;
    g_angle = DRV_SERVO_CENTRE_DEG;
    g_speed = 0;
    g_noise_mm = 0;
}

void sim_counters_get(sim_counters_t *p_out)
{
    *p_out = g_counters;
}

/*=====================================================================*/
/* Ray casting                                                         */
/*=====================================================================*/

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

                if ((x >= g_boxes[i].x_min) && (x <= g_boxes[i].x_max) &&
                    ((best == 0.0) || (t < best)))
                {
                    best = t;
                }
            }
        }
    }

    return best;
}

/** Shortest return inside the 15 deg beam, as a real HC-SR04 reports. */
static double sim_beam_range(double bearing_deg)
{
    double best = 0.0;
    int    i;

    for (i = 0; i < SIM_BEAM_RAYS; i++)
    {
        const double off = -SIM_BEAM_HALF_DEG +
                           ((2.0 * SIM_BEAM_HALF_DEG * i) /
                            (SIM_BEAM_RAYS - 1));
        const double r   = sim_cast_ray(bearing_deg + off);

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

static void sim_count(uint16_t *p)
{
    if (*p < 0xFFFFU) { (*p)++; }
}

/*=====================================================================*/
/* drv_servo.h                                                         */
/*=====================================================================*/

bool drv_servo_init(void)
{
    g_angle = DRV_SERVO_CENTRE_DEG;
    return true;
}

bool drv_servo_set_angle(int16_t angle_deg)
{
    int16_t c = angle_deg;

    if (c < DRV_SERVO_MIN_DEG) { c = DRV_SERVO_MIN_DEG; }
    if (c > DRV_SERVO_MAX_DEG) { c = DRV_SERVO_MAX_DEG; }

    if (c != g_angle)
    {
        g_counters.servo_moves++;
    }
    g_angle = c;

    return (c == angle_deg);
}

int16_t drv_servo_get_angle(void)
{
    return g_angle;
}

uint32_t drv_servo_settle_ms(int16_t from_deg, int16_t to_deg)
{
    int32_t d = (int32_t)to_deg - (int32_t)from_deg;

    if (d < 0) { d = -d; }

    return (((uint32_t)d * SIM_TRAVEL_MS_60) / 60U) + SIM_DAMPING_MS;
}

void drv_servo_release(void)
{
}

/*=====================================================================*/
/* drv_ultrasonic.h                                                    */
/*=====================================================================*/

bool drv_ultrasonic_init(void)
{
    g_fired = 0;
    return true;
}

uint32_t drv_ultrasonic_time_us(void)
{
    return tkshim_now_ms() * 1000U;
}

ultrasonic_status_t drv_ultrasonic_ping(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t st   = ULTRASONIC_NO_ECHO;
    uint16_t            mm   = 0xFFFFU;
    uint32_t            echo = 0U;

    if (g_fired)
    {
        const uint32_t since = tkshim_now_ms() - g_last_trig;

        if (since < SIM_QUIET_MS)
        {
            (void)tk_dly_tsk((RELTIM)(SIM_QUIET_MS - since));
        }
    }

    if (g_fault == 1)
    {
        sim_count(&g_stats.rejects);
        st = ULTRASONIC_ECHO_STUCK;
    }
    else
    {
        g_last_trig = tkshim_now_ms();
        g_fired     = 1;
        g_counters.pings++;
        sim_count(&g_stats.pings);

        (void)tk_dly_tsk(SIM_WINDOW_MS);

        if (g_fault == 2)
        {
            sim_count(&g_stats.no_response);
            st = ULTRASONIC_NO_RESPONSE;
        }
        else
        {
            const double range =
                sim_beam_range((double)(g_angle - DRV_SERVO_CENTRE_DEG));

            if (range <= 0.0)
            {
                sim_count(&g_stats.no_echo);
                st = ULTRASONIC_NO_ECHO;
            }
            else
            {
                const int32_t r = (int32_t)(range + 0.5) + sim_noise();

                echo = (uint32_t)((2.0 * range) / 0.343);

                if (r < (int32_t)DRV_US_MIN_VALID_MM)
                {
                    mm = (uint16_t)DRV_US_MIN_VALID_MM;
                    st = ULTRASONIC_TOO_CLOSE;
                }
                else if (r > (int32_t)DRV_US_MAX_VALID_MM)
                {
                    st = ULTRASONIC_TOO_FAR;
                }
                else
                {
                    mm = (uint16_t)r;
                    st = ULTRASONIC_OK;
                }
            }
        }
    }

    if (p_result != NULL)
    {
        *p_result = drv_us_result_pack(mm, echo, st);
    }

    return st;
}

ultrasonic_status_t drv_ultrasonic_ping_median(ultrasonic_result_t *p_result)
{
    uint16_t v[DRV_US_MEDIAN_SAMPLES];
    uint8_t  n = 0U;
    uint8_t  faults = 0U;
    uint8_t  i;
    uint16_t mm = 0xFFFFU;
    ultrasonic_status_t st;

    for (i = 0U; i < (uint8_t)DRV_US_MEDIAN_SAMPLES; i++)
    {
        ultrasonic_result_t one;
        const ultrasonic_status_t s = drv_ultrasonic_ping(&one);

        if (s == ULTRASONIC_OK)
        {
            v[n] = drv_us_result_mm(one);
            n++;
        }
        else if ((s == ULTRASONIC_NO_RESPONSE) || (s == ULTRASONIC_ECHO_STUCK))
        {
            faults++;
        }
        else
        {
            /* Answered. */
        }
    }

    if (n >= 2U)
    {
        uint8_t a;

        for (a = 1U; a < n; a++)
        {
            const uint16_t key = v[a];
            uint8_t        b   = a;

            while ((b > 0U) && (v[b - 1U] > key))
            {
                v[b] = v[b - 1U];
                b--;
            }
            v[b] = key;
        }

        mm = v[n / 2U];
        st = ULTRASONIC_OK;
    }
    else
    {
        st = (faults > 0U) ? ULTRASONIC_NO_RESPONSE : ULTRASONIC_NO_ECHO;
    }

    *p_result = drv_us_result_pack(mm, 0U, st);

    return st;
}

void drv_ultrasonic_get_stats(ultrasonic_stats_t *p_stats)
{
    *p_stats = g_stats;
}
