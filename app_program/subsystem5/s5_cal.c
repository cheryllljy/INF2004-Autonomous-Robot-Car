/*
 *      s5_cal.c
 *      Subsystem 5 -- guided calibration, make S5CAL=1.
 *      See s5_cal.h and testing/subsystem5/CALIBRATION.md.
 */

#include "s5_cal.h"

#if S5_CAL_BUILD

#include <tk/tkernel.h>

#include "../subsystem2/motion_control.h"
#include "s5_hw.h"
#include "s5_math.h"
#include "s5_odom.h"
#include "s5_path.h"
#include "s5_port.h"
#include "s5_scan.h"
#include "s5_sensor.h"

#define S5_CAL_PINGS    (20U)
#define S5_CAL_SPEED    (60.0f) /* step 4, mm/s */
#define S5_CAL_CIRCLE_S (40U)   /* step 4 gives up after this */
#define S5_US_PER_MS    (1000U)

#define S5_CAL_MAG_N    (300U)  /* step 4: compass samples kept */

/* make S5CAL=3: the angles the servo is asked for, in this order, each held
 * long enough to read it off a protractor (0.1 deg; + = left). Left to
 * right as the sweep goes, then back from the right. */
static const int16_t g_cal_ang[] = { 600,  450,  300,  150, 0, -150,
                                     -300, -S5_SERVO_LIMIT_R_DDEG,
                                     -300, -150, 0 };
#define S5_CAL_ANG_N    (sizeof(g_cal_ang) / sizeof(g_cal_ang[0]))
#define S5_CAL_ANG_MS   (8000U) /* each angle held this long */

static s5_sweep_t g_sw;
static s5_profile_t g_pr;
static int32_t g_off; /* step 2's range correction, used in step 3 */
/* step 4's compass samples: encoders' heading (rad) and
 * the field (counts) */
static float g_mag_e[S5_CAL_MAG_N];
static int16_t g_mag_x[S5_CAL_MAG_N];
static int16_t g_mag_y[S5_CAL_MAG_N];

static void s5_cal_wait_ms(uint32_t ms)
{
    const uint32_t t0 = s5_hw_us();

    while ((s5_hw_us() - t0) < (ms * S5_US_PER_MS))
    {
        s5_sensor_poll();
        s5_poll_wait();
    }
}

static void s5_cal_countdown(const char *p_what)
{
    int32_t s;

    S5_LOG("[VC-S5] CAL set up %s\n", p_what);
    for (s = S5_CAL_SETUP_S; s > 0; s -= 5)
    {
        S5_LOG("[VC-S5] CAL   measuring in %d s\n", (int)s);
        s5_cal_wait_ms((s >= 5) ? 5000U : ((uint32_t)s * 1000U));
    }
}

static void s5_cal_sort(uint16_t *p_v, uint32_t n)
{
    uint32_t i;

    for (i = 1U; i < n; i++)
    {
        const uint16_t key = p_v[i];
        uint32_t j = i;

        while ((j > 0U) && (p_v[j - 1U] > key))
        {
            p_v[j] = p_v[j - 1U];
            j--;
        }
        p_v[j] = key;
    }
}

/* n straight-ahead ranges (only answers); returns how many, sorted. */
static uint32_t s5_cal_pings(uint16_t *p_mm, uint32_t n)
{
    s5_ping_t p = { 0 };
    uint32_t got = 0U;
    uint32_t last;
    const uint32_t t0 = s5_hw_us();

    s5_sensor_guard();
    s5_cal_wait_ms(300U);
    (void)s5_sensor_ping(&p);
    last = p.seq;
    while ((got < n) && ((s5_hw_us() - t0) < (5000U * S5_US_PER_MS)))
    {
        s5_sensor_poll();
        s5_poll_wait();
        if (s5_sensor_ping(&p) && (p.seq != last))
        {
            last = p.seq;
            if (p.b_valid && (0 == p.ddeg) && (0U != p.mm))
            {
                p_mm[got] = p.mm;
                got++;
            }
        }
    }
    s5_cal_sort(p_mm, got);
    return got;
}

/* The obstacle logic's own sweep, from where the car stands. */
static bool s5_cal_sweep(void)
{
    const uint32_t seq = s5_sensor_scan();
    const uint32_t t0 = s5_hw_us();
    const s5_view_t view = { 0.0f, 0.0f, 0.0f };
    uint32_t i;

    while (!s5_sensor_scan_get(seq, &g_sw))
    {
        if ((s5_hw_us() - t0) > ((uint32_t)S5_SCAN_TIMEOUT_MS * S5_US_PER_MS))
        {
            s5_sensor_guard();
            S5_LOG("[VC-S5] CAL the sweep did not finish: is the sensor "
                   "answering?\n");
            return false;
        }
        s5_sensor_poll();
        s5_poll_wait();
    }
    s5_sensor_guard();
    for (i = 0U; i < g_sw.n; i++)
    {
        int32_t mm = (int32_t)g_sw.s[i].mm;

        if (0 != mm)
        {
            mm += g_off;
            g_sw.s[i].mm =
                (uint16_t)((mm < S5_RANGE_MIN_MM) ? S5_RANGE_MIN_MM : mm);
        }
        S5_LOG("[VC-S5] CAL   angle %d (0.1 deg): %d mm\n", (int)g_sw.s[i].ddeg,
               (int)g_sw.s[i].mm);
    }
    s5_scan_profile(g_sw.s, g_sw.n, &view, &g_pr);
    if (!g_pr.b_found)
    {
        S5_LOG("[VC-S5] CAL no box within 30 deg of straight ahead\n");
        return false;
    }
    if (!g_pr.b_left_seen || !g_pr.b_right_seen)
    {
        S5_LOG("[VC-S5] CAL an edge of the box is outside the sweep: "
               "move it further away or centre it, then run again\n");
        return false;
    }
    return true;
}

static void s5_cal_servo(void)
{
    S5_LOG("[VC-S5] CAL step 1/4: servo direction and trim\n");
    S5_LOG("[VC-S5] CAL   the sensor turns 45 deg to the car's LEFT now\n");
    s5_sensor_look(450);
    s5_cal_wait_ms(3000U);
    S5_LOG("[VC-S5] CAL   ... now 45 deg to the car's RIGHT\n");
    s5_sensor_look(-450);
    s5_cal_wait_ms(3000U);
    S5_LOG("[VC-S5] CAL   ... now straight ahead\n");
    s5_sensor_guard();
    s5_cal_wait_ms(1000U);
    S5_LOG("[VC-S5] CAL   left and right swapped? set S5_SERVO_INVERT to %d "
           "and run again\n",
           (0 != S5_SERVO_INVERT) ? 0 : 1);
    s5_cal_countdown("step 1: the box straight ahead of the car, face 300 mm "
                     "from the sensor, square to the car");
    if (s5_cal_sweep())
    {
        const int32_t c =
            ((int32_t)g_pr.left_ddeg + (int32_t)g_pr.right_ddeg) / 2;

        S5_LOG("[VC-S5] CAL step 1 result: the box's middle reads %d (0.1 "
               "deg). If the box really is straight ahead, set "
               "S5_SERVO_TRIM_DDEG to %d (now %d)\n",
               (int)c, (int)(S5_SERVO_TRIM_DDEG + c), (int)S5_SERVO_TRIM_DDEG);
    }
}

static void s5_cal_range(void)
{
    uint16_t mm[S5_CAL_PINGS];
    uint32_t n;

    S5_LOG("[VC-S5] CAL step 2/4: range\n");
    s5_cal_countdown("step 2: the box face square to the sensor, 200 mm from "
                     "the sensor's front (the metal cans)");
    n = s5_cal_pings(mm, S5_CAL_PINGS);
    if (n < (S5_CAL_PINGS / 2U))
    {
        S5_LOG("[VC-S5] CAL step 2: only %d answers out of %d: check the "
               "wiring and the box\n",
               (int)n, (int)S5_CAL_PINGS);
        return;
    }
    {
        const int32_t med = (int32_t)mm[n / 2U];

        g_off = S5_CAL_RANGE_MM - med;
        S5_LOG("[VC-S5] CAL step 2 result: median %d mm (%d to %d over %d "
               "pings): set S5_RANGE_OFFSET_MM to %d (now %d)\n",
               (int)med, (int)mm[0], (int)mm[n - 1U], (int)n,
               (int)(S5_RANGE_OFFSET_MM + g_off), (int)S5_RANGE_OFFSET_MM);
    }
}

static void s5_cal_beam(int32_t dist, const char *p_name)
{
    S5_LOG("[VC-S5] CAL step 3/4 (%s): beam half-angle at %d mm\n", p_name,
           (int)dist);
    if (S5_BEAM_NEAR_MM == dist)
    {
        s5_cal_countdown("step 3: the box's 21 cm side facing the car, "
                         "centred, face 200 mm from the sensor");
    }
    else
    {
        s5_cal_countdown("step 3: the box's 21 cm side facing the car, "
                         "centred, face 300 mm from the sensor");
    }
    if (s5_cal_sweep())
    {
        const float face = (float)g_pr.face_mm;
        const float half = s5_atan2f(0.5f * (float)S5_CAL_BOX_WIDTH_MM, face) *
                           (S5_RAD2DEG_F * 10.0f);
        const int32_t span = (int32_t)g_pr.left_ddeg - (int32_t)g_pr.right_ddeg;
        const int32_t beam = s5_roundf((((float)span) - (2.0f * half)) / 2.0f);

        S5_LOG("[VC-S5] CAL step 3 result: face %d mm, echoes over %d, the "
               "box itself %d (0.1 deg): set S5_BEAM_%s_DDEG to %d (now %d; "
               "with it the box read %d mm wide)\n",
               (int)g_pr.face_mm, (int)span, (int)s5_roundf(2.0f * half),
               p_name, (int)beam, (int)g_pr.beam_ddeg,
               (int)s5_roundf(g_pr.width));
    }
}

/* the compass from step 4's circle. The field's centre
 * (min/max), its strength, which way it turns with the car, and how far
 * it strays from the encoders on the way round. */
static void s5_cal_mag(uint32_t n, float turned)
{
    int32_t x_lo = 32767;
    int32_t x_hi = -32768;
    int32_t y_lo = 32767;
    int32_t y_hi = -32768;
    int32_t x0;
    int32_t y0;
    float r = 0.0f;
    float c = 0.0f; /* the compass's turn so far, unwrapped */
    float last = 0.0f;
    float worst = 0.0f;
    float k;
    int32_t sign;
    uint32_t i;

    if (n < 20U)
    {
        S5_LOG("[VC-S5] CAL compass: only %d samples from Buddy 4's IMU "
               "(is it plugged in? see its \"monitor: IMU\" lines): no "
               "compass\n",
               (int)n);
        return;
    }
    for (i = 0U; i < n; i++)
    {
        x_lo = (g_mag_x[i] < x_lo) ? g_mag_x[i] : x_lo;
        x_hi = (g_mag_x[i] > x_hi) ? g_mag_x[i] : x_hi;
        y_lo = (g_mag_y[i] < y_lo) ? g_mag_y[i] : y_lo;
        y_hi = (g_mag_y[i] > y_hi) ? g_mag_y[i] : y_hi;
    }
    x0 = (x_lo + x_hi) / 2;
    y0 = (y_lo + y_hi) / 2;
    for (i = 0U; i < n; i++)
    {
        const float dx = (float)(g_mag_x[i] - x0);
        const float dy = (float)(g_mag_y[i] - y0);
        const float a = s5_atan2f(dy, dx);

        r += s5_sqrtf((dx * dx) + (dy * dy));
        if (i > 0U)
        {
            c += s5_wrapf(a - last);
        }
        last = a;
    }
    r /= (float)n;
    /* the car turned left: the compass's angle went up or down */
    sign = (c >= 0.0f) ? 1 : -1;
    k = (s5_absf(turned) > 1.0f) ? (s5_absf(c) / turned) : 1.0f;
    c = 0.0f;
    for (i = 0U; i < n; i++)
    {
        const float a = s5_atan2f((float)(g_mag_y[i] - y0),
                                  (float)(g_mag_x[i] - x0));

        if (i > 0U)
        {
            c += s5_wrapf(a - last);
        }
        last = a;
        {
            const float d = s5_absf(((float)sign * c) - (k * g_mag_e[i]));

            worst = (d > worst) ? d : worst;
        }
    }
    S5_LOG("[VC-S5] CAL compass: set S5_MAG_SIGN %d, S5_MAG_X0 %d, "
           "S5_MAG_Y0 %d, S5_MAG_R %d (x %d..%d, y %d..%d)\n",
           (int)sign, (int)x0, (int)y0, (int)s5_roundf(r), (int)x_lo,
           (int)x_hi, (int)y_lo, (int)y_hi);
    S5_LOG("[VC-S5] CAL compass: it turned %d deg by the compass, %d by the "
           "encoders; at most %d deg apart on the way round\n",
           (int)s5_roundf(s5_absf(k * turned) * S5_RAD2DEG_F),
           (int)s5_roundf(turned * S5_RAD2DEG_F),
           (int)s5_roundf(worst * S5_RAD2DEG_F));
    if ((x_hi - x_lo) < 400)
    {
        S5_LOG("[VC-S5] CAL compass: the field hardly changed as the car "
               "turned: no compass (leave S5_MAG_SIGN 0)\n");
    }
    else if (worst > (10.0f * S5_DEG2RAD_F))
    {
        S5_LOG("[VC-S5] CAL compass: more than 10 deg apart: iron or "
               "wires near the IMU bend the field; leave S5_MAG_SIGN 0, or "
               "set S5_MAG_USE 0 to only log it\n");
    }
    else
    {
        S5_LOG("[VC-S5] CAL compass: good; by it S5_TURN_PCT would be %d "
               "(now %d)\n",
               (int)s5_roundf((float)S5_TURN_PCT * k), (int)S5_TURN_PCT);
    }
}

static void s5_cal_turn(void)
{
    s5_odom_t o;
    motion_status_t m;
    float turned = 0.0f;
    float last;
    int16_t l;
    int16_t r;
    const uint32_t t0 = s5_hw_us();
    uint32_t n_mag = 0U;
    uint32_t i_loop = 0U;
    int32_t mx;
    int32_t my;

    S5_LOG("[VC-S5] CAL step 4/4: how far the car really turns\n");
    s5_cal_countdown("step 4: the car on the floor with room for a 40 cm "
                     "circle; lay a strip of tape along its side to see "
                     "where it pointed. It drives one circle to the LEFT");
    s5_odom_init(&o);
    motion_get_status(&m);
    s5_odom_update(&o, m.ticks_l, m.ticks_r);
    last = o.psi;
    s5_path_wheels(S5_CAL_SPEED, 1.0f / (float)S5_CAL_CIRCLE_MM, &l, &r);
    while ((turned < (2.0f * S5_PI_F)) &&
           ((s5_hw_us() - t0) < ((S5_CAL_CIRCLE_S + (uint32_t)S5_CAL_SETUP_S) *
                                 1000U * S5_US_PER_MS)))
    {
        (void)motion_set_wheel_speeds(l, r);
        tk_dly_tsk(20);
        motion_get_status(&m);
        s5_odom_update(&o, m.ticks_l, m.ticks_r);
        turned += s5_wrapf(o.psi - last);
        last = o.psi;
        /* the compass, every 4th pass (80 ms) */
        if ((0U == (i_loop % 4U)) && (n_mag < S5_CAL_MAG_N) &&
            s5_hw_mag(&mx, &my))
        {
            g_mag_e[n_mag] = turned;
            g_mag_x[n_mag] = (int16_t)mx;
            g_mag_y[n_mag] = (int16_t)my;
            n_mag++;
        }
        i_loop++;
    }
    (void)motion_stop();
    S5_LOG("[VC-S5] CAL step 4 result: the encoders say %d deg. Short of a "
           "full circle by A deg: set S5_TURN_PCT to %d x (360 - A) / 360; "
           "past it by A deg: %d x (360 + A) / 360\n",
           (int)s5_roundf(turned * S5_RAD2DEG_F), (int)S5_TURN_PCT,
           (int)S5_TURN_PCT);
    s5_cal_mag(n_mag, turned);
}

/* make S5CAL=3: holds the servo at each angle of g_cal_ang so the real
 * angle can be measured; nothing is worked out on the car. */
static void s5_cal_angles(void)
{
    uint32_t i;

    S5_LOG("[VC-S5] CAL servo angles only (make S5CAL=3): %d stops, %d s "
           "each. At each one, write down where the sensor really points\n",
           (int)S5_CAL_ANG_N, (int)(S5_CAL_ANG_MS / 1000U));
    s5_sensor_guard();
    s5_cal_wait_ms(5000U);
    for (i = 0U; i < S5_CAL_ANG_N; i++)
    {
        const int32_t a = (int32_t)g_cal_ang[i];
        const int32_t a_abs = (a < 0) ? -a : a;
        const char *p_dir = "(straight ahead)";

        if (a > 0)
        {
            p_dir = "to the car's LEFT";
        }
        else if (a < 0)
        {
            p_dir = "to the car's RIGHT";
        }
        else
        {
            /* straight ahead */
        }
        S5_LOG("[VC-S5] CAL servo stop %d/%d: asked %d deg %s\n",
               (int)(i + 1U), (int)S5_CAL_ANG_N, (int)(a_abs / 10), p_dir);
        s5_sensor_look(a);
        s5_cal_wait_ms(S5_CAL_ANG_MS);
    }
    s5_sensor_guard();
    S5_LOG("[VC-S5] CAL servo angles done: send the %d angles you wrote "
           "down\n",
           (int)S5_CAL_ANG_N);
}

void s5_cal_run(void)
{
    if (3 == S5_CAL_BUILD)
    {
        s5_cal_angles();
        for (;;)
        {
            s5_sensor_poll();
            s5_poll_wait();
        }
    }
    S5_LOG("[VC-S5] CAL calibration build: 4 steps, about 2 minutes "
           "(testing/subsystem5/CALIBRATION.md)\n");
    if (2 != S5_CAL_BUILD)
    {
        s5_cal_servo();
        s5_cal_range();
        s5_cal_beam(S5_BEAM_NEAR_MM, "NEAR");
        s5_cal_beam(S5_BEAM_FAR_MM, "FAR");
    }
    else
    {
        /* make S5CAL=2: the circle only (turn and
         * compass) */
        S5_LOG("[VC-S5] CAL step 4 only (make S5CAL=2)\n");
    }
    s5_cal_turn();
    S5_LOG("[VC-S5] CAL finished: copy the numbers into s5_config.h, then "
           "build and flash the normal image (without S5CAL)\n");
    for (;;)
    {
        s5_sensor_poll();
        s5_poll_wait();
    }
}

#else
typedef int s5_cal_not_built_t; /* ISO C: no empty file */
#endif /* S5_CAL_BUILD */
