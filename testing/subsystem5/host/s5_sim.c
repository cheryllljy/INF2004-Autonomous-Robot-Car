/**
 * @file    s5_sim.c
 * @brief   HOST TEST ONLY. See s5_sim.h. Also the
 *          host's s5_hw.c (the same five functions) and s5_host_log().
 */

#include "s5_sim.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "motion_mock.h"
#include "s5_config.h"
#include "s5_hw.h"

#define PI          (3.14159265358979323846)
#define D2R         (PI / 180.0)
#define CAR_FRONT   (135.0)
#define CAR_REAR    (45.0)
#define CAR_HALF_W  (65.0)
#define LINE_HALF   (9.0)
#define RISE_US     (500U) /* TRIG to ECHO rising: the 8-cycle burst */
#define LOG_MAX     (600)
#define LOG_LEN     (240)

sim_world_t g_sim;

static uint32_t g_us;
static uint32_t g_ms;
static uint64_t g_rng;
/* the compass's noise has its own generator, so adding
 * the compass leaves every other noise (pings, dropouts) as it was for a
 * seed and the logs stay comparable with the ones before it */
static uint64_t g_rng_mag;
static double g_servo;     /* real angle now, deg, + = left */
static double g_servo_cmd; /* real angle it is driving to */
static uint32_t g_pings;
/* the ping in flight */
static int g_fired;
static uint32_t g_rise_us;
static uint32_t g_fall_us;
static int g_rises;
static int g_counting; /* the gated counter runs (zeroed at fire) */
/* our line */
static double g_px[SIM_MAX_PTS];
static double g_py[SIM_MAX_PTS];
static int g_np;
static double g_tx, g_ty, g_th;
/* log */
static char g_log[LOG_MAX][LOG_LEN];
static int g_log_n;
static int g_log_longest;
static int g_verbose;

/*=====================================================================*/
/* Small things                                                          */
/*=====================================================================*/

static double rnd01(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 7;
    g_rng ^= g_rng << 17;
    return (double)(g_rng >> 11) * (1.0 / 9007199254740992.0);
}

static double rnd_gauss(void)
{
    /* sum of 4 uniforms, scaled to sd 1 */
    return (rnd01() + rnd01() + rnd01() + rnd01() - 2.0) * 1.7320508;
}

static double rnd_mag_gauss(void)
{
    const uint64_t keep = g_rng;
    double v;

    g_rng = g_rng_mag;
    v = rnd_gauss();
    g_rng_mag = g_rng;
    g_rng = keep;
    return v;
}

void sim_reset(uint32_t seed)
{
    memset(&g_sim, 0, sizeof g_sim);
    g_sim.beam_deg = 22.0;
    g_sim.corner_frac = 1.0;
    g_sim.noise_mm = 2.0;
    g_sim.drop = 0.02;
    g_sim.temp_c = 25.0;
    g_sim.sonar_ahead = 70.0;
    g_sim.no_echo_ms = 38.0;
    g_sim.servo_dps = 450.0;
    g_sim.ir_ahead = 125.0;
    g_sim.ir_half = 12.5; /* 25 mm apart (was 15) */
    g_sim.n_lane = 1;
    g_sim.mag_on = 1;
    g_sim.mag_r = 2500.0;
    g_sim.mag_dir_deg = 37.0;
    g_sim.mag_err_deg = 2.0;
    g_us = 0xFFFFFFFFU - 3000000U; /* wraps 3 s in */
    g_ms = 0U;
    g_rng = 0x9E3779B97F4A7C15ULL ^ ((uint64_t)seed * 0x100000001B3ULL);
    if (0U == g_rng)
    {
        g_rng = 1U;
    }
    g_rng_mag = g_rng ^ 0xD1B54A32D192ED03ULL;
    if (0U == g_rng_mag)
    {
        g_rng_mag = 1U;
    }
    (void)rnd01();
    g_servo = 0.0;
    g_servo_cmd = 0.0;
    g_pings = 0U;
    g_fired = 0;
    g_counting = 0;
    g_log_n = 0;
    g_log_longest = 0;
    sim_track_start();
    sim_track_straight(6000.0);
}

/*=====================================================================*/
/* Track                                                                 */
/*=====================================================================*/

static void t_point(void)
{
    if (g_np < SIM_MAX_PTS)
    {
        g_px[g_np] = g_tx;
        g_py[g_np] = g_ty;
        g_np++;
    }
}

void sim_track_start(void)
{
    g_np = 0;
    g_tx = -400.0; /* the line starts behind the car */
    g_ty = 0.0;
    g_th = 0.0;
    t_point();
}

void sim_track_straight(double len)
{
    const int n = (int)(len / 5.0);
    int i;

    for (i = 0; i < n; i++)
    {
        g_tx += (len / n) * cos(g_th * D2R);
        g_ty += (len / n) * sin(g_th * D2R);
        t_point();
    }
}

void sim_track_arc(double radius, double angle_deg)
{
    const double len = fabs(angle_deg) * D2R * radius;
    const int n = (int)(len / 5.0) + 1;
    const double dth = angle_deg / n;
    int i;

    for (i = 0; i < n; i++)
    {
        g_th += dth / 2.0;
        g_tx += (len / n) * cos(g_th * D2R);
        g_ty += (len / n) * sin(g_th * D2R);
        g_th += dth / 2.0;
        t_point();
    }
}

/* signed distance from (x, y) to our line moved left by `lane` mm
 * (+ = the point is to the left of it) */
static double lane_dist(double x, double y, double lane)
{
    double best = 1e18;
    double sign = 1.0;
    int i;

    for (i = 0; i + 1 < g_np; i++)
    {
        const double dx = g_px[i + 1] - g_px[i];
        const double dy = g_py[i + 1] - g_py[i];
        const double len = sqrt(dx * dx + dy * dy);
        /* the lane's segment: moved along the left normal */
        const double nx = -dy / len;
        const double ny = dx / len;
        const double ax = g_px[i] + lane * nx;
        const double ay = g_py[i] + lane * ny;
        double t = ((x - ax) * dx + (y - ay) * dy) / (len * len);
        double d;

        if (t < 0.0)
        {
            t = 0.0;
        }
        if (t > 1.0)
        {
            t = 1.0;
        }
        d = hypot(x - (ax + t * dx), y - (ay + t * dy));
        if (d < best)
        {
            best = d;
            sign = (((x - ax) * nx + (y - ay) * ny) >= 0.0) ? 1.0 : -1.0;
        }
    }
    return sign * best;
}

/*=====================================================================*/
/* Car                                                                   */
/*=====================================================================*/

void sim_car_place(double x, double y, double th_deg)
{
    const double a = g_sim.track_deg * D2R;
    const double wx = x * cos(a) - y * sin(a);
    const double wy = x * sin(a) + y * cos(a);

    /* mock: x = left (world Y), y = forward (world X) */
    mm_reset(wy, wx, th_deg + g_sim.track_deg);
}

void sim_car_pose(double *p_x, double *p_y, double *p_th_rad)
{
    double mx;
    double my;
    double psi;
    const double a = -g_sim.track_deg * D2R;

    mm_get_truth(&mx, &my, &psi);
    *p_x = my * cos(a) - mx * sin(a);
    *p_y = my * sin(a) + mx * cos(a);
    *p_th_rad = (psi - g_sim.track_deg) * D2R;
}

void sim_body(double ahead, double left, double *p_x, double *p_y)
{
    double x;
    double y;
    double th;

    sim_car_pose(&x, &y, &th);
    *p_x = x + ahead * cos(th) - left * sin(th);
    *p_y = y + ahead * sin(th) + left * cos(th);
}

bool sim_ir_black(bool b_left)
{
    double x;
    double y;
    int k;

    sim_body(g_sim.ir_ahead, b_left ? g_sim.ir_half : -g_sim.ir_half, &x,
             &y);
    for (k = 0; k < g_sim.n_lane; k++)
    {
        if (fabs(lane_dist(x, y, g_sim.lane[k])) <= LINE_HALF)
        {
            return true;
        }
    }
    for (k = 0; k < g_sim.n_seg; k++)
    {
        const double *p_s = g_sim.seg[k];
        const double dx = p_s[2] - p_s[0];
        const double dy = p_s[3] - p_s[1];
        double t = ((x - p_s[0]) * dx + (y - p_s[1]) * dy) /
                   ((dx * dx) + (dy * dy));

        t = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
        if (hypot(x - (p_s[0] + t * dx), y - (p_s[1] + t * dy)) <= LINE_HALF)
        {
            return true;
        }
    }
    return false;
}

int sim_ir_lane(double *p_off)
{
    double x;
    double y;
    double best = 1e18;
    int bk = 0;
    int k;

    sim_body(g_sim.ir_ahead, 0.0, &x, &y);
    for (k = 0; k < g_sim.n_lane; k++)
    {
        const double d = lane_dist(x, y, g_sim.lane[k]);

        if (fabs(d) < fabs(best))
        {
            best = d;
            bk = k;
        }
    }
    *p_off = best;
    return bk;
}

/*=====================================================================*/
/* Boxes                                                                 */
/*=====================================================================*/

static void box_corners(const sim_box_t *p_b, double *p_x, double *p_y)
{
    /* anticlockwise: near right, far right, far left, near left */
    static const double sx[4] = { -1.0, 1.0, 1.0, -1.0 };
    static const double sy[4] = { -1.0, -1.0, 1.0, 1.0 };
    const double c = cos(p_b->rot_deg * D2R);
    const double s = sin(p_b->rot_deg * D2R);
    int i;

    for (i = 0; i < 4; i++)
    {
        const double lx = sx[i] * p_b->d / 2.0;
        const double ly = sy[i] * p_b->w / 2.0;

        p_x[i] = p_b->cx + lx * c - ly * s;
        p_y[i] = p_b->cy + lx * s + ly * c;
    }
}

static void car_corners(double *p_x, double *p_y)
{
    sim_body(CAR_FRONT, -CAR_HALF_W, &p_x[0], &p_y[0]);
    sim_body(CAR_FRONT, CAR_HALF_W, &p_x[1], &p_y[1]);
    sim_body(-CAR_REAR, CAR_HALF_W, &p_x[2], &p_y[2]);
    sim_body(-CAR_REAR, -CAR_HALF_W, &p_x[3], &p_y[3]);
}

static double seg_point(double ax, double ay, double bx, double by,
                        double px, double py)
{
    const double dx = bx - ax;
    const double dy = by - ay;
    double t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);

    if (t < 0.0)
    {
        t = 0.0;
    }
    if (t > 1.0)
    {
        t = 1.0;
    }
    return hypot(px - (ax + t * dx), py - (ay + t * dy));
}

static int inside(const double *p_x, const double *p_y, double px,
                  double py)
{
    int i;

    for (i = 0; i < 4; i++)
    {
        const int j = (i + 1) % 4;
        const double cr = (p_x[j] - p_x[i]) * (py - p_y[i]) -
                          (p_y[j] - p_y[i]) * (px - p_x[i]);

        if (cr < 0.0)
        {
            return 0;
        }
    }
    return 1;
}

/* the car's corners are in the order front right, front left, rear left,
 * rear right: anticlockwise too */
double sim_clearance(void)
{
    double cx[4];
    double cy[4];
    double best = 1e18;
    int b;

    car_corners(cx, cy);
    for (b = 0; b < g_sim.n_box; b++)
    {
        double bx[4];
        double by[4];
        int i;
        int j;

        if (!g_sim.box[b].on)
        {
            continue;
        }
        box_corners(&g_sim.box[b], bx, by);
        for (i = 0; i < 4; i++)
        {
            if (inside(bx, by, cx[i], cy[i]) || inside(cx, cy, bx[i], by[i]))
            {
                return -1.0;
            }
        }
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 4; j++)
            {
                const int k = (j + 1) % 4;
                double d = seg_point(bx[j], by[j], bx[k], by[k], cx[i], cy[i]);

                if (d < best)
                {
                    best = d;
                }
                d = seg_point(cx[j], cy[j], cx[k], cy[k], bx[i], by[i]);
                if (d < best)
                {
                    best = d;
                }
            }
        }
    }
    return best;
}

/*=====================================================================*/
/* Sonar                                                                 */
/*=====================================================================*/

static int seg_cross(double ax, double ay, double bx, double by, double cx,
                     double cy, double dx, double dy)
{
    const double d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const double d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
    const double d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
    const double d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);

    return ((d1 * d2) < 0.0) && ((d3 * d4) < 0.0);
}

/* the way from the sensor to (qx, qy) on box `self` crosses another box */
static int hidden(double sx, double sy, double qx, double qy, int self)
{
    int b;

    for (b = 0; b < g_sim.n_box; b++)
    {
        double bx[4];
        double by[4];
        int j;

        if ((b == self) || !g_sim.box[b].on)
        {
            continue;
        }
        box_corners(&g_sim.box[b], bx, by);
        for (j = 0; j < 4; j++)
        {
            const int k = (j + 1) % 4;

            if (seg_cross(sx, sy, qx, qy, bx[j], by[j], bx[k], by[k]))
            {
                return 1;
            }
        }
    }
    return 0;
}

static double ang_diff(double a, double b)
{
    double d = a - b;

    while (d > PI)
    {
        d -= 2.0 * PI;
    }
    while (d < -PI)
    {
        d += 2.0 * PI;
    }
    return d;
}

double sim_sonar_now(void)
{
    double sx;
    double sy;
    double x;
    double y;
    double th;
    double best = 0.0;
    const double beam = g_sim.beam_deg * D2R;
    const double cbeam = g_sim.corner_frac * beam;
    double look;
    int b;

    sim_body(g_sim.sonar_ahead, g_sim.sonar_left, &sx, &sy);
    sim_car_pose(&x, &y, &th);
    look = th + g_servo * D2R;
    for (b = 0; b < g_sim.n_box; b++)
    {
        double bx[4];
        double by[4];
        int i;

        if (!g_sim.box[b].on)
        {
            continue;
        }
        box_corners(&g_sim.box[b], bx, by);
        for (i = 0; i < 4; i++)
        {
            const int j = (i + 1) % 4;
            const double ex = bx[j] - bx[i];
            const double ey = by[j] - by[i];
            const double len = sqrt(ex * ex + ey * ey);
            /* anticlockwise corners: the outward normal is to the right */
            const double nx = ey / len;
            const double ny = -ex / len;
            const double dist = (sx - bx[i]) * nx + (sy - by[i]) * ny;
            const double t = ((sx - bx[i]) * ex + (sy - by[i]) * ey) /
                             (len * len);

            /* the face, square on */
            if ((dist > 0.0) && (t >= 0.0) && (t <= 1.0) &&
                (fabs(ang_diff(atan2(-ny, -nx), look)) <= beam))
            {
                const double qx = sx - dist * nx;
                const double qy = sy - dist * ny;

                if (!hidden(sx, sy, qx, qy, b) &&
                    ((0.0 == best) || (dist < best)))
                {
                    best = dist;
                }
            }
            /* corner i: seen if the sensor is outside one of its faces */
            {
                const int p = (i + 3) % 4;
                const double fx = bx[i] - bx[p];
                const double fy = by[i] - by[p];
                const double fl = sqrt(fx * fx + fy * fy);
                const double out_prev =
                    (sx - bx[p]) * (fy / fl) + (sy - by[p]) * (-fx / fl);
                const double r = hypot(bx[i] - sx, by[i] - sy);

                if (((dist > 0.0) || (out_prev > 0.0)) &&
                    (fabs(ang_diff(atan2(by[i] - sy, bx[i] - sx), look)) <=
                     cbeam) &&
                    !hidden(sx, sy, bx[i], by[i], b) &&
                    ((0.0 == best) || (r < best)))
                {
                    best = r;
                }
            }
        }
    }
    return best;
}

uint32_t sim_pings(void)
{
    return g_pings;
}

/*=====================================================================*/
/* Clock and servo                                                       */
/*=====================================================================*/

void sim_step_1ms(void)
{
    const double step = g_sim.servo_dps / 1000.0;
    const double d = g_servo_cmd - g_servo;

    g_servo += (d > step) ? step : ((d < -step) ? -step : d);
    g_us += 1000U;
    g_ms++;
}

uint32_t sim_now_us(void)
{
    return g_us;
}

uint32_t sim_ms(void)
{
    return g_ms;
}

double sim_servo_deg(void)
{
    return g_servo;
}

/*=====================================================================*/
/* s5_hw.h on the simulation                                             */
/*=====================================================================*/

bool s5_hw_init(void)
{
    g_fired = 0;
    s5_hw_servo(0);
    g_servo = g_servo_cmd;
    return true;
}

uint32_t s5_hw_us(void)
{
    return g_us;
}

void s5_hw_servo(int32_t ddeg)
{
    int32_t a = ddeg;
    double real;

    if (a > S5_SERVO_LIMIT_DDEG)
    {
        a = S5_SERVO_LIMIT_DDEG;
    }
    if (a < -S5_SERVO_LIMIT_R_DDEG)
    {
        a = -S5_SERVO_LIMIT_R_DDEG;
    }
    a += S5_SERVO_TRIM_DDEG;
#if (S5_SERVO_INVERT != 0)
    a = -a;
#endif
    /* the pulse is 900 + a (0.1 deg); the real servo turns the sensor */
    real = (g_sim.servo_invert ? -(double)a : (double)a) / 10.0 +
           g_sim.servo_trim_deg;
    g_servo_cmd = real;
}

static int echo_high(void)
{
    return g_fired && g_rises && ((int32_t)(g_us - g_rise_us) >= 0) &&
           ((int32_t)(g_us - g_fall_us) < 0);
}

bool s5_hw_fire(void)
{
    double r;

    if (echo_high())
    {
        return false;
    }
    g_pings++;
    g_fired = 1;
    g_counting = 1;
    g_rises = !g_sim.dead;
    g_rise_us = g_us + RISE_US;
    r = sim_sonar_now();
    if ((r > 0.0) && (rnd01() < g_sim.drop))
    {
        r = 0.0;
    }
    if (r > 0.0)
    {
        const double c = (331.3 + 0.606 * g_sim.temp_c) / 1000.0; /* mm/us */
        double mm = r + g_sim.range_off_mm + g_sim.noise_mm * rnd_gauss();

        if (mm < 5.0)
        {
            mm = 5.0;
        }
        g_fall_us = g_rise_us + (uint32_t)(2.0 * mm / c);
    }
    else
    {
        g_fall_us = g_rise_us + (uint32_t)(g_sim.no_echo_ms * 1000.0);
    }
    return true;
}

s5_echo_t s5_hw_echo(uint32_t *p_us)
{
    uint32_t count = 0U;

    if (g_counting && g_rises && ((int32_t)(g_us - g_rise_us) > 0))
    {
        const uint32_t end =
            ((int32_t)(g_us - g_fall_us) < 0) ? g_us : g_fall_us;

        count = end - g_rise_us;
    }
    g_counting = 0; /* frozen */
    *p_us = count & 0xFFFFU;
    if (echo_high())
    {
        return S5_ECHO_NONE;
    }
    if (0U == count)
    {
        return S5_ECHO_DEAD;
    }
    return S5_ECHO_OK;
}

/*=====================================================================*/
/* Log                                                                   */
/*=====================================================================*/

void s5_host_log(const char *p_fmt, ...)
{
    char buf[LOG_LEN];
    va_list ap;
    int n;

    va_start(ap, p_fmt);
    n = vsnprintf(buf, sizeof buf, p_fmt, ap);
    va_end(ap);
    if (n > g_log_longest)
    {
        g_log_longest = n;
    }
    if (g_log_n < LOG_MAX)
    {
        (void)snprintf(g_log[g_log_n], LOG_LEN, "%s", buf);
        g_log_n++;
    }
    if (g_verbose)
    {
        (void)printf("    %7.3f  %s", g_ms / 1000.0, buf);
    }
}

void sim_log_verbose(int verbose)
{
    g_verbose = verbose;
}

int sim_log_count(void)
{
    return g_log_n;
}

const char *sim_log_line(int i)
{
    return ((i >= 0) && (i < g_log_n)) ? g_log[i] : "";
}

int sim_log_find(const char *p_text, int from)
{
    int i;

    for (i = (from < 0) ? 0 : from; i < g_log_n; i++)
    {
        if (NULL != strstr(g_log[i], p_text))
        {
            return i;
        }
    }
    return -1;
}

int sim_log_longest(void)
{
    return g_log_longest;
}

/* Buddy 4's magnetometer on the simulated car */
bool s5_hw_mag(int32_t *p_x, int32_t *p_y)
{
    double x;
    double y;
    double th;
    double a;
    double fx;

    if (!g_sim.mag_on)
    {
        return false;
    }
    sim_car_pose(&x, &y, &th);
    /* the field's direction seen from the car, with the error left */
    a = (g_sim.mag_dir_deg * D2R) - th +
        (g_sim.mag_err_deg * D2R * sin(2.0 * th));
    fx = 0.0;
    if ((g_sim.mag_iron != 0.0) && (g_sim.n_box > 0) &&
        (hypot(x - g_sim.box[0].cx, y - g_sim.box[0].cy) < g_sim.mag_iron_mm))
    {
        fx = g_sim.mag_iron;
    }
    *p_x = (int32_t)lround(g_sim.mag_x0 + (g_sim.mag_r * cos(a)) + fx +
                           (3.0 * rnd_mag_gauss()));
    *p_y = (int32_t)lround(g_sim.mag_y0 + (g_sim.mag_r * sin(a)) +
                           (3.0 * rnd_mag_gauss()));
    return true;
}
