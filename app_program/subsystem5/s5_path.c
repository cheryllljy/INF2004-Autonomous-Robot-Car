/*
 *      s5_path.c
 *      Subsystem 5 -- the bypass path and the steering that follows it.
 *      See s5_path.h.
 */

#include "s5_path.h"

#include "s5_config.h"
#include "s5_math.h"
#include "s5_odom.h"

/* steepest point of S(t): S'(1/2) = 1.875; tightest: |S''| at
 * t = (3 - sqrt 3) / 6 = 5.7735 */
#define S5_S1_MAX   (1.875f)
#define S5_S2_MAX   (5.7735f)
#define S5_RAMP_MIN (40.0f) /* mm, a ramp is never shorter */
#define S5_CHECK_MM (10.0f) /* footprint checked this often */
/* no corner of the outline is further than this ahead of / behind the
 * axle, at any heading (front or rear plus the half width: a little more
 * than the corner's distance) */
#define S5_REACH_FRONT_MM ((float)(S5_CAR_FRONT_MM + S5_CAR_HALF_WIDTH_MM))
#define S5_REACH_REAR_MM  ((float)(S5_CAR_REAR_MM + S5_CAR_HALF_WIDTH_MM))
#define S5_APPROACH_MAX   (0.785f) /* 45 deg: steepest way back to a path */

static float s5_ramp_s(float t)
{
    return t * t * t * (10.0f + (t * (-15.0f + (6.0f * t))));
}

static float s5_ramp_s1(float t)
{
    const float u = t * (1.0f - t);

    return 30.0f * u * u;
}

static float s5_ramp_s2(float t)
{
    return 60.0f * t * (1.0f - t) * (1.0f - (2.0f * t));
}

static float s5_ramp_len(float dy, int32_t heading_deg, int32_t radius_mm)
{
    const float a = s5_absf(dy);
    const float by_heading =
        (S5_S1_MAX * a) / s5_tanf((float)heading_deg * S5_DEG2RAD_F);
    const float by_radius = s5_sqrtf(S5_S2_MAX * a * (float)radius_mm);
    float l = (by_heading > by_radius) ? by_heading : by_radius;

    if (l < S5_RAMP_MIN)
    {
        l = S5_RAMP_MIN;
    }
    return l;
}

float s5_path_ramp_len(float dy)
{
    return s5_ramp_len(dy, S5_MAX_HEADING_DEG, S5_MIN_RADIUS_MM);
}

float s5_path_ramp_len_steep(float dy)
{
    return s5_ramp_len(dy, S5_STEEP_HEADING_DEG, S5_STEEP_RADIUS_MM);
}

/* the ramp back in, sharper than the default */
float s5_path_ramp_len_in(float dy)
{
    return s5_ramp_len(dy, S5_IN_HEADING_DEG, S5_IN_RADIUS_MM);
}

/* one ramp from y_from to y_to over len, at t = 0..1 */
static void s5_ramp(float y_from, float y_to, float len, float t,
                    s5_ref_t *p_ref, float *p_y2)
{
    const float d = y_to - y_from;

    p_ref->y = y_from + (d * s5_ramp_s(t));
    p_ref->slope = (d * s5_ramp_s1(t)) / len;
    *p_y2 = (d * s5_ramp_s2(t)) / (len * len);
}

void s5_path_eval(const s5_path_t *p_p, float x, s5_ref_t *p_ref)
{
    float y2 = 0.0f;

    p_ref->slope = 0.0f;
    if (x <= p_p->x0)
    {
        p_ref->y = p_p->y0;
    }
    else if (x < (p_p->x0 + p_p->l_out))
    {
        s5_ramp(p_p->y0, p_p->a, p_p->l_out, (x - p_p->x0) / p_p->l_out, p_ref,
                &y2);
    }
    else if (x <= p_p->x_back)
    {
        p_ref->y = p_p->a;
    }
    else if (x < (p_p->x_back + p_p->l_in))
    {
        s5_ramp(p_p->a, p_p->y_end, p_p->l_in, (x - p_p->x_back) / p_p->l_in,
                p_ref, &y2);
    }
    else
    {
        p_ref->y = p_p->y_end;
    }
    {
        const float q = 1.0f + (p_ref->slope * p_ref->slope);

        p_ref->curv = y2 / (q * s5_sqrtf(q));
    }
}

float s5_path_end(const s5_path_t *p_p)
{
    return p_p->x_back + p_p->l_in;
}

/* Separating axes: the car (a rectangle at the axle pose) against the
 * box grown by margin (axis-aligned in the line frame). */
static bool s5_car_hits(float x, float y, float c, float s,
                        const s5_box_t *p_box, float margin)
{
    const float hl = 0.5f * (float)(S5_CAR_FRONT_MM + S5_CAR_REAR_MM);
    const float hw = (float)S5_CAR_HALF_WIDTH_MM;
    const float off = 0.5f * (float)(S5_CAR_FRONT_MM - S5_CAR_REAR_MM);
    const float cx = x + (off * c);
    const float cy = y + (off * s);
    const float bx0 = p_box->x_near - margin;
    const float bx1 = p_box->x_far + margin;
    const float by0 = p_box->y_right - margin;
    const float by1 = p_box->y_left + margin;
    const float bcx = 0.5f * (bx0 + bx1);
    const float bcy = 0.5f * (by0 + by1);
    const float bhx = 0.5f * (bx1 - bx0);
    const float bhy = 0.5f * (by1 - by0);
    const float ac = s5_absf(c);
    const float as = s5_absf(s);
    float ext;
    float d;

    /* line-frame x and y */
    ext = (hl * ac) + (hw * as);
    if (((cx + ext) < bx0) || ((cx - ext) > bx1))
    {
        return false;
    }
    ext = (hl * as) + (hw * ac);
    if (((cy + ext) < by0) || ((cy - ext) > by1))
    {
        return false;
    }
    /* the car's own axes */
    d = ((bcx - cx) * c) + ((bcy - cy) * s);
    if (s5_absf(d) > (hl + (bhx * ac) + (bhy * as)))
    {
        return false;
    }
    d = ((cx - bcx) * s) + ((bcy - cy) * c);
    if (s5_absf(d) > (hw + (bhx * as) + (bhy * ac)))
    {
        return false;
    }
    return true;
}

bool s5_path_car_hits(float x, float y, float psi, const s5_box_t *p_box,
                      float margin)
{
    return s5_car_hits(x, y, s5_cosf(psi), s5_sinf(psi), p_box, margin);
}

bool s5_path_clear(const s5_path_t *p_p, const s5_box_t *p_box, float margin,
                   float x_from, float x_to)
{
    /* Only where the outline can reach the box: no part of the car is
     * further ahead of the axle than S5_REACH_FRONT_MM, nor further
     * behind than S5_REACH_REAR_MM, whatever its heading. */
    const float lo = p_box->x_near - margin - S5_REACH_FRONT_MM;
    const float hi = p_box->x_far + margin + S5_REACH_REAR_MM;
    float x = (x_from > lo) ? x_from : lo;
    float end = (x_to < hi) ? x_to : hi;
    bool b_last = false;

    if (x > end)
    {
        return true; /* the car never gets near it */
    }
    x_to = end;

    for (;;)
    {
        s5_ref_t r;
        float c;

        s5_path_eval(p_p, x, &r);
        /* heading from the slope: cos = 1 / sqrt(1 + slope^2) */
        c = 1.0f / s5_sqrtf(1.0f + (r.slope * r.slope));
        if (s5_car_hits(x, r.y, c, r.slope * c, p_box, margin))
        {
            return false;
        }
        if (b_last)
        {
            break;
        }
        x += S5_CHECK_MM;
        if (x >= x_to)
        {
            x = x_to;
            b_last = true;
        }
    }
    return true;
}

float s5_path_steer(const s5_ref_t *p_ref, float y, float psi, float s0)
{
    const float psi_ref = s5_atanf(p_ref->slope);
    const float e_y = (y - p_ref->y) * s5_cosf(psi_ref);
    const float e_psi = s5_wrapf(psi - psi_ref);
    const float lim = (float)S5_MAX_CURV_PPM * 1.0e-6f;
    /* Head back to the path at atan(e_y / 2 s0), at most 45 deg, and
     * turn towards that heading at 2 / s0: linearised, both errors die
     * out as (1 + s / s0) exp(-s / s0), critically damped, over the
     * distance s driven. */
    const float want = s5_clampf(-s5_atanf(e_y / (2.0f * s0)), -S5_APPROACH_MAX,
                                 S5_APPROACH_MAX);
    const float k = p_ref->curv + ((2.0f / s0) * s5_sinf(want - e_psi));

    return s5_clampf(k, -lim, lim);
}

void s5_path_wheels(float v, float curv, int16_t *p_l, int16_t *p_r)
{
    const float half = 0.5f * s5_odom_track_mm();
    const float minw = (float)S5_MIN_WHEEL_MM_S;
    float l = v * (1.0f - (curv * half));
    float r = v * (1.0f + (curv * half));

    if (v > 0.0f)
    {
        l = (l < minw) ? minw : l;
        r = (r < minw) ? minw : r;
    }
    else if (v < 0.0f)
    {
        l = (l > -minw) ? -minw : l;
        r = (r > -minw) ? -minw : r;
    }
    else
    {
        l = 0.0f;
        r = 0.0f;
    }
    *p_l = (int16_t)s5_roundf(l);
    *p_r = (int16_t)s5_roundf(r);
}
