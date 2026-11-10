/*
 *      s5_odom.c
 *      Subsystem 5 -- dead reckoning from the encoder ticks. See s5_odom.h.
 */

#include "s5_odom.h"

#include "../subsystem2/motion_config.h" /* MM_PER_TICK, TRACK_WIDTH_MM */
#include "s5_config.h"
#include "s5_math.h"

/* More than this between two updates (20 ms apart) is not driving: the
 * counters were reset or jumped. Take the new counts, move nothing. */
#define S5_ODOM_JUMP_MM (100.0f)

void s5_odom_init(s5_odom_t *p_o)
{
    p_o->x = 0.0f;
    p_o->y = 0.0f;
    p_o->psi = 0.0f;
    p_o->dist = 0.0f;
    p_o->ticks_l = 0;
    p_o->ticks_r = 0;
    p_o->b_ticks = false;
}

float s5_odom_track_mm(void)
{
    return (TRACK_WIDTH_MM * 100.0f) / (float)S5_TURN_PCT;
}

void s5_odom_update(s5_odom_t *p_o, int32_t ticks_l, int32_t ticks_r)
{
    float dl;
    float dr;
    float ds;
    float dpsi;
    float mid;

    if (!p_o->b_ticks)
    {
        p_o->ticks_l = ticks_l;
        p_o->ticks_r = ticks_r;
        p_o->b_ticks = true;
        return;
    }
    dl = (float)(ticks_l - p_o->ticks_l) * MM_PER_TICK;
    dr = (float)(ticks_r - p_o->ticks_r) * MM_PER_TICK;
    p_o->ticks_l = ticks_l;
    p_o->ticks_r = ticks_r;
    if ((s5_absf(dl) > S5_ODOM_JUMP_MM) || (s5_absf(dr) > S5_ODOM_JUMP_MM))
    {
        return;
    }
    ds = 0.5f * (dl + dr);
    dpsi = (dr - dl) / s5_odom_track_mm();
    mid = p_o->psi + (0.5f * dpsi);
    p_o->x += ds * s5_cosf(mid);
    p_o->y += ds * s5_sinf(mid);
    p_o->psi = s5_wrapf(p_o->psi + dpsi);
    p_o->dist += s5_absf(ds);
}

void s5_odom_set_pose(s5_odom_t *p_o, float x, float y, float psi)
{
    p_o->x = x;
    p_o->y = y;
    p_o->psi = s5_wrapf(psi);
}

void s5_odom_point(const s5_odom_t *p_o, float ahead, float left, float *p_x,
                   float *p_y)
{
    const float c = s5_cosf(p_o->psi);
    const float s = s5_sinf(p_o->psi);

    *p_x = p_o->x + (ahead * c) - (left * s);
    *p_y = p_o->y + (ahead * s) + (left * c);
}
