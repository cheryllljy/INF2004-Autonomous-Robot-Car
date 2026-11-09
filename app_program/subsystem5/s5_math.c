/*
 *      s5_math.c
 *      Subsystem 5 -- float helpers without libm (nyx, 2026-11-07).
 *      See s5_math.h.
 */

#include "s5_math.h"

#define S5_HALF_PI_F (1.57079633f)
#define S5_TWO_PI_F  (6.28318531f)

float s5_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

float s5_clampf(float x, float lo, float hi)
{
    float r = x;

    if (r < lo)
    {
        r = lo;
    }
    if (r > hi)
    {
        r = hi;
    }
    return r;
}

float s5_wrapf(float a)
{
    float r = a;
    uint32_t guard = 0U;

    while ((r > S5_PI_F) && (guard < 64U))
    {
        r -= S5_TWO_PI_F;
        guard++;
    }
    while ((r <= -S5_PI_F) && (guard < 64U))
    {
        r += S5_TWO_PI_F;
        guard++;
    }
    return r;
}

float s5_sinf(float a)
{
    float x = s5_wrapf(a);
    float x2;

    /* fold onto -pi/2 .. pi/2, where the series converges fast */
    if (x > S5_HALF_PI_F)
    {
        x = S5_PI_F - x;
    }
    else if (x < -S5_HALF_PI_F)
    {
        x = -S5_PI_F - x;
    }
    else
    {
        /* already there */
    }
    x2 = x * x;
    /* Taylor to x^9: error under 4e-6 at pi/2 */
    return x * (1.0f -
                (x2 / 6.0f) *
                    (1.0f - (x2 / 20.0f) *
                                (1.0f - (x2 / 42.0f) * (1.0f - (x2 / 72.0f)))));
}

float s5_cosf(float a)
{
    return s5_sinf(a + S5_HALF_PI_F);
}

float s5_tanf(float a)
{
    float c = s5_cosf(a);

    if (s5_absf(c) < 0.0872f) /* beyond 85 deg: hold at 85 */
    {
        c = (c < 0.0f) ? -0.0872f : 0.0872f;
    }
    return s5_sinf(a) / c;
}

float s5_atanf(float x)
{
    float r;
    const float ax = s5_absf(x);
    const float t = (ax > 1.0f) ? (1.0f / ax) : ax;
    const float t2 = t * t;

    /* minimax on 0..1, error about 1e-5 */
    r = t * (0.99997726f +
             t2 * (-0.33262347f +
                   t2 * (0.19354346f +
                         t2 * (-0.11643287f +
                               t2 * (0.05265332f + t2 * -0.01172120f)))));
    if (ax > 1.0f)
    {
        r = S5_HALF_PI_F - r;
    }
    return (x < 0.0f) ? -r : r;
}

float s5_atan2f(float y, float x)
{
    float r;

    if (0.0f == x)
    {
        if (y > 0.0f)
        {
            r = S5_HALF_PI_F;
        }
        else if (y < 0.0f)
        {
            r = -S5_HALF_PI_F;
        }
        else
        {
            r = 0.0f;
        }
    }
    else
    {
        r = s5_atanf(y / x);
        if (x < 0.0f)
        {
            r += (y < 0.0f) ? -S5_PI_F : S5_PI_F;
        }
    }
    return r;
}

float s5_sqrtf(float x)
{
    union
    {
        float f;
        uint32_t u;
    } v;
    float r;
    uint32_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }
    v.f = x;
    v.u = 0x1FBD1DF5U + (v.u >> 1); /* first guess from the exponent */
    r = v.f;
    for (i = 0U; i < 3U; i++)
    {
        r = 0.5f * (r + (x / r));
    }
    return r;
}

int32_t s5_roundf(float x)
{
    return (x < 0.0f) ? (int32_t)(x - 0.5f) : (int32_t)(x + 0.5f);
}
