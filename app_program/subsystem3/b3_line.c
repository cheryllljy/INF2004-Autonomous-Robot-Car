#include "b3_line.h"
#include <string.h>

void b3_line_init(b3_line_t *p_line, uint32_t now_us)
{
    (void)memset(p_line, 0, sizeof(*p_line));
    p_line->left.changed_us = now_us;
    p_line->right.changed_us = now_us;
    p_line->last_seen_us = now_us;
    p_line->kick = B3_STEER_KICK_CURV;
}

void b3_line_hold(b3_line_t *p_line)
{
    p_line->memory = 0;
    p_line->steer = 0;
    p_line->black_um = 0U;
    /* the nudge stays tuned: it depends on the sensors, not the track */
    p_line->last_touch = 0;
}

/* a new touch on side `side` (-1 / +1) */
static void tune_kick(b3_line_t *p_line, int8_t side)
{
    int64_t kick = p_line->kick;

    if ((0 != p_line->last_touch) &&
        (p_line->touch_um < (B3_STEER_OVERSHOOT_MM * 1000U)))
    {
        if (p_line->last_touch != side)
        {
            kick = (kick * B3_STEER_SHRINK_PCT) / 100;
        }
        else
        {
            kick = (kick * B3_STEER_GROW_PCT) / 100;
        }
    }
    else if (p_line->last_touch == side)
    {
        kick = (kick * B3_STEER_GROW_PCT) / 100;
    }
    else
    {
        /* first touch, or long after the last one: keep it */
    }
    if (kick < B3_STEER_KICK_MIN_CURV)
    {
        kick = B3_STEER_KICK_MIN_CURV;
    }
    if (kick > B3_STEER_KICK_CURV)
    {
        kick = B3_STEER_KICK_CURV;
    }
    p_line->kick = (int32_t)kick;
    p_line->last_touch = side;
    p_line->touch_um = 0U;
}

static bool filtered(b3_filter_t *p_filter, bool sample, uint32_t now_us)
{
    if (sample != p_filter->candidate)
    {
        p_filter->candidate = sample;
        p_filter->changed_us = now_us;
    }
    if ((now_us - p_filter->changed_us) >= B3_LINE_DEBOUNCE_US)
    {
        p_filter->stable = p_filter->candidate;
    }
    return p_filter->stable;
}

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t clamp32(int32_t v, int32_t limit)
{
    if (v > limit)
    {
        v = limit;
    }
    if (v < -limit)
    {
        v = -limit;
    }
    return v;
}

/* line speed for a curve: from the base speed down to
 * B3_CURVE_SPEED_MM_S as |curv| reaches B3_CURVE_FULL_CURV. */
static int32_t curve_speed(int32_t base, int32_t curv)
{
    int32_t mag = abs32(curv);

    if (base <= B3_CURVE_SPEED_MM_S)
    {
        return base;
    }
    if (mag > B3_CURVE_FULL_CURV)
    {
        mag = B3_CURVE_FULL_CURV;
    }
    return base - (((base - B3_CURVE_SPEED_MM_S) * mag) / B3_CURVE_FULL_CURV);
}

/* Tightest curve at speed v that keeps the inner wheel at
 * B3_MIN_WHEEL_MM_S: (v - min) / v = curv x track / 2e6. */
static int32_t curve_limit(int32_t v)
{
    if (v <= B3_MIN_WHEEL_MM_S)
    {
        return 0;
    }
    return (int32_t)(((int64_t)2000000 * (v - B3_MIN_WHEEL_MM_S)) /
                     ((int64_t)v * B3_WHEEL_TRACK_MM));
}

b3_line_result_t b3_line_step(b3_line_t *p_line, bool left_black,
                              bool right_black, uint32_t now_us,
                              int16_t base_mm_s)
{
    b3_line_result_t result = { 0 };
    int32_t base = base_mm_s;
    uint32_t dt_us = 0U;
    int32_t ds_um;
    int32_t target;
    int32_t limit;
    int32_t v;
    int32_t corr;
    bool escape;
    bool bend_bb;

    result.left_black = filtered(&p_line->left, left_black, now_us);
    result.right_black = filtered(&p_line->right, right_black, now_us);
    if (base < 0)
    {
        base = 0;
    }
    if (base > B3_MAX_REQUEST_MM_S)
    {
        base = B3_MAX_REQUEST_MM_S;
    }
    /* Straddle layout: white on both sides is on track. A black side means
     * the line has slid under that sensor, so steer back towards it. */
    if (result.left_black && result.right_black)
    {
        result.state = B3_LINE_CROSSING;
        result.error = 0;
    }
    else if (result.left_black)
    {
        result.state = B3_LINE_LEFT;
        result.error = -1;
    }
    else if (result.right_black)
    {
        result.state = B3_LINE_RIGHT;
        result.error = 1;
    }
    else
    {
        result.state = B3_LINE_CENTRED;
        result.error = 0;
    }
    if (result.left_black || result.right_black)
    {
        p_line->ever_seen = true;
        p_line->last_seen_us = now_us;
    }
    p_line->last_error = result.error;

    if (p_line->started)
    {
        dt_us = now_us - p_line->last_us;
        if (dt_us > B3_STEER_MAX_DT_US)
        {
            dt_us = B3_STEER_MAX_DT_US;
        }
    }
    p_line->started = true;
    p_line->last_us = now_us;
    /* both black while a curve is learnt is the bend's
     * tape seen at a steep angle (or a crossing in the bend), not a cue to
     * drive straight: keep the curve (B3_STEER_BB_* in b3_config.h). Not
     * when the side against the curve was black: that is the curve ending,
     * handled below as before. */
    bend_bb = (B3_LINE_CROSSING == result.state) &&
              (abs32(p_line->memory) >= B3_STEER_BB_CURV) &&
              (((int64_t)p_line->side * p_line->memory) <= 0);
    if ((!bend_bb) && (result.error != p_line->side))
    {
        if ((0 == result.error) &&
            (((int64_t)p_line->side * p_line->memory) > 0) &&
            (p_line->black_um >= (B3_STEER_CURVE_END_MM * 1000U)))
        {
            /* the side against the learnt curve stayed
             * black, so the curve has ended: keep the steering it left
             * instead of turning back to the old curve */
            p_line->memory = p_line->steer;
        }
        p_line->side = result.error;
        p_line->side_since_us = now_us;
        p_line->black_um = 0U;
        if (0 != result.error)
        {
            tune_kick(p_line, result.error);
        }
    }

    /* distance driven since the last step, at the speed asked then */
    ds_um =
        (int32_t)(((int64_t)curve_speed(base, p_line->memory) * dt_us) / 1000);
    if (p_line->touch_um < (B3_STEER_OVERSHOOT_MM * 1000U))
    {
        p_line->touch_um += (uint32_t)ds_um;
    }
    /* how long both have been black */
    if (B3_LINE_CROSSING != result.state)
    {
        p_line->bb_um = 0U;
    }
    else if (p_line->bb_um < (B3_STEER_BB_CROSS_MM * 1000U))
    {
        p_line->bb_um += (uint32_t)ds_um;
    }
    else
    {
        /* long enough: no need to count on (no overflow) */
    }

    /* curve memory and nudge (B3_STEER_* in b3_config.h).
     * error -1 = IR_L black = steer left = + curvature. */
    if (bend_bb)
    {
        /* the learnt curve stays (no fading, no
         * learning). The black side's run is broken, so a side black again
         * afterwards learns, and pivots (B3_STEER_ESCAPE_MM), from 0 as
         * before. Longer than any crossing, the tape runs across the pair
         * because the line turns tighter than the car: nudge to the inside
         * of the curve as a black inner side would. */
        target = p_line->memory;
        p_line->black_um = 0U;
        if (p_line->bb_um >= (B3_STEER_BB_CROSS_MM * 1000U))
        {
            target += (p_line->memory > 0) ? p_line->kick : -p_line->kick;
        }
    }
    else if (0 != result.error)
    {
        p_line->black_um += (uint32_t)ds_um;
        if (((int64_t)result.error * p_line->memory) > 0)
        {
            /* the black side is against the learnt
             * curve: forget it at least as fast as with both sides white */
            p_line->memory -= (int32_t)(((int64_t)p_line->memory * ds_um) /
                                        ((int64_t)B3_STEER_FORGET_MM * 1000));
        }
        /* learns faster the longer the side stays black: a touch is a
         * nudge, a bend the car is not turning enough for is learnt
         * quickly */
        {
            /* faster when the black side asks for more of a tight curve
             * already learnt (B3_STEER_LEARN_TIGHT_CURV) */
            const int64_t rate =
                ((((int64_t)result.error * p_line->memory) < 0) &&
                 (abs32(p_line->memory) >= B3_STEER_TIGHT_FROM_CURV))
                    ? B3_STEER_LEARN_TIGHT_CURV
                    : B3_STEER_LEARN_CURV;

            p_line->memory -=
                (int32_t)(((int64_t)result.error * rate * ds_um *
                           ((B3_STEER_ESCAPE_MM * 1000) +
                            (int64_t)p_line->black_um)) /
                          ((int64_t)10000 * B3_STEER_ESCAPE_MM * 1000));
        }
        /* a touch gets the tuned nudge; a side that stays black grows it
         * back to the full nudge over B3_STEER_KICK_RAMP_MM */
        int64_t kick = p_line->kick;

        if (p_line->black_um >= (B3_STEER_KICK_RAMP_MM * 1000U))
        {
            kick = B3_STEER_KICK_CURV;
        }
        else
        {
            kick += ((B3_STEER_KICK_CURV - kick) * (int64_t)p_line->black_um) /
                    ((int64_t)B3_STEER_KICK_RAMP_MM * 1000);
        }
        target = p_line->memory - (result.error * (int32_t)kick);
    }
    else
    {
        /* fades towards straight; rounded away from 0 so it gets there */
        const int64_t span = (int64_t)B3_STEER_FORGET_MM * 1000;
        int64_t fade = ((int64_t)p_line->memory * ds_um) / span;

        if ((0 == fade) && (0 != p_line->memory) && (ds_um > 0))
        {
            fade = (p_line->memory > 0) ? 1 : -1;
        }
        p_line->memory -= (int32_t)fade;
        target = p_line->memory;
    }

    /* the car slows for the curve it has learnt, and the inner wheel keeps
     * B3_MIN_WHEEL_MM_S at that speed */
    v = curve_speed(base, p_line->memory);
    limit = curve_limit(v);
    p_line->memory = clamp32(p_line->memory, limit);
    target = clamp32(target, limit);

    /* towards the target by at most B3_STEER_SLEW_CURV per mm */
    {
        const int32_t step =
            (int32_t)(((int64_t)B3_STEER_SLEW_CURV * ds_um) / 1000);

        if (target > (p_line->steer + step))
        {
            target = p_line->steer + step;
        }
        if (target < (p_line->steer - step))
        {
            target = p_line->steer - step;
        }
    }
    p_line->steer = clamp32(target, limit);

    corr =
        (int32_t)(((int64_t)p_line->steer * v * B3_WHEEL_TRACK_MM) / 2000000);
    escape = (0 != result.error) &&
             (p_line->black_um >= (B3_STEER_ESCAPE_MM * 1000U)) &&
             (abs32(p_line->memory) >= limit) && (v > 0);
    if (escape)
    {
        /* hairpin: stop the inner wheel, pivot on it */
        corr = -result.error * v;
    }
    else if (corr > (v - B3_MIN_WHEEL_MM_S))
    {
        corr = (v > B3_MIN_WHEEL_MM_S) ? (v - B3_MIN_WHEEL_MM_S) : 0;
    }
    else if (corr < -(v - B3_MIN_WHEEL_MM_S))
    {
        corr = (v > B3_MIN_WHEEL_MM_S) ? -(v - B3_MIN_WHEEL_MM_S) : 0;
    }
    else
    {
        /* inside the limits */
    }
    result.sharp_turn = (v < base);
    result.curvature = p_line->steer;
    result.left_mm_s = (int16_t)(v - corr);
    result.right_mm_s = (int16_t)(v + corr);
    return result;
}
