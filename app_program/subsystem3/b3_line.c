#include "b3_line.h"
#include <string.h>

void b3_line_init(b3_line_t *p_line, uint32_t now_us)
{
    (void)memset(p_line, 0, sizeof(*p_line));
    p_line->left.changed_us = now_us;
    p_line->right.changed_us = now_us;
    p_line->last_seen_us = now_us;
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

b3_line_result_t b3_line_step(b3_line_t *p_line, bool left_black,
                              bool right_black, uint32_t now_us,
                              int16_t base_mm_s)
{
    b3_line_result_t result = { 0 };
    int16_t correction;
    result.left_black = filtered(&p_line->left, left_black, now_us);
    result.right_black = filtered(&p_line->right, right_black, now_us);
    if (base_mm_s < 0)
    {
        base_mm_s = 0;
    }
    if (base_mm_s > B3_MAX_REQUEST_MM_S)
    {
        base_mm_s = B3_MAX_REQUEST_MM_S;
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

    /* Sharp turn: one side black for B3_SHARP_TURN_US. */
    if (result.error != p_line->side)
    {
        p_line->side = result.error;
        p_line->side_since_us = now_us;
    }
    if ((B3_SHARP_TURN_US != 0U) && (0 != result.error) &&
        ((now_us - p_line->side_since_us) >= B3_SHARP_TURN_US))
    {
        p_line->sharp = result.error;
    }
    if (0 != p_line->sharp)
    {
        if (result.error == p_line->sharp)
        {
            p_line->sharp_last_us = now_us;
        }
        else if ((B3_LINE_CROSSING == result.state) || (0 != result.error) ||
                 ((now_us - p_line->sharp_last_us) >= B3_SHARP_HOLD_US))
        {
            p_line->sharp = 0; /* crossing, other side, or straight again */
            p_line->recovering = (B3_LINE_CROSSING != result.state);
            p_line->sharp_end_us = now_us;
        }
    }
    /* Ramp instead of steps (2026-11-04, the car jerked on the figure 8):
     * the longer one side stays black, the slower the car and the harder
     * the correction, reaching a pivot (inner wheel 0) at B3_SHARP_TURN_US.
     * ramp_pm is 0..1000 per mille. */
    {
        int32_t ramp_pm = 0;
        int32_t base = base_mm_s;
        int32_t corr = B3_CORRECTION_MM_S;

        if (0 != p_line->sharp)
        {
            result.sharp_turn = true;
            ramp_pm = 1000;
        }
        else if ((B3_SHARP_TURN_US != 0U) && (0 != result.error))
        {
            uint32_t held = now_us - p_line->side_since_us;
            ramp_pm = (int32_t)(((uint64_t)held * 1000U) / B3_SHARP_TURN_US);
            if (ramp_pm > 1000)
            {
                ramp_pm = 1000;
            }
        }
        if (p_line->recovering && (0 == p_line->sharp))
        {
            /* Ease back up to speed after a sharp turn (B3_SHARP_RECOVER_US). */
            uint32_t since = now_us - p_line->sharp_end_us;
            if ((B3_LINE_CROSSING == result.state) ||
                (since >= B3_SHARP_RECOVER_US))
            {
                p_line->recovering = false;
            }
            else if (base > B3_SHARP_SPEED_MM_S)
            {
                int32_t cap =
                    B3_SHARP_SPEED_MM_S +
                    (int32_t)(((uint64_t)(base - B3_SHARP_SPEED_MM_S) * since) /
                              B3_SHARP_RECOVER_US);
                if (base > cap)
                {
                    base = cap;
                }
            }
        }
        if (base > B3_SHARP_SPEED_MM_S)
        {
            base -= ((base - B3_SHARP_SPEED_MM_S) * ramp_pm) / 1000;
        }
        if (corr > base)
        {
            corr = base;
        }
        corr += ((base - corr) * ramp_pm) / 1000;
        base_mm_s = (int16_t)base;
        if (0 != result.error)
        {
            correction = (int16_t)(result.error * corr);
        }
        else if (result.sharp_turn && (B3_LINE_CROSSING != result.state))
        {
            /* Side white again inside the hold: the line may have
             * crossed under the sensor and out, so keep turning gently
             * towards it instead of driving straight off the curve. */
            correction = (int16_t)(p_line->sharp * B3_CORRECTION_MM_S);
        }
        else
        {
            correction = 0;
        }
    }

    /* Never reverse a wheel in the line-tracking controller. */
    if (correction > base_mm_s)
    {
        correction = base_mm_s;
    }
    if (correction < -base_mm_s)
    {
        correction = (int16_t)-base_mm_s;
    }
    result.left_mm_s = (int16_t)(base_mm_s + correction);
    result.right_mm_s = (int16_t)(base_mm_s - correction);
    return result;
}
