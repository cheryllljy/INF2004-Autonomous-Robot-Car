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
    b3_line_result_t result = {0};
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
    correction = (int16_t)(result.error * B3_CORRECTION_MM_S);
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
