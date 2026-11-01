/**
 * @file    obstacle_geometry.c
 * @brief   Polar samples -> obstacle profile -> bypass decision.
 *
 * This file contains no hardware access, no kernel calls, no timing
 * and no global mutable state. Everything is a pure function of its
 * arguments, which is why it can be compiled and tested on a laptop
 * (see test/test_geometry.c). Keeping the maths and the policy out of
 * the task file is the main reason the subsystem stays readable.
 *
 * All arithmetic is integer. The Cortex-M0+ has no FPU, so a single
 * float multiply costs roughly 50 cycles of library code and drags in
 * about 2 kB of soft-float routines.
 */

#include "obstacle.h"

#include "board_config.h"

#include <stddef.h>

/*=====================================================================*/
/* Tunable model constants                                             */
/*=====================================================================*/

/** A sample belongs to the same object if it is within this much of
 *  the nearest return. Larger values merge nearby objects. */
#define OBS_DEPTH_TOL_MM        (120)

/** Returns further than this are treated as background, not object. */
#define OBS_BACKGROUND_MM       (900)

/** Range jump between angular neighbours that marks an object edge.
 *  Region growing stops here, which is what stops a wall two metres
 *  away from being merged into the box on the line. */
#define OBS_EDGE_STEP_MM        (100)

/**
 * The HC-SR04 beam is roughly 15 degrees wide, so an object appears
 * wider than it is by about 2 * d * tan(7.5 deg) = 0.264 * d.
 * Stored as Q10: 0.264 * 1024 = 270. Calibrate with test G3.
 */
#define OBS_BEAM_CORR_Q10       (270)

/** Never report a width below this after correction. */
#define OBS_MIN_WIDTH_MM        (30)

/** Lateral corridor the chassis needs to pass an object. */
#define OBS_CORRIDOR_MM         (((int32_t)BOARD_HALF_WIDTH_MM * 2) + \
                                 (int32_t)OBSTACLE_SAFETY_MARGIN_MM)

/** Half-width of the band the car will actually drive through. Only a
 *  return inside this band can be "the obstacle"; anything outside it
 *  is scenery that limits the bypass but is not in the way. */
#define OBS_PATH_HALF_MM        ((int32_t)BOARD_HALF_WIDTH_MM + \
                                 (int32_t)OBSTACLE_SAFETY_MARGIN_MM)

/** A sweep with no obstacle is only trusted as clear at or above this
 *  confidence. Each silent (UNKNOWN) bearing costs its share. */
#define OBS_CLEAR_MIN_CONF      (80)

/**
 * Bearings inside this cone look down the driving path. The path band
 * (+/- OBS_PATH_HALF_MM = 135 mm) at the trigger distance (300 mm)
 * subtends atan(135 / 300) = 24 degrees, rounded up. A silent bearing
 * here is a blind spot exactly where an obstacle would be, so the
 * sweep cannot be called clear no matter how many others answered.
 */
#define OBS_PATH_CONE_DEG       (25)

/*=====================================================================*/
/* Fixed-point trigonometry                                            */
/*=====================================================================*/

#define SIN_STEP_DEG            (5)
#define SIN_ONE_Q15             (32768)

/** sin(x) in Q15 for x = 0, 5, 10 ... 90 degrees. 38 bytes of flash. */
static const uint16_t g_sin_q15[19] =
{
        0U,  2856U,  5690U,  8481U, 11207U, 13848U, 16384U,
    18795U, 21062U, 23170U, 25100U, 26842U, 28378U, 29697U,
    30792U, 31651U, 32270U, 32643U, 32768U
};

/**
 * @brief   sin(deg) in Q15, linearly interpolated between table steps.
 * @param   deg  -180..180. Values outside are clamped.
 * @return  -32768..32768.
 *
 * Interpolation error against the true sine is below 0.05 %, which is
 * far smaller than the ranging error, so a bigger table would buy
 * nothing.
 */
static int32_t obs_sin_q15(int16_t deg)
{
    int32_t sign = 1;
    int32_t a    = (int32_t)deg;
    int32_t idx;
    int32_t frac;
    int32_t lo;
    int32_t hi;

    if (a < 0)
    {
        a    = -a;
        sign = -1;
    }

    if (a > 180)
    {
        a = 180;
    }

    if (a > 90)
    {
        a = 180 - a;
    }

    idx  = a / SIN_STEP_DEG;
    frac = a % SIN_STEP_DEG;
    lo   = (int32_t)g_sin_q15[idx];
    hi   = (frac == 0) ? lo : (int32_t)g_sin_q15[idx + 1];

    return sign * (lo + (((hi - lo) * frac) / SIN_STEP_DEG));
}

/**
 * @brief   Lateral offset of a polar sample. Positive is to the left.
 */
static int32_t obs_lateral_mm(int16_t bearing_deg, uint16_t range_mm)
{
    return ((int32_t)range_mm * obs_sin_q15(bearing_deg)) / SIN_ONE_Q15;
}

/**
 * @brief   True if this sample carries a usable range.
 */
static bool obs_is_valid(uint16_t range_mm)
{
    return ((range_mm != OBSTACLE_RANGE_UNKNOWN) &&
            (range_mm != OBSTACLE_RANGE_CLEAR));
}

/*=====================================================================*/
/* Profile construction                                                */
/*=====================================================================*/

/**
 * @brief   Clear a profile to the safe default: not valid.
 */
static void obs_profile_reset(obstacle_profile_t *p_out)
{
    p_out->valid           = false;
    p_out->nearest_mm      = OBSTACLE_RANGE_UNKNOWN;
    p_out->bearing_deg     = 0;
    p_out->width_mm        = 0U;
    p_out->left_edge_deg   = 0;
    p_out->right_edge_deg  = 0;
    p_out->gap_left_mm     = 0U;
    p_out->gap_right_mm    = 0U;
    p_out->swing_left_mm   = 0U;
    p_out->swing_right_mm  = 0U;
    p_out->confidence_pct  = 0U;
    p_out->sample_count    = 0U;
    p_out->timestamp_ms    = 0U;
    p_out->action          = OBSTACLE_ACT_STOP;
}

/**
 * @brief   Index of the closest usable return INSIDE THE DRIVING PATH,
 *          or count if nothing is in the path.
 *
 * Picking the nearest return anywhere in the sweep is wrong: a wall
 * 170 mm off to the side at -75 degrees is closer than a box 177 mm
 * dead ahead, but only the box is in the way. Bench sweep E1 caught
 * exactly that (see test G9).
 */
static uint8_t obs_find_nearest(const obstacle_sample_t *p_samples,
                                uint8_t count,
                                uint16_t *p_nearest_mm)
{
    uint8_t  best_idx = count;
    uint16_t best_mm  = 0U;
    uint8_t  i;

    for (i = 0U; i < count; i++)
    {
        const uint16_t r = p_samples[i].range_mm;

        if (obs_is_valid(r) && (r < (uint16_t)OBS_BACKGROUND_MM))
        {
            int32_t x = obs_lateral_mm(p_samples[i].bearing_deg, r);

            if (x < 0)
            {
                x = -x;
            }

            if ((x <= OBS_PATH_HALF_MM) &&
                ((best_idx == count) || (r < best_mm)))
            {
                best_mm  = r;
                best_idx = i;
            }
        }
    }

    *p_nearest_mm = best_mm;

    return best_idx;
}

/**
 * @brief   Does this return continue the surface we are tracing?
 * @param   range_mm     Candidate neighbour.
 * @param   prev_mm      Last return accepted on this side.
 * @param   nearest_mm   Closest point of the object.
 *
 * Two gates. The step gate catches a sudden jump in range, which is
 * the edge of a body. The depth gate stops a long shallow slope from
 * growing the object indefinitely. A silent bearing fails both, so
 * the object never grows across a gap in the data.
 */
static bool obs_same_object(uint16_t range_mm,
                            uint16_t prev_mm,
                            uint16_t nearest_mm)
{
    bool same = false;

    if (obs_is_valid(range_mm) &&
        (range_mm < (uint16_t)OBS_BACKGROUND_MM) &&
        (range_mm <= (uint16_t)(nearest_mm + OBS_DEPTH_TOL_MM)))
    {
        const int32_t step = (int32_t)range_mm - (int32_t)prev_mm;

        same = ((step <= OBS_EDGE_STEP_MM) && (step >= -OBS_EDGE_STEP_MM));
    }

    return same;
}

/**
 * @brief   Base confidence minus the share of bearings that were silent.
 */
static uint8_t obs_confidence(int32_t base, uint8_t unknowns, uint8_t count)
{
    int32_t c = base - (((int32_t)unknowns * 100) / (int32_t)count);

    if (c < 0)   { c = 0; }
    if (c > 100) { c = 100; }

    return (uint8_t)c;
}

void obstacle_profile_build(const obstacle_sample_t *p_samples,
                            uint8_t count,
                            obstacle_profile_t *p_out)
{
    /* Reset first so a rejected call still leaves the caller with the
     * safe default (not valid, action STOP). */
    if (p_out != NULL)
    {
        obs_profile_reset(p_out);
    }

    if ((p_samples != NULL) && (p_out != NULL) &&
        (count > 0U) && (count <= (uint8_t)OBSTACLE_MAX_SAMPLES))
    {
        uint16_t nearest     = 0U;
        uint8_t  unknowns    = 0U;
        bool     blind_ahead = false;
        uint8_t  centre;
        uint8_t  i;

        p_out->sample_count = count;

        for (i = 0U; i < count; i++)
        {
            if (p_samples[i].range_mm == OBSTACLE_RANGE_UNKNOWN)
            {
                unknowns++;
            }
        }

        centre = obs_find_nearest(p_samples, count, &nearest);

        for (i = 0U; i < count; i++)
        {
            const int16_t b = p_samples[i].bearing_deg;

            if ((p_samples[i].range_mm == OBSTACLE_RANGE_UNKNOWN) &&
                (b <= OBS_PATH_CONE_DEG) && (b >= -OBS_PATH_CONE_DEG))
            {
                blind_ahead = true;
            }
        }

        if ((unknowns == count) || ((centre >= count) && blind_ahead))
        {
            /* Every bearing silent, or nothing found but a blind spot
             * straight ahead. Either way we cannot say the path is
             * clear, so the profile stays invalid and the planner says
             * STOP. Before this check a dead sensor produced
             * "CONTINUE, 100 %". */
        }
        else if (centre >= count)
        {
            /* Nothing in the driving path. Only as trustworthy as the
             * share of bearings that actually answered. */
            p_out->valid          = true;
            p_out->nearest_mm     = OBSTACLE_RANGE_CLEAR;
            p_out->gap_left_mm    = (uint16_t)OBSTACLE_GAP_OPEN_MM;
            p_out->gap_right_mm   = (uint16_t)OBSTACLE_GAP_OPEN_MM;
            p_out->confidence_pct = obs_confidence(100, unknowns, count);
        }
        else
        {
            uint8_t  lo_idx = centre;
            uint8_t  hi_idx = centre;
            uint16_t prev   = nearest;
            bool     grow;
            bool     left_unknown  = false;
            bool     right_unknown = false;
            int32_t  min_x       = 0;
            int32_t  max_x       = 0;
            int32_t  bearing_sum = 0;
            int32_t  block_left  = (int32_t)OBSTACLE_GAP_OPEN_MM;
            int32_t  block_right = -(int32_t)OBSTACLE_GAP_OPEN_MM;
            int32_t  gap_l;
            int32_t  gap_r;
            int32_t  width;
            int32_t  swing_l;
            int32_t  swing_r;
            int32_t  conf = 100;
            uint8_t  hits;

            /* Grow the object outwards from its closest in-path point
             * until the surface breaks. */
            grow = true;
            while (grow && ((hi_idx + 1U) < count))
            {
                const uint16_t r = p_samples[hi_idx + 1U].range_mm;

                if (obs_same_object(r, prev, nearest))
                {
                    hi_idx++;
                    prev = r;
                }
                else
                {
                    grow = false;
                }
            }

            grow = true;
            prev = nearest;
            while (grow && (lo_idx > 0U))
            {
                const uint16_t r = p_samples[lo_idx - 1U].range_mm;

                if (obs_same_object(r, prev, nearest))
                {
                    lo_idx--;
                    prev = r;
                }
                else
                {
                    grow = false;
                }
            }

            hits = (uint8_t)((hi_idx - lo_idx) + 1U);

            /* Extent of the object, and its true closest point (growth
             * may have reached a return closer than the seed). */
            for (i = lo_idx; i <= hi_idx; i++)
            {
                const uint16_t r = p_samples[i].range_mm;
                const int32_t  x =
                    obs_lateral_mm(p_samples[i].bearing_deg, r);

                if (i == lo_idx)
                {
                    min_x = x;
                    max_x = x;
                }
                else
                {
                    if (x < min_x) { min_x = x; }
                    if (x > max_x) { max_x = x; }
                }

                if (r < nearest)
                {
                    nearest = r;
                }

                bearing_sum += (int32_t)p_samples[i].bearing_deg;
            }

            /* Everything outside the object limits the bypass. A silent
             * bearing on a side means we cannot see that corridor, so
             * we refuse to plan through it. */
            for (i = 0U; i < count; i++)
            {
                if ((i < lo_idx) || (i > hi_idx))
                {
                    const uint16_t r = p_samples[i].range_mm;

                    if (r == OBSTACLE_RANGE_UNKNOWN)
                    {
                        if (i > hi_idx)
                        {
                            left_unknown = true;
                        }
                        else
                        {
                            right_unknown = true;
                        }
                    }
                    else if (obs_is_valid(r))
                    {
                        const int32_t x =
                            obs_lateral_mm(p_samples[i].bearing_deg, r);

                        if ((x > max_x) && (x < block_left))
                        {
                            block_left = x;
                        }
                        if ((x < min_x) && (x > block_right))
                        {
                            block_right = x;
                        }
                    }
                    else
                    {
                        /* CLEAR: sensor answered, nothing there. */
                    }
                }
            }

            width = (max_x - min_x) -
                    (((int32_t)nearest * OBS_BEAM_CORR_Q10) / 1024);

            if (width < OBS_MIN_WIDTH_MM)
            {
                width = OBS_MIN_WIDTH_MM;
            }

            gap_l = block_left - max_x;
            gap_r = min_x - block_right;

            if (left_unknown)  { gap_l = 0; }
            if (right_unknown) { gap_r = 0; }

            if (gap_l < 0) { gap_l = 0; }
            if (gap_r < 0) { gap_r = 0; }
            if (gap_l > (int32_t)OBSTACLE_GAP_OPEN_MM)
            {
                gap_l = (int32_t)OBSTACLE_GAP_OPEN_MM;
            }
            if (gap_r > (int32_t)OBSTACLE_GAP_OPEN_MM)
            {
                gap_r = (int32_t)OBSTACLE_GAP_OPEN_MM;
            }

            swing_l = max_x + OBS_PATH_HALF_MM;
            swing_r = -min_x + OBS_PATH_HALF_MM;

            if (swing_l < 0) { swing_l = 0; }
            if (swing_r < 0) { swing_r = 0; }

            /* An object running off the end of the sweep has an edge we
             * never saw. */
            if (hi_idx == (count - 1U)) { conf -= 25; }
            if (lo_idx == 0U)           { conf -= 25; }
            if (hits < 2U)              { conf -= 20; }

            p_out->valid          = true;
            p_out->nearest_mm     = nearest;
            p_out->bearing_deg    =
                (int16_t)(bearing_sum / (int32_t)hits);
            p_out->width_mm       = (uint16_t)width;
            p_out->left_edge_deg  = p_samples[hi_idx].bearing_deg;
            p_out->right_edge_deg = p_samples[lo_idx].bearing_deg;
            p_out->gap_left_mm    = (uint16_t)gap_l;
            p_out->gap_right_mm   = (uint16_t)gap_r;
            p_out->swing_left_mm  = (uint16_t)swing_l;
            p_out->swing_right_mm = (uint16_t)swing_r;
            p_out->confidence_pct = obs_confidence(conf, unknowns, count);
        }

        p_out->action = obstacle_plan(p_out);
    }
}

/*=====================================================================*/
/* Avoidance policy                                                    */
/*=====================================================================*/

obstacle_action_t obstacle_plan(const obstacle_profile_t *p_profile)
{
    obstacle_action_t action = OBSTACLE_ACT_STOP;

    if ((p_profile != NULL) && p_profile->valid)
    {
        if (p_profile->nearest_mm == OBSTACLE_RANGE_CLEAR)
        {
            /* "Nothing seen" only counts as clear if enough of the
             * sweep actually answered. */
            if (p_profile->confidence_pct >= (uint8_t)OBS_CLEAR_MIN_CONF)
            {
                action = OBSTACLE_ACT_CONTINUE;
            }
            else
            {
                action = OBSTACLE_ACT_SLOW;
            }
        }
        else if (p_profile->nearest_mm > (uint16_t)OBSTACLE_TRIGGER_MM)
        {
            action = OBSTACLE_ACT_CONTINUE;
        }
        else if (p_profile->nearest_mm < (uint16_t)OBSTACLE_HARD_STOP_MM)
        {
            /* Inside the turning circle: back off and scan again. */
            action = OBSTACLE_ACT_REVERSE;
        }
        else if (p_profile->confidence_pct < 40U)
        {
            action = OBSTACLE_ACT_SLOW;
        }
        else
        {
            const bool left_ok =
                ((int32_t)p_profile->gap_left_mm >= OBS_CORRIDOR_MM);
            const bool right_ok =
                ((int32_t)p_profile->gap_right_mm >= OBS_CORRIDOR_MM);

            if (left_ok && right_ok)
            {
                /* Both fit: take the one that deviates least. */
                if (p_profile->swing_left_mm <= p_profile->swing_right_mm)
                {
                    action = OBSTACLE_ACT_BYPASS_LEFT;
                }
                else
                {
                    action = OBSTACLE_ACT_BYPASS_RIGHT;
                }
            }
            else if (left_ok)
            {
                action = OBSTACLE_ACT_BYPASS_LEFT;
            }
            else if (right_ok)
            {
                action = OBSTACLE_ACT_BYPASS_RIGHT;
            }
            else
            {
                action = OBSTACLE_ACT_REVERSE;
            }
        }
    }

    return action;
}
