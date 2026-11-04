/**
 * @file    obstacle_geometry.c
 * @brief   Packed polar samples -> obstacle profile -> bypass decision.
 *
 * No hardware, no kernel calls, no timing, no global mutable state.
 * Every function is a pure function of its arguments, which is why the
 * file is compiled and tested on a PC (testing/subsystem5/host).
 *
 * All arithmetic is integer. The Cortex-M0+ has no FPU; one float
 * multiply costs ~50 cycles of library code and pulls in ~2 kB of
 * soft-float routines.
 *
 * CR1 note: the samples arrive packed (16 bits each). They are decoded
 * one at a time into locals with s5_sample_bearing()/s5_sample_range();
 * the sweep is never unpacked into a second array.
 */

#include "obstacle_model.h"
#include "s5_config.h"  /* v3.0: remeasure and watch constants (macros only) */

#include <stddef.h>     /* NULL; this file never includes the kernel */

/*=====================================================================*/
/* Tunable model constants                                             */
/*=====================================================================*/

/** A sample belongs to the same object if within this of the nearest. */
#define OBS_DEPTH_TOL_MM        (120)

/** Returns further than this are background, not object. */
#define OBS_BACKGROUND_MM       (900)

/** Range jump between neighbours that marks an object edge. */
#define OBS_EDGE_STEP_MM        (100)

/**
 * The HC-SR04 beam is ~15 deg wide, so an object looks wider by about
 * 2 * d * tan(7.5 deg) = 0.264 * d. Q10: 0.264 * 1024 = 270.
 * Calibrate with bench test G3 / sim scenario 6.
 */
#define OBS_BEAM_CORR_Q10       (270)

#define OBS_MIN_WIDTH_MM        (30)

/** Lateral corridor the chassis needs to pass an object. */
#define OBS_CORRIDOR_MM         (((int32_t)OBSTACLE_HALF_WIDTH_MM * 2) + \
                                 (int32_t)OBSTACLE_SAFETY_MARGIN_MM)

/** Half-width of the band the car drives through. Only a return inside
 *  it can be "the obstacle"; anything outside is scenery. */
#define OBS_PATH_HALF_MM        ((int32_t)OBSTACLE_HALF_WIDTH_MM + \
                                 (int32_t)OBSTACLE_SAFETY_MARGIN_MM)

/** A sweep with no obstacle is only trusted as clear at or above this. */
#define OBS_CLEAR_MIN_CONF      (80)

/** Bearings looking down the driving path: atan(135 / 300) = 24 deg,
 *  rounded up. A silent bearing here is a blind spot exactly where an
 *  obstacle would be, so the sweep cannot be called clear. */
#define OBS_PATH_CONE_DEG       (25)

/*=====================================================================*/
/* Fixed-point trigonometry                                            */
/*=====================================================================*/

#define SIN_STEP_DEG            (5)
#define SIN_ONE_Q15             (32768)

/** sin(x) in Q15 for x = 0, 5 ... 90 degrees. 38 bytes of flash. */
static const uint16_t g_sin_q15[19] =
{
        0U,  2856U,  5690U,  8481U, 11207U, 13848U, 16384U,
    18795U, 21062U, 23170U, 25100U, 26842U, 28378U, 29697U,
    30792U, 31651U, 32270U, 32643U, 32768U
};

/** sin(deg) in Q15, linear between table steps. Error < 0.05 %. */
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

/** Lateral offset of a polar sample. Positive is to the left. */
static int32_t obs_lateral_mm(int16_t bearing_deg, uint16_t range_mm)
{
    return ((int32_t)range_mm * obs_sin_q15(bearing_deg)) / SIN_ONE_Q15;
}

/** True if this range carries a usable distance. */
static bool obs_is_valid(uint16_t range_mm)
{
    return ((range_mm != OBSTACLE_RANGE_UNKNOWN) &&
            (range_mm != OBSTACLE_RANGE_CLEAR));
}

bool obstacle_in_path(int16_t bearing_deg, uint16_t range_mm)
{
    bool in_path = false;

    if (obs_is_valid(range_mm) && (range_mm < (uint16_t)OBS_BACKGROUND_MM))
    {
        int32_t x = obs_lateral_mm(bearing_deg, range_mm);

        if (x < 0)
        {
            x = -x;
        }

        in_path = (x <= OBS_PATH_HALF_MM);
    }

    return in_path;
}

/*=====================================================================*/
/* Profile construction                                                */
/*=====================================================================*/

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
 * Picking the nearest return anywhere is wrong: a wall 170 mm off to
 * the side is closer than a box 177 mm dead ahead, but only the box is
 * in the way (bench sweep E1, test G9).
 */
static uint8_t obs_find_nearest(const s5_sample_t *p_samples,
                                uint8_t count,
                                uint16_t *p_nearest_mm)
{
    uint8_t  best_idx = count;
    uint16_t best_mm  = 0U;
    uint8_t  i;

    for (i = 0U; i < count; i++)
    {
        const uint16_t r = s5_sample_range(p_samples[i]);

        if (obstacle_in_path(s5_sample_bearing(p_samples[i]), r) &&
            ((best_idx == count) || (r < best_mm)))
        {
            best_mm  = r;
            best_idx = i;
        }
    }

    *p_nearest_mm = best_mm;

    return best_idx;
}

/**
 * @brief   Does this return continue the surface being traced?
 *
 * Step gate: a sudden range jump is the edge of a body. Depth gate:
 * stops a long shallow slope growing the object forever. A silent
 * bearing fails both, so the object never grows across missing data.
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

/** Base confidence minus the share of bearings that were silent. */
static uint8_t obs_confidence(int32_t base, uint8_t unknowns, uint8_t count)
{
    int32_t c = base - (((int32_t)unknowns * 100) / (int32_t)count);

    if (c < 0)   { c = 0; }
    if (c > 100) { c = 100; }

    return (uint8_t)c;
}

void obstacle_profile_build(const s5_sample_t *p_samples,
                            uint8_t count,
                            obstacle_profile_t *p_out)
{
    /* Reset first so a rejected call still leaves the safe default. */
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
            const int16_t  b = s5_sample_bearing(p_samples[i]);
            const uint16_t r = s5_sample_range(p_samples[i]);

            if (r == OBSTACLE_RANGE_UNKNOWN)
            {
                unknowns++;

                if ((b <= OBS_PATH_CONE_DEG) && (b >= -OBS_PATH_CONE_DEG))
                {
                    blind_ahead = true;
                }
            }
        }

        centre = obs_find_nearest(p_samples, count, &nearest);

        if ((unknowns == count) || ((centre >= count) && blind_ahead))
        {
            /* All silent, or nothing found but a blind spot straight
             * ahead: cannot call it clear. Profile stays invalid and the
             * planner says STOP. Before this check a dead sensor gave
             * "CONTINUE, 100 %". */
        }
        else if (centre >= count)
        {
            /* Nothing in the driving path. Only as trustworthy as the
             * share of bearings that answered. */
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

            /* Grow outwards from the closest in-path point until the
             * surface breaks. */
            grow = true;
            while (grow && ((hi_idx + 1U) < count))
            {
                const uint16_t r = s5_sample_range(p_samples[hi_idx + 1U]);

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
                const uint16_t r = s5_sample_range(p_samples[lo_idx - 1U]);

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

            /* Extent, and the true closest point (growth may have
             * reached a return closer than the seed). */
            for (i = lo_idx; i <= hi_idx; i++)
            {
                const int16_t  b = s5_sample_bearing(p_samples[i]);
                const uint16_t r = s5_sample_range(p_samples[i]);
                const int32_t  x = obs_lateral_mm(b, r);

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

                bearing_sum += (int32_t)b;
            }

            /* Everything outside the object limits the bypass. A silent
             * bearing on a side means that corridor is unseen, so we
             * refuse to plan through it. */
            for (i = 0U; i < count; i++)
            {
                if ((i < lo_idx) || (i > hi_idx))
                {
                    const uint16_t r = s5_sample_range(p_samples[i]);

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
                            obs_lateral_mm(s5_sample_bearing(p_samples[i]),
                                           r);

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
            p_out->bearing_deg    = (int16_t)(bearing_sum / (int32_t)hits);
            p_out->width_mm       = (uint16_t)width;
            p_out->left_edge_deg  = s5_sample_bearing(p_samples[hi_idx]);
            p_out->right_edge_deg = s5_sample_bearing(p_samples[lo_idx]);
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
    return obstacle_plan_within(p_profile, (uint16_t)OBSTACLE_TRIGGER_MM);
}

obstacle_action_t obstacle_plan_within(const obstacle_profile_t *p_profile,
                                       uint16_t trigger_mm)
{
    obstacle_action_t action = OBSTACLE_ACT_STOP;

    if ((p_profile != NULL) && p_profile->valid)
    {
        if (p_profile->nearest_mm == OBSTACLE_RANGE_CLEAR)
        {
            /* "Nothing seen" only counts if enough of the sweep
             * answered. */
            if (p_profile->confidence_pct >= (uint8_t)OBS_CLEAR_MIN_CONF)
            {
                action = OBSTACLE_ACT_CONTINUE;
            }
            else
            {
                action = OBSTACLE_ACT_SLOW;
            }
        }
        else if (p_profile->nearest_mm > trigger_mm)
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

uint16_t obstacle_pass_distance_mm(const obstacle_profile_t *p_profile)
{
    uint32_t pass = 0U;

    if ((p_profile != NULL) && p_profile->valid &&
        obs_is_valid(p_profile->nearest_mm))
    {
        pass = (uint32_t)p_profile->nearest_mm +
               (uint32_t)OBSTACLE_ASSUMED_DEPTH_MM +
               (uint32_t)OBSTACLE_ROBOT_LENGTH_MM +
               (uint32_t)OBSTACLE_PASS_MARGIN_MM;

        if (pass > 0xFFFFU)
        {
            pass = 0xFFFFU;
        }
    }

    return (uint16_t)pass;
}

/*=====================================================================*/
/* v3.0: calibrated refinement, reverse distance, side watch           */
/*=====================================================================*/

int32_t obstacle_sin_q15_ddeg(int32_t ddeg)
{
    int32_t sign = 1;
    int32_t a    = ddeg;
    int32_t idx;
    int32_t frac;
    int32_t lo;
    int32_t hi;

    /* Wrap into -1800..1800 first (dead reckoning hands in any heading). */
    while (a > 1800)
    {
        a -= 3600;
    }
    while (a < -1800)
    {
        a += 3600;
    }

    if (a < 0)
    {
        a    = -a;
        sign = -1;
    }

    if (a > 900)
    {
        a = 1800 - a;
    }

    idx  = a / (SIN_STEP_DEG * 10);
    frac = a % (SIN_STEP_DEG * 10);
    lo   = (int32_t)g_sin_q15[idx];
    hi   = (frac == 0) ? lo : (int32_t)g_sin_q15[idx + 1];

    return sign * (lo + (((hi - lo) * frac) / (SIN_STEP_DEG * 10)));
}

/** tan(ddeg) * r, via the sine table. |ddeg| < 85 deg. */
static int32_t obs_tan_mm(uint16_t r_mm, int32_t ddeg)
{
    const int32_t c = obstacle_sin_q15_ddeg(900 - ddeg);

    return (c > 0) ? (((int32_t)r_mm * obstacle_sin_q15_ddeg(ddeg)) / c) : 0;
}

/** atan(x / y) in whole degrees, by bisection on the table. */
static int16_t obs_atan_deg(int32_t x_mm, int32_t y_mm)
{
    int32_t lo = 0;
    int32_t hi = 89;
    int32_t sign = 1;

    if (y_mm <= 0)
    {
        return 0;
    }

    if (x_mm < 0)
    {
        x_mm = -x_mm;
        sign = -1;
    }

    while ((hi - lo) > 1)
    {
        const int32_t mid = (lo + hi) / 2;

        if (obs_tan_mm((uint16_t)((y_mm > 0xFFFF) ? 0xFFFF : y_mm), mid * 10) <= x_mm)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    if ((obs_tan_mm((uint16_t)y_mm, hi * 10) - x_mm) <
        (x_mm - obs_tan_mm((uint16_t)y_mm, lo * 10)))
    {
        lo = hi;
    }

    return (int16_t)(sign * lo);
}

/**
 * Edge angle in 0.1 deg from a hit/miss interval. Left edge (towards
 * +): the beam hears it h early, so subtract h; right edge: add h.
 */
static int32_t obs_edge_ddeg(const s5_edge_probe_t *p_probe,
                             int16_t sample_deg, int32_t step_ddeg,
                             int32_t beam_ddeg, int32_t towards_left)
{
    int32_t hit  = (int32_t)sample_deg * 10;
    int32_t miss = hit + (towards_left * step_ddeg);

    if ((p_probe != NULL) && p_probe->valid)
    {
        hit  = (int32_t)p_probe->hit_ddeg;
        miss = (int32_t)p_probe->miss_ddeg;
    }

    return ((hit + miss) / 2) - (towards_left * beam_ddeg);
}

void obstacle_profile_refine(obstacle_profile_t *p_profile,
                             const s5_geom_cal_t *p_cal,
                             const s5_edge_probes_t *p_probes)
{
    if ((p_profile != NULL) && (p_cal != NULL) && p_profile->valid &&
        obs_is_valid(p_profile->nearest_mm))
    {
        obstacle_profile_t *p = p_profile;
        const bool left_seen  = (p->left_edge_deg < p_cal->sweep_limit_deg);
        const bool right_seen = (p->right_edge_deg > -p_cal->sweep_limit_deg);
        const int32_t step_ddeg = (int32_t)p_cal->step_deg * 10;
        const int32_t beam_ddeg = (int32_t)p_cal->beam_half_ddeg;
        /* swing = edge + path half; the nearest point is inside the path,
         * so the swings were never clamped and this is exact. */
        const int32_t raw_max = (int32_t)p->swing_left_mm - OBS_PATH_HALF_MM;
        const int32_t raw_min = OBS_PATH_HALF_MM - (int32_t)p->swing_right_mm;
        int32_t    max_x = raw_max;
        int32_t    min_x = raw_min;
        int32_t    width;
        int32_t    centre;
        int32_t    gap_l = (int32_t)p->gap_left_mm;
        int32_t    gap_r = (int32_t)p->gap_right_mm;

        /* The course's obstacles are boxes and posts with a face square
         * to the car, so an edge at angle e sits nearest * tan(e) to the
         * side. A corrected edge never moves OUT past what was measured
         * (that would only come from a wrong calibration). */
        if (left_seen)
        {
            const int32_t e = obs_edge_ddeg(
                (p_probes != NULL) ? &p_probes->left : NULL,
                p->left_edge_deg, step_ddeg, beam_ddeg, 1);
            const int32_t x = obs_tan_mm(p->nearest_mm, e);

            max_x = (x < raw_max) ? x : raw_max;
        }

        if (right_seen)
        {
            const int32_t e = obs_edge_ddeg(
                (p_probes != NULL) ? &p_probes->right : NULL,
                p->right_edge_deg, step_ddeg, beam_ddeg, -1);
            const int32_t x = obs_tan_mm(p->nearest_mm, e);

            min_x = (x > raw_min) ? x : raw_min;
        }

        if (max_x < min_x)
        {
            /* Thinner than the correction: a post. Collapse to a line. */
            const int32_t mid = (max_x + min_x) / 2;

            max_x = mid;
            min_x = mid;
        }

        width = max_x - min_x;

        if (width < OBS_MIN_WIDTH_MM)
        {
            const int32_t grow = (OBS_MIN_WIDTH_MM - width + 1) / 2;

            max_x += grow;
            min_x -= grow;
            width  = max_x - min_x;
        }

        centre = (max_x + min_x) / 2;

        p->width_mm       = (uint16_t)width;
        p->bearing_deg    = obs_atan_deg(centre, (int32_t)p->nearest_mm);
        if (left_seen)
        {
            p->left_edge_deg = obs_atan_deg(max_x, (int32_t)p->nearest_mm);
        }
        if (right_seen)
        {
            p->right_edge_deg = obs_atan_deg(min_x, (int32_t)p->nearest_mm);
        }
        p->swing_left_mm  = (uint16_t)((max_x + OBS_PATH_HALF_MM > 0) ?
                                       (max_x + OBS_PATH_HALF_MM) : 0);
        p->swing_right_mm = (uint16_t)((OBS_PATH_HALF_MM - min_x > 0) ?
                                       (OBS_PATH_HALF_MM - min_x) : 0);

        /* An unseen edge means an unseen corridor: never plan through
         * it. A seen one widens by what the object shrank. */
        if (!left_seen)
        {
            gap_l = 0;
        }
        else if ((gap_l > 0) && (gap_l < (int32_t)OBSTACLE_GAP_OPEN_MM))
        {
            gap_l += raw_max - max_x;
        }
        else
        {
            /* 0 = blocked or unseen; OPEN stays open. */
        }

        if (!right_seen)
        {
            gap_r = 0;
        }
        else if ((gap_r > 0) && (gap_r < (int32_t)OBSTACLE_GAP_OPEN_MM))
        {
            gap_r += min_x - raw_min;
        }
        else
        {
            /* As above. */
        }

        if (gap_l > (int32_t)OBSTACLE_GAP_OPEN_MM)
        {
            gap_l = (int32_t)OBSTACLE_GAP_OPEN_MM;
        }
        if (gap_r > (int32_t)OBSTACLE_GAP_OPEN_MM)
        {
            gap_r = (int32_t)OBSTACLE_GAP_OPEN_MM;
        }

        p->gap_left_mm  = (uint16_t)gap_l;
        p->gap_right_mm = (uint16_t)gap_r;
        p->action       = obstacle_plan_within(
            p, (p_cal->trigger_mm != 0U) ? p_cal->trigger_mm
                                         : (uint16_t)OBSTACLE_TRIGGER_MM);
    }
}

uint16_t obstacle_reverse_mm(const obstacle_profile_t *p_profile)
{
    uint32_t rev = 0U;

    if ((p_profile != NULL) && (p_profile->action == OBSTACLE_ACT_REVERSE))
    {
        uint32_t target = (uint32_t)S5_REMEASURE_FAR_MM;
        uint32_t near_mm = 0U;

        if (obs_is_valid(p_profile->nearest_mm))
        {
            near_mm = (uint32_t)p_profile->nearest_mm;

            if (near_mm < (uint32_t)OBSTACLE_HARD_STOP_MM)
            {
                target = (uint32_t)S5_REMEASURE_NEAR_MM;
            }
        }

        rev = (target > near_mm) ? (target - near_mm) : 0U;

        if (rev < (uint32_t)S5_REVERSE_MIN_MM)
        {
            rev = (uint32_t)S5_REVERSE_MIN_MM;
        }
        if (rev > (uint32_t)S5_REVERSE_MAX_MM)
        {
            rev = (uint32_t)S5_REVERSE_MAX_MM;
        }
    }

    return (uint16_t)rev;
}

uint16_t obstacle_watch_lateral_mm(uint16_t range_mm, int16_t watch_deg,
                                   int16_t beam_half_ddeg)
{
    uint32_t lat = 0xFFFFU;

    if (obs_is_valid(range_mm))
    {
        int32_t a = ((int32_t)watch_deg * 10) + (int32_t)beam_half_ddeg;

        if (a > 900)
        {
            a = 900;
        }

        lat = ((uint32_t)range_mm * (uint32_t)obstacle_sin_q15_ddeg(a)) /
              (uint32_t)SIN_ONE_Q15;
    }

    return (uint16_t)lat;
}

uint16_t obstacle_watch_remaining_mm(uint16_t lateral_mm, int16_t watch_deg,
                                     int16_t beam_half_ddeg)
{
    int32_t a = ((int32_t)watch_deg * 10) + (int32_t)beam_half_ddeg;
    int32_t ahead = 0;
    int32_t total;

    if (a < 50)
    {
        a = 50;     /* 5 deg: a near-forward look would see "forever" */
    }
    if (a > 900)
    {
        a = 900;    /* past sideways: the ray clears level with us */
    }

    if (a < 900)
    {
        /* cot(a) = sin(90 - a) / sin(a) */
        ahead = (int32_t)(((int64_t)lateral_mm *
                           obstacle_sin_q15_ddeg(900 - a)) /
                          obstacle_sin_q15_ddeg(a));
    }

    total = ahead + (int32_t)S5_REAR_FROM_SENSOR_MM +
            (int32_t)S5_PASS_MARGIN_MM;

    if (total > 0xFFFF)
    {
        total = 0xFFFF;
    }

    return (uint16_t)total;
}
