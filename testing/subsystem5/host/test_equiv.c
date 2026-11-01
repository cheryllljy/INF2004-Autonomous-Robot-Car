/**
 * @file    test_equiv.c
 * @brief   HOST TEST. CR1 "no loss of functionality" for the geometry:
 *          v1 (unpacked samples, 4 B each) and v2 (packed, 2 B each)
 *          must give the same profile and action for every sweep the
 *          firmware can produce.
 *
 * 20 000 random sweeps on the 10 deg scan grid, with ranges mixed to
 * hit every branch: in-path objects, side walls, background, CLEAR and
 * UNKNOWN. Deterministic seed, so a failure is reproducible.
 */

#include "obstacle_model.h"
#include "v1_adapter.h"

#include <stdio.h>
#include <stdlib.h>

#define N_SWEEPS    (20000)

static uint32_t g_seed = 12345U;

static uint32_t rnd(uint32_t n)
{
    g_seed = (g_seed * 1103515245U) + 12345U;
    return (g_seed >> 8) % n;
}

static uint16_t random_range(void)
{
    const uint32_t k = rnd(100U);
    uint16_t       r;

    if (k < 25U)      { r = OBSTACLE_RANGE_CLEAR; }
    else if (k < 35U) { r = OBSTACLE_RANGE_UNKNOWN; }
    else if (k < 75U) { r = (uint16_t)(60U + rnd(400U)); }  /* objects */
    else              { r = (uint16_t)(30U + rnd(2471U)); } /* anything */

    return r;
}

int main(void)
{
    int sweeps;
    int mismatches = 0;
    int by_action[OBSTACLE_ACT_COUNT] = { 0 };
    int invalid = 0;

    for (sweeps = 0; sweeps < N_SWEEPS; sweeps++)
    {
        int16_t            b[OBSTACLE_MAX_SAMPLES];
        uint16_t           r[OBSTACLE_MAX_SAMPLES];
        s5_sample_t        s[OBSTACLE_MAX_SAMPLES];
        uint8_t            n = 0U;
        int16_t            g;
        obstacle_profile_t v2;
        v1_out_t           v1;

        /* Random subset of the 15 grid bearings, ascending. */
        for (g = -70; g <= 70; g = (int16_t)(g + 10))
        {
            if ((rnd(100U) < 60U) && (n < (uint8_t)OBSTACLE_MAX_SAMPLES))
            {
                b[n] = g;
                r[n] = random_range();
                s[n] = s5_sample_pack(b[n], r[n]);
                n++;
            }
        }

        if (n == 0U)
        {
            continue;
        }

        v1_build(b, r, n, &v1);
        obstacle_profile_build(s, n, &v2);

        if ((v1.valid != v2.valid) ||
            (v1.nearest_mm != v2.nearest_mm) ||
            (v1.bearing_deg != v2.bearing_deg) ||
            (v1.width_mm != v2.width_mm) ||
            (v1.left_edge_deg != v2.left_edge_deg) ||
            (v1.right_edge_deg != v2.right_edge_deg) ||
            (v1.gap_left_mm != v2.gap_left_mm) ||
            (v1.gap_right_mm != v2.gap_right_mm) ||
            (v1.swing_left_mm != v2.swing_left_mm) ||
            (v1.swing_right_mm != v2.swing_right_mm) ||
            (v1.confidence_pct != v2.confidence_pct) ||
            (v1.sample_count != v2.sample_count) ||
            (v1.action != (int)v2.action))
        {
            mismatches++;

            if (mismatches <= 5)
            {
                printf("  MISMATCH sweep %d: v1 action %d nearest %u,"
                       " v2 action %d nearest %u\n", sweeps, v1.action,
                       (unsigned)v1.nearest_mm, (int)v2.action,
                       (unsigned)v2.nearest_mm);
            }
        }

        if (!v2.valid) { invalid++; }
        by_action[v2.action]++;
    }

    printf("%d sweeps compared, %d mismatches\n", N_SWEEPS, mismatches);
    printf("coverage: invalid %d, CONTINUE %d, SLOW %d, STOP %d,"
           " LEFT %d, RIGHT %d, REVERSE %d\n", invalid,
           by_action[OBSTACLE_ACT_CONTINUE], by_action[OBSTACLE_ACT_SLOW],
           by_action[OBSTACLE_ACT_STOP], by_action[OBSTACLE_ACT_BYPASS_LEFT],
           by_action[OBSTACLE_ACT_BYPASS_RIGHT],
           by_action[OBSTACLE_ACT_REVERSE]);

    return (mismatches == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
