/**
 * @file    test_geometry.c
 * @brief   Host-side tests for obstacle_profile_build / obstacle_plan.
 *
 * Runs on a PC, no hardware and no RTOS:
 *
 *   cc -DOBSTACLE_HOST_TEST -Iinclude -Wall -Wextra -Werror \
 *      src/obstacle_geometry.c test/test_geometry.c -o build/tests
 *   ./build/tests
 *
 * These are the repeatable subsystem tests the brief asks for. Every
 * time a model constant is retuned, run this before flashing.
 */

#include "obstacle.h"

#include <stdio.h>
#include <stdlib.h>

static int g_failures = 0;
static int g_checks   = 0;

static void check(bool condition, const char *what)
{
    g_checks++;

    if (!condition)
    {
        g_failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_near(int32_t got, int32_t want, int32_t tol,
                       const char *what)
{
    const int32_t diff = (got > want) ? (got - want) : (want - got);

    g_checks++;

    if (diff > tol)
    {
        g_failures++;
        printf("  FAIL: %s (got %d, want %d +/- %d)\n",
               what, (int)got, (int)want, (int)tol);
    }
}

/*---------------------------------------------------------------------
 * G1: empty track. Every bearing reports no echo.
 *-------------------------------------------------------------------*/
static void test_clear_track(void)
{
    static const obstacle_sample_t samples[5] =
    {
        { -60, OBSTACLE_RANGE_CLEAR },
        { -30, OBSTACLE_RANGE_CLEAR },
        {   0, OBSTACLE_RANGE_CLEAR },
        {  30, OBSTACLE_RANGE_CLEAR },
        {  60, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G1 clear track\n");
    obstacle_profile_build(samples, 5U, &profile);

    check(profile.valid, "profile is valid");
    check(profile.nearest_mm == OBSTACLE_RANGE_CLEAR, "nearest CLEAR");
    check(profile.action == OBSTACLE_ACT_CONTINUE, "action CONTINUE");
}

/*---------------------------------------------------------------------
 * G2: a 60 mm post dead ahead at 250 mm, nothing either side.
 *-------------------------------------------------------------------*/
static void test_narrow_post(void)
{
    static const obstacle_sample_t samples[7] =
    {
        { -60, OBSTACLE_RANGE_CLEAR },
        { -30, OBSTACLE_RANGE_CLEAR },
        { -10, OBSTACLE_RANGE_CLEAR },
        {   0, 250U                 },
        {  10, OBSTACLE_RANGE_CLEAR },
        {  30, OBSTACLE_RANGE_CLEAR },
        {  60, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G2 narrow post at 250 mm\n");
    obstacle_profile_build(samples, 7U, &profile);

    check(profile.valid, "profile is valid");
    check_near(profile.nearest_mm, 250, 0, "nearest range");
    check_near(profile.bearing_deg, 0, 2, "bearing");
    check(profile.width_mm <= 80U, "narrow object stays narrow");
    check(profile.action == OBSTACLE_ACT_BYPASS_LEFT,
          "free on both sides, prefers left");
}

/*---------------------------------------------------------------------
 * G3: a 200 mm wide box at 300 mm. Checks the beam-width correction.
 *
 * Raw lateral extent is 2 * 300 * sin(20) = 205 mm; the beam inflates
 * it by about 0.264 * 300 = 79 mm, so the corrected width should land
 * near 126 mm. Re-measure a real box and retune OBS_BEAM_CORR_Q10
 * until this test matches reality.
 *-------------------------------------------------------------------*/
static void test_wide_box(void)
{
    static const obstacle_sample_t samples[7] =
    {
        { -60, OBSTACLE_RANGE_CLEAR },
        { -20, 305U                 },
        { -10, 300U                 },
        {   0, 298U                 },
        {  10, 301U                 },
        {  20, 306U                 },
        {  60, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G3 wide box at 300 mm\n");
    obstacle_profile_build(samples, 7U, &profile);

    check(profile.valid, "profile is valid");
    check_near(profile.nearest_mm, 298, 0, "nearest range");
    check_near(profile.left_edge_deg, 20, 0, "left edge");
    check_near(profile.right_edge_deg, -20, 0, "right edge");
    check_near(profile.width_mm, 126, 25, "beam-corrected width");
}

/*---------------------------------------------------------------------
 * G4: obstacle on the line with a wall close on the left. The wall is
 *     a separate body (big range discontinuity), so region growing
 *     must not swallow it into the obstacle, and the only corridor
 *     wide enough for the chassis is on the right.
 *-------------------------------------------------------------------*/
static void test_wall_on_left(void)
{
    static const obstacle_sample_t samples[7] =
    {
        { -60, OBSTACLE_RANGE_CLEAR },
        { -30, OBSTACLE_RANGE_CLEAR },
        { -10, 260U                 },
        {   0, 250U                 },
        {  10, 265U                 },
        {  30, 480U                 },  /* wall, x = +240 mm        */
        {  60, 420U                 }   /* wall, x = +364 mm        */
    };
    obstacle_profile_t profile;

    printf("G4 separate wall close on the left\n");
    obstacle_profile_build(samples, 7U, &profile);

    check(profile.valid, "profile is valid");
    check_near(profile.left_edge_deg, 10, 0, "wall excluded from object");
    check_near(profile.right_edge_deg, -10, 0, "right edge found");
    check(profile.gap_left_mm < profile.gap_right_mm,
          "left corridor is the tighter one");
    check(profile.action == OBSTACLE_ACT_BYPASS_RIGHT,
          "left corridor too narrow, goes right");
}

/*---------------------------------------------------------------------
 * G5: already too close to steer round it.
 *-------------------------------------------------------------------*/
static void test_too_close(void)
{
    static const obstacle_sample_t samples[3] =
    {
        { -20, OBSTACLE_RANGE_CLEAR },
        {   0, 90U                  },
        {  20, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G5 object inside the hard stop distance\n");
    obstacle_profile_build(samples, 3U, &profile);

    check(profile.action == OBSTACLE_ACT_REVERSE, "backs off");
}

/*---------------------------------------------------------------------
 * G6: defensive. Bad input must leave a safe profile, not garbage.
 *-------------------------------------------------------------------*/
static void test_bad_input(void)
{
    obstacle_profile_t profile;
    obstacle_sample_t  one = { 0, 400U };

    printf("G6 defensive checks\n");

    profile.valid  = true;
    profile.action = OBSTACLE_ACT_CONTINUE;
    obstacle_profile_build(NULL, 5U, &profile);
    check(!profile.valid, "NULL samples leaves profile invalid");
    check(profile.action == OBSTACLE_ACT_STOP, "defaults to STOP");

    profile.valid  = true;
    profile.action = OBSTACLE_ACT_CONTINUE;
    obstacle_profile_build(&one, 0U, &profile);
    check(!profile.valid, "zero count leaves profile invalid");

    obstacle_profile_build(NULL, 0U, NULL);   /* must not crash */
    check(obstacle_plan(NULL) == OBSTACLE_ACT_STOP, "NULL plan STOPs");
}

/*---------------------------------------------------------------------
 * G7: both sides open, but the object sits left of the centreline.
 *     Passing on the right deviates less, so the planner must pick it.
 *-------------------------------------------------------------------*/
static void test_least_deviation(void)
{
    static const obstacle_sample_t samples[7] =
    {
        { -60, OBSTACLE_RANGE_CLEAR },
        { -30, OBSTACLE_RANGE_CLEAR },
        {   0, OBSTACLE_RANGE_CLEAR },
        {  20, 320U                 },
        {  30, 300U                 },
        {  40, 318U                 },
        {  60, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G7 object offset to the left, both sides open\n");
    obstacle_profile_build(samples, 7U, &profile);

    check(profile.valid, "profile is valid");
    check(profile.bearing_deg > 20, "bearing is to the left");
    check(profile.swing_right_mm < profile.swing_left_mm,
          "passing right deviates less");
    check(profile.action == OBSTACLE_ACT_BYPASS_RIGHT,
          "takes the shorter detour");
}

/*---------------------------------------------------------------------
 * G8: the sensor never answered in any direction. This is the bench
 *     failure: it must not come out as a clear track.
 *-------------------------------------------------------------------*/
static void test_all_silent(void)
{
    static const obstacle_sample_t samples[5] =
    {
        { -60, OBSTACLE_RANGE_UNKNOWN },
        { -30, OBSTACLE_RANGE_UNKNOWN },
        {   0, OBSTACLE_RANGE_UNKNOWN },
        {  30, OBSTACLE_RANGE_UNKNOWN },
        {  60, OBSTACLE_RANGE_UNKNOWN }
    };
    obstacle_profile_t profile;

    printf("G8 sensor silent in every direction\n");
    obstacle_profile_build(samples, 5U, &profile);

    check(!profile.valid, "silence is not a valid profile");
    check(profile.action == OBSTACLE_ACT_STOP, "action STOP");
}

/*---------------------------------------------------------------------
 * G9: some directions clear, some silent, nothing close. Still not
 *     enough to claim the way is open.
 *-------------------------------------------------------------------*/
static void test_partly_silent(void)
{
    static const obstacle_sample_t samples[5] =
    {
        { -60, OBSTACLE_RANGE_CLEAR   },
        { -30, OBSTACLE_RANGE_CLEAR   },
        {   0, OBSTACLE_RANGE_UNKNOWN },
        {  30, OBSTACLE_RANGE_CLEAR   },
        {  60, OBSTACLE_RANGE_CLEAR   }
    };
    obstacle_profile_t profile;

    printf("G9 straight ahead silent, sides clear\n");
    obstacle_profile_build(samples, 5U, &profile);

    check(!profile.valid, "cannot claim clear with a blind spot ahead");
    check(profile.action == OBSTACLE_ACT_STOP, "action STOP");
}

/*---------------------------------------------------------------------
 * G10: object found, one direction silent. Profile stands, but with
 *      lower confidence than the same scene fully answered (G2 = 80).
 *-------------------------------------------------------------------*/
static void test_object_with_gap(void)
{
    static const obstacle_sample_t samples[7] =
    {
        { -60, OBSTACLE_RANGE_CLEAR   },
        { -30, OBSTACLE_RANGE_CLEAR   },
        { -10, OBSTACLE_RANGE_CLEAR   },
        {   0, 250U                   },
        {  10, OBSTACLE_RANGE_CLEAR   },
        {  30, OBSTACLE_RANGE_CLEAR   },
        {  60, OBSTACLE_RANGE_UNKNOWN }
    };
    obstacle_profile_t profile;

    printf("G10 object ahead, one direction silent\n");
    obstacle_profile_build(samples, 7U, &profile);

    check(profile.valid, "object still profiled");
    check(profile.confidence_pct <= 70U, "confidence drops for the gap");
}

/*---------------------------------------------------------------------
 * G11: REAL BENCH DATA. Every bearing silent (NO_RESP). Used to come
 *     out as "valid, CONTINUE, 100 %". A deaf sensor must STOP.
 *-------------------------------------------------------------------*/
static void test_bench_all_silent(void)
{
    static const obstacle_sample_t samples[11] =
    {
        { -75, OBSTACLE_RANGE_UNKNOWN }, { -60, OBSTACLE_RANGE_UNKNOWN }, { -45, OBSTACLE_RANGE_UNKNOWN }, { -30, OBSTACLE_RANGE_UNKNOWN },
        { -15, OBSTACLE_RANGE_UNKNOWN }, {   0, OBSTACLE_RANGE_UNKNOWN }, {  15, OBSTACLE_RANGE_UNKNOWN }, {  30, OBSTACLE_RANGE_UNKNOWN },
        {  45, OBSTACLE_RANGE_UNKNOWN }, {  60, OBSTACLE_RANGE_UNKNOWN }, {  75, OBSTACLE_RANGE_UNKNOWN }
    };
    obstacle_profile_t profile;

    printf("G11 bench: every bearing silent\n");
    obstacle_profile_build(samples, 11U, &profile);

    check(!profile.valid, "silent sweep is not a valid profile");
    check(profile.action == OBSTACLE_ACT_STOP, "silent sweep STOPs");
}

/*---------------------------------------------------------------------
 * G12: REAL BENCH DATA. Something 170 mm away at -75 deg (off to the
 *     right, not in the path) and something 177 mm dead ahead. The old
 *     code picked the side object as "the obstacle" and chose to swing
 *     299 mm right around it. The thing in the way is the one ahead,
 *     the right side is blocked, so the answer is left.
 *-------------------------------------------------------------------*/
static void test_bench_side_object(void)
{
    static const obstacle_sample_t samples[11] =
    {
        { -75, 170U }, { -60, 170U }, { -45, 183U }, { -30, OBSTACLE_RANGE_CLEAR },
        { -15, OBSTACLE_RANGE_CLEAR },  {   0, 177U }, {  15, 177U }, {  30, OBSTACLE_RANGE_CLEAR },
        {  45, OBSTACLE_RANGE_CLEAR },  {  60, OBSTACLE_RANGE_CLEAR },  {  75, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G12 bench: closer object off to the side\n");
    obstacle_profile_build(samples, 11U, &profile);

    check(profile.valid, "profile is valid");
    check_near(profile.nearest_mm, 177, 0, "object ahead chosen, not side");
    check(profile.bearing_deg >= 0, "bearing is ahead, not at -60");
    check(profile.gap_right_mm < profile.gap_left_mm,
          "side object blocks the right corridor");
    check(profile.action == OBSTACLE_ACT_BYPASS_LEFT, "goes left");
}

/*---------------------------------------------------------------------
 * G13: object ahead at 250 mm with one half of the sweep silent. We
 *      cannot see the silent side, so we must not plan through it.
 *-------------------------------------------------------------------*/
static void test_half_silent(void)
{
    static const obstacle_sample_t left_dead[7] =
    {
        { -60, OBSTACLE_RANGE_CLEAR }, { -30, OBSTACLE_RANGE_CLEAR }, { -10, 255U }, { 0, 250U },
        {  10, 255U }, {  30, OBSTACLE_RANGE_UNKNOWN }, { 60, OBSTACLE_RANGE_UNKNOWN }
    };
    static const obstacle_sample_t right_dead[7] =
    {
        { -60, OBSTACLE_RANGE_UNKNOWN }, { -30, OBSTACLE_RANGE_UNKNOWN }, { -10, 255U }, { 0, 250U },
        {  10, 255U }, {  30, OBSTACLE_RANGE_CLEAR }, { 60, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G13 object ahead, one side silent\n");

    obstacle_profile_build(left_dead, 7U, &profile);
    check(profile.gap_left_mm == 0U, "silent left side has no corridor");
    check(profile.action != OBSTACLE_ACT_BYPASS_LEFT,
          "never plans through the silent left side");

    obstacle_profile_build(right_dead, 7U, &profile);
    check(profile.gap_right_mm == 0U, "silent right side has no corridor");
    check(profile.action != OBSTACLE_ACT_BYPASS_RIGHT,
          "never plans through the silent right side");
}

/*---------------------------------------------------------------------
 * G14: nothing in the path, but some bearings silent. One flaky ping
 *      in eleven is tolerated; three are not.
 *-------------------------------------------------------------------*/
static void test_partly_silent_clear(void)
{
    static const obstacle_sample_t one_dead[11] =
    {
        { -75, OBSTACLE_RANGE_CLEAR }, { -60, OBSTACLE_RANGE_CLEAR }, { -45, OBSTACLE_RANGE_CLEAR }, { -30, OBSTACLE_RANGE_CLEAR },
        { -15, OBSTACLE_RANGE_CLEAR }, {   0, OBSTACLE_RANGE_CLEAR }, {  15, OBSTACLE_RANGE_CLEAR }, {  30, OBSTACLE_RANGE_CLEAR },
        {  45, OBSTACLE_RANGE_CLEAR }, {  60, OBSTACLE_RANGE_CLEAR }, {  75, OBSTACLE_RANGE_UNKNOWN }
    };
    static const obstacle_sample_t three_dead[11] =
    {
        { -75, OBSTACLE_RANGE_UNKNOWN }, { -60, OBSTACLE_RANGE_UNKNOWN }, { -45, OBSTACLE_RANGE_UNKNOWN }, { -30, OBSTACLE_RANGE_CLEAR },
        { -15, OBSTACLE_RANGE_CLEAR }, {   0, OBSTACLE_RANGE_CLEAR }, {  15, OBSTACLE_RANGE_CLEAR }, {  30, OBSTACLE_RANGE_CLEAR },
        {  45, OBSTACLE_RANGE_CLEAR }, {  60, OBSTACLE_RANGE_CLEAR }, {  75, OBSTACLE_RANGE_CLEAR }
    };
    obstacle_profile_t profile;

    printf("G14 clear path, some bearings silent\n");

    obstacle_profile_build(one_dead, 11U, &profile);
    check(profile.action == OBSTACLE_ACT_CONTINUE,
          "1 of 11 silent still continues");

    obstacle_profile_build(three_dead, 11U, &profile);
    check(profile.action == OBSTACLE_ACT_SLOW,
          "3 of 11 silent slows down instead of trusting it");
}

int main(void)
{
    test_clear_track();
    test_narrow_post();
    test_wide_box();
    test_wall_on_left();
    test_too_close();
    test_bad_input();
    test_least_deviation();
    test_bench_all_silent();
    test_bench_side_object();
    test_half_silent();
    test_partly_silent_clear();
    test_all_silent();
    test_partly_silent();
    test_object_with_gap();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
