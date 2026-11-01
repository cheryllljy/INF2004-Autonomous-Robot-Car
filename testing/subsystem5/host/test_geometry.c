/**
 * @file    test_geometry.c
 * @brief   HOST TEST. obstacle_profile_build / obstacle_plan from
 *          packed samples. Same scenes as v1 tests G1..G14.
 *
 * Each scene is written as (bearing, range) pairs for readability and
 * packed with s5_sample_pack() before the call, exactly as the task
 * stores them. G11, G12 and G14 came from a v0 bench sweep on a 15 deg
 * grid; the firmware scan only uses the 10 deg grid, so those three are
 * re-gridded (noted at each one) and the assertions re-checked.
 */

#include "obstacle_model.h"

#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    int16_t  b;
    uint16_t r;
} vec_t;

#define CLR     OBSTACLE_RANGE_CLEAR
#define UNK     OBSTACLE_RANGE_UNKNOWN

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

/** Pack a scene and build. Off-grid bearings are a test-writing bug. */
static void build(const vec_t *v, uint8_t n, obstacle_profile_t *p)
{
    s5_sample_t s[OBSTACLE_MAX_SAMPLES];
    uint8_t     i;

    for (i = 0U; i < n; i++)
    {
        check(s5_bearing_on_grid(v[i].b), "scene bearing on 10 deg grid");
        s[i] = s5_sample_pack(v[i].b, v[i].r);
    }

    obstacle_profile_build(s, n, p);
}

static void g1_clear_track(void)
{
    static const vec_t v[5] = { {-60,CLR}, {-30,CLR}, {0,CLR}, {30,CLR}, {60,CLR} };
    obstacle_profile_t p;

    printf("G1 clear track\n");
    build(v, 5U, &p);
    check(p.valid, "profile is valid");
    check(p.nearest_mm == OBSTACLE_RANGE_CLEAR, "nearest CLEAR");
    check(p.action == OBSTACLE_ACT_CONTINUE, "action CONTINUE");
}

static void g2_narrow_post(void)
{
    static const vec_t v[7] = { {-60,CLR}, {-30,CLR}, {-10,CLR}, {0,250U},
                                {10,CLR}, {30,CLR}, {60,CLR} };
    obstacle_profile_t p;

    printf("G2 narrow post at 250 mm\n");
    build(v, 7U, &p);
    check(p.valid, "profile is valid");
    check_near(p.nearest_mm, 250, 0, "nearest range");
    check_near(p.bearing_deg, 0, 2, "bearing");
    check(p.width_mm <= 80U, "narrow object stays narrow");
    check(p.action == OBSTACLE_ACT_BYPASS_LEFT, "free both sides, prefers left");
}

static void g3_wide_box(void)
{
    static const vec_t v[7] = { {-60,CLR}, {-20,305U}, {-10,300U}, {0,298U},
                                {10,301U}, {20,306U}, {60,CLR} };
    obstacle_profile_t p;

    printf("G3 wide box at 300 mm\n");
    build(v, 7U, &p);
    check(p.valid, "profile is valid");
    check_near(p.nearest_mm, 298, 0, "nearest range");
    check_near(p.left_edge_deg, 20, 0, "left edge");
    check_near(p.right_edge_deg, -20, 0, "right edge");
    check_near(p.width_mm, 126, 25, "beam-corrected width");
}

static void g4_wall_on_left(void)
{
    static const vec_t v[7] = { {-60,CLR}, {-30,CLR}, {-10,260U}, {0,250U},
                                {10,265U}, {30,480U}, {60,420U} };
    obstacle_profile_t p;

    printf("G4 separate wall close on the left\n");
    build(v, 7U, &p);
    check(p.valid, "profile is valid");
    check_near(p.left_edge_deg, 10, 0, "wall excluded from object");
    check_near(p.right_edge_deg, -10, 0, "right edge found");
    check(p.gap_left_mm < p.gap_right_mm, "left corridor is the tighter one");
    check(p.action == OBSTACLE_ACT_BYPASS_RIGHT, "goes right");
}

static void g5_too_close(void)
{
    static const vec_t v[3] = { {-20,CLR}, {0,90U}, {20,CLR} };
    obstacle_profile_t p;

    printf("G5 object inside the hard stop distance\n");
    build(v, 3U, &p);
    check(p.action == OBSTACLE_ACT_REVERSE, "backs off");
}

static void g6_bad_input(void)
{
    obstacle_profile_t p;
    const s5_sample_t  one = s5_sample_pack(0, 400U);

    printf("G6 defensive checks\n");
    p.valid  = true;
    p.action = OBSTACLE_ACT_CONTINUE;
    obstacle_profile_build(NULL, 5U, &p);
    check(!p.valid, "NULL samples leaves profile invalid");
    check(p.action == OBSTACLE_ACT_STOP, "defaults to STOP");

    p.valid  = true;
    p.action = OBSTACLE_ACT_CONTINUE;
    obstacle_profile_build(&one, 0U, &p);
    check(!p.valid, "zero count leaves profile invalid");

    obstacle_profile_build(&one, (uint8_t)(OBSTACLE_MAX_SAMPLES + 1U), &p);
    check(!p.valid, "count above the buffer leaves profile invalid");

    obstacle_profile_build(NULL, 0U, NULL);     /* must not crash */
    check(obstacle_plan(NULL) == OBSTACLE_ACT_STOP, "NULL plan STOPs");
}

static void g7_least_deviation(void)
{
    static const vec_t v[7] = { {-60,CLR}, {-30,CLR}, {0,CLR}, {20,320U},
                                {30,300U}, {40,318U}, {60,CLR} };
    obstacle_profile_t p;

    printf("G7 object offset to the left, both sides open\n");
    build(v, 7U, &p);
    check(p.valid, "profile is valid");
    check(p.bearing_deg > 20, "bearing is to the left");
    check(p.swing_right_mm < p.swing_left_mm, "passing right deviates less");
    check(p.action == OBSTACLE_ACT_BYPASS_RIGHT, "takes the shorter detour");
}

static void g8_all_silent(void)
{
    static const vec_t v[5] = { {-60,UNK}, {-30,UNK}, {0,UNK}, {30,UNK}, {60,UNK} };
    obstacle_profile_t p;

    printf("G8 sensor silent in every direction\n");
    build(v, 5U, &p);
    check(!p.valid, "silence is not a valid profile");
    check(p.action == OBSTACLE_ACT_STOP, "action STOP");
}

static void g9_partly_silent(void)
{
    static const vec_t v[5] = { {-60,CLR}, {-30,CLR}, {0,UNK}, {30,CLR}, {60,CLR} };
    obstacle_profile_t p;

    printf("G9 straight ahead silent, sides clear\n");
    build(v, 5U, &p);
    check(!p.valid, "cannot claim clear with a blind spot ahead");
    check(p.action == OBSTACLE_ACT_STOP, "action STOP");
}

static void g10_object_with_gap(void)
{
    static const vec_t v[7] = { {-60,CLR}, {-30,CLR}, {-10,CLR}, {0,250U},
                                {10,CLR}, {30,CLR}, {60,UNK} };
    obstacle_profile_t p;

    printf("G10 object ahead, one direction silent\n");
    build(v, 7U, &p);
    check(p.valid, "object still profiled");
    check(p.confidence_pct <= 70U, "confidence drops for the gap");
}

/* G11: bench, every bearing silent. v0 grid -75..75 step 15,
 * re-gridded to -70, -60, -40, -30, -10, 0, 20, 30, 50, 60, 70. */
static void g11_bench_all_silent(void)
{
    static const vec_t v[11] = { {-70,UNK}, {-60,UNK}, {-40,UNK}, {-30,UNK},
                                 {-10,UNK}, {0,UNK}, {20,UNK}, {30,UNK},
                                 {50,UNK}, {60,UNK}, {70,UNK} };
    obstacle_profile_t p;

    printf("G11 bench: every bearing silent\n");
    build(v, 11U, &p);
    check(!p.valid, "silent sweep is not a valid profile");
    check(p.action == OBSTACLE_ACT_STOP, "silent sweep STOPs");
}

/* G12: bench, side object at -75/-60/-45 (170/170/183 mm) and an
 * object 177 mm dead ahead. Re-gridded as G11. */
static void g12_bench_side_object(void)
{
    static const vec_t v[11] = { {-70,170U}, {-60,170U}, {-40,183U}, {-30,CLR},
                                 {-10,CLR}, {0,177U}, {20,177U}, {30,CLR},
                                 {50,CLR}, {60,CLR}, {70,CLR} };
    obstacle_profile_t p;

    printf("G12 bench: closer object off to the side\n");
    build(v, 11U, &p);
    check(p.valid, "profile is valid");
    check_near(p.nearest_mm, 177, 0, "object ahead chosen, not side");
    check(p.bearing_deg >= 0, "bearing is ahead, not at -60");
    check(p.gap_right_mm < p.gap_left_mm, "side object blocks the right");
    check(p.action == OBSTACLE_ACT_BYPASS_LEFT, "goes left");
}

static void g13_half_silent(void)
{
    static const vec_t left_dead[7] = { {-60,CLR}, {-30,CLR}, {-10,255U},
                                        {0,250U}, {10,255U}, {30,UNK}, {60,UNK} };
    static const vec_t right_dead[7] = { {-60,UNK}, {-30,UNK}, {-10,255U},
                                         {0,250U}, {10,255U}, {30,CLR}, {60,CLR} };
    obstacle_profile_t p;

    printf("G13 object ahead, one side silent\n");
    build(left_dead, 7U, &p);
    check(p.gap_left_mm == 0U, "silent left side has no corridor");
    check(p.action != OBSTACLE_ACT_BYPASS_LEFT, "never plans through it");

    build(right_dead, 7U, &p);
    check(p.gap_right_mm == 0U, "silent right side has no corridor");
    check(p.action != OBSTACLE_ACT_BYPASS_RIGHT, "never plans through it");
}

/* G14: clear path, some bearings silent. Re-gridded as G11. */
static void g14_partly_silent_clear(void)
{
    static const vec_t one_dead[11] = { {-70,CLR}, {-60,CLR}, {-40,CLR}, {-30,CLR},
                                        {-10,CLR}, {0,CLR}, {20,CLR}, {30,CLR},
                                        {50,CLR}, {60,CLR}, {70,UNK} };
    static const vec_t three_dead[11] = { {-70,UNK}, {-60,UNK}, {-40,UNK}, {-30,CLR},
                                          {-10,CLR}, {0,CLR}, {20,CLR}, {30,CLR},
                                          {50,CLR}, {60,CLR}, {70,CLR} };
    obstacle_profile_t p;

    printf("G14 clear path, some bearings silent\n");
    build(one_dead, 11U, &p);
    check(p.action == OBSTACLE_ACT_CONTINUE, "1 of 11 silent still continues");
    build(three_dead, 11U, &p);
    check(p.action == OBSTACLE_ACT_SLOW, "3 of 11 silent slows down");
}

/* G15 (v2): "past the obstacle" distance, tutor feedback 6.5. */
static void g15_pass_distance(void)
{
    static const vec_t v[7] = { {-60,CLR}, {-30,CLR}, {-10,CLR}, {0,250U},
                                {10,CLR}, {30,CLR}, {60,CLR} };
    obstacle_profile_t p;

    printf("G15 pass distance = nearest + depth + robot length + margin\n");
    build(v, 7U, &p);
    check_near(obstacle_pass_distance_mm(&p),
               250 + (int32_t)OBSTACLE_ASSUMED_DEPTH_MM +
               (int32_t)OBSTACLE_ROBOT_LENGTH_MM +
               (int32_t)OBSTACLE_PASS_MARGIN_MM, 0, "pass distance");

    p.nearest_mm = OBSTACLE_RANGE_CLEAR;
    check(obstacle_pass_distance_mm(&p) == 0U, "no obstacle, no distance");
    check(obstacle_pass_distance_mm(NULL) == 0U, "NULL safe");
}

/* G16 (v2): in-path filter used to centre the fine scan. */
static void g16_in_path(void)
{
    printf("G16 in-path band is +/-135 mm\n");
    check(obstacle_in_path(0, 250U), "dead ahead is in the path");
    check(!obstacle_in_path(-60, 170U), "side wall at -60 deg, 170 mm is not");
    check(!obstacle_in_path(0, OBSTACLE_RANGE_CLEAR), "CLEAR is not an object");
    check(!obstacle_in_path(0, OBSTACLE_RANGE_UNKNOWN), "UNKNOWN is not an object");
    check(!obstacle_in_path(0, 950U), "background is not the obstacle");
}

int main(void)
{
    g1_clear_track();
    g2_narrow_post();
    g3_wide_box();
    g4_wall_on_left();
    g5_too_close();
    g6_bad_input();
    g7_least_deviation();
    g8_all_silent();
    g9_partly_silent();
    g10_object_with_gap();
    g11_bench_all_silent();
    g12_bench_side_object();
    g13_half_silent();
    g14_partly_silent_clear();
    g15_pass_distance();
    g16_in_path();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
