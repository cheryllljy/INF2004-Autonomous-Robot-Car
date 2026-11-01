/**
 * @file    test_pack.c
 * @brief   HOST TEST. CR1 encoders: every field at and around its
 *          limits, round trips, saturation, the wire format, and the
 *          message-size comparison (binary packet vs the text line it
 *          replaces).
 */

#include "obstacle_model.h"
#include "drv_ultrasonic.h"
#include "v1_adapter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static uint32_t g_seed = 99U;

static uint32_t rnd(uint32_t n)
{
    g_seed = (g_seed * 1103515245U) + 12345U;
    return (g_seed >> 8) % n;
}

static void p1_range(void)
{
    uint32_t mm;
    bool     ok = true;

    printf("P1 12-bit range code\n");

    for (mm = 0U; mm <= 4093U; mm++)
    {
        ok = ok && (s5_range_decode(s5_range_encode((uint16_t)mm)) == mm);
    }
    check(ok, "every distance 0..4093 mm survives");
    check(s5_range_decode(s5_range_encode(OBSTACLE_RANGE_CLEAR)) ==
          OBSTACLE_RANGE_CLEAR, "CLEAR survives as the 16-bit code");
    check(s5_range_decode(s5_range_encode(OBSTACLE_RANGE_UNKNOWN)) ==
          OBSTACLE_RANGE_UNKNOWN, "UNKNOWN survives as the 16-bit code");
    check(s5_range_encode(5000U) == S5_RANGE12_MAX_MM,
          "a too-big distance saturates, never wraps into a sentinel");
    check(s5_range_encode(4094U) == S5_RANGE12_MAX_MM,
          "4094 mm does not alias CLEAR");
}

static void p2_sample(void)
{
    int16_t b;
    bool    ok = true;

    printf("P2 16-bit sample (4-bit bearing index + 12-bit range)\n");

    for (b = -70; b <= 70; b = (int16_t)(b + 10))
    {
        const uint16_t ranges[4] = { 30U, 2500U, OBSTACLE_RANGE_CLEAR,
                                     OBSTACLE_RANGE_UNKNOWN };
        uint8_t        i;

        for (i = 0U; i < 4U; i++)
        {
            const s5_sample_t s = s5_sample_pack(b, ranges[i]);

            ok = ok && (s5_sample_bearing(s) == b) &&
                 (s5_sample_range(s) == ranges[i]);
        }
    }
    check(ok, "15 bearings x 4 edge ranges round-trip");
    check(sizeof(s5_sample_t) == 2U, "sample is 2 bytes");
    check(s5_sample_bearing(s5_sample_pack(-75, 100U)) == -70,
          "servo-limit bearing clamps to the last grid point");
    check(s5_sample_bearing(s5_sample_pack(14, 100U)) == 10,
          "off-grid bearing rounds to nearest grid point");
    check(!s5_bearing_on_grid(15) && s5_bearing_on_grid(-70),
          "grid check");
}

static void p3_front(void)
{
    uint32_t st;
    bool     ok = true;

    printf("P3 32-bit front word (range 12 | status 3 | stamp 17)\n");

    for (st = 0U; st < (uint32_t)ULTRASONIC_STATUS_COUNT; st++)
    {
        const s5_front_t w = s5_front_pack(2500U, st, 123456U);

        ok = ok && (s5_front_status(w) == st) &&
             (s5_front_range(w) == 2500U);
    }
    check(ok, "every status code with the max range");

    {
        const s5_front_t w = s5_front_pack(245U, 0U, 131071U);

        check(s5_age_ms(s5_front_stamp(w), 131071U + 250U) == 250U,
              "age correct across the 17-bit wrap");
        check(s5_age_ms(s5_front_stamp(w), 131071U) == 0U, "age 0 now");
    }
}

static void p4_profile_round_trip(void)
{
    int  i;
    bool ok = true;

    printf("P4 profile: 16 B, 2000 random in-range round trips\n");

    for (i = 0; i < 2000; i++)
    {
        obstacle_profile_t        p;
        obstacle_profile_t        q;
        obstacle_profile_packed_t pk;

        p.valid          = (rnd(2U) != 0U);
        p.nearest_mm     = (rnd(10U) == 0U) ? OBSTACLE_RANGE_CLEAR
                                            : (uint16_t)(30U + rnd(2471U));
        p.bearing_deg    = (int16_t)((int32_t)rnd(151U) - 75);
        p.width_mm       = (uint16_t)rnd(4096U);
        p.left_edge_deg  = (int16_t)((int32_t)rnd(151U) - 75);
        p.right_edge_deg = (int16_t)((int32_t)rnd(151U) - 75);
        p.gap_left_mm    = (uint16_t)rnd(OBSTACLE_GAP_OPEN_MM + 1U);
        p.gap_right_mm   = (uint16_t)rnd(OBSTACLE_GAP_OPEN_MM + 1U);
        p.swing_left_mm  = (uint16_t)rnd(4096U);
        p.swing_right_mm = (uint16_t)rnd(4096U);
        p.confidence_pct = (uint8_t)rnd(101U);
        p.sample_count   = (uint8_t)rnd(OBSTACLE_MAX_SAMPLES + 1U);
        p.timestamp_ms   = rnd(1000000U);
        p.action         = (obstacle_action_t)rnd(OBSTACLE_ACT_COUNT);

        obstacle_profile_pack(&p, &pk);
        obstacle_profile_unpack(&pk, &q);

        ok = ok && (q.valid == p.valid) && (q.nearest_mm == p.nearest_mm) &&
             (q.bearing_deg == p.bearing_deg) && (q.width_mm == p.width_mm) &&
             (q.left_edge_deg == p.left_edge_deg) &&
             (q.right_edge_deg == p.right_edge_deg) &&
             (q.gap_left_mm == p.gap_left_mm) &&
             (q.gap_right_mm == p.gap_right_mm) &&
             (q.swing_left_mm == p.swing_left_mm) &&
             (q.swing_right_mm == p.swing_right_mm) &&
             (q.confidence_pct == p.confidence_pct) &&
             (q.sample_count == p.sample_count) &&
             (q.action == p.action) &&
             (q.timestamp_ms == (p.timestamp_ms & 0x1FFFFU));
    }
    check(ok, "every field survives (stamp keeps its low 17 bits)");

    {
        obstacle_profile_t        p;
        obstacle_profile_t        q;
        obstacle_profile_packed_t pk;

        memset(&p, 0, sizeof(p));
        p.valid        = true;
        p.nearest_mm   = 200U;
        p.width_mm     = 9000U;     /* impossible, must saturate */
        p.gap_left_mm  = 3000U;
        p.action       = OBSTACLE_ACT_BYPASS_LEFT;
        obstacle_profile_pack(&p, &pk);
        obstacle_profile_unpack(&pk, &q);
        check(q.width_mm == 4095U, "oversized width saturates at 4095");
        check(q.gap_left_mm == 2047U, "oversized gap saturates at 2047");
        check(q.action == OBSTACLE_ACT_BYPASS_LEFT,
              "saturation does not spill into the next field");

        pk.w[3] |= (7U << 12);     /* corrupt action -> code 7 */
        obstacle_profile_unpack(&pk, &q);
        check(q.action == OBSTACLE_ACT_STOP,
              "a corrupt action code decodes as STOP, never a manoeuvre");
    }
}

static void p5_health(void)
{
    obstacle_health_t        h;
    obstacle_health_t        g;
    obstacle_health_packed_t pk;

    printf("P5 health: 8 B round trip at limits\n");

    memset(&h, 0, sizeof(h));
    h.state             = OBSTACLE_ST_FAULT;
    h.fault             = OBSTACLE_FAULT_ASSIST_TIMEOUT;
    h.estop_requested   = true;
    h.has_profile       = true;
    h.odometry_ok       = true;
    h.reverses          = 2U;
    h.last_status       = (uint8_t)ULTRASONIC_BAD_PARAM;
    h.deadline_overruns = 255U;
    h.sensor_faults     = 255U;
    h.scans_completed   = 65535U;
    h.build_us_max      = 65535U;
    obstacle_health_pack(&h, &pk);
    memset(&g, 0, sizeof(g));               /* so padding compares equal */
    obstacle_health_unpack(&pk, &g);

    check(memcmp(&h, &g, sizeof(h)) == 0, "all fields at their maximum");
    check(sizeof(obstacle_health_packed_t) == 8U, "health is 8 bytes");
}

static void p6_us_result(void)
{
    ultrasonic_result_t r;

    printf("P6 ping result: 4 B (range 12 | echo 15 | status 3)\n");

    r = drv_us_result_pack(1234U, 7195U, ULTRASONIC_OK);
    check((drv_us_result_mm(r) == 1234U) &&
          (drv_us_result_echo_us(r) == 7195U) &&
          (drv_us_result_status(r) == ULTRASONIC_OK), "round trip");

    r = drv_us_result_pack(OBSTACLE_RANGE_UNKNOWN, 40000U,
                           ULTRASONIC_NO_ECHO);
    check(drv_us_result_echo_us(r) == 32767U, "echo saturates at 15 bits");
    check(drv_us_result_status(r) == ULTRASONIC_NO_ECHO,
          "saturation does not spill into status");
}

static void p7_telemetry(void)
{
    obstacle_profile_t        p;
    obstacle_profile_packed_t pk;
    obstacle_profile_packed_t pk2;
    obstacle_health_t         h;
    obstacle_health_packed_t  hk;
    obstacle_health_packed_t  hk2;
    s5_front_t                f;
    s5_front_t                f2;
    uint8_t                   buf[64];
    char                      text[256];
    uint32_t                  n;
    int                       text_len;

    printf("P7 telemetry: binary packet vs the JSON line it replaces\n");

    memset(&p, 0, sizeof(p));
    p.valid = true; p.nearest_mm = 248U; p.bearing_deg = 2;
    p.width_mm = 62U; p.left_edge_deg = 10; p.right_edge_deg = -10;
    p.gap_left_mm = 1500U; p.gap_right_mm = 1500U;
    p.swing_left_mm = 166U; p.swing_right_mm = 165U;
    p.confidence_pct = 80U; p.sample_count = 12U; p.timestamp_ms = 51234U;
    p.action = OBSTACLE_ACT_BYPASS_RIGHT;
    memset(&h, 0, sizeof(h));
    h.state = OBSTACLE_ST_ASSIST; h.has_profile = true; h.odometry_ok = true;
    h.scans_completed = 3U; h.build_us_max = 41U;

    obstacle_profile_pack(&p, &pk);
    obstacle_health_pack(&h, &hk);
    f = s5_front_pack(248U, (uint32_t)ULTRASONIC_OK, 51234U);

    n = s5_tlm_encode(buf, (uint32_t)sizeof(buf), f, &hk, &pk);
    check(n == S5_TLM_LEN, "packet is 29 bytes");
    check(s5_tlm_decode(buf, n, &f2, &hk2, &pk2) &&
          (f2 == f) && (memcmp(&hk, &hk2, sizeof(hk)) == 0) &&
          (memcmp(&pk, &pk2, sizeof(pk)) == 0), "decode gives the same words");
    check(s5_tlm_encode(buf, 28U, f, &hk, &pk) == 0U,
          "too-small buffer is refused, not overrun");
    buf[0] = 0x25U;
    check(!s5_tlm_decode(buf, n, &f2, &hk2, &pk2),
          "a different version nibble is rejected");

    /* The same information as a text line, the way it would be sent
     * without CR1. Field names kept short to be fair to text. */
    text_len = snprintf(text, sizeof(text),
        "{\"front\":%u,\"st\":%u,\"state\":\"ASSIST\",\"fault\":\"NONE\","
        "\"valid\":1,\"near\":%u,\"brg\":%d,\"w\":%u,\"le\":%d,\"re\":%d,"
        "\"gl\":%u,\"gr\":%u,\"sl\":%u,\"sr\":%u,\"conf\":%u,\"n\":%u,"
        "\"act\":\"BYPASS_RIGHT\",\"scans\":%u,\"t\":%lu}",
        248U, 0U, (unsigned)p.nearest_mm, p.bearing_deg,
        (unsigned)p.width_mm, p.left_edge_deg, p.right_edge_deg,
        (unsigned)p.gap_left_mm, (unsigned)p.gap_right_mm,
        (unsigned)p.swing_left_mm, (unsigned)p.swing_right_mm,
        (unsigned)p.confidence_pct, (unsigned)p.sample_count,
        (unsigned)h.scans_completed, (unsigned long)p.timestamp_ms);

    printf("  MESSAGE SIZE  text %d B  ->  binary %u B  (%d %% smaller)\n",
           text_len, (unsigned)n, (int)(100 - ((100 * (int)n) / text_len)));
    check(n < (uint32_t)text_len, "binary is smaller than text");
}

static void p8_size_table(void)
{
    printf("P8 RAM per data item, v1 vs v2 (this host's ABI; the ARM\n"
           "   numbers come from arm-none-eabi-size, see TEST_LOG.md)\n");
    printf("  %-26s %6s %6s\n", "item", "v1 B", "v2 B");
    printf("  %-26s %6u %6u\n", "sample", v1_sizeof_sample(),
           (unsigned)sizeof(s5_sample_t));
    printf("  %-26s %6u %6u\n", "sample buffer x16",
           16U * v1_sizeof_sample(), 16U * (unsigned)sizeof(s5_sample_t));
    printf("  %-26s %6u %6u\n", "published profile", v1_sizeof_profile(),
           (unsigned)sizeof(obstacle_profile_packed_t));
    printf("  %-26s %6u %6u\n", "health", v1_sizeof_health(),
           (unsigned)sizeof(obstacle_health_packed_t));
    printf("  %-26s %6u %6u\n", "ping result", v1_sizeof_us_result(),
           (unsigned)sizeof(ultrasonic_result_t));
    printf("  %-26s %6u %6u\n", "ultrasonic stats", v1_sizeof_us_stats(),
           (unsigned)sizeof(ultrasonic_stats_t));

    check(sizeof(s5_sample_t) < v1_sizeof_sample(), "sample shrank");
    check(sizeof(obstacle_profile_packed_t) < v1_sizeof_profile(),
          "profile shrank");
    check(sizeof(obstacle_health_packed_t) < v1_sizeof_health(),
          "health shrank");
}

int main(void)
{
    p1_range();
    p2_sample();
    p3_front();
    p4_profile_round_trip();
    p5_health();
    p6_us_result();
    p7_telemetry();
    p8_size_table();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);

    return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
