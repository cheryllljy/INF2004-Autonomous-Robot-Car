#include "b3_barcode.h"
#include "b3_line.h"
#include "school_samples.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int g_checks;
#define CHECK(expression) do { \
    ++g_checks; \
    if (!(expression)) { \
        (void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static void make_frame(char symbol, uint32_t narrow, uint32_t wide,
                        uint32_t widths[B3_BARCODE_RUNS])
{
    const char frame[3] = {'*', symbol, '*'};
    size_t out = 0U;
    size_t c;
    for (c = 0U; c < 3U; ++c)
    {
        uint16_t pattern;
        size_t bit;
        CHECK(b3_code39_pattern(frame[c], &pattern));
        for (bit = 0U; bit < 9U; ++bit)
        {
            widths[out++] = ((pattern & (1U << (8U - bit))) != 0U) ? wide : narrow;
        }
        if (c < 2U)
        {
            widths[out++] = narrow;
        }
    }
    CHECK(B3_BARCODE_RUNS == out);
}

static void reverse_frame(uint32_t widths[B3_BARCODE_RUNS])
{
    size_t i;
    for (i = 0U; i < B3_BARCODE_RUNS / 2U; ++i)
    {
        uint32_t temporary = widths[i];
        widths[i] = widths[B3_BARCODE_RUNS - 1U - i];
        widths[B3_BARCODE_RUNS - 1U - i] = temporary;
    }
}

static void test_pdf_fixtures(void)
{
    size_t page;
    b3_code39_result_t decoded;
    for (page = 0U; page < 4U; ++page)
    {
        uint32_t reversed[B3_BARCODE_RUNS];
        CHECK(b3_code39_decode(g_school_runs[page], &decoded));
        CHECK(g_school_symbols[page] == decoded.symbol);
        CHECK(!decoded.reversed);
        (void)memcpy(reversed, g_school_runs[page], sizeof(reversed));
        reverse_frame(reversed);
        CHECK(b3_code39_decode(reversed, &decoded));
        CHECK(g_school_symbols[page] == decoded.symbol);
        CHECK(decoded.reversed);
    }
    (void)puts("PASS: all four school PDF samples, both reading directions");
}

static void test_alphabet_and_speed(void)
{
    const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    const uint32_t scales[] = {1000U, 10000U, 50000U};
    size_t c;
    size_t scale;
    size_t i;
    uint16_t pattern;
    CHECK(b3_code39_pattern('A', &pattern) && (0x109U == pattern));
    CHECK(b3_code39_pattern('B', &pattern) && (0x049U == pattern));
    CHECK(b3_code39_pattern('C', &pattern) && (0x148U == pattern));
    CHECK(b3_code39_pattern('D', &pattern) && (0x019U == pattern));
    CHECK(!b3_code39_pattern('a', &pattern));
    for (c = 0U; c < sizeof(alphabet) - 1U; ++c)
    {
        for (scale = 0U; scale < sizeof(scales) / sizeof(scales[0]); ++scale)
        {
            uint32_t widths[B3_BARCODE_RUNS];
            b3_code39_result_t decoded;
            make_frame(alphabet[c], scales[scale], scales[scale] * 3U, widths);
            /* Deterministic +/- 8% edge-width error. */
            for (i = 0U; i < B3_BARCODE_RUNS; ++i)
            {
                widths[i] = widths[i] * (92U + (uint32_t)((i * 7U) % 17U)) / 100U;
            }
            CHECK(b3_code39_decode(widths, &decoded));
            CHECK(alphabet[c] == decoded.symbol);
            CHECK(!decoded.reversed);
            reverse_frame(widths);
            CHECK(b3_code39_decode(widths, &decoded));
            CHECK(alphabet[c] == decoded.symbol);
            CHECK(decoded.reversed);
        }
    }
    (void)puts("PASS: 43 symbols at three time scales, jitter and reverse scans");
}

static void test_invalid_frames(void)
{
    uint32_t widths[B3_BARCODE_RUNS];
    b3_code39_result_t result;
    size_t i;
    make_frame('*', 1000U, 3000U, widths);
    CHECK(!b3_code39_decode(widths, &result));
    
    /* Too close to narrow width: reject. */
    make_frame('A', 1000U, 1700U, widths);
    CHECK(!b3_code39_decode(widths, &result));

    /* Code 39 allows 2:1 and 2.2:1: accept. */
    make_frame('A', 1000U, 2000U, widths);
    CHECK(b3_code39_decode(widths, &result));
    make_frame('A', 1000U, 2200U, widths);
    CHECK(b3_code39_decode(widths, &result));

    /* Within school-barcode tolerance: accept. */
    make_frame('A', 1000U, 2500U, widths);
    CHECK(b3_code39_decode(widths, &result));

    /* Nominal supplied barcode ratio: accept. */
    make_frame('A', 1000U, 3000U, widths);
    CHECK(b3_code39_decode(widths, &result));

    /* Too wide relative to narrow: reject. */
    make_frame('A', 1000U, 5600U, widths);
    CHECK(!b3_code39_decode(widths, &result));

    make_frame('A', 1000U, 1200U, widths);
    CHECK(!b3_code39_decode(widths, &result));
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        widths[i] = UINT32_MAX;
    }
    CHECK(!b3_code39_decode(widths, &result));
    CHECK(!b3_code39_decode(NULL, &result));
    (void)puts("PASS: malformed framing, bad gaps, missing pulses and bad width ratios");
}

static b3_edge_t feed_frame(b3_barcode_t *p_scan, uint32_t start,
                            const uint32_t times[B3_BARCODE_RUNS],
                            const uint32_t *p_ticks, uint32_t leading_us)
{
    size_t i;
    b3_barcode_result_t result;
    b3_edge_t edge = {0};
    edge.time_us = start - leading_us;
    edge.position_valid = (NULL != p_ticks);
    b3_barcode_init(p_scan, edge);
    edge.time_us = start;
    edge.position_ticks = 1000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(p_scan, edge, &result));
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        edge.time_us += times[i];
        edge.position_ticks += (NULL != p_ticks) ? p_ticks[i] : 0U;
        edge.black = !edge.black;
        CHECK(!b3_barcode_edge(p_scan, edge, &result));
    }
    return edge;
}

static void test_stream_and_quiet(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge;
    make_frame('A', 10000U, 30000U, times);
    edge = feed_frame(&scan, 500000U, times, NULL, 200000U);
    CHECK(scan.pending);
    /* nyx, 2026-11-09: both '*' read, so no trailing white is needed: the
     * letter comes at the first poll after the last bar */
    edge.time_us += 1000U;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK('A' == result.symbol);
    CHECK(!result.used_encoder);
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    edge = feed_frame(&scan, 500000U, times, NULL, 50000U);
    edge.time_us += 200000U;
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    CHECK(1U == scan.rejected);
    /* nyx, 2026-11-06: the white measured and needed are kept for the VC. */
    CHECK(B3_REJECT_LEADING_QUIET == scan.reject_reason);
    CHECK(B3_BARCODE_RUNS == scan.reject_count);
    CHECK(!scan.reject_encoder);
    CHECK(50000U == scan.reject_lead);
    CHECK((10000U * B3_QUIET_NARROW_MULTIPLE) == scan.reject_need);
    /* nyx, 2026-11-09: black again 4 mm after the last bar (the car's
     * reason 8 at 30 mm/s, 136 ms) still gives the letter, at that black
     * edge, with no reject */
    edge = feed_frame(&scan, 500000U, times, NULL, 200000U);
    edge.time_us += 4000U;
    edge.black = true;
    CHECK(b3_barcode_edge(&scan, edge, &result));
    CHECK('A' == result.symbol);
    CHECK(0U == scan.rejected);
    edge.time_us += 30000U;
    edge.black = false;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    CHECK(0U == scan.rejected);
    /* ... but with the stop '*' spoiled (here a 'B' third, as the first 29
     * elements of *AB* read), black that soon after is the next symbol of
     * a longer barcode: still rejected for the short trailing white */
    {
        uint32_t ab[B3_BARCODE_RUNS];
        uint32_t b[B3_BARCODE_RUNS];
        size_t i;

        make_frame('A', 10000U, 30000U, ab);
        make_frame('B', 10000U, 30000U, b);
        for (i = 20U; i < B3_BARCODE_RUNS; ++i)
        {
            ab[i] = b[i - 10U];
        }
        edge = feed_frame(&scan, 500000U, ab, NULL, 200000U);
        CHECK(scan.pending && ('A' == scan.decoded.symbol) &&
              !scan.decoded.both_stars);
        edge.time_us += 10000U; /* the inter-character gap */
        edge.black = true;
        CHECK(!b3_barcode_edge(&scan, edge, &result));
        edge.time_us += 10000U;
        edge.black = false;
        CHECK(!b3_barcode_edge(&scan, edge, &result));
        CHECK(1U == scan.rejected);
        CHECK(B3_REJECT_TRAILING_QUIET == scan.reject_reason);
    }
    /* Quiet completion can be discovered by the next barcode's first edge. */
    edge = feed_frame(&scan, 500000U, times, NULL, 200000U);
    edge.time_us += 200000U;
    edge.black = true;
    CHECK(b3_barcode_edge(&scan, edge, &result));
    CHECK('A' == result.symbol);
    (void)puts("PASS: leading/trailing quiet zones and exactly one event per frame");
}

static void test_encoder_widths(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    uint32_t ticks[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_code39_result_t raw_result;
    b3_edge_t edge;
    size_t i;
    make_frame('Z', 10000U, 30000U, times);
    make_frame('Z', 10U, 30U, ticks);
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        if (0U == i % 2U)
        {
            times[i] *= 4U; /* Deliberate speed variation within characters. */
        }
    }
    CHECK(!b3_code39_decode(times, &raw_result));
    edge = feed_frame(&scan, 500000U, times, ticks, 300000U);
    edge.time_us += 200000U;
    edge.position_ticks += 200U;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK('Z' == result.symbol);
    CHECK(result.used_encoder);
    /* Coarse wheel encoders cannot resolve narrow bars: time fallback. */
    make_frame('B', 10000U, 30000U, times);
    make_frame('B', 1U, 3U, ticks);
    edge = feed_frame(&scan, 500000U, times, ticks, 200000U);
    edge.time_us += 200000U;
    edge.position_ticks += 200U;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK(!result.used_encoder);
    CHECK('B' == result.symbol);
    (void)puts("PASS: encoder width fusion and low-resolution encoder fallback");
}

static void test_any_size(void)
{
    static const uint32_t narrow[] = {300U, 5000U, 30000U, 150000U, 250000U};
    uint32_t times[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge;
    size_t k;
    /* From 0.03 mm to 25 mm narrow bars at 100 mm/s, quiet zone 10 narrow. */
    for (k = 0U; k < sizeof(narrow) / sizeof(narrow[0]); ++k)
    {
        make_frame('Q', narrow[k], narrow[k] * 3U, times);
        edge = feed_frame(&scan, 500000U, times, NULL, narrow[k] * 11U);
        CHECK(scan.pending);
        edge.time_us += narrow[k] * 11U;
        CHECK(b3_barcode_poll(&scan, edge, &result));
        CHECK('Q' == result.symbol);
    }
    /* *Z* captured on the car (H09, 2026-10-30): the threshold skews narrow
       black to ~15 ms and wide white to ~110 ms, a 7.5x spread. */
    {
        static const uint32_t car[B3_BARCODE_RUNS] = {
            16827U, 81236U, 18807U, 37487U, 76933U, 35960U, 74065U, 35060U,
            19679U, 36965U, 21383U, 112812U, 77487U, 40733U, 75796U, 42415U,
            16308U, 42326U, 15073U, 43733U, 14813U, 101473U, 13871U, 34415U,
            65280U, 38852U, 77933U, 41790U, 16419U};
        edge = feed_frame(&scan, 500000U, car, NULL, 300000U);
        CHECK(scan.pending);
        edge.time_us += 300000U;
        CHECK(b3_barcode_poll(&scan, edge, &result));
        CHECK('Z' == result.symbol);
    }
    /* A white run over B3_MAX_ELEMENT_RATIO x the shortest element aborts at
       once, even though it is far under the absolute B3_MAX_ELEMENT_US cap. */
    make_frame('Q', 5000U, 15000U, times);
    edge = feed_frame(&scan, 500000U, times, NULL, 60000U);
    b3_barcode_init(&scan, edge);
    edge.black = false;
    edge.time_us += 60000U;
    b3_barcode_init(&scan, edge);
    edge.time_us += 60000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(&scan, edge, &result));   /* start */
    edge.time_us += 5000U;
    edge.black = false;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    edge.time_us += 5000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    CHECK(b3_barcode_active(&scan));
    edge.time_us += (B3_MAX_ELEMENT_RATIO + 1U) * 5000U;
    edge.black = true;
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    CHECK(!b3_barcode_active(&scan));
    (void)puts("PASS: any barcode size from 0.03 mm to 25 mm bars; relative long-element abort; skewed car scan");
}

static void test_timeout_noise_wrap(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge = {0};
    b3_barcode_init(&scan, edge);
    edge.time_us = 200000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    edge.time_us += 10U;
    edge.black = false;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    /* A 10 us blip is sensor chatter: the start is dropped, not a reject. */
    CHECK(0U == scan.rejected);
    CHECK(1U == scan.glitches);
    CHECK(!b3_barcode_active(&scan));
    edge.time_us += 200000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    edge.time_us += B3_MAX_ELEMENT_US + 1U;
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    CHECK(!b3_barcode_active(&scan));
    make_frame('C', 10000U, 30000U, times);
    edge = feed_frame(&scan, UINT32_MAX - 50000U, times, NULL, 200000U);
    edge.time_us += 200000U;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK('C' == result.symbol);
    (void)puts("PASS: noise rejection, stuck-input timeout and 32-bit timer rollover");
}

/* Car slows from base speed to half over the barcode: later bars take twice
 * as long, and the measured speed on each edge rescales them back. */
static void test_slowdown_compensation(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge = {0};
    size_t i;
    const uint16_t slow = (uint16_t)(B3_BASE_SPEED_MM_S / 2);
    make_frame('A', 10000U, 30000U, times);
    edge.time_us = 300000U;
    b3_barcode_init(&scan, edge);
    edge.time_us = 500000U;
    edge.black = true;
    edge.speed_mm_s = B3_BASE_SPEED_MM_S;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        bool late = (i >= 10U);
        edge.time_us += late ? (times[i] * 2U) : times[i];
        edge.speed_mm_s = late ? slow : (uint16_t)B3_BASE_SPEED_MM_S;
        edge.black = !edge.black;
        CHECK(!b3_barcode_edge(&scan, edge, &result));
    }
    CHECK(scan.pending);
    edge.time_us += 200000U; /* 100 ms at base speed, needs 10 x narrow */
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK('A' == result.symbol);
    (void)puts("PASS: slowing down over a barcode is rescaled by measured speed");
}

/* nyx, 2026-11-06: one 41 us IR_B flicker in the white just before the
 * first bar (or chatter on the bar's own edge) used to restart the white
 * count there, so a good scan was rejected: 29/29 elements, reason 7. The
 * white now carries on across the blip, in time and encoder mode. */
static void test_leading_blip(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    uint32_t ticks[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge;
    size_t i;
    uint32_t gap;
    uint32_t mode;
    make_frame('A', 10000U, 30000U, times);
    make_frame('A', 20U, 60U, ticks);
    for (mode = 0U; mode < 2U; ++mode)
    {
        /* gap from the end of the blip to the first bar */
        for (gap = 300U; gap <= 80000U; gap *= 4U)
        {
            edge = (b3_edge_t){0};
            edge.position_valid = (1U == mode);
            edge.time_us = 300000U;
            b3_barcode_init(&scan, edge);
            edge.time_us = 500000U - gap - 41U;
            edge.position_ticks = 400U - (gap / 500U);
            edge.black = true;
            CHECK(!b3_barcode_edge(&scan, edge, &result));
            edge.time_us += 41U;
            edge.black = false;
            CHECK(!b3_barcode_edge(&scan, edge, &result));
            CHECK(!b3_barcode_active(&scan));
            edge.time_us = 500000U;
            edge.position_ticks = 400U;
            edge.black = true;
            CHECK(!b3_barcode_edge(&scan, edge, &result));
            for (i = 0U; i < B3_BARCODE_RUNS; ++i)
            {
                edge.time_us += times[i];
                edge.position_ticks += ticks[i];
                edge.black = !edge.black;
                CHECK(!b3_barcode_edge(&scan, edge, &result));
            }
            CHECK(scan.pending);
            edge.time_us += 100000U;
            edge.position_ticks += 200U;
            CHECK(b3_barcode_poll(&scan, edge, &result));
            CHECK('A' == result.symbol);
            CHECK((1U == mode) == result.used_encoder);
            CHECK(0U == scan.rejected);
            CHECK(1U == scan.glitches);
        }
    }
    /* Encoder mode: a white that really is too short still fails, with the
       numbers in ticks (1000 ticks before, needs 10 x 200). */
    make_frame('A', 200U, 600U, ticks);
    edge = feed_frame(&scan, 500000U, times, ticks, 300000U);
    edge.time_us += 200000U;
    edge.position_ticks += 3000U;
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    CHECK(1U == scan.rejected);
    CHECK(B3_REJECT_LEADING_QUIET == scan.reject_reason);
    CHECK(scan.reject_encoder);
    CHECK(1000U == scan.reject_lead);
    CHECK((200U * B3_QUIET_NARROW_MULTIPLE) == scan.reject_need);
    (void)puts("PASS: a blip just before the first bar keeps the white before it");
}

/* nyx, 2026-11-06: an 18 mm black mark 105-125 mm before a barcode at
 * 50 mm/s (20 ms/mm, 6 ticks/mm). The mark starts a scan, the white after
 * it passes B3_MAX_ELEMENT_US and the scan is dropped. The white before the
 * barcode must still count from the end of the mark (it used to restart at
 * the drop, leaving 5-25 mm, so the barcode failed with reason 7). With
 * polls every 10 ms, and with no poll in the white (dropped on the edge). */
static void test_mark_before_barcode(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    uint32_t ticks[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge;
    size_t i;
    uint32_t mode;
    uint32_t gap_mm;
    make_frame('A', 40000U, 100000U, times); /* narrow 2 mm, wide 5 mm */
    make_frame('A', 12U, 30U, ticks);
    for (mode = 0U; mode < 4U; ++mode)
    {
        bool polled = (mode < 2U);
        for (gap_mm = 105U; gap_mm <= 125U; gap_mm += 10U)
        {
            uint32_t white_end;
            edge = (b3_edge_t){0};
            edge.position_valid = (1U == (mode & 1U));
            edge.time_us = 100000U;
            b3_barcode_init(&scan, edge);
            edge.time_us = 1000000U;
            edge.position_ticks = 6000U;
            edge.black = true;
            CHECK(!b3_barcode_edge(&scan, edge, &result));
            edge.time_us += 18U * 20000U;
            edge.position_ticks += 18U * 6U;
            edge.black = false;
            CHECK(!b3_barcode_edge(&scan, edge, &result));
            CHECK(b3_barcode_active(&scan));
            white_end = edge.time_us + (gap_mm * 20000U);
            if (polled)
            {
                b3_edge_t now = edge;
                while ((now.time_us + 10000U) < white_end)
                {
                    now.time_us += 10000U;
                    now.position_ticks = edge.position_ticks +
                        (((now.time_us - edge.time_us) * 6U) / 20000U);
                    CHECK(!b3_barcode_poll(&scan, now, &result));
                }
                CHECK(!b3_barcode_active(&scan));
            }
            edge.time_us = white_end;
            edge.position_ticks += gap_mm * 6U;
            edge.black = true;
            CHECK(!b3_barcode_edge(&scan, edge, &result));
            CHECK(1U == scan.rejected);
            CHECK(B3_REJECT_TOO_LONG == scan.reject_reason);
            CHECK(b3_barcode_active(&scan));
            for (i = 0U; i < B3_BARCODE_RUNS; ++i)
            {
                edge.time_us += times[i];
                edge.position_ticks += ticks[i];
                edge.black = !edge.black;
                CHECK(!b3_barcode_edge(&scan, edge, &result));
            }
            CHECK(scan.pending);
            edge.time_us += 1000000U;
            edge.position_ticks += 300U;
            CHECK(b3_barcode_poll(&scan, edge, &result));
            CHECK('A' == result.symbol);
            CHECK((1U == (mode & 1U)) == result.used_encoder);
            CHECK(1U == scan.rejected);
        }
    }
    (void)puts("PASS: a black mark 10-13 cm before a barcode at 50 mm/s does not shorten its leading white");
}

/* DO comparator chatter: a 41 us blip inside a bar and another in the
 * trailing white are absorbed, and the barcode still decodes. */
static void test_edge_blips(void)
{
    uint32_t times[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge = {0};
    size_t i;
    make_frame('A', 10000U, 30000U, times);
    edge.time_us = 300000U;
    b3_barcode_init(&scan, edge);
    edge.time_us = 500000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        if ((5U == i) || (12U == i))
        {
            uint32_t at = edge.time_us + (times[i] / 2U);
            b3_edge_t blip = edge;
            blip.time_us = at;
            blip.black = !edge.black;
            CHECK(!b3_barcode_edge(&scan, blip, &result));
            blip.time_us = at + 41U;
            blip.black = edge.black;
            CHECK(!b3_barcode_edge(&scan, blip, &result));
        }
        edge.time_us += times[i];
        edge.black = !edge.black;
        CHECK(!b3_barcode_edge(&scan, edge, &result));
    }
    CHECK(scan.pending);
    {
        /* nyx, 2026-11-09: both '*' read, no trailing white is needed
         * any more, so a blip in it gives the letter at its black edge */
        b3_edge_t blip = edge;
        blip.time_us += 20000U;
        blip.black = true;
        CHECK(b3_barcode_edge(&scan, blip, &result));
        CHECK('A' == result.symbol);
        blip.time_us += 41U;
        blip.black = false;
        CHECK(!b3_barcode_edge(&scan, blip, &result));
    }
    CHECK(0U == scan.rejected);
    CHECK(3U == scan.glitches); /* the trailing blip is still absorbed */
    (void)puts("PASS: sub-200 us sensor blips inside a barcode are absorbed");
}

/* Car capture 2026-11-04 (nyx): every element fine except the first and
 * last bars, read 3-4x too long. Trimming them decodes the middle letter. */
/* nyx, 2026-11-09: the car's reason-6 reject (widths in us as the VC
 * printed them): the time widths decode as 'Z' read back to front with one
 * '*' spoiled, but the scan was decoded from wheel-encoder distances that
 * were off. A failed encoder decode now falls back to the time widths. */
static void test_encoder_fallback_to_time(void)
{
    static const uint32_t car[B3_BARCODE_RUNS] = {
        84500U, 153700U, 361500U, 122700U, 340400U, 132300U, 104200U,
        341000U, 198700U, 44000U, 143600U, 107200U, 121400U, 122500U,
        345600U, 133900U, 328200U, 372000U, 108200U, 130400U, 110200U,
        133900U, 331300U, 120900U, 395300U, 44500U, 137300U, 377200U,
        92100U };
    uint32_t ticks[B3_BARCODE_RUNS];
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_code39_result_t raw_result;
    b3_edge_t edge;
    size_t i;
    CHECK(b3_code39_decode(car, &raw_result));
    CHECK('Z' == raw_result.symbol);
    CHECK(raw_result.reversed);
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        /* 30 mm/s, 0.16 mm per tick: ~190 ticks per second; every bar
         * reads 3x too long in distance (encoders not matching the IR) */
        ticks[i] = (car[i] * 190U) / 1000000U;
        if (0U == i % 2U)
        {
            ticks[i] *= 3U;
        }
    }
    CHECK(!b3_code39_decode(ticks, &raw_result));
    edge = feed_frame(&scan, 3000000U, car, ticks, 3000000U);
    edge.time_us += 3000000U;
    edge.position_ticks += 600U;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK('Z' == result.symbol);
    CHECK(!result.used_encoder);
    CHECK(1U == scan.timed_fallbacks);
    (void)puts("PASS: car scan whose encoder widths fail decodes from its "
               "time widths ('Z', reason 6 on 2026-11-09)");
}

static void test_trim_outer_bars(void)
{
    static const uint32_t car[B3_BARCODE_RUNS] = {
        278600U, 177800U, 125400U, 80700U, 315300U, 94100U, 318900U, 91000U, 64600U, 145100U,
        57000U, 335800U, 332900U, 48200U, 361400U, 80500U, 121600U, 97100U, 83100U, 132600U,
        52500U, 350600U, 59000U, 110300U, 312300U, 90000U, 334500U, 56500U, 236800U};
    b3_code39_result_t direct;
    b3_barcode_t scan;
    b3_barcode_result_t result;
    b3_edge_t edge;
    static const uint32_t car2[B3_BARCODE_RUNS] = { /* second capture, 33 ms thin bar */
        238000U, 227100U, 85300U, 116000U, 306000U, 48500U, 319600U, 95300U, 59500U, 161300U,
        33000U, 351400U, 312700U, 76200U, 342800U, 73000U, 122900U, 91500U, 110700U, 96100U,
        117600U, 287000U, 125600U, 69100U, 332400U, 84600U, 332800U, 65100U, 146600U};
    CHECK(!b3_code39_decode(car, &direct));
#if B3_TRIM_OUTER_BARS
    edge = feed_frame(&scan, 500000U, car2, NULL, 3000000U);
    CHECK(scan.pending);
    CHECK('Z' == scan.decoded.symbol);
#endif
    edge = feed_frame(&scan, 500000U, car, NULL, 3000000U);
#if B3_TRIM_OUTER_BARS
    CHECK(scan.pending);
    CHECK(1U == scan.trimmed);
    edge.time_us += 3000000U;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    (void)printf("PASS: car scan with stretched outer bars decodes as '%c' after trimming\n",
                 result.symbol);
#else
    (void)edge;
    (void)result;
    CHECK(1U == scan.rejected);
#endif
}

static void test_line_logic(void)
{
    b3_line_t line;
    b3_line_result_t result;
    /* Straddle layout: both white = on track, drive straight from the start. */
    b3_line_init(&line, 0U);
    result = b3_line_step(&line, false, false, 0U, 120);
    CHECK(!result.stop && (B3_LINE_CENTRED == result.state));
    CHECK((120 == result.left_mm_s) && (120 == result.right_mm_s));
    /* A 0.4 ms black flicker is filtered out. */
    (void)b3_line_step(&line, true, false, 4100U, 120);
    result = b3_line_step(&line, false, false, 4500U, 120);
    CHECK(B3_LINE_CENTRED == result.state);
    (void)b3_line_step(&line, true, false, 5000U, 120);
    result = b3_line_step(&line, true, false, 8000U, 120);
    CHECK(B3_LINE_LEFT == result.state);
    /* nyx, 2026-11-07: the steering slews, so give the nudge 20 ms */
    result = b3_line_step(&line, true, false, 28000U, 120);
    CHECK(result.left_mm_s < result.right_mm_s); /* IR_L black: turn left */
    (void)b3_line_step(&line, false, true, 29000U, 120);
    result = b3_line_step(&line, false, true, 32000U, 120);
    CHECK(B3_LINE_RIGHT == result.state);
    result = b3_line_step(&line, false, true, 72000U, 120);
    CHECK(result.right_mm_s < result.left_mm_s); /* IR_R black: turn right */
    {
        const int32_t turning = result.curvature;

        (void)b3_line_step(&line, true, true, 73000U, 120);
        result = b3_line_step(&line, true, true, 86000U, 120);
        CHECK(B3_LINE_CROSSING == result.state && !result.stop);
        /* nyx, 2026-11-07: a crossing eases back towards the learnt
         * curve, which is small after these few ms */
        CHECK((turning < 0) && (result.curvature > turning));
    }
    result = b3_line_step(&line, true, true, 176000U, 120);
    /* nyx, 2026-11-07 (patch 17): straight again; the speed may still be
     * a little under 120 while the learnt curve fades (slows from R670) */
    CHECK((abs(result.left_mm_s - result.right_mm_s) <= 6) &&
          (result.left_mm_s >= 100) && (result.left_mm_s <= 120));
    (void)b3_line_step(&line, false, false, 177000U, 120);
    result = b3_line_step(&line, false, false, 230000U, 120);
    CHECK(B3_LINE_CENTRED == result.state && !result.stop);
    b3_line_init(&line, UINT32_MAX - 1000U);
    (void)b3_line_step(&line, true, false, UINT32_MAX - 1000U, 20);
    result = b3_line_step(&line, true, false, 2000U, 20);
    CHECK(B3_LINE_LEFT == result.state);
    CHECK((result.left_mm_s >= 0) && (result.right_mm_s >= 0));
    (void)puts("PASS: straddle line rules, steering direction, filtering and crossings");

    /* Smooth steering (nyx, 2026-11-07, B3_STEER_*): a side going black
     * nudges towards it at once; held black, the curve tightens and the
     * car slows, but the inner wheel keeps B3_MIN_WHEEL_MM_S; white again,
     * the learnt curve carries on and fades; a hairpin stops the inner
     * wheel; b3_line_hold() drives straight; the same curve at any speed. */
    {
        uint32_t t;
        int first_diff;
        int held_diff;
        int white_diff;
        int32_t curv_slow;

        b3_line_init(&line, 0U);
        (void)b3_line_step(&line, false, true, 0U, 120);
        result = b3_line_step(&line, false, true, 10000U, 120);
        first_diff = result.left_mm_s - result.right_mm_s;
        CHECK((B3_LINE_RIGHT == result.state) && (first_diff > 0) &&
              (result.right_mm_s >= B3_MIN_WHEEL_MM_S) && (result.curvature < 0));
        /* held black: steering keeps tightening, never jumps, until the
         * hairpin rule stops the inner wheel */
        held_diff = first_diff;
        {
            int32_t curv = result.curvature;

            for (t = 20000U; t <= 5000000U; t += 10000U)
            {
                const int before = held_diff;

                result = b3_line_step(&line, false, true, t, 120);
                if (0 == result.right_mm_s)
                {
                    break; /* hairpin reached */
                }
                held_diff = result.left_mm_s - result.right_mm_s;
                /* the curve only tightens (the wheel difference may ease
                 * a little as the car slows for it) */
                CHECK(result.curvature <= curv);
                curv = result.curvature;
                CHECK(abs(held_diff - before) <= 12); /* <= 12 mm/s per 10 ms */
                CHECK(result.right_mm_s >= B3_MIN_WHEEL_MM_S);
                CHECK((result.left_mm_s + result.right_mm_s) / 2 >= B3_CURVE_SPEED_MM_S);
            }
        }
        CHECK(held_diff > first_diff);
        CHECK(result.sharp_turn); /* slowed for the curve */
        /* hairpin: the inner wheel stops (a pivot on it) */
        CHECK(0 == result.right_mm_s);
        /* white again: the learnt curve carries on, gentler */
        result = b3_line_step(&line, false, false, t + 10000U, 120);
        result = b3_line_step(&line, false, false, t + 20000U, 120);
        white_diff = result.left_mm_s - result.right_mm_s;
        CHECK((white_diff > 0) && (result.right_mm_s >= B3_MIN_WHEEL_MM_S));
        /* ... and fades to straight */
        for (t = t + 30000U; t <= 30000000U; t += 10000U)
        {
            result = b3_line_step(&line, false, false, t, 120);
            if (0 == result.curvature)
            {
                break;
            }
        }
        CHECK((120 == result.left_mm_s) && (120 == result.right_mm_s) && (0 == result.curvature));
        /* someone else drove: straight at once */
        b3_line_init(&line, 0U);
        for (t = 0U; t <= 300000U; t += 10000U)
        {
            result = b3_line_step(&line, true, false, t, 120);
        }
        CHECK(result.left_mm_s < result.right_mm_s);
        b3_line_hold(&line);
        /* 10 ms later (the sensor filter still reads IR_L black): one
         * slew step from straight, not the curve learnt before */
        result = b3_line_step(&line, false, false, t, 120);
        CHECK(abs((int)result.curvature) <= (2 * B3_STEER_SLEW_CURV));
        CHECK((abs(result.left_mm_s - 120) <= 5) && (abs(result.right_mm_s - 120) <= 5));
        /* the same curvature after the same 10 mm at 100 and 30 mm/s */
        b3_line_init(&line, 0U);
        for (t = 0U; t <= 100000U; t += 10000U)
        {
            result = b3_line_step(&line, true, false, t, 100);
        }
        curv_slow = result.curvature;
        b3_line_init(&line, 0U);
        for (t = 0U; t <= 330000U; t += 10000U)
        {
            result = b3_line_step(&line, true, false, t, 30);
        }
        CHECK((result.curvature > 0) && (abs((int)(result.curvature - curv_slow)) <= (int)(curv_slow / 10)));
        CHECK((result.left_mm_s < result.right_mm_s) && (result.left_mm_s >= B3_MIN_WHEEL_MM_S));
    }
    (void)puts("PASS: smooth steering nudges, learns the curve, slows, keeps the inner wheel turning, pivots in a hairpin, fades, holds");

}

/* nyx, 2026-11-09: both IRs black inside a learnt bend (the tape crosses
 * the pair at a steep angle) keep the curve, and longer than a crossing
 * they nudge to the inside; on a straight both black is still a crossing
 * (B3_STEER_BB_* in b3_config.h). */
static void test_bend_both_black(void)
{
    b3_line_t line;
    b3_line_result_t result = { 0 };
    uint32_t t;
    int32_t learnt;
    int32_t curv_in = 0;
    bool nudged = false;

    /* learn a left curve tighter than B3_STEER_BB_CURV: IR_L black */
    b3_line_init(&line, 0U);
    for (t = 0U; t <= 5000000U; t += 10000U)
    {
        result = b3_line_step(&line, true, false, t, 100);
        if (line.memory >= B3_STEER_BB_CURV)
        {
            break;
        }
    }
    CHECK(line.memory >= B3_STEER_BB_CURV);
    learnt = 0;
    /* both black: a crossing to the VehicleController, but the car keeps
     * turning left; the learnt curve neither fades nor grows (the nudge
     * for the black IR_L eases off) */
    for (t += 10000U; t <= 10000000U; t += 10000U)
    {
        result = b3_line_step(&line, true, true, t, 100);
        if (0U == line.bb_um)
        {
            continue; /* the sensor filter */
        }
        if (0 == learnt)
        {
            learnt = line.memory; /* after the filter's last IR_L step */
        }
        CHECK(B3_LINE_CROSSING == result.state && !result.stop);
        CHECK(learnt == line.memory);
        CHECK((result.curvature >= learnt) &&
              (result.left_mm_s < result.right_mm_s));
        CHECK(result.left_mm_s >= B3_MIN_WHEEL_MM_S);
        if (line.bb_um < (B3_STEER_BB_CROSS_MM * 1000U))
        {
            curv_in = result.curvature;
        }
        else if (result.curvature >= (learnt + (line.kick / 2)))
        {
            /* longer than any crossing: nudged to the inside */
            nudged = (curv_in < result.curvature);
            break;
        }
        else
        {
            /* the nudge slews in */
        }
    }
    CHECK(nudged);

    /* on a straight (nothing learnt) both black drives straight on */
    b3_line_init(&line, 0U);
    for (t = 0U; t <= 200000U; t += 10000U)
    {
        result = b3_line_step(&line, false, false, t, 100);
    }
    for (; t <= 600000U; t += 10000U)
    {
        result = b3_line_step(&line, true, true, t, 100);
    }
    CHECK((B3_LINE_CROSSING == result.state) && (0 == result.curvature));
    CHECK(result.left_mm_s == result.right_mm_s);
    (void)puts("PASS: both IRs black in a learnt bend keep the curve and nudge inward; on a straight they drive on");
}

int main(void)
{
    test_pdf_fixtures();
    test_alphabet_and_speed();
    test_invalid_frames();
    test_stream_and_quiet();
    test_encoder_widths();
    test_timeout_noise_wrap();
    test_any_size();
    test_slowdown_compensation();
    test_edge_blips();
    test_leading_blip();
    test_mark_before_barcode();
    test_trim_outer_bars();
    test_encoder_fallback_to_time();
    test_line_logic();
    test_bend_both_black();
    (void)printf("All %u checks passed. Host logic tests; no hardware timing claim.\n", g_checks);
    return EXIT_SUCCESS;
}
