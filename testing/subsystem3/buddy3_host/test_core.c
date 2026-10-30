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
    make_frame('A', 1000U, 3800U, widths);
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
    edge.time_us += 99999U;
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    ++edge.time_us;
    CHECK(b3_barcode_poll(&scan, edge, &result));
    CHECK('A' == result.symbol);
    CHECK(!result.used_encoder);
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    edge = feed_frame(&scan, 500000U, times, NULL, 50000U);
    edge.time_us += 200000U;
    CHECK(!b3_barcode_poll(&scan, edge, &result));
    CHECK(1U == scan.rejected);
    edge = feed_frame(&scan, 500000U, times, NULL, 200000U);
    edge.time_us += 90000U;
    edge.black = true;
    CHECK(!b3_barcode_edge(&scan, edge, &result));
    CHECK(1U == scan.rejected);
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
    CHECK(1U == scan.rejected);
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
    CHECK(result.left_mm_s < result.right_mm_s); /* IR_L black: turn left */
    (void)b3_line_step(&line, false, true, 9000U, 120);
    result = b3_line_step(&line, false, true, 12000U, 120);
    CHECK(B3_LINE_RIGHT == result.state);
    CHECK(result.right_mm_s < result.left_mm_s); /* IR_R black: turn right */
    (void)b3_line_step(&line, true, true, 13000U, 120);
    result = b3_line_step(&line, true, true, 16000U, 120);
    CHECK(B3_LINE_CROSSING == result.state && !result.stop);
    CHECK((120 == result.left_mm_s) && (120 == result.right_mm_s));
    (void)b3_line_step(&line, false, false, 17000U, 120);
    result = b3_line_step(&line, false, false, 70000U, 120);
    CHECK(B3_LINE_CENTRED == result.state && !result.stop);
    b3_line_init(&line, UINT32_MAX - 1000U);
    (void)b3_line_step(&line, true, false, UINT32_MAX - 1000U, 20);
    result = b3_line_step(&line, true, false, 2000U, 20);
    CHECK(B3_LINE_LEFT == result.state);
    CHECK((result.left_mm_s >= 0) && (result.right_mm_s >= 0));
    (void)puts("PASS: straddle line rules, steering direction, filtering and crossings");

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
    test_line_logic();
    (void)printf("All %u checks passed. Host logic tests; no hardware timing claim.\n", g_checks);
    return EXIT_SUCCESS;
}
