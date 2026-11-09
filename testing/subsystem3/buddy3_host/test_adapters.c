/* Deterministic simulation of the micro T-Kernel/BSP boundary. No hardware. */
#include "buddy3.h"
#include "b3_port.h"
#include "b3_code39.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t g_now = 1000U;
static uint32_t g_pins = UINT32_MAX;
/* nyx, 2026-11-06: encoder position for Buddy 3, off unless a test turns
 * it on: ticks = g_pos_base + rate x (g_now - g_pos_t0) */
static bool g_pos_on;
static uint32_t g_pos_base;
static uint32_t g_pos_t0;
static int32_t g_pos_rate; /* ticks per second, - = backwards */

static uint32_t pos_now(void)
{
    return g_pos_base +
           (uint32_t)(((int64_t)g_pos_rate * (int64_t)(g_now - g_pos_t0)) /
                      1000000);
}

static void pos_rate(int32_t ticks_per_s)
{
    g_pos_base = pos_now();
    g_pos_t0 = g_now;
    g_pos_rate = ticks_per_s;
}

static bool mock_position(uint32_t *p_ticks)
{
    *p_ticks = pos_now();
    return g_pos_on;
}
static uint32_t g_pending;
static uint32_t g_initialised;
static bool g_debug_high; /* any GPIO write: Buddy 3 must make none */
static uint32_t g_outputs;
static bool g_irq_enabled;
static bool g_irq_installed;
static unsigned int g_irq_calls;
static unsigned int g_checks;
static FP g_task;
static jmp_buf g_task_wait;

#define CHECK(expression) do { \
    ++g_checks; \
    if (!(expression)) { \
        (void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

ER b3_port_init(void) { return E_OK; }
void b3_port_lock(b3_irq_state_t *p_state) { *p_state = 0U; }
void b3_port_unlock(b3_irq_state_t state) { (void)state; }
ER b3_port_gpio_input_init(unsigned int pin, bool pull_up)
{
    (void)pull_up;
    g_initialised |= (1U << pin);
    return E_OK;
}
ER b3_port_gpio_output_init(unsigned int pin, bool initial_high)
{
    g_outputs |= (1U << pin);
    g_debug_high = g_debug_high || initial_high;
    return E_OK;
}
uint32_t b3_port_gpio_get_all(void) { return g_pins; }
void b3_port_gpio_put(unsigned int pin, bool value)
{
    (void)pin;
    (void)value;
    g_debug_high = true;
}
uint32_t b3_port_time_us(void) { return g_now; }
void b3_port_barcode_irq_enable(bool enabled) { g_irq_enabled = enabled; }
uint32_t b3_port_barcode_irq_pending(void) { return g_pending; }
void b3_port_barcode_irq_ack(uint32_t events) { g_pending &= ~events; }
ER b3_port_install_barcode_irq(void)
{
    g_irq_installed = true;
    return E_OK;
}
void b3_port_dispatch_barcode_irq(void)
{
    uint32_t events = g_pending & B3_GPIO_EDGE_BOTH;
    if (events != 0U)
    {
        b3_port_barcode_irq_ack(events);
        b3_ir_gpio_irq(B3_IR_B_GPIO, events);
    }
}

ID tk_cre_tsk(const T_CTSK *p_create)
{
    CHECK((p_create->tskatr & TA_RNG3) == TA_RNG3);
#if TK_SUPPORT_SMP
    CHECK((p_create->tskatr & TA_ASSPRC) != 0U);
    CHECK(TP_PRC1 == p_create->assprc);
#endif
    CHECK(NULL == p_create->bufptr);
    CHECK(B3_TASK_STACK_BYTES == (unsigned int)p_create->stksz);
    g_task = p_create->task;
    return 1;
}
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
ER tk_del_tsk(ID id) { (void)id; return E_OK; }
ER tk_dly_tsk(uint32_t milliseconds)
{
    CHECK(B3_SERVICE_DELAY_MS == milliseconds);
    longjmp(g_task_wait, 1); /* Simulate suspension after one service pass. */
}
ER tk_ter_tsk(ID id) { (void)id; return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
ID tk_cre_cyc(const T_CCYC *p_create)
{
    CHECK(B3_SERVICE_DELAY_MS == p_create->cyctim);
    CHECK((p_create->cycatr & TA_STA) != 0U);
    return 2;
}
ER tk_slp_tsk(int32_t timeout)
{
    CHECK(TMO_FEVR == timeout);
    longjmp(g_task_wait, 1); /* Simulate sleeping until the cyclic wake-up. */
}
void tk_ext_tsk(void) { longjmp(g_task_wait, 2); }

static void one_task_pass(void)
{
    if (0 == setjmp(g_task_wait))
    {
        g_task(0, NULL);
    }
}

static void emit(bool black)
{
    g_now += 1000U;
    bool high = (black == (B3_IR_B_BLACK_LEVEL != 0U));
    if (high)
    {
        g_pins |= (1U << B3_IR_B_GPIO);
    }
    else
    {
        g_pins &= ~(1U << B3_IR_B_GPIO);
    }
    ++g_irq_calls;
    b3_ir_gpio_irq(B3_IR_B_GPIO,
                   high ? B3_GPIO_EDGE_RISE : B3_GPIO_EDGE_FALL);
    CHECK(!g_debug_high);
}

static void test_driver(void)
{
    unsigned int i;
    b3_edge_t edge;
    b3_ir_sample_t sample;
    CHECK(E_OK == b3_ir_init(NULL));
    CHECK(g_irq_installed == (B3_INSTALL_IO_BANK0_HANDLER != 0U));
    CHECK((g_initialised & (1U << B3_IR_L_GPIO)) != 0U);
    CHECK((g_initialised & (1U << B3_IR_B_GPIO)) != 0U);
    CHECK((g_initialised & (1U << B3_IR_R_GPIO)) != 0U);
    CHECK(0U == g_outputs); /* no diagnostic output pins */
    b3_ir_gpio_irq(0U, B3_GPIO_EDGE_RISE);
    CHECK(B3_IR_EMPTY == b3_ir_pop(&edge));
    emit(true);
    CHECK(!b3_ir_read_when_idle(&sample));
    CHECK(B3_IR_EDGE == b3_ir_pop(&edge));
    CHECK(edge.black && (g_now == edge.time_us));
    CHECK(b3_ir_read_when_idle(&sample));
    g_pending = B3_GPIO_EDGE_FALL;
    CHECK(!b3_ir_read_when_idle(&sample));
    g_pending = 0U;
    /* coalesced rise+fall, pin still black: a glitch, dropped */
    b3_ir_gpio_irq(B3_IR_B_GPIO, B3_GPIO_EDGE_RISE | B3_GPIO_EDGE_FALL);
    CHECK(B3_IR_EMPTY == b3_ir_pop(&edge));
    CHECK(0U == b3_ir_fault_count());
    CHECK(1U == b3_ir_glitch_count());
    /* coalesced rise+fall, pin now white: chatter at a real transition,
       queued as one white edge */
    g_now += 1000U;
    g_pins = (B3_IR_B_BLACK_LEVEL != 0U) ? (g_pins & ~(1U << B3_IR_B_GPIO))
                                         : (g_pins | (1U << B3_IR_B_GPIO));
    b3_ir_gpio_irq(B3_IR_B_GPIO, B3_GPIO_EDGE_RISE | B3_GPIO_EDGE_FALL);
    CHECK(B3_IR_EDGE == b3_ir_pop(&edge));
    CHECK(!edge.black && (g_now == edge.time_us));
    CHECK(0U == b3_ir_fault_count());
    CHECK(E_OK == b3_ir_init(NULL));
    for (i = 0U; i <= B3_EDGE_QUEUE_SIZE; ++i)
    {
        emit(0U == i % 2U);
    }
    CHECK(b3_ir_fault_count() > 0U);
    CHECK(B3_IR_DISCONTINUITY == b3_ir_pop(&edge));
    CHECK(B3_IR_EMPTY == b3_ir_pop(&edge));
    (void)puts("PASS: ISR filtering, queue overflow, coalesced edges and quiet-zone race guard");
}


static void test_task_boundary(void)
{
    b3_context_t context = {0};
    b3_snapshot_t snapshot;
    g_pins = UINT32_MAX;
    CHECK(E_OK == buddy3_start(mock_position));
    CHECK(g_irq_enabled);
    CHECK(E_OBJ == buddy3_start(NULL));
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(snapshot.stop_required);
    context.time_us = g_now;
    context.follow_enabled = true;
    context.scan_enabled = true;
    context.motion_valid = true;
    context.left_measured_mm_s = 120;
    context.right_measured_mm_s = 120;
    buddy3_set_context(&context);
    /* Both line sensors over black, in the configured polarity. */
    g_pins = (B3_IR_L_BLACK_LEVEL != 0U) ? (g_pins | (1U << B3_IR_L_GPIO))
                                         : (g_pins & ~(1U << B3_IR_L_GPIO));
    g_pins = (B3_IR_R_BLACK_LEVEL != 0U) ? (g_pins | (1U << B3_IR_R_GPIO))
                                         : (g_pins & ~(1U << B3_IR_R_GPIO));
    one_task_pass();
    g_now += 5000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(snapshot.control_active);
    CHECK(!snapshot.stop_required);
    CHECK((B3_BASE_SPEED_MM_S == snapshot.line.left_mm_s) && (B3_BASE_SPEED_MM_S == snapshot.line.right_mm_s));

    /* A broad transverse patch is a candidate; a short barcode bar is not. */
    emit(true);
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    g_now += 40000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(!snapshot.junction_candidate);
    g_now += 40000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    g_now += 40000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(snapshot.junction_candidate);
    CHECK(1U == snapshot.junction_sequence);
    emit(false);
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
#if B3_JUNCTION_NEEDS_IR_B
    CHECK(!snapshot.junction_candidate);
#else
    /* Straddle layout: IR_L and IR_R alone decide; IR_B leaving the tape
       first (it is not level with them) does not end the junction. */
    CHECK(snapshot.junction_candidate);
    CHECK(1U == snapshot.junction_sequence);
    g_pins = (B3_IR_L_BLACK_LEVEL != 0U) ? (g_pins & ~(1U << B3_IR_L_GPIO))
                                         : (g_pins | (1U << B3_IR_L_GPIO));
    for (unsigned int pass = 0U; pass < 5U; ++pass) /* line sensors are debounced */
    {
        g_now += 5000U;
        context.time_us = g_now;
        buddy3_set_context(&context);
        one_task_pass();
    }
    buddy3_get_snapshot(&snapshot);
    /* nyx, 2026-11-08: still the same junction while IR_R is on the tape
       (each sensor's own run counts now); over once both are white. */
    CHECK(snapshot.junction_candidate);
    CHECK(1U == snapshot.junction_sequence);
    g_pins = (B3_IR_R_BLACK_LEVEL != 0U) ? (g_pins & ~(1U << B3_IR_R_GPIO))
                                         : (g_pins | (1U << B3_IR_R_GPIO));
    for (unsigned int pass = 0U; pass < 5U; ++pass)
    {
        g_now += 5000U;
        context.time_us = g_now;
        buddy3_set_context(&context);
        one_task_pass();
    }
    buddy3_get_snapshot(&snapshot);
    CHECK(!snapshot.junction_candidate);
    CHECK(1U == snapshot.junction_sequence);
    g_pins = (B3_IR_L_BLACK_LEVEL != 0U) ? (g_pins | (1U << B3_IR_L_GPIO))
                                         : (g_pins & ~(1U << B3_IR_L_GPIO));
    g_pins = (B3_IR_R_BLACK_LEVEL != 0U) ? (g_pins | (1U << B3_IR_R_GPIO))
                                         : (g_pins & ~(1U << B3_IR_R_GPIO));
#endif

    /* Barcode slowdown: a thin IR_B bar slows the car until IR_B has read
     * white for B3_SCAN_SLOW_WHITE_US, then base speed comes back. */
    g_pins = (B3_IR_L_BLACK_LEVEL != 0U) ? (g_pins & ~(1U << B3_IR_L_GPIO))
                                         : (g_pins | (1U << B3_IR_L_GPIO));
    g_pins = (B3_IR_R_BLACK_LEVEL != 0U) ? (g_pins & ~(1U << B3_IR_R_GPIO))
                                         : (g_pins | (1U << B3_IR_R_GPIO));
    emit(false);
    for (unsigned int pass = 0U; pass < 200U; ++pass)
    {
        g_now += 5000U;
        context.time_us = g_now;
        buddy3_set_context(&context);
        one_task_pass();
    }
    buddy3_get_snapshot(&snapshot);
    CHECK((B3_BASE_SPEED_MM_S == snapshot.line.left_mm_s) && (B3_BASE_SPEED_MM_S == snapshot.line.right_mm_s));
    emit(true);
    emit(false); /* thin bar, both edges queued before the next pass */
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass(); /* edges are read after this pass's speed is set */
    g_now += 5000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK((B3_SCAN_SPEED_MM_S == snapshot.line.left_mm_s) &&
          (B3_SCAN_SPEED_MM_S == snapshot.line.right_mm_s));
    g_now += B3_SCAN_SLOW_WHITE_US - 10000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(B3_SCAN_SPEED_MM_S == snapshot.line.left_mm_s);
    g_now += 20000U;
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK((B3_BASE_SPEED_MM_S == snapshot.line.left_mm_s) && (B3_BASE_SPEED_MM_S == snapshot.line.right_mm_s));

    context.stop_requested = true;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(snapshot.stop_required);
    CHECK((0 == snapshot.line.left_mm_s) && (0 == snapshot.line.right_mm_s));
    context.stop_requested = false;
    buddy3_set_context(&context);
    g_now += B3_FEEDBACK_MAX_AGE_US + 1U;
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(snapshot.stop_required && !snapshot.control_active);
#if B3_ENABLE_EXAMPLE_NAV_MAP
    CHECK(B3_NAV_LEFT == buddy3_map_symbol('A'));
    CHECK(B3_NAV_RIGHT == buddy3_map_symbol('B'));
    CHECK(B3_NAV_STRAIGHT == buddy3_map_symbol('C'));
    CHECK(B3_NAV_UTURN == buddy3_map_symbol('D'));
    CHECK(B3_NAV_NONE == buddy3_map_symbol('Z'));
#else
    CHECK(B3_NAV_NONE == buddy3_map_symbol('A'));
#endif
    (void)puts("PASS: micro T-Kernel adapter startup, bounded task pass and stale-context stop");
}

/* nyx, 2026-11-06: helpers for the held-letter / junction / fault checks. */
static void set_line_black(bool left, bool right)
{
    g_pins = ((B3_IR_L_BLACK_LEVEL != 0U) == left) ? (g_pins | (1U << B3_IR_L_GPIO))
                                                   : (g_pins & ~(1U << B3_IR_L_GPIO));
    g_pins = ((B3_IR_R_BLACK_LEVEL != 0U) == right) ? (g_pins | (1U << B3_IR_R_GPIO))
                                                    : (g_pins & ~(1U << B3_IR_R_GPIO));
}

static void run_for(b3_context_t *p_context, uint32_t duration_us)
{
    uint32_t end = g_now + duration_us;
    while ((int32_t)(end - g_now) > 0)
    {
        g_now += 5000U;
        p_context->time_us = g_now;
        buddy3_set_context(p_context);
        one_task_pass();
    }
}

/* Drive one '*' symbol '*' barcode under IR_B: narrow 20 ms, wide 60 ms,
 * after lead_us of white. */
static void drive_barcode_lead(b3_context_t *p_context, char symbol,
                               uint32_t lead_us)
{
    const char frame[3] = {'*', symbol, '*'};
    uint32_t widths[B3_BARCODE_RUNS];
    size_t out = 0U;
    for (size_t c = 0U; c < 3U; ++c)
    {
        uint16_t pattern = 0U;
        CHECK(b3_code39_pattern(frame[c], &pattern));
        for (size_t bit = 0U; bit < 9U; ++bit)
        {
            widths[out++] = ((pattern & (1U << (8U - bit))) != 0U) ? 60000U : 20000U;
        }
        if (c < 2U)
        {
            widths[out++] = 20000U;
        }
    }
    CHECK(B3_BARCODE_RUNS == out);
    run_for(p_context, lead_us);       /* leading quiet zone */
    emit(true);                        /* first bar starts */
    for (size_t i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        g_now += widths[i] - 1000U;    /* emit() adds 1 ms */
        emit(0U != (i & 1U));          /* bar ends -> white, gap ends -> black */
        p_context->time_us = g_now;
        buddy3_set_context(p_context);
        one_task_pass();
    }
    run_for(p_context, 300000U);       /* trailing quiet zone */
}

static void drive_barcode(b3_context_t *p_context, char symbol)
{
    drive_barcode_lead(p_context, symbol, 600000U);
}

static void test_held_symbol_and_faults(void)
{
    b3_context_t context = {0};
    b3_snapshot_t snapshot;
    uint32_t junctions;

    context.follow_enabled = true;
    context.scan_enabled = true;
    context.motion_valid = true;
    context.left_measured_mm_s = 120;
    context.right_measured_mm_s = 120;
    set_line_black(false, false);
    emit(false);
    run_for(&context, 1000000U);
    buddy3_get_snapshot(&snapshot);
    junctions = snapshot.junction_sequence;

    /* A decoded letter is held, not acted on. */
    drive_barcode(&context, 'A');
    buddy3_get_snapshot(&snapshot);
    CHECK('A' == snapshot.held_symbol);
    CHECK(junctions == snapshot.junction_sequence);

    /* Junction at 25 mm/s with a dead right encoder: still detected, and
       the held letter moves to that junction. */
    context.left_measured_mm_s = 25;
    context.right_measured_mm_s = 0;
    set_line_black(true, true);
    run_for(&context, 800000U);         /* 20 mm of black at 25 mm/s */
    buddy3_get_snapshot(&snapshot);
    CHECK((junctions + 1U) == snapshot.junction_sequence);
    CHECK('A' == snapshot.junction_symbol);
    CHECK('\0' == snapshot.held_symbol);
    set_line_black(false, false);
    context.left_measured_mm_s = 120;
    context.right_measured_mm_s = 120;
    run_for(&context, 100000U);

    /* nyx, 2026-11-06: a speed dip in the middle of the tape no longer
       hides the junction (distance driven, not slowest speed x time). */
    junctions = snapshot.junction_sequence;
    context.left_measured_mm_s = 25;
    context.right_measured_mm_s = 25;
    set_line_black(true, true);
    run_for(&context, 200000U);
    context.left_measured_mm_s = 5;     /* one slow speed window */
    context.right_measured_mm_s = 5;
    run_for(&context, 20000U);
    context.left_measured_mm_s = 25;
    context.right_measured_mm_s = 25;
    run_for(&context, 400000U);         /* ~15 mm in all */
    buddy3_get_snapshot(&snapshot);
    CHECK((junctions + 1U) == snapshot.junction_sequence);
    set_line_black(false, false);
    run_for(&context, 100000U);

    /* A short black patch (6 mm) is not a junction, and says why. */
    {
        const uint32_t misses = snapshot.patch_misses;

        junctions = snapshot.junction_sequence;
        set_line_black(true, true);
        run_for(&context, 240000U);     /* 6 mm at 25 mm/s */
        set_line_black(false, false);
        run_for(&context, 100000U);
        buddy3_get_snapshot(&snapshot);
        CHECK(junctions == snapshot.junction_sequence);
        CHECK((misses + 1U) == snapshot.patch_misses);
        CHECK(B3_PATCH_SHORT == (b3_patch_why_t)snapshot.patch_miss_why);
        CHECK((snapshot.patch_miss_mm >= 4U) && (snapshot.patch_miss_mm <= 8U));
        CHECK((snapshot.patch_miss_l_mm >= 4U) && (snapshot.patch_miss_l_mm <= 8U));
        CHECK((snapshot.patch_miss_r_mm >= 4U) && (snapshot.patch_miss_r_mm <= 8U));
    }

    /* nyx, 2026-11-08: the tape met at an angle. IR_L is on it 15 mm,
       IR_R enters 12 mm in, so both are black together for only 3 mm
       (the car read 5): each sensor's own run counts, once. */
    {
        const uint32_t misses = snapshot.patch_misses;

        junctions = snapshot.junction_sequence;
        set_line_black(true, false);
        run_for(&context, 480000U);     /* 12 mm at 25 mm/s */
        set_line_black(true, true);
        run_for(&context, 120000U);     /* 3 mm together */
        set_line_black(false, true);
        run_for(&context, 480000U);     /* 12 mm IR_R alone */
        set_line_black(false, false);
        run_for(&context, 100000U);
        buddy3_get_snapshot(&snapshot);
        CHECK((junctions + 1U) == snapshot.junction_sequence);
        CHECK(misses == snapshot.patch_misses);
        CHECK(!snapshot.junction_candidate);
    }

    /* Runs that never overlap are not one crossing: IR_L leaves the line
       2 mm before IR_R meets it (line following with the car swinging
       across), 8 mm apart, or one sensor, then the car standing (turning
       on the spot), then the other. No junction, and nothing to report. */
    {
        const uint32_t misses = snapshot.patch_misses;

        junctions = snapshot.junction_sequence;
        set_line_black(true, false);
        run_for(&context, 600000U);     /* 15 mm */
        set_line_black(false, false);
        run_for(&context, 80000U);      /* 2 mm of white */
        set_line_black(false, true);
        run_for(&context, 600000U);
        set_line_black(false, false);
        run_for(&context, 400000U);
        set_line_black(true, false);
        run_for(&context, 600000U);
        set_line_black(false, false);
        run_for(&context, 320000U);     /* 8 mm of white */
        set_line_black(false, true);
        run_for(&context, 600000U);
        set_line_black(false, false);
        run_for(&context, 400000U);     /* 10 mm to the next mark */
        buddy3_get_snapshot(&snapshot);
        CHECK(junctions == snapshot.junction_sequence);
        set_line_black(true, false);
        run_for(&context, 600000U);
        set_line_black(false, false);
        context.left_measured_mm_s = 0;
        context.right_measured_mm_s = 0;
        run_for(&context, 500000U);
        set_line_black(false, true);
        run_for(&context, 100000U);
        context.left_measured_mm_s = 25;
        context.right_measured_mm_s = 25;
        run_for(&context, 600000U);
        set_line_black(false, false);
        run_for(&context, 400000U);
        buddy3_get_snapshot(&snapshot);
        CHECK(junctions == snapshot.junction_sequence);
        CHECK(misses == snapshot.patch_misses);
    }

    /* A branch under IR_L only, with IR_R just touching the line for 3 mm
       in the middle: not a junction, and the log gives both runs. */
    {
        const uint32_t misses = snapshot.patch_misses;

        junctions = snapshot.junction_sequence;
        set_line_black(true, false);
        run_for(&context, 320000U);     /* 8 mm */
        set_line_black(true, true);
        run_for(&context, 120000U);     /* 3 mm */
        set_line_black(true, false);
        run_for(&context, 360000U);     /* 9 mm */
        set_line_black(false, false);
        run_for(&context, 100000U);
        buddy3_get_snapshot(&snapshot);
        CHECK(junctions == snapshot.junction_sequence);
        CHECK((misses + 1U) == snapshot.patch_misses);
        CHECK(B3_PATCH_SHORT == (b3_patch_why_t)snapshot.patch_miss_why);
        CHECK((snapshot.patch_miss_l_mm >= 18U) && (snapshot.patch_miss_l_mm <= 21U));
        CHECK((snapshot.patch_miss_r_mm >= 1U) && (snapshot.patch_miss_r_mm <= 4U));
        CHECK((snapshot.patch_miss_mm >= 1U) && (snapshot.patch_miss_mm <= 4U));
    }
    context.left_measured_mm_s = 120;
    context.right_measured_mm_s = 120;

    /* STOP/RESET from the controller forgets a held letter. */
    drive_barcode(&context, 'B');
    buddy3_get_snapshot(&snapshot);
    CHECK('B' == snapshot.held_symbol);
    buddy3_clear_held_symbol();
    run_for(&context, 10000U);
    buddy3_get_snapshot(&snapshot);
    CHECK('\0' == snapshot.held_symbol);
    CHECK(!snapshot.stop_required);

    /* nyx, 2026-11-06: scanning is off during a turn or stop. If IR_B stayed
     * white, the white before the next barcode counts from before the
     * pause, so a barcode soon after line following resumes still decodes
     * (100 ms of white since resuming; 10 x 20 ms narrow is needed). */
    {
        uint32_t rejected;

        buddy3_get_snapshot(&snapshot);
        rejected = snapshot.rejected_scans;
        run_for(&context, 300000U);
        context.scan_enabled = false;
        run_for(&context, 500000U);
        context.scan_enabled = true;
        drive_barcode_lead(&context, 'C', 100000U);
        buddy3_get_snapshot(&snapshot);
        CHECK('C' == snapshot.held_symbol);
        CHECK(rejected == snapshot.rejected_scans);

        /* IR_B saw black during the pause, so the white starts after it:
         * 20 ms + 100 ms is too short, reason 7 with the times kept. */
        context.scan_enabled = false;
        run_for(&context, 100000U);
        emit(true);
        run_for(&context, 50000U);
        emit(false);
        run_for(&context, 20000U);
        context.scan_enabled = true;
        drive_barcode_lead(&context, 'D', 100000U);
        buddy3_get_snapshot(&snapshot);
        CHECK('C' == snapshot.held_symbol);
        CHECK((rejected + 1U) == snapshot.rejected_scans);
        CHECK(B3_REJECT_LEADING_QUIET == (b3_reject_reason_t)snapshot.reject_reason);
        CHECK(!snapshot.reject_encoder);
        CHECK((snapshot.reject_lead >= 100000U) && (snapshot.reject_lead <= 130000U));
        CHECK((snapshot.reject_need >= 180000U) && (snapshot.reject_need <= 220000U));
        buddy3_clear_held_symbol();
        run_for(&context, 10000U);
    }

    /* nyx, 2026-11-06: the VC stops on a barcode start, backs up with
     * scanning off and reads it slowly. The decoder measures white as net
     * encoder distance, so the white before the barcode must start where
     * the reverse ended, not where IR_B left the barcode backwards (that
     * gave about 0 mm: reason 7). 300 ticks/s; narrow 20 ms = 6 ticks, so
     * 60 ticks of white are needed; the reverse goes 90 ticks past it. */
    {
        uint32_t rejected;

        g_pos_on = true;
        pos_rate(300);
        context.base_speed_mm_s = 100;
        run_for(&context, 1000000U);
        emit(true); /* first bar */
        run_for(&context, 20000U);
        emit(false);
        run_for(&context, 60000U);
        emit(true); /* second bar: the VC backs up from here */
        run_for(&context, 10000U);
        buddy3_get_snapshot(&snapshot);
        CHECK(snapshot.scan_active && (snapshot.scan_elements >= 2U));
        rejected = snapshot.rejected_scans;
        context.scan_enabled = false;
        pos_rate(0);
        run_for(&context, 100000U);
        pos_rate(-300); /* IR_B crosses the two bars backwards */
        run_for(&context, 10000U);
        emit(false);
        run_for(&context, 60000U);
        emit(true);
        run_for(&context, 20000U);
        emit(false);
        run_for(&context, 300000U); /* 90 ticks of white backwards */
        pos_rate(0);
        run_for(&context, 100000U);
        context.base_speed_mm_s = 50;
        context.scan_enabled = true;
        pos_rate(300);
        drive_barcode_lead(&context, 'E', 300000U);
        buddy3_get_snapshot(&snapshot);
        CHECK('E' == snapshot.held_symbol);
        CHECK(rejected == snapshot.rejected_scans);
        CHECK(50 == snapshot.line.left_mm_s);
        context.base_speed_mm_s = 0;
        buddy3_clear_held_symbol();
        run_for(&context, 10000U);
        g_pos_on = false;
    }

    /* A serious fault from another Buddy 3 task stops the car for good. */
    buddy3_report_fault(B3_FAULT_MBF_SEND, E_IO);
    run_for(&context, 10000U);
    buddy3_get_snapshot(&snapshot);
    CHECK(snapshot.stop_required);
    CHECK(B3_FAULT_MBF_SEND == (b3_fault_t)snapshot.fault_source);
    CHECK(E_IO == snapshot.task_error);
    CHECK((0 == snapshot.line.left_mm_s) && (0 == snapshot.line.right_mm_s));
    buddy3_report_fault(B3_FAULT_MBX_RECEIVE, E_OBJ);   /* first cause kept */
    run_for(&context, 50000U);
    buddy3_get_snapshot(&snapshot);
    CHECK(B3_FAULT_MBF_SEND == (b3_fault_t)snapshot.fault_source);
    CHECK(snapshot.stop_required);
    (void)puts("PASS: letter held until a 25 mm/s junction, speed dips tolerated, short patch reported, cleared on request, faults latch a stop");
}

int main(void)
{
    test_driver();
    test_task_boundary();
    test_held_symbol_and_faults();
    (void)printf("All %u adapter checks passed using test doubles.\n", g_checks);
    return EXIT_SUCCESS;
}
