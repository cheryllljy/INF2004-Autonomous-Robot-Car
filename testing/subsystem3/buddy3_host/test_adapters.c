/* Deterministic simulation of the micro T-Kernel/BSP boundary. No hardware. */
#include "buddy3.h"
#include "b3_port.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t g_now = 1000U;
static uint32_t g_pins = UINT32_MAX;
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
static bool g_sampler_installed;
static int32_t g_adc = 300; /* IR_B AO reading for the analog build */

/* AO levels the analog emit() drives, on the wired polarity. */
#define AO_BLACK ((B3_IR_B_BLACK_LEVEL != 0U) ? 3500 : 595)
#define AO_WHITE ((B3_IR_B_BLACK_LEVEL != 0U) ? 300 : 3795)

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
ER b3_port_adc_input_init(unsigned int pin)
{
    g_initialised |= (1U << pin);
    return E_OK;
}
int32_t b3_port_adc_read(unsigned int channel)
{
    CHECK(B3_IR_B_ADC_CH == channel);
    return g_adc;
}
ER b3_port_install_barcode_sampler(void)
{
    g_sampler_installed = true;
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

#if B3_BARCODE_ANALOG
/* One sampler reading, 500 us after the last. */
static void sample(int32_t raw)
{
    g_now += 500U;
    g_adc = raw;
    b3_ir_analog_sample(g_now, (uint32_t)raw);
    CHECK(!g_debug_high);
}

/* A clean colour change: two samples at the new level. */
static void emit(bool black)
{
    ++g_irq_calls;
    sample(black ? AO_BLACK : AO_WHITE);
    sample(black ? AO_BLACK : AO_WHITE);
}
#else
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
#endif

static void test_driver(void)
{
    unsigned int i;
    b3_edge_t edge;
    b3_ir_sample_t sample;
    g_adc = AO_WHITE;
    CHECK(E_OK == b3_ir_init(NULL));
#if B3_BARCODE_ANALOG
    CHECK(g_sampler_installed && !g_irq_installed);
#else
    CHECK(!g_sampler_installed);
    CHECK(g_irq_installed == (B3_INSTALL_IO_BANK0_HANDLER != 0U));
#endif
    CHECK((g_initialised & (1U << B3_IR_L_GPIO)) != 0U);
    CHECK((g_initialised & (1U << B3_IR_B_GPIO)) != 0U);
    CHECK((g_initialised & (1U << B3_IR_R_GPIO)) != 0U);
    CHECK(0U == g_outputs); /* no diagnostic output pins */
    b3_ir_gpio_irq(0U, B3_GPIO_EDGE_RISE);
    CHECK(B3_IR_EMPTY == b3_ir_pop(&edge));
    emit(true);
    CHECK(!b3_ir_read_when_idle(&sample));
    CHECK(B3_IR_EDGE == b3_ir_pop(&edge));
#if B3_BARCODE_ANALOG
    /* timed at the threshold crossing, between the last white sample and
       the first black one */
    CHECK(edge.black && (edge.time_us > g_now - 1000U) &&
          (edge.time_us < g_now - 500U));
#else
    CHECK(edge.black && (g_now == edge.time_us));
#endif
    CHECK(b3_ir_read_when_idle(&sample));
    g_pending = B3_GPIO_EDGE_FALL;
    CHECK(!b3_ir_read_when_idle(&sample));
    g_pending = 0U;
#if !B3_BARCODE_ANALOG
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
#endif
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

#if B3_BARCODE_ANALOG
/* Pop every queued edge; return how many, and the last one. */
static unsigned int drain(b3_edge_t *p_last)
{
    unsigned int n = 0U;
    while (B3_IR_EDGE == b3_ir_pop(p_last))
    {
        ++n;
    }
    return n;
}

/* Raw AO value for a level on the black-is-high scale. */
static int32_t ao(int32_t level)
{
    return (B3_IR_B_BLACK_LEVEL != 0U) ? level : (4095 - level);
}

static void test_analog(void)
{
    b3_edge_t edge;
    b3_ir_analog_stats_t stats;
    unsigned int i;
    uint32_t t0;

    /* Noise on white with no contrast learned yet: no edges. */
    g_adc = ao(300);
    CHECK(E_OK == b3_ir_init(NULL));
    for (i = 0U; i < 200U; ++i)
    {
        sample(ao((0U == i % 2U) ? 250 : 450));
    }
    CHECK(0U == drain(&edge));
    b3_ir_analog_stats(&stats);
    CHECK(!stats.black_now && (uint32_t)(stats.black - stats.white) < B3_ANALOG_MIN_CONTRAST);

    /* A single-sample spike is not an edge, even with contrast learned. */
    for (i = 0U; i < 20U; ++i)
    {
        sample(ao(3500));
    }
    for (i = 0U; i < 40U; ++i)
    {
        sample(ao(300));
    }
    CHECK(2U == drain(&edge)); /* the wide bar above: black, then white */
    CHECK(!edge.black);
    sample(ao(3500));
    sample(ao(300));
    sample(ao(300));
    CHECK(0U == drain(&edge));

    /* A thin bar that only reaches 2300 of a 300..3500 range (62 %) still
       gives both edges; its dip back to 1500 does not end it early. */
    sample(ao(2300));
    sample(ao(2300));
    sample(ao(2300));
    CHECK(1U == drain(&edge) && edge.black);
    sample(ao(1700)); /* above 1900 - hysteresis: still black */
    sample(ao(2300));
    CHECK(0U == drain(&edge));
    sample(ao(300));
    sample(ao(300));
    CHECK(1U == drain(&edge) && !edge.black);

    /* A dip that reaches only 22 % gives no edge at all. */
    for (i = 0U; i < 10U; ++i)
    {
        sample(ao(1000));
    }
    for (i = 0U; i < 10U; ++i)
    {
        sample(ao(300));
    }
    CHECK(0U == drain(&edge));

    /* Crossing the threshold without the hysteresis is a near miss. */
    b3_ir_analog_stats(&stats);
    i = stats.near_misses;
    sample(ao(stats.threshold + 10));
    sample(ao(300));
    b3_ir_analog_stats(&stats);
    CHECK(stats.near_misses == i + 1U);
    CHECK(0U == drain(&edge));

    /* Edge timing: a ramp that crosses the threshold 3/4 of the way
       between two samples is timed there, not at the confirm sample. */
    b3_ir_analog_stats(&stats);
    sample(ao(stats.threshold - 300));
    t0 = g_now;
    sample(ao(stats.threshold + 100));
    sample(ao(3500));
    sample(ao(3500));
    CHECK(1U == drain(&edge) && edge.black);
    CHECK(edge.time_us == t0 + 375U);

    /* The levels adapt: after a long run at a lower contrast (sensor
       raised), a bar of the new size is still found. */
    for (i = 0U; i < 40U; ++i)
    {
        sample(ao(300));
    }
    (void)drain(&edge);
    t0 = 0U; /* edges counted in the second half */
    for (i = 0U; i < 30000U; ++i)
    {
        sample(ao((0U == (i / 20U) % 2U) ? 1400 : 600));
        if (19U == i % 20U)
        {
            t0 += (i >= 15000U) ? drain(&edge) : 0U;
            (void)drain(&edge);
        }
    }
    b3_ir_analog_stats(&stats);
    CHECK((stats.black < 1500U) && (stats.black > 1300U));
    CHECK((stats.white > 500U) && (stats.white < 700U));
    CHECK(750U == t0); /* one edge per 20-sample bar or gap */

    /* The queue still reports overflow as a discontinuity. */
    for (i = 0U; i <= B3_EDGE_QUEUE_SIZE; ++i)
    {
        sample(ao((0U == i % 2U) ? 600 : 1400));
        sample(ao((0U == i % 2U) ? 600 : 1400));
    }
    CHECK(B3_IR_DISCONTINUITY == b3_ir_pop(&edge));
    (void)drain(&edge);
    (void)puts("PASS: analog IR_B: noise and spike rejection, thin bar, hysteresis, interpolated timing, adaptation");
}
#endif

static void test_task_boundary(void)
{
    b3_context_t context = {0};
    b3_snapshot_t snapshot;
    g_pins = UINT32_MAX;
    CHECK(E_OK == buddy3_start(NULL));
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
    CHECK((120 == snapshot.line.left_mm_s) && (120 == snapshot.line.right_mm_s));

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
    emit(false);
    context.time_us = g_now;
    buddy3_set_context(&context);
    one_task_pass();
    buddy3_get_snapshot(&snapshot);
    CHECK(!snapshot.junction_candidate);

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

int main(void)
{
    test_driver();
#if B3_BARCODE_ANALOG
    test_analog();
#endif
    test_task_boundary();
    (void)printf("All %u adapter checks passed using test doubles.\n", g_checks);
    return EXIT_SUCCESS;
}
