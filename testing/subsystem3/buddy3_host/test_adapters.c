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
static bool g_debug_high;
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
    g_initialised |= (1U << pin);
    if (B3_DEBUG_GPIO == pin) g_debug_high = initial_high;
    return E_OK;
}
uint32_t b3_port_gpio_get_all(void) { return g_pins; }
void b3_port_gpio_put(unsigned int pin, bool value)
{
    if (B3_DEBUG_GPIO == pin) g_debug_high = value;
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
        b3_ir_gpio_irq(B3_BARCODE_GPIO, events);
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
    bool high = (black == (B3_BARCODE_BLACK_LEVEL != 0U));
    if (high)
    {
        g_pins |= (1U << B3_BARCODE_GPIO);
    }
    else
    {
        g_pins &= ~(1U << B3_BARCODE_GPIO);
    }
    ++g_irq_calls;
    b3_ir_gpio_irq(B3_BARCODE_GPIO,
                   high ? B3_GPIO_EDGE_RISE : B3_GPIO_EDGE_FALL);
    CHECK(!g_debug_high);
}

static void test_driver(void)
{
    unsigned int i;
    b3_edge_t edge;
    b3_ir_sample_t sample;
    CHECK(E_OK == b3_ir_init(NULL));
    CHECK(g_irq_installed);
    CHECK((g_initialised & (1U << B3_LINE_LEFT_GPIO)) != 0U);
    CHECK((g_initialised & (1U << B3_LINE_RIGHT_GPIO)) != 0U);
    CHECK((g_initialised & (1U << B3_BARCODE_GPIO)) != 0U);
    // CHECK((g_initialised & (1U << B3_DEBUG_GPIO)) != 0U);
    #if B3_DEBUG_ENABLED
        CHECK((g_initialised & (1U << B3_DEBUG_GPIO)) != 0U);
    #else
        CHECK((g_initialised & (1U << B3_DEBUG_GPIO)) == 0U);
    #endif
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
    b3_ir_gpio_irq(B3_BARCODE_GPIO, B3_GPIO_EDGE_RISE | B3_GPIO_EDGE_FALL);
    CHECK(B3_IR_DISCONTINUITY == b3_ir_pop(&edge));
    CHECK(1U == b3_ir_fault_count());
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
    g_pins &= ~((1U << B3_LINE_LEFT_GPIO) | (1U << B3_LINE_RIGHT_GPIO));
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
    CHECK(B3_NAV_NONE == buddy3_map_symbol('A')); /* Unconfirmed mapping disabled. */
    (void)puts("PASS: micro T-Kernel adapter startup, bounded task pass and stale-context stop");
}

int main(void)
{
    test_driver();
    test_task_boundary();
    (void)printf("All %u adapter checks passed using test doubles.\n", g_checks);
    return EXIT_SUCCESS;
}
