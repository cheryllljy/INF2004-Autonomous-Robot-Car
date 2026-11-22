#include "b3_ir.h"

#include "b3_port.h"


_Static_assert(B3_EDGE_QUEUE_SIZE > 2U, "Edge queue is too small");


static volatile b3_edge_t g_edges[B3_EDGE_QUEUE_SIZE];

static volatile uint32_t g_head;
static volatile uint32_t g_tail;
static volatile uint32_t g_fault_count;

static volatile bool g_discontinuity;

/* Level of the last barcode edge queued (or the level at init). An
 * interrupt that finds the pin at this level was a flick and is dropped. */
static bool g_b_black;

/* Interrupts dropped as flicks (pin back at the level last queued). */
static uint32_t g_glitch_count;

static b3_position_reader_t g_position_reader;

#if defined(APP_TEST) && APP_TEST
static volatile uint32_t g_log_time_us[B3_EDGE_LOG_SIZE];
static volatile uint8_t g_log_events[B3_EDGE_LOG_SIZE];
static volatile uint16_t g_log_peak[B3_EDGE_LOG_SIZE];
static volatile uint32_t g_log_count;
#endif


/*
 * Return true when the selected GPIO is currently
 * at the configured electrical level representing black.
 */
static bool black_level(uint32_t pins, uint32_t pin, uint32_t level)
{
    return (((pins >> pin) & 1U) == level);
}


/*
 * Poll the barcode sensor IR_B.
 */
static b3_edge_t sample_barcode(uint32_t pins)
{
    b3_edge_t edge = { 0 };


    edge.time_us = b3_port_time_us();


    edge.black = black_level(pins, B3_IR_B_GPIO, B3_IR_B_BLACK_LEVEL);


    if (0 != g_position_reader)
    {
        edge.position_valid = g_position_reader(&edge.position_ticks);
    }


    return edge;
}


/*
 * Fill a sample from one GPIO snapshot.
 */
static void sample_all(uint32_t pins, b3_ir_sample_t *p_sample)
{
    p_sample->left_black = black_level(pins, B3_IR_L_GPIO, B3_IR_L_BLACK_LEVEL);


    p_sample->right_black =
        black_level(pins, B3_IR_R_GPIO, B3_IR_R_BLACK_LEVEL);


    p_sample->barcode = sample_barcode(pins);
}


/*
 * Add one edge to the circular queue. Caller holds the port lock.
 *
 * FAULT TYPE 2: the queue is full, so timing continuity is lost.
 */
static void push_edge_locked(const b3_edge_t *p_edge)
{
    uint32_t next = (g_head + 1U) % B3_EDGE_QUEUE_SIZE;


    if (next == g_tail)
    {
        ++g_fault_count;


        g_discontinuity = true;


        return;
    }


    g_edges[g_head] = *p_edge;


    g_head = next;
}


#if defined(APP_TEST) && APP_TEST
/*
 * Test build only: add one entry to the raw edge log. Caller holds the
 * port lock.
 */
static void log_entry_locked(uint32_t time_us, uint8_t events, uint32_t peak)
{
    if (g_log_count < B3_EDGE_LOG_SIZE)
    {
        g_log_time_us[g_log_count] = time_us;

        g_log_events[g_log_count] = events;

        g_log_peak[g_log_count] = (uint16_t)peak;
    }

    ++g_log_count;
}
#endif


/*
 * Initialise IR sensors and barcode interrupt.
 * The interrupt is left disabled; buddy3_start() enables it
 * once the worker task exists.
 */
ER b3_ir_init(b3_position_reader_t reader)
{
    static const uint32_t pins[] = { B3_IR_R_GPIO, B3_IR_L_GPIO, B3_IR_B_GPIO };

    ER status;

    b3_irq_state_t saved;

    uint32_t i;


    status = b3_port_init();


    if (E_OK != status)
    {
        return status;
    }


    /*
     * Disable barcode interrupt while setting up.
     */
    b3_port_barcode_irq_enable(false);


    /*
     * IR_R, IR_L and IR_B inputs.
     */
    for (i = 0U; i < (sizeof(pins) / sizeof(pins[0])); ++i)
    {
        status = b3_port_gpio_input_init(pins[i], B3_INPUT_PULL_UP != 0U);


        if (E_OK != status)
        {
            return status;
        }
    }


    /*
     * Reset barcode edge queue.
     */
    b3_port_lock(&saved);


    g_head = 0U;

    g_tail = 0U;

    g_fault_count = 0U;

    g_discontinuity = false;

    g_position_reader = reader;

    g_b_black = sample_barcode(b3_port_gpio_get_all()).black;

    g_glitch_count = 0U;


    b3_port_unlock(saved);


    /*
     * Clear stale barcode interrupt flags.
     */
    b3_port_barcode_irq_ack(B3_GPIO_EDGE_BOTH);


#if B3_INSTALL_IO_BANK0_HANDLER

    status = b3_port_install_barcode_irq();


    if (E_OK != status)
    {
        return status;
    }

#endif


    return E_OK;
}


/*
 * Enable or disable barcode interrupt.
 */
void b3_ir_enable_barcode_irq(bool enabled)
{
    b3_port_barcode_irq_enable(enabled);
}


/*
 * IR_B (barcode) GPIO interrupt callback.
 */
void b3_ir_gpio_irq(uint32_t gpio, uint32_t events)
{
    uint32_t edge_events;

    b3_irq_state_t saved;

    b3_edge_t edge;


    edge_events = events & B3_GPIO_EDGE_BOTH;


    /*
     * Ignore unrelated GPIO interrupts.
     */
    if ((B3_IR_B_GPIO != gpio) || (0U == edge_events))
    {
        return;
    }


    b3_port_lock(&saved);


#if defined(APP_TEST) && APP_TEST
    /*
     * Test build only: log every interrupt before any filtering.
     */
    log_entry_locked(b3_port_time_us(), (uint8_t)edge_events, 0U);
#endif


    /*
     * Rise, fall or both pending: take the level from the pin. The
     * dispatcher cleared the pending flags before calling here, so an
     * edge after this read raises a new interrupt and the last edge queued
     * always ends at the pin's real level. The comparator can flick for
     * tens of microseconds as a bar edge passes; a flick that is already
     * over leaves the pin at the level last queued and is dropped. Taking
     * the level from the edge type instead could queue the same colour
     * twice: a flick's last edge is lost when it comes just before the
     * flag of an earlier edge of the same type is cleared.
     */
    edge = sample_barcode(b3_port_gpio_get_all());


    if (edge.black == g_b_black)
    {
        ++g_glitch_count;
    }
    else
    {
        uint32_t last = (g_head + B3_EDGE_QUEUE_SIZE - 1U) %
                        B3_EDGE_QUEUE_SIZE;


        /*
         * The level went back within B3_MIN_EDGE_US of the edge still
         * waiting at the end of the queue: that edge was a flick, so it is
         * taken back instead of queuing a second one. Fast chatter then
         * cannot fill the queue between two task passes.
         */
        if ((g_head != g_tail) &&
            ((edge.time_us - g_edges[last].time_us) < B3_MIN_EDGE_US))
        {
            g_head = last;

            ++g_glitch_count;
        }
        else
        {
            push_edge_locked(&edge);
        }


        g_b_black = edge.black;
    }


    b3_port_unlock(saved);
}


uint32_t b3_ir_glitch_count(void)
{
    return g_glitch_count;
}


#if defined(APP_TEST) && APP_TEST
void b3_ir_edge_log_reset(void)
{
    b3_irq_state_t saved;


    b3_port_lock(&saved);

    g_log_count = 0U;

    b3_port_unlock(saved);
}


uint32_t b3_ir_edge_log_read(uint32_t *p_time_us, uint8_t *p_events,
                             uint16_t *p_peak, uint32_t max)
{
    b3_irq_state_t saved;

    uint32_t total;

    uint32_t i;


    b3_port_lock(&saved);

    total = g_log_count;

    for (i = 0U; (i < max) && (i < total) && (i < B3_EDGE_LOG_SIZE); ++i)
    {
        p_time_us[i] = g_log_time_us[i];

        p_events[i] = g_log_events[i];

        if (0 != p_peak)
        {
            p_peak[i] = g_log_peak[i];
        }
    }

    b3_port_unlock(saved);


    return total;
}
#endif


/*
 * Remove one edge from the barcode edge queue.
 */
b3_ir_pop_status_t b3_ir_pop(b3_edge_t *p_edge)
{
    b3_ir_pop_status_t status;

    b3_irq_state_t saved;


    status = B3_IR_EMPTY;


    b3_port_lock(&saved);


    /*
     * If timing continuity was lost,
     * discard the current partial queue.
     */
    if (g_discontinuity)
    {
        g_tail = g_head;


        g_discontinuity = false;


        status = B3_IR_DISCONTINUITY;
    }


    /*
     * Otherwise return the oldest queued edge.
     */
    else if (g_tail != g_head)
    {
        *p_edge = g_edges[g_tail];


        g_tail = (g_tail + 1U) % B3_EDGE_QUEUE_SIZE;


        status = B3_IR_EDGE;
    }


    b3_port_unlock(saved);


    return status;
}


/*
 * Clear barcode edge queue.
 *
 * returns true if anything was thrown away (an edge or a
 * lost-edge mark).
 */
bool b3_ir_flush(void)
{
    b3_irq_state_t saved;

    bool discarded;


    b3_port_lock(&saved);


    discarded = (g_tail != g_head) || g_discontinuity;


    g_tail = g_head;


    g_discontinuity = false;


    b3_port_unlock(saved);


    return discarded;
}


/*
 * Poll the IR_L, IR_R and IR_B sensors.
 */
b3_ir_sample_t b3_ir_read(void)
{
    b3_ir_sample_t sample;

    b3_irq_state_t saved;


    b3_port_lock(&saved);


    sample_all(b3_port_gpio_get_all(), &sample);


    b3_port_unlock(saved);


    return sample;
}


/*
 * Poll sensors only when barcode edge processing
 * is idle.
 */
bool b3_ir_read_when_idle(b3_ir_sample_t *p_sample)
{
    bool ready;

    b3_irq_state_t saved;


    ready = false;


    b3_port_lock(&saved);


    if ((g_head == g_tail) && !g_discontinuity &&
        (0U == b3_port_barcode_irq_pending()))
    {
        sample_all(b3_port_gpio_get_all(), p_sample);


        ready = (0U == b3_port_barcode_irq_pending());
    }


    b3_port_unlock(saved);


    return ready;
}


/*
 * Return number of barcode edge faults.
 */
uint32_t b3_ir_fault_count(void)
{
    uint32_t result;

    b3_irq_state_t saved;


    b3_port_lock(&saved);


    result = g_fault_count;


    b3_port_unlock(saved);


    return result;
}
