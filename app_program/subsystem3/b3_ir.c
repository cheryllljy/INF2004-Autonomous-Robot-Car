#include "b3_ir.h"

#include "b3_port.h"


_Static_assert(
    B3_EDGE_QUEUE_SIZE > 2U,
    "Edge queue is too small"
);


/*
 * TEMPORARY DIAGNOSTIC GPIOs
 *
 * GP2  = reserved for barcode decoder rejection
 * GP5  = valid barcode edge queued
 * GP6  = barcode ISR entered
 * GP16 = BOTH rising + falling edge flags received
 * GP17 = edge queue became full
 *
 * GP3 = scan_enabled       (buddy3.c)
 * GP4 = scan_active        (bench)
 * GP7 = barcode sensor DOUT
 */
#define B3_DIAG_BARCODE_REJECT_GPIO   2U
#define B3_DIAG_VALID_EDGE_GPIO       5U
#define B3_DIAG_BOTH_EDGE_GPIO       16U
#define B3_DIAG_QUEUE_FULL_GPIO      17U


static volatile b3_edge_t g_edges[B3_EDGE_QUEUE_SIZE];

static volatile uint32_t g_head;
static volatile uint32_t g_tail;
static volatile uint32_t g_fault_count;

static volatile bool g_discontinuity;

static b3_position_reader_t g_position_reader;


/*
 * Return true when the selected GPIO is currently
 * at the configured electrical level representing black.
 */
static bool black_level(
    uint32_t pins,
    unsigned int pin,
    unsigned int level)
{
    return (((pins >> pin) & 1U) == level);
}


/*
 * Poll the barcode sensor.
 */
static b3_edge_t sample_barcode(uint32_t pins)
{
    b3_edge_t edge = {0};


    edge.time_us =
        b3_port_time_us();


    edge.black =
        black_level(
            pins,
            B3_BARCODE_GPIO,
            B3_BARCODE_BLACK_LEVEL
        );


    if (0 != g_position_reader)
    {
        edge.position_valid =
            g_position_reader(
                &edge.position_ticks
            );
    }


    return edge;
}


/*
 * Initialise IR sensors and barcode interrupt.
 */
ER b3_ir_init(b3_position_reader_t reader)
{
    ER status;

    b3_irq_state_t saved;


    status =
        b3_port_init();


    if (E_OK != status)
    {
        return status;
    }


    /*
     * Disable barcode interrupt while setting up.
     */
    b3_port_barcode_irq_enable(false);


    /*
     * Left line sensor.
     */
    status =
        b3_port_gpio_input_init(
            B3_LINE_LEFT_GPIO,
            B3_INPUT_PULL_UP != 0U
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * Right line sensor.
     */
    status =
        b3_port_gpio_input_init(
            B3_LINE_RIGHT_GPIO,
            B3_INPUT_PULL_UP != 0U
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * Barcode sensor.
     */
    status =
        b3_port_gpio_input_init(
            B3_BARCODE_GPIO,
            B3_INPUT_PULL_UP != 0U
        );


    if (E_OK != status)
    {
        return status;
    }


#if B3_DEBUG_ENABLED

    /*
     * GP6:
     * barcode ISR entered.
     */
    status =
        b3_port_gpio_output_init(
            B3_DEBUG_GPIO,
            false
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * GP5:
     * valid edge queued.
     */
    status =
        b3_port_gpio_output_init(
            B3_DIAG_VALID_EDGE_GPIO,
            false
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * GP2:
     * reserved for barcode decoder rejection.
     *
     * b3_ir.c does NOT turn this on anymore.
     */
    status =
        b3_port_gpio_output_init(
            B3_DIAG_BARCODE_REJECT_GPIO,
            false
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * GP16:
     * BOTH rising + falling edge flags.
     */
    status =
        b3_port_gpio_output_init(
            B3_DIAG_BOTH_EDGE_GPIO,
            false
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * GP17:
     * edge queue full.
     */
    status =
        b3_port_gpio_output_init(
            B3_DIAG_QUEUE_FULL_GPIO,
            false
        );


    if (E_OK != status)
    {
        return status;
    }

#endif


    /*
     * Reset barcode edge queue.
     */
    b3_port_lock(&saved);


    g_head =
        0U;

    g_tail =
        0U;

    g_fault_count =
        0U;

    g_discontinuity =
        false;

    g_position_reader =
        reader;


    b3_port_unlock(saved);


    /*
     * Clear stale barcode interrupt flags.
     */
    b3_port_barcode_irq_ack(
        B3_GPIO_EDGE_BOTH
    );


#if B3_INSTALL_IO_BANK0_HANDLER

    status =
        b3_port_install_barcode_irq();


    if (E_OK != status)
    {
        return status;
    }

#endif


    /*
     * TEMPORARY TEST:
     *
     * Keep barcode interrupt enabled.
     */
    b3_port_barcode_irq_enable(true);


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
 * Barcode GPIO interrupt callback.
 */
void b3_ir_gpio_irq(
    unsigned int gpio,
    uint32_t events)
{
    uint32_t next;

    uint32_t edge_events;

    b3_irq_state_t saved;

    b3_edge_t edge;


    edge_events =
        events & B3_GPIO_EDGE_BOTH;


    /*
     * Ignore unrelated GPIO interrupts.
     */
    if ((B3_BARCODE_GPIO != gpio) ||
        (0U == edge_events))
    {
        return;
    }


#if B3_DEBUG_ENABLED

    /*
     * GP6:
     *
     * We successfully entered the barcode ISR.
     */
    b3_port_gpio_put(
        B3_DEBUG_GPIO,
        true
    );

#endif


    b3_port_lock(&saved);


    /*
     * Determine next position in circular queue.
     */
    next =
        (g_head + 1U) %
        B3_EDGE_QUEUE_SIZE;


    /*
     * FAULT TYPE 1
     *
     * Both rising and falling edge flags were already
     * pending when the ISR processed the GPIO.
     *
     * This means we cannot determine the individual
     * transition timing accurately.
     *
     * GP16 will latch ON.
     */
    if (edge_events ==
        B3_GPIO_EDGE_BOTH)
    {
        ++g_fault_count;


        g_discontinuity =
            true;


#if B3_DEBUG_ENABLED

        b3_port_gpio_put(
            B3_DIAG_BOTH_EDGE_GPIO,
            true
        );

#endif
    }


    /*
     * FAULT TYPE 2
     *
     * Circular edge queue is full.
     *
     * GP17 will latch ON.
     */
    else if (next == g_tail)
    {
        ++g_fault_count;


        g_discontinuity =
            true;


#if B3_DEBUG_ENABLED

        b3_port_gpio_put(
            B3_DIAG_QUEUE_FULL_GPIO,
            true
        );

#endif
    }


    /*
     * Normal single rising OR falling edge.
     */
    else
    {
        edge =
            (b3_edge_t){0};


        /*
         * Timestamp this edge.
         */
        edge.time_us =
            b3_port_time_us();


        /*
         * Determine black/white directly from the
         * interrupt edge.
         *
         * Do NOT read GPIO again here because the
         * physical pin may already have changed.
         */
        if (edge_events ==
            B3_GPIO_EDGE_RISE)
        {
            /*
             * Rising:
             * LOW -> HIGH
             */
            edge.black =
                (B3_BARCODE_BLACK_LEVEL != 0U);
        }
        else
        {
            /*
             * Falling:
             * HIGH -> LOW
             */
            edge.black =
                (B3_BARCODE_BLACK_LEVEL == 0U);
        }


        /*
         * Optional encoder position.
         */
        if (0 != g_position_reader)
        {
            edge.position_valid =
                g_position_reader(
                    &edge.position_ticks
                );
        }


        /*
         * Add valid edge to circular queue.
         */
        g_edges[g_head] =
            edge;


        g_head =
            next;


#if B3_DEBUG_ENABLED

        /*
         * GP5:
         * at least one valid barcode edge
         * entered the queue.
         */
        b3_port_gpio_put(
            B3_DIAG_VALID_EDGE_GPIO,
            true
        );

#endif
    }


    b3_port_unlock(saved);
}


/*
 * Remove one edge from the barcode edge queue.
 */
b3_ir_pop_status_t b3_ir_pop(
    b3_edge_t *p_edge)
{
    b3_ir_pop_status_t status;

    b3_irq_state_t saved;


    status =
        B3_IR_EMPTY;


    b3_port_lock(&saved);


    /*
     * If timing continuity was lost,
     * discard the current partial queue.
     */
    if (g_discontinuity)
    {
        g_tail =
            g_head;


        g_discontinuity =
            false;


        status =
            B3_IR_DISCONTINUITY;
    }


    /*
     * Otherwise return the oldest queued edge.
     */
    else if (g_tail != g_head)
    {
        *p_edge =
            g_edges[g_tail];


        g_tail =
            (g_tail + 1U) %
            B3_EDGE_QUEUE_SIZE;


        status =
            B3_IR_EDGE;
    }


    b3_port_unlock(saved);


    return status;
}


/*
 * Clear barcode edge queue.
 */
void b3_ir_flush(void)
{
    b3_irq_state_t saved;


    b3_port_lock(&saved);


    g_tail =
        g_head;


    g_discontinuity =
        false;


    b3_port_unlock(saved);
}


/*
 * Poll all three IR sensors.
 */
b3_ir_sample_t b3_ir_read(void)
{
    b3_ir_sample_t sample;

    b3_irq_state_t saved;

    uint32_t pins;


    b3_port_lock(&saved);


    pins =
        b3_port_gpio_get_all();


    sample.left_black =
        black_level(
            pins,
            B3_LINE_LEFT_GPIO,
            B3_LEFT_BLACK_LEVEL
        );


    sample.right_black =
        black_level(
            pins,
            B3_LINE_RIGHT_GPIO,
            B3_RIGHT_BLACK_LEVEL
        );


    sample.barcode =
        sample_barcode(pins);


    b3_port_unlock(saved);


    return sample;
}


/*
 * Poll sensors only when barcode edge processing
 * is idle.
 */
bool b3_ir_read_when_idle(
    b3_ir_sample_t *p_sample)
{
    bool ready;

    b3_irq_state_t saved;


    ready =
        false;


    b3_port_lock(&saved);


    if ((g_head == g_tail) &&
        !g_discontinuity &&
        (0U ==
         b3_port_barcode_irq_pending()))
    {
        uint32_t pins;


        pins =
            b3_port_gpio_get_all();


        p_sample->left_black =
            black_level(
                pins,
                B3_LINE_LEFT_GPIO,
                B3_LEFT_BLACK_LEVEL
            );


        p_sample->right_black =
            black_level(
                pins,
                B3_LINE_RIGHT_GPIO,
                B3_RIGHT_BLACK_LEVEL
            );


        p_sample->barcode =
            sample_barcode(pins);


        ready =
            (0U ==
             b3_port_barcode_irq_pending());
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


    result =
        g_fault_count;


    b3_port_unlock(saved);


    return result;
}