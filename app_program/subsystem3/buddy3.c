#include "buddy3.h"
#include "b3_port.h"



static b3_context_t g_context;
static b3_snapshot_t g_snapshot;

static b3_nav_event_t g_events[B3_EVENT_QUEUE_SIZE];

static uint32_t g_event_head;
static uint32_t g_event_tail;

static b3_line_t g_line;
static b3_barcode_t g_scan;

static bool g_started;

/* Worker task and the cyclic handler that wakes it every
   B3_SERVICE_DELAY_MS. A cyclic wake-up keeps a steady period; a
   tk_dly_tsk() after each pass stretches to almost two 10 ms ticks. */
static ID g_task_id;
static ID g_cyc_id;

static bool g_motion_good;

static int32_t g_min_speed;
static int32_t g_max_speed;

static uint32_t g_sequence;

static uint32_t g_side_black_since;
static bool g_side_black;

static int32_t g_side_min_speed;
static bool g_side_motion_good;

static bool g_event_fault;

static uint32_t g_dropped_events;

static uint32_t g_last_service_us;
static uint32_t g_max_service_gap_us;

#if defined(APP_TEST) && APP_TEST
/* Test build only: set by the test, cleared by the task (single writer
   of g_max_service_gap_us stays the task). */
static volatile bool g_reset_max_gap;

void buddy3_test_reset_max_gap(void)
{
    g_reset_max_gap =
        true;
}
#endif


/*
 * Update Buddy3 context from the vehicle controller /
 * bench program.
 */
void buddy3_set_context(
    const b3_context_t *p_context)
{
    b3_irq_state_t saved;


    b3_port_lock(&saved);

    g_context =
        *p_context;

    b3_port_unlock(saved);
}


/*
 * Read the latest Buddy3 output snapshot.
 */
void buddy3_get_snapshot(
    b3_snapshot_t *p_snapshot)
{
    b3_irq_state_t saved;


    b3_port_lock(&saved);

    *p_snapshot =
        g_snapshot;

    b3_port_unlock(saved);
}


/*
 * Retrieve the next navigation event.
 */
bool buddy3_pop_navigation(
    b3_nav_event_t *p_event)
{
    bool available;
    b3_irq_state_t saved;


    available =
        false;


    b3_port_lock(&saved);


    if (g_event_tail != g_event_head)
    {
        *p_event =
            g_events[g_event_tail];


        if (g_event_fault)
        {
            p_event->navigation_allowed =
                false;
        }


        g_event_tail =
            (g_event_tail + 1U) %
            B3_EVENT_QUEUE_SIZE;


        available =
            true;
    }


    b3_port_unlock(saved);


    return available;
}


/*
 * Convert a decoded barcode symbol into
 * a navigation command.
 */
b3_nav_command_t buddy3_map_symbol(
    char symbol)
{
#if B3_ENABLE_EXAMPLE_NAV_MAP

    /*
     * Replace ONLY this mapping when the lecturer
     * confirms the actual course barcode meanings.
     */
    switch (symbol)
    {
        case 'A':
            return B3_NAV_LEFT;

        case 'B':
            return B3_NAV_RIGHT;

        case 'C':
            return B3_NAV_STRAIGHT;

        case 'D':
            return B3_NAV_UTURN;

        default:
            return B3_NAV_NONE;
    }

#else

    (void)symbol;

    return B3_NAV_NONE;

#endif
}


/*
 * Return absolute magnitude of a bounded
 * signed sensor value.
 */
static int32_t magnitude(
    int32_t value)
{
    return (value < 0) ?
        -value :
        value;
}


/*
 * Record whether robot motion is suitable
 * for timed barcode decoding.
 */
static void record_motion(
    const b3_context_t *p_context,
    bool fresh)
{
    int32_t speed;

    bool imu_ok;


    speed =
        ((int32_t)p_context->left_measured_mm_s +
         p_context->right_measured_mm_s) /
        2;


    imu_ok =
        !p_context->imu_valid ||
        (
            (p_context->yaw_rate_mdeg_s >=
             -B3_MAX_YAW_MDEG_S) &&
            (p_context->yaw_rate_mdeg_s <=
             B3_MAX_YAW_MDEG_S)
        );


    if (!fresh ||
        !p_context->motion_valid ||
        (speed <= 0) ||
        (
            magnitude(
                (int32_t)p_context->left_measured_mm_s -
                p_context->right_measured_mm_s
            ) > 60
        ) ||
        !imu_ok ||
        p_context->terrain_unstable ||
        p_context->stop_requested)
    {
        g_motion_good =
            false;
    }


    if (speed < g_min_speed)
    {
        g_min_speed =
            speed;
    }


    if (speed > g_max_speed)
    {
        g_max_speed =
            speed;
    }
}


/*
 * Publish a successfully decoded barcode.
 */
static void publish_barcode(
    b3_snapshot_t *p_output,
    b3_barcode_result_t barcode,
    const b3_context_t *p_context,
    bool fresh)
{
    uint32_t next;

    b3_irq_state_t saved;

    b3_nav_event_t event =
        (b3_nav_event_t){0};

    bool speed_ok;


    speed_ok =
        barcode.used_encoder ||
        (
            (g_min_speed > 0) &&
            (
                (int64_t)g_max_speed * 100 <=
                (int64_t)g_min_speed *
                B3_MAX_TIMED_SPEED_RATIO_PCT
            )
        );


    event.sequence =
        ++g_sequence;

    event.barcode =
        barcode;

    event.command =
        buddy3_map_symbol(
            barcode.symbol
        );


    event.navigation_allowed =
        fresh &&
        p_context->follow_enabled &&
        !p_context->stop_requested &&
        !p_output->line.stop &&
        g_motion_good &&
        speed_ok &&
        !g_event_fault &&
        (B3_NAV_NONE != event.command);


    p_output->last_barcode =
        event;


    b3_port_lock(&saved);


    next =
        (g_event_head + 1U) %
        B3_EVENT_QUEUE_SIZE;


    if (next == g_event_tail)
    {
        ++g_dropped_events;

        g_event_fault =
            true;
    }
    else
    {
        g_events[g_event_head] =
            event;

        g_event_head =
            next;
    }


    b3_port_unlock(saved);
}


/*
 * Run one complete Buddy3 service cycle.
 */
static void service_once(void)
{
    b3_snapshot_t output;

    b3_context_t context;

    b3_ir_sample_t sample;

    b3_barcode_result_t decoded;

    b3_edge_t edge;

    b3_ir_pop_status_t status;

    bool fresh;
    bool scan_enabled;

    uint32_t now;
    uint32_t service_gap;

    b3_irq_state_t saved;

    uint32_t i;

    int16_t base_speed;


    status =
        B3_IR_EMPTY;

    base_speed =
        B3_BASE_SPEED_MM_S;


    /*
     * Copy shared context / snapshot.
     */
    b3_port_lock(&saved);

    context =
        g_context;

    output =
        g_snapshot;

    b3_port_unlock(saved);


    /*
     * Read current IR sensor state.
     */
    sample =
        b3_ir_read();


    now =
        sample.barcode.time_us;


    service_gap =
        now -
        g_last_service_us;


    /*
     * Context is considered fresh if the controller
     * updated it recently enough.
     */
    fresh =
        (now - context.time_us) <=
        B3_FEEDBACK_MAX_AGE_US;


#if defined(APP_TEST) && APP_TEST
    if (g_reset_max_gap)
    {
        g_max_service_gap_us =
            0U;

        g_reset_max_gap =
            false;
    }
#endif


    if ((now - g_last_service_us) >
        g_max_service_gap_us)
    {
        g_max_service_gap_us =
            now -
            g_last_service_us;
    }


    g_last_service_us =
        now;


    /*
     * Reduce speed while reading a barcode.
     */
    if (b3_barcode_active(&g_scan))
    {
        base_speed =
            B3_SCAN_SPEED_MM_S;
    }


    if (context.terrain_unstable)
    {
        base_speed =
            B3_TERRAIN_SPEED_MM_S;
    }


    /*
     * Process line-following sensors.
     */
    output.line =
        b3_line_step(
            &g_line,
            sample.left_black,
            sample.right_black,
            now,
            base_speed
        );


    /*
     * Barcode processing is allowed only when:
     *
     * - context is fresh
     * - scanning has been requested
     * - stop has not been requested
     * - terrain is stable
     */
    scan_enabled =
        fresh &&
        context.scan_enabled &&
        !context.stop_requested &&
        !context.terrain_unstable;


    /*
     * Reset motion statistics whenever
     * no barcode is active.
     */
    if (!b3_barcode_active(&g_scan))
    {
        g_min_speed =
            INT32_MAX;

        g_max_speed =
            0;

        g_motion_good =
            true;
    }


    record_motion(
        &context,
        fresh &&
        (service_gap <=
         B3_OUTPUT_MAX_AGE_US)
    );


    /*
     * If scanning is disabled, discard queued
     * barcode data and reset the decoder.
     */
    if (!scan_enabled)
    {
        b3_ir_flush();

        b3_barcode_abort(
            &g_scan,
            sample.barcode
        );
    }
    else
    {
        /*
         * Consume queued barcode edges.
         *
         * Processing is bounded so a noisy IR sensor
         * cannot starve the rest of the task.
         */
        for (i = 0U;
             i < B3_EDGE_QUEUE_SIZE;
             ++i)
        {
            status =
                b3_ir_pop(
                    &edge
                );


            if (B3_IR_EMPTY == status)
            {
                break;
            }


            /*
             * Lost edge timing means the partial
             * barcode is no longer trustworthy.
             */
            if (B3_IR_DISCONTINUITY == status)
            {
                sample =
                    b3_ir_read();


                b3_barcode_abort(
                    &g_scan,
                    sample.barcode
                );


                break;
            }


            /*
             * Feed the valid edge into the
             * barcode state machine.
             */
            if (b3_barcode_edge(
                    &g_scan,
                    edge,
                    &decoded))
            {
                publish_barcode(
                    &output,
                    decoded,
                    &context,
                    fresh
                );
            }
        }


        /*
         * Once the edge queue is empty,
         * poll for completion of the trailing
         * white quiet zone.
         */
        if ((B3_IR_EMPTY == status) &&
            b3_ir_read_when_idle(
                &sample
            ))
        {
            if (b3_barcode_poll(
                    &g_scan,
                    sample.barcode,
                    &decoded))
            {
                publish_barcode(
                    &output,
                    decoded,
                    &context,
                    fresh
                );
            }
        }
    }


    /*
     * Junction candidate handling: IR_L, IR_R and IR_B
     * all black for long enough means a wide crossing patch.
     */
    if (sample.barcode.black &&
        output.line.left_black &&
        output.line.right_black &&
        !g_side_black)
    {
        g_side_black_since =
            now;

        g_side_min_speed =
            INT32_MAX;

        g_side_motion_good =
            true;
    }


    g_side_black =
        sample.barcode.black &&
        output.line.left_black &&
        output.line.right_black;


    if (g_side_black)
    {
        int32_t speed;


        speed =
            ((int32_t)context.left_measured_mm_s +
             context.right_measured_mm_s) /
            2;


        if (speed < g_side_min_speed)
        {
            g_side_min_speed =
                speed;
        }


        if (!fresh ||
            !context.motion_valid ||
            context.stop_requested ||
            context.terrain_unstable ||
            (speed <
             B3_JUNCTION_MIN_SPEED_MM_S) ||
            (service_gap >
             B3_OUTPUT_MAX_AGE_US))
        {
            g_side_motion_good =
                false;
        }
    }


    output.scan_active =
        b3_barcode_active(
            &g_scan
        );


    /*
     * Detect possible large black patch / junction.
     */
    output.junction_candidate =
        g_side_black &&
        g_side_motion_good &&
        (
            (!output.scan_active) ||
            (
                g_scan.collecting &&
                (0U == g_scan.count)
            )
        ) &&
        (
            (uint64_t)
            (now - g_side_black_since) *
            (uint32_t)g_side_min_speed >=
            (uint64_t)
            B3_JUNCTION_MIN_PATCH_UM *
            1000U
        );


    output.control_active =
        fresh &&
        context.follow_enabled;


    output.stop_required =
        !fresh ||
        context.stop_requested ||
        output.line.stop ||
        g_event_fault;


    if (!output.control_active ||
        output.stop_required)
    {
        output.line.left_mm_s =
            0;

        output.line.right_mm_s =
            0;
    }


    output.time_us =
        sample.barcode.time_us;


    output.edge_faults =
        b3_ir_fault_count();


    output.rejected_scans =
        g_scan.rejected;


    output.dropped_events =
        g_dropped_events;


    output.max_service_gap_us =
        g_max_service_gap_us;


    /*
     * Publish updated snapshot.
     */
    b3_port_lock(&saved);

    g_snapshot =
        output;

    b3_port_unlock(saved);
}


/*
 * Buddy3 worker task.
 */
static void sensor_task(
    INT start_code,
    void *p_extended)
{
    ER status;

    (void)start_code;
    (void)p_extended;


    for (;;)
    {
        service_once();

        /*
         * Sleep until the cyclic handler wakes us (task context).
         */
        status =
            tk_slp_tsk(
                TMO_FEVR
            );


        if (E_OK != status)
        {
            b3_irq_state_t saved;


            b3_port_lock(&saved);


            g_snapshot.task_error =
                status;

            g_snapshot.stop_required =
                true;

            g_snapshot.line.left_mm_s =
                0;

            g_snapshot.line.right_mm_s =
                0;


            b3_port_unlock(saved);


            b3_ir_enable_barcode_irq(
                false
            );


            tk_ext_tsk();
        }
    }
}


/*
 * Cyclic handler: wake the worker every B3_SERVICE_DELAY_MS.
 */
static void service_cyc_handler(
    void *p_extended)
{
    (void)p_extended;

    (void)tk_wup_tsk(
        g_task_id
    );
}


/*
 * Start Buddy3 subsystem.
 */
ER buddy3_start(
    b3_position_reader_t position_reader)
{
    T_CTSK create =
        (T_CTSK){0};

    ID task_id;

    ER status;

    b3_ir_sample_t sample;


    if (g_started)
    {
        return E_OBJ;
    }


    /*
     * Initialise IR subsystem.
     */
    status =
        b3_ir_init(
            position_reader
        );


    if (E_OK != status)
    {
        return status;
    }


    /*
     * Capture initial sensor state.
     */
    sample =
        b3_ir_read();


    b3_line_init(
        &g_line,
        sample.barcode.time_us
    );


    b3_barcode_init(
        &g_scan,
        sample.barcode
    );


    /*
     * Reset output/context structures.
     */
    g_snapshot =
        (b3_snapshot_t){0};

    g_context =
        (b3_context_t){0};


    g_snapshot.stop_required =
        true;

    g_snapshot.line.stop =
        true;


    g_event_head =
        0U;

    g_event_tail =
        0U;


    g_dropped_events =
        0U;

    g_event_fault =
        false;


    g_sequence =
        0U;


    g_side_black =
        false;


    g_last_service_us =
        sample.barcode.time_us;

    g_max_service_gap_us =
        0U;


    /*
     * Create Buddy3 worker task.
     */
#if TK_SUPPORT_SMP

    create.tskatr =
        TA_HLNG |
        TA_RNG3 |
        TA_ASSPRC;

    create.assprc =
        TP_PRC1;

#else

    create.tskatr =
        TA_HLNG |
        TA_RNG3;

#endif


    create.task =
        (FP)sensor_task;

    create.itskpri =
        B3_TASK_PRIORITY;

    create.stksz =
        (SZ)B3_TASK_STACK_BYTES;

    create.bufptr =
        0;


    task_id =
        tk_cre_tsk(
            &create
        );


    if (task_id < 0)
    {
        return task_id;
    }


    g_started =
        true;


    /*
     * Enable barcode GPIO interrupt.
     */
    b3_ir_enable_barcode_irq(
        true
    );


    /*
     * Start Buddy3 worker task.
     */
    status =
        tk_sta_tsk(
            task_id,
            0
        );


    if (E_OK == status)
    {
        T_CCYC cyclic =
            (T_CCYC){0};


        g_task_id =
            task_id;

        cyclic.cycatr =
            TA_HLNG |
            TA_STA;

        cyclic.cychdr =
            (FP)service_cyc_handler;

        cyclic.cyctim =
            B3_SERVICE_DELAY_MS;

        cyclic.cycphs =
            B3_SERVICE_DELAY_MS;


        g_cyc_id =
            tk_cre_cyc(
                &cyclic
            );


        if (g_cyc_id < 0)
        {
            status =
                g_cyc_id;

            (void)tk_ter_tsk(
                task_id
            );
        }
    }


    if (E_OK != status)
    {
        b3_ir_enable_barcode_irq(
            false
        );

        (void)tk_del_tsk(
            task_id
        );

        g_started =
            false;
    }


    return status;
}