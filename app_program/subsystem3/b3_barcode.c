#include "b3_barcode.h"
#include <stddef.h>


/*
 * Initialise barcode decoder.
 */
void b3_barcode_init(
    b3_barcode_t *p_scan,
    b3_edge_t initial)
{
    /*
     * Reset all barcode state.
     */
    *p_scan =
        (b3_barcode_t){0};


    /*
     * Remember initial sensor state.
     *
     * Normally this should be WHITE before
     * beginning a barcode scan.
     */
    p_scan->last =
        initial;
}


/*
 * Abort/reset current barcode attempt.
 */
void b3_barcode_abort(
    b3_barcode_t *p_scan,
    b3_edge_t initial)
{
    p_scan->last =
        initial;

    p_scan->count =
        0U;

    p_scan->collecting =
        false;

    p_scan->pending =
        false;

    p_scan->positions_valid =
        false;
}


/*
 * Return true while a barcode is currently
 * being collected or waiting for trailing quiet.
 */
bool b3_barcode_active(
    const b3_barcode_t *p_scan)
{
    return
        p_scan->collecting ||
        p_scan->pending;
}


/*
 * Reject the current barcode attempt.
 */
static void reject(
    b3_barcode_t *p_scan,
    b3_edge_t edge)
{
    ++p_scan->rejected;


    b3_barcode_abort(
        p_scan,
        edge
    );
}


/*
 * Check whether the trailing white quiet zone
 * is long enough to finish the barcode.
 */
static bool finish(
    b3_barcode_t *p_scan,
    b3_edge_t current,
    b3_barcode_result_t *p_result)
{
    uint32_t quiet;


    if (!p_scan->pending)
    {
        return false;
    }


    /*
     * Encoder-based quiet-zone measurement.
     */
    if (p_scan->used_encoder)
    {
        if (!current.position_valid)
        {
            return false;
        }


        quiet =
            current.position_ticks -
            p_scan->last.position_ticks;


        if (quiet >
            B3_MAX_ENCODER_RUN_TICKS)
        {
            return false;
        }
    }


    /*
     * Time-based quiet-zone measurement.
     */
    else
    {
        quiet =
            current.time_us -
            p_scan->last.time_us;
    }


    /*
     * Not enough trailing white yet.
     */
    if (quiet <
        p_scan->required_trailing)
    {
        return false;
    }


    /*
     * Complete result.
     */
    p_result->symbol =
        p_scan->decoded.symbol;

    p_result->reversed =
        p_scan->decoded.reversed;

    p_result->used_encoder =
        p_scan->used_encoder;

    p_result->first_edge_us =
        p_scan->started_us;

    p_result->finished_us =
        current.time_us;


    p_scan->pending =
        false;


    return true;
}


/*
 * Once all B3_BARCODE_RUNS widths have been
 * collected, try to decode Code 39.
 */
static bool prepare_result(
    b3_barcode_t *p_scan)
{
    bool enough_ticks;

    uint32_t leading;

    size_t i;


    enough_ticks =
        p_scan->positions_valid;


    /*
     * Encoder widths are only usable if every
     * element has enough encoder resolution.
     */
    for (i = 0U;
         i < B3_BARCODE_RUNS;
         ++i)
    {
        if (p_scan->tick_widths[i] <
            B3_MIN_NARROW_ENCODER_TICKS)
        {
            enough_ticks =
                false;
        }
    }


    p_scan->used_encoder =
        enough_ticks;


    /*
     * Decode either:
     *
     * encoder widths
     *
     * OR
     *
     * time widths
     */
    if (!b3_code39_decode(
            enough_ticks ?
                p_scan->tick_widths :
                p_scan->time_widths,
            &p_scan->decoded))
    {
        return false;
    }


    /*
     * Check leading quiet zone.
     */
    leading =
        enough_ticks ?
            p_scan->leading_ticks :
            p_scan->leading_us;


    if ((uint64_t)leading <
        (uint64_t)
        p_scan->decoded.first_narrow *
        B3_QUIET_NARROW_MULTIPLE)
    {
        return false;
    }


    /*
     * Required trailing quiet zone is based
     * on the decoded narrow element.
     */
    p_scan->required_trailing =
        p_scan->decoded.last_narrow *
        B3_QUIET_NARROW_MULTIPLE;


    p_scan->pending =
        true;

    p_scan->collecting =
        false;


    return true;
}


/*
 * Time mode only: an element far longer than the shortest one seen in this
 * scan cannot be part of the same barcode, whatever its printed size.
 * Encoder mode is skipped because wheel speed may vary between elements.
 */
static bool too_long_for_scan(
    const b3_barcode_t *p_scan,
    uint32_t elapsed)
{
    return
        !p_scan->positions_valid &&
        (p_scan->count >= 2U) &&
        ((uint64_t)elapsed >
         (uint64_t)p_scan->min_time_width *
         B3_MAX_ELEMENT_RATIO);
}


/*
 * Process one barcode GPIO edge.
 */
bool b3_barcode_edge(
    b3_barcode_t *p_scan,
    b3_edge_t edge,
    b3_barcode_result_t *p_result)
{
    uint32_t elapsed;

    bool emitted;


    elapsed =
        edge.time_us -
        p_scan->last.time_us;


    emitted =
        false;


    /*
     * Consecutive edges should alternate:
     *
     * BLACK -> WHITE -> BLACK -> WHITE ...
     *
     * Receiving the same state twice means
     * we lost timing continuity or received
     * duplicate/noisy transitions.
     */
    if (edge.black ==
        p_scan->last.black)
    {
        reject(
            p_scan,
            edge
        );

        return false;
    }


    /*
     * Barcode pattern has already been decoded.
     *
     * We are waiting for sufficient trailing
     * white quiet space.
     */
    if (p_scan->pending)
    {
        /*
         * Incoming black edge ends the trailing
         * white quiet zone.
         */
        if ((edge.time_us -
             p_scan->started_us) <=
            B3_MAX_SCAN_US)
        {
            emitted =
                finish(
                    p_scan,
                    edge,
                    p_result
                );
        }


        if (!emitted)
        {
            ++p_scan->rejected;
        }


        p_scan->pending =
            false;
    }


    /*
     * Already collecting barcode widths.
     */
    if (p_scan->collecting)
    {
        uint32_t ticks;


        ticks =
            edge.position_ticks -
            p_scan->last.position_ticks;


        /*
         * Reject impossible/noisy timing.
         */
        if ((elapsed <
             B3_MIN_EDGE_US) ||

            (elapsed >
             B3_MAX_ELEMENT_US) ||

            ((edge.time_us -
              p_scan->started_us) >
             B3_MAX_SCAN_US) ||

            too_long_for_scan(
                p_scan,
                elapsed
            ))
        {
            reject(
                p_scan,
                edge
            );

            return emitted;
        }


        if ((0U == p_scan->count) ||
            (elapsed < p_scan->min_time_width))
        {
            p_scan->min_time_width =
                elapsed;
        }


        /*
         * Store width of previous black/white run.
         */
        p_scan->time_widths[
            p_scan->count
        ] =
            elapsed;


        p_scan->tick_widths[
            p_scan->count
        ] =
            ticks;


        /*
         * Encoder measurements remain valid
         * only if every edge has a valid position.
         */
        p_scan->positions_valid =
            p_scan->positions_valid &&
            edge.position_valid &&
            (ticks <=
             B3_MAX_ENCODER_RUN_TICKS);


        ++p_scan->count;


        p_scan->last =
            edge;


        /*
         * Full barcode frame collected.
         */
        if (B3_BARCODE_RUNS ==
            p_scan->count)
        {
            if (!prepare_result(
                    p_scan))
            {
                reject(
                    p_scan,
                    edge
                );
            }
        }


        return emitted;
    }


    /*
     * START CONDITION
     *
     * A barcode scan begins only when:
     *
     * 1. this new edge represents BLACK
     *
     * 2. the sensor had previously remained
     *    WHITE for at least B3_MIN_START_WHITE_US
     */
    if (edge.black &&
        (elapsed >=
         B3_MIN_START_WHITE_US))
    {
        p_scan->collecting =
            true;


        p_scan->count =
            0U;


        p_scan->started_us =
            edge.time_us;


        p_scan->leading_us =
            elapsed;


        p_scan->leading_ticks =
            edge.position_ticks -
            p_scan->last.position_ticks;


        p_scan->positions_valid =
            p_scan->last.position_valid &&
            edge.position_valid &&
            (p_scan->leading_ticks <=
             B3_MAX_ENCODER_RUN_TICKS);
    }


    /*
     * Always remember latest valid edge.
     */
    p_scan->last =
        edge;


    return emitted;
}


/*
 * Poll barcode state when there are no queued
 * GPIO edges remaining.
 */
bool b3_barcode_poll(
    b3_barcode_t *p_scan,
    b3_edge_t current,
    b3_barcode_result_t *p_result)
{
    /*
     * If raw sensor level has changed, a queued
     * GPIO edge must be processed first.
     */
    if (current.black !=
        p_scan->last.black)
    {
        return false;
    }


    /*
     * Entire scan has taken too long.
     */
    if (b3_barcode_active(p_scan) &&
        ((current.time_us -
          p_scan->started_us) >
         B3_MAX_SCAN_US))
    {
        reject(
            p_scan,
            current
        );

        return false;
    }


    /*
     * Barcode decoded.
     *
     * Wait for trailing white quiet zone.
     */
    if (p_scan->pending)
    {
        return finish(
            p_scan,
            current,
            p_result
        );
    }


    /*
     * Individual black/white element lasted
     * too long.
     */
    if (p_scan->collecting &&
        (((current.time_us -
           p_scan->last.time_us) >
          B3_MAX_ELEMENT_US) ||
         too_long_for_scan(
             p_scan,
             current.time_us -
             p_scan->last.time_us)))
    {
        reject(
            p_scan,
            current
        );
    }


    return false;
}