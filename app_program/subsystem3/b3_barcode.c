#include "b3_barcode.h"
#include <stddef.h>


/*
 * Convert a time width to "time at B3_BASE_SPEED_MM_S" using the measured
 * speed, so slowing down during a scan does not stretch the later bars.
 * Speed 0 (unknown) leaves the width as it is.
 */
static uint32_t at_base_speed(uint32_t us, uint16_t speed_mm_s)
{
#if B3_SCAN_SPEED_COMPENSATE
    if (speed_mm_s > 0U)
    {
        return (uint32_t)(((uint64_t)us * speed_mm_s) /
                          (uint64_t)B3_BASE_SPEED_MM_S);
    }
#else
    (void)speed_mm_s;
#endif
    return us;
}


/* Initialise the barcode decoder. */
void b3_barcode_init(b3_barcode_t *p_scan, b3_edge_t initial)
{
    *p_scan = (b3_barcode_t){ 0 };


    /* initial sensor state, normally WHITE before a scan */
    p_scan->last = initial;
}


/* Abort/reset the current barcode attempt. */
void b3_barcode_abort(b3_barcode_t *p_scan, b3_edge_t initial)
{
    p_scan->last = initial;

    p_scan->count = 0U;

    p_scan->collecting = false;

    p_scan->pending = false;

    p_scan->positions_valid = false;

    p_scan->prev_valid = false;

    p_scan->glitch = false;
}


/* True while a barcode is being collected or waiting for trailing quiet. */
bool b3_barcode_active(const b3_barcode_t *p_scan)
{
    return p_scan->collecting || p_scan->pending;
}


/* Abort the scan, noting why for the VC's log when one was under way. */
void b3_barcode_drop(b3_barcode_t *p_scan, b3_edge_t initial,
                     b3_drop_reason_t reason, uint32_t info)
{
    if (b3_barcode_active(p_scan))
    {
        ++p_scan->dropped;

        p_scan->drop_reason = (uint8_t)reason;

        p_scan->drop_count = p_scan->count;

        p_scan->drop_info = info;
    }


    b3_barcode_abort(p_scan, initial);
}


/* Reject the current barcode attempt. */
static void note_reject(b3_barcode_t *p_scan, b3_reject_reason_t reason,
                        uint32_t element_us)
{
    ++p_scan->rejected;

    p_scan->reject_reason = (uint8_t)reason;

    p_scan->reject_count = p_scan->count;

    p_scan->reject_element_us = element_us;

    /* the elements measured so far, for the VC's log */
    {
        size_t k;

        for (k = 0U; k < B3_BARCODE_RUNS; ++k)
        {
            p_scan->reject_widths[k] =
                (k < p_scan->count) ? p_scan->time_widths[k] : 0U;
        }
    }
}


static void reject(b3_barcode_t *p_scan, b3_edge_t edge,
                   b3_reject_reason_t reason, uint32_t element_us)
{
    note_reject(p_scan, reason, element_us);


    b3_barcode_abort(p_scan, edge);
}


/* Finish the barcode once the trailing white quiet zone is long enough. */
static bool finish(b3_barcode_t *p_scan, b3_edge_t current,
                   b3_barcode_result_t *p_result)
{
    uint32_t quiet;


    if (!p_scan->pending)
    {
        return false;
    }


    /* encoder-based quiet-zone measurement */
    if (p_scan->used_encoder)
    {
        if (!current.position_valid)
        {
            return false;
        }


        quiet = current.position_ticks - p_scan->last.position_ticks;


        if (quiet > B3_MAX_ENCODER_RUN_TICKS)
        {
            return false;
        }
    }


    /* time-based quiet-zone measurement */
    else
    {
        quiet = at_base_speed(current.time_us - p_scan->last.time_us,
                              current.speed_mm_s);
    }


    if (quiet < p_scan->required_trailing)
    {
        return false;
    }


    p_result->symbol = p_scan->decoded.symbol;

    p_result->reversed = p_scan->decoded.reversed;

    p_result->used_encoder = p_scan->used_encoder;

    p_result->first_edge_us = p_scan->started_us;

    p_result->finished_us = current.time_us;


    p_scan->pending = false;


    return true;
}


/*
 * Decode one set of widths into p_scan->decoded, retrying with the outer
 * bars trimmed (B3_TRIM_OUTER_BARS).
 */
static bool decode_widths(b3_barcode_t *p_scan, const uint32_t *p_src)
{
    if (b3_code39_decode(p_src, &p_scan->decoded))
    {
        return true;
    }
#if B3_TRIM_OUTER_BARS
    {
        /*
         * The outer bars can read 3-4x too long while every other element
         * is fine. The first, ninth, 21st and last elements of the '*'
         * frames are narrow bars whichever way the car drives, so trim the
         * outer bar to its partner and decode again. The middle letter is
         * still checked in full.
         */
        uint32_t trimmed[B3_BARCODE_RUNS];
        size_t i;

        for (i = 0U; i < B3_BARCODE_RUNS; ++i)
        {
            trimmed[i] = p_src[i];
        }

        if (trimmed[0] > trimmed[8])
        {
            trimmed[0] = trimmed[8];
        }

        if (trimmed[B3_BARCODE_RUNS - 1U] > trimmed[B3_BARCODE_RUNS - 9U])
        {
            trimmed[B3_BARCODE_RUNS - 1U] = trimmed[B3_BARCODE_RUNS - 9U];
        }

        if (b3_code39_decode(trimmed, &p_scan->decoded))
        {
            ++p_scan->trimmed;
            return true;
        }
    }
#endif
    return false;
}


static b3_reject_reason_t prepare_result(b3_barcode_t *p_scan)
{
    bool enough_ticks;

    uint32_t leading;

    size_t i;


    enough_ticks = p_scan->positions_valid;


    /* encoder widths need enough encoder resolution in every element */
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        if (p_scan->tick_widths[i] < B3_MIN_NARROW_ENCODER_TICKS)
        {
            enough_ticks = false;
        }
    }


    p_scan->used_encoder = enough_ticks;


    if (!decode_widths(p_scan, enough_ticks ? p_scan->tick_widths
                                            : p_scan->time_widths))
    {
        /* Encoder distances can be off (a wheel missing ticks or
         * slipping). Code 39 is ratio-based, so try the time widths before
         * giving up; the quiet zones are then measured in time. */
        if (!enough_ticks || !decode_widths(p_scan, p_scan->time_widths))
        {
            return B3_REJECT_DECODE;
        }
        enough_ticks = false;
        p_scan->used_encoder = false;
        ++p_scan->timed_fallbacks;
    }


    /* leading quiet zone */
    leading = enough_ticks ? p_scan->leading_ticks : p_scan->leading_us;


    if ((uint64_t)leading <
        (uint64_t)p_scan->decoded.first_narrow * B3_QUIET_NARROW_MULTIPLE)
    {
        /* keep the numbers so the VC can print them */
        uint64_t need =
            (uint64_t)p_scan->decoded.first_narrow * B3_QUIET_NARROW_MULTIPLE;

        p_scan->reject_lead = leading;
        p_scan->reject_need = (need > UINT32_MAX) ? UINT32_MAX : (uint32_t)need;
        p_scan->reject_encoder = enough_ticks;

        return B3_REJECT_LEADING_QUIET;
    }


    /* Trailing quiet zone, from the decoded narrow element. With both '*'
     * read nothing can follow (Code 39 has no data after the stop '*'), so
     * none is needed. With one '*' spoiled the third symbol may be a
     * letter of a longer barcode, so the white is still needed. */
    p_scan->required_trailing =
        p_scan->decoded.both_stars
            ? 0U
            : (p_scan->decoded.last_narrow *
               B3_TRAILING_NARROW_MULTIPLE);


    p_scan->pending = true;

    p_scan->collecting = false;


    return B3_REJECT_NONE;
}


/*
 * Where to restart after a scan is given up while the sensor still reads
 * the level of the last edge. When that level is white, the white started
 * at the last edge: keep it, or the leading white of a barcode following
 * soon after looks too short.
 */
static b3_edge_t white_start(const b3_barcode_t *p_scan, b3_edge_t current)
{
    return p_scan->last.black ? current : p_scan->last;
}


/* A bar much shorter than every bar measured so far in this scan
 * (B3_SPECK_PCT): a dark speck in a gap, not a bar. Checked when the speck
 * ends, so the gap it split carries on from where it began, however short
 * the piece of gap before it was. Needs two bars first. Element 0 is a bar,
 * so bars have even numbers. */
static bool is_speck(const b3_barcode_t *p_scan, uint32_t width)
{
    uint32_t narrowest = UINT32_MAX;
    uint32_t seen = 0U;
    uint32_t k;

    if ((0U == B3_SPECK_PCT) || p_scan->pending ||
        (p_scan->count >= B3_BARCODE_RUNS) || (0U != (p_scan->count & 1U)))
    {
        return false;
    }
    for (k = 0U; k < p_scan->count; k += 2U)
    {
        if (p_scan->time_widths[k] < narrowest)
        {
            narrowest = p_scan->time_widths[k];
        }
        ++seen;
    }
    return (seen >= 2U) &&
           (((uint64_t)width * 100U) < ((uint64_t)narrowest * B3_SPECK_PCT));
}


static bool too_long_for_scan(const b3_barcode_t *p_scan, uint32_t elapsed)
{
    return !p_scan->positions_valid && (p_scan->count >= 2U) &&
           ((uint64_t)elapsed >
            (uint64_t)p_scan->min_time_width * B3_MAX_ELEMENT_RATIO);
}


/* Process one barcode GPIO edge. */
bool b3_barcode_edge(b3_barcode_t *p_scan, b3_edge_t edge,
                     b3_barcode_result_t *p_result)
{
    uint32_t elapsed;

    bool emitted;


    elapsed = edge.time_us - p_scan->last.time_us;


    emitted = false;


    /*
     * Sensor chatter: the DO comparator can flick for tens of microseconds
     * as a bar edge passes. A blip shorter than B3_MIN_EDGE_US is removed
     * so the element around it carries on, instead of rejecting the scan.
     */
    if (p_scan->glitch)
    {
        /*
         * A black edge came during the trailing white. If white is back
         * within B3_MIN_EDGE_US it was a blip: ignore both edges and keep
         * waiting for the quiet zone. Otherwise the black was real.
         */
        p_scan->glitch = false;

        if (!edge.black &&
            ((edge.time_us - p_scan->glitch_us) < B3_MIN_EDGE_US))
        {
            ++p_scan->glitches;

            return false;
        }

        if (p_scan->pending)
        {
            note_reject(p_scan, B3_REJECT_TRAILING_QUIET,
                        p_scan->glitch_us - p_scan->last.time_us);

            p_scan->pending = false;
        }

        /* Real black from glitch_us (a poll may already have finished). */
        p_scan->last.time_us = p_scan->glitch_us;

        p_scan->last.black = true;

        elapsed = edge.time_us - p_scan->last.time_us;
    }

    /*
     * Same colour as the last edge. The interrupt queues only changes of
     * level, so this comes from the decoder being reset from a pin read
     * just as an edge came in, or from a flick whose other edge was lost.
     * Not scanning: the level was already known, so the older edge is
     * kept (a white started there). Scanning: within B3_MIN_EDGE_US it is
     * a flick and the element carries on; later it is a missed edge and
     * the scan is rejected below.
     */
    if (edge.black == p_scan->last.black)
    {
        if (!b3_barcode_active(p_scan))
        {
            return false;
        }

        if (p_scan->collecting && (elapsed < B3_MIN_EDGE_US))
        {
            ++p_scan->glitches;

            return false;
        }
    }

    if (p_scan->collecting &&
        ((elapsed < B3_MIN_EDGE_US) ||
         is_speck(p_scan, at_base_speed(elapsed, edge.speed_mm_s))))
    {
        size_t k;

        ++p_scan->glitches;

        if ((0U == p_scan->count) && !edge.black && !p_scan->lead_from.black)
        {
            /* the "first bar" was only a blip in the white before the
             * barcode: the white carries on from where it really started,
             * or it looks too short */
            b3_barcode_abort(p_scan, p_scan->lead_from);

            return false;
        }

        if ((0U == p_scan->count) || !p_scan->prev_valid ||
            (edge.black != p_scan->prev.black))
        {
            /* A blip that cannot be undone: start again from this edge. */
            b3_barcode_drop(p_scan, edge, B3_DROP_FLICKER, 0U);

            return false;
        }

        /* Drop the blip's two edges: the element before it continues. */
        --p_scan->count;

        p_scan->last = p_scan->prev;

        p_scan->prev_valid = false;

        p_scan->min_time_width = UINT32_MAX;

        for (k = 0U; k < p_scan->count; ++k)
        {
            if (p_scan->time_widths[k] < p_scan->min_time_width)
            {
                p_scan->min_time_width = p_scan->time_widths[k];
            }
        }

        return false;
    }


    /* Edges must alternate black/white; the same state twice means lost
     * timing continuity or duplicate/noisy transitions. */
    if (edge.black == p_scan->last.black)
    {
        reject(p_scan, edge, B3_REJECT_SAME_LEVEL, elapsed);

        return false;
    }


    /* decoded already: waiting for the trailing white quiet zone, which
     * this black edge ends */
    if (p_scan->pending)
    {
        if ((edge.time_us - p_scan->started_us) <= B3_MAX_SCAN_US)
        {
            emitted = finish(p_scan, edge, p_result);
        }


        if (!emitted && ((edge.time_us - p_scan->started_us) <= B3_MAX_SCAN_US))
        {
            /* Wait for the next edge to tell a blip from real black. */
            p_scan->glitch = true;

            p_scan->glitch_us = edge.time_us;

            return false;
        }

        if (!emitted)
        {
            note_reject(p_scan,
                        ((edge.time_us - p_scan->started_us) > B3_MAX_SCAN_US)
                            ? B3_REJECT_SCAN_TIMEOUT
                            : B3_REJECT_TRAILING_QUIET,
                        elapsed);
        }


        p_scan->pending = false;
    }


    if (p_scan->collecting)
    {
        /* reject impossible/noisy timing */
        b3_reject_reason_t reason = B3_REJECT_NONE;

        if (elapsed < B3_MIN_EDGE_US)
        {
            reason = B3_REJECT_TOO_SHORT;
        }
        else if (elapsed > B3_MAX_ELEMENT_US)
        {
            reason = B3_REJECT_TOO_LONG;
        }
        else if ((edge.time_us - p_scan->started_us) > B3_MAX_SCAN_US)
        {
            reason = B3_REJECT_SCAN_TIMEOUT;
        }
        else if (too_long_for_scan(p_scan, elapsed))
        {
            reason = B3_REJECT_RATIO;
        }

        if ((B3_REJECT_NONE != reason) && edge.black && !p_scan->last.black)
        {
            /* the element that was too long is white, so it can be the
             * white before a barcode and this black edge its first bar:
             * keep the white's start and fall through to the start
             * condition below (see white_start()) */
            reject(p_scan, p_scan->last, reason, elapsed);
        }
        else if (B3_REJECT_NONE != reason)
        {
            reject(p_scan, edge, reason, elapsed);

            return emitted;
        }
        else
        {
            /* element accepted: stored below */
        }
    }


    if (p_scan->collecting)
    {
        uint32_t ticks;


        ticks = edge.position_ticks - p_scan->last.position_ticks;


        if ((0U == p_scan->count) || (elapsed < p_scan->min_time_width))
        {
            p_scan->min_time_width = elapsed;
        }


        p_scan->time_widths[p_scan->count] =
            at_base_speed(elapsed, edge.speed_mm_s);


        p_scan->tick_widths[p_scan->count] = ticks;


        /* encoder widths are valid only if every edge had a position */
        p_scan->positions_valid = p_scan->positions_valid &&
                                  edge.position_valid &&
                                  (ticks <= B3_MAX_ENCODER_RUN_TICKS);


        ++p_scan->count;


        p_scan->prev = p_scan->last;

        p_scan->prev_valid = true;

        p_scan->last = edge;


        if (B3_BARCODE_RUNS == p_scan->count)
        {
            b3_reject_reason_t why = prepare_result(p_scan);

            if (B3_REJECT_NONE != why)
            {
                reject(p_scan, edge, why, elapsed);
            }
        }


        return emitted;
    }


    /* Start condition: this edge is BLACK and the sensor stayed WHITE for
     * at least B3_MIN_START_WHITE_US before it. */
    if (edge.black && (elapsed >= B3_MIN_START_WHITE_US))
    {
        p_scan->collecting = true;


        p_scan->count = 0U;


        p_scan->started_us = edge.time_us;


        /* where this white began (see the blip check) */
        p_scan->lead_from = p_scan->last;


        p_scan->leading_us = at_base_speed(elapsed, edge.speed_mm_s);


        p_scan->leading_ticks =
            edge.position_ticks - p_scan->last.position_ticks;


        p_scan->positions_valid =
            p_scan->last.position_valid && edge.position_valid &&
            (p_scan->leading_ticks <= B3_MAX_ENCODER_RUN_TICKS);
    }


    p_scan->last = edge;


    return emitted;
}


/* Poll barcode state when no queued GPIO edges remain. */
bool b3_barcode_poll(b3_barcode_t *p_scan, b3_edge_t current,
                     b3_barcode_result_t *p_result)
{
    /* a changed raw level means a queued GPIO edge must be handled first */
    if (current.black != p_scan->last.black)
    {
        return false;
    }


    /* whole scan took too long */
    if (b3_barcode_active(p_scan) &&
        ((current.time_us - p_scan->started_us) > B3_MAX_SCAN_US))
    {
        reject(p_scan, white_start(p_scan, current), B3_REJECT_SCAN_TIMEOUT,
               current.time_us - p_scan->last.time_us);

        return false;
    }


    /* decoded: wait for the trailing white quiet zone */
    if (p_scan->pending)
    {
        return finish(p_scan, current, p_result);
    }


    /* one black/white element lasted too long */
    if (p_scan->collecting &&
        (((current.time_us - p_scan->last.time_us) > B3_MAX_ELEMENT_US) ||
         too_long_for_scan(p_scan, current.time_us - p_scan->last.time_us)))
    {
        uint32_t held = current.time_us - p_scan->last.time_us;

        reject(p_scan, white_start(p_scan, current),
               (held > B3_MAX_ELEMENT_US) ? B3_REJECT_TOO_LONG
                                          : B3_REJECT_RATIO,
               held);
    }


    return false;
}
