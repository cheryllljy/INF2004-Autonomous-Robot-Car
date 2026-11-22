#ifndef B3_BARCODE_H
#define B3_BARCODE_H

#include "b3_code39.h"

typedef struct
{
    uint32_t time_us;
    uint32_t position_ticks;
    bool position_valid;
    bool black;
    uint16_t speed_mm_s; /* measured car speed at this edge, 0 = unknown */
} b3_edge_t;

typedef struct
{
    char symbol;
    bool reversed;
    bool used_encoder;
    uint32_t first_edge_us;
    uint32_t finished_us;
} b3_barcode_result_t;

/* Why the last scan was rejected (printed by the VC over MQTT). */
typedef enum
{
    B3_REJECT_NONE = 0,
    /* two edges of the same colour in a row (missed edge) */
    B3_REJECT_SAME_LEVEL,
    /* element shorter than B3_MIN_EDGE_US (noise/bounce) */
    B3_REJECT_TOO_SHORT,
    B3_REJECT_TOO_LONG, /* element longer than B3_MAX_ELEMENT_US */
    /* element > B3_MAX_ELEMENT_RATIO x shortest in this scan */
    B3_REJECT_RATIO,
    B3_REJECT_SCAN_TIMEOUT, /* whole scan longer than B3_MAX_SCAN_US */
    /* 29 elements seen but not a valid Code 39 pattern */
    B3_REJECT_DECODE,
    B3_REJECT_LEADING_QUIET, /* white before the barcode < 10 x narrow */
    /* black came back before the white after it was long enough */
    B3_REJECT_TRAILING_QUIET
} b3_reject_reason_t;

/* Why a scan under way ended without a reject (printed by the VC). */
typedef enum
{
    B3_DROP_NONE = 0,
    /* a flick that could not be undone: restarted from it */
    B3_DROP_FLICKER,
    B3_DROP_SCAN_OFF, /* the VehicleController turned scanning off */
    /* the VehicleController's context was older than
     * B3_FEEDBACK_MAX_AGE_US */
    B3_DROP_LATE,
    B3_DROP_STOP,     /* stop requested */
    B3_DROP_TERRAIN,  /* uneven ground reported by the IMU */
    B3_DROP_OVERFLOW  /* the edge queue was full, edges were lost */
} b3_drop_reason_t;

typedef struct
{
    uint32_t time_widths[B3_BARCODE_RUNS];
    uint32_t tick_widths[B3_BARCODE_RUNS];
    b3_edge_t last;
    b3_edge_t prev; /* edge before last, to undo a sensor blip */
    uint32_t glitches;
    /* scans decoded only after trimming the outer bars */ /* blips shorter than B3_MIN_EDGE_US absorbed */
    uint32_t trimmed;
    /* scans decoded from the time widths after the
     * encoder widths failed */
    uint32_t timed_fallbacks;
    bool prev_valid;
    /* time of a black edge seen in the trailing white */
    uint32_t glitch_us;
    /* that edge is waiting to be confirmed or dropped */
    bool glitch;
    b3_code39_result_t decoded;
    /* the white edge the leading white started from, so a
     * blip on the first bar can hand the white back instead of restarting
     * it (that restart caused reason 7 on the car). */
    b3_edge_t lead_from;
    uint32_t leading_us;
    uint32_t leading_ticks;
    uint32_t started_us;
    uint32_t required_trailing;
    uint32_t min_time_width;
    uint32_t rejected;
    /* duration of the element that caused the last reject */
    uint32_t reject_element_us;
    uint8_t reject_reason; /* b3_reject_reason_t of the last reject */
    /* elements measured (of B3_BARCODE_RUNS) at the last reject */
    uint8_t reject_count;
    /* widths (us) of the elements measured at the last reject, 0 after */
    uint32_t reject_widths[B3_BARCODE_RUNS];
    /* scans dropped without a reject, why the last one was (a
     * b3_drop_reason_t), its elements and, for B3_DROP_LATE, the age of
     * the context in ms */
    uint32_t dropped;
    uint8_t drop_reason;
    uint8_t drop_count;
    uint32_t drop_info;
    /* last LEADING_QUIET reject: white measured before the
     * first bar and the white needed (B3_QUIET_NARROW_MULTIPLE x narrow), in
     * encoder ticks if reject_encoder, else in us. */
    uint32_t reject_lead;
    uint32_t reject_need;
    bool reject_encoder;
    uint8_t count;
    bool collecting;
    bool pending;
    bool positions_valid;
    bool used_encoder;
} b3_barcode_t;

/* Reset after lost edges, manoeuvres or a scan interrupted by stopping. */
void b3_barcode_init(b3_barcode_t *p_scan, b3_edge_t initial);
void b3_barcode_abort(b3_barcode_t *p_scan, b3_edge_t initial);
/* Abort, and count it as a dropped scan when one was under way. */
void b3_barcode_drop(b3_barcode_t *p_scan, b3_edge_t initial,
                     b3_drop_reason_t reason, uint32_t info);
bool b3_barcode_edge(b3_barcode_t *p_scan, b3_edge_t edge,
                     b3_barcode_result_t *p_result);
bool b3_barcode_poll(b3_barcode_t *p_scan, b3_edge_t current,
                     b3_barcode_result_t *p_result);
bool b3_barcode_active(const b3_barcode_t *p_scan);

#endif
