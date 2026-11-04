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
    B3_REJECT_SAME_LEVEL,     /* two edges of the same colour in a row (missed edge) */
    B3_REJECT_TOO_SHORT,      /* element shorter than B3_MIN_EDGE_US (noise/bounce) */
    B3_REJECT_TOO_LONG,       /* element longer than B3_MAX_ELEMENT_US */
    B3_REJECT_RATIO,          /* element > B3_MAX_ELEMENT_RATIO x shortest in this scan */
    B3_REJECT_SCAN_TIMEOUT,   /* whole scan longer than B3_MAX_SCAN_US */
    B3_REJECT_DECODE,         /* 29 elements seen but not a valid Code 39 pattern */
    B3_REJECT_LEADING_QUIET,  /* white before the barcode < 10 x narrow */
    B3_REJECT_TRAILING_QUIET  /* black came back before the white after it was long enough */
} b3_reject_reason_t;

typedef struct
{
    uint32_t time_widths[B3_BARCODE_RUNS];
    uint32_t tick_widths[B3_BARCODE_RUNS];
    b3_edge_t last;
    b3_edge_t prev;             /* edge before last, to undo a sensor blip */
    uint32_t glitches;
    uint32_t trimmed;           /* scans decoded only after trimming the outer bars */          /* blips shorter than B3_MIN_EDGE_US absorbed */
    bool prev_valid;
    uint32_t glitch_us;         /* time of a black edge seen in the trailing white */
    bool glitch;                /* that edge is waiting to be confirmed or dropped */
    b3_code39_result_t decoded;
    uint32_t leading_us;
    uint32_t leading_ticks;
    uint32_t started_us;
    uint32_t required_trailing;
    uint32_t min_time_width;
    uint32_t rejected;
    uint32_t reject_element_us; /* duration of the element that caused the last reject */
    uint8_t reject_reason;      /* b3_reject_reason_t of the last reject */
    uint8_t reject_count;       /* elements measured (of B3_BARCODE_RUNS) at the last reject */
    uint32_t reject_widths[B3_BARCODE_RUNS]; /* widths (us) of the last DECODE reject */
    uint8_t count;
    bool collecting;
    bool pending;
    bool positions_valid;
    bool used_encoder;
} b3_barcode_t;

/* Reset after lost edges, manoeuvres or a scan interrupted by stopping. */
void b3_barcode_init(b3_barcode_t *p_scan, b3_edge_t initial);
void b3_barcode_abort(b3_barcode_t *p_scan, b3_edge_t initial);
bool b3_barcode_edge(b3_barcode_t *p_scan, b3_edge_t edge,
                     b3_barcode_result_t *p_result);
bool b3_barcode_poll(b3_barcode_t *p_scan, b3_edge_t current,
                     b3_barcode_result_t *p_result);
bool b3_barcode_active(const b3_barcode_t *p_scan);

#endif
