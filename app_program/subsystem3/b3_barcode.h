#ifndef B3_BARCODE_H
#define B3_BARCODE_H

#include "b3_code39.h"

typedef struct
{
    uint32_t time_us;
    uint32_t position_ticks;
    bool position_valid;
    bool black;
} b3_edge_t;

typedef struct
{
    char symbol;
    bool reversed;
    bool used_encoder;
    uint32_t first_edge_us;
    uint32_t finished_us;
} b3_barcode_result_t;

typedef struct
{
    uint32_t time_widths[B3_BARCODE_RUNS];
    uint32_t tick_widths[B3_BARCODE_RUNS];
    b3_edge_t last;
    b3_code39_result_t decoded;
    uint32_t leading_us;
    uint32_t leading_ticks;
    uint32_t started_us;
    uint32_t required_trailing;
    uint32_t rejected;
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
