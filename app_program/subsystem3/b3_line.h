#ifndef B3_LINE_H
#define B3_LINE_H

#include <stdbool.h>
#include <stdint.h>
#include "b3_config.h"

/* Straddle layout: IR_L and IR_R sit either side of the line.
 *   both white  CENTRED   on track, drive straight
 *   IR_L black  LEFT      drifted right, line under IR_L: steer left
 *   IR_R black  RIGHT     drifted left, line under IR_R: steer right
 *   both black  CROSSING  a crossing line or junction patch: drive straight
 * LOST is no longer produced (both white is the normal state). */
typedef enum
{
    B3_LINE_CENTRED = 0,
    B3_LINE_LEFT,
    B3_LINE_RIGHT,
    B3_LINE_LOST,
    B3_LINE_CROSSING
} b3_line_state_t;

typedef struct
{
    bool stable;
    bool candidate;
    uint32_t changed_us;
} b3_filter_t;

typedef struct
{
    b3_filter_t left;
    b3_filter_t right;
    uint32_t last_seen_us;
    int8_t last_error;
    bool ever_seen;
} b3_line_t;

typedef struct
{
    b3_line_state_t state;
    int8_t error; /* -1 = line under IR_L (steer left); +1 = under IR_R (steer right); 0 = straight. */
    int16_t left_mm_s;
    int16_t right_mm_s;
    bool left_black;
    bool right_black;
    bool stop;
    bool recovery_needed;
} b3_line_result_t;

void b3_line_init(b3_line_t *p_line, uint32_t now_us);
b3_line_result_t b3_line_step(b3_line_t *p_line, bool left_black,
                              bool right_black, uint32_t now_us,
                              int16_t base_mm_s);

#endif
