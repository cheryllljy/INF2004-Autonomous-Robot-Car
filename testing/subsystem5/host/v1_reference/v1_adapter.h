/* HOST TEST ONLY. Plain-type bridge to the v1 geometry, so the v1 and
 * v2 headers never meet in one translation unit. */
#ifndef V1_ADAPTER_H
#define V1_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool     valid;
    uint16_t nearest_mm;
    int16_t  bearing_deg;
    uint16_t width_mm;
    int16_t  left_edge_deg;
    int16_t  right_edge_deg;
    uint16_t gap_left_mm;
    uint16_t gap_right_mm;
    uint16_t swing_left_mm;
    uint16_t swing_right_mm;
    uint8_t  confidence_pct;
    uint8_t  sample_count;
    int      action;
} v1_out_t;

void v1_build(const int16_t *p_bearing, const uint16_t *p_range,
              uint8_t count, v1_out_t *p_out);

/* sizeof() of the v1 data types, for the CR1 size table. */
unsigned v1_sizeof_sample(void);
unsigned v1_sizeof_profile(void);
unsigned v1_sizeof_health(void);
unsigned v1_sizeof_us_result(void);
unsigned v1_sizeof_us_stats(void);

#endif
