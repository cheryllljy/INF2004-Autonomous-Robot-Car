#ifndef B3_CODE39_H
#define B3_CODE39_H

#include <stdbool.h>
#include <stdint.h>
#include "b3_config.h"

typedef struct
{
    char symbol;
    bool reversed;
    uint32_t first_narrow;
    uint32_t last_narrow;
} b3_code39_result_t;

/* All 43 basic Code 39 data symbols plus the '*' framing symbol. */
bool b3_code39_pattern(char symbol, uint16_t *p_pattern);
bool b3_code39_decode(const uint32_t widths[B3_BARCODE_RUNS],
                      b3_code39_result_t *p_result);

#endif
