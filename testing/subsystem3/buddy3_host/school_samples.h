/* Independent run lengths extracted from the supplied school PDF.
 * Raster row at 25% height, away from the printed A/Z letters.
 * Units are pixels at 2x PDF scale; scan order is left to right. */
#ifndef SCHOOL_SAMPLES_H
#define SCHOOL_SAMPLES_H
#include <stdint.h>
static const uint32_t g_school_runs[4][29] = {
    {39U, 119U, 40U, 40U, 118U, 40U, 119U, 40U, 39U, 40U, 119U, 39U, 40U, 40U, 39U, 119U, 40U, 39U, 119U, 40U, 39U, 119U, 40U, 40U, 118U, 40U, 119U, 40U, 39U},
    {20U, 60U, 20U, 20U, 59U, 20U, 60U, 20U, 20U, 20U, 59U, 20U, 20U, 20U, 20U, 60U, 20U, 19U, 60U, 20U, 20U, 60U, 20U, 19U, 60U, 20U, 60U, 20U, 20U},
    {39U, 119U, 40U, 40U, 118U, 40U, 119U, 40U, 39U, 40U, 39U, 119U, 119U, 40U, 119U, 39U, 40U, 40U, 39U, 40U, 39U, 119U, 40U, 40U, 118U, 40U, 119U, 40U, 39U},
    {20U, 60U, 20U, 20U, 59U, 20U, 60U, 20U, 20U, 20U, 20U, 59U, 60U, 20U, 59U, 20U, 20U, 20U, 20U, 20U, 20U, 59U, 20U, 20U, 60U, 20U, 60U, 19U, 20U},
};
static const char g_school_symbols[4] = {'A', 'A', 'Z', 'Z'};
#endif
