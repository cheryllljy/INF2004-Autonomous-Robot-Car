/* Original decoder. Standard Code 39 symbol encodings; see docs/SOURCES.md. */
#include "b3_code39.h"
#include <stddef.h>

/* const data is linked into the Pico firmware image, normally flash .rodata. */
static const char g_symbols[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
static const uint16_t g_patterns[] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x0A8U,
    0x0A2U, 0x08AU, 0x02AU, 0x094U
};
_Static_assert(sizeof(g_patterns) / sizeof(g_patterns[0]) == sizeof(g_symbols) - 1U,
               "Code 39 table lengths must agree");

bool b3_code39_pattern(char symbol, uint16_t *p_pattern)
{
    size_t i;
    if (NULL == p_pattern)
    {
        return false;
    }
    for (i = 0U; i < sizeof(g_patterns) / sizeof(g_patterns[0]); ++i)
    {
        if (symbol == g_symbols[i])
        {
            *p_pattern = g_patterns[i];
            return true;
        }
    }
    return false;
}

/*
 * Threshold skew: if the sensor threshold sits nearer white or black, every
 * bar reads longer or shorter by the same amount and every space the
 * opposite way (seen on the car 2026-10-30: narrow bars 15 ms, narrow spaces
 * 40 ms). Every Code 39 character has at least one narrow bar and one narrow
 * space, so half the difference between the shortest space and the shortest
 * bar estimates that shift. Elements 0, 2, 4, 6, 8 are bars.
 */
static int64_t skew_of(const uint32_t widths[9])
{
    uint32_t min_bar = UINT32_MAX;
    uint32_t min_space = UINT32_MAX;
    size_t i;
    for (i = 0U; i < 9U; ++i)
    {
        if (0U == (i % 2U))
        {
            if (widths[i] < min_bar)
            {
                min_bar = widths[i];
            }
        }
        else if (widths[i] < min_space)
        {
            min_space = widths[i];
        }
    }
    return ((int64_t)min_space - (int64_t)min_bar) / 2;
}

static bool decode_symbol(const uint32_t raw[9], char *p_symbol,
                          uint32_t *p_narrow, int64_t *p_skew)
{
    uint32_t widths[9];
    int64_t skew;
    uint32_t sorted[9];
    uint64_t narrow_sum = 0U;
    uint64_t wide_sum = 0U;
    uint16_t pattern = 0U;
    size_t i;
    size_t j;
    for (i = 0U; i < 9U; ++i)
    {
        if ((0U == raw[i]) || (raw[i] > B3_MAX_ENCODER_RUN_TICKS))
        {
            return false;
        }
    }
    skew = skew_of(raw);
    for (i = 0U; i < 9U; ++i)
    {
        int64_t w = (0U == (i % 2U)) ? ((int64_t)raw[i] + skew)
                                     : ((int64_t)raw[i] - skew);
        if (w <= 0)
        {
            return false;
        }
        widths[i] = (uint32_t)w;
        sorted[i] = widths[i];
    }
    *p_skew = skew;
    /* Fixed nine-element insertion sort. Never called from an ISR. */
    for (i = 1U; i < 9U; ++i)
    {
        uint32_t value = sorted[i];
        j = i;
        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    for (i = 0U; i < 6U; ++i)
    {
        narrow_sum += sorted[i];
    }
    for (i = 6U; i < 9U; ++i)
    {
        wide_sum += sorted[i];
    }
    
    /* Require two clearly separated width groups.
    *
    * A Code 39 character contains:
    *   6 narrow elements
    *   3 wide elements
    *
    * Therefore:
    *   average_wide / average_narrow
    * = (wide_sum / 3) / (narrow_sum / 6)
    * = (2 * wide_sum) / narrow_sum
    *
    * The supplied school barcode is approximately 3:1.
    */
    if (((uint64_t)sorted[6] * 100U < (uint64_t)sorted[5] * 150U) ||
        ((uint64_t)sorted[5] * 100U > (uint64_t)sorted[0] * 180U) ||
        ((uint64_t)sorted[8] * 100U > (uint64_t)sorted[6] * 180U) ||
        (wide_sum * 200U <
            narrow_sum * B3_WIDE_NARROW_RATIO_MIN_PCT) ||
        (wide_sum * 200U >
            narrow_sum * B3_WIDE_NARROW_RATIO_MAX_PCT))
    {
        return false;
    }

    for (i = 0U; i < 9U; ++i)
    {
        pattern = (uint16_t)(pattern << 1U);
        if (widths[i] >= sorted[6])
        {
            pattern = (uint16_t)(pattern | 1U);
        }
    }
    for (i = 0U; i < sizeof(g_patterns) / sizeof(g_patterns[0]); ++i)
    {
        if (pattern == g_patterns[i])
        {
            *p_symbol = g_symbols[i];
            *p_narrow = (uint32_t)((narrow_sum + 3U) / 6U);
            return true;
        }
    }
    return false;
}

static bool decode_forward(const uint32_t widths[B3_BARCODE_RUNS],
                           b3_code39_result_t *p_result)
{
    char first;
    char middle;
    char last;
    uint32_t n_first;
    uint32_t n_middle;
    uint32_t n_last;
    int64_t s_first;
    int64_t s_middle;
    int64_t s_last;
    int64_t gap;
    int64_t gap_reference;
    if (!decode_symbol(&widths[0], &first, &n_first, &s_first) ||
        !decode_symbol(&widths[10], &middle, &n_middle, &s_middle) ||
        !decode_symbol(&widths[20], &last, &n_last, &s_last) ||
        ('*' != first) || ('*' != last) || ('*' == middle))
    {
        return false;
    }
    /* Gaps are spaces, so they carry the same skew as their neighbours. */
    gap = (int64_t)widths[9] - (s_first + s_middle) / 2;
    gap_reference = ((int64_t)n_first + n_middle) / 2;
    if ((gap * 2 < gap_reference) || (gap > gap_reference * 2))
    {
        return false;
    }
    gap = (int64_t)widths[19] - (s_middle + s_last) / 2;
    gap_reference = ((int64_t)n_middle + n_last) / 2;
    if ((gap * 2 < gap_reference) || (gap > gap_reference * 2))
    {
        return false;
    }
    p_result->symbol = middle;
    p_result->first_narrow = n_first;
    p_result->last_narrow = n_last;
    return true;
}

bool b3_code39_decode(const uint32_t widths[B3_BARCODE_RUNS],
                      b3_code39_result_t *p_result)
{
    uint32_t reverse[B3_BARCODE_RUNS];
    size_t i;
    uint32_t temporary;
    if ((NULL == widths) || (NULL == p_result))
    {
        return false;
    }
    if (decode_forward(widths, p_result))
    {
        p_result->reversed = false;
        return true;
    }
    for (i = 0U; i < B3_BARCODE_RUNS; ++i)
    {
        reverse[i] = widths[B3_BARCODE_RUNS - 1U - i];
    }
    if (!decode_forward(reverse, p_result))
    {
        return false;
    }
    p_result->reversed = true;
    temporary = p_result->first_narrow;
    p_result->first_narrow = p_result->last_narrow;
    p_result->last_narrow = temporary;
    return true;
}
