/**
 * @file    s5_pack.h
 * @brief   Change Requirement 1 (minimum-bit data) for subsystem 5:
 *          field widths, encoders and decoders for the words that are
 *          stored, shared between tasks, or sent over MQTT.
 *
 * Pure C. No kernel, no hardware, so every line here is unit tested on
 * a PC (testing/subsystem5/host/test_pack.c) and again on the robot
 * (subsystem5_test.c, S5-U01..U06).
 *
 * Two packing styles, one rule for when to use each
 * -------------------------------------------------
 *  1. Explicit shifts and masks (this file) for anything SHARED or
 *     SENT. Reasons:
 *       - C bit-field order and padding are compiler-defined, so a
 *         bit-field struct cannot be the MQTT wire format. The same
 *         shift/mask layout is used in RAM and on the wire.
 *       - A whole word can be built in a local variable and written
 *         with ONE 32-bit store. On the Cortex-M0+ that store is
 *         atomic; a bit-field write is a read-modify-write, and the
 *         M0+ has no LDREX/STREX to make that safe across tasks or
 *         across the two RP2040 cores.
 *  2. C bit-fields (unsigned int only, Barr-C) for task-PRIVATE state
 *     that a single task reads and writes (s5_task_ctx_t in
 *     obstacle_task.c). No other context touches it, so the
 *     compiler's read-modify-write cannot race.
 *
 * Every field width below is the smallest that covers the valid range.
 * The data representation table in testing/subsystem5/TEST_LOG.md
 * lists each one with its range.
 */
#ifndef S5_PACK_H
#define S5_PACK_H

#include <stdbool.h>
#include <stdint.h>

/*=====================================================================*/
/* Generic field helpers                                               */
/*=====================================================================*/

/** Mask of the low `width` bits. Valid for width 1..31. */
#define S5_MASK(width)              ((uint32_t)((1UL << (width)) - 1UL))

/** Extract a field. */
#define S5_GET(word, shift, width)  \
    (((uint32_t)(word) >> (shift)) & S5_MASK(width))

/** Place a value into a field position. Bits above `width` are cut
 *  off, so callers saturate first (see the s5_sat_* helpers). */
#define S5_PUT(value, shift, width) \
    (((uint32_t)(value) & S5_MASK(width)) << (shift))

/** Compile-time check that does not need C11. */
#define S5_STATIC_ASSERT(cond, name) \
    typedef char s5_static_assert_##name[(cond) ? 1 : -1]

/**
 * Narrow a value into a `width`-bit field of a task-private bit-field
 * struct. Use only where a static assert or the code path already
 * proves the value fits; the mask makes that proof visible to the
 * compiler (-Wconversion) and to a reviewer.
 */
#define S5_FIT(value, width)        \
    ((unsigned int)((uint32_t)(value) & S5_MASK(width)))

/** Saturate an unsigned value to `width` bits instead of wrapping. */
static inline uint32_t s5_sat_u(uint32_t value, uint32_t width)
{
    const uint32_t max = S5_MASK(width);

    return (value > max) ? max : value;
}

/*=====================================================================*/
/* Range: 12 bits                                                      */
/*=====================================================================*/

/*
 * Valid sensor readings are 30..2500 mm. 11 bits stop at 2047, so 12
 * bits (0..4095) are needed. The two top codes are the sentinels.
 *
 * The public API still uses the 16-bit sentinels 0xFFFE / 0xFFFF that
 * the rest of the team already compares against. Only storage and the
 * wire use the 12-bit codes, so nobody else's code changes (decided in
 * the Tribunal 1 live log).
 */
#define S5_RANGE_BITS               (12U)
#define S5_RANGE12_CLEAR            (0xFFEU)    /* answered, nothing    */
#define S5_RANGE12_UNKNOWN          (0xFFFU)    /* stale, fault, blind  */
#define S5_RANGE12_MAX_MM           (0xFFDU)    /* 4093, saturation cap */

#define S5_RANGE_CLEAR_MM           (0xFFFEU)   /* public 16-bit code   */
#define S5_RANGE_UNKNOWN_MM         (0xFFFFU)   /* public 16-bit code   */

static inline uint32_t s5_range_encode(uint16_t range_mm)
{
    uint32_t code;

    if (range_mm == (uint16_t)S5_RANGE_UNKNOWN_MM)
    {
        code = S5_RANGE12_UNKNOWN;
    }
    else if (range_mm == (uint16_t)S5_RANGE_CLEAR_MM)
    {
        code = S5_RANGE12_CLEAR;
    }
    else if (range_mm > (uint16_t)S5_RANGE12_MAX_MM)
    {
        code = S5_RANGE12_MAX_MM;
    }
    else
    {
        code = (uint32_t)range_mm;
    }

    return code;
}

static inline uint16_t s5_range_decode(uint32_t code)
{
    const uint32_t c = code & S5_MASK(S5_RANGE_BITS);
    uint16_t       mm;

    if (c == S5_RANGE12_UNKNOWN)
    {
        mm = (uint16_t)S5_RANGE_UNKNOWN_MM;
    }
    else if (c == S5_RANGE12_CLEAR)
    {
        mm = (uint16_t)S5_RANGE_CLEAR_MM;
    }
    else
    {
        mm = (uint16_t)c;
    }

    return mm;
}

/*=====================================================================*/
/* Timestamps: 17 bits of milliseconds                                 */
/*=====================================================================*/

/*
 * Stored stamps are only ever used to compute an AGE ("is this reading
 * older than 250 ms?"), never an absolute time. 17 bits wrap every
 * 131 s, and modular subtraction gives the right age for anything
 * younger than that. A full 32-bit stamp would waste 15 bits.
 */
#define S5_TIME_BITS                (17U)

static inline uint32_t s5_time_stamp(uint32_t now_ms)
{
    return now_ms & S5_MASK(S5_TIME_BITS);
}

static inline uint32_t s5_age_ms(uint32_t stamp, uint32_t now_ms)
{
    return (now_ms - stamp) & S5_MASK(S5_TIME_BITS);
}

/*=====================================================================*/
/* Polar sample: 16 bits (was 32)                                      */
/*=====================================================================*/

/*
 * Every bearing the scan visits is a multiple of 10 deg inside
 * +/-70 deg (coarse -60..60 step 30, fine +/-30 step 10 around it,
 * clipped to the +/-75 deg servo limit). That is 15 values, so the
 * bearing is stored as a 4-bit index: idx = bearing / 10 + 7.
 *
 *   bit 15..12  bearing index 0..14     (4 bits)
 *   bit 11..0   range, 12-bit code      (12 bits)
 */
typedef uint16_t s5_sample_t;

#define S5_SAMPLE_RANGE_SHIFT       (0U)
#define S5_SAMPLE_BEARING_SHIFT     (12U)
#define S5_SAMPLE_BEARING_BITS      (4U)

#define S5_BEARING_STEP_DEG         (10)
#define S5_BEARING_MIN_DEG          (-70)
#define S5_BEARING_MAX_DEG          (70)
#define S5_BEARING_INDEX_OFFSET     (7)

/** True if the bearing sits on the 10 deg grid the scan uses. */
static inline bool s5_bearing_on_grid(int16_t bearing_deg)
{
    return ((bearing_deg >= S5_BEARING_MIN_DEG) &&
            (bearing_deg <= S5_BEARING_MAX_DEG) &&
            ((bearing_deg % S5_BEARING_STEP_DEG) == 0));
}

static inline s5_sample_t s5_sample_pack(int16_t bearing_deg,
                                         uint16_t range_mm)
{
    int32_t b = (int32_t)bearing_deg;
    int32_t idx;

    /* Clamp, then round to the nearest grid point. Off-grid input is
     * a caller bug; it is made safe rather than undefined. */
    if (b < S5_BEARING_MIN_DEG) { b = S5_BEARING_MIN_DEG; }
    if (b > S5_BEARING_MAX_DEG) { b = S5_BEARING_MAX_DEG; }

    idx = ((b - S5_BEARING_MIN_DEG) + (S5_BEARING_STEP_DEG / 2)) /
          S5_BEARING_STEP_DEG;

    return (s5_sample_t)(S5_PUT((uint32_t)idx, S5_SAMPLE_BEARING_SHIFT,
                                S5_SAMPLE_BEARING_BITS) |
                         S5_PUT(s5_range_encode(range_mm),
                                S5_SAMPLE_RANGE_SHIFT, S5_RANGE_BITS));
}

static inline int16_t s5_sample_bearing(s5_sample_t s)
{
    const int32_t idx = (int32_t)S5_GET(s, S5_SAMPLE_BEARING_SHIFT,
                                        S5_SAMPLE_BEARING_BITS);

    return (int16_t)((idx - S5_BEARING_INDEX_OFFSET) * S5_BEARING_STEP_DEG);
}

static inline uint16_t s5_sample_range(s5_sample_t s)
{
    return s5_range_decode(S5_GET(s, S5_SAMPLE_RANGE_SHIFT,
                                  S5_RANGE_BITS));
}

/*=====================================================================*/
/* Front word: 32 bits, shared without a lock                          */
/*=====================================================================*/

/*
 * The forward range is read by VehicleController every control cycle,
 * possibly on the other core. Range, status and stamp live in ONE word
 * written by ONE task with ONE store, so a reader always sees a
 * matching set. No mutex is needed, and the guard loop never waits on
 * a slow reader.
 *
 *   bit 31..15  stamp, ms, 17 bits
 *   bit 14..12  last ultrasonic status, 3 bits (8 codes)
 *   bit 11..0   range, 12-bit code
 */
typedef uint32_t s5_front_t;

#define S5_FRONT_RANGE_SHIFT        (0U)
#define S5_FRONT_STATUS_SHIFT       (12U)
#define S5_FRONT_STATUS_BITS        (3U)
#define S5_FRONT_TIME_SHIFT         (15U)

static inline s5_front_t s5_front_pack(uint16_t range_mm, uint32_t status,
                                       uint32_t now_ms)
{
    return S5_PUT(s5_range_encode(range_mm), S5_FRONT_RANGE_SHIFT,
                  S5_RANGE_BITS) |
           S5_PUT(status, S5_FRONT_STATUS_SHIFT, S5_FRONT_STATUS_BITS) |
           S5_PUT(s5_time_stamp(now_ms), S5_FRONT_TIME_SHIFT,
                  S5_TIME_BITS);
}

static inline uint16_t s5_front_range(s5_front_t w)
{
    return s5_range_decode(S5_GET(w, S5_FRONT_RANGE_SHIFT, S5_RANGE_BITS));
}

static inline uint32_t s5_front_status(s5_front_t w)
{
    return S5_GET(w, S5_FRONT_STATUS_SHIFT, S5_FRONT_STATUS_BITS);
}

static inline uint32_t s5_front_stamp(s5_front_t w)
{
    return S5_GET(w, S5_FRONT_TIME_SHIFT, S5_TIME_BITS);
}

S5_STATIC_ASSERT(S5_FRONT_TIME_SHIFT + S5_TIME_BITS == 32U, front_fills_32);
S5_STATIC_ASSERT(sizeof(s5_sample_t) == 2U, sample_is_16_bits);

#endif /* S5_PACK_H */
