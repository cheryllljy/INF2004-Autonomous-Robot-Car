/**
 * @file    drv_ultrasonic.h
 * @brief   HC-SR04 on TRIG GP16 / ECHO GP17, timed in hardware by
 *          PWM slice 0 in level-gated mode. No interrupt.
 *
 * Called only from the obstacle task, the owner of GP16, GP17 and PWM
 * slice 0 (TEAM.md section 3).
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "s5_pack.h"

#define DRV_US_MIN_VALID_MM         (30U)
#define DRV_US_MAX_VALID_MM         (2500U)
#define DRV_US_MEDIAN_SAMPLES       (3U)

/** Result of one ping. 8 values -> 3 bits. */
typedef enum
{
    ULTRASONIC_OK = 0,        /* valid range                           */
    ULTRASONIC_NO_ECHO,       /* sensor answered (ECHO rose) and heard
                                 nothing in range: path CLEAR, not a
                                 fault                                 */
    ULTRASONIC_NO_RESPONSE,   /* ECHO never rose: dead sensor, no 5 V,
                                 TRIG/ECHO swapped. ALWAYS a fault.    */
    ULTRASONIC_TOO_CLOSE,     /* echo shorter than the 30 mm blind zone*/
    ULTRASONIC_TOO_FAR,       /* beyond 2500 mm                        */
    ULTRASONIC_ECHO_STUCK,    /* ECHO high before triggering: fault    */
    ULTRASONIC_NOT_READY,     /* init not done or failed               */
    ULTRASONIC_BAD_PARAM,     /* NULL pointer                          */
    ULTRASONIC_STATUS_COUNT
} ultrasonic_status_t;

#define DRV_US_STATUS_BITS          (3U)

S5_STATIC_ASSERT(ULTRASONIC_STATUS_COUNT <= (1U << DRV_US_STATUS_BITS),
                 us_status_fits);

/**
 * One ping result, packed into 32 bits (CR1). Was 12 bytes.
 *
 *   bit 29..27  status (3)
 *   bit 26..12  echo width, us, saturating at 32767 (15)
 *   bit 11..0   range, 12-bit code (12)
 *
 * The echo is read at most 25 ms after the trigger, so 15 bits hold
 * every width that can actually be measured.
 */
typedef uint32_t ultrasonic_result_t;

#define DRV_US_RES_RANGE_SHIFT      (0U)
#define DRV_US_RES_ECHO_SHIFT       (12U)
#define DRV_US_RES_ECHO_BITS        (15U)
#define DRV_US_RES_STATUS_SHIFT     (27U)

static inline ultrasonic_result_t drv_us_result_pack(uint16_t range_mm,
                                                     uint32_t echo_us,
                                                     ultrasonic_status_t st)
{
    return S5_PUT(s5_range_encode(range_mm), DRV_US_RES_RANGE_SHIFT,
                  S5_RANGE_BITS) |
           S5_PUT(s5_sat_u(echo_us, DRV_US_RES_ECHO_BITS),
                  DRV_US_RES_ECHO_SHIFT, DRV_US_RES_ECHO_BITS) |
           S5_PUT((uint32_t)st, DRV_US_RES_STATUS_SHIFT, DRV_US_STATUS_BITS);
}

static inline uint16_t drv_us_result_mm(ultrasonic_result_t r)
{
    return s5_range_decode(S5_GET(r, DRV_US_RES_RANGE_SHIFT, S5_RANGE_BITS));
}

static inline uint32_t drv_us_result_echo_us(ultrasonic_result_t r)
{
    return S5_GET(r, DRV_US_RES_ECHO_SHIFT, DRV_US_RES_ECHO_BITS);
}

static inline ultrasonic_status_t drv_us_result_status(ultrasonic_result_t r)
{
    return (ultrasonic_status_t)S5_GET(r, DRV_US_RES_STATUS_SHIFT,
                                       DRV_US_STATUS_BITS);
}

/**
 * Counters, 16-bit saturating (CR1). Was 5 x 32 bits = 20 bytes.
 * Written only by the obstacle task. Each counter is its own aligned
 * halfword, so a reader on another core never sees a torn counter;
 * the set as a whole may be one ping out of step, which is fine for
 * telemetry.
 */
typedef struct
{
    uint16_t pings;
    uint16_t no_response;     /* the A5 metric: target < 1 %           */
    uint16_t no_echo;
    uint16_t rejects;         /* out of range, stuck                   */
} ultrasonic_stats_t;

/** Configure TRIG, ECHO and PWM slice 0. */
bool drv_ultrasonic_init(void);

/**
 * @brief   One ping. Blocks the calling task (in the kernel, not
 *          spinning) for the quiet-time remainder plus ~20-30 ms.
 */
ultrasonic_status_t drv_ultrasonic_ping(ultrasonic_result_t *p_result);

/** Median of three pings. OK only if at least two were valid. */
ultrasonic_status_t drv_ultrasonic_ping_median(ultrasonic_result_t *p_result);

void drv_ultrasonic_get_stats(ultrasonic_stats_t *p_stats);

/** Microsecond clock used for the timing evidence (build time). */
uint32_t drv_ultrasonic_time_us(void);

#endif /* DRV_ULTRASONIC_H */
