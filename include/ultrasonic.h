/**
 * @file    ultrasonic.h
 * @brief   HC-SR04 ranger driver, interrupt driven, non-blocking CPU.
 *
 * Layer   : driver. Owns the TRIG/ECHO pins and one semaphore. Knows
 *           nothing about servos, obstacles or the mission.
 *
 * The echo pulse is timed by a GPIO edge interrupt, not by polling.
 * The calling task sleeps while the sound is in flight, so a ping
 * costs a few microseconds of CPU instead of up to 25 ms of spinning.
 *
 * Thread safety: one owner task only. The whole subsystem accesses the
 * sensor from a single task, so no lock is needed and none is taken.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

/** Returned in distance_mm when no usable range was obtained. */
#define ULTRASONIC_RANGE_NONE       (0xFFFFU)

/** Shortest range the sensor can be trusted at (ringing blind zone). */
#define ULTRASONIC_MIN_VALID_MM     (30U)

/** Longest range accepted. Beyond this the echo is too weak to trust. */
#define ULTRASONIC_MAX_VALID_MM     (2500U)

/** Number of pings combined by ultrasonic_ping_median(). Must be odd. */
#define ULTRASONIC_MEDIAN_SAMPLES   (3U)

/** Outcome of a single ping. */
typedef enum
{
    ULTRASONIC_OK = 0,        /**< Range is valid.                    */
    ULTRASONIC_NO_ECHO,       /**< Timed out: open space, absorbent
                                   surface, or an object inside the
                                   blind zone. NEVER read as "clear". */
    ULTRASONIC_TOO_CLOSE,     /**< Echo shorter than the blind zone.  */
    ULTRASONIC_TOO_FAR,       /**< Beyond ULTRASONIC_MAX_VALID_MM.    */
    ULTRASONIC_ECHO_STUCK,    /**< ECHO line high before trigger.     */
    ULTRASONIC_NOT_READY,     /**< ultrasonic_init() not called / ok. */
    ULTRASONIC_BAD_PARAM      /**< NULL pointer.                      */
} ultrasonic_status_t;

/** Result of one measurement. */
typedef struct
{
    uint16_t            distance_mm;  /**< ULTRASONIC_RANGE_NONE if
                                           status != ULTRASONIC_OK.  */
    uint32_t            echo_us;      /**< Raw pulse width.          */
    ultrasonic_status_t status;
} ultrasonic_result_t;

/** Health counters, published to telemetry by Buddy 1. */
typedef struct
{
    uint32_t ping_count;
    uint32_t timeout_count;
    uint32_t reject_count;    /**< Out of range or stuck echo.       */
    uint32_t spurious_edges;
} ultrasonic_stats_t;

/**
 * @brief   Configure pins, create the echo semaphore, install the ISR.
 * @return  true on success.
 * @note    Must be called from a task, after the kernel has started.
 */
bool ultrasonic_init(void);

/**
 * @brief   Take one range measurement.
 * @param   p_result  Filled in on every outcome, including failures.
 * @return  Copy of p_result->status, for convenience.
 *
 * Blocks the calling task for at most
 * ULTRASONIC_QUIET_MS + ULTRASONIC_TIMEOUT_MS (see ultrasonic.c).
 * The mandatory quiet period between pings is enforced inside this
 * function, so a caller that did useful work since the previous ping
 * (for example moving the servo) pays no extra delay for it.
 */
ultrasonic_status_t ultrasonic_ping(ultrasonic_result_t *p_result);

/**
 * @brief   Median of ULTRASONIC_MEDIAN_SAMPLES pings.
 * @param   p_result  Median of the valid samples.
 * @return  ULTRASONIC_OK if at least two samples were valid.
 *
 * Costs about 3x the time of one ping. Use it for the fine scan and
 * for width measurement, not for the periodic forward guard.
 */
ultrasonic_status_t ultrasonic_ping_median(ultrasonic_result_t *p_result);

/**
 * @brief   Copy the health counters out.
 */
void ultrasonic_get_stats(ultrasonic_stats_t *p_stats);

#endif /* ULTRASONIC_H */
