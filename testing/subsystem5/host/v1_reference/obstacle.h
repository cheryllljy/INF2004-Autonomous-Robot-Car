/**
 * @file    obstacle.h
 * @brief   Buddy 5 public interface: forward guard, obstacle profiling
 *          and bypass recommendation.
 *
 * THIS HEADER IS THE CONTRACT WITH THE REST OF THE TEAM.
 * Nothing outside src/obstacle.c may touch the servo or the ranger.
 *
 * Ownership
 * ---------
 *   This module owns : the servo, the ultrasonic sensor, the decision
 *                      of which side to pass on.
 *   This module never: drives a motor, reads the line sensors, or
 *                      publishes MQTT. It produces data and a
 *                      recommendation; the vehicle controller acts.
 *
 * Bearing convention (same as servo.h, shifted so ahead is zero)
 *
 *          +75 deg  \     |     /  -75 deg
 *            LEFT    \    |    /    RIGHT
 *                     \   0   /
 *                      [ car ]
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Define OBSTACLE_HOST_TEST when compiling the pure geometry and
 * policy functions on a PC. The kernel-facing half of this header is
 * then omitted, so test/ needs no RTOS and no hardware.
 */
#ifndef OBSTACLE_HOST_TEST
#include <tk/tkernel.h>
#endif

/**
 * Distance sentinels. The difference between them is a safety
 * decision, not a formatting detail:
 *
 *   CLEAR   - a healthy ping came back with no echo, so there is
 *             nothing within the sensor's useful range. Cruise.
 *   UNKNOWN - the reading is stale, the sensor is faulted, or an
 *             object is suspected inside the 30 mm blind zone.
 *             Never treat this as clear. Slow down or stop.
 */
#define OBSTACLE_RANGE_CLEAR        (0xFFFEU)
#define OBSTACLE_RANGE_UNKNOWN      (0xFFFFU)

/** Range at which a scan is triggered. See README timing budget. */
#define OBSTACLE_TRIGGER_MM         (300U)

/** Too close to steer around; back off instead. */
#define OBSTACLE_HARD_STOP_MM       (120U)

/** Lateral clearance the chassis needs to squeeze past an object. */
#define OBSTACLE_SAFETY_MARGIN_MM   (60U)

/** Lateral gap reported when a sector returned no echo at all. */
#define OBSTACLE_GAP_OPEN_MM        (1500U)

/** Maximum samples kept for one profile (coarse pass + fine pass). */
#define OBSTACLE_MAX_SAMPLES        (16U)

/** Event flag bits. Wait on obstacle_get_flag_id() with tk_wai_flg(). */
#define OBSTACLE_EVT_DETECTED       (0x0001U)  /**< Something ahead.  */
#define OBSTACLE_EVT_PROFILED       (0x0002U)  /**< Profile is ready. */
#define OBSTACLE_EVT_CLEARED        (0x0004U)  /**< Path is clear.    */
#define OBSTACLE_EVT_FAULT          (0x0008U)  /**< Sensor unhealthy. */

/** What this subsystem recommends the vehicle controller should do. */
typedef enum
{
    OBSTACLE_ACT_CONTINUE = 0,  /**< Nothing in the way.             */
    OBSTACLE_ACT_SLOW,          /**< Something ahead, keep ranging.  */
    OBSTACLE_ACT_STOP,          /**< Stop now, data is unusable.     */
    OBSTACLE_ACT_BYPASS_LEFT,
    OBSTACLE_ACT_BYPASS_RIGHT,
    OBSTACLE_ACT_REVERSE        /**< Boxed in: back off and rescan.  */
} obstacle_action_t;

/** Where the subsystem is in its cycle. Useful in telemetry. */
typedef enum
{
    OBSTACLE_ST_INIT = 0,
    OBSTACLE_ST_GUARD,      /**< Sensor ahead, pinging periodically. */
    OBSTACLE_ST_CONFIRM,    /**< Candidate seen, debouncing it.      */
    OBSTACLE_ST_COARSE,     /**< Wide sweep in progress.             */
    OBSTACLE_ST_FINE,       /**< Narrow sweep around the target.     */
    OBSTACLE_ST_PUBLISH,    /**< Building and publishing a profile.  */
    OBSTACLE_ST_ASSIST,     /**< Controller is bypassing; we range
                                 straight ahead at full rate.        */
    OBSTACLE_ST_FAULT
} obstacle_state_t;

/** One polar sample. Pure data, no hardware, easy to unit test. */
typedef struct
{
    int16_t  bearing_deg;   /**< Negative right, positive left.      */
    uint16_t range_mm;      /**< mm, or OBSTACLE_RANGE_CLEAR (sensor
                                 answered, nothing in range), or
                                 OBSTACLE_RANGE_UNKNOWN (silent).  */
} obstacle_sample_t;

/** The profile handed to the vehicle controller. */
typedef struct
{
    bool     valid;             /**< False: treat as STOP.           */
    uint16_t nearest_mm;        /**< Closest point on the object.    */
    int16_t  bearing_deg;       /**< Bearing of the object centre.   */
    uint16_t width_mm;          /**< Beam-corrected apparent width.  */
    int16_t  left_edge_deg;
    int16_t  right_edge_deg;
    uint16_t gap_left_mm;       /**< Free lateral corridor, left.    */
    uint16_t gap_right_mm;      /**< Free lateral corridor, right.   */
    uint16_t swing_left_mm;     /**< Sideways offset the chassis
                                     centre must take to clear the
                                     object on the left...            */
    uint16_t swing_right_mm;    /**< ...and on the right. The planner
                                     picks the smaller one, which is
                                     how "minimise deviation from the
                                     original route" is implemented.  */
    uint8_t  confidence_pct;    /**< 0..100, from sample agreement.  */
    uint8_t  sample_count;
    uint32_t timestamp_ms;
    obstacle_action_t action;
} obstacle_profile_t;

/** Health snapshot for telemetry. */
typedef struct
{
    obstacle_state_t state;
    uint32_t scans_completed;
    uint32_t deadline_overruns; /**< Guard tick missed its period.   */
    uint32_t sensor_faults;
} obstacle_health_t;

#ifndef OBSTACLE_HOST_TEST

/*=====================================================================*/
/* Lifecycle                                                           */
/*=====================================================================*/

/**
 * @brief   Create the kernel objects, drivers and scan task.
 * @return  true on success. On false the caller must not drive.
 * @note    Call once from usermain(), after the kernel is running.
 */
bool obstacle_init(void);

/*=====================================================================*/
/* Fast path: called by the vehicle controller every control cycle     */
/*=====================================================================*/

/**
 * @brief   Most recent forward range.
 * @return  Distance in mm, or OBSTACLE_RANGE_UNKNOWN when the reading
 *          is stale or the sensor reported no echo.
 *
 * SAFETY RULE FOR CALLERS: OBSTACLE_RANGE_UNKNOWN means "I do not
 * know", never "the path is clear". An object inside the 30 mm blind
 * zone produces exactly this value. Treat it as a reason to slow down.
 */
uint16_t obstacle_get_front_mm(void);

/**
 * @brief   Copy the latest profile out under the module lock.
 * @param   p_out  Destination, untouched if false is returned.
 * @return  true if a profile has ever been published.
 */
bool obstacle_get_profile(obstacle_profile_t *p_out);

/**
 * @brief   Copy the health counters out for telemetry.
 */
void obstacle_get_health(obstacle_health_t *p_out);

/**
 * @brief   Event flag ID so the controller can block instead of poll.
 */
ID obstacle_get_flag_id(void);

/**
 * @brief   Tell the subsystem the bypass manoeuvre is finished and the
 *          line has been reacquired. Returns it to the guard state.
 */
void obstacle_resume(void);

#endif /* OBSTACLE_HOST_TEST */

/*=====================================================================*/
/* Pure functions - no hardware, no kernel. Unit tested on the host.   */
/*=====================================================================*/

/**
 * @brief   Turn a set of polar samples into an obstacle profile.
 * @param   p_samples  Array sorted by bearing, ascending.
 * @param   count      Number of entries, 1..OBSTACLE_MAX_SAMPLES.
 * @param   p_out      Destination profile. action is not set here.
 *
 * Deterministic, integer only, no side effects. See test/ for the
 * host-side test vectors used as evidence in the report.
 */
void obstacle_profile_build(const obstacle_sample_t *p_samples,
                            uint8_t count,
                            obstacle_profile_t *p_out);

/**
 * @brief   Choose a bypass action from a profile.
 * @param   p_profile  A profile from obstacle_profile_build().
 * @return  The recommended action.
 *
 * Policy lives here and nowhere else. To change how the robot decides,
 * change this one function and re-run the host tests.
 */
obstacle_action_t obstacle_plan(const obstacle_profile_t *p_profile);

#endif /* OBSTACLE_H */
