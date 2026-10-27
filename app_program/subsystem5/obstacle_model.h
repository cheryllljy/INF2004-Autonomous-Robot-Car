/**
 * @file    obstacle_model.h
 * @brief   Subsystem 5 data model: actions, states, faults, the obstacle
 *          profile and health, their packed (CR1) forms, and the pure
 *          functions that build and plan from them.
 *
 * No kernel, no hardware. Everything declared here is implemented in
 * obstacle_geometry.c and s5_pack.c and is unit tested on a PC.
 *
 * Bearing convention: 0 = straight ahead, negative = right,
 * positive = left.
 */
#ifndef OBSTACLE_MODEL_H
#define OBSTACLE_MODEL_H

#include <stdbool.h>
#include <stdint.h>

/* No <stddef.h> here: the kernel's tk/syslib.h defines its own size_t,
 * and this header is included next to it. Lengths are uint32_t. */

#include "s5_pack.h"

/*=====================================================================*/
/* Ranges and thresholds                                               */
/*=====================================================================*/

/*
 * CLEAR   - a healthy ping, nothing within the sensor's useful range.
 * UNKNOWN - stale, faulted, or something may be in the 30 mm blind
 *           zone. Never treat UNKNOWN as clear: slow down or stop.
 */
#define OBSTACLE_RANGE_CLEAR        (S5_RANGE_CLEAR_MM)
#define OBSTACLE_RANGE_UNKNOWN      (S5_RANGE_UNKNOWN_MM)

#define OBSTACLE_TRIGGER_MM         (300U)  /* start the scan           */
#define OBSTACLE_HARD_STOP_MM       (120U)  /* too close to steer round */
#define OBSTACLE_SAFETY_MARGIN_MM   (60U)
#define OBSTACLE_GAP_OPEN_MM        (1500U) /* gap with nothing seen    */
#define OBSTACLE_MAX_SAMPLES        (16U)   /* 5 coarse + 7 fine + spare*/

/** Half the widest part of the chassis, wheels included. Measured. */
#define OBSTACLE_HALF_WIDTH_MM      (75U)

/*
 * "Past the obstacle" (tutor feedback, Statechart 6.5). The forward
 * sensor cannot see the far side of a box, so the bypass ends after a
 * fixed encoder distance on the offset path:
 *
 *   pass = nearest + assumed depth + robot length + margin
 *
 * TODO(measure): robot length on the real chassis, and the deepest
 * obstacle on the course. Until then these are conservative guesses.
 */
#define OBSTACLE_ROBOT_LENGTH_MM    (200U)
#define OBSTACLE_ASSUMED_DEPTH_MM   (150U)
#define OBSTACLE_PASS_MARGIN_MM     (60U)

/*=====================================================================*/
/* Enumerations, each with the field width it is stored in             */
/*=====================================================================*/

/** Recommendation to VehicleController. 6 values -> 3 bits. */
typedef enum
{
    OBSTACLE_ACT_CONTINUE = 0,  /* nothing in the way                  */
    OBSTACLE_ACT_SLOW,          /* something ahead, keep ranging       */
    OBSTACLE_ACT_STOP,          /* stop now, data unusable             */
    OBSTACLE_ACT_BYPASS_LEFT,
    OBSTACLE_ACT_BYPASS_RIGHT,
    OBSTACLE_ACT_REVERSE,       /* too close or boxed in, back off     */
    OBSTACLE_ACT_COUNT
} obstacle_action_t;

#define S5_ACTION_BITS              (3U)

/** ObstacleScan state. 9 values -> 4 bits (8 would fit in 3; adding
 *  WaitStopped after Tribunal 1 made it 9). */
typedef enum
{
    OBSTACLE_ST_INIT = 0,
    OBSTACLE_ST_GUARD,          /* sensor ahead, one ping per 60 ms    */
    OBSTACLE_ST_CONFIRM,        /* one close return, checking a second */
    OBSTACLE_ST_WAIT_STOP,      /* wait for odometry speed = 0         */
    OBSTACLE_ST_COARSE,         /* 5-point sweep                       */
    OBSTACLE_ST_FINE,           /* 7-point sweep round the target      */
    OBSTACLE_ST_PUBLISH,        /* build, plan, publish                */
    OBSTACLE_ST_ASSIST,         /* controller is manoeuvring           */
    OBSTACLE_ST_FAULT,
    OBSTACLE_ST_COUNT
} obstacle_state_t;

#define S5_STATE_BITS               (4U)

/** Why ObstacleScan is in FAULT. 5 values -> 3 bits. */
typedef enum
{
    OBSTACLE_FAULT_NONE = 0,
    OBSTACLE_FAULT_SENSOR,          /* 10 hard errors in a row; sets the
                                       E-stop cause bit                 */
    OBSTACLE_FAULT_STOP_TIMEOUT,    /* car not stopped within 1 s       */
    OBSTACLE_FAULT_REVERSE_LIMIT,   /* third REVERSE in a row           */
    OBSTACLE_FAULT_ASSIST_TIMEOUT,  /* no resume within 5 s             */
    OBSTACLE_FAULT_COUNT
} obstacle_fault_t;

#define S5_FAULT_BITS               (3U)

S5_STATIC_ASSERT(OBSTACLE_ACT_COUNT <= (1U << S5_ACTION_BITS), action_fits);
S5_STATIC_ASSERT(OBSTACLE_ST_COUNT <= (1U << S5_STATE_BITS), state_fits);
S5_STATIC_ASSERT(OBSTACLE_FAULT_COUNT <= (1U << S5_FAULT_BITS), fault_fits);

/*=====================================================================*/
/* Obstacle profile                                                    */
/*=====================================================================*/

/**
 * Working view of a profile. Built on the stack by the geometry, read
 * by callers after obstacle_get_profile(). Never stored or shared in
 * this form: storage and the wire use obstacle_profile_packed_t.
 */
typedef struct
{
    bool              valid;          /* false: treat as STOP          */
    uint16_t          nearest_mm;     /* closest point, or CLEAR       */
    int16_t           bearing_deg;    /* centre of the object          */
    uint16_t          width_mm;       /* beam-corrected                */
    int16_t           left_edge_deg;
    int16_t           right_edge_deg;
    uint16_t          gap_left_mm;    /* free corridor, left           */
    uint16_t          gap_right_mm;   /* free corridor, right          */
    uint16_t          swing_left_mm;  /* sideways offset to clear left */
    uint16_t          swing_right_mm; /* ...and right; smaller wins    */
    uint8_t           confidence_pct;
    uint8_t           sample_count;
    uint32_t          timestamp_ms;   /* low 17 bits after unpacking   */
    obstacle_action_t action;
} obstacle_profile_t;

/**
 * Packed profile: 16 bytes instead of 32. Layout (bit ranges):
 *
 *   w[0]  31..24 bearing+75 (8)  23..12 width (12)  11..0 nearest (12)
 *   w[1]  31..27 samples (5)     26..16 gapL (11)   15..8 rightEdge+75 (8)
 *                                                    7..0 leftEdge+75 (8)
 *   w[2]  31 spare  30 valid (1) 29..23 conf (7)    22..11 swingL (12)
 *                                                   10..0 gapR (11)
 *   w[3]  31..15 stamp ms (17)   14..12 action (3)  11..0 swingR (12)
 *
 * Bearings are -75..+75, stored with +75 so the field is unsigned
 * (Barr-C: no signed bit-fields, no bitwise ops on signed values).
 */
typedef struct
{
    uint32_t w[4];
} obstacle_profile_packed_t;

S5_STATIC_ASSERT(sizeof(obstacle_profile_packed_t) == 16U, profile_16_bytes);

/*=====================================================================*/
/* Health                                                              */
/*=====================================================================*/

/** Working view of the health record, for telemetry and tests. */
typedef struct
{
    obstacle_state_t state;
    obstacle_fault_t fault;
    bool             estop_requested; /* S5 set its E-stop cause bit   */
    bool             has_profile;
    bool             odometry_ok;     /* last WaitStopped used odometry*/
    uint8_t          reverses;        /* 0..2 in a row                 */
    uint8_t          last_status;     /* ultrasonic_status_t, 3 bits   */
    uint8_t          deadline_overruns; /* saturates at 255            */
    uint8_t          sensor_faults;     /* saturates at 255            */
    uint16_t         scans_completed;   /* saturates at 65535          */
    uint16_t         build_us_max;      /* worst profile build time    */
} obstacle_health_t;

/**
 * Packed health: 8 bytes. The v1 health struct was 16 bytes and held
 * only four of these fields.
 *
 *   w[0]  31..24 sensor faults (8)  23..16 overruns (8)
 *         15..13 last status (3)    12..11 reverses (2)
 *         10 odometry ok  9 has profile  8 estop requested
 *          7 spare        6..4 fault (3)  3..0 state (4)
 *   w[1]  31..16 build us max (16)  15..0 scans (16)
 */
typedef struct
{
    uint32_t w[2];
} obstacle_health_packed_t;

S5_STATIC_ASSERT(sizeof(obstacle_health_packed_t) == 8U, health_8_bytes);

/*=====================================================================*/
/* Telemetry packet (MQTT payload for Buddy 1)                         */
/*=====================================================================*/

/*
 * Binary, little-endian, 29 bytes. Replaces a ~150-byte JSON line.
 *
 *   byte 0        version (high nibble) | message type (low nibble)
 *   bytes 1..4    front word   (see s5_pack.h)
 *   bytes 5..12   health w[0], w[1]
 *   bytes 13..28  profile w[0..3]
 *
 * Decode on the dashboard with the same shifts as obstacle_model.h.
 * A different version nibble means "layout changed, do not guess".
 */
#define S5_TLM_VERSION              (1U)
#define S5_TLM_TYPE_OBSTACLE        (5U)
#define S5_TLM_LEN                  (29U)

/*=====================================================================*/
/* Pure functions (obstacle_geometry.c)                                */
/*=====================================================================*/

/**
 * @brief   Turn a packed polar sweep into a profile and an action.
 * @param   p_samples  Packed samples sorted by bearing, ascending.
 * @param   count      1..OBSTACLE_MAX_SAMPLES.
 * @param   p_out      Always written; invalid + STOP on bad input.
 */
void obstacle_profile_build(const s5_sample_t *p_samples,
                            uint8_t count,
                            obstacle_profile_t *p_out);

/** @brief Choose an action from a profile. Policy lives only here. */
obstacle_action_t obstacle_plan(const obstacle_profile_t *p_profile);

/** @brief True if a return lies inside the band the car drives through. */
bool obstacle_in_path(int16_t bearing_deg, uint16_t range_mm);

/**
 * @brief   Forward encoder travel on the offset path after which the car
 *          is past the obstacle. 0 if the profile has no obstacle.
 */
uint16_t obstacle_pass_distance_mm(const obstacle_profile_t *p_profile);

/*=====================================================================*/
/* Packing (s5_pack.c)                                                 */
/*=====================================================================*/

void obstacle_profile_pack(const obstacle_profile_t *p_in,
                           obstacle_profile_packed_t *p_out);
void obstacle_profile_unpack(const obstacle_profile_packed_t *p_in,
                             obstacle_profile_t *p_out);

void obstacle_health_pack(const obstacle_health_t *p_in,
                          obstacle_health_packed_t *p_out);
void obstacle_health_unpack(const obstacle_health_packed_t *p_in,
                            obstacle_health_t *p_out);

/**
 * @brief   Serialise the telemetry packet.
 * @return  Bytes written (S5_TLM_LEN), or 0 if the buffer is too small.
 */
uint32_t s5_tlm_encode(uint8_t *p_buf, uint32_t len,
                     s5_front_t front,
                     const obstacle_health_packed_t *p_health,
                     const obstacle_profile_packed_t *p_profile);

/** @brief Reference decoder (dashboard side and tests). */
bool s5_tlm_decode(const uint8_t *p_buf, uint32_t len,
                   s5_front_t *p_front,
                   obstacle_health_packed_t *p_health,
                   obstacle_profile_packed_t *p_profile);

#endif /* OBSTACLE_MODEL_H */
