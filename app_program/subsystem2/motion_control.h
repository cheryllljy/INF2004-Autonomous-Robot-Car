/*
 * motion_control.h - MotionControl public API
 *
 * 1. Commands go through the motion_cmd message buffer (FR26).
 * 2. The e-stop flag is checked every 20 ms control cycle (NFR21).
 * 3. Status is read with motion_get_status().
 * 4. ISRs may only call the ISR-safe functions.
 *
 * Units: distance mm, speed mm/s, angle degrees, steer percent.
 * Sign convention: +distance = forward, +angle = LEFT (counter-clockwise)
 *
 * Threading: every function is safe to call from any task (not from an ISR).
 * Commands are posted to the MotionTask queue; nothing here touches the
 * motors directly from the caller's context.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

/* Set to 1 by the calibration build option (see the makefile). Tested with
 * #if, so 0 means "off". */
#ifndef MOTION_CALIBRATION_BUILD
#define MOTION_CALIBRATION_BUILD (0)
#endif

/* 1 = build motion_demo.c: a mock VehicleController that drives scripted
 * moves for bench testing. */
#ifndef MOTION_DEMO
#define MOTION_DEMO (0)
#endif


#include <stdint.h>
#include <stdbool.h>

/* ---------- E-stop causes: one bit per cause in the e-stop flag ---------- */
#define MOTION_ESTOP_VEHICLE  (0x00000001U) /* VehicleController */
#define MOTION_ESTOP_IMU      (0x00000002U) /* IMU */
#define MOTION_ESTOP_OBSTACLE (0x00000004U) /* ObstacleScan */

/* ---------- Command outcome bits (vehicle_evt flag) ---------- */
#define MOTION_EVT_DONE (0x1U) /* motion executed */
/* command refused */
#define MOTION_EVT_REJECTED (0x2U)
#define MOTION_EVT_FAULT    (0x4U) /* motion / encoder stalled */
#define MOTION_EVT_ESTOP    (0x8U) /* e-stop occurred */

/* ---------- Types ---------- */

typedef enum
{
    MOTION_STATE_INIT = 0,
    MOTION_STATE_IDLE,
    MOTION_STATE_DRIVING, /* move using distance or fixed speed */
    MOTION_STATE_TURNING,
    MOTION_STATE_STOPPING, /* slow down till stop */
    MOTION_STATE_FAULT,
    MOTION_STATE_LINE_FOLLOWING, /* steering from LineSense */
    MOTION_STATE_ESTOP
#ifdef MOTION_CALIBRATION_BUILD
        ,
    MOTION_STATE_CALIBRATE, /* development firmware only */
#endif
} motion_state_t;

typedef enum
{
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_STALL_L,
    MOTION_FAULT_STALL_R,
    MOTION_FAULT_ENCODER_TIMEOUT,
    MOTION_FAULT_LINE_STALE /* for line_following */
} motion_fault_t;

/* Returned when SUBMITTING a command */
typedef enum
{
    MOTION_OK = 0,
    MOTION_ERR_INVALID, /* bad argument (e.g. speed <= 0, wrong mode) */
    MOTION_ERR_QUEUE_FULL /* motion_cmd message buffer full */
} motion_err_t;

/* Outcome of the last command (motion_status_t.last_result) */
typedef enum
{
    MOTION_RESULT_NONE = 0, /* no result yet */
    MOTION_RESULT_DONE,     /* target reached within tolerance */
    MOTION_RESULT_STOPPED,  /* interrupted by motion_stop() */
    MOTION_RESULT_FAULT, /* stall or encoder fault */
    MOTION_RESULT_ESTOP,
    MOTION_RESULT_REJECTED /* command was refused */
} motion_result_t;

/* motion_cmd message buffer format */
typedef enum
{
    MOTION_CMD_MOVE = 0,
    MOTION_CMD_TURN,
    MOTION_CMD_LINE_FOLLOW,
    MOTION_CMD_SET_SPEED,
    MOTION_CMD_STOP,
    MOTION_CMD_RESET,     /* clear fault/e-stop then IDLE */
    MOTION_CMD_RESET_TRIP /* reset distance to 0 */
} motion_cmd_type_t;

typedef struct
{
    motion_cmd_type_t type;
    int32_t p1; /* MOVE: distance mm (negative = backward) */
                /* TURN: angle deg (negative = right) */
                /* LINE_FOLLOW: speed mm/s */
                /* SET_SPEED: left wheel speed mm/s */

    int32_t p2; /* MOVE: speed mm/s */
                /* TURN: rate deg/s */
                /* SET_SPEED: right wheel speed mm/s */
} motion_cmd_msg_t;

/* Snapshot for telemetry and decision logic */
typedef struct
{
    motion_state_t state;
    motion_fault_t fault;
    motion_result_t last_result; /* outcome of most recent command */
    uint32_t estop_sources;      /* latched estop bit */
    uint32_t timestamp_ms;       /* time of event */
    uint32_t cycle_count;        /* watchdog check-in for control cycle */
    uint32_t overrun_count;      /* track late or missed control cycles */
    int16_t speed_l_mmps;
    int16_t speed_r_mmps;
    int32_t ticks_l;
    int32_t ticks_r;
    int32_t dist_l_mm;         /* left wheel distance */
    int32_t dist_r_mm;         /* right wheel distance */
    int32_t distance_total_mm; /* mean of both wheels */
    int32_t distance_trip_mm;  /* dist since MOTION_CMD_RESET_TRIP */
    /* encoder-estimated, since last reset; + = left */
    int32_t heading_deg;
} motion_status_t;

/* ---------- Lifecycle ---------- */

/* Set up PWM, encoder interrupts, queues. Call once before tasks start. */
bool motion_init(void);

/* Create MotionTask. Call after motion_init(). Task enters IDLE when ready. */
bool motion_start(void);

/* ---------- Commands (non-blocking, require IDLE) ---------- */

/* <0 = backward */
motion_err_t motion_move(int32_t distance_mm, int32_t speed_mmps);
/* + = left */
motion_err_t motion_turn(int32_t angle_deg, int32_t rate_dps);
motion_err_t motion_line_follow(int32_t speed_mmps);
motion_err_t motion_set_wheel_speeds(int32_t leftmmps, int32_t right_mmps);
/* slows down until stopped */
motion_err_t motion_stop(void);
/* clears fault/estop */
motion_err_t motion_reset(void);
motion_err_t motion_reset_trip(void);

/* For barcode instructions */
motion_err_t motion_move_forward(int32_t distance_mm);
motion_err_t motion_move_backward(int32_t distance_mm);
motion_err_t motion_turn_left(int32_t angle_deg);
motion_err_t motion_turn_right(int32_t angle_deg);
motion_err_t motion_u_turn(void);

/* ---------- E-stop ---------- */
/* Set the caller's source bit; takes effect at the next control cycle. */
void motion_estop_request(uint32_t source_bits);

/* Reset by VehicleController: clear bits then motion_reset() */
void motion_estop_clear(uint32_t source_bits);
/* the E-stop flag's bits set now (MOTION_ESTOP_*), 0 = none */
uint32_t motion_estop_sources(void);

/* Raise MOTION_EVT_* bits on VehicleController's vehicle_evt flag.
 * flgid = that flag's ID, bit_shift = first bit of Motion's 4-bit group.
 * Until this is called, results are only visible in the status. */
void motion_attach_vehicle_evt(int32_t flgid, uint32_t bit_shift);

/* ---------- Status (for any tasks) ---------- */
void motion_get_status(motion_status_t *out);

/* ---------- ISR-safe (no mutexes) ----------
 * motion_encoder_isr: GPIO dispatcher callback, counts encoder edges
 * motion_encoder_ticks_isr: lock-free read of both counters (for BarcodeISR)
 * motion_ticks_to_mm: converts a tick count using calibrated constants */
void motion_encoder_isr(uint32_t gpio);
void motion_encoder_ticks_isr(int32_t *left, int32_t *right);
int32_t motion_ticks_to_mm(int32_t ticks);

#if MOTION_CALIBRATION_BUILD
/* ---------- Bring-up / calibration (development firmware only) ---------- */
/* -1000..1000 */
motion_err_t motion_raw_set_pwm(int32_t left_pm, int32_t right_pm);
void motion_raw_get_ticks(int32_t *left, int32_t *right);

/* calibration tasks (motion_cal.c): call from subsystem2_init() & start() */
void motion_cal_init(void);
void motion_cal_start(void);
#endif /* MOTION_CALIBRATION_BUILD */

#if MOTION_DEMO
/* Bench demo (motion_demo.c): call from subsystem2_init() / _start() */
void motion_demo_init(void);
void motion_demo_start(void);
#endif

#endif /* MOTION_CONTROL_H */
