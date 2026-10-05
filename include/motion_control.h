/*
 * motion_control.h - Buddy 2 public API
 *
 * Units: distance mm, speed mm/s, angle degrees, steer percent.
 * Sign convention: +distance = forward, +angle = LEFT (counter-clockwise),
 *                  steer +100 = full left, -100 = full right.
 *
 * Threading: every function is safe to call from any task (not from an ISR).
 * Commands are posted to the MotionTask queue; nothing here touches the
 * motors directly from the caller's context.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

/* ---------- Types ---------- */

typedef enum {
    MOTION_STATE_INIT = 0,
    MOTION_STATE_IDLE,
#ifdef MOTION_CALIBRATION_BUILD
    MOTION_STATE_CALIBRATE,     /* development firmware only */
#endif
    MOTION_STATE_DRIVING,
    MOTION_STATE_TURNING,
    MOTION_STATE_STOPPING,
    MOTION_STATE_FAULT
} motion_state_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_STALL_L,
    MOTION_FAULT_STALL_R,
    MOTION_FAULT_ENCODER_TIMEOUT
} motion_fault_t;

/* Returned when SUBMITTING a command */
typedef enum {
    MOTION_OK = 0,
    MOTION_ERR_BUSY,        /* distance/turn command while not idle */
    MOTION_ERR_INVALID,     /* bad argument (e.g. speed <= 0, wrong mode) */
    MOTION_ERR_FAULT,       /* controller is in FAULT; call motion_clear_fault() */
    MOTION_ERR_QUEUE_FULL
} motion_err_t;

/* Returned when a motion COMPLETES (motion_wait_done) */
typedef enum {
    MOTION_RESULT_DONE = 0, /* target reached within tolerance */
    MOTION_RESULT_STOPPED,  /* interrupted by motion_stop() */
    MOTION_RESULT_TIMEOUT,  /* wait timed out; motion may still be running */
    MOTION_RESULT_FAULT     /* stall or encoder fault */
} motion_result_t;

/* Snapshot for telemetry (Buddy 1) and decision logic (Buddies 3, 5) */
typedef struct {
    motion_state_t state;
    motion_fault_t fault;
    int16_t  speed_l_mmps;
    int16_t  speed_r_mmps;
    int32_t  ticks_l;
    int32_t  ticks_r;
    int32_t  distance_mm;      /* mean of both wheels since last reset */
    int32_t  heading_deg;      /* encoder-estimated, since last reset; + = left */
} motion_status_t;

/* ---------- Lifecycle ---------- */

/* Set up PWM, encoder interrupts, queues. Call once before the scheduler starts. */
bool motion_init(void);

/* Create MotionTask. Call after motion_init(). Task enters IDLE when ready. */
bool motion_start(void);

/* ---------- Distance / angle commands (non-blocking, require IDLE) ---------- */

motion_err_t motion_move(int32_t distance_mm, int32_t speed_mmps);  /* <0 = backward */
motion_err_t motion_turn(int32_t angle_deg, int32_t rate_dps);      /* + = left */

/* Names matching the project write-up; thin wrappers over the two above */
motion_err_t motion_move_forward(int32_t distance_mm);
motion_err_t motion_move_backward(int32_t distance_mm);
motion_err_t motion_turn_left(int32_t angle_deg);
motion_err_t motion_turn_right(int32_t angle_deg);

/* ---------- Continuous velocity commands (allowed in any non-fault state) ---------- */

/* For line following: call every control cycle; each call replaces the setpoint.
 * left  = speed * (100 - steer) / 100
 * right = speed * (100 + steer) / 100                                        */
motion_err_t motion_drive(int32_t speed_mmps, int32_t steer_pct);
motion_err_t motion_set_wheel_speeds(int32_t left_mmps, int32_t right_mmps);

/* ---------- Stop / completion ---------- */

/* hard = false: ramp down. hard = true: brake immediately. Valid in any state. */
void motion_stop(bool hard);

/* Block until the current motion completes, is stopped, faults, or times out. */
motion_result_t motion_wait_done(uint32_t timeout_ms);

bool motion_is_busy(void);

/* ---------- Status ---------- */

void motion_get_status(motion_status_t *out);
void motion_reset_odometry(void);        /* zero distance and heading */
void motion_clear_fault(void);           /* FAULT -> IDLE */

#ifdef MOTION_CALIBRATION_BUILD
/* ---------- Bring-up / calibration (development firmware only) ----------
 * Used for Phases 1-4 from a serial test harness. They return
 * MOTION_ERR_INVALID unless the controller is in CALIBRATE mode.          */

motion_err_t motion_enter_calibration(void);
motion_err_t motion_exit_calibration(void);
motion_err_t motion_raw_set_pwm(int32_t left_pm, int32_t right_pm); /* -1000..1000 */
void         motion_raw_get_ticks(int32_t *left, int32_t *right);
#endif /* MOTION_CALIBRATION_BUILD */

// #ifdef __cplusplus
// }
// #endif

#endif /* MOTION_CONTROL_H */
