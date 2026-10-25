/*
 * motion_control.h - Buddy 2 public API
 *
 * 1. VehicleController sends the motion commands through the motion_cmd message buffer (FR26)
 * 
 * 2. The e-stop flag (Emergency Stop) is checked every cycle (20ms) (NFR21)
 * 
 * 3. MotionControl data can be read using motion_get_status()
 * 
 * 4. ISRs can only call ISR-safe functions 
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

/* Set to 1 by the calibration build option (see the makefile). Same style as
 * the template's APP_TEST: tested with #if, so a value of 0 means "off". */
#ifndef MOTION_CALIBRATION_BUILD
#define MOTION_CALIBRATION_BUILD    (0)
#endif

/* 1 = build motion_demo.c: a mock VehicleController that drives scripted moves
 * for bench testing. Set to 1 temporarily, then back to 0. */
#ifndef MOTION_DEMO
#define MOTION_DEMO                 (1)
#endif


#include <stdint.h>
#include <stdbool.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

/* ---------- E-stop causes ---------- 
 * one bit per cause in estop event flag
 * Might want to put in shared kernel-object header. */
#define MOTION_ESTOP_VEHICLE    (0x00000001U)   /* VehicleController */
#define MOTION_ESTOP_IMU        (0x00000002U)   /* IMU */
#define MOTION_ESTOP_OBSTACLE   (0x00000004U)   /* ObstacleScan */

/* ---------- Event status to VehicleController ---------- 
 * for vehicle_evt flag to inform VehicleController whether 
 * motion commands was carried out/fault/estop 
 * Confirm with Team on usage */
 #define MOTION_EVT_DONE    (0x1U)  /* motion executed */
// #define MOTION_EVT_STOPPED (0x2U)  /* motion failed */
 #define MOTION_EVT_REJECTED (0x2U)  /* refuse command ! check with team what are the states*/
 #define MOTION_EVT_FAULT   (0x4U)  /* motion / encoder stalled */
 #define MOTION_EVT_ESTOP   (0x8U)  /* e-stop occurred */

/* ---------- Types ---------- */

typedef enum {
    MOTION_STATE_INIT = 0,
    MOTION_STATE_IDLE,
    MOTION_STATE_DRIVING,   /* move using distance or fixed speed */
    MOTION_STATE_TURNING,
    MOTION_STATE_STOPPING,  /* slow down till stop */
    MOTION_STATE_FAULT,
    MOTION_STATE_LINE_FOLLOWING,    /* steering from LineSense */
    MOTION_STATE_ESTOP
#ifdef MOTION_CALIBRATION_BUILD
    , MOTION_STATE_CALIBRATE,     /* development firmware only */
#endif
} motion_state_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_STALL_L,
    MOTION_FAULT_STALL_R,
    MOTION_FAULT_ENCODER_TIMEOUT,
    MOTION_FAULT_LINE_STALE         /* for line_following */
} motion_fault_t;

/* Returned when SUBMITTING a command */
typedef enum {
    MOTION_OK = 0,
    // MOTION_ERR_BUSY,        /* distance/turn command while not idle */
    MOTION_ERR_INVALID,     /* bad argument (e.g. speed <= 0, wrong mode) */
    // MOTION_ERR_FAULT,       /* controller is in FAULT; call motion_clear_fault() */
    MOTION_ERR_QUEUE_FULL   /* motion_cmd message buffer full */
} motion_err_t;

/* Returned when a motion COMPLETES (motion_wait_done) */
typedef enum {
    MOTION_RESULT_NONE = 0, /* Check why */
    MOTION_RESULT_DONE, /* target reached within tolerance */
    MOTION_RESULT_STOPPED,  /* interrupted by motion_stop() */
    // MOTION_RESULT_TIMEOUT,  /* wait timed out; motion may still be running */
    MOTION_RESULT_FAULT,    /* stall or encoder fault */
    MOTION_RESULT_ESTOP,
    MOTION_RESULT_REJECTED  /* command was refused */     
} motion_result_t;

/* motion_cmd message buffer format */
typedef enum {
    MOTION_CMD_MOVE = 0,
    MOTION_CMD_TURN,
    MOTION_CMD_LINE_FOLLOW,
    MOTION_CMD_SET_SPEED,
    MOTION_CMD_STOP,
    MOTION_CMD_RESET,       /* clear fault/e-stop then IDLE */
    MOTION_CMD_RESET_TRIP   /* reset distance to 0 */
} motion_cmd_type_t;

typedef struct {
    motion_cmd_type_t type;
    int32_t p1;             /* for: MOVE: distance mm (negative for backward)*/
                            /*      TURN: angle deg (negative for right) */
                            /*      LINE_FOLLOW: speed mm/s */
                            /*      SET_SPEED: left motor speed mm/s */

    int32_t p2;             /* for: MOVE: speed mm/s  */
                            /*      TURN: rate deg/s */
                            /*      SET_SPEED: right motor speed mm/s */
} motion_cmd_msg_t;

/* Snapshot for telemetry (Buddy 1) and decision logic (Buddies 3, 5) */
typedef struct {
    motion_state_t state;
    motion_fault_t fault;
    motion_result_t last_result;    /* outcome of most recent command */
    uint32_t  estop_sources;      /* latched estop bit */
    uint32_t  timestamp_ms;       /* time of event */
    uint32_t  cycle_count;        /* watchdog check-in for control cycle */
    uint32_t  overrun_count;      /* track late or missed control cycles */
    int16_t  speed_l_mmps;
    int16_t  speed_r_mmps;
    int32_t  ticks_l;
    int32_t  ticks_r;
    int32_t  dist_l_mm;             /* dist of left wheel */
    int32_t  dist_r_mm;             /* dist of right wheel */
    int32_t  distance_total_mm;     /* mean of both wheel */
    int32_t  distance_trip_mm;      /* dist since MOTION_CMD_RESET_TRIP */
    int32_t  heading_deg;           /* encoder-estimated, since last reset; + = left */
} motion_status_t;

/* ---------- Input from LineSense ---------- 
* Confirm with line sharing buddy 3
* read from lineSense? */

/* ---------- Lifecycle ---------- */

/* Set up PWM, encoder interrupts, queues. Call once before the scheduler starts. */
bool motion_init(void);

/* Create MotionTask. Call after motion_init(). Task enters IDLE when ready. */
bool motion_start(void);

/* ---------- Commands from VehicleController (non-blocking, require IDLE) ---------- */

motion_err_t motion_move(int32_t distance_mm, int32_t speed_mmps);  /* <0 = backward */
motion_err_t motion_turn(int32_t angle_deg, int32_t rate_dps);      /* + = left */
motion_err_t motion_line_follow(int32_t speed_mmps);
motion_err_t motion_set_wheel_speeds(int32_t leftmmps, int32_t right_mmps);
motion_err_t motion_stop(void);                                     /* slows down until stop*/
motion_err_t motion_reset(void);                                    /* clears fault/estop */
motion_err_t motion_reset_trip(void);

/* For barcode instructions */
motion_err_t motion_move_forward(int32_t distance_mm);
motion_err_t motion_move_backward(int32_t distance_mm);
motion_err_t motion_turn_left(int32_t angle_deg);
motion_err_t motion_turn_right(int32_t angle_deg);
motion_err_t motion_u_turn(void);

/* ---------- E-stop ---------- */
/* for tasks that uses tk_set_flg, set their source bit 
 * Takes effect at start of next control cycle */
void motion_estop_request(uint32_t source_bits);

/* Reset by VehicleController: clear bits then motion_reset() */
void motion_estop_clear(uint32_t source_bits);

/* PROPOSED: let MotionControl raise its MOTION_EVT_* bits on VehicleController's
 * vehicle_evt flag. flgid = that flag's ID, bit_shift = first bit of Motion's
 * 4-bit group. Until this is called, results are only visible in the status. */
void motion_attach_vehicle_evt(int32_t flgid, uint32_t bit_shift);

/* ---------- Status (for any tasks) ---------- */
void motion_get_status(motion_status_t *out);

/* ---------- ISR-safe (no mutexes) ---------- 
 * motion_encoder_isr: called by shared GPIO dispatcher for encoder pins - increment tick counter
 * motion_encoder_ticks_isr: lock-free read of both counters, for BarcodeISR
 * (edge ring buffer stores the encoder count with every edge) 
 * motion_ticks_to_mm: converts a tick count using calibrated constants */
void motion_encoder_isr(uint32_t gpio);
void motion_encoder_ticks_isr(int32_t *left, int32_t *right);
int32_t motion_ticks_to_mm(int32_t ticks);

// /* ---------- Continuous velocity commands (allowed in any non-fault state) ---------- */

// /* For line following: call every control cycle; each call replaces the setpoint.
//  * left  = speed * (100 - steer) / 100
//  * right = speed * (100 + steer) / 100                                        */
// motion_err_t motion_drive(int32_t speed_mmps, int32_t steer_pct);
// motion_err_t motion_set_wheel_speeds(int32_t left_mmps, int32_t right_mmps);

// /* ---------- Stop / completion ---------- */

// /* hard = false: ramp down. hard = true: brake immediately. Valid in any state. */
// void motion_stop(bool hard);

// /* Block until the current motion completes, is stopped, faults, or times out. */
// motion_result_t motion_wait_done(uint32_t timeout_ms);

// bool motion_is_busy(void);

// /* ---------- Status ---------- */

// void motion_get_status(motion_status_t *out);
// void motion_reset_odometry(void);        /* zero distance and heading */
// void motion_clear_fault(void);           /* FAULT -> IDLE */

#if MOTION_CALIBRATION_BUILD
/* ---------- Bring-up / calibration (development firmware only) ----------
 * Used for Phases 1-4 from a serial test harness. They return
 * MOTION_ERR_INVALID unless the controller is in CALIBRATE mode.          */

// motion_err_t motion_enter_calibration(void);
// motion_err_t motion_exit_calibration(void);
motion_err_t motion_raw_set_pwm(int32_t left_pm, int32_t right_pm); /* -1000..1000 */
void         motion_raw_get_ticks(int32_t *left, int32_t *right);

/* calibration tasks (motion_cal.c): call from subsystem2_init() & start() */
void motion_cal_init(void);
void motion_cal_start(void);
#endif /* MOTION_CALIBRATION_BUILD */

#if MOTION_DEMO
/* Bench demo (motion_demo.c): call from subsystem2_init() / subsystem2_start() */
void motion_demo_init(void);
void motion_demo_start(void);
#endif


// #ifdef __cplusplus
// }
// #endif

#endif /* MOTION_CONTROL_H */
