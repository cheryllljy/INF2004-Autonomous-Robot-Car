/*
 * motion_config.h - MotionControl pins and tunables (NFR26).
 * Values marked TODO still need a measurement or a decision.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---------- Pins ---------- */
/* GP2, edge IRQ */
#define PIN_ENC_L (2U)
/* GP3 */
#define PIN_ENC_R (3U)
/* PWM slice 4, channel A */
#define PIN_M1_A (8U)
/* PWM slice 4, channel B */
#define PIN_M1_B (9U)
/* PWM slice 5, channel A */
#define PIN_M2_A (10U)
/* PWM slice 5, channel B */
#define PIN_M2_B          (11U)
#define MOTION_LEFT_IS_M1 (1) /* M1 = left, M2 = right */
/* 1 if PWM on input A turns the LEFT wheel forward */
#define MOTOR_L_FORWARD_IS_A (1)
/* same for the RIGHT wheel (mirrored motors: often 0) */
#define MOTOR_R_FORWARD_IS_A (1)
#define MOTOR_PWM_FREQ_HZ    (20000U)

/* ---------- Encoder (measured, both edges counted) ---------- */
/* left 12701, right 12732 ticks per 10 turns */
#define ENCODER_TICKS_PER_REV (1271.7f)
/* every transition is a tick (used by motion_cal.c) */
#define ENCODER_EDGES_PER_SLOT (2)
/* ignore edges closer than this (chatter) */
#define ENCODER_MIN_EDGE_US (100U)

/* ---------- Geometry ---------- */
#define WHEEL_DIAMETER_MM      (65.0f)  /* mm */
#define TRACK_WIDTH_MM         (112.0f) /* mm */
#define MC_PI                  (3.14159265f)
#define WHEEL_CIRCUMFERENCE_MM (MC_PI * WHEEL_DIAMETER_MM)
/* TODO: correct with the 1-2 m drive test */
#define DIST_SCALE (1.000f)
#define MM_PER_TICK \
    ((WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV) * DIST_SCALE)

/* ---------- Motor model (measured on the floor, 20 kHz PWM) ---------- */
/* duty in permille (0..1000) */
#define MOTOR_PWM_MAX (1000)
/* speed is roughly 5 ticks/s per pm above this */
#define MOTOR_FF_OFFSET_PM    (50)
#define MOTOR_FF_TICKS_PER_PM (5.0f)
/* duty that reliably starts a stopped wheel */
#define MOTOR_BREAKAWAY_PM (120)
/* optional lowest duty for a moving wheel, SET_SPEED mode only; 0 = off.
 * Stops a slow inner wheel stalling, but is also the slowest the wheel can
 * go, so it widens tight curves. Set just above the floor dead-zone duty. */
#ifndef MOTOR_MIN_RUN_PM
#define MOTOR_MIN_RUN_PM (0)
#endif
/* cap on the duty the controller may use: full power is far above
 * the 300 mm/s cruise, so a broken encoder cannot make the car race */
#define MOTION_DUTY_MAX_PM (600)
/* duty at the cap but wheel speed < half the setpoint for this long = fault */
#define MOTION_RUNAWAY_MS (500U)
/* left/right matched within about 3% */
#define MOTOR_L_GAIN (1.000f)
#define MOTOR_R_GAIN (1.000f)

/* ---------- Control loop ---------- */
/* NFR5: 20 ms, jitter within +/-2 ms   */
#define MOTION_CONTROL_PERIOD_MS (20U)
/* speed averaged over 2 x 20 ms = 40 ms: short enough to follow the
 * steering ramp; 4 gives less duty noise if the wheels sound rough. */
#ifndef SPEED_WINDOW_PERIODS
#define SPEED_WINDOW_PERIODS         (2U)
#endif
#define MOTION_MAX_SPEED_MMPS        (300)
#define MOTION_DEFAULT_SPEED_MMPS    (200)
#define MOTION_DEFAULT_TURN_RATE_DPS (90)
/* slowest speed used while finishing a move */
#define MOTION_MIN_SPEED_MMPS (40)
/* trapezoid ramp up and down */
#define MOTION_ACCEL_MMPS2 (400)
/* SET_SPEED mode: the mean speed ramps at MOTION_ACCEL_MMPS2, but the
 * left/right split changes this fast (mm/s^2 of wheel difference), so
 * steering changes show up quickly. 0 = plain per-wheel ramp. */
#ifndef MOTION_STEER_ACCEL_MMPS2
#define MOTION_STEER_ACCEL_MMPS2 (2000)
#endif
/* below this a wheel counts as stopped */
#define MOTION_STOPPED_SPEED_MMPS (10)
/* start braking this much earlier: speed estimate and motor lag */
#define MOTION_STOP_LAG_S (0.10f)
/* permille per mm/s. Stiff enough to hold the wheel speed difference at
 * bend entry (the inner wheel gets pushed along, the outer one loaded).
 * Check: a wheel at 15-25 mm/s must not sag or pulse. */
#define MOTION_SPEED_KP (1.6f)
/* permille per (mm/s * s), TODO tune */
#define MOTION_SPEED_KI (3.0f)
/* mm/s of correction per mm of left/right error TODO tune */
#define MOTION_HEADING_KP (2.0f)

/* ---------- Faults ---------- */
/* high PWM + no ticks for this long */
#define MOTION_STALL_TIMEOUT_MS (300U)
/* "high PWM" threshold for stall check */
#define MOTION_STALL_MIN_PWM_PM (300)
/* TODO: reject line data older than this */
#define MOTION_LINE_STALE_MS (50U)
/* TODO: steer applied on LINE_DRIFT_* */
#define LINE_STEER_DRIFT_PCT (20)

/* ---------- Tolerances (NFR12, NFR13) ---------- */
#define MOTION_DIST_TOL_MM  (3)
#define MOTION_TURN_TOL_DEG (2)

/* ---------- micro T-Kernel objects ---------- */
/* uT-Kernel: LOWER number = HIGHER priority */
#define MOTION_TASK_PRIORITY (1)
/* TODO: size from measured usage */
#define MOTION_TASK_STACK_BYTES (2048U)
/* messages the motion_cmd buffer holds */
#define MOTION_CMD_QUEUE_DEPTH (4U)
/* ms motion_get_status() waits at most for the status lock, so one stuck
 * holder cannot freeze every caller. The control task never waits for it. */
#ifndef MOTION_STATUS_WAIT_MS
#define MOTION_STATUS_WAIT_MS (5)
#endif

#endif /* MOTION_CONFIG_H */
