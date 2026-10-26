/*
 * motion_config.h - Buddy 2 (MotionControl) configuration, v3
 *
 * NFR26 asks for ONE configuration header for pins and tunables: paste this
 * into the team's config header, or #include it from there.
 * Values marked TODO still need a measurement or a decision.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---------- Pins (from the Week 6 pin table) ---------- */
#define PIN_ENC_L               (2U)    /* GP2, edge IRQ  // ori = 2                          */
#define PIN_ENC_R               (3U)    /* GP3            // ori = 3                          */
#define PIN_M1_A                (8U)    /* PWM slice 4, channel A                   */
#define PIN_M1_B                (9U)    /* PWM slice 4, channel B                   */
#define PIN_M2_A                (10U)   /* PWM slice 5, channel A                   */
#define PIN_M2_B                (11U)   /* PWM slice 5, channel B                   */
#define MOTION_LEFT_IS_M1       (1)     /* M1 = left, M2 = right */
#define MOTOR_L_FORWARD_IS_A    (1)     /* TODO: 1 if PWM on input A turns the LEFT wheel forward  */
#define MOTOR_R_FORWARD_IS_A    (1)     /* TODO: same for the RIGHT wheel (mirrored motors: often 0) */
#define MOTOR_PWM_FREQ_HZ       (20000U)

/* ---------- Encoder (measured: 10-turn hand test, both edges counted) ---------- */
#define ENCODER_TICKS_PER_REV   (1271.7f)   /* left 12701, right 12732 ticks per 10 turns */
#define ENCODER_EDGES_PER_SLOT  (2)         /* every transition is a tick (used by motion_cal.c) */
#define ENCODER_MIN_EDGE_US     (100U)      /* ignore edges closer than this (chatter)    */

/* ---------- Geometry ---------- */
#define WHEEL_DIAMETER_MM       (65.0f)     /* 6.5cm = 65mm       */
#define TRACK_WIDTH_MM          (112.0f)    /* 11.2cm == 112mm */
#define MC_PI                   (3.14159265f)
#define WHEEL_CIRCUMFERENCE_MM  (MC_PI * WHEEL_DIAMETER_MM)
#define DIST_SCALE              (1.000f)    /* TODO: correct with the 1-2 m drive test */
#define MM_PER_TICK             ((WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV) * DIST_SCALE)

/* ---------- Motor model (measured on the floor, 20 kHz PWM, battery state: note it) ---------- */
#define MOTOR_PWM_MAX           (1000)      /* duty in permille (0..1000)           */
#define MOTOR_FF_OFFSET_PM      (50)        /* speed is roughly 5 ticks/s per pm above this */
#define MOTOR_FF_TICKS_PER_PM   (5.0f)
#define MOTOR_BREAKAWAY_PM      (120)       /* duty that reliably starts a stopped wheel */
#define MOTION_DUTY_MAX_PM      (600)       /* cap on the duty the controller may use: full power is far above
                                               the 300 mm/s cruise, so a broken encoder cannot make the car race */
#define MOTION_RUNAWAY_MS       (500U)      /* duty at the cap but wheel speed < half the setpoint for this long = fault */
#define MOTOR_L_GAIN            (1.000f)    /* left/right matched within about 3% */
#define MOTOR_R_GAIN            (1.000f)

/* ---------- Control loop ---------- */
#define MOTION_CONTROL_PERIOD_MS    (20U)   /* NFR5: 20 ms, jitter within +/-2 ms   */
#define SPEED_WINDOW_PERIODS        (4U)    /* speed averaged over 4 x 20 ms = 80 ms */
#define MOTION_MAX_SPEED_MMPS       (300)
#define MOTION_DEFAULT_SPEED_MMPS   (200)
#define MOTION_DEFAULT_TURN_RATE_DPS (90)
#define MOTION_MIN_SPEED_MMPS       (40)    /* slowest speed used while finishing a move */
#define MOTION_ACCEL_MMPS2          (400)   /* trapezoid ramp up and down           */
#define MOTION_STOPPED_SPEED_MMPS   (10)    /* below this a wheel counts as stopped */
#define MOTION_STOP_LAG_S           (0.10f) /* start braking this much earlier: speed estimate and motor lag */
#define MOTION_SPEED_KP             (0.5f)  /* permille per mm/s          TODO tune */
#define MOTION_SPEED_KI             (3.0f)  /* permille per (mm/s * s)    TODO tune */
#define MOTION_HEADING_KP           (2.0f)  /* mm/s of correction per mm of left/right error TODO tune */

/* ---------- Faults ---------- */
#define MOTION_STALL_TIMEOUT_MS     (300U)  /* high PWM + no ticks for this long    */
#define MOTION_STALL_MIN_PWM_PM     (300)   /* "high PWM" threshold for stall check */
#define MOTION_LINE_STALE_MS        (50U)   /* TODO: reject line data older than this */
#define LINE_STEER_DRIFT_PCT        (20)    /* TODO: steer applied on LINE_DRIFT_*  */

/* ---------- Tolerances (NFR12, NFR13) ---------- */
#define MOTION_DIST_TOL_MM      (3)
#define MOTION_TURN_TOL_DEG     (2)

/* ---------- micro T-Kernel objects ---------- */
#define MOTION_TASK_PRIORITY    (1)         /* uT-Kernel: LOWER number = HIGHER priority */
#define MOTION_TASK_STACK_BYTES (2048U)     /* TODO: size from measured usage       */
#define MOTION_CMD_QUEUE_DEPTH  (4U)        /* messages the motion_cmd buffer holds */

// /* 1 = this module installs its own GPIO interrupt for the encoder pins.
//  * Set 0 once the team's shared GPIO dispatcher calls motion_encoder_isr(). */
// #define MOTION_HW_OWN_IRQ       (1)
// #define MOTION_HW_IRQ_NUMBER    (13U)       /* RP2040 IO_IRQ_BANK0                  */
// #define MOTION_HW_IRQ_LEVEL     (1)         /* TODO: check the template's EnableInt() levels */

#endif /* MOTION_CONFIG_H */