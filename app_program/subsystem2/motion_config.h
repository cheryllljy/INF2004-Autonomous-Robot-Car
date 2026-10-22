/*
 * motion_config.h - Buddy 2 (Motion Control) configuration
 *
 * Every value marked TODO is a PLACEHOLDER. Replace it with a real
 * measurement as you complete Phases 1-4. Nothing here is calibrated yet.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---------- Pins (TODO: take from the ROBO PICO pinout) ---------- */
#define PIN_ENC_L           (2U)   /* GP2 */
#define PIN_ENC_R           (3U)   /* GP3*/
#define PIN_M1_A            (8U)   /* PWM slice 4, ch A */
#define PIN_M1_B            (9U)   /* PWM slice 4, ch B */
#define PIN_M2_A            (10U)  /* PWM slice 5, ch A*/
#define PIN_M2_B            (11U)  /* PWM slice 5, ch B */
#define MOTION_LEFT_IS_M1   (1)    /* M1: Left, M2: Right*/
#define MOTOR_PWM_FREQ_HZ   (20000U)    

/* ---------- Physical constants: measure / count by inspection ---------- */
#define WHEEL_DIAMETER_MM       (65.0f)   /* Diameter = 6.5cm = 65mm */ 
#define TRACK_WIDTH_MM          (130.0f)  /* TODO: centre-to-centre of wheels;
                                           refine in Phase 4 spin test */
#define ENCODER_SLOTS_PER_REV   (20)      /* datasheet PPR, PWM frequency : 20 kHz  */
#define ENCODER_EDGES_PER_SLOT  (2)       /* 1 = rising only, 2 = both edges */
#define ENCODER_TO_WHEEL_RATIO  (1.0f)    /* >1 if encoder is on motor shaft
                                           (motor revs per wheel rev) */
#define ENCODER_MIN_EDGE_US     (500U)  /* confirm with debounce */

/* ---------- Derived constants (do not edit) ---------- */
#define MC_PI                   (3.14159265f)
#define ENCODER_TICKS_PER_REV   ((float)ENCODER_SLOTS_PER_REV * \
                                 (float)ENCODER_EDGES_PER_SLOT * ENCODER_TO_WHEEL_RATIO)
#define WHEEL_CIRCUMFERENCE_MM  (MC_PI * WHEEL_DIAMETER_MM)

/* ---------- Calibration results (Phase 2-4; defaults are neutral) ----------
 * Measured offline with the calibration firmware (build with
 * -DMOTION_CALIBRATION_BUILD), then pasted here as compile-time constants.
 * The race firmware never calibrates at run time. */
#define DIST_SCALE              (1.000f)  /* from the 1-2 m drive test */
#define MM_PER_TICK             ((WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV) * DIST_SCALE)
#define MOTOR_L_DEADZONE_PM     (0)       /* TODO: duty permille where wheel moves */
#define MOTOR_R_DEADZONE_PM     (0)       /* TODO */
#define MOTOR_L_GAIN            (1.000f)  /* TODO: left/right mismatch trim */
#define MOTOR_R_GAIN            (1.000f)  /* TODO */

/* ---------- Control timing (based on design) ---------- */
#define MOTOR_PWM_MAX           (1000)    /* duty in permille (0..1000) */
#define MOTION_CONTROL_PERIOD_MS    (20U)  /* control loop period */
#define SPEED_WINDOW_PERIODS        (5U)   /* speed averaged over N periods;
                                           choose so window holds >= ~4 ticks
                                           at the lowest speed you control */
#define MOTION_MAX_SPEED_MMPS       (300) /* TODO: set after Phase 3 */
#define MOTION_ACCEL_MMPS2          (400) /* trapezoid profile ramp */
// #define MOTION_STALL_TIMEOUT_MS     300 /* high PWM + no ticks for this long */
// #define MOTION_STALL_MIN_PWM_PM     300 /* "high PWM" threshold for stall check */

/* ---------- Faults ---------- */
#define MOTION_STALL_TIMEOUTS_MS    (300U)  /* high PWM + no ticks for this long*/
#define MOTION_STALL_MIN_PWM_PM     (300)   /* "high PWM" threshold for stall check */
#define MOTION_LINE_STATE_MS        (50U)   /* TODO: reject line data older than this */

/* ---------- Line following (TODO) ---------- */
#define LINE_STEER_DRIFT_PCT        (20)    /* apply steer on line_drift, corrects when drift occurs */

/* ---------- Tolerances ---------- */
#define MOTION_DIST_TOL_MM      (3)
#define MOTION_TURN_TOL_DEG     (2)

/* ---------- micro T-Kernel objects ---------- */
#define MOTION_TASK_PRIORITY    (1)       /* uT-Kernel (lower number = higher priority), MotionControl is highest */
#define MOTION_TASK_STACK_BYTES (2048U)     /* TODO: size from measured usage*/
#define MOTION_CMD_QUEUE_LEN    (4U)

#endif /* MOTION_CONFIG_H */
