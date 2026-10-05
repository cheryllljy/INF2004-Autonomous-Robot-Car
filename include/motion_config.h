/*
 * motion_config.h - Buddy 2 (Motion Control) configuration
 *
 * Every value marked TODO is a PLACEHOLDER. Replace it with a real
 * measurement as you complete Phases 1-4. Nothing here is calibrated yet.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---------- Pins (TODO: take from the ROBO PICO pinout) ---------- */
#define PIN_MOTOR_L_PWM     0   /* TODO */
#define PIN_MOTOR_L_DIR     0   /* TODO */
#define PIN_MOTOR_R_PWM     0   /* TODO */
#define PIN_MOTOR_R_DIR     0   /* TODO */
#define PIN_ENC_L           2
#define PIN_ENC_R           3   

/* ---------- Physical constants: measure / count by inspection ---------- */
#define WHEEL_DIAMETER_MM       65.0f   /* Diameter = 6.5cm = 65mm */ 
#define TRACK_WIDTH_MM          130.0f  /* TODO: centre-to-centre of wheels;
                                           refine in Phase 4 spin test */
#define ENCODER_SLOTS_PER_REV   20      /* datasheet PPR, PWM frequency : 20 kHz  */
#define ENCODER_EDGES_PER_SLOT  2       /* 1 = rising only, 2 = both edges */
#define ENCODER_TO_WHEEL_RATIO  1.0f    /* >1 if encoder is on motor shaft
                                           (motor revs per wheel rev) */

/* ---------- Derived constants (do not edit) ---------- */
#define MC_PI                   3.14159265f
#define ENCODER_TICKS_PER_REV   ((float)ENCODER_SLOTS_PER_REV * \
                                 ENCODER_EDGES_PER_SLOT * ENCODER_TO_WHEEL_RATIO)
#define WHEEL_CIRCUMFERENCE_MM  (MC_PI * WHEEL_DIAMETER_MM)

/* ---------- Calibration results (Phase 2-4; defaults are neutral) ----------
 * Measured offline with the calibration firmware (build with
 * -DMOTION_CALIBRATION_BUILD), then pasted here as compile-time constants.
 * The race firmware never calibrates at run time. */
#define DIST_SCALE              1.000f  /* from the 1-2 m drive test */
#define MM_PER_TICK             ((WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV) \
                                 * DIST_SCALE)
#define MOTOR_L_DEADZONE_PM     0       /* TODO: duty permille where wheel moves */
#define MOTOR_R_DEADZONE_PM     0       /* TODO */
#define MOTOR_L_GAIN            1.000f  /* TODO: left/right mismatch trim */
#define MOTOR_R_GAIN            1.000f  /* TODO */

/* ---------- Design choices (tunable) ---------- */
#define MOTOR_PWM_MAX           1000    /* duty in permille (0..1000) */
#define MOTION_CONTROL_PERIOD_MS    20  /* control loop period */
#define SPEED_WINDOW_PERIODS        5   /* speed averaged over N periods;
                                           choose so window holds >= ~4 ticks
                                           at the lowest speed you control */
#define MOTION_MAX_SPEED_MMPS       300 /* TODO: set after Phase 3 */
#define MOTION_ACCEL_MMPS2          400 /* trapezoid profile ramp */
#define MOTION_STALL_TIMEOUT_MS     300 /* high PWM + no ticks for this long */
#define MOTION_STALL_MIN_PWM_PM     300 /* "high PWM" threshold for stall check */

/* ---------- Tolerances ---------- */
#define MOTION_DIST_TOL_MM      3
#define MOTION_TURN_TOL_DEG     2

/* ---------- RTOS ---------- */
#define MOTION_TASK_PRIORITY    4       /* above sensor tasks, below ISR work */
#define MOTION_TASK_STACK_WORDS 512
#define MOTION_CMD_QUEUE_LEN    4

#endif /* MOTION_CONFIG_H */
