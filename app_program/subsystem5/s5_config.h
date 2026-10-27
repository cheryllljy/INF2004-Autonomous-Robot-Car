/**
 * @file    s5_config.h
 * @brief   Subsystem 5 private configuration: pins, timing, limits.
 *
 * Every board- or tuning-dependent number of subsystem 5 lives here.
 * A wiring change must never need a .c edit. Pins are claimed in
 * TEAM.md section 2; keep the two in step.
 */
#ifndef S5_CONFIG_H
#define S5_CONFIG_H

/*=====================================================================*/
/* Pins (GPIO numbers, not header pin numbers)                         */
/*=====================================================================*/

/*
 *   GP15  servo signal, Robo Pico SERVO 4 header.  PWM slice 7 B.
 *         GP14 (slice 7 A) must stay unused as PWM: same counter.
 *   GP16  HC-SR04 TRIG, Grove 4.  Plain GPIO output (SIO).
 *   GP17  HC-SR04 ECHO via the level converter (LV1).  Used as the
 *         PWM slice 0 B INPUT: the slice counts microseconds while
 *         ECHO is high, so no GPIO interrupt is needed.
 *         GP16 (slice 0 A) is SIO, so the slice output never reaches
 *         a pin. GP0/GP1 (also slice 0) are UART0, not PWM.
 *
 * Never connect the HC-SR04 5 V ECHO straight to GP17: RP2040 GPIO is
 * not 5 V tolerant (3.6 V abs max).
 */
#define S5_SERVO_GPIO               (15U)
#define S5_TRIG_GPIO                (16U)
#define S5_ECHO_GPIO                (17U)

/** PWM slice and channel follow from the pin: slice = (gpio >> 1) & 7,
 *  channel B = odd pin. Checked at compile time in the drivers. */
#define S5_SERVO_PWM_SLICE          (7U)
#define S5_ECHO_PWM_SLICE           (0U)

/*=====================================================================*/
/* Timing                                                              */
/*=====================================================================*/

/*
 * The kernel tick is 10 ms (CNF_TIMER_PERIOD). Every period below is a
 * whole number of ticks so the cyclic handler is exact.
 */

/** Guard ping period. Cannot go below the sensor's 60 ms quiet time. */
#define S5_GUARD_PERIOD_MS          (60U)

/** Close returns in a row before a scan (GUARD -> CONFIRM -> scan). */
#define S5_CONFIRM_SAMPLES          (2U)

/** A forward range older than this is reported as UNKNOWN. */
#define S5_STALE_MS                 (250U)

/** Consecutive hard sensor errors (no response / stuck) -> FAULT. */
#define S5_FAULT_LIMIT              (10U)

/** Consecutive good pings needed to leave a SENSOR fault. */
#define S5_RECOVER_SAMPLES          (3U)

/** Retry period while the sensor is faulted. */
#define S5_FAULT_RETRY_MS           (500U)

/** A no-echo this soon after a close return is the blind zone, not
 *  free space. */
#define S5_BLIND_WINDOW_MS          (400U)
#define S5_BLIND_RANGE_MM           (200U)

/* WaitStopped (tutor feedback: do not sweep while the car moves) */
#define S5_STOPPED_SPEED_MM_S       (5)     /* |speed| at or below = 0 */
#define S5_STOPPED_HOLD_MS          (100U)  /* must stay stopped this  */
#define S5_STOP_TIMEOUT_MS          (1000U) /* not stopped -> FAULT    */
#define S5_STOP_FALLBACK_MS         (500U)  /* no odometry registered  */

/** No obstacle_resume() within this while in ASSIST -> FAULT. */
#define S5_ASSIST_TIMEOUT_MS        (5000U)

/** REVERSE results in a row allowed; the next one is a FAULT. */
#define S5_MAX_REVERSES             (2U)

/*=====================================================================*/
/* Scan pattern                                                        */
/*=====================================================================*/

#define S5_COARSE_POINTS            (5U)
#define S5_FINE_POINTS              (7U)
#define S5_FINE_STEP_DEG            (10)
#define S5_SWEEP_LIMIT_DEG          (70)    /* last 10 deg grid point
                                               inside the +/-75 servo
                                               travel limit */

#endif /* S5_CONFIG_H */
