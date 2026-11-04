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

/** ASSIST / WATCH watchdog. v3.0: a full bypass takes longer than 5 s,
 *  so this now means "no word from VehicleController for 5 s" (any
 *  command, or obstacle_assist_kick() which VC calls every loop). A
 *  controller that crashed mid-bypass still ends in FAULT. */
#define S5_ASSIST_TIMEOUT_MS        (5000U)

/** REVERSE results in a row allowed; the next one is a FAULT. */
#define S5_MAX_REVERSES             (2U)

/*=====================================================================*/
/* Scan pattern                                                        */
/*=====================================================================*/

#define S5_COARSE_POINTS            (5U)
#define S5_FINE_POINTS              (7U)
#define S5_FINE_STEP_DEG            (10)
/* v2.2: +/-30 deg. On the robot the bracket hits the Pico board
 * beyond that (bench, 2026-11-01). Remount the bracket to widen it, then
 * raise this, the coarse bearings and DRV_SERVO_MIN/MAX_DEG together. */
#define S5_SWEEP_LIMIT_DEG          (30)

/** v3.0 edge probes: extra pings per seen edge, each halving the
 *  interval where the edge can be (10 -> 5 -> 2.5 deg). */
#define S5_EDGE_PROBES              (2U)

/** A probe return counts as "the same object" within this of the
 *  nearest point (the geometry's depth tolerance). */
#define S5_PROBE_DEPTH_MM           (120U)

/*=====================================================================*/
/* v3.0: calibration build                                             */
/*=====================================================================*/

/* 1 = subsystem5_start() runs the guided calibration task
 * (s5_calibrate.c) instead of the obstacle task. Prints the values for
 * s5_calib.h. Set back to 0 for the robot. Same style as Buddy 2's
 * MOTION_DEMO switch. */
#ifndef S5_CAL_BUILD
#define S5_CAL_BUILD                (0)
#endif

/** Air temperature for the speed of sound, deg C. 331.3 + 0.606 T m/s.
 *  At 25 C that is 346.5 m/s; the v2 constant (343, 20 C) read 1 %
 *  long. The range calibration absorbs whatever is left. */
#define S5_AIR_TEMP_C               (25)

/** Diameter of the thin post used by calibration step 1, mm. A pen or
 *  a straw. Measured with a ruler; its own angular size is removed
 *  from the beam measurement. */
#define S5_CAL_POST_DIAMETER_MM     (10U)

/*=====================================================================*/
/* v3.0: WATCH (turn and scan while passing the obstacle)              */
/*=====================================================================*/

/* On the pass leg VehicleController asks S5 to WATCH one side. The
 * servo alternates: front ping, side ping at S5_WATCH_DEG, front ping...
 * so the car keeps a front guard AND measures the gap to the obstacle
 * the whole way past it. */

/** Side look angle. At the bracket limit today; 90 after a remount. */
#define S5_WATCH_DEG                (S5_SWEEP_LIMIT_DEG)

/** A side return whose sideways distance is under this is the obstacle
 *  beside us (further is scenery). */
#define S5_WATCH_SEEN_LAT_MM        (300U)

/** Side looks in a row with nothing beside us before "clearing". */
#define S5_WATCH_CLEAR_LOOKS        (2U)

/** Travel with the obstacle never seen beside us before giving up on
 *  the side data (phase BLIND: the controller falls back on the
 *  profile's pass distance). */
#define S5_WATCH_BLIND_MM           (300U)

/** Sideways clearance between the car's flank and the obstacle under
 *  which S5 raises S5_EVT_SIDE_CLOSE. */
#define S5_SIDE_CLOSE_MM            (40U)

/** Sensor axis to the back of the chassis, mm. TODO(measure). */
#define S5_REAR_FROM_SENSOR_MM      (150U)

/** Extra travel once the rear is level with the obstacle's far end. */
#define S5_PASS_MARGIN_MM           (60U)

/*=====================================================================*/
/* v3.0: reverse to remeasure                                          */
/*=====================================================================*/

/* Too close (under the 120 mm hard stop), or the obstacle fills the
 * +/-30 deg view so an edge is never seen: S5 publishes REVERSE with a
 * distance, VehicleController backs off slowly, then asks for a rescan.
 * The third REVERSE in a row is still FAULT REVERSE_LIMIT. */

/** Back off to at least this when inside the hard-stop range. */
#define S5_REMEASURE_NEAR_MM        (250U)

/** Back off to this when an edge was outside the view: at 400 mm the
 *  +/-30 deg sweep covers about +/-230 mm instead of +/-125 mm. */
#define S5_REMEASURE_FAR_MM         (400U)

#define S5_REVERSE_MIN_MM           (50U)
#define S5_REVERSE_MAX_MM           (250U)

#endif /* S5_CONFIG_H */
