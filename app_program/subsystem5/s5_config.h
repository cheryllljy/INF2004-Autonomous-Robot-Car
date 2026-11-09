/*
 *      s5_config.h
 *      Subsystem 5 (ultrasonic + servo obstacle avoidance), rewritten from
 *      scratch by nyx on 2026-11-07. Every pin, size and tuning number of
 *      subsystem 5 lives here; a wiring or tuning change never needs a .c
 *      edit. The values marked CALIBRATE come from the calibration build
 *      (S5_CAL_BUILD, testing/subsystem5/CALIBRATION.md).
 *
 *      Conventions used everywhere in subsystem 5
 *        - lengths in mm, angles in 0.1 deg ("ddeg") in the integer API
 *        - + angle = LEFT (counter-clockwise), like Buddy 2's motion API
 *        - servo angle 0 = straight ahead
 *        - the car frame has its origin at the middle of the wheel axle,
 *          +x forward, +y left
 *        - the line frame (used during a bypass) has its origin on the
 *          line where the IR pair was when the obstacle was seen, +x along
 *          the line in the driving direction, +y left of the line
 */

#ifndef S5_CONFIG_H
#define S5_CONFIG_H

/* ---------------------------------------------------------------------
 *  Pins (unchanged wiring). GP17 is the B input of PWM slice 0: the
 *  slice counts microseconds while ECHO is high, so no GPIO interrupt is
 *  needed (the one IO_BANK0 handler belongs to gpio_dispatch.c).
 *  ECHO goes through the level converter: never wire the HC-SR04's 5 V
 *  ECHO straight to GP17.
 * --------------------------------------------------------------------- */
#define S5_SERVO_GPIO      (15U) /* PWM slice 7 B, Robo Pico SERVO 4 */
#define S5_TRIG_GPIO       (16U) /* plain output */
#define S5_ECHO_GPIO       (17U) /* PWM slice 0 B, gated input */
#define S5_SERVO_PWM_SLICE (7U)
#define S5_ECHO_PWM_SLICE  (0U)

/* ---------------------------------------------------------------------
 *  Task
 * --------------------------------------------------------------------- */
#define S5_TASK_PRI (20) /* ObstacleScan band (TEAM.md section 4) */
#define S5_POLL_MS  (10) /* sensor polls, from a cyclic handler */

/* ---------------------------------------------------------------------
 *  Car geometry, mm, from the middle of the wheel axle
 * --------------------------------------------------------------------- */
#define S5_SONAR_AHEAD_MM    (70)  /* sensor face (nyx, 2026-11-07) */
#define S5_IR_AHEAD_MM       (125) /* IR_L / IR_R pair (nyx, 2026-11-06) */
#define S5_IR_HALF_GAP_MM    (15)  /* each IR this far off the centre */
#define S5_LINE_HALF_MM      (9)   /* 18 mm tape */
#define S5_CAR_FRONT_MM      (135) /* front of the car */
#define S5_CAR_REAR_MM       (45)  /* back of the car (behind the axle) */
#define S5_CAR_HALF_WIDTH_MM (65)  /* 130 mm wide */

/* ---------------------------------------------------------------------
 *  Servo (SG90 on the sensor bracket)
 * --------------------------------------------------------------------- */
/* How far the bracket can turn each way without touching anything. */
#define S5_SERVO_LIMIT_DDEG (600)
/* CALIBRATE step 1: 1 if "left" turns the sensor to the right. */
#ifndef S5_SERVO_INVERT
#define S5_SERVO_INVERT (0)
#endif
/* CALIBRATE step 1: added to every angle so 0 really is straight ahead.
 * nyx, 2026-11-08: -52 (run 1: box middle 5.2 deg right; run 2 with -52:
 * the box-only echoes and the range profile are centred). */
#ifndef S5_SERVO_TRIM_DDEG
#define S5_SERVO_TRIM_DDEG (-52)
#endif
#define S5_SERVO_PULSE_MIN_US (500U)  /* physical 0 deg (full right) */
#define S5_SERVO_PULSE_MAX_US (2500U) /* physical 180 deg (full left) */
/* Loaded SG90: 60 deg in 140 ms, then ringing dies down. */
#define S5_SERVO_MS_PER_60DEG (140)
#define S5_SERVO_SETTLE_MS    (30)

/* ---------------------------------------------------------------------
 *  HC-SR04P
 * --------------------------------------------------------------------- */
#define S5_PING_PERIOD_MS (60) /* datasheet: at least 60 ms apart */
/* Move the servo this long after a trigger: every echo from closer than
 * 1.7 m is back by then and the counter holds it. */
#define S5_ECHO_DONE_MS (10)
/* Read the echo this long after the trigger: S5_RANGE_MAX_MM there and
 * back is 11.6 ms. ECHO still high then = nothing within range (clear). */
#define S5_ECHO_READ_MS (12)
#define S5_AIR_TEMP_C   (25)
/* CALIBRATE step 2: added to every range.
 * nyx, 2026-11-08: 0. Run 1 read 11 mm short, run 2 up to 15 mm long;
 * all eight placements average about +3 mm. */
#ifndef S5_RANGE_OFFSET_MM
#define S5_RANGE_OFFSET_MM (0)
#endif
#define S5_RANGE_MIN_MM (20)   /* closer reads as 20 */
#define S5_RANGE_MAX_MM (2000) /* further counts as nothing there */
/* FR25: this many pings in a row without any answer (ECHO never rose)
 * is a dead sensor, never a clear path. */
#define S5_DEAD_PINGS (5U)

/* ---------------------------------------------------------------------
 *  Detection while line following (FR21: within 300 mm)
 * --------------------------------------------------------------------- */
#define S5_DETECT_MM      (300) /* nyx, 2026-11-07: 30 cm */
#define S5_DETECT_CONFIRM (2U)  /* pings in a row at or under it */
#define S5_STALE_MS       (250) /* NFR12: an older range is unknown */
/* FR25: driving with no new ping for this long is a sensor fault */
#define S5_SILENT_MS (1000)

/* ---------------------------------------------------------------------
 *  Scan (the car stands still)
 * --------------------------------------------------------------------- */
#define S5_SCAN_STEP_DDEG (60) /* coarse sweep, +60 .. -60 deg */
#define S5_SCAN_FINE      (2U) /* halvings per edge: 6 -> 3 -> 1.5 deg */
/* the fine probes come at the edge from the left, like the sweep, so the
 * servo's gear backlash is the same for every sample */
#define S5_SCAN_OVERSHOOT_DDEG (40)
/* only these central bearings can hold the face the guard stopped for */
#define S5_FACE_CONE_DDEG (300)
/* a return this far behind the face is something else, not the object */
#define S5_EDGE_DEPTH_MM (120)
/* closer than this, the edges may be outside the sweep: back off first */
#define S5_SCAN_MIN_MM  (150)
#define S5_SCAN_DIST_MM (250) /* ... to here */
/* an edge outside the sweep: back off this much more and scan again */
#define S5_RESCAN_BACK_MM  (150)
#define S5_MAX_RESCANS     (2U)
#define S5_SCAN_TIMEOUT_MS (5000)

/* CALIBRATE step 3: beam half-angle, ddeg. The echoes of an object go on
 * this far past each of its corners as the sensor turns (the HC-SR04's
 * datasheet says 7.5 deg; a flat box face answers ~22 deg off-axis).
 * Measured with the box at two distances; in between it is interpolated,
 * outside it is held. */
/* nyx, 2026-11-08: two calibration runs gave NEAR 30 and -24, FAR 257
 * and 244. On the track (face 277..283 mm) 235 reads the 21 cm face as
 * 210 mm and the 16 cm face as 185 mm; 155 read them 355 and 345. */
#ifndef S5_BEAM_NEAR_DDEG
#define S5_BEAM_NEAR_DDEG (0)
#endif
#ifndef S5_BEAM_FAR_DDEG
#define S5_BEAM_FAR_DDEG (235)
#endif
#define S5_BEAM_NEAR_MM (200)
#define S5_BEAM_FAR_MM  (300)

/* Both sides within this of each other: pick one at random. */
#define S5_TIE_MM (10)

/* ---------------------------------------------------------------------
 *  Bypass path. Smooth "potential field" shape on a straight line: the
 *  line pulls the car back, the object pushes it out. Planned as
 *  minimum-jerk ramps (no corner anywhere, curvature 0 where a ramp
 *  starts and ends) and tracked from the encoders.
 * --------------------------------------------------------------------- */
#define S5_CLEARANCE_MM    (45)  /* car side to the object's edge */
#define S5_MAX_OFFSET_MM   (400) /* never further from the line than this */
#define S5_MAX_HEADING_DEG (40)  /* steepest point of a ramp */
#define S5_MIN_RADIUS_MM   (150) /* tightest point of a ramp */
/* nyx, 2026-11-08: no room for that ramp out (the car stops 30 cm from
 * the object): a steeper one, down to this, before backing up at all */
#define S5_STEEP_HEADING_DEG (60)
#define S5_STEEP_RADIUS_MM   (110)
/* nyx, 2026-11-08: the ramp back in, sharper than the ramp out (less for
 * the encoders to drift over; the IRs pick the line up) */
#ifndef S5_IN_HEADING_DEG
#define S5_IN_HEADING_DEG (50)
#endif
#ifndef S5_IN_RADIUS_MM
#define S5_IN_RADIUS_MM (130)
#endif
/* While a ramp turns the car out or back in, no part of the car comes
 * closer to the object than this (checked on the planned path, corners
 * of the car included; the straight pass keeps S5_CLEARANCE_MM). */
#define S5_RAMP_MARGIN_MM (30)
/* object depth (front to back) when the look across cannot fix the far
 * corner: the car turns back in after this much object (or later, if the
 * sensor heard something further on). Set it to the deepest object on the
 * course. Also how far another object found close behind the first is
 * taken to reach (the car then passes both). */
#define S5_DEPTH_DEFAULT_MM (200)
#define S5_DEPTH_MIN_MM     (60)
/* the deepest the far corner may be: one object, or two close together */
#define S5_DEPTH_MAX_MM (1000)

#define S5_SPEED_MM_S         (100) /* along the bypass */
#define S5_SLOW_MM_S          (60)  /* settling onto the line */
#define S5_REVERSE_SPEED_MM_S (60)
#define S5_REVERSE_MAX_MM     (500) /* never back up further, in total */
/* Buddy 2 gives a wheel no drive at 5 mm/s or less and kicks one under
 * 10 mm/s, so a moving wheel is never asked for less than this */
#define S5_MIN_WHEEL_MM_S (15)

/* tracking: errors die out over about this distance (critically damped) */
#define S5_TRACK_DIST_MM (80)
#define S5_ALIGN_DIST_MM (60)
#define S5_MAX_CURV_PPM  (10000) /* tightest command, 1e6 / radius mm */

/* CALIBRATE step 4: how far the car really turns for 100 deg of encoder
 * turn (wheels slip and scrub). nyx's junction pivots measured 1/1.3 ..
 * 1/1.6 (63..77); the smooth bypass arcs slip less. Used both to dead
 * reckon and to steer. */
/* nyx, 2026-11-08: the 40 cm left circle ended ~10 deg past the tape
 * (370 for 360 by encoder): 100 x 370 / 360. Second circle (step 4 with
 * the compass) stopped short, 355 deg by the compass: 103 x 355 / 360. */
#ifndef S5_TURN_PCT
#define S5_TURN_PCT (102)
#endif

/* Passing, the sonar looks this far towards the object and ranges its far
 * corner from several places (the "repulsive" part, measured, not
 * assumed): how deep the object is (where the way back starts) and where
 * its side really is. A side closer than the sweep said moves the rest of
 * the pass further out. */
#define S5_WATCH_DDEG     (400)
#define S5_WATCH_RANGE_MM (450)
#define S5_SIDE_TOL_MM    (12) /* closer than planned by more: move out */
#define S5_SIDE_MAX_MM    (60) /* moves out by at most this */

/* ---------------------------------------------------------------------
 *  Back onto the line
 * --------------------------------------------------------------------- */
/* coming back, an IR seeing black further than this from where the line
 * should be is another lane's line, not ours */
#define S5_LINE_WINDOW_MM (100)
/* nyx, 2026-11-08: ... and the pick-up (both IRs crossing it) this far.
 * On the track the encoders put the line 7-12 cm off on the way back;
 * S5_CAP_LINE_DEG keeps a junction's branch near it out */
#define S5_CAP_WINDOW_MM (150)
/* nyx, 2026-11-08: a line the IRs cross more than this off the direction
 * of ours (from where each IR met it, in the encoders' frame) is a
 * junction's branch, not ours. The encoders drift a few degrees on the
 * way round. */
#define S5_CAP_LINE_DEG (20)
/* ... and both IRs black at once is our line only with the car crossing
 * at least this steeply (30 mm between the IRs, 18 mm tape: 53 deg, less
 * the drift); less steep, a line across ours */
#define S5_CAP_BOTH_DEG (45)
/* line not met at the end of the ramp: carry on at this angle ... */
/* nyx, 2026-11-08: 12 -> 30. On the track the car ended 5 cm beside the
 * line with the encoders some degrees off; at 12 deg it never got there */
#define S5_SEEK_DEG    (30)
#define S5_SEEK_MAX_MM (400) /* ... this far (FR24: within 500 mm) */
/* nyx, 2026-11-08: nothing black on the way back: looking on, the first
 * IR black this near where the line should be is our line */
#define S5_SEEK_WINDOW_MM (300)
/* settled: IR pair centred on the line within this and this straight */
#define S5_ALIGN_TOL_MM  (5)
#define S5_ALIGN_TOL_DEG (3)
#define S5_ALIGN_MAX_MM  (250) /* hand over anyway after this */

/* line history while following, for the line's direction: up to 1 m of
 * the IR pair's track. Buddy 3 drives straight while both IRs are white,
 * so the track zig-zags about 1.5 deg either side of the line; a fit over
 * a long stretch averages that out (over 300 mm it can be 1.5 deg off,
 * which is 26 mm at the end of a 1 m bypass). */
#define S5_HIST_STEP_MM (20)
#define S5_HIST_LEN     (50U)
#define S5_HIST_MIN     (8U) /* fewer: use the car's heading */

/* ---------------------------------------------------------------------
 *  Compass (nyx, 2026-11-08)
 *
 *  Buddy 4's LSM303D has no gyro, but its magnetometer gives the car's
 *  heading. The encoders' heading drifts a few degrees on the way round
 *  (the line came back 7-12 cm off on the track); the compass pulls it
 *  back. The motors' magnets move the field's centre, so it needs
 *  calibration step 4 (make S5CAL=2 runs that step only): it prints
 *  these four. S5_MAG_SIGN 0 = not calibrated: compass not used.
 * --------------------------------------------------------------------- */
#ifndef S5_MAG_SIGN
/* nyx, 2026-11-08: step 4 on the floor: x -4227..3931, y -5626..3625,
 * 355 deg by the compass for 360 by the encoders, but up to 13 deg apart
 * on the way round, so S5_MAG_USE 0 below (logged only) */
#define S5_MAG_SIGN (1)  /* +1 / -1: which way it turns with the car */
#endif
#ifndef S5_MAG_X0
#define S5_MAG_X0 (-148) /* the field's centre, counts (0.16 mgauss) */
#endif
#ifndef S5_MAG_Y0
#define S5_MAG_Y0 (-1000)
#endif
#ifndef S5_MAG_R
#define S5_MAG_R (4367)  /* the field's strength round the centre */
#endif
/* 1: the compass corrects the encoders' heading; 0: only logged.
 * nyx, 2026-11-08: 0 until a track run shows it agrees with the IRs */
#ifndef S5_MAG_USE
#define S5_MAG_USE (0)
#endif
#define S5_MAG_TAU_MS  (1000) /* pulls the heading over about this long */
/* ... by at most this per 100 mm the wheels go (encoders 3 % apart drift
 * about 1.5 deg per 100 mm); turning on the spot, a wheel goes
 * S5_MAG_ARM_MM per radian */
#define S5_MAG_RATE_DEG (2)
#define S5_MAG_ARM_MM   (56.0f)
/* nyx, 2026-11-08: the compass and encoders this far apart or less: left
 * alone. A compass is a few degrees off at some headings even when
 * calibrated (iron near the IMU); good encoders are better than that over
 * a bypass, so only drift past this is pulled back (in the simulator,
 * with 2 deg of compass error, pulling at every degree took the car into
 * a second box in C35 and C38) */
#ifndef S5_MAG_DEAD_DEG
#define S5_MAG_DEAD_DEG (3)
#endif
#define S5_MAG_TOL_PCT (25)   /* field strength off by more: iron nearby */
#define S5_MAG_MAX_DEG (30)   /* compass and encoders further apart: skip */
#define S5_MAG_AGE_MS  (100)  /* a sample older than this is not used */

/* ---------------------------------------------------------------------
 *  Episode limits
 * --------------------------------------------------------------------- */
#define S5_STILL_MM_S      (10)    /* both wheels under this: standing */
#define S5_STILL_MS        (100)   /* ... for this long */
#define S5_STOP_WAIT_MS    (1500)  /* go on anyway after this */
#define S5_EPISODE_MS      (60000) /* whole bypass, blocked time excluded */
#define S5_BLOCKED_SCAN_MS (3000)  /* blocked: look again this often */
#define S5_CLEAR_MS        (1000)  /* blocked: path clear this long = go */
#define S5_CLEAR_MM        (450)   /* ... clear means nothing within this */
/* a wheel asked to move that gives no encoder ticks for this long */
#define S5_DEAD_WHEEL_MS (600)
/* nyx, 2026-11-08: ... stops the car this long and goes on, this many
 * times per bypass, before the bypass fails (a wheel turning on the spot
 * slowly can stall; a dead encoder still fails, 1.5 s later) */
#define S5_WHEEL_TRIES   (1)
#define S5_WHEEL_REST_MS (300)

/* ---------------------------------------------------------------------
 *  Line search (nyx, 2026-11-08): robot.ps1 search. The car starts off
 *  the line (on white), drives straight until the IRs meet a line, drives
 *  its axle onto it and turns on the spot to face along it, then Buddy 3
 *  follows it. s5_search_begin() in s5_avoid.c.
 * --------------------------------------------------------------------- */
#define S5_SEARCH_MM_S (60)   /* straight on at this speed */
#define S5_SEARCH_MAX_MM (2000) /* no line within this: stop (fault 0x40) */
/* one IR black: the other must meet the line within this (lines from
 * 11 deg to the car up; shallower ones Buddy 3 takes) ... */
#define S5_SEARCH_PAIR_MM (150)
/* ... and one IR black this long with the other still white is a line
 * running along the car (or the car started on one): Buddy 3 follows it.
 * A line crossed at 27 deg or more leaves the IR within 40 mm (18 mm
 * tape); 60 handed a bend over too late for Buddy 3 (host test Q19). */
#ifndef S5_SEARCH_ALONG_MM
#define S5_SEARCH_ALONG_MM (40)
#endif
/* which way to face along a line straight across (both IRs black at
 * once, or within S5_SEARCH_SQUARE_DEG of square): +1 left, -1 right.
 * At any other angle the car turns the shorter way. */
#define S5_SEARCH_TURN (1)
#define S5_SEARCH_SQUARE_DEG (8)
/* the turn on the spot onto the line slows down this far from the end */
#ifndef S5_SEARCH_SLOW_DEG
#define S5_SEARCH_SLOW_DEG (20)
#endif
/* the turn on the spot onto the line (up to 90 deg) may take this long */
#define S5_SEARCH_PIVOT_MS (8000)

/* ---------------------------------------------------------------------
 *  Calibration build (testing/subsystem5/CALIBRATION.md): make S5CAL=1
 *  gives a _s5cal image whose S5 task runs the guided calibration
 *  instead of the obstacle logic. Flash the normal image afterwards.
 *  nyx, 2026-11-08: make S5CAL=2 (_s5cal4 image) runs step 4 only (the
 *  circle: turn and compass).
 * --------------------------------------------------------------------- */
#ifndef S5_CAL_BUILD
#define S5_CAL_BUILD (0)
#endif
#define S5_CAL_BOX_WIDTH_MM (210) /* nyx's box: 21 x 16 cm */
#define S5_CAL_RANGE_MM     (200) /* step 2 distance */
#define S5_CAL_SETUP_S      (15)  /* seconds to place things */
#define S5_CAL_CIRCLE_MM    (150) /* step 4: radius of the circle */

#endif /* S5_CONFIG_H */
