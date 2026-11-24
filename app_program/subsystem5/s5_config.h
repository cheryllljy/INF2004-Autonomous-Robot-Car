/*
 *      s5_config.h
 *      Subsystem 5 (ultrasonic + servo obstacle avoidance). Every pin,
 *      size and tuning number of subsystem 5 lives here; a wiring or
 *      tuning change never needs a .c edit. The values marked CALIBRATE
 *      come from the calibration build (S5_CAL_BUILD,
 *      testing/subsystem5/CALIBRATION.md).
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
/* ObstacleScan sits above LineSense (17), IMU (18) and the VC (22) */
#define S5_TASK_PRI (16)
#define S5_POLL_MS  (10) /* sensor polls, from a cyclic handler */

/* ---------------------------------------------------------------------
 *  Car geometry, mm, from the middle of the wheel axle
 * --------------------------------------------------------------------- */
#define S5_SONAR_AHEAD_MM    (70)  /* sensor face */
#define S5_IR_AHEAD_MM       (125) /* IR_L / IR_R pair */
/* 25 mm apart, measured */
#define S5_IR_HALF_GAP_MM    (12.5f) /* each IR this far off the centre */
#define S5_LINE_HALF_MM      (9)   /* 18 mm tape */
#define S5_CAR_FRONT_MM      (135) /* front of the car */
#define S5_CAR_REAR_MM       (45)  /* back of the car (behind the axle) */
#define S5_CAR_HALF_WIDTH_MM (65)  /* 130 mm wide */

/* ---------------------------------------------------------------------
 *  Servo (SG90 on the sensor bracket)
 * --------------------------------------------------------------------- */
/* How far the bracket can turn to the left without touching anything. */
#define S5_SERVO_LIMIT_DDEG (600)
/* ... and to the right: at 60 deg the sensor's wires hold the bracket
 * back, it stops short and comes back off by 5-10 deg. */
#ifndef S5_SERVO_LIMIT_R_DDEG
#define S5_SERVO_LIMIT_R_DDEG (500)
#endif
/* CALIBRATE step 1: 1 if "left" turns the sensor to the right. */
#ifndef S5_SERVO_INVERT
#define S5_SERVO_INVERT (0)
#endif
/* CALIBRATE step 1: added to every angle so 0 really is straight ahead. */
/* Auto trim at start-up. With an object put straight
 * ahead of the car (on the line, 100..400 mm from the sensor) when it is
 * switched on, the sensor sweeps it twice and corrects the trim below by
 * where the middle of its echoes reads (for this run only, nothing is
 * saved). Nothing there, or the two sweeps disagree: the trim below as it
 * is. 0: off. */
#ifndef S5_AUTO_TRIM
#define S5_AUTO_TRIM (1)
#endif
#define S5_AUTO_TRIM_WAIT_MS   (2000) /* after power-up, before the first */
#define S5_AUTO_TRIM_NEAR_MM   (100)  /* the object's face within these */
#define S5_AUTO_TRIM_FAR_MM    (400)
#define S5_AUTO_TRIM_MAX_DDEG  (250)  /* never correct by more (25 deg) */
#define S5_AUTO_TRIM_AGREE_DDEG (30)  /* the two sweeps within 3 deg */
/* The objects on the track are 80..250 mm wide. A
 * sweep from the line that reads narrower is widened to the least around
 * the middle of its echoes; one that reads wider (echoes from the floor or
 * from beside the object) is cut back to the most around that middle. Not
 * when an edge is past the sweep (then it is not known which echoes are
 * the object's). */
/* Two-distance sweep. After the first sweep from the
 * line (face nearer than S5_TWO_NEAR_MAX_MM, both edges in it) the car
 * backs up S5_TWO_BACK_MM and sweeps again; the width comes from how much
 * narrower the object looks from further, which leaves the beam out
 * (s5_scan_two_width()). Both sweeps stay nearer than about 25 cm, under
 * the 300+ mm echoes heard all round on the track. 0: off. */
#ifndef S5_TWO_DIST
#define S5_TWO_DIST (1)
#endif
#define S5_TWO_NEAR_MAX_MM  (190)
/* How far back the second sweep is taken. The width comes from the
 * difference of the two spans, so further apart is less sensitive to
 * span error; too far back and the track's own 300 mm echoes swamp the
 * object's edges. */
#ifndef S5_TWO_BACK_MM
#define S5_TWO_BACK_MM      (60)
#endif
#define S5_TWO_MIN_APART_MM (40)  /* the faces at least this far apart */
#define S5_TWO_BEAM_MIN_DEG (0)   /* a solved beam outside these: not used */
#define S5_TWO_BEAM_MAX_DEG (25)
/* the second sweep counts an echo as the object's only out to where its
 * corners should be from there (from the first sweep's corners) plus this:
 * from 22 cm the 300+ mm echoes around are within S5_EDGE_DEPTH_MM */
#define S5_TWO_DEPTH_SPARE_MM (25)
#define S5_OBJ_MIN_MM (80)
#ifndef S5_OBJ_MAX_MM
#define S5_OBJ_MAX_MM (250)
#endif
#ifndef S5_SERVO_TRIM_DDEG
#define S5_SERVO_TRIM_DDEG (-52)
#endif
#define S5_SERVO_PULSE_MIN_US (500U)  /* physical 0 deg (full right) */
#define S5_SERVO_PULSE_MAX_US (2500U) /* physical 180 deg (full left) */
/* Loaded SG90: 60 deg in 140 ms, then ringing dies down. 15 ms of
 * ring-down lets the profile finish within NFR11's 1.8 s; raise it if
 * the box width reads wrong on the car (25 ms: about 1.9 s). */
#define S5_SERVO_MS_PER_60DEG (140)
#ifndef S5_SERVO_SETTLE_MS
#define S5_SERVO_SETTLE_MS (15)
#endif

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
/* CALIBRATE step 2: added to every range (the sensor reads about 14 mm
 * long). */
#ifndef S5_RANGE_OFFSET_MM
#define S5_RANGE_OFFSET_MM (-14)
#endif
#define S5_RANGE_MIN_MM (20)   /* closer reads as 20 */
#define S5_RANGE_MAX_MM (2000) /* further counts as nothing there */
/* FR25: this many pings in a row without any answer (ECHO never rose)
 * is a dead sensor, never a clear path. */
#define S5_DEAD_PINGS (5U)

/* ---------------------------------------------------------------------
 *  Detection while line following (FR21: within 300 mm)
 * --------------------------------------------------------------------- */
/* line following (and the line search) stops for an object this near */
#ifndef S5_STOP_MM
#define S5_STOP_MM (180) /* 18 cm */
#endif
/* during a bypass, another object this near ahead or in the pass lane
 * stops the car (18 cm here left no room to go round a second box) */
#define S5_DETECT_MM (300) /* 30 cm */
#define S5_DETECT_CONFIRM (2U)  /* pings in a row at or under it */
#define S5_STALE_MS       (250) /* NFR10: an older range is unknown */
/* FR25: driving with no new ping for this long is a sensor fault */
#define S5_SILENT_MS (1000)

/* ---------------------------------------------------------------------
 *  Scan (the car stands still)
 * --------------------------------------------------------------------- */
#ifndef S5_SCAN_STEP_DDEG
#define S5_SCAN_STEP_DDEG (100) /* coarse sweep, +60 .. -50 deg */
#endif
/* 1: the sweep starts as soon as the object is detected, while the car
 * brakes; 0: the servo turns to the first bearing then and the sweep
 * starts once the car stands still (S5_STILL_MS) */
#ifndef S5_SCAN_WHILE_STOPPING
#define S5_SCAN_WHILE_STOPPING (1)
#endif
#ifndef S5_SCAN_FINE
#ifndef S5_SCAN_FINE
#define S5_SCAN_FINE (2U) /* halvings per edge: 10 -> 5 -> 2.5 deg */
#endif
#endif
/* the fine probes come at the edge from the left, like the sweep, so the
 * servo's gear backlash is the same for every sample */
#define S5_SCAN_OVERSHOOT_DDEG (40)
/* only these central bearings can hold the face the guard stopped for */
#define S5_FACE_CONE_DDEG (300)
/* a return this far behind the face is something else, not the object */
#define S5_EDGE_DEPTH_MM (120)
/* No longer used: the car sweeps from where it stopped, however close,
 * and backs up only when no way round is open. */
#define S5_SCAN_MIN_MM  (150)
#define S5_SCAN_DIST_MM (250) /* ... to here */
/* an edge outside the sweep: back off this much more and scan again,
 * only when no way round is open at all */
#define S5_RESCAN_BACK_MM  (150)
#define S5_MAX_RESCANS     (2U)
#define S5_SCAN_TIMEOUT_MS (5000)

/* CALIBRATE step 3: beam half-angle, ddeg. The echoes of an object go on
 * this far past each of its corners as the sensor turns (the HC-SR04's
 * datasheet says 7.5 deg; a flat box face answers ~22 deg off-axis).
 * Measured with the box at two distances; in between it is interpolated,
 * outside it is held. */
#ifndef S5_BEAM_NEAR_DDEG
#define S5_BEAM_NEAR_DDEG (120)
#endif
/* A box off the line is heard only about 19 deg past its corners, so
 * 180 reads it a little wider than it is: on the safe side. */
#ifndef S5_BEAM_FAR_DDEG
#define S5_BEAM_FAR_DDEG (180)
#endif
#define S5_BEAM_NEAR_MM (200)
#define S5_BEAM_FAR_MM  (300)

/* 1 = the corners come from the ranges of the
 * outermost echoes (sqrt(EDGE^2 - FACE^2), s5_scan.h), not from the angle
 * where the echoes stop. Needs no beam angle and no exact servo angle.
 * 0 = the angle method only, as before (to compare on the car: the log
 * prints both either way). */
#ifndef S5_EDGE_BY_RANGE
#define S5_EDGE_BY_RANGE (1)
#endif
/* a range that jumps by more than this from the bearing before it (mm,
 * plus S5_EDGE_STEP_MM_PER_DEG for each degree between them) is a mix
 * with something behind the object, not its corner. A flat face cannot
 * rise faster than ~3.3 mm per degree at 30 cm. */
#define S5_EDGE_JUMP_MM        (8)
#define S5_EDGE_STEP_MM_PER_DEG (3.5f)
/* an edge echo is checked against one at least this far further in
 * (0.1 deg) */
#define S5_EDGE_PAIR_DDEG (50)
/* a corner within about 5 cm of the sonar's axis reads less than 4 mm
 * longer than the face at 30 cm, which is the sensor's noise: whatever
 * answered straight ahead is taken to reach at least this far to each
 * side of the sonar */
#define S5_EDGE_MIN_MM (45)
/* the corner is where the ranges start growing over
 * two steps in a row by more than S5_EDGE_GROW_MM + S5_EDGE_GROW_PER_DDEG
 * / 100 per 0.1 deg (s5_scan.c). 0: off. */
#ifndef S5_EDGE_KNEE
#define S5_EDGE_KNEE (1)
#endif
#define S5_EDGE_GROW_MM       (3)
#define S5_EDGE_GROW_PER_DDEG (12) /* 1.2 mm per deg */
/* the least half-width taken for a narrow object
 * placed by the middle of its echoes (s5_scan.c) */
#define S5_NARROW_HALF_MIN_MM (S5_OBJ_MIN_MM / 2)
#ifndef S5_NARROW_ON
#define S5_NARROW_ON (1)
#endif
/* added to a side got by range, S5_EDGE_PAD_MM2 /
 * side (mm), at most S5_EDGE_PAD_MAX_MM */
#define S5_EDGE_PAD_MM2    (600)
#define S5_EDGE_PAD_MAX_MM (12)
/* the range method is used for a sweep from the line: the sonar within
 * this of it and the car this straight along it */
#define S5_EDGE_ON_LINE_MM  (40)
#define S5_EDGE_ON_LINE_DEG (15)

/* Both sides within this of each other: pick one at random. */
#define S5_TIE_MM (10)
/* 1: an echo beside the object, in the pass lane,
 * marks that side blocked (or maybe blocked), as before. 0: no side is
 * blocked by what the sweep saw beside the object (the side watch still
 * looks while passing). */
#ifndef S5_SIDE_BLOCK
#define S5_SIDE_BLOCK (0)
#endif

/* Taken off the measured width, half at each edge: the sweep read
 * objects about 5 cm wider than they are on the track. Each pass is
 * up to half of it closer to the object than S5_CLEARANCE_MM plans. */
#ifndef S5_WIDTH_TRIM_MM
#define S5_WIDTH_TRIM_MM (50)
#endif

/* The sonar hears the floor 54-71 cm out on every bearing the box does
 * not cover (it moves with the car). A side only "maybe blocked" (beam
 * edge) by an echo at
 * least this far is still taken at the first object when it is the
 * shorter way round (s5_pick()); a nearer echo still rules it out. */
#ifndef S5_FLOOR_MIN_MM
#define S5_FLOOR_MIN_MM (530)
#endif

/* ---------------------------------------------------------------------
 *  Bypass path. Smooth "potential field" shape on a straight line: the
 *  line pulls the car back, the object pushes it out. Planned as
 *  minimum-jerk ramps (no corner anywhere, curvature 0 where a ramp
 *  starts and ends) and tracked from the encoders.
 * --------------------------------------------------------------------- */
/* S5_FAST picks the bypass speed and the gap to the object together. In
 * the host simulator (path tracking, encoder errors, a 22 deg beam) the
 * car drifts 15..25 mm towards the object at 200 mm/s, so a 30 mm gap
 * needs about 130 mm/s, and 200 mm/s needs about 45 mm.
 *   0: 130 mm/s, 30 mm (tight; drive to the line about 6..9 s)
 *   1: 230 mm/s, 50 mm
 * The side watch moves the pass in or out to hold the gap (S5_SIDE_*). */
#ifndef S5_FAST
#define S5_FAST (1)
#endif
#ifndef S5_CLEARANCE_MM
/* car side to the object's edge */
#define S5_CLEARANCE_MM ((0 != S5_FAST) ? 50 : 30)
#endif
#define S5_MAX_OFFSET_MM   (400) /* never further from the line than this */
#define S5_MAX_HEADING_DEG (40)  /* steepest point of a ramp */
#define S5_MIN_RADIUS_MM   (150) /* tightest point of a ramp */
/* no room for that ramp out (the car stops 30 cm from
 * the object): a steeper one, down to this, before backing up at all */
#define S5_STEEP_HEADING_DEG (60)
#define S5_STEEP_RADIUS_MM   (110)
/* no backing up for room to turn out. Too close for the
 * steep ramp: turn on the spot towards the side, by as little as clears
 * the object (from S5_OUT_PIVOT_MIN_DEG up to 90 deg, in steps of
 * S5_OUT_PIVOT_STEP_DEG), drive straight out to the pass, turn back on
 * the spot. The car backs up only when not even that is open. */
#define S5_OUT_PIVOT_MIN_DEG  (45)
#define S5_OUT_PIVOT_STEP_DEG (15)
/* The ramp back in, as steep as the ramp out (less for the encoders to
 * drift over; the IRs pick the line up). */
#ifndef S5_IN_HEADING_DEG
#define S5_IN_HEADING_DEG (60)
#endif
#ifndef S5_IN_RADIUS_MM
#define S5_IN_RADIUS_MM (130)
#endif
/* While a ramp turns the car out or back in, no part of the car comes
 * closer to the object than this (checked on the planned path, corners
 * of the car included; the straight pass keeps S5_CLEARANCE_MM). */
#define S5_RAMP_MARGIN_MM (30)
/* 1: no room for a ramp out where the car stopped,
 * back up (at most S5_OUT_BACK_MAX_MM) for one, instead of turning out on
 * the spot. 0: turn on the spot first, as before. */
#ifndef S5_OUT_PREFER_BACK
#define S5_OUT_PREFER_BACK (1)
#endif
/* a ramp out by 20 cm is about 36 cm long (S5_STEEP_HEADING_DEG): from an
 * 18 cm stop the car needs about 45 cm of line behind it for one */
#define S5_OUT_BACK_MAX_MM (450)
/* object depth (front to back) when the look across cannot fix the far
 * corner: the car turns back in after this much object (or later, if the
 * sensor heard something further on). Set it to the deepest object on the
 * course. Also how far another object found close behind the first is
 * taken to reach (the car then passes both). The objects on the track
 * are 8..25 cm; a deeper box whose far corner the side watch misses can
 * be clipped. */
#ifndef S5_DEPTH_DEFAULT_MM
#define S5_DEPTH_DEFAULT_MM (150)
#endif
#define S5_DEPTH_MIN_MM     (60)
/* the deepest the far corner may be: one object, or two close together */
#define S5_DEPTH_MAX_MM (1000)

/* The host tests keep 110 / 80 (testing/subsystem5/host/Makefile): at
 * these speeds a box turned 15 deg to the line is passed only 1-3 mm
 * away in the bypass simulator. */
#ifndef S5_SPEED_MM_S
/* see S5_FAST above */
#ifndef S5_SPEED_MM_S
#define S5_SPEED_MM_S ((0 != S5_FAST) ? 230 : 130) /* along the bypass */
#endif
#endif
#ifndef S5_SLOW_MM_S
#define S5_SLOW_MM_S (100) /* settling onto the line */
#endif
/* backing up straight along the line for room for one smooth ramp out
 * (S5_OUT_PREFER_BACK) */
#ifndef S5_REVERSE_SPEED_MM_S
#define S5_REVERSE_SPEED_MM_S (100)
#endif
#define S5_REVERSE_MAX_MM     (500) /* never back up further, in total */
/* Buddy 2 gives a wheel no drive at 5 mm/s or less and kicks one under
 * 10 mm/s, so a moving wheel is never asked for less than this */
#define S5_MIN_WHEEL_MM_S (15)

/* tracking: errors die out over about this distance (critically damped) */
/* 150 in S5_FAST: at 230 mm/s, 80 made the car weave and lose 10..20 mm
 * of the gap */
#ifndef S5_TRACK_DIST_MM
#define S5_TRACK_DIST_MM ((0 != S5_FAST) ? 150 : 80)
#endif
#define S5_ALIGN_DIST_MM (60)
/* Encoder scale learning. The right wheel's mm per
 * tick is taken as (1 + S5_ODOM_LR_PPM / 1e6) times the left's (start
 * value); after each bypass the angle the two IRs met the line at says how
 * far the encoders' heading drifted, and the ratio moves by
 * S5_ODOM_LEARN_PCT of what that drift says (kept within
 * S5_ODOM_LR_MAX_PPM). On the track the encoders said 2..11 deg less
 * than the IRs on every bypass, both sides. 0: no learning. */
#ifndef S5_ODOM_LR_PPM
#define S5_ODOM_LR_PPM (0)
#endif
#ifndef S5_ODOM_LEARN
#define S5_ODOM_LEARN (1)
#endif
#define S5_ODOM_LEARN_PCT   (50)
#define S5_ODOM_LR_MAX_PPM  (30000) /* 3 % */
#define S5_ODOM_LEARN_MIN_MM (500)  /* only after a bypass this long */
#define S5_ODOM_LEARN_MAX_DEG (15)  /* and a drift under this */
#define S5_MAX_CURV_PPM  (10000) /* tightest command, 1e6 / radius mm */

/* CALIBRATE step 4: how far the car really turns for 100 deg of encoder
 * turn (wheels slip and scrub). Junction pivots measured 1/1.3 ..
 * 1/1.6 (63..77); the smooth bypass arcs slip less. Used both to dead
 * reckon and to steer. */
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
/* ... and back in by at most this (the side further
 * away than the sweep said), and only for a change of more than this */
#define S5_SIDE_IN_MAX_MM (30)
#define S5_SIDE_STEP_MM   (4)

/* ---------------------------------------------------------------------
 *  Back onto the line
 * --------------------------------------------------------------------- */
/* coming back, an IR seeing black further than this from where the line
 * should be is another lane's line, not ours */
#define S5_LINE_WINDOW_MM (100)
/* ... and the pick-up (both IRs crossing it) this far.
 * On the track the encoders put the line 7-12 cm off on the way back;
 * S5_CAP_LINE_DEG keeps a junction's branch near it out */
#define S5_CAP_WINDOW_MM (150)
/* A line the IRs cross more than this off the direction of ours (from
 * where each IR met it, in the encoders' frame) is a junction's branch,
 * not ours. The encoders drift a few degrees on the way round, and a
 * bend behind the box can run 24 deg off; a branch at 45 deg still
 * stays out. */
#ifndef S5_CAP_LINE_DEG
#define S5_CAP_LINE_DEG (30)
/* A line the IRs meet within this of where the encoders put ours is
 * ours, whatever direction the IRs make of it: at 200+ mm/s the two IRs
 * meet the tape a few ms apart, which can read as any angle. */
#define S5_CAP_OURS_MM (40)
#endif
/* ... and both IRs black at once is our line only with the car crossing
 * at least this steeply (25 mm between the IRs, 18 mm tape: 44 deg, less
 * the drift); less steep, a line across ours */
#define S5_CAP_BOTH_DEG (36)
/* Line not met at the end of the ramp: carry on at this angle. 30,
 * because the car can end 5 cm beside the line with the encoders some
 * degrees off. */
#define S5_SEEK_DEG    (30)
#define S5_SEEK_MAX_MM (400) /* ... this far (FR24: within 500 mm) */
/* nothing black on the way back: looking on, the first IR black this
 * near where the line should be is our line */
#define S5_SEEK_WINDOW_MM (300)
/* Second try before giving up (FR24): the encoders can put the line far
 * enough over that the 30 deg look runs out beside it. When
 * S5_SEEK_MAX_MM is used up the car turns in to this angle for this much
 * further, the window this wide. S5_SEEK2_MM 0: off. */
#define S5_SEEK2_DEG       (60)
#ifndef S5_SEEK2_MM
#define S5_SEEK2_MM        (400)
#endif
#define S5_SEEK2_WINDOW_MM (600)
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
 *  Compass
 *
 *  Buddy 4's LSM303D has no gyro, but its magnetometer gives the car's
 *  heading. The encoders' heading drifts a few degrees on the way round
 *  (the line came back 7-12 cm off on the track); the compass pulls it
 *  back. The motors' magnets move the field's centre, so it needs
 *  calibration step 4 (make S5CAL=2 runs that step only): it prints
 *  these four. S5_MAG_SIGN 0 = not calibrated: compass not used.
 * --------------------------------------------------------------------- */
#ifndef S5_MAG_SIGN
/* Calibration found the compass up to 13 deg off the encoders on the way
 * round, so S5_MAG_USE below is 0 (logged only). */
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
 * 0 until a track run shows it agrees with the IRs */
#ifndef S5_MAG_USE
#define S5_MAG_USE (0)
#endif
#define S5_MAG_TAU_MS  (1000) /* pulls the heading over about this long */
/* ... by at most this per 100 mm the wheels go (encoders 3 % apart drift
 * about 1.5 deg per 100 mm); turning on the spot, a wheel goes
 * S5_MAG_ARM_MM per radian */
#define S5_MAG_RATE_DEG (2)
#define S5_MAG_ARM_MM   (56.0f)
/* The compass and encoders this far apart or less: left alone. A compass
 * is a few degrees off at some headings even when calibrated (iron near
 * the IMU); good encoders are better than that over a bypass, so only
 * drift past this is pulled back: with 2 deg of compass error, pulling
 * at every degree took the car into a second box in the simulator. */
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
/* ... stops the car this long and goes on, this many
 * times per bypass, before the bypass fails (a wheel turning on the spot
 * slowly can stall; a dead encoder still fails, 1.5 s later) */
#define S5_WHEEL_TRIES   (1)
#define S5_WHEEL_REST_MS (300)

/* ---------------------------------------------------------------------
 *  Line search: robot.ps1 search. The car starts off
 *  the line (on white), drives straight until the IRs meet a line, drives
 *  its axle onto it and turns on the spot to face along it, then Buddy 3
 *  follows it. s5_search_begin() in s5_avoid.c.
 * --------------------------------------------------------------------- */
#define S5_SEARCH_MM_S (60)   /* straight on at this speed */
#define S5_SEARCH_MAX_MM (2000) /* no line within this: stop (fault 0x40) */
/* one IR black: the other must meet the line within this (lines from
 * 10 deg to the car up; shallower ones Buddy 3 takes) ... */
#define S5_SEARCH_PAIR_MM (150)
/* ... and one IR black this long with the other still white is a line
 * running along the car (or the car started on one): Buddy 3 follows it.
 * A line crossed at 27 deg or more leaves the IR within 40 mm (18 mm
 * tape). */
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
/* The turn onto the line stops on the IRs (the first one black, then
 * the other); without them it goes on to this far past
 * the direction worked out from the crossing (a bend's chord can be 25
 * deg further round), then Buddy 3 looks */
#ifndef S5_SEARCH_PAST_DEG
#define S5_SEARCH_PAST_DEG (30)
#endif

/* ---------------------------------------------------------------------
 *  Calibration build (testing/subsystem5/CALIBRATION.md): make S5CAL=1
 *  gives a _s5cal image whose S5 task runs the guided calibration
 *  instead of the obstacle logic. Flash the normal image afterwards.
 *  make S5CAL=2 (_s5cal4 image) runs step 4 only (the
 *  circle: turn and compass).
 * --------------------------------------------------------------------- */
#ifndef S5_CAL_BUILD
#define S5_CAL_BUILD (0)
#endif
#define S5_CAL_BOX_WIDTH_MM (210) /* the track box: 21 x 16 cm */
#define S5_CAL_RANGE_MM     (200) /* step 2 distance */
#define S5_CAL_SETUP_S      (15)  /* seconds to place things */
#define S5_CAL_CIRCLE_MM    (150) /* step 4: radius of the circle */

#endif /* S5_CONFIG_H */
