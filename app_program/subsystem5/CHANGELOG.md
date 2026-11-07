# Subsystem 5 (ObstacleScan) changelog

Buddy 5, student ID 2502410. Newest first. v1 stays in
`app_program/old_src/` (not built) and in git history; nothing is
overwritten.

## v3.4 (2026-11-06): faster sweep, back to the car's own line

nyx's report (2026-11-06): the sweep is too slow; after turning away
from the box the car does not get back to its line behind it.

### Sweep (S5)
- `S5_SCAN_SLEW_DPS` 90 -> 300 deg/s and `S5_WATCH_SLEW_DPS` 180 -> 300.
  Still one smooth pass, 1 deg every 3 ms. A scan takes 2.6 s instead of
  5.0 s (sim S2); most of what is left is the pings themselves.

### Way back to the line (vc_obstacle.c, IN)
- The course has lanes. With the beam not calibrated, the 21 cm box
  scans about 35 cm wide and the detour goes about 29 cm out, past the
  next lane's line; coming back, the IRs met that line first and the car
  rejoined the wrong lane (sim C20 before the fix: rejoined 250 mm off).
  A line met while dead reckoning still has the car more than half the
  bypass offset (at least `VC_OBS_LINE_WINDOW_MIN_MM`, 100) from its own
  line is now ignored and printed as another lane. While searching, the
  first line is taken.
- Near the line (within `VC_OBS_ARC_NEAR_MM` by dead reckoning, after
  the first IR hit, and while searching) the arc slows to 60 mm/s.
- Once the line has been between the IRs, the moment it is under one IR
  again Buddy 3 takes over: it steers towards it. Before, VC drove on by
  dead reckoning, which can carry the line out past the far IR, and with
  the IRs straddling the line "both white" off the track looks centred.
- The arc gives up by distance (the longest path back at the shallowest
  approach + the line search + `VC_OBS_ARC_SLACK_MM`), not after 12 s,
  so a slower car is not cut off. 40 s remains as a stall cap.
- New log lines: `back to the line: N mm to the side ...`, `IR_x on the
  line, N mm from where dead reckoning has it`, `crossed a line N mm from
  ours by dead reckoning: another lane, ignored`, `line under IR_x,
  Buddy 3 takes over`; every FAIL now prints where the car was.

### Tests
- test_car: after the handover, Buddy 3's real line logic (`b3_line.c`,
  running on every pass as on the car) follows at its base speed for
  400 mm, and the car must still be on the line. New scenarios: C19
  nyx's 210 mm box with a 22 deg flat-face beam, beam not calibrated;
  C20 the same with lane lines 250 mm either side; C21 the same with the
  step 4 beam (separate build, test_car_cal); C22 / C23 IRs 120 / 20 mm
  ahead of the axle where VC assumes 60. 23 scenarios pass.

## v3.3 (2026-11-06): calibrate the beam on the real obstacle

nyx's report (2026-11-06): the 21 cm box reads 42 cm wide.

- The sweep range (+/-60) does not make it wider; the beam does. The
  width is the angle where the box stops answering, minus the beam
  half-angle, and `S5_CAL_BEAM_HALF_DDEG` is still the datasheet 7.5 deg.
  A flat face echoes from much further off-axis than the thin post of
  step 1, so every edge was placed too far out.
- New calibration step 4/4 (`s5_calibrate.c`, `S5_CAL_BUILD 1`): the
  course's box (`S5_CAL_OBSTACLE_WIDTH_MM`, 210) square to the car at
  300 mm. The servo creeps 1 deg at a time over the whole sweep, twice
  each way; the beam half-angle is how far past the box's own corners it
  still answers. When step 4 passes it replaces step 1's beam in the
  printed block (step 1's value is printed in a comment).
- Host test H06a now runs all four steps (post beam 12 deg, box beam
  22 deg: found 22.1). New H06c: the obstacle task on a 21 cm box with a
  22 deg flat-face beam reads 297..354 mm wide with the old 7.5 deg and
  208..214 mm with the step 4 value.

## v3.2 (2026-11-06): smooth sweep, no rescan loop, find the line

nyx's report (2026-11-06): the sweep is too fast and jumpy; the car
keeps doing detect, reverse, sweep, pause, drive forward, sweep, reverse;
it went left although the right side was shorter; after passing it did
not come back to the line, drove straight, and the sensor kept sweeping.

### Sweep (S5)
- One smooth pass instead of jumps: the servo turns 1 deg at a time at
  `S5_SCAN_SLEW_DPS` (90 deg/s) from one sweep limit to the other and
  pings every 10 deg (13 bearings at +/-60). Bearings inside
  +/-`S5_SWEEP_MEDIAN_DEG` (20) use the median of 3 on the way past, so
  the fine pass does not turn back. Edge probes take the nearer edge
  first. Side looks while passing and the return to centre turn at
  `S5_WATCH_SLEW_DPS` (180 deg/s). New `drv_servo_get_angle_ddeg()`.
- Scan time (sim S2) 1.5 s -> 5.0 s at 90 deg/s (2.6 s at 300). Every
  bearing of the +/-60 view is measured now, so NFR11 (1.8 s) is no
  longer met at any servo speed; test_sim checks the new budget.
- A first scan plans round anything up to `OBSTACLE_TRIGGER_MM` +
  `S5_PLAN_MARGIN_MM` (400 mm). A box the guard saw at 300 mm used to
  scan as CONTINUE at 310, so the car drove on and stopped again.
- A rescan VC asked for always stays in ASSIST and raises PROFILED, never
  CLEARED, so VC decides what happens next.
- `DRV_SERVO_INVERT` (drv_servo.c) can be set from the build. A wrong
  servo direction mirrors every bearing: the car picks the wrong side
  and WATCH looks at the empty side.

### VehicleController (vc_obstacle.c)
- The rescan after the slow back-off no longer ends the episode when it
  says CONTINUE, STOP or SLOW: VC plans from the first scan, with the
  nearest point moved out by the back-off (or the fresh front range).
  This was the reverse / sweep / drive / reverse loop.
- The arc back only hands over to line following after an IR has seen
  the line. If dead reckoning says "on the line" first, the car keeps
  closing in at 20 deg for up to 400 mm, then stops with a fault instead
  of driving straight off the track (with the IRs straddling the line,
  "both white" off the track looks like "centred").
- Dead encoder check: one wheel moving 150 mm while the other reads
  under 5 mm stops the episode with a fault (nyx's GP3 encoder). The
  heading is worked out from the two encoders only, so a dead one makes
  every turn and the arc back wrong.
- `VC_OBS_TIMEOUT_MS` 45 -> 60 s for the slower scans.

### Follow-up (same day): no sweeping unless driving, firmer return
- nyx: "I don't want the servo to keep sweeping, only when it detects an
  object", and the car still did not get back to the line.
- `obstacle_set_armed()`: VC arms S5 only in LINE_FOLLOW, TURNING and
  OBSTACLE. Parked or stopped, S5 keeps the front range fresh but never
  stops-and-sweeps (a hand in front used to start a sweep that VC threw
  away, over and over). Sim S13.
- WATCH: `S5_WATCH_FRONT_LOOKS` 0 holds the sensor turned towards the
  obstacle while passing instead of swinging side / front / side. 1 gives
  back the front look on the pass leg.
- Arc back: until an IR has seen the line the car cuts back at least
  `VC_OBS_APPROACH_MIN_DEG` (15 deg), so drifting dead reckoning no longer
  makes it creep alongside the line; search length 400 -> 600 mm.

### Tests
- test_car: C14 odometry 25 % short, C15 right encoder dead (safe
  stop), C16 no line where the car comes back (safe stop), C17/C18
  heading reads 12 % over / under (turn slip). 18/18 pass.
- test_sim: S13 parked (no sweep until armed).

## v3.1 (2026-11-06): wider sweep, slow reverse and rescan, smooth return

nyx's requests (2026-11-06): the sonar bracket now sweeps a wider angle;
measure the object to about +/-1 cm; always reverse slowly and scan
again; leave the obstacle by a small, computed angle towards the
clearer side so the car does not drift into another lane; come back to
the centre of the track smoothly; the car is about 180 x 130 mm.

### Sweep and measurement (S5)
- `S5_SWEEP_LIMIT_DEG` 30 -> 60, `DRV_SERVO_MIN/MAX_DEG` 60/120 ->
  30/150. Static asserts keep the limit on the 10 deg grid, inside the
  packed bearing range (+/-70) and inside the servo limits.
  TODO(measure): the limit must match the real bracket.
- The coarse pass stays at -30..+30 (new `S5_COARSE_OUTER/MID_DEG`):
  a +/-60 coarse pass took 1957-2188 ms and broke NFR11 (1800 ms).
  Instead `obs_extend_edges()` follows an edge outwards 10 deg at a time,
  up to the limit, only while the object still runs off the measured
  bearings. S2 scan time 1391 -> 1518 ms.
- `S5_EDGE_PROBES` 2 -> 3 (10 -> 1.25 deg, about +/-3 mm per edge at
  300 mm). Host v3 test: mean width error 10 -> 9 mm on the ideal
  robot; test_cal calibrated faulty robot 24 -> 9 mm. The sim's
  sonar noise and beam model limit it; +/-10 mm on the real car still
  needs the calibration run.
- Height cannot be measured: the HC-SR04 sweeps one horizontal plane and
  returns only the nearest echo. Nothing reports a height.
- A rescan that VC asks for always uses the far remeasure trigger
  (`rescan_far`), so an obstacle the car backed away from is never
  called CONTINUE.

### Car size and margins
- `OBSTACLE_HALF_WIDTH_MM` 75 -> 65, `OBSTACLE_ROBOT_LENGTH_MM` 200 ->
  180, `S5_REAR_FROM_SENSOR_MM` 150 -> 180 (TODO(measure)),
  `OBSTACLE_SAFETY_MARGIN_MM` 60 -> 40, `VC_OBS_EXTRA_MM` 20 -> 10.
  The in-path band is now +/-105 mm and the corridor 170 mm.

### VehicleController bypass (`vc_obstacle.c/.h`)
- Every bypass first reverses slowly (`VC_OBS_REV_SPEED_MM_S` 100 -> 40)
  to stand about `VC_OBS_REMEASURE_MM` (350) from the face, scans again
  and plans from the second profile. Each profile prints its width,
  edges, centre, nearest point and swings.
- Out angle: the smallest angle from `VC_OBS_TURN_MIN_DEG` (20) up to
  STEEP (70) that clears the swing before the face, from
  lateral * cos(a) <= forward * sin(a). The "blocked -> steeper" fallback
  is kept.
- Clearer side: when both sides are open and the swings are within
  20 mm, the side with the bigger gap wins (`obs_prefer_clearer()`,
  refine only, so the CR1 equivalence test is unchanged).
- Return: TURN_IN / straight IN / ALIGN replaced by one arc
  (`VC_OBS_IN`, `motion_set_wheel_speeds`): heading target from the
  lateral offset (200 mm look-ahead, max 30 deg), the IR pair resets the
  lateral offset when it straddles the line, and it hands straight to
  LINE_FOLLOW without stopping. Something ahead -> stop and replan;
  12 s timeout -> fail. TURN_IN and ALIGN stay in the enum (unused) so
  telemetry step numbers do not move.

### Tests (`testing/subsystem5/host`)
- G7 vector `{20, 320}` -> `{20, 300}` and the G16 title: the 130 mm car's
  path band is +/-105 mm.
- v1 reference: `BOARD_HALF_WIDTH_MM` / `OBSTACLE_SAFETY_MARGIN_MM` can be
  set from the Makefile (`V1_CAR`), so the CR1 equivalence test compares
  the same car (0 mismatches).
- test_car: chassis 180 x 130; C10 now expects the second box to be
  seen by the wider scan and passed on the right; new C12 (small box
  hidden behind the first, found beside the lane) and C13 (second box
  on the line, hidden, found on the arc back: stop, rescan, replan; v3.0
  stopped in FAULT here).
- Car sim, v3.0 -> v3.1: rejoin error within +/-13 mm (was +/-7 mm, now
  with no stop at the line); closest approach 36-97 mm (was 29-105);
  episodes about 6 s longer (reverse at 40 mm/s plus the second scan).

## v3.0 (2026-11-04): prof feedback, calibration, side watch, reverse, VC bypass

Prof feedback after Tribunal 1 (meeting 2026-11-03):
1. "However you adjust it you can't get the centre point" -> calibrate.
2. Turn and scan while passing the obstacle, so the car cannot hit it.
3. Work with VehicleController: too close to measure well -> reverse and
   measure again.

### 1. Calibration (`s5_calib.h`, `s5_calibrate.c`, drivers, geometry)
- Three stacked errors explained in `s5_calib.h`: servo trim (horn spline
  teeth ~15 deg apart, so it cannot be fixed by re-seating), gear
  backlash (stops short on the side it came from), beam width (an
  object looks wider, edges and centre move). Plus range scale/offset.
- `drv_servo`: every command gets trim + a backlash lead in the
  direction of travel (`drv_servo_correct_ddeg()`, pure, shared with
  the simulator). New 0.1 deg command for the edge probes.
- `drv_ultrasonic`: speed of sound from `S5_AIR_TEMP_C` (25 C = 346.4
  m/s; v2's 343 read 1 % long), then `true = raw * scale / 4096 +
  offset` from the calibration.
- Geometry: `obstacle_profile_refine()` runs after the unchanged v2
  build (so the CR1 equivalence test S5-H03 still holds). Each seen edge
  is placed half way between the last bearing that heard it and the
  first that did not, each moved in by the measured beam half-angle.
  Centre = middle of the corrected edges (v2 averaged the hit bearings).
- Edge probes: two extra pings per seen edge bisect that interval
  (10 -> 5 -> 2.5 deg). Paid for by no longer re-pinging coarse
  bearings in the fine pass. Simulated scan 1256 -> 1391 ms (NFR11 1.8 s).
- Host result (S5-H05 V2, 12 cases, both edges seen): centre error
  1.4 -> 0.4 deg, width error 28 -> 10 mm.
- Guided calibration build (`S5_CAL_BUILD 1`): servo creep 60..120 deg
  in 1 deg steps both ways at a thin post (trim, backlash, beam), board
  at 100/200/300/500 mm (least squares fit, NFR15 check per point),
  then a verify sweep with the new values live. Prints the block for
  `s5_calib.h`. Guide: `testing/subsystem5/CALIBRATION.md`.
- Host result (S5-H06): against a simulated servo 3.5 deg off with
  1.5 deg backlash, a 12 deg beam and a range 3 % long - 8 mm, it finds
  trim -3.5, backlash 1.5, beam 11.0, scale 3983/4096 (true 3977),
  offset 7 mm (true 7.8). The obstacle task's mean centre error on that
  robot goes 3.8 -> 1.6 deg, width error 48 -> 24 mm.
- Health bit 7 (was spare) = calibrated; shown in telemetry and S5-U14.

### 2. WATCH: turn and scan while passing
- New state `OBSTACLE_ST_WATCH` (10 states, still 4 bits). On the pass
  leg the servo alternates front ping / side ping at `S5_WATCH_DEG`
  towards the obstacle.
- Phases LOOKING -> SEEN -> CLEARING -> PASSED (or BLIND), in a new
  lock-free 32-bit watch word (side, phase, reverse mm /2, lateral mm /4,
  last side range).
- `S5_EVT_SIDE_CLOSE`: flank closer than 40 mm to the face.
- `S5_EVT_PASSED`: measured, not guessed. When the side look goes clear
  the obstacle's far end is L * cot(w + h) ahead (L = closest face seen,
  w = look angle, h = beam half-angle); the car drives that plus sensor-
  to-rear plus margin. v2 assumed every obstacle 150 mm deep: the 450 mm
  box in simulation C4 would have been hit on the way back.
- The ASSIST watchdog now means "no word from VC for 5 s"
  (`obstacle_assist_kick()` every VC loop), because a full bypass takes
  longer than 5 s.

### 3. Reverse to remeasure
- `obstacle_reverse_mm()`: inside the hard stop back to 250 mm, an edge
  outside the +/-30 deg view (or boxed in) back to 400 mm; 50..250 mm.
  Published in the watch word before PROFILED.
- An edge at the sweep limit is now "unseen": that side's gap is 0, not
  open (v2 would have planned a bypass through a side it never saw).
- `obstacle_rescan()`: VC backs off, S5 waits for the stop and sweeps
  again, planning round anything up to 500 mm (else the backed-off
  obstacle reads CONTINUE and the car drives back into it - found in
  simulation C3). Third REVERSE in a row is still REVERSE_LIMIT.

### 4. VehicleController (Buddy 5 hook)
- New `vehicle_controller/vc_obstacle.c/.h` (Buddy 5 owns it): wait for
  the scan, reverse + rescan, the bypass legs (turn out 45 deg or 70 deg
  if close, out, parallel, pass with WATCH, side-step 10 deg on
  SIDE_CLOSE, turn in, drive until Buddy 3's IR sees the line plus the
  IR-to-axle distance, align), dead-reckoned pose and the rejoin error
  printed against +/-50 mm, replan limits and timeouts.
- HARD_STOP: instant motor cut through Buddy 2's `MOTION_ESTOP_OBSTACLE`,
  released once both wheels are still. Only while driving forward (S5
  keeps reporting under 120 mm while backing away). Not the latched
  system E-stop.
- `vehicle_controller.c` (shared): includes vc_obstacle.h; `enter()`
  ends the manoeuvre when OBSTACLE is left early; odometry speed for S5
  is the faster wheel, not the mean (turning on the spot the mean is 0
  and WaitStopped would sweep while spinning); `read_obstacle()` routes
  events to vc_obstacle, cuts on HARD_STOP, raises the new fault bit on
  an S5 mission fault while driving (clears it while parked);
  `act()` runs vc_obs_step() in OBSTACLE.
- `vc_types.h` (shared): `VC_FAULT_SPARE` (bit 6) -> `VC_FAULT_OBSTACLE`.
- `app_main.c` (shared): `subsystem5_init/start` switched back on (off
  since 2026-11-02).

### Evidence
- Host (`cd testing/subsystem5/host && make`): all v2 tests still pass,
  plus S5-H05 (21 checks), S5-H06 (calibration), S5-H07 (2-D closed-loop
  bypass, 11 scenarios: no collision, rejoin within 7 mm of the line,
  boxed-in ends in a safe FAULT stop).
- Bugs the simulation found and that are fixed here: the "passed"
  formula used the inner beam edge (car went 160 mm further than
  needed); HARD_STOP cut the motors while backing away, pinning the car
  (C6); a backed-off obstacle read CONTINUE (C3); a rescan sent while
  S5 was in WATCH was dropped and the 60 ms tick stayed off (C10); a
  leg was blocked only under 150 mm, too late at 150 mm/s with 50 mm of
  car in front of the sensor (C10); a wall beyond the line stopped the
  way back (C5).
  Logs: `testing/subsystem5/logs/2026-11-04_*`.
- Target: car, TEST=1 and calibration images build with no new warnings
  (strict -Wall -Wextra -Wconversion on all S5 and VC-obstacle files).
- Telemetry packet 29 -> 33 B (version 2), nobody decoded version 1.

## v2.2 (2026-11-01): scan limited to +/-30 deg

- Bench: the bracket hits the Pico board beyond +/-30 deg; the bench
  sweep at +/-30 deg clears it.
- `drv_servo.h`: hard limits 15..165 -> 60..120 deg, so no command can
  reach the board. `s5_config.h`: `S5_SWEEP_LIMIT_DEG` 70 -> 30.
  `obstacle_task.c`: coarse bearings {-60,-30,0,30,60} ->
  {-30,-20,0,20,30}; the fine pass fills -10/+10 and is clipped to
  +/-30.
- Trade-off: at 300 mm the sensor sees about +/-150 mm to each side
  (the driving band is +/-135 mm). A wall further out is not seen, so a
  side gap can read as open when it is not. Remounting the bracket and
  restoring +/-70 deg is the proper fix.
- Bearing field kept at 4 bits (7 values would fit 3) so a wider sweep
  needs no format change. Stated in the CR1 table.
- `subsystem5_test.c` S5-U10 now fails only on a SENSOR fault. On the
  bench (no VehicleController) an object in front ends in
  ASSIST_TIMEOUT by design; that was failing U10 for the wrong reason.
- Host tests: all pass; simulated scan shorter (see TEST_LOG).

## v2.1 (2026-11-01): sonar waits on kernel time

- Bench (S5 bench image, laptop USB): v2.0 gave one ping every ~3 s
  instead of 4 per second, with correct distances and 0 no-response.
  The waits used TIMER_TIMERAWL as a 1 MHz clock; in this port it does
  not run at 1 MHz, so every wait stretched ~10x.
- `drv_ultrasonic.c`: quiet period (60 ms) and read window (30 ms) now
  use `tk_get_otm()`. Bench after the fix: one line every 300 ms
  (250 ms loop + ping), 72 pings, 0 no-response, 125-1131 mm tracked.
- Ping wall time 20-30 ms -> 30-40 ms; guard period still 60 ms.
- Only the 12 us trigger pulse still reads TIMER (a slow TIMER just
  makes it longer, which the HC-SR04 accepts). The profile build time
  in health and S5-U08 also reads TIMER, so treat that number as
  relative until the TIMER rate is confirmed.
- Note for Buddy 3: `b3_port_time_us()` reads the same TIMER register.
- Sim updated to the 30 ms window; all host tests re-run. Simulated
  scan 1 401 -> 1 533 ms (NFR11: 1.8 s). Worst-case first range after
  an object appears ~100 ms (v2.0 ~92 ms, v1 82 ms).

## v2.0 (2026-10-20): port to the team build, CR1, tutor feedback

### Port to the micro T-Kernel make build
- Code moved into `app_program/subsystem5/`; entry points are
  `subsystem5_init()` / `subsystem5_start()` as the template requires.
- No Pico SDK runtime in this build, so both drivers were rewritten at
  register level (`in_w/out_w/set_w/clr_w`, BSP `gpio_*` and `pwm_*`),
  the same way Buddy 3's port works. Renamed to `drv_servo.*`,
  `drv_ultrasonic.*` (README: drivers live in the subsystem folder).
- PWM block reset is released by S5 at init (`config.h` has
  `USE_PTMR = 0`, so the kernel leaves PWM in reset).

### ECHO timing: interrupt removed
- v1 timed ECHO with a GPIO edge interrupt on IO_IRQ_BANK0. Buddy 3
  already installs the only handler on that line; a second `tk_def_int`
  would replace it and kill barcode decoding.
- v2 uses GP17 as the B input of PWM slice 0 in level-gated mode: the
  counter counts microseconds only while ECHO is high. No ISR, so no
  IRQ contention and no ISR/task shared data.
- Cost: the counter is read 20-30 ms after the trigger (10 ms kernel
  tick) instead of at the falling edge. Worst case first range after an
  object appears: 82 ms (v1) -> about 92 ms (v2), +3 mm at 300 mm/s.

### Change Requirement 1 (minimum-bit data)
- 12-bit range code (30..2500 mm + CLEAR/UNKNOWN), 4-bit bearing index,
  17-bit ms stamps, 3-bit status/action/fault, 4-bit state.
- Packed: sample 4 -> 2 B, sample buffer 64 -> 32 B, profile 32 -> 16 B,
  health 16 -> 8 B, ping result 12 -> 4 B, stats 20 -> 8 B, task
  control state 13 separate bytes -> one 4-byte bit-field word.
- Shared data rules: front range is one 32-bit word, one writer, one
  store (lock-free); profile/health under the mutex; events in a kernel
  event flag. Bit-fields only for task-private state.
- MQTT payload: 29-byte binary packet (was ~200 B as JSON text).
- Static RAM: 225 B (v1) -> 135 B (v2), measured with arm-none-eabi.
- Public getters keep the 16-bit sentinels 0xFFFE / 0xFFFF, so no
  other subsystem's code changes.

### Tutor feedback and Tribunal 1 decisions
- WaitStopped: no sweep until odometry speed = 0 for 100 ms; 1 s
  timeout -> FAULT (STOP_TIMEOUT). Without registered odometry a fixed
  500 ms wait is used and flagged in health (`odometry_ok = 0`).
- E-stop only on sensor fault: S5 sets its own cause bit (registered by
  the E-stop owner), never clears it. Normal obstacle stops use
  `S5_EVT_DETECTED` / `S5_EVT_HARD_STOP` and VehicleController's STOP.
- HARD_STOP event under 120 mm (guard and assist).
- Third REVERSE in a row -> FAULT (REVERSE_LIMIT); the third profile is
  published as STOP.
- 5 s ASSIST watchdog -> FAULT (ASSIST_TIMEOUT).
- Mission faults are latched until `obstacle_reset_fault()`.
- "Past the obstacle": `obstacle_pass_distance_mm()` = nearest +
  assumed depth + robot length + margin (TODO: measure the constants).
- Fine scan centres on the nearest IN-PATH return (v1: nearest
  anywhere, so a side wall could pull the fine scan off target).
- Fine scan starts from the end nearest the servo (less travel).
- CONFIRM is now a real state (Guard -> Confirm -> WaitStopped).
- Median ping no longer reports NO_ECHO (= clear) when the pings
  failed; it reports NO_RESPONSE.
- `obstacle_resume()` sets an event-flag bit instead of a `volatile
  bool`; stale resumes are dropped on entering ASSIST.

### Tests
- Host: 173 geometry checks (G1..G16), 31 packing checks (P1..P8),
  20 000-sweep v1/v2 equivalence (0 mismatches), 11 simulation
  scenarios (S1..S12). `cd testing/subsystem5/host && make`.
- On target: S5-U01..U11 in `subsystem5_test.c` (`make TEST=1`).

## v1.0 (2026-10-18): Week 6 / Pre-Tribunal version
- Pico SDK CMake project, edge-interrupt echo, coarse + fine sweep,
  region-growing geometry, 46 geometry checks, 6 sim scenarios.
