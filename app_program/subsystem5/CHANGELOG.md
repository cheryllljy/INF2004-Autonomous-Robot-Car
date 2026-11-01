# Subsystem 5 (ObstacleScan) changelog

Buddy 5, student ID 2502410. Newest first. v1 stays in
`app_program/old_src/` (not built) and in git history; nothing is
overwritten.

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
