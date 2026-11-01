# Subsystem 5 — test log

**Subsystem:** ObstacleScan (ultrasonic + servo, obstacle profiling)  ·  **Student ID:** 2502410  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

### Host tests (PC, no hardware): `cd testing/subsystem5/host && make`

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-H01 | Geometry and policy, scenes G1..G16 (clear track, post, box, wall, too close, bad input, least deviation, silent sensor, bench data, pass distance, in-path band) | FR21-FR25, NFR14 | auto | `make geometry` | `173 checks, 0 failures` |
| S5-H02 | CR1 encoders: every field at its limits, saturation, wire format, size table, text vs binary size | CR1 items 1-3 | auto | `make pack` | `31 checks, 0 failures` |
| S5-H03 | CR1 no loss of function: v1 geometry (unpacked) vs v2 (packed) on 20 000 random sweeps | CR1 item 4 | auto | `make equiv` | `0 mismatches` |
| S5-H04 | State machine on a virtual clock, scenarios S1-S5, S7-S12 (incl. WaitStopped, STOP_TIMEOUT, REVERSE_LIMIT, ASSIST_TIMEOUT, E-stop set never cleared, sensor recovery) | FR24, FR25, NFR9, NFR11, tutor feedback | auto | `make sim` | every scenario `0 failures` |

### On-target tests: `cd build_make && make TEST=1 -j8`

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Sample pack/unpack, 15 bearings x 4 edge ranges | CR1 | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | Front word range and 17-bit stamp age across the wrap | CR1 | auto | `subsystem5_test.c` | PASS |
| S5-U03 | Profile pack/unpack round trip, every field | CR1 | auto | `subsystem5_test.c` | PASS |
| S5-U04 | Health pack/unpack at field limits | CR1 | auto | `subsystem5_test.c` | PASS |
| S5-U05 | Packed sizes on the ARM compiler: 2 / 16 / 8 / 4 B | CR1 | auto | `subsystem5_test.c` | PASS |
| S5-U06 | Telemetry packet is 29 B | CR1, IF-S5-6 | auto | `subsystem5_test.c` | PASS |
| S5-U07 | Narrow post from packed samples gives a bypass | FR25 | auto | `subsystem5_test.c` | PASS |
| S5-U08 | Profile build time on the Cortex-M0+ (printed as `[INFO]`) | CR1 item 4 | auto | `subsystem5_test.c` | PASS, under 1000 us |
| S5-U09 | Obstacle task running, >= 10 pings in 1.5 s | NFR9 | auto | `subsystem5_test.c` | PASS |
| S5-U10 | Sensor answering (no SENSOR fault; ASSIST_TIMEOUT on the bench is expected without VehicleController) | A5 | auto | needs the sonar wired | PASS |
| S5-U11 | No missed 60 ms guard ticks | NFR9 | auto | `subsystem5_test.c` | PASS |
| S5-M01 | 500-ping no-response rate on the power bank | A5 | manual | TEST=1 image, sensor at a wall for 30 s, read `no_response` / `pings` from the `[INFO]` line | < 1 % |
| S5-M02 | Servo travel 60 deg on the power bank | A5b | manual | phone slow-motion video of a 0 -> 60 deg move | <= 140 ms + 25 ms |
| S5-M03 | Range accuracy at 50, 100, 200, 300 mm, 20 readings each | NFR15 | manual | tape-measured board, log front range | within +/-10 mm |
| S5-M04 | ECHO never above 3.3 V on GP17 | safety | manual | scope or meter on GP17 during pings | <= 3.3 V |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-20 | S5-H01..H04 | not committed yet | Buddy 5 (generated run) | PASS | `logs/2026-10-20_host_tests_precommit.txt` | Re-run after committing and add a row with the commit hash |
| 2026-11-01 | bench (v2.0 driver) | not committed | Buddy 5 | FAIL | serial log, bench image v1 | Distances correct, 0 no-response, but one ping every ~3 s instead of 4/s: waits used TIMER_TIMERAWL, which is not 1 MHz in this port. Fixed in v2.1 |
| 2026-11-01 | S5-U01..U11 (v2.1 test image) | not committed | Buddy 5 | 10 PASS, U10 FAIL | serial log | U10 failed on ASSIST_TIMEOUT (no VehicleController on the bench), not a sensor fault: 1241 pings, 0 no-response. Test corrected in v2.2. Profile build 10 us (TIMER-based) |
| 2026-11-01 | bench (v2.1 driver) | not committed | Buddy 5 | PASS | serial log, bench image v2 | One line every 300 ms, 72 pings, 0 no-response, 125-1131 mm tracked by hand; servo sweep +/-30 deg clears the board |

---

## CR1 evidence (Week 13)

### 1. Data representation table

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Range (front, sample, profile nearest) | 30..2500 mm, plus CLEAR and UNKNOWN | 12 | 0..4093 mm, 0xFFE = CLEAR, 0xFFF = UNKNOWN; saturates at 4093, never wraps into a sentinel. Public API keeps the 16-bit codes 0xFFFE / 0xFFFF |
| Sample bearing | -70..+70 deg, 10 deg grid (15 values); v2.2 scans only -30..+30 (7 values) | 4 | index = bearing / 10 + 7. 7 values would fit 3 bits; 4 are kept so a remounted bracket can scan wider with no format change |
| Profile bearing, left edge, right edge | -75..+75 deg | 8 each | stored +75 (unsigned, Barr-C) |
| Width, swing left, swing right | 0..4095 mm | 12 each | saturating |
| Gap left, gap right | 0..1500 mm | 11 each | 1500 = open |
| Confidence | 0..100 % | 7 | |
| Sample count | 0..16 | 5 | |
| Valid / has profile / odometry ok / E-stop requested | yes/no | 1 each | |
| Action | 6 values | 3 | CONTINUE, SLOW, STOP, BYPASS_L, BYPASS_R, REVERSE; codes 6-7 decode as STOP |
| ObstacleScan state | 9 values | 4 | INIT, GUARD, CONFIRM, WAIT_STOP, COARSE, FINE, PUBLISH, ASSIST, FAULT |
| Fault cause | 5 values | 3 | NONE, SENSOR, STOP_TIMEOUT, REVERSE_LIMIT, ASSIST_TIMEOUT |
| Ultrasonic status | 8 values | 3 | OK, NO_ECHO, NO_RESPONSE, TOO_CLOSE, TOO_FAR, ECHO_STUCK, NOT_READY, BAD_PARAM |
| Echo width | 0..25 000 us measurable | 15 | saturates at 32 767 |
| Timestamps (front, profile) | used only as an age | 17 | ms, modular age correct up to 131 s |
| Fault run / good run / reverses | 0..10 / 0..3 / 0..2 | 4 / 2 / 2 | task-private bit-fields |
| Deadline overruns, sensor faults | counters | 8 each | saturate at 255 |
| Scans, ping counters | counters | 16 each | saturate at 65 535 |
| Profile build worst time | 0..65 535 us | 16 | saturating |
| S5 fault in the shared fault bitmask | yes/no | 1 | S5's E-stop cause bit (registered by the E-stop owner) |
| Events to VehicleController | 5 events | 5 bits | `flg_obstacle`: DETECTED, HARD_STOP, PROFILED, CLEARED, FAULT |

Two packing styles, one rule: **explicit shifts and masks** for everything
shared or sent (same layout in RAM and on the wire; a word is built in a
local and stored once), **C bit-fields** only for task-private state.

### 2. RAM before / after (measured, `logs/2026-10-20_ram_v1_vs_v2.txt`)

| Item | v1 | v2 |
|---|---|---|
| Sample buffer (16 samples) | 64 B | 32 B |
| Published profile | 32 B | 16 B |
| Health | 16 B | 8 B (and holds more fields) |
| Ping result | 12 B | 4 B |
| Ultrasonic stats | 20 B | 8 B |
| Task control scalars | 13 B in 7 variables | 4 B, one bit-field word |
| **Subsystem static RAM (data + bss)** | **225 B** | **135 B (-40 %)** |
| Flash (text) | 4 148 B | 7 256 B (+3.1 KB: encoders, WaitStopped, faults, telemetry) |
| Task stack | 1 024 B | 2 048 B (team `app_start_task` default; not affected by CR1; high-water not measured yet) |

v2 also carries new state that v1 did not have (WaitStopped timers,
odometry and E-stop registration), so the like-for-like saving is larger
than 90 B.

### 3. Message size before / after (`make pack`, P7)

| Message | Before | After |
|---|---|---|
| Obstacle status to MQTT | 201 B JSON text | 29 B binary (-86 %) |
| Profile copy between tasks (mutex hold) | 32 B | 16 B |
| Ping result returned to the task | 12 B | 4 B |

### 4. Correctness and performance

- Geometry: v1 and v2 agree on all 20 000 random sweeps (S5-H03).
- State machine: all simulation scenarios pass (S5-H04). Simulated scan
  time 1 256 ms with the v2.2 +/-30 deg sweep and 30 ms read window
  (v2.1: 1 533 ms at +/-70 deg; NFR11 limit 1.8 s).
- Detection: first forward range at most ~100 ms after an object appears
  (v1: 82 ms). The 10 ms kernel tick sets the echo read window, not CR1.
- CPU: packing adds a few shifts per field. Profile build time is
  printed on the robot by S5-U08; record it here.
- Overruns: S5-U11 must show 0 missed guard ticks.
