# Subsystem 5 — test log

**Subsystem:** obstacle sensor and bypass (HC-SR04P on an SG90 servo), rewritten from scratch by nyx on 2026-11-07  ·  **Student ID:** 2502410  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results, and
`CALIBRATION.md` for the calibration build.

The version before the rewrite, with its own test cases (S5-H01..H07,
S5-U01..U14, S5-M01..M08) and its CR1 evidence, is in git history:
`git show b5289da:testing/subsystem5/TEST_LOG.md`. Those IDs are retired
and not reused; their result rows are kept below.

## Test cases

Define a test here before running it. Test IDs never change.

### Host tests (PC, no hardware): `cd testing/subsystem5/host && make`

The host tests compile the real subsystem 5 sources against a simulated
HC-SR04P, servo, IR pair and car (`s5_sim.c`, `motion_mock.c`), with the
real Buddy 3 line follower (`b3_line.c`).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-H11 | sin, cos, tan, atan2, sqrt, round, wrap without libm | NFR (no libm on target) | auto | `make unit` | `[PASS] S5-H11` |
| S5-H12 | the real sweep (S5 task code) on the simulated sensor: boxes 8..32 cm wide, faces 20..38 cm away, 3 offsets | FR22 | auto | `make unit` | width within 20 mm, each edge within 15 mm, sweep under 4 s |
| S5-H13 | 15 % of echoes missed, 30 seeds: nyx's 21 cm box | FR22 | auto | `make unit` | width within 25 mm |
| S5-H14 | room beside the object: another box or a wall where the car would pass rules that side out (only that side); one well out of the way does not | FR23 | auto | `make unit` | `[PASS] S5-H14` |
| S5-H15 | bypass ramps of 1..400 mm: no steeper than 40 deg, no tighter than R150, smooth, end where asked | FR23, smooth swerve | auto | `make unit` | `[PASS] S5-H15` |
| S5-H16 | car-outline-to-box check against a dense distance computation (30 mm margin) | FR23 | auto | `make unit` | within -3 / +6 mm |
| S5-H17 | steering law: starts 12 mm / 5 deg off, follows a 215 mm bypass | FR24 | auto | `make unit` | within 6 mm, ends on the line |
| S5-H18 | dead reckoning: quarter circle R300 | FR24 | auto | `make unit` | ends at (300, 300), 90 deg |
| S5-H19 | ping timing: 14+ pings/s, a box at 250 mm reads 250 +- 8 mm and counts as close, a 150 ms no-echo pulse is not a dead sensor, LOOK keeps pinging while its angle creeps at 10 and 60 deg/s | FR21, NFR12 | auto | `make unit` (also `build/test_s5_unit -p 20`) | `[PASS] S5-H19` at 10 and 20 ms polls |
| S5-H20 | no ECHO pulse at all: dead within 1 s; a working sensor is not | FR25 | auto | `make unit` | `[PASS] S5-H20` |
| S5-H21 | whole episodes on a simulated car (S5 task, bypass in the VC's task, Buddy 3, Buddy 2 with 60 ms lag), scenarios C1..C60 below, 3 seeds each (10 for a full run) | FR21-FR25 | auto | `make car`; full: `build/test_s5_car -s 10` and `-s 10 -p 20` | no contact, 10 mm clearance or more, handed back within 12 mm of OUR line, stays on it (24 mm) for 300 mm after; L1..L3 shown only |

S5-H21 scenarios:

| ID | Scenario | ID | Scenario |
|---|---|---|---|
| C1 | nyx's 21 x 16 cm box on the line, 2 m ahead | C23 | car starts 10 mm off the line, 8 deg to it |
| C2 | box 60 mm right of the line: left is shorter | C24 | sensor mounted 10 mm left of the centre |
| C3 | box 60 mm left: right is shorter | C25 | two boxes 1.4 m apart |
| C4 | box 25 mm right: left shorter by 50 mm | C26 | no sensor: FR25 fault |
| C5 | narrow 10 x 10 cm box | C27 | right encoder dies at the obstacle: gives up |
| C6 | wide 30 cm box | C28 | box taken away while the car stands |
| C7 | deep box, 21 x 40 cm | C29 | noisy sensor: 5 mm noise, 10 % dropouts |
| C8 | box put down 100 mm in front of the car | C30 | box 120 mm right: line clear of it |
| C9 | box put down 50 mm in front of the car | C31 | second box 56 cm behind the first |
| C10 | walls both sides: waits (fault bit 0x40, no E-stop), left wall taken away | C32 | second box 75 cm behind, 30 mm left |
| C11 | walls both sides: waits, then the box is taken away | C33 | second box 40 cm behind the first |
| C12 | second box beside it on the left: goes right | C34 | second box 30 cm behind the first |
| C13 | other lanes 250 mm either side: back onto OUR line | C35 | second box 30 cm behind, 60 mm left: in the left pass lane |
| C14 | encoders 3 % long, real track 5 % wider | C36 | second box 40 cm behind, 100 mm right |
| C15 | encoders 3 % short, track 5 % narrower, lanes | C37 | second box 40 cm behind, 30 cm wide |
| C16 | real beam 19 deg (calibration says 22) | C38 | second box 40 cm behind, 40 cm deep |
| C17 | real beam 25 deg (calibration says 22) | C39 | three boxes, gaps 50 and 30 cm, 30 mm left / right |
| C18 | servo 4 deg off to the left, slow servo | C40 | second box 15 cm behind, 50 mm into the left pass lane (review D1) |
| C19 | 150 ms no-echo pulse (slow HC-SR04P variant) | C41 | 3 cm post put down in the left pass lane as the car turns out (review D1) |
| C20 | track at 37 deg to the odometry frame | L1 | LIMIT: real beam 16 deg (calibration 22) |
| C21 | box right after an R500 bend | L2 | LIMIT: real beam 28 deg (calibration 22) |
| C22 | box turned 15 deg |  |  |
| C42 | 21 x 16 cm box, car stops 30 cm short: goes round with no backing up (2026-11-08) | C47 | line 70 mm further right behind the box: IRs pick it up |
| C43 | the same box turned, 16 cm face: no backing up | C48 | left encoder 0.5 % long, right 0.5 % short: ends ~6 cm left of the line, no IR sees black (nyx's track run) |
| C44 | real track 10 % narrower (turns 10 % more): IRs pick the line up | C49 | the same mirrored: ends ~6 cm right |
| C45 | real track 10 % wider (turns 10 % less): IRs pick the line up | C50 | left encoder 0.75 % long, right 0.75 % short: ends ~10 cm left |
| C46 | line 70 mm further left behind the box: IRs pick it up | C51 | the same mirrored: ends ~10 cm right |
| C52 | junction's branch across the way back (going left): the IRs leave it alone (2026-11-08) | C57 | encoders 2 % apart (left long): the compass keeps the heading |
| C53 | junction where the car turns onto the line (going left): the turn is not cut short | C58 | no compass (IMU unplugged), encoders 1 % apart: encoders and IRs only |
| C54 | junction where the car turns onto the line (going right): the turn is not cut short | C59 | compass up to 6 deg off with heading (poor calibration), encoders 1 % apart |
| C55 | junction's branch on the side the car comes back from (going right): left alone | C60 | iron in the box pulls the field 30 % near it: the compass is not used there |
| C56 | junction's branch at 45 deg across the way back (going left): left alone | L3 | LIMIT: the IRs cross our line right where a junction's branch meets it |

nyx, 2026-11-08: C52..C56 have encoders 1 % apart, so the car comes back
off the line, past or onto a junction's branch (tape across our line from
it). From `logs/2026-11-08_host_tests_5.txt` on, the simulated car in
every scenario has a calibrated compass (Buddy 4's magnetometer, up to
2 deg off with heading) unless the scenario says otherwise; C58 is the
car with `S5_MAG_SIGN` 0, as before calibration step 4.

In C31..C41 the episode ends on the line behind the last box; a second
box met in the pass lane or on the way back stops the car, which sweeps
again and goes round it too, never back across the box it has just gone
round. Rare layouts the car cannot plan from where
it stops (two boxes even closer, wider ones) end waiting with fault bit
0x40 and no E-stop, as for no way round.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-H22 | line search (`robot.ps1 search`, nyx 2026-11-08): from a white start the car drives straight until the IRs meet a line, then lines up along it and Buddy 3 follows it; scenarios Q1..Q31 and none below, 5 seeds each (10 for a full run) | VC start, Buddy 3 hand-over | auto | `make search`; full: `build/test_search -s 10` | handed to Buddy 3, then 300..700 mm followed with the IR centre within 15 mm of the line, heading within 12 deg of along it; "none" stops with `no line within` |

S5-H22 scenarios (angle = the line to the car's heading at the start):

| ID | Scenario | ID | Scenario |
|---|---|---|---|
| Q1..Q3 | line straight across (90 deg), 40 cm ahead, from the other side, 1.2 m ahead: turns LEFT (`S5_SEARCH_TURN`) | Q19 | started on the line, 3 deg off: Buddy 3 takes it |
| Q4 / Q5 | 85 / 95 deg (within 8 of square): LEFT | Q20 | started on the line, 2 deg off the other way |
| Q6 / Q7 | 80 / 100 deg: the shorter way | Q21 | started on the line, a R400 left bend 60 deg 40 cm ahead |
| Q8 / Q9 | 60 / 120 deg | Q22 | started on the line in that bend, IR_L on the tape |
| Q10 / Q11 | 45 / 135 deg | Q23 | 45 deg, encoders 2 % long and L/R 0.5 % apart |
| Q12 / Q13 / Q14 | 30 / 35 / 25 deg | Q24 | straight across, encoders 2 % long and L/R 0.5 % apart |
| Q15 / Q16 / Q17 | 20 / 15 / 10 deg: Buddy 3 takes it from one IR | Q25 | a box in the way, taken away after 4 s, then the line |
| Q18 | -30 deg, coming from the left of the line | Q26 / Q27 | straight across, real track 10 % wider / narrower |
| none | facing away from the line: stops with the fault (0x40) after 2 m | Q28 / Q29 | 60 deg track 10 % wider / 30 deg track 10 % narrower |
|  |  | Q30 / Q31 | straight across / 45 deg, real IRs 12 mm off the centre (not 15) |

### On-target tests: `cd build_make && make TEST=1 -j8`

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-T01 | the S5 task pings | NFR12 | auto | `subsystem5_test.c` | 12 or more pings in 1 s |
| S5-T02 | the HC-SR04P answers on TRIG GP16 / ECHO GP17 | FR25 | auto | needs the sensor wired and powered | `[TEST] S5-T02 PASS`, range printed |
| S5-T03 | the maths on the RP2040 (soft float, no libm) | NFR | auto | `subsystem5_test.c` | within 1e-4 |
| S5-T04 | width of a 210 mm box at 250 mm from a synthetic sweep done the way the S5 task does it (coarse, then fine at each edge), with the calibrated beam | FR22 | auto | `subsystem5_test.c` | 210 +- 15 mm (passes for any beam 12..30 deg) |
| S5-T05 | a box 80 mm right of the line: left is the shorter side | FR23 | auto | `subsystem5_test.c` | PASS |
| S5-T06 | something beside the box blocks that side only | FR23 | auto | `subsystem5_test.c` | PASS |
| S5-T07 | bypass ramp within 40 deg and R150 | FR23 | auto | `subsystem5_test.c` | PASS |
| S5-T08 | car outline check: too late a ramp hits the box | FR23 | auto | `subsystem5_test.c` | PASS |
| S5-T09 | encoder ticks to distance | FR24 | auto | `subsystem5_test.c` | PASS |

### On the car (manual)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-M09 | calibration | FR22 | manual | `CALIBRATION.md`, `make S5CAL=1`; copy the 5 values into `s5_config.h`; record them in Notes | afterwards nyx's box reads 210 +- 10 mm |
| S5-M10 | nyx's box on the line, car line following | FR21-FR24 | manual | box 1 m ahead on a straight; serial or MQTT log; 3 runs | stops at 30 cm or more; sweeps; goes round the shorter side (random when centred); no contact; `back on the line`; follows the SAME line on |
| S5-M11 | box off centre (6 cm right of the line) | FR23 | manual | as M10 | goes LEFT every time |
| S5-M12 | box put down 10 cm in front of the stopped car | FR23 | manual | car stopped on the line, box placed, `robot.ps1 go` | `only N mm away: backing off`, sweeps, goes round; no contact |
| S5-M13 | no way round: box with another box or a wall close on each side | FR23, fault bit | manual | as M10 | stops, `no way round ... fault bit 0x40 set, no E-stop`; carries on by itself once one side is cleared |
| S5-M14 | width of other objects | FR22 | manual | a 10 cm and a 30 cm object, tape measure | printed width within 15 mm |
| S5-M15 | sensor unplugged while line following | FR25 | manual | pull the ECHO lead | `sensor fault (FR25)`, E-stop with fault bit 0x08 |

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
| 2026-11-04 | S5-H01..H07 (v3.0) | branch buddy5-v3.0, not merged | Buddy 5 (generated run) | PASS | `logs/2026-11-04_host_tests_v3.0.txt`, `logs/2026-11-04_bypass_sim_v3.0.txt` | H05 centre error 1.4 -> 0.4 deg, width 28 -> 10 mm. H06 finds trim -35, backlash 15, beam 110, scale 3983, offset 7 on the faulty sim robot. H07 11/11 scenarios, no collision, rejoin within 7 mm, C7 safe stop. Simulated scan 1391 ms (NFR11 1.8 s) |
| 2026-11-07 | — | — | nyx | — | — | Subsystem 5 rewritten from scratch. The rows above are for the old version (logs in git history at b5289da) |
| 2026-11-07 | S5-H11..H21 | not committed (nyx's PC) | nyx (generated run) | PASS | `logs/2026-11-07_host_tests.txt` | After the independent review (D1 fixes). H21 (C1..C41): 430 runs at 10 ms polls and 430 at 20 ms, 0 failed, every pass 18 mm or more from the box; L1 (beam 6 deg off) passes 9 mm from it. Reviewers' layouts (not in `make`): 612 second-box layouts x 4 seeds, 9 runs touched a box (168 before the fixes); 187 more layouts x 10 seeds, 3 where a car can get round (106 before), plus 105 in layouts no car can clear (a box put down 55 mm or less in front of it) |
| 2026-11-08 | S5-H11..H21 | not committed (nyx's PC) | nyx (generated run) | 1059 PASS, 1 FAIL | `logs/2026-11-08_host_tests.txt` | After nyx's first track runs: steeper ramp out before any backing up (no back-up from 30 cm, C42/C43), IR pick-up of the line (both IRs, or one then the other), search at 30 deg with the first line ahead taken as ours when no IR saw black. H21 (C1..C51): 530 runs at 10 ms polls, 1 failed (C36 seed 4 of 10: two boxes 40 cm apart, the far-corner look took both as one 567 mm deep object and the car waits with 0x40, no E-stop); 530 at 20 ms, 0 failed. With the old 12 deg search C48 (car ends 66 mm off the line, as on the track) fails |
| 2026-11-08 | S5-H11..H21 | not committed (nyx's PC) | nyx (generated run) | 1059 PASS, 1 FAIL | `logs/2026-11-08_host_tests_2.txt` | After nyx's second track run: a side only the edge of an echo's beam reaches is used when the other side is blocked outright (both only maybe: waits, C10/C11); ramp back in sharper (S5_IN_HEADING_DEG 50, S5_IN_RADIUS_MM 130; 60 deg / R110 hit the second box in C38 and lost the line in C39 in 1 of 10 seeds). H21: 530 runs at 10 ms, 1 failed (C36 seed 4, as before); 530 at 20 ms, 0 failed |
| 2026-11-08 | S5-H11..H21 | not committed (nyx's PC) | nyx (generated run) | 1059 PASS, 1 FAIL | `logs/2026-11-08_host_tests_3.txt` | The pick-up takes the angle to the line from how far the car went between the two IRs meeting the tape (not the encoders' heading) and drives on and turns by it. H21: 530 runs at 10 ms, 1 failed (C36 seed 4, as before); 530 at 20 ms, 0 failed. In C50 (encoders drifted) the IRs gave 19 deg where the encoders said 30 |
| 2026-11-08 | S5-H11..H21 | not committed (nyx's PC) | nyx (generated run) | 1179 PASS, 1 FAIL | `logs/2026-11-08_host_tests_4.txt` | After nyx's third track run (the car took a junction's branch 5 cm from our line): the line's direction from where the two IRs met the tape; a crossing more than 20 deg off it, or both IRs black at once under 45 deg, is a junction and left alone; the turn onto the line counts the IRs only within 6 deg of straight. H21 (C1..C56, L1..L3): 590 runs at 10 ms, 1 failed (C36 seed 4, as before); 590 at 20 ms, 0 failed. L3 (crossing where a branch meets our line, shown only) got round in this run |
| 2026-11-08 | S5-H11..H21 | not committed (nyx's PC) | nyx (generated run) | 1259 PASS, 1 FAIL | `logs/2026-11-08_host_tests_5.txt` | The compass (Buddy 4's magnetometer, `make S5CAL=2` calibrates it) pulls the encoders' heading back when the two are more than 3 deg apart; the drive onto the line stops 15 mm short (the wheels ran on 8 mm past it) and the turn ends with the IRs pointed at the line, 1 deg past (C45 had handed over 14 mm off). H21 (C1..C60, L1..L3): 630 runs at 10 ms, 1 failed (C36 seed 4, as before); 630 at 20 ms, 0 failed; L3 got round in every run. Correcting at every degree of a compass 2 deg off put the car into the second box in C35 and C38; that is why the 3 deg dead band. The simulated compass's noise has its own random numbers, so other noise per seed is as in `_4` |
| 2026-11-08 | S5-H11..H22 | not committed (nyx's PC) | nyx (generated run) | 1579 PASS, 1 FAIL | `logs/2026-11-08_host_tests_6.txt` | New S5-H22: line search (`robot.ps1 search`): from white the car drives straight until the IRs meet a line, lines up along it and hands over to Buddy 3. 320 runs (Q1..Q31 and none, 10 seeds), 0 failed. Also the junction rule now counts each IR's own black run (Buddy 3). H21: 630 runs at 10 ms, 1 failed (C36 seed 4, as before); 630 at 20 ms, 0 failed |
