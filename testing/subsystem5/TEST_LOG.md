# Subsystem 5 — test log

**Subsystem:** obstacle sensor and bypass (HC-SR04P on an SG90 servo), rewritten from scratch on 2026-11-07  ·  **Student ID:** 2502410  ·  **Code:** `app_program/subsystem5/`

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
| S5-H11 | sin, cos, tan, atan2, sqrt, round, wrap without libm | (no Week 6 req) | auto | `make unit` | `[PASS] S5-H11` |
| S5-H12 | the real sweep (S5 task code) on the simulated sensor: boxes 8..32 cm wide, faces 20..38 cm away, 3 offsets | FR21, NFR14 | auto | `make unit` | width within 20 mm, each edge within 15 mm, sweep under 2 s |
| S5-H23 | NFR9/NFR11: time from detection to the finished profile on the simulated car (sweep starts while braking) | NFR9, NFR11 | auto | `make car`, then `build/test_s5_car v` timed as in `logs/2026-11-14_profile_time.txt` | median within 1.8 s |
| S5-H13 | 15 % of echoes missed, 30 seeds: the 21 cm box | FR21, NFR14 | auto | `make unit` | width within 25 mm |
| S5-H14 | room beside the object: another box or a wall where the car would pass rules that side out (only that side); one well out of the way does not | FR22 | auto | `make unit` | `[PASS] S5-H14` |
| S5-H15 | bypass ramps of 1..400 mm: no steeper than 40 deg, no tighter than R150, smooth, end where asked | FR22, smooth swerve | auto | `make unit` | `[PASS] S5-H15` |
| S5-H16 | car-outline-to-box check against a dense distance computation (30 mm margin) | FR22 | auto | `make unit` | within -3 / +6 mm |
| S5-H17 | steering law: starts 12 mm / 5 deg off, follows a 215 mm bypass | FR23 | auto | `make unit` | within 6 mm, ends on the line |
| S5-H18 | dead reckoning: quarter circle R300 | FR23 | auto | `make unit` | ends at (300, 300), 90 deg |
| S5-H19 | ping timing: 14+ pings/s, a box at 250 mm reads 250 +- 8 mm and counts as close, a 150 ms no-echo pulse is not a dead sensor, LOOK keeps pinging while its angle creeps at 10 and 60 deg/s | FR21, NFR10, NFR15 | auto | `make unit` (also `build/test_s5_unit -p 20`) | `[PASS] S5-H19` at 10 and 20 ms polls |
| S5-H20 | no ECHO pulse at all: dead within 1 s; a working sensor is not | FR25 | auto | `make unit` | `[PASS] S5-H20` |
| S5-H21 | whole episodes on a simulated car (S5 task, bypass in the VC's task, Buddy 3, Buddy 2 with 60 ms lag), scenarios C1..C70 below (C61..C67 added 2026-11-09: the line bends behind the box; C68..C70 added 2026-11-09: box put down close, no backing up), 3 seeds each (10 for a full run) | FR21-FR25 | auto | `make car`; full: `build/test_s5_car -s 10` and `-s 10 -p 20` | no contact, 10 mm clearance or more, handed back within 12 mm of OUR line, stays on it (24 mm) for 300 mm after; L1..L4 shown only |

S5-H21 scenarios:

| ID | Scenario | ID | Scenario |
|---|---|---|---|
| C1 | the 21 x 16 cm box on the line, 2 m ahead | C23 | car starts 10 mm off the line, 8 deg to it |
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
| C18 | servo 4 deg off to the left, slow servo | C40 | second box 15 cm behind, 50 mm into the left pass lane |
| C19 | 150 ms no-echo pulse (slow HC-SR04P variant) | C41 | 3 cm post put down in the left pass lane as the car turns out |
| C20 | track at 37 deg to the odometry frame | L1 | LIMIT: real beam 16 deg (calibration 22) |
| C21 | box right after an R500 bend | L2 | LIMIT: real beam 28 deg (calibration 22) |
| C22 | box turned 15 deg |  |  |
| C42 | 21 x 16 cm box, car stops 30 cm short: goes round with no backing up (2026-11-08) | C47 | line 70 mm further right behind the box: IRs pick it up |
| C43 | the same box turned, 16 cm face: no backing up | C48 | left encoder 0.5 % long, right 0.5 % short: ends ~6 cm left of the line, no IR sees black (a track run) |
| C44 | real track 10 % narrower (turns 10 % more): IRs pick the line up | C49 | the same mirrored: ends ~6 cm right |
| C45 | real track 10 % wider (turns 10 % less): IRs pick the line up | C50 | left encoder 0.75 % long, right 0.75 % short: ends ~10 cm left |
| C46 | line 70 mm further left behind the box: IRs pick it up | C51 | the same mirrored: ends ~10 cm right |
| C52 | junction's branch across the way back (going left): the IRs leave it alone (2026-11-08) | C57 | encoders 2 % apart (left long): the compass keeps the heading |
| C53 | junction where the car turns onto the line (going left): the turn is not cut short | C58 | no compass (IMU unplugged), encoders 1 % apart: encoders and IRs only |
| C54 | junction where the car turns onto the line (going right): the turn is not cut short | C59 | compass up to 6 deg off with heading (poor calibration), encoders 1 % apart |
| C55 | junction's branch on the side the car comes back from (going right): left alone | C60 | iron in the box pulls the field 30 % near it: the compass is not used there |
| C56 | junction's branch at 45 deg across the way back (going left): left alone | L3 | LIMIT: the IRs cross our line right where a junction's branch meets it |
| C61 | line bends left R300 behind the box, car comes back from the left (2026-11-09) | C65 | line bends left R200 behind the box, car comes back from the left |
| C62 | line bends right R300 behind the box, car comes back from the left | C66 | line bends left R500 from the box's back, car from the left, encoders 1 % apart |
| C63 | line bends left R300 behind the box, car comes back from the right | C67 | line bends right R500 from the box's back, car from the left, encoders 1 % apart |
| C64 | line bends right R300 behind the box, car comes back from the right | L4 | LIMIT: line bends right R200 behind the box, car from the left (one IR meets its edge, the bend takes it away) |
| C68 | box put down 150 mm in front: turns out on the spot, no backing up (2026-11-09) | C70 | 25 cm wide box put down 200 mm in front, encoders 1 % apart: no backing up |
| C69 | box put down 150 mm in front, 40 mm left: no backing up |  |  |

the car backs up only when no way round is open (both
sides blocked, or an edge outside the sweep and no side open), or when
not even a turn on the spot clears the box. Too close for a ramp, it
turns out on the spot (45, 60, 75 or 90 deg), drives straight out to
the pass offset and turns back. C68..C70 allow 1 mm of reversing; C8
(box 100 mm from the car's front) now goes round with none. A box 50 mm
from the front (C9) has its edges outside the +-60 deg sweep, so the car
still backs off 160 mm to see it.

C52..C56 have encoders 1 % apart, so the car comes back
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
| S5-H22 | line search (`robot.ps1 search`): from a white start the car drives straight until the IRs meet a line, then lines up along it and Buddy 3 follows it; scenarios Q1..Q48 and none below (Q32..Q48 added 2026-11-09: the line is a bend, or an IR starts on it), 5 seeds each (10 for a full run) | VC start, Buddy 3 hand-over | auto | `make search`; full: `build/test_search -s 10` | handed to Buddy 3, then 300..700 mm followed with the IR centre within 15 mm of the line, heading within 12 deg of along it; "none" stops with `no line within` |

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
|  |  | Q30 / Q31 | straight across / 45 deg, real IRs 12 mm off the centre (not 15; since 2026-11-09 the config says 12.5) |

Added 2026-11-09 (the car did not pick up a curved line at the start). The line bends 90 deg from 40 cm before the car's x (Q47: a straight line; Q48: a 60 deg bend from the car's x):

| ID | Scenario | ID | Scenario |
|---|---|---|---|
| Q32 / Q38 | across a R300 left / right bend, 48 deg to it, from outside | Q40 / Q41 | across a R200 left / right bend, from outside |
| Q33 | across a R300 left bend from inside, 60 deg to it | Q42 | square across a R200 left bend from inside: LEFT, along it |
| Q34 / Q39 | near a R300 left / right bend, outside, IRs 6 cm off, 40 deg in | Q43 / Q44 | Q40 / Q42 with wheels that turn the car 20 % less |
| Q35 | beside a R300 left bend, inside, IRs 6 cm off, on its tangent | Q45 | Q38 with real IRs 12 mm off the centre |
| Q36 | Q32 with wheels that turn the car 30 % less | Q46 | Q41 with encoders 2 % long and L/R 0.5 % apart |
| Q37 | square across a R300 left bend from inside: LEFT, along it | Q47 | started on a straight line 8 mm right of it, IR_L on the tape |
|  |  | Q48 | started on the line in a R400 right bend, IR_R on the tape (Q22 mirrored) |

Buddy 3's line following on a simulated car (`make follow`, `test_follow.c`
with the real `b3_line.c` and `motion_mock.c`): scenarios F1..F36 (straights,
bends R150..R500, S-curves, a circle, a hairpin, standing starts on a bend
F31..F36; F23..F30 are bends tighter than R130, shown only, LIMIT when lost).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| B3-H10 | line following with Buddy 2's 20 ms goal, ramp and a 60 ms wheel lag; IRs 12.5 mm (as built) and 20 mm off the centre | FR11 | auto | `make follow` (`test_follow -n`, `-s 20 -n`) | `All line-following scenarios passed` |
| B3-H11 | as B3-H10 with Buddy 2's real PI wheel loop on a motor fitted to the car's `[VC] curve:` log of 2026-11-09 (15 % more drive per mm/s than the feed-forward, turning load 20 pm; added 2026-11-09) | FR11 | auto | `make follow` (`test_follow -m 50,1.15 -y 20,10 -n`) | `All line-following scenarios passed`; the curve log line is shown |
| B3-H12 | both IRs black for 60 mm inside a bend, as the car logged (added 2026-11-09) | FR11 | auto, shown only | `make follow` (`test_follow -x 200,60 -n -q`, also with `-m 50,1`) | shown: lines lost (the IRs see nothing useful for 60 mm) |

### On-target tests: `cd build_make && make TEST=1 -j8`

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-T01 | the S5 task pings | NFR10 | auto | `subsystem5_test.c` | 12 or more pings in 1 s |
| S5-T02 | the HC-SR04P answers on TRIG GP16 / ECHO GP17 | FR25 | auto | needs the sensor wired and powered | `[TEST] S5-T02 PASS`, range printed |
| S5-T03 | the maths on the RP2040 (soft float, no libm) | (no Week 6 req) | auto | `subsystem5_test.c` | within 1e-4 |
| S5-T04 | width of a 210 mm box at 250 mm from a synthetic sweep done the way the S5 task does it (coarse, then fine at each edge), with the calibrated beam | FR21, NFR14 | auto | `subsystem5_test.c` | 210 +- 15 mm (passes for any beam 12..30 deg) |
| S5-T05 | a box 80 mm right of the line: left is the shorter side | FR22 | auto | `subsystem5_test.c` | PASS |
| S5-T06 | something beside the box blocks that side only | FR22 | auto | `subsystem5_test.c` | PASS |
| S5-T07 | bypass ramp within 40 deg and R150 | FR22 | auto | `subsystem5_test.c` | PASS |
| S5-T08 | car outline check: too late a ramp hits the box | FR22 | auto | `subsystem5_test.c` | PASS |
| S5-T09 | encoder ticks to distance | FR23 | auto | `subsystem5_test.c` | PASS |

### On the car (manual)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-M09 | calibration | FR21, NFR14, NFR15 | manual | `CALIBRATION.md`, `make S5CAL=1`; copy the 5 values into `s5_config.h`; record them in Notes | afterwards the track box reads 210 +- 10 mm |
| S5-M10 | the track box on the line, car line following | FR21-FR23 | manual | box 1 m ahead on a straight; serial or MQTT log; 3 runs | stops at 30 cm or more; sweeps; goes round the shorter side (random when centred); no contact; `back on the line`; follows the SAME line on |
| S5-M11 | box off centre (6 cm right of the line) | FR22 | manual | as M10 | goes LEFT every time |
| S5-M12 | box put down 10 cm in front of the stopped car | FR22 | manual | car stopped on the line, box placed, `robot.ps1 go` | sweeps from where it stands (the sweep starts at detection), goes round; no contact |
| S5-M13 | no way round: box with another box or a wall close on each side | FR22, fault bit | manual | as M10 | stops, `no way round ... fault bit 0x40 set, no E-stop`; carries on by itself once one side is cleared |
| S5-M14 | width of other objects | FR21, NFR14 | manual | a 10 cm and a 30 cm object, tape measure | NFR14 limit (team, 2026-11-14): never more than 20 % narrower than the object, up to 170 mm wider accepted (the sonar beam reads small faces wide, which only widens the bypass). The 2026-11-14 runs read a 20 cm box 195 and 247 mm and a 9 cm face 257 mm |
| S5-M15 | sensor unplugged while line following | FR25 | manual | pull the ECHO lead | `sensor fault (FR25)`, E-stop with fault bit 0x08 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-20 | S5-H01..H04 | not committed yet | Buddy 5 | PASS | `logs/2026-10-20_host_tests_precommit.txt` | Re-run after committing and add a row with the commit hash |
| 2026-11-01 | bench (v2.0 driver) | not committed | Buddy 5 | FAIL | serial log, bench image v1 | Distances correct, 0 no-response, but one ping every ~3 s instead of 4/s: waits used TIMER_TIMERAWL, which is not 1 MHz in this port. Fixed in v2.1 |
| 2026-11-01 | S5-U01..U11 (v2.1 test image) | not committed | Buddy 5 | 10 PASS, U10 FAIL | serial log | U10 failed on ASSIST_TIMEOUT (no VehicleController on the bench), not a sensor fault: 1241 pings, 0 no-response. Test corrected in v2.2. Profile build 10 us (TIMER-based) |
| 2026-11-01 | bench (v2.1 driver) | not committed | Buddy 5 | PASS | serial log, bench image v2 | One line every 300 ms, 72 pings, 0 no-response, 125-1131 mm tracked by hand; servo sweep +/-30 deg clears the board |
| 2026-11-04 | S5-H01..H07 (v3.0) | branch buddy5-v3.0, not merged | Buddy 5 | PASS | `logs/2026-11-04_host_tests_v3.0.txt`, `logs/2026-11-04_bypass_sim_v3.0.txt` | H05 centre error 1.4 -> 0.4 deg, width 28 -> 10 mm. H06 finds trim -35, backlash 15, beam 110, scale 3983, offset 7 on the faulty sim robot. H07 11/11 scenarios, no collision, rejoin within 7 mm, C7 safe stop. Simulated scan 1391 ms (NFR11 1.8 s) |
| 2026-11-07 | — | — | Buddy 3 | — | — | Subsystem 5 rewritten from scratch. The rows above are for the old version (logs in git history at b5289da) |
| 2026-11-07 | S5-H11..H21 | not committed | Buddy 3 | PASS | `logs/2026-11-07_host_tests.txt` | After a code review. H21 (C1..C41): 430 runs at 10 ms polls and 430 at 20 ms, 0 failed, every pass 18 mm or more from the box; L1 (beam 6 deg off) passes 9 mm from it. Reviewers' layouts (not in `make`): 612 second-box layouts x 4 seeds, 9 runs touched a box (168 before the fixes); 187 more layouts x 10 seeds, 3 where a car can get round (106 before), plus 105 in layouts no car can clear (a box put down 55 mm or less in front of it) |
| 2026-11-08 | S5-H11..H21 | not committed | Buddy 3 | 1059 PASS, 1 FAIL | `logs/2026-11-08_host_tests.txt` | After the first track runs: steeper ramp out before any backing up (no back-up from 30 cm, C42/C43), IR pick-up of the line (both IRs, or one then the other), search at 30 deg with the first line ahead taken as ours when no IR saw black. H21 (C1..C51): 530 runs at 10 ms polls, 1 failed (C36 seed 4 of 10: two boxes 40 cm apart, the far-corner look took both as one 567 mm deep object and the car waits with 0x40, no E-stop); 530 at 20 ms, 0 failed. With the old 12 deg search C48 (car ends 66 mm off the line, as on the track) fails |
| 2026-11-08 | S5-H11..H21 | not committed | Buddy 3 | 1059 PASS, 1 FAIL | `logs/2026-11-08_host_tests_2.txt` | After the second track run: a side only the edge of an echo's beam reaches is used when the other side is blocked outright (both only maybe: waits, C10/C11); ramp back in sharper (S5_IN_HEADING_DEG 50, S5_IN_RADIUS_MM 130; 60 deg / R110 hit the second box in C38 and lost the line in C39 in 1 of 10 seeds). H21: 530 runs at 10 ms, 1 failed (C36 seed 4, as before); 530 at 20 ms, 0 failed |
| 2026-11-08 | S5-H11..H21 | not committed | Buddy 3 | 1059 PASS, 1 FAIL | `logs/2026-11-08_host_tests_3.txt` | The pick-up takes the angle to the line from how far the car went between the two IRs meeting the tape (not the encoders' heading) and drives on and turns by it. H21: 530 runs at 10 ms, 1 failed (C36 seed 4, as before); 530 at 20 ms, 0 failed. In C50 (encoders drifted) the IRs gave 19 deg where the encoders said 30 |
| 2026-11-08 | S5-H11..H21 | not committed | Buddy 3 | 1179 PASS, 1 FAIL | `logs/2026-11-08_host_tests_4.txt` | After the third track run (the car took a junction's branch 5 cm from our line): the line's direction from where the two IRs met the tape; a crossing more than 20 deg off it, or both IRs black at once under 45 deg, is a junction and left alone; the turn onto the line counts the IRs only within 6 deg of straight. H21 (C1..C56, L1..L3): 590 runs at 10 ms, 1 failed (C36 seed 4, as before); 590 at 20 ms, 0 failed. L3 (crossing where a branch meets our line, shown only) got round in this run |
| 2026-11-08 | S5-H11..H21 | not committed | Buddy 3 | 1259 PASS, 1 FAIL | `logs/2026-11-08_host_tests_5.txt` | The compass (Buddy 4's magnetometer, `make S5CAL=2` calibrates it) pulls the encoders' heading back when the two are more than 3 deg apart; the drive onto the line stops 15 mm short (the wheels ran on 8 mm past it) and the turn ends with the IRs pointed at the line, 1 deg past (C45 had handed over 14 mm off). H21 (C1..C60, L1..L3): 630 runs at 10 ms, 1 failed (C36 seed 4, as before); 630 at 20 ms, 0 failed; L3 got round in every run. Correcting at every degree of a compass 2 deg off put the car into the second box in C35 and C38; that is why the 3 deg dead band. The simulated compass's noise has its own random numbers, so other noise per seed is as in `_4` |
| 2026-11-08 | S5-H11..H22 | not committed | Buddy 3 | 1579 PASS, 1 FAIL | `logs/2026-11-08_host_tests_6.txt` | New S5-H22: line search (`robot.ps1 search`): from white the car drives straight until the IRs meet a line, lines up along it and hands over to Buddy 3. 320 runs (Q1..Q31 and none, 10 seeds), 0 failed. Also the junction rule now counts each IR's own black run (Buddy 3). H21: 630 runs at 10 ms, 1 failed (C36 seed 4, as before); 630 at 20 ms, 0 failed |
| 2026-11-09 | S5-H11..H22 | not committed | Buddy 3 | 1749 PASS, 1 FAIL | `logs/2026-11-09_host_tests_7.txt` | the search did not pick up a curved line at the start, and the IRs are 25 mm apart (S5_IR_HALF_GAP_MM 15 -> 12.5, simulators too; S5_CAP_BOTH_DEG 45 -> 36 to match). The search's turn onto the line now stops on the IRs (the IR on the side turned towards black, then stop when the other goes black or the tape is centred), not at the angle from the crossing (a bend's chord is up to 25 deg off it); a start with one IR on a line turns onto it the same way. New Q32..Q48; with the old turn 7 of the 15 bend cases failed. H22: 490 runs (Q1..Q48 and none, 10 seeds), 0 failed; 980 at 20 seeds, 0 failed. H21: 630 runs at 10 ms, 1 failed (C36 seed 4, as before); 630 at 20 ms, 0 failed. B3-H10 shown-only default run at 25 mm: 8 scenarios over the weaving limit (4 at 30 mm), no line lost |
| 2026-11-09 | S5-H11..H22 | not committed (branch 37) | Buddy 3 | 1909 PASS, 1 FAIL | `logs/2026-11-09_host_tests_8.txt` | after a bypass the car found the line but did not line up along it well and sometimes went off it; use the search's IR rule. The bypass's turn onto the line now stops on the IRs too (the IR on the side turned towards black, then the other, or the tape centred), counted from 25 deg before the crossing's direction (S5_CAP_BEND_DEG); black under both IRs at once, or under the far one first, is a junction, and the turn goes by the angle as before; an IR already on the line at the start of the turn turns towards it. The car stands still before Buddy 3 takes over (it was handed over still turning). S5_CAP_LINE_DEG 20 -> 30 (a R200 bend runs 24 deg off 80 mm in). motion_mock: STOPPING ends as Buddy 2's does (setpoints under 5 mm/s for 60 ms, was wheels exactly 0, 1.2 s with lag). New C61..C67 (the line bends behind the box) and L4: the old turn failed 46 of their 70 runs (10 seeds). H21: 710 runs at 10 ms, 1 failed (C36 seed 4, as before); 710 at 20 ms, 0 failed; shown only: L1 1/10, L2 10/10, L3 9/10, L4 9/10 got round. H22: 490 runs, 0 failed |
| 2026-11-09 | S5-H11..H22 | not committed (branch 37) | Buddy 3 | 1977 PASS, 8 FAIL (all 8 in the shown-only B3-H10 smoothness run) | `logs/2026-11-09_host_tests_9.txt` | the car reversed when it stopped close to the box; reverse only when no way round is open. Too close for a ramp, the car now turns out on the spot (45, 60, 75 or 90 deg, the first that clears the box), drives straight out to the pass offset and turns back, instead of backing up for room. No backing off before the sweep when close, and none to see the other edge while one side is open. It backs up only with both sides blocked (then sweeps again, then waits with fault bit 0x40 as before), an edge outside the sweep and no side open, or no room even to turn on the spot. New C68..C70 (box put down 150-200 mm in front, 1 mm of reversing allowed). C8 (100 mm in front) no longer backs up; C9 (50 mm) still backs off 160 mm because the box's edges are outside the sweep. H21: 740 runs at 10 ms, 0 failed (C36 seed 4 now passes); 740 at 20 ms, 0 failed; shown only: L1 2/10, L2 10/10, L3 10/10, L4 6/10 got round (C68..C70 shift the L cases' random numbers; at 30 seeds L4 is 24/30 and L1 2/30 with both the old and the new code). H22: 490 runs, 0 failed. VC-H01: 222 runs, 0 failed |
| 2026-11-09 | S5-H11..H22, B3-H10..H12 | not committed (branch 37) | Buddy 3 | 2025 PASS, 16 FAIL (15 in the shown-only B3-H10 smoothness run, 1 in the shown-only 1/1.6 turning run) | `logs/2026-11-09_host_tests_10.txt` | line following on curves still really bad, also when starting on the curve. The car's `[VC] curve:` log: at each bend entry the inner wheel ran 9-20 % faster than asked for about 100 mm, and the worst windows had both IRs black, which Buddy 3 treated as a crossing (curve forgotten, car straightened). Now both IRs black while a curve of R670 or tighter is learnt keep the curve and after 30 mm nudge inward (`b3_line.c`, B3_STEER_BB_*); Buddy 2's MOTION_SPEED_KP 0.5 -> 1.6 (`motion_config.h`, for Buddy 2's owner to confirm; `motion_mock.c` MM_KP); bends at 35 mm/s with an inner wheel floor of 19 (was 30 / 15; with KP 0.5 go back to those). New F31..F36 (standing starts on a bend), B3-H11 (Buddy 2's PI loop on a motor fitted to the car's curve log): 0 lost; B3-H12 (both black for 60 mm in a bend, shown): 0 lost with ideal wheels, 7 with the PI loop. Wheel-model bench (not in make): lines lost 30 -> 2 over 10 wheel models and 43 -> 2 over 8 car-fitted ones, about 9 % faster. H21: 740 runs at 10 ms, 0 failed; 740 at 20 ms, 0 failed; shown only L1 2/10, L2 10/10, L3 10/10, L4 6/10 (as before). H22: 490 runs, 0 failed. VC-H01: all passed. Smoothness (shown only): 15 FAIL, 6 of them the new standing starts; the old code gives 14 on the same scenarios. Buddy 3 host tests (`testing/subsystem3/logs/2026-11-09_buddy3_host_tests.txt`): 5413 + 4137 checks passed, new checks for both black in a bend and for the barcode change: a 29/29 decode with both '*' read needs no trailing white (the car rejected such decodes with reason 8 when IR_B met black 4 mm after the last bar); with one '*' spoiled 4 narrow of white is still needed. Firmware MQTT and MQTT TEST builds: 0 warnings |
| 2026-11-14 | S5-H11..H23, B3-H10..H12 | not committed (branch 39) | Buddy 3 | S5-H12/H13 PASS, H21 222 runs 0 failed, line search 245 passed, junction turns passed (the shown-only B3-H10 smoothness rows unchanged) | `logs/2026-11-14_profile_time.txt` | the obstacle sweep starts at detection while the car brakes, 10 deg coarse steps, servo ring-down 15 ms: detection to profile median 1.74 s (was 2.8 s). Firmware MQTT, MQTT TEST and non-MQTT builds: 0 warnings |
