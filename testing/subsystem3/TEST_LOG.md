# Subsystem 3 — test log

**Subsystem:** Consumer (example) + Buddy 3 line following and barcodes  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| B3-U01 | IR pin map: IR_R GP26, IR_L GP27, IR_B GP28 (ADC0–2) | pin map | auto | `subsystem3_test.c` | `[TEST] B3-U01 PASS` |
| B3-U02 | Code 39 table: 43 symbols + `*`, each 3 of 9 elements wide | FR14 | auto | `subsystem3_test.c` | `[TEST] B3-U02 PASS` |
| B3-U03 | `*X*` decodes to X for all 43 symbols, forward and reversed | FR14 | auto | `subsystem3_test.c` | `[TEST] B3-U03 PASS` |
| B3-U04 | frame without `*` start/stop, or `*` as data, is rejected | FR14, FR15 | auto | `subsystem3_test.c` | `[TEST] B3-U04 PASS` |
| B3-U05 | wide:narrow ratio check (reject 1.2:1 and 6:1, accept 2.5:1) | FR15, NFR19 | auto | `subsystem3_test.c` | `[TEST] B3-U05 PASS` |
| B3-U06 | timed edge stream `*A*` at 100 mm/s decodes to A | FR14, NFR7 | auto | `subsystem3_test.c` | `[TEST] B3-U06 PASS` |
| B3-U07 | frame with a short leading quiet zone is rejected | FR15 | auto | `subsystem3_test.c` | `[TEST] B3-U07 PASS` |
| B3-U08 | IR_L and IR_R black: centred, equal wheel speeds | FR11 | auto | `subsystem3_test.c` | `[TEST] B3-U08 PASS` |
| B3-U09 | line under IR_L: steer left, no wheel reverses | FR11 | auto | `subsystem3_test.c` | `[TEST] B3-U09 PASS` |
| B3-U10 | line under IR_R: steer right, no wheel reverses | FR11 | auto | `subsystem3_test.c` | `[TEST] B3-U10 PASS` |
| B3-U11 | line lost: slow search, then stop after the grace time | FR13 | auto | `subsystem3_test.c` | `[TEST] B3-U11 PASS` |
| B3-I01 | Buddy 3 task keeps publishing snapshots | NFR7 | auto | `subsystem3_test.c` | `[TEST] B3-I01 PASS` |
| B3-I02 | no controller context: stop requested, zero speeds | safety | auto | `subsystem3_test.c` | `[TEST] B3-I02 PASS` |
| B3-I03 | follow disabled: line error never reaches the wheels (mode gate) | FR26 | auto | `subsystem3_test.c` | `[TEST] B3-I03 PASS` |
| B3-I04 | follow enabled: control active | FR11 | auto | `subsystem3_test.c` | `[TEST] B3-I04 PASS` |
| B3-I05 | stale controller context: stop requested, zero speeds | safety | auto | `subsystem3_test.c` | `[TEST] B3-I05 PASS` |
| B3-I06 | longest IR service gap ≤ 20 ms (printed) | NFR7 | auto | `subsystem3_test.c` | `[TEST] B3-I06 PASS` |
| B3-H00 | kernel ADC driver `adca` opens | FR12 | auto | `subsystem3_test.c` | `[TEST] B3-H00 PASS` |
| B3-H01 | all sensors on white read white; ADC agrees with DO | FR12, NFR20 | manual + auto | place robot on white when prompted | `[TEST] B3-H01 PASS` + printed counts |
| B3-H02 | all sensors on black read black; ADC agrees with DO | FR12, NFR20 | manual + auto | place sensors on black when prompted | `[TEST] B3-H02 PASS` + printed counts |
| B3-H03 | robot centred on the line: line state CENTRED | FR11 | manual + auto | place robot on the line when prompted | `[TEST] B3-H03 PASS` |
| B3-H04 | a real barcode decodes on IR_B | FR14, NFR19 | manual + auto | push the robot over a barcode when prompted | `[TEST] B3-H04 PASS` + symbol |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
