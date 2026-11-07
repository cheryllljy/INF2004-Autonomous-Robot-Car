/* Buddy 3 configuration. GPIO numbers, NOT Pico physical pin numbers. */
#ifndef B3_CONFIG_H
#define B3_CONFIG_H

/* IR sensor pin map (Robo Pico Grove ports, ADC-capable pins).
 *   IR_R = GP26 (ADC0)  right line sensor
 *   IR_L = GP27 (ADC1)  left line sensor
 *   IR_B = GP28 (ADC2)  barcode sensor and junction check
 * IR_L/IR_R: the module's DO output is read digitally; its threshold is set
 * by the module's potentiometer. IR_B: DO too, as an edge interrupt. The test
 * build also reads the pins on the ADC as a self-check (subsystem3_test.c). */
#define B3_IR_R_GPIO 26U
#define B3_IR_L_GPIO 27U
#define B3_IR_B_GPIO 28U

/* ADC channels of the same pins (GPn = ADC(n - 26)), used by the tests. */
#define B3_IR_R_ADC_CH (B3_IR_R_GPIO - 26U)
#define B3_IR_L_ADC_CH (B3_IR_L_GPIO - 26U)
#define B3_IR_B_ADC_CH (B3_IR_B_GPIO - 26U)

/* 1 = Buddy 3 installs the RP2040 IO_IRQ_BANK0 vector itself.
 * Use this for the standalone bench test. Set to 0 in the integrated robot
 * when the team already owns a shared GPIO interrupt dispatcher. */
#define B3_INSTALL_IO_BANK0_HANDLER 0U

/* Electrical level that means "black". Check with test B3-H01. */
// set for phototransistor + pull-up: black = high
#define B3_IR_R_BLACK_LEVEL 1U
// set for phototransistor + pull-up: black = high
#define B3_IR_L_BLACK_LEVEL 1U
/* phototransistor + pull-up: black = high */
#define B3_IR_B_BLACK_LEVEL 1U
#define B3_INPUT_PULL_UP    1U

/* IR_B barcode input: the module's DO goes to GP28. Edges come from the
 * GPIO interrupt and the module's potentiometer sets the black/white
 * threshold. (The AO/ADC mode was removed on 2026-11-04: nyx is staying on
 * DO.) */

#define B3_TASK_PRIORITY       17
#define B3_TASK_STACK_BYTES    4096U
#define B3_SERVICE_DELAY_MS    10U
#define B3_FEEDBACK_MAX_AGE_US 100000U
#define B3_OUTPUT_MAX_AGE_US   50000U
#define B3_EDGE_QUEUE_SIZE     128U
#define B3_EVENT_QUEUE_SIZE    8U

/* Line sensors straddle the line: IR_L and IR_R sit either side of it and
 * read WHITE while the car is on track (spacing must exceed the 18 mm line).
 * A sensor reading BLACK means the car drifted so the line is now under that
 * sensor, and the car steers back towards it (left black: turn left). */
#define B3_LINE_DEBOUNCE_US 2000U
/* unused since the straddle layout */
#define B3_LINE_LOST_GRACE_US 50000U
/* nyx, 2026-11-06: 100 mm/s; the VC backs up and reads each barcode at
 * B3_SCAN_SPEED_MM_S (vehicle_controller/vc_barcode.c) */
#ifndef B3_BASE_SPEED_MM_S /* host tests build other speeds */
#define B3_BASE_SPEED_MM_S 100
#endif
#define B3_MAX_REQUEST_MM_S 400
/* was 40: smaller kick when a sensor just touches the line, less weaving on straights (2026-11-04) */
#define B3_CORRECTION_MM_S 25
/* Sharp turns / U-turns (2026-11-04). If the same side sensor stays black
 * for B3_SHARP_TURN_US, the line is bending faster than the normal
 * correction can follow: speed ramps down to B3_SHARP_SPEED_MM_S and the
 * correction ramps up until the inner wheel stops (pivot), smoothly over
 * that time rather than in one step. The slow speed is held for
 * B3_SHARP_HOLD_US after the side goes white again (a U-turn flickers),
 * and ends at once on a crossing (both black) so junctions still see
 * >= B3_JUNCTION_MIN_SPEED_MM_S. B3_SHARP_TURN_US 0 turns this off. */
/* was 150 ms: slow down earlier in a curve */
#define B3_SHARP_TURN_US    100000U
#define B3_SHARP_SPEED_MM_S 60 /* was 60 */
/* was 300 ms: shorter gentle turn after the curve, less overshoot */
#define B3_SHARP_HOLD_US 150000U
/* After a sharp turn ends, speed climbs back from B3_SHARP_SPEED_MM_S to
 * the base speed over this time instead of jumping (the jump made the car
 * weave on the straight after a turn). A crossing skips it, so junctions
 * still see full speed. */
#define B3_SHARP_RECOVER_US    400000U
#define B3_RECOVERY_SPEED_MM_S 60
/* Slow down over a barcode so IR_B catches thin bars (nyx, 2026-11-04; was
 * 100). Any black seen by IR_B starts the slow speed; it ends only after IR_B
 * has read white for B3_SCAN_SLOW_WHITE_US (longer than the widest gap inside
 * a barcode), so the car stays at one speed across the whole barcode even if
 * the decoder gives up halfway. Bar widths are measured in time, so each width
 * is also rescaled by the measured wheel speed (B3_SCAN_SPEED_COMPENSATE).
 * Safety: the slow speed lasts at most B3_SCAN_SLOW_MAX_US, and a crossing
 * (both line sensors black) ends it so a junction is measured at base speed. */
#define B3_SCAN_SPEED_MM_S    30 /* nyx, 2026-11-06: was 50 */
#define B3_SCAN_SLOW_WHITE_US 800000U /* 30 mm of white at 50 mm/s */
#define B3_SCAN_SLOW_MAX_US   6000000U
/* Retry a failed decode with the first/last bar trimmed to the other narrow
 * bar of the same '*' (seen 3-4x too long on the car, 2026-11-04). */
#define B3_TRIM_OUTER_BARS       1
#define B3_SCAN_SPEED_COMPENSATE 0
#define B3_TERRAIN_SPEED_MM_S    70
#define B3_MAX_YAW_MDEG_S        30000

/* Barcode limits. Decoding is ratio-based, so any printed size works: the
 * narrow bar is measured from the barcode itself and every check below that
 * matters is relative to it (wide:narrow ratio, gaps, quiet zones,
 * B3_MAX_ELEMENT_RATIO). The absolute times are only loose safety caps. */
/* * + gap + char + gap + * */ /* 9 + 1 + 9 + 1 + 9 */
#define B3_BARCODE_RUNS 29U
#define B3_MIN_EDGE_US  200U
/* cap: a 200 mm element at 100 mm/s */
#define B3_MAX_ELEMENT_US 2000000U
/* time mode: element > 12x the shortest so far aborts.
 * Was 6: a 3:1 wide white plus threshold skew
 * (narrow black 14 ms, wide white 113 ms) hit it
 * on the car, 2026-10-30. */
#define B3_MAX_ELEMENT_RATIO 12U
#define B3_MAX_SCAN_US       30000000U
/* real check: leading quiet >= 10 narrow */
#define B3_MIN_START_WHITE_US    2000U
#define B3_QUIET_NARROW_MULTIPLE 10U
/* Trailing white needed after a full 29-element decode (nyx, 2026-11-05).
 * Was the same 10 x narrow as the leading zone, but on the car the next black
 * came back 667 ms (20 mm at 30 mm/s) after a good decode and every scan was
 * rejected with reason 8. Once all 29 elements decoded, the only thing that
 * could follow is an inter-character gap (about 1 narrow), so 4 x narrow
 * already proves the barcode has ended. */
#define B3_TRAILING_NARROW_MULTIPLE  4U
#define B3_MIN_NARROW_ENCODER_TICKS  5U
#define B3_MAX_ENCODER_RUN_TICKS     1000000U
#define B3_MAX_TIMED_SPEED_RATIO_PCT 120U

/*
 * Detection threshold is deliberately below the nominal 18 mm track width
 * to tolerate sensor placement, edge error and speed-measurement error.
 */
/* nyx, 2026-11-06: 14 -> 12 -> 8 mm. Both sensors are black together
 * only where their crossings of the 18 mm tape overlap; at a slight angle
 * one sensor enters the tape a few mm before the other, and the comparator
 * only reads black once most of the spot is on the tape. On the car real
 * junctions measured 9 and 11 mm. A wide bar of a 100 % barcode is about
 * 7.5 mm, and junctions are blocked anyway while a barcode scan has more
 * than B3_JUNCTION_MAX_SCAN_RUNS elements. */
#define B3_JUNCTION_MIN_PATCH_UM 8000U
/* nyx, 2026-11-06: was 80, so at the 25 mm/s base speed no junction was
 * ever reported. The patch length is speed x time, so any steady forward
 * speed works; this only rules out a car that is standing or pivoting.
 * The speed used is the faster wheel when one encoder reads 0 (GP3). */
#define B3_JUNCTION_MIN_SPEED_MM_S 15
/* Straddle layout (2026-10-31): IR_L and IR_R both black is the crossing.
 * Requiring IR_B black at the same moment missed every junction on the car
 * because IR_B is not level with IR_L/IR_R, so it crosses the tape at a
 * different time. 1 = old rule (all three black together). */
#define B3_JUNCTION_NEEDS_IR_B 0U
/* IR_B may already have crossed the junction tape and started a barcode
 * scan with one element; more elements than this means a real barcode. */
#define B3_JUNCTION_MAX_SCAN_RUNS (B3_JUNCTION_NEEDS_IR_B ? 0U : 1U)

/* Barcode -> command map in buddy3_map_symbol() (buddy3.c). Off: from
 * 2026-10-31 Buddy 3 only reports the letter (nav event .barcode.symbol)
 * and the MQTT side maps it to a command, so every symbol maps to
 * B3_NAV_NONE here. 1 = the brief's example A/B/C/D map, for bench tests. */
#define B3_ENABLE_EXAMPLE_NAV_MAP 0U


/* Code 39 allows wide:narrow from 2:1 to 3:1 (school barcode ~3:1).
 * Accept that whole range with tolerance for sensor/robot movement error. */
#define B3_WIDE_NARROW_RATIO_MIN_PCT 180U
/* was 370: the car's IR_B reads printed 3:1 bars as 4-4.7:1 (2026-11-04). */
#define B3_WIDE_NARROW_RATIO_MAX_PCT 550U
/* Narrowest vs widest narrow element in one character (was a fixed 180).
 * On the car narrow bars and gaps varied 48-125 ms in one letter while the
 * wide ones were all 3x longer, so a wider spread is safe as long as the two
 * groups stay clearly apart (the separate 1.5x gap check). */
/* 260 failed a 'Z' with a 33 ms thin bar next to 123 ms ones */
#define B3_NARROW_SPREAD_MAX_PCT 400U
/* 1: both start and stop '*' must decode. 0: the letter plus one '*'. */
#define B3_NEED_BOTH_STARS 0


/* Course/barcode dimensions from the supplied reference.
 *
 * These are nominal physical dimensions for validation and documentation.
 * Barcode decoding itself should remain ratio-based because measured time
 * depends on vehicle speed.
 */
#define B3_BARCODE_LENGTH_NOMINAL_UM 141000U /* 141 mm */
#define B3_BARCODE_NARROW_NOMINAL_UM 3000U   /* ~3 mm */
#define B3_BARCODE_WIDE_NOMINAL_UM   9000U   /* ~9 mm */

#define B3_TRACK_WIDTH_NOMINAL_UM       18000U /* 18 mm */
#define B3_BARCODE_TRACK_GAP_NOMINAL_UM 20000U /* 20 mm */
// wait till buddy 2 gives a reliable wheel-distance information, then we can add an actual 141mm distance validation

#if (B3_IR_R_GPIO < 26U) || (B3_IR_R_GPIO > 28U) || (B3_IR_L_GPIO < 26U) || \
    (B3_IR_L_GPIO > 28U) || (B3_IR_B_GPIO < 26U) || (B3_IR_B_GPIO > 28U)
#error "IR sensors must be on the ADC pins GP26..GP28"
#endif


#if (B3_IR_L_GPIO == B3_IR_R_GPIO) || (B3_IR_L_GPIO == B3_IR_B_GPIO) || \
    (B3_IR_R_GPIO == B3_IR_B_GPIO)
#error "Each IR input requires a distinct GPIO"
#endif
#endif
