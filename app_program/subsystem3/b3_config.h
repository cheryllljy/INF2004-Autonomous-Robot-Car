/* Buddy 3 configuration. GPIO numbers, NOT Pico physical pin numbers. */
#ifndef B3_CONFIG_H
#define B3_CONFIG_H

/* IR sensor pin map (Robo Pico Grove ports, ADC-capable pins).
 *   IR_R = GP26 (ADC0)  right line sensor
 *   IR_L = GP27 (ADC1)  left line sensor
 *   IR_B = GP28 (ADC2)  barcode sensor and junction check
 * IR_L/IR_R: the module's DO output is read digitally; its threshold is set
 * by the module's potentiometer. IR_B: see B3_BARCODE_ANALOG below. The test
 * build also reads the pins on the ADC as a self-check (subsystem3_test.c). */
#define B3_IR_R_GPIO                  26U
#define B3_IR_L_GPIO                  27U
#define B3_IR_B_GPIO                  28U

/* ADC channels of the same pins (GPn = ADC(n - 26)), used by the tests. */
#define B3_IR_R_ADC_CH                (B3_IR_R_GPIO - 26U)
#define B3_IR_L_ADC_CH                (B3_IR_L_GPIO - 26U)
#define B3_IR_B_ADC_CH                (B3_IR_B_GPIO - 26U)

/* 1 = Buddy 3 installs the RP2040 IO_IRQ_BANK0 vector itself.
 * Use this for the standalone bench test. Set to 0 in the integrated robot
 * when the team already owns a shared GPIO interrupt dispatcher. */
#define B3_INSTALL_IO_BANK0_HANDLER   0U

/* Electrical level that means "black". Check with test B3-H01. */
#define B3_IR_R_BLACK_LEVEL           1U // set for phototransistor + pull-up: black = high
#define B3_IR_L_BLACK_LEVEL           1U // set for phototransistor + pull-up: black = high
#define B3_IR_B_BLACK_LEVEL           1U /* phototransistor + pull-up: black = high */
#define B3_INPUT_PULL_UP              1U

/* IR_B barcode input.
 * 0 = wire the module's DO to GP28. Edges come from the GPIO interrupt and
 *     the module's potentiometer sets the black/white threshold.
 * 1 = wire the module's AO to GP28 (ADC2). A TIMER alarm samples it every
 *     B3_ANALOG_SAMPLE_US and the threshold follows the white and black
 *     levels actually measured, so thin bars that only partly fill the
 *     sensor spot still give edges and the pot no longer matters.
 *     B3_IR_B_BLACK_LEVEL 1 means AO reads higher over black. */
#ifndef B3_BARCODE_ANALOG
#define B3_BARCODE_ANALOG             0U  /* 1 = AO mode; off: module AO swung only ~0.1 V (2026-10-27) */
#endif
#define B3_ANALOG_SAMPLE_US           1000U /* 1 kHz: a 3 mm bar at 100 mm/s spans 30 samples */
#define B3_ANALOG_THRESHOLD_PCT       50U   /* edge point between white (0) and black (100) */
#define B3_ANALOG_HYST_PCT            10U   /* must pass threshold +/- this to switch colour */
#define B3_ANALOG_CONFIRM_SAMPLES     2U    /* samples past the hysteresis before an edge counts */
#define B3_ANALOG_MIN_CONTRAST        300U  /* ADC counts black - white before any edge counts */
#define B3_ANALOG_ATTACK_SHIFT        1U    /* levels move 1/2 of the way to a new extreme per sample */
#define B3_ANALOG_DECAY_SHIFT         12U   /* and relax back with a 4096-sample (~4 s) time constant */
#define B3_ANALOG_TIMER_ALARM         3U    /* RP2040 TIMER alarm used (IRQ number = alarm number) */
#define B3_ANALOG_IRQ_PRIORITY        1

#define B3_TASK_PRIORITY              17
#define B3_TASK_STACK_BYTES           4096U
#define B3_SERVICE_DELAY_MS           10U
#define B3_FEEDBACK_MAX_AGE_US        100000U
#define B3_OUTPUT_MAX_AGE_US          50000U
#define B3_EDGE_QUEUE_SIZE            128U
#define B3_EVENT_QUEUE_SIZE           8U

/* Line sensors straddle the line: IR_L and IR_R sit either side of it and
 * read WHITE while the car is on track (spacing must exceed the 18 mm line).
 * A sensor reading BLACK means the car drifted so the line is now under that
 * sensor, and the car steers back towards it (left black: turn left). */
#define B3_LINE_DEBOUNCE_US           2000U
#define B3_LINE_LOST_GRACE_US         50000U /* unused since the straddle layout */
#define B3_BASE_SPEED_MM_S            120
#define B3_MAX_REQUEST_MM_S           400
#define B3_CORRECTION_MM_S            40
#define B3_RECOVERY_SPEED_MM_S        50
#define B3_SCAN_SPEED_MM_S            100
#define B3_TERRAIN_SPEED_MM_S         70
#define B3_MAX_YAW_MDEG_S             30000

/* Barcode limits. Decoding is ratio-based, so any printed size works: the
 * narrow bar is measured from the barcode itself and every check below that
 * matters is relative to it (wide:narrow ratio, gaps, quiet zones,
 * B3_MAX_ELEMENT_RATIO). The absolute times are only loose safety caps. */
#define B3_BARCODE_RUNS               29U  /* * + gap + char + gap + * */ /* 9 + 1 + 9 + 1 + 9 */
#define B3_MIN_EDGE_US                200U
#define B3_MAX_ELEMENT_US             2000000U /* cap: a 200 mm element at 100 mm/s */
#define B3_MAX_ELEMENT_RATIO          12U      /* time mode: element > 12x the shortest so far aborts.
                                                  Was 6: a 3:1 wide white plus threshold skew
                                                  (narrow black 14 ms, wide white 113 ms) hit it
                                                  on the car, 2026-10-30. */
#define B3_MAX_SCAN_US                30000000U
#define B3_MIN_START_WHITE_US         2000U  /* real check: leading quiet >= 10 narrow */
#define B3_QUIET_NARROW_MULTIPLE      10U
#define B3_MIN_NARROW_ENCODER_TICKS   5U
#define B3_MAX_ENCODER_RUN_TICKS      1000000U
#define B3_MAX_TIMED_SPEED_RATIO_PCT  120U

/*
 * Detection threshold is deliberately below the nominal 18 mm track width
 * to tolerate sensor placement, edge error and speed-measurement error.
 */
#define B3_JUNCTION_MIN_PATCH_UM      14000U 
#define B3_JUNCTION_MIN_SPEED_MM_S    80

/* Barcode -> command map in buddy3_map_symbol() (buddy3.c). Off: from
 * 2026-10-31 Buddy 3 only reports the letter (nav event .barcode.symbol)
 * and the MQTT side maps it to a command, so every symbol maps to
 * B3_NAV_NONE here. 1 = the brief's example A/B/C/D map, for bench tests. */
#define B3_ENABLE_EXAMPLE_NAV_MAP     0U


/* Code 39 allows wide:narrow from 2:1 to 3:1 (school barcode ~3:1).
 * Accept that whole range with tolerance for sensor/robot movement error. */
#define B3_WIDE_NARROW_RATIO_MIN_PCT   180U
#define B3_WIDE_NARROW_RATIO_MAX_PCT   370U


/* Course/barcode dimensions from the supplied reference.
 *
 * These are nominal physical dimensions for validation and documentation.
 * Barcode decoding itself should remain ratio-based because measured time
 * depends on vehicle speed.
 */
#define B3_BARCODE_LENGTH_NOMINAL_UM     141000U   /* 141 mm */
#define B3_BARCODE_NARROW_NOMINAL_UM       3000U   /* ~3 mm */
#define B3_BARCODE_WIDE_NOMINAL_UM         9000U   /* ~9 mm */

#define B3_TRACK_WIDTH_NOMINAL_UM          18000U   /* 18 mm */
#define B3_BARCODE_TRACK_GAP_NOMINAL_UM    20000U   /* 20 mm */
// wait till buddy 2 gives a reliable wheel-distance information, then we can add an actual 141mm distance validation

#if (B3_IR_R_GPIO < 26U) || (B3_IR_R_GPIO > 28U) || \
    (B3_IR_L_GPIO < 26U) || (B3_IR_L_GPIO > 28U) || \
    (B3_IR_B_GPIO < 26U) || (B3_IR_B_GPIO > 28U)
#error "IR sensors must be on the ADC pins GP26..GP28"
#endif

#if B3_BARCODE_ANALOG && \
    ((B3_ANALOG_HYST_PCT >= B3_ANALOG_THRESHOLD_PCT) || \
     ((B3_ANALOG_THRESHOLD_PCT + B3_ANALOG_HYST_PCT) >= 100U) || \
     (B3_ANALOG_CONFIRM_SAMPLES < 1U) || (B3_ANALOG_TIMER_ALARM > 3U))
#error "Check the B3_ANALOG_ settings"
#endif

#if (B3_IR_L_GPIO == B3_IR_R_GPIO) || \
    (B3_IR_L_GPIO == B3_IR_B_GPIO) || \
    (B3_IR_R_GPIO == B3_IR_B_GPIO)
#error "Each IR input requires a distinct GPIO"
#endif
#endif
