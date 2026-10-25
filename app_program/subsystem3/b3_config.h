/* Buddy 3 configuration. GPIO numbers, NOT Pico physical pin numbers. */
#ifndef B3_CONFIG_H
#define B3_CONFIG_H

/* IR sensor pin map (Robo Pico Grove ports, ADC-capable pins).
 *   IR_R = GP26 (ADC0)  right line sensor
 *   IR_L = GP27 (ADC1)  left line sensor
 *   IR_B = GP28 (ADC2)  barcode sensor (edge IRQ) and junction check
 * Each module's DO output is read digitally; its threshold is set by the
 * module's potentiometer. The test build also reads the same pins through
 * the kernel ADC driver as a self-check (subsystem3_test.c). */
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
#define B3_IR_R_BLACK_LEVEL           0U
#define B3_IR_L_BLACK_LEVEL           0U
#define B3_IR_B_BLACK_LEVEL           1U /* phototransistor + pull-up: black = high */
#define B3_INPUT_PULL_UP              1U

#define B3_TASK_PRIORITY              17
#define B3_TASK_STACK_BYTES           4096U
#define B3_SERVICE_DELAY_MS           10U
#define B3_FEEDBACK_MAX_AGE_US        100000U
#define B3_OUTPUT_MAX_AGE_US          50000U
#define B3_EDGE_QUEUE_SIZE            128U
#define B3_EVENT_QUEUE_SIZE           8U

#define B3_LINE_DEBOUNCE_US           2000U
#define B3_LINE_LOST_GRACE_US         50000U
#define B3_BASE_SPEED_MM_S            120
#define B3_MAX_REQUEST_MM_S           400
#define B3_CORRECTION_MM_S            40
#define B3_RECOVERY_SPEED_MM_S        50
#define B3_SCAN_SPEED_MM_S            100
#define B3_TERRAIN_SPEED_MM_S         70
#define B3_MAX_YAW_MDEG_S             30000

/* Bench starting values. Record actual narrow/wide times before tuning. */
#define B3_BARCODE_RUNS               29U  /* * + gap + char + gap + * */ /* 9 + 1 + 9 + 1 + 9 */
#define B3_MIN_EDGE_US                200U
#define B3_MAX_ELEMENT_US             300000U
#define B3_MAX_SCAN_US                5000000U
#define B3_MIN_START_WHITE_US         20000U
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

/* The write-up calls A/B/C/D an EXAMPLE, not a confirmed course mapping. */
#define B3_ENABLE_EXAMPLE_NAV_MAP     0U


/* School barcode is approximately 3:1 wide:narrow.
 * Allow tolerance for sensor/robot movement error. */
#define B3_WIDE_NARROW_RATIO_MIN_PCT   230U
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

#if (B3_IR_L_GPIO == B3_IR_R_GPIO) || \
    (B3_IR_L_GPIO == B3_IR_B_GPIO) || \
    (B3_IR_R_GPIO == B3_IR_B_GPIO)
#error "Each IR input requires a distinct GPIO"
#endif
#endif
