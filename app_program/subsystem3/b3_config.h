/* Buddy 3 configuration. GPIO numbers, NOT Pico physical pin numbers. */
#ifndef B3_CONFIG_H
#define B3_CONFIG_H

/* TEAM-REPO INTEGRATION: GP13/14 avoid the template's optional ADC driver
 * ownership of GP26/27/28. The user-tested ZIP used GP26/GP27: you must
 * rewire or reconcile the ADC owner before enabling the Buddy3 startup.
 * Keep the tested GP7 barcode input; the GP7 pin must still be claimed in TEAM.md.
 * Debug pins are disabled here so they cannot disturb other subsystems. */
/* Safe standalone bench defaults for the supplied mtk3smp-rp2040 profile.
 * GP26..GP28 were intentionally NOT used here because the reference port's
 * default sample-device driver claims them as ADC inputs.
 * Reconcile these values with the team's final Robo Pico pin map. */
// #define B3_LINE_LEFT_GPIO             13U
// #define B3_LINE_RIGHT_GPIO            14U
// #define B3_BARCODE_GPIO               15U
#define B3_LINE_LEFT_GPIO             13U
#define B3_LINE_RIGHT_GPIO            14U
#define B3_BARCODE_GPIO               7U // NOT USING GP28 BECAUSE CANT CONNECT WITHOUT FEMALE TO MALE JUMPER WIRES
#define B3_DEBUG_GPIO                 6U
#define B3_DEBUG_ENABLED              0U

/* 1 = Buddy 3 installs the RP2040 IO_IRQ_BANK0 vector itself.
 * Use this for the standalone bench test. Set to 0 in the integrated robot
 * when the team already owns a shared GPIO interrupt dispatcher. */
#define B3_INSTALL_IO_BANK0_HANDLER   1U

/* Digital OUT/DO connections. Measure each board's black/white polarity. */
#define B3_LEFT_BLACK_LEVEL           0U
#define B3_RIGHT_BLACK_LEVEL          0U
#define B3_BARCODE_BLACK_LEVEL        1U // behaves like a phototransistor with pull-up resistor, so black = low
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

#if (B3_LINE_LEFT_GPIO > 29U) || \
    (B3_LINE_RIGHT_GPIO > 29U) || \
    (B3_BARCODE_GPIO > 29U) || \
    (B3_DEBUG_GPIO > 29U)
#error "GPIO number is outside the RP2040 GPIO range"
#endif

#if (B3_LINE_LEFT_GPIO == B3_LINE_RIGHT_GPIO) || \
    (B3_LINE_LEFT_GPIO == B3_BARCODE_GPIO) || \
    (B3_LINE_RIGHT_GPIO == B3_BARCODE_GPIO)
#error "Each IR input requires a distinct GPIO"
#endif
#if B3_DEBUG_ENABLED && ((B3_DEBUG_GPIO == B3_LINE_LEFT_GPIO) || \
    (B3_DEBUG_GPIO == B3_LINE_RIGHT_GPIO) || (B3_DEBUG_GPIO == B3_BARCODE_GPIO))
#error "Debug GPIO conflicts with an IR input"
#endif
#endif
