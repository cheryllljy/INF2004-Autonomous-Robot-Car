/* Buddy 3 configuration. GPIO numbers, NOT Pico physical pin numbers. */
#ifndef B3_CONFIG_H
#define B3_CONFIG_H

/* IR sensor pin map (Robo Pico Grove ports, ADC-capable pins).
 *   IR_R = GP26 (ADC0)  right line sensor
 *   IR_L = GP27 (ADC1)  left line sensor
 *   IR_B = GP28 (ADC2)  barcode sensor and junction check
 * All three read the module's DO (threshold set by its potentiometer); IR_B
 * as an edge interrupt. The test build also reads them on the ADC. */
#define B3_IR_R_GPIO 26U
#define B3_IR_L_GPIO 27U
#define B3_IR_B_GPIO 28U

/* ADC channels of the same pins (GPn = ADC(n - 26)), used by the tests. */
#define B3_IR_R_ADC_CH (B3_IR_R_GPIO - 26U)
#define B3_IR_L_ADC_CH (B3_IR_L_GPIO - 26U)
#define B3_IR_B_ADC_CH (B3_IR_B_GPIO - 26U)

/* 1 = Buddy 3 installs the RP2040 IO_IRQ_BANK0 vector itself (standalone
 * bench test). 0 when the team owns a shared GPIO interrupt dispatcher. */
#define B3_INSTALL_IO_BANK0_HANDLER 0U

/* Electrical level that means "black" (phototransistor + pull-up: black =
 * high). Check with test B3-H01. */
#define B3_IR_R_BLACK_LEVEL 1U
#define B3_IR_L_BLACK_LEVEL 1U
#define B3_IR_B_BLACK_LEVEL 1U
#define B3_INPUT_PULL_UP    1U

#define B3_TASK_PRIORITY       17
#define B3_TASK_STACK_BYTES    4096U
#define B3_SERVICE_DELAY_MS    10U
#define B3_FEEDBACK_MAX_AGE_US 100000U
#define B3_OUTPUT_MAX_AGE_US   50000U
#define B3_EDGE_QUEUE_SIZE     128U
#define B3_EVENT_QUEUE_SIZE    8U

/* Line sensors straddle the 18 mm line and read WHITE while on track. A
 * sensor reading BLACK means the car drifted; it steers back towards that
 * side (left black: turn left). */
#define B3_LINE_DEBOUNCE_US 2000U
/* unused since the straddle layout */
#define B3_LINE_LOST_GRACE_US 50000U
/* Line lost: neither IR_L nor IR_R black for this much wheel travel. The
 * VehicleController then stops with fault 0x20. Both white is also normal
 * on track, so keep it above the longest no-touch drive (printed at the end
 * of every run). */
#ifndef B3_LINE_LOST_MM
#define B3_LINE_LOST_MM 2000
#endif
/* the VC reads each barcode at B3_SCAN_SPEED_MM_S (vc_barcode.c) */
#ifndef B3_BASE_SPEED_MM_S /* host tests build other speeds */
#define B3_BASE_SPEED_MM_S 100
#endif
#define B3_MAX_REQUEST_MM_S 400
/* Smooth line following. Steering is a curvature (1e6 / radius in mm, so
 * the same value is the same path at any speed):
 *   - a side going black nudges towards it by up to B3_STEER_KICK_CURV;
 *   - while it stays black a curve memory grows by B3_STEER_LEARN_CURV per
 *     10 mm, so the car settles on the bend's own curve;
 *   - with both sides white the memory fades over B3_STEER_FORGET_MM;
 *   - speed drops towards B3_CURVE_SPEED_MM_S as the curve tightens to
 *     B3_CURVE_FULL_CURV;
 *   - the inner wheel never goes below B3_MIN_WHEEL_MM_S.
 * No track shape is assumed. */
#ifndef B3_STEER_KICK_CURV      /* host simulations try other values */
#define B3_STEER_KICK_CURV 4400 /* 227 mm radius */
#endif
/* The nudge tunes itself to the sensor spacing: a touch on the other side
 * within B3_STEER_OVERSHOOT_MM means it overshot, so it shrinks to
 * B3_STEER_SHRINK_PCT (not below B3_STEER_KICK_MIN_CURV); the same side
 * again means too weak, so it grows by B3_STEER_GROW_PCT (not above
 * B3_STEER_KICK_CURV). This stops left-right weaving. */
#ifndef B3_STEER_KICK_MIN_CURV
#define B3_STEER_KICK_MIN_CURV 1500
#endif
#ifndef B3_STEER_OVERSHOOT_MM
#define B3_STEER_OVERSHOOT_MM 100
#endif
#ifndef B3_STEER_SHRINK_PCT
#define B3_STEER_SHRINK_PCT 92
#endif
#ifndef B3_STEER_GROW_PCT
#define B3_STEER_GROW_PCT 110
#endif
/* a side that stays black grows the nudge back to B3_STEER_KICK_CURV over
 * this much driving, so a bend is still caught quickly */
#ifndef B3_STEER_KICK_RAMP_MM
#define B3_STEER_KICK_RAMP_MM 20
#endif
/* The wheels turn the car less than asked (caster drag, slip), so a
 * slower learn rate let the line slide out past the outer IR. */
#ifndef B3_STEER_LEARN_CURV
#define B3_STEER_LEARN_CURV 300 /* per 10 mm driven with a side black */
#endif
/* Already in a bend tighter than B3_STEER_TIGHT_FROM_CURV (R250) and the
 * inner side black again: learn at B3_STEER_LEARN_TIGHT_CURV instead, so
 * tight semicircles are not lost. Using it everywhere lost curved branches
 * after a junction turn. */
#ifndef B3_STEER_TIGHT_FROM_CURV
#define B3_STEER_TIGHT_FROM_CURV 4000
#endif
#ifndef B3_STEER_LEARN_TIGHT_CURV
#define B3_STEER_LEARN_TIGHT_CURV 450
#endif
/* IR_L black while a left curve of at least B3_CURVE_FULL_CURV is learnt:
 * that curve is learnt this % as fast and the nudge is this % of the usual,
 * because the car turns less in left bends than in right ones. 100: off. */
#ifndef B3_STEER_LEFT_PCT
#define B3_STEER_LEFT_PCT 150
#endif
#ifndef B3_STEER_FORGET_MM
#define B3_STEER_FORGET_MM 80
#endif
/* a side against the learnt curve black for this long ends the curve; the
 * steering it left is kept. Shorter values let brief inside touches drop a
 * bend's curve. */
#ifndef B3_STEER_CURVE_END_MM
#define B3_STEER_CURVE_END_MM 40
#endif
#ifndef B3_CURVE_FULL_CURV
#define B3_CURVE_FULL_CURV 1500 /* 670 mm radius */
#endif
/* Bend speed. Needs Buddy 2's speed KP 1.6 (MOTION_SPEED_KP,
 * motion_config.h); with a lower KP use 30 here and 15 for
 * B3_MIN_WHEEL_MM_S. */
#ifndef B3_CURVE_SPEED_MM_S
#define B3_CURVE_SPEED_MM_S 40
#endif
/* the steering moves towards what the sensors ask by at most this much
 * per mm driven, so a sensor touching the line is a nudge, not a jerk */
#ifndef B3_STEER_SLEW_CURV
#define B3_STEER_SLEW_CURV 400
#endif
/* Hairpin: a side still black after this much driving with the steering
 * already as tight as B3_MIN_WHEEL_MM_S allows. The inner wheel then
 * stops (a pivot on it) until the side is white. */
#ifndef B3_STEER_ESCAPE_MM
#define B3_STEER_ESCAPE_MM 10
#endif
/* Inner wheel floor; not lower: Buddy 2 only cuts a wheel's power, never
 * brakes it, so slower inner wheels were dragged along unevenly and the
 * car turned wider than asked, then pivoted. Tightest smooth curve ~R112. */
#ifndef B3_MIN_WHEEL_MM_S
#define B3_MIN_WHEEL_MM_S 20 /* half of B3_CURVE_SPEED_MM_S */
#endif
/* Both IRs black in a bend: the tape crosses the IR pair at an angle, so
 * both can read black for 50+ mm without a crossing. A crossing tape gives
 * at most ~25 mm both black, so while a curve of B3_STEER_BB_CURV or tighter
 * is learnt, both black keeps the learnt curve for B3_STEER_BB_CROSS_MM and,
 * if longer, also nudges to the inside. Below that curve both black is
 * still a crossing. */
#ifndef B3_STEER_BB_CURV
#define B3_STEER_BB_CURV B3_CURVE_FULL_CURV /* fully slowed for it */
#endif
#ifndef B3_STEER_BB_CROSS_MM
#define B3_STEER_BB_CROSS_MM 30
#endif
/* wheel centre to wheel centre, as Buddy 2's TRACK_WIDTH_MM */
#define B3_WHEEL_TRACK_MM 112
/* a longer gap between two steps counts as this long */
#define B3_STEER_MAX_DT_US     50000U
#define B3_RECOVERY_SPEED_MM_S 60
/* Slow down over a barcode so IR_B catches thin bars. Any black on IR_B
 * starts the slow speed; it ends after IR_B reads white for
 * B3_SCAN_SLOW_WHITE_US (longer than any gap inside a barcode), so the
 * speed is constant across the barcode. Safety: it lasts at most
 * B3_SCAN_SLOW_MAX_US, and a crossing (both line sensors black) ends it so
 * a junction is measured at base speed. */
#define B3_SCAN_SPEED_MM_S    30
#define B3_SCAN_SLOW_WHITE_US 800000U /* 30 mm of white at 50 mm/s */
#define B3_SCAN_SLOW_MAX_US   6000000U
/* Retry a failed decode with the first/last bar trimmed to the other narrow
 * bar of the same '*' (seen 3-4x too long on the car). */
#define B3_TRIM_OUTER_BARS       1
/* rescale each bar width by the measured wheel speed */
#define B3_SCAN_SPEED_COMPENSATE 0
#define B3_TERRAIN_SPEED_MM_S    70
#define B3_MAX_YAW_MDEG_S        30000

/* Barcode limits. Decoding is ratio-based, so any printed size works: the
 * narrow bar is measured from the barcode itself and the checks that matter
 * are relative to it. The absolute times are only loose safety caps. */
/* * + gap + char + gap + * */ /* 9 + 1 + 9 + 1 + 9 */
#define B3_BARCODE_RUNS 29U
#define B3_MIN_EDGE_US  200U
/* a dark speck in a gap: a bar shorter than this % of the narrowest bar so
 * far is joined to the gap around it, like a blip under B3_MIN_EDGE_US.
 * Real narrow bars read down to about 55 % of each other. 0: off. */
#ifndef B3_SPECK_PCT
#define B3_SPECK_PCT 20U
#endif
/* cap: a 200 mm element at 100 mm/s */
#define B3_MAX_ELEMENT_US 2000000U
/* time mode: element > 12x the shortest so far aborts (a wide white plus
 * threshold skew can reach ~8x) */
#define B3_MAX_ELEMENT_RATIO 12U
#define B3_MAX_SCAN_US       30000000U
/* real check: leading quiet >= 10 narrow */
#define B3_MIN_START_WHITE_US    2000U
#define B3_QUIET_NARROW_MULTIPLE 10U
/* Trailing white needed after a full 29-element decode, only when a '*'
 * was spoiled (B3_NEED_BOTH_STARS 0); with both '*' read none is needed
 * (b3_barcode.c prepare_result()). After all 29 elements only an
 * inter-character gap (~1 narrow) could follow, so 4 x narrow is enough. */
#define B3_TRAILING_NARROW_MULTIPLE  4U
#define B3_MIN_NARROW_ENCODER_TICKS  5U
#define B3_MAX_ENCODER_RUN_TICKS     1000000U
#define B3_MAX_TIMED_SPEED_RATIO_PCT 120U

/* Junction: each line sensor's own run over the tape must be at least this
 * long and the two runs must overlap (junction_update() in buddy3.c). Kept
 * well below the 18 mm tape: the car meets the tape at an angle and the
 * comparator reads black late (real junctions measured 7-11 mm). Junctions
 * are blocked while a barcode scan has more than B3_JUNCTION_MAX_SCAN_RUNS
 * elements. */
#define B3_JUNCTION_MIN_PATCH_UM 6000U
/* Patch length is speed x time; this only rules out a car that is standing
 * or pivoting. The faster wheel is used when one encoder reads 0 (GP3). */
#define B3_JUNCTION_MIN_SPEED_MM_S 15
/* 0: IR_L and IR_R both black is the crossing. IR_B is not level with
 * them, so requiring it black at the same moment misses junctions.
 * 1 = all three black together. */
#define B3_JUNCTION_NEEDS_IR_B 0U
/* IR_B may already have crossed the junction tape and started a barcode
 * scan with one element; more elements than this means a real barcode. */
#define B3_JUNCTION_MAX_SCAN_RUNS (B3_JUNCTION_NEEDS_IR_B ? 0U : 1U)

/* Barcode -> command map in buddy3_map_symbol() (buddy3.c). 0: Buddy 3
 * only reports the letter (nav event .barcode.symbol) and the MQTT side
 * maps it. 1 = the brief's example A/B/C/D map, for bench tests. */
#define B3_ENABLE_EXAMPLE_NAV_MAP 0U

/* Code 39 allows wide:narrow from 2:1 to 3:1. Accept that range with
 * tolerance for sensor/movement error. */
#define B3_WIDE_NARROW_RATIO_MIN_PCT 180U
/* the car's IR_B reads printed 3:1 bars as 4-4.7:1 */
#define B3_WIDE_NARROW_RATIO_MAX_PCT 550U
/* Narrowest vs widest narrow element in one character. Narrow elements
 * vary a lot on the car; this is safe while narrow and wide stay clearly
 * apart (the separate 1.5x gap check). */
#define B3_NARROW_SPREAD_MAX_PCT 400U
/* 1: both start and stop '*' must decode. 0: the letter plus one '*'. */
#define B3_NEED_BOTH_STARS 0

/* Nominal course/barcode dimensions, for validation and documentation.
 * Decoding stays ratio-based because measured time depends on speed. */
#define B3_BARCODE_LENGTH_NOMINAL_UM 141000U /* 141 mm */
#define B3_BARCODE_NARROW_NOMINAL_UM 3000U   /* ~3 mm */
#define B3_BARCODE_WIDE_NOMINAL_UM   9000U   /* ~9 mm */

#define B3_TRACK_WIDTH_NOMINAL_UM       18000U /* 18 mm */
#define B3_BARCODE_TRACK_GAP_NOMINAL_UM 20000U /* 20 mm */
// TODO: validate the 141 mm barcode length once Buddy 2's wheel distance
// is reliable

#if (B3_IR_R_GPIO < 26U) || (B3_IR_R_GPIO > 28U) || (B3_IR_L_GPIO < 26U) || \
    (B3_IR_L_GPIO > 28U) || (B3_IR_B_GPIO < 26U) || (B3_IR_B_GPIO > 28U)
#error "IR sensors must be on the ADC pins GP26..GP28"
#endif

#if (B3_IR_L_GPIO == B3_IR_R_GPIO) || (B3_IR_L_GPIO == B3_IR_B_GPIO) || \
    (B3_IR_R_GPIO == B3_IR_B_GPIO)
#error "Each IR input requires a distinct GPIO"
#endif
#endif
