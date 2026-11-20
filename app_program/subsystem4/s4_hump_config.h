#ifndef S4_HUMP_CONFIG_H
#define S4_HUMP_CONFIG_H

/* Axis indexes: X=0, Y=1, Z=2. Confirm against the sensor mounting. */
#define S4_HUMP_LATERAL_AXIS  1U
#define S4_HUMP_VERTICAL_AXIS 2U
#define S4_HUMP_ROLL_SIGN     1

#define S4_HUMP_TRACK_WIDTH_MM      112U
#define S4_HUMP_START_HEIGHT_MM     8U
#define S4_HUMP_START_SAMPLES       4U /* consecutive valid samples */
#define S4_HUMP_LEVEL_HEIGHT_MM     4U
#define S4_HUMP_MIN_PEAK_HEIGHT_MM  8U
#define S4_HUMP_LEVEL_SAMPLES       1U
#define S4_HUMP_CAL_SAMPLES         100U
#define S4_HUMP_ACCEL_TOLERANCE_PCT 5U
#define S4_HUMP_FILTER_SHIFT        3U

/* ---- Terrain analysis report ---- */
#define S4_TERRAIN_REPORT_INTERVAL_MS  5000U
#define S4_TERRAIN_PROFILE_SPACING_MM  50U

/* ---- Tilt E-stop ----
 * E-stop while one side of the car is lifted more than this (side lift,
 * the same measure as the hump height) for S4_TILT_ESTOP_SAMPLES samples
 * in a row. The course hump is about 10 mm high under one wheel and must
 * not stop the car; while driving, bumps and steering read short peaks
 * of 20-50 mm, which the 200 ms hold filters out. */
#ifndef S4_TILT_ESTOP_MM
#define S4_TILT_ESTOP_MM             30U
#endif
#define S4_TILT_ESTOP_SAMPLES        20U /* 200 ms at 10 ms */

/* ---- Pick-up E-stop ----
 * Lifting the car by hand shows in the accelerometer as a jolt (the
 * size of the acceleration away from the 1 g measured at start-up) and
 * usually as a tilt forwards, backwards or sideways. A car held level
 * and still in the air reads 1 g again, so VehicleController's check of
 * the three IRs (no floor under them) stays as well. */
/* jolt: more than this percent away from 1 g ... */
#ifndef S4_LIFT_JOLT_PCT
#define S4_LIFT_JOLT_PCT             25U
#endif
/* ... for this many samples in a row (50 ms at 10 ms) */
#ifndef S4_LIFT_JOLT_SAMPLES
#define S4_LIFT_JOLT_SAMPLES         5U
#endif
/* tilt in any direction: cos of the angle to the start-up gravity below
 * this percent (91 = about 24 deg) ... */
#ifndef S4_LIFT_TILT_COS_PCT
#define S4_LIFT_TILT_COS_PCT         91U
#endif
/* ... for this many samples in a row (200 ms at 10 ms) */
#ifndef S4_LIFT_TILT_SAMPLES
#define S4_LIFT_TILT_SAMPLES         20U
#endif

#endif