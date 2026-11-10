#ifndef S4_HUMP_CONFIG_H
#define S4_HUMP_CONFIG_H

/* Axis indexes: X=0, Y=1, Z=2. Confirm against the sensor mounting. */
#define S4_HUMP_LATERAL_AXIS  1U
#define S4_HUMP_VERTICAL_AXIS 2U
#define S4_HUMP_ROLL_SIGN     1

#define S4_HUMP_TRACK_WIDTH_MM      112U
#define S4_HUMP_START_HEIGHT_MM     8U
#define S4_HUMP_LEVEL_HEIGHT_MM     4U
#define S4_HUMP_MIN_PEAK_HEIGHT_MM  8U
#define S4_HUMP_LEVEL_SAMPLES       1U
#define S4_HUMP_CAL_SAMPLES         100U
#define S4_HUMP_ACCEL_TOLERANCE_PCT 5U
#define S4_HUMP_FILTER_SHIFT        3U

/* ---- Terrain analysis report ---- */
#define S4_TERRAIN_REPORT_INTERVAL_MS  5000U
#define S4_TERRAIN_PROFILE_SPACING_MM  50U

#endif