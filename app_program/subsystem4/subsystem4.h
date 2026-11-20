/*
 *      subsystem4.h
 *      Subsystem 4 -- IMU sampling and monitor reporting.
 *
 *      Public interface of subsystem 4.  Other subsystems include it as
 *      "../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "imu.h"
#include "s4_hump_config.h"

#define S4_PRI_MONITOR 18 /* priority band 18-19 (TEAM.md) */
#define S4_PRI_IMU     18
#define S4_ALARM_MS    2000 /* report anyway if nothing for 2 s */
/* pause/resume the LED every N reports */
#define S4_LED_TOGGLE_REPORTS 6
#define S4_IMU_SAMPLE_MS      10
#define S4_IMU_MAX_AGE_MS     100

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX 48
#define FLG_BATCH      0x0002 /* set by subsystem 3 */
#define FLG_ALARM      0x0001 /* private: set by our alarm handler */

typedef struct
{
    imu_vector_t accel;
    imu_vector_t mag;
    imu_status_t accel_status;
    imu_status_t mag_status;
    uint32_t timestamp_ms;
    bool has_sample;
} s4_imu_snapshot_t;

extern ID mbf_summary;
extern ID flg_events;
extern ID mtx_imu;

extern ER subsystem4_publish_imu_snapshot(const s4_imu_snapshot_t *p_snapshot);
extern ER subsystem4_get_imu_snapshot(s4_imu_snapshot_t *p_snapshot);
/* true after the hump detector sees S4_HUMP_START_HEIGHT_MM or more for
 * S4_HUMP_START_SAMPLES consecutive valid samples, until level again;
 * the VehicleController keeps the car at VC_HUMP_MIN_MM_S or more */
extern bool subsystem4_on_hump(void);
/* side lift now, in mm, the same measure as the hump height */
extern uint8_t subsystem4_side_lift_mm(void);
/* True during intentional obstacle or barcode stop/reverse motion, when a
 * jolt alone is not evidence of a pick-up (tilt detection remains active). */
extern void subsystem4_hold_jolt(bool b_hold);
/* tilt from level (start-up) in any direction, 0.1 deg: pitch on a ramp
 * as well as roll */
extern uint16_t subsystem4_tilt_ddeg(void);

/* Private to subsystem 4 */
extern ID alm_monitor;
extern UW s4_lines_received; /* for the tests */
extern void monitor_task(INT stacd, void *exinf);
extern void imu_task(INT stacd, void *exinf);
extern void monitor_alarm_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
