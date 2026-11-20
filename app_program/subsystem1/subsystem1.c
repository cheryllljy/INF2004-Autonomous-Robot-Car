
/*
 * subsystem1.c
 *
 * INF2004 Autonomous Robot Car - Buddy 1
 *
 * Responsibilities:
 * - Initialize shared barcode mapping.
 * - Collect actual sensor data from Buddy 2, 3 and 4.
 * - Maintain a race-safe cached sensor snapshot.
 * - Supply non-blocking telemetry to the MQTT network task.
 *
 * Networking stays inside cyw43_utk_task.
 * All car movement remains controlled by VehicleController.
 */

#include "subsystem1.h"
#include "buddy1_mapping.h"

#include "../vehicle_controller/vehicle_controller.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/buddy3.h"
#include "../subsystem3/b3_port.h"
#include "../subsystem4/subsystem4.h"

#include <stdint.h>
#include <stdbool.h>

#define B1_EXPORT_SENSOR_SIZE 42u
#define B1_EXPORT_LINE_MAX_AGE_US 100000u

#define B1_SENSOR_SAMPLE_MS 50u
#define B1_SENSOR_CACHE_MAX_AGE_US 1000000u
#define B1_SENSOR_TASK_PRI 24

/*
 * Hump observation filters.
 *
 * These do not modify Buddy 4's detector.
 * They only determine which observations
 * Buddy 1 counts in its telemetry.
 */
#define B1_HUMP_MIN_SPEED_MM_S 20
#define B1_HUMP_CONFIRM_SAMPLES 2u
#define B1_HUMP_CLEAR_SAMPLES 2u
#define B1_HUMP_COOLDOWN_US 1500000u

static uint8_t b1_hump_events = 0u;
static uint8_t b1_hump_high_samples = 0u;
static uint8_t b1_hump_clear_samples = 0u;

static bool b1_hump_armed = true;
static bool b1_hump_confirmed = false;
static bool b1_hump_has_event = false;

static uint32_t b1_hump_last_event_us = 0u;

/*
 * Controller transition monitoring.
 */
static bool b1_vc_state_seen = false;
static vc_state_t b1_previous_vc_state = VC_STATE_IDLE;

/*
 * Sensor cache shared between:
 *
 * Producer: Buddy 1 sensor sampling task.
 * Consumer: MQTT network task.
 *
 * The MQTT side uses TMO_POL so it never
 * waits for the cache mutex.
 *
 * No calls to teammate sensor APIs occur
 * while this cache mutex is held.
 */
static ID b1_sensor_cache_mutex = 0;

static uint8_t b1_sensor_cache[B1_EXPORT_SENSOR_SIZE];

static uint32_t b1_sensor_cache_time_us = 0u;
static bool b1_sensor_cache_valid = false;

/*
 * Reset Buddy 1's observed hump count when
 * VehicleController returns to a fault-free IDLE.
 */
static void b1_reset_observed_humps_if_controller_reset(void)
{
    uint32_t status = vc_get_status_word();
    vc_state_t state = VC_SW_STATE(status);
    uint8_t faults = VC_SW_FAULTS(status);

    if (b1_vc_state_seen &&
        b1_previous_vc_state != VC_STATE_IDLE &&
        state == VC_STATE_IDLE &&
        faults == 0u)
    {
        b1_hump_events = 0u;
        b1_hump_high_samples = 0u;
        b1_hump_clear_samples = 0u;

        b1_hump_armed = true;
        b1_hump_confirmed = false;
        b1_hump_has_event = false;

        b1_hump_last_event_us = 0u;
    }

    b1_previous_vc_state = state;
    b1_vc_state_seen = true;
}

/*
 * Identify movement using actual wheel speed
 * and VehicleController state.
 */
static bool b1_motion_is_driving(
    const motion_status_t *p_motion)
{
    int32_t left;
    int32_t right;
    uint32_t status;

    if (NULL == p_motion)
    {
        return false;
    }

    left = (int32_t)p_motion->speed_l_mmps;
    right = (int32_t)p_motion->speed_r_mmps;

    status = vc_get_status_word();

    if (VC_SW_STATE(status) != VC_STATE_LINE_FOLLOW)
    {
        return false;
    }

    if (left < 0)
    {
        left = -left;
    }

    if (right < 0)
    {
        right = -right;
    }

    return (
        left >= B1_HUMP_MIN_SPEED_MM_S ||
        right >= B1_HUMP_MIN_SPEED_MM_S
    );
}

/*
 * Confirm observed hump events.
 *
 * A sustained rise counts once.
 * After clearing, another rise can be counted
 * only after the minimum event interval.
 *
 * Note: Buddy 4 remains the source of the
 * actual hump detection.
 */
static void b1_update_hump(
    bool raw_hump,
    bool driving,
    uint32_t now_us)
{
    if (!raw_hump)
    {
        b1_hump_high_samples = 0u;

        if (b1_hump_clear_samples <
            B1_HUMP_CLEAR_SAMPLES)
        {
            b1_hump_clear_samples++;
        }

        if (b1_hump_clear_samples >=
            B1_HUMP_CLEAR_SAMPLES)
        {
            b1_hump_confirmed = false;
            b1_hump_armed = true;
        }

        return;
    }

    b1_hump_clear_samples = 0u;

    if (!driving)
    {
        b1_hump_high_samples = 0u;
        b1_hump_armed = false;
        b1_hump_confirmed = false;
        return;
    }

    if (!b1_hump_armed)
    {
        return;
    }

    if (b1_hump_high_samples <
        B1_HUMP_CONFIRM_SAMPLES)
    {
        b1_hump_high_samples++;
    }

    if (b1_hump_high_samples <
        B1_HUMP_CONFIRM_SAMPLES)
    {
        return;
    }

    b1_hump_armed = false;

    if (!b1_hump_has_event ||
        (uint32_t)(
            now_us - b1_hump_last_event_us
        ) >= B1_HUMP_COOLDOWN_US)
    {
        b1_hump_events++;
        b1_hump_confirmed = true;
        b1_hump_has_event = true;

        b1_hump_last_event_us = now_us;
    }
}

/*
 * Pack little-endian sensor values.
 */
static void b1_export_u16(
    uint8_t *p,
    uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)(value >> 8);
}

static void b1_export_u32(
    uint8_t *p,
    uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

/*
 * Existing controller interface.
 *
 * This is a quick read of the packed
 * VehicleController status word.
 */
int32_t b1_export_controller_status(
    uint32_t *output)
{
    if (NULL == output)
    {
        return -1;
    }

#if VC_ENABLE
    *output = vc_get_status_word();
    return 0;
#else
    return -1;
#endif
}

/*
 * Build the 42-byte sensor payload from
 * actual teammate APIs.
 *
 * Called only by the Buddy 1 sensor task.
 *
 * Do not call this function inside an
 * lwIP MQTT timer or callback.
 */
static int32_t b1_build_sensor_payload(
    uint8_t *output,
    uint32_t capacity)
{
    uint32_t i;

    if (NULL == output ||
        capacity < B1_EXPORT_SENSOR_SIZE)
    {
        return -1;
    }

    for (i = 0u; i < B1_EXPORT_SENSOR_SIZE; i++)
    {
        output[i] = 0u;
    }

#if VC_ENABLE
    {
        b3_snapshot_t line = { 0 };
        motion_status_t motion = { 0 };
        s4_imu_snapshot_t imu = { 0 };

        uint32_t now_us = b3_port_time_us();
        SYSTIM imu_now = { 0 };

        output[0] = 1u;

        b1_reset_observed_humps_if_controller_reset();

        /*
         * Buddy 3:
         * Line-following measurements.
         */
        buddy3_get_snapshot(&line);

        if (line.time_us != 0u &&
            ((uint32_t)(
                now_us - line.time_us
            ) <= B1_EXPORT_LINE_MAX_AGE_US))
        {
            output[1] |= 0x01u;

            output[2] = (uint8_t)line.line.state;
            output[3] = (uint8_t)line.line.error;

            if (line.line.left_black)
            {
                output[4] |= 0x01u;
            }

            if (line.line.right_black)
            {
                output[4] |= 0x02u;
            }

            if (line.control_active)
            {
                output[4] |= 0x04u;
            }

            if (line.scan_active)
            {
                output[4] |= 0x08u;
            }

            if (line.stop_required)
            {
                output[4] |= 0x10u;
            }

            b1_export_u16(
                &output[8],
                (uint16_t)line.line.left_mm_s
            );

            b1_export_u16(
                &output[10],
                (uint16_t)line.line.right_mm_s
            );
        }

        /*
         * Buddy 2:
         * Wheel speed, movement and encoders.
         */
        motion_get_status(&motion);

        output[1] |= 0x02u;
        output[5] = (uint8_t)motion.state;
        output[6] = (uint8_t)motion.fault;

        b1_export_u16(
            &output[12],
            (uint16_t)motion.speed_l_mmps
        );

        b1_export_u16(
            &output[14],
            (uint16_t)motion.speed_r_mmps
        );

        b1_export_u32(
            &output[16],
            (uint32_t)motion.distance_total_mm
        );

        b1_export_u32(
            &output[20],
            (uint32_t)motion.distance_trip_mm
        );

        b1_export_u32(
            &output[24],
            (uint32_t)motion.ticks_l
        );

        b1_export_u32(
            &output[28],
            (uint32_t)motion.ticks_r
        );

        b1_export_u32(
            &output[38],
            (uint32_t)motion.heading_deg
        );

        /*
         * Buddy 4:
         * Actual accelerometer and hump status.
         *
         * Reading Buddy 4 may block on its
         * internal mutex. This function now runs
         * outside the MQTT network task.
         */
        {
            ER imu_er =
                subsystem4_get_imu_snapshot(&imu);

            ER clock_er = tk_get_tim(&imu_now);

            bool imu_fresh =
                (imu_er >= E_OK &&
                 imu.has_sample &&
                 imu.accel_status == IMU_OK &&
                 clock_er >= E_OK &&
                 (uint32_t)(
                     imu_now.lo - imu.timestamp_ms
                 ) <= S4_IMU_MAX_AGE_MS);

            if (imu_fresh)
            {
                bool hump_now =
                    subsystem4_on_hump();

                bool driving =
                    b1_motion_is_driving(&motion);

                output[1] |= 0x04u;
                output[1] |= 0x08u;

                b1_export_u16(
                    &output[32],
                    (uint16_t)imu.accel.x
                );

                b1_export_u16(
                    &output[34],
                    (uint16_t)imu.accel.y
                );

                b1_export_u16(
                    &output[36],
                    (uint16_t)imu.accel.z
                );

                b1_update_hump(
                    hump_now,
                    driving,
                    now_us
                );

                if (b1_hump_confirmed)
                {
                    output[4] |= 0x20u;
                }
            }

            output[7] = b1_hump_events;
        }
    }

    return 0;
#else
    return -1;
#endif
}

/*
 * MQTT-facing sensor export.
 *
 * Copies the last fully prepared snapshot.
 *
 * TMO_POL means that if another task is
 * updating the cache, this function returns
 * immediately instead of blocking.
 *
 * No teammate sensor API is called here.
 */
int32_t b1_export_sensor_payload(
    uint8_t *output,
    uint32_t capacity)
{
    ER result;
    uint32_t age_us;

    if (NULL == output ||
        capacity < B1_EXPORT_SENSOR_SIZE ||
        b1_sensor_cache_mutex < E_OK)
    {
        return -1;
    }

    result = tk_loc_mtx(
        b1_sensor_cache_mutex,
        TMO_POL
    );

    if (result < E_OK)
    {
        return -1;
    }

    if (!b1_sensor_cache_valid)
    {
        (void)tk_unl_mtx(b1_sensor_cache_mutex);
        return -1;
    }

    age_us = (uint32_t)(
        b3_port_time_us() -
        b1_sensor_cache_time_us
    );

    if (age_us > B1_SENSOR_CACHE_MAX_AGE_US)
    {
        (void)tk_unl_mtx(b1_sensor_cache_mutex);
        return -1;
    }

    uint32_t i;

    for (i = 0u; i < B1_EXPORT_SENSOR_SIZE; i++)
    {
        output[i] = b1_sensor_cache[i];
    }

    (void)tk_unl_mtx(b1_sensor_cache_mutex);

    return 0;
}

/*
 * Dedicated sensor sampling task.
 *
 * Collects sensor data every 50 ms.
 *
 * MQTT remains free to handle:
 * - FR3 probes and acknowledgements
 * - Incoming START/STOP/RESET/SEARCH
 * - Barcode mapping
 * - Heartbeats
 * - Connection and reconnection
 */
static void b1_sensor_task(
    INT stacd,
    void *exinf)
{
    uint8_t working[B1_EXPORT_SENSOR_SIZE];
    ER result;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        if (0 == b1_build_sensor_payload(
                     working,
                     (uint32_t)sizeof(working)))
        {
            result = tk_loc_mtx(
                b1_sensor_cache_mutex,
                TMO_FEVR
            );

            if (result >= E_OK)
            {
                uint32_t i;

                for (i = 0u; i < B1_EXPORT_SENSOR_SIZE; i++)
                {
                    b1_sensor_cache[i] = working[i];
                }

                b1_sensor_cache_time_us =
                    b3_port_time_us();

                b1_sensor_cache_valid = true;

                (void)tk_unl_mtx(
                    b1_sensor_cache_mutex
                );
            }
        }

        (void)tk_dly_tsk(
            B1_SENSOR_SAMPLE_MS
        );
    }
}

/*
 * Subsystem 1 initialization.
 *
 * Initialize barcode storage and
 * create the sensor-cache mutex.
 */
ER subsystem1_init(void)
{
    T_CMTX attributes = { 0 };

    if (b1_mapping_store_init() != 0)
    {
        tm_printf(
            (UB *)"[BUDDY1] Mapping store initialization failed\n"
        );

        return E_SYS;
    }

    attributes.mtxatr = TA_INHERIT;

    b1_sensor_cache_mutex =
        tk_cre_mtx(&attributes);

    if (b1_sensor_cache_mutex < E_OK)
    {
        tm_printf(
            (UB *)"[BUDDY1] Sensor cache mutex creation failed\n"
        );

        return b1_sensor_cache_mutex;
    }

    tm_printf(
        (UB *)"[BUDDY1] Mapping and sensor cache initialized\n"
    );

    return E_OK;
}

/*
 * Start the Buddy 1 sensor task.
 *
 * The task does not own Wi-Fi or MQTT.
 * It cannot command the motors.
 */
ER subsystem1_start(void)
{
#if VC_ENABLE
    ID task_id = app_start_task(
        (FP)b1_sensor_task,
        B1_SENSOR_TASK_PRI,
        0u,
        "b1_sensor_task"
    );

    return (task_id < E_OK) ? task_id : E_OK;
#else
    return E_OK;
#endif
}

/* End of subsystem1.c */
