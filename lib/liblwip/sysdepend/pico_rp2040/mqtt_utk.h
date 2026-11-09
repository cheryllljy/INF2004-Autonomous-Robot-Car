
/*
 * INF2004 - Buddy 1 public communication API.
 *
 * Networking runs in cyw43_utk_task.
 * Controller must not call lwIP directly.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

#include "../../../../app_program/subsystem1/buddy1_cr1.h"

void mqtt_utk_start(void);

/*
 * Controller provides REAL robot values.
 *
 * No test samples are automatically published.
 *
 * Returns 0 if accepted, -1 if invalid.
 */
int32_t b1_mqtt_set_robot_status(uint8_t controller_state, uint8_t fault_flags,
                                 uint8_t metric_0_100);

/*
 * Returns -1 until controller data exists.
 */
int32_t b1_mqtt_get_robot_status(B1_Telemetry *output);

typedef struct
{
    uint8_t wifi_up;
    uint8_t mqtt_up;
    uint8_t healthy;
    uint8_t link_lost_latched;
    uint8_t safety_stop_required;
    uint8_t loss_reason;
} B1_FR3_Status;

/* nyx, 2026-11-07: how long the broker may stay silent before the car
 * stops (was 3 s). On the phone hotspot one lost packet's TCP
 * retransmission alone can take a few seconds, which stopped the car
 * many times per run. */
#ifndef B1_FR3_TIMEOUT_MS
#define B1_FR3_TIMEOUT_MS 10000u
#endif

#define B1_FR3_REASON_NONE 0u
#define B1_FR3_REASON_WIFI 1u
#define B1_FR3_REASON_MQTT 2u

/*
 * Safe status snapshot for VehicleController.
 */
int32_t b1_fr3_get_status(B1_FR3_Status *output);

/*
 * Request an authorized safety-latch reset.
 *
 * Returns:
 *  0 = request accepted for network-task processing
 * -1 = communication currently unhealthy
 *
 * Return 0 does NOT mean the latch has
 * already cleared. Call get_status() afterward.
 *
 * VehicleController must ensure:
 * - Motors physically stopped
 * - Fault cause has been addressed
 * - Valid RESET command received
 * - A later START is required to resume
 */
int32_t b1_fr3_controller_reset(void);

#endif
