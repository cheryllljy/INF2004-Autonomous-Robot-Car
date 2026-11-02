
/*
 * buddy1_mqtt.h
 *
 * Buddy 1 MQTT telemetry preparation.
 * MQTT networking remains in mqtt_utk.c.
 */

#ifndef BUDDY1_MQTT_H
#define BUDDY1_MQTT_H

#include <stdint.h>
#include "buddy1_cr1.h"

/*
 * Prepare the compact CR1 telemetry payload.
 *
 * Returns 0 on success.
 * Returns -1 for invalid input.
 */
int buddy1_mqtt_prepare_telemetry(
    const B1_Telemetry *telemetry,
    uint8_t payload[B1_TELEMETRY_BYTES]
);

#endif /* BUDDY1_MQTT_H */
