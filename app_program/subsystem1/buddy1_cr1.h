
#ifndef BUDDY1_CR1_H
#define BUDDY1_CR1_H

#include <stdint.h>


/*
 * CR1: compact binary application data.
 *
 * These are provisional field definitions.
 * Final telemetry fields must be agreed with the team.
 */

#define B1_TELEMETRY_BYTES 3u
#define B1_COMMAND_BYTES   1u

/* Shared fault-bit positions: provisional mapping. */
#define B1_FAULT_MOTOR_STALL  (1u << 0)
#define B1_FAULT_ENCODER      (1u << 1)
#define B1_FAULT_IMU          (1u << 2)
#define B1_FAULT_NO_ECHO      (1u << 3)
#define B1_FAULT_WIFI_LOSS    (1u << 4)

/*
 * Application values before packing.
 *
 * This struct is an input format, NOT the wire format.
 * Do not send the struct directly over MQTT.
 */
typedef struct {
    uint8_t controller_state; /* 0..7 */
    uint8_t wifi_connected;   /* 0..1 */
    uint8_t mqtt_connected;   /* 0..1 */
    uint8_t fault_flags;      /* eight individual fault bits */
    uint8_t metric_0_100;     /* example measurement */
} B1_Telemetry;

/*
 * Pack values into three bytes.
 * Returns 0 on success, -1 for invalid arguments.
 */
int b1_telemetry_pack(
    const B1_Telemetry *input,
    uint8_t output[B1_TELEMETRY_BYTES]
);

/* Decode a three-byte telemetry payload. */
int b1_telemetry_unpack(
    const uint8_t input[B1_TELEMETRY_BYTES],
    B1_Telemetry *output
);

/* Encode command ID 0..7 into one byte. */
int b1_command_pack(uint8_t command, uint8_t *output);

/* Decode a compact command byte. */
int b1_command_unpack(uint8_t input, uint8_t *command);

#endif /* BUDDY1_CR1_H */
