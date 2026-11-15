
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
#define B1_FAULT_MOTOR_STALL (1u << 0)
#define B1_FAULT_ENCODER     (1u << 1)
#define B1_FAULT_IMU         (1u << 2)
#define B1_FAULT_NO_ECHO     (1u << 3)
#define B1_FAULT_WIFI_LOSS   (1u << 4)

/*
 * Application values before packing.
 *
 * This struct is an input format, NOT the wire format.
 * Do not send the struct directly over MQTT.
 */
typedef struct
{
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
int32_t b1_telemetry_pack(const B1_Telemetry *input,
                          uint8_t output[B1_TELEMETRY_BYTES]);

/* Decode a three-byte telemetry payload. */
int32_t b1_telemetry_unpack(const uint8_t input[B1_TELEMETRY_BYTES],
                            B1_Telemetry *output);

/*
 * Run telemetry: sent every 200 ms on inf2004/robot/run so the dashboard
 * can record and chart each run. Bit layout in buddy1_cr1.c.
 */
#define B1_RUN_BYTES 15u

typedef struct
{
    uint8_t seq;              /* counts packets, wraps */
    uint8_t controller_state; /* 0..7 */
    uint8_t wifi_connected;   /* 0..1 */
    uint8_t mqtt_connected;   /* 0..1 */
    uint8_t ir_l;             /* 0..1: left line IR sees black */
    uint8_t ir_r;             /* 0..1: right line IR sees black */
    uint8_t ir_b;             /* 0..1: barcode IR sees black */
    uint8_t fault_flags;      /* eight individual fault bits */
    int16_t speed_l_mm_s;     /* sent as -512..511 */
    int16_t speed_r_mm_s;     /* sent as -512..511 */
    uint32_t distance_mm;     /* low 20 bits sent */
    uint16_t range_mm;        /* 0 = nothing; sent up to 2047 */
    char letter;              /* 'A'..'Z', '\0' = none */
    uint8_t junctions;        /* low 6 bits sent */
    uint8_t lift_mm;          /* side lift now, sent up to 127 */
    uint8_t hump_mm;          /* run's highest hump, sent up to 127 */
    uint32_t uptime_s;        /* since boot, low 20 bits sent */
} B1_RunTelemetry;

/* Pack into B1_RUN_BYTES bytes. Returns 0, or -1 for invalid arguments. */
int32_t b1_run_pack(const B1_RunTelemetry *input,
                    uint8_t output[B1_RUN_BYTES]);

/* Encode command ID 0..7 into one byte. */
int32_t b1_command_pack(uint8_t command, uint8_t *output);

/* Decode a compact command byte. */
int32_t b1_command_unpack(uint8_t input, uint8_t *command);

#endif /* BUDDY1_CR1_H */
