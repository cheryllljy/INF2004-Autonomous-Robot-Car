
/*
 * buddy1_cr1.c
 *
 * Buddy 1 - CR1 compact binary encoding.
 *
 * Handles:
 * - Three-byte telemetry packing
 * - Three-byte telemetry decoding
 * - Compact command encoding
 * - MQTT telemetry payload preparation
 *
 * Does not connect to Wi-Fi or MQTT.
 */

#include "buddy1_cr1.h"
#include "buddy1_mqtt.h"

/*
 * Telemetry wire format, least-significant bit first:
 *
 * Bits  0..2  Controller state
 * Bit      3  Wi-Fi connected
 * Bit      4  MQTT connected
 * Bits  5..12 Fault bitmask
 * Bits 13..19 Normalized measurement (0..100)
 * Bits 20..23 Reserved, must be zero
 *
 * Total wire size: 3 bytes.
 *
 * These field definitions are provisional until
 * the team's final telemetry specification is agreed.
 */

int32_t b1_telemetry_pack(const B1_Telemetry *input,
                          uint8_t output[B1_TELEMETRY_BYTES])
{
    uint32_t packed;

    if (0 == input || 0 == output)
    {
        return -1;
    }

    if (input->controller_state > 7u || input->wifi_connected > 1u ||
        input->mqtt_connected > 1u || input->metric_0_100 > 100u)
    {
        return -1;
    }

    packed = ((uint32_t)input->controller_state) |
             ((uint32_t)input->wifi_connected << 3) |
             ((uint32_t)input->mqtt_connected << 4) |
             ((uint32_t)input->fault_flags << 5) |
             ((uint32_t)input->metric_0_100 << 13);

    output[0] = (uint8_t)(packed & 0xFFu);
    output[1] = (uint8_t)((packed >> 8) & 0xFFu);
    output[2] = (uint8_t)((packed >> 16) & 0xFFu);

    return 0;
}

int32_t b1_telemetry_unpack(const uint8_t input[B1_TELEMETRY_BYTES],
                            B1_Telemetry *output)
{
    uint32_t packed;

    if (0 == input || 0 == output)
    {
        return -1;
    }

    packed = ((uint32_t)input[0]) | ((uint32_t)input[1] << 8) |
             ((uint32_t)input[2] << 16);

    if ((packed & 0xF00000u) != 0u)
    {
        return -1;
    }

    output->controller_state = (uint8_t)(packed & 0x07u);

    output->wifi_connected = (uint8_t)((packed >> 3) & 0x01u);

    output->mqtt_connected = (uint8_t)((packed >> 4) & 0x01u);

    output->fault_flags = (uint8_t)((packed >> 5) & 0xFFu);

    output->metric_0_100 = (uint8_t)((packed >> 13) & 0x7Fu);

    if (output->metric_0_100 > 100u)
    {
        return -1;
    }

    return 0;
}

/*
 * Legacy compact command helper.
 *
 * FR2 network commands use their separate
 * versioned three-byte format.
 */
int32_t b1_command_pack(uint8_t command, uint8_t *output)
{
    if (0 == output || command > 7u)
    {
        return -1;
    }

    *output = (uint8_t)(command & 0x07u);

    return 0;
}

int32_t b1_command_unpack(uint8_t input, uint8_t *command)
{
    if (0 == command || (input & 0xF8u) != 0u)
    {
        return -1;
    }

    *command = (uint8_t)(input & 0x07u);

    return 0;
}

/*
 * Moved from buddy1_mqtt.c.
 *
 * Prepare a three-byte CR1 payload for MQTT.
 * The MQTT network task performs the publish.
 */
int32_t buddy1_mqtt_prepare_telemetry(const B1_Telemetry *telemetry,
                                      uint8_t payload[B1_TELEMETRY_BYTES])
{
    if (0 == telemetry || 0 == payload)
    {
        return -1;
    }

    return b1_telemetry_pack(telemetry, payload);
}
