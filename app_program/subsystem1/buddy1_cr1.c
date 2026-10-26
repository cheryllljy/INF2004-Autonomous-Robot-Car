
#include "buddy1_cr1.h"

/*
 * Binary telemetry layout, least-significant bit first:
 *
 * Bits  0..2  Controller state (3 bits)
 * Bit      3  Wi-Fi connected (1 bit)
 * Bit      4  MQTT connected (1 bit)
 * Bits  5..12 Fault bitmask (8 bits)
 * Bits 13..19 Example measurement 0..100 (7 bits)
 * Bits 20..23 Reserved (zero)
 *
 * Total: 20 used bits in 3 bytes.
 */

/* Encode application values as compact binary data. */
int b1_telemetry_pack(
    const B1_Telemetry *input,
    uint8_t output[B1_TELEMETRY_BYTES])
{
    uint32_t packed;

    if (input == 0 || output == 0)
        return -1;

    /* Reject values outside their defined ranges. */
    if (input->controller_state > 7u ||
        input->wifi_connected > 1u ||
        input->mqtt_connected > 1u ||
        input->metric_0_100 > 100u)
    {
        return -1;
    }

    packed =
        ((uint32_t)input->controller_state) |
        ((uint32_t)input->wifi_connected << 3) |
        ((uint32_t)input->mqtt_connected << 4) |
        ((uint32_t)input->fault_flags << 5) |
        ((uint32_t)input->metric_0_100 << 13);

    output[0] = (uint8_t)(packed & 0xFFu);
    output[1] = (uint8_t)((packed >> 8) & 0xFFu);
    output[2] = (uint8_t)((packed >> 16) & 0xFFu);

    return 0;
}

/* Decode binary data back into application values. */
int b1_telemetry_unpack(
    const uint8_t input[B1_TELEMETRY_BYTES],
    B1_Telemetry *output)
{
    uint32_t packed;

    if (input == 0 || output == 0)
        return -1;

    packed =
        ((uint32_t)input[0]) |
        ((uint32_t)input[1] << 8) |
        ((uint32_t)input[2] << 16);

    /* Reject unexpected reserved bits. */
    if ((packed & 0xF00000u) != 0u)
        return -1;

    output->controller_state =
        (uint8_t)(packed & 0x07u);

    output->wifi_connected =
        (uint8_t)((packed >> 3) & 0x01u);

    output->mqtt_connected =
        (uint8_t)((packed >> 4) & 0x01u);

    output->fault_flags =
        (uint8_t)((packed >> 5) & 0xFFu);

    output->metric_0_100 =
        (uint8_t)((packed >> 13) & 0x7Fu);

    if (output->metric_0_100 > 100u)
        return -1;

    return 0;
}

/*
 * Commands currently use IDs from 0 to 7.
 * Final meanings must match the agreed command table.
 */
int b1_command_pack(uint8_t command, uint8_t *output)
{
    if (output == 0 || command > 7u)
        return -1;

    *output = (uint8_t)(command & 0x07u);

    return 0;
}

int b1_command_unpack(uint8_t input, uint8_t *command)
{
    if (command == 0 || (input & 0xF8u) != 0u)
        return -1;

    *command = (uint8_t)(input & 0x07u);

    return 0;
}
