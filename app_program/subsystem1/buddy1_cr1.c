
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

/*
 * Run telemetry wire format, least-significant bit first:
 *
 * Bits   0..7   Packet count
 * Bits   8..10  Controller state
 * Bit       11  Wi-Fi connected
 * Bit       12  MQTT connected
 * Bits  13..15  IR_L, IR_R, IR_B black
 * Bits  16..23  Fault bitmask
 * Bits  24..33  Left wheel speed, mm/s, signed
 * Bits  34..43  Right wheel speed, mm/s, signed
 * Bits  44..63  Distance, mm, low 20 bits
 * Bits  64..74  Ultrasonic range, mm, 0 = nothing
 * Bits  75..79  Barcode letter, 0 = none, 1 = 'A' .. 26 = 'Z'
 * Bits  80..85  Junctions reached, low 6 bits
 * Bits  86..92  Side lift now, mm
 * Bits  93..99  Highest hump (peak side lift) this run, mm
 * Bits 100..119 Uptime, s, low 20 bits
 *
 * Total wire size: 15 bytes.
 */

static void b1_put_bits(uint8_t *p_out, uint32_t *p_pos, uint32_t value,
                        uint32_t width)
{
    uint32_t i;

    for (i = 0u; i < width; i++)
    {
        if (0u != ((value >> i) & 1u))
        {
            p_out[(*p_pos) >> 3] |= (uint8_t)(1u << ((*p_pos) & 7u));
        }
        (*p_pos)++;
    }
}

static uint32_t b1_clamp_u(uint32_t value, uint32_t max)
{
    return (value > max) ? max : value;
}

static uint32_t b1_signed10(int16_t value)
{
    int32_t v = value;

    if (v > 511)
    {
        v = 511;
    }
    if (v < -512)
    {
        v = -512;
    }
    return (uint32_t)v & 0x3FFu;
}

int32_t b1_run_pack(const B1_RunTelemetry *input,
                    uint8_t output[B1_RUN_BYTES])
{
    uint32_t pos = 0u;
    uint32_t i;
    uint32_t letter = 0u;

    if (0 == input || 0 == output)
    {
        return -1;
    }

    for (i = 0u; i < B1_RUN_BYTES; i++)
    {
        output[i] = 0u;
    }
    if ((input->letter >= 'A') && (input->letter <= 'Z'))
    {
        letter = (uint32_t)(input->letter - 'A') + 1u;
    }

    b1_put_bits(output, &pos, input->seq, 8u);
    b1_put_bits(output, &pos, input->controller_state & 7u, 3u);
    b1_put_bits(output, &pos, input->wifi_connected & 1u, 1u);
    b1_put_bits(output, &pos, input->mqtt_connected & 1u, 1u);
    b1_put_bits(output, &pos, input->ir_l & 1u, 1u);
    b1_put_bits(output, &pos, input->ir_r & 1u, 1u);
    b1_put_bits(output, &pos, input->ir_b & 1u, 1u);
    b1_put_bits(output, &pos, input->fault_flags, 8u);
    b1_put_bits(output, &pos, b1_signed10(input->speed_l_mm_s), 10u);
    b1_put_bits(output, &pos, b1_signed10(input->speed_r_mm_s), 10u);
    b1_put_bits(output, &pos, input->distance_mm & 0xFFFFFu, 20u);
    b1_put_bits(output, &pos, b1_clamp_u(input->range_mm, 2047u), 11u);
    b1_put_bits(output, &pos, letter, 5u);
    b1_put_bits(output, &pos, input->junctions & 0x3Fu, 6u);
    b1_put_bits(output, &pos, b1_clamp_u(input->lift_mm, 127u), 7u);
    b1_put_bits(output, &pos, b1_clamp_u(input->hump_mm, 127u), 7u);
    b1_put_bits(output, &pos, input->uptime_s & 0xFFFFFu, 20u);

    return 0;
}
