
/*
 * Buddy 1 - Change Requirement 1 tests
 *
 * Tests compact telemetry and command encoding.
 * No Wi-Fi or MQTT broker required.
 */

#include "subsystem1.h"
#include "buddy1_cr1.h"
#include "../test_report.h"

#if APP_TEST

void b1_cr1_test(void)
{
    B1_Telemetry original = {
        .controller_state = 5,
        .wifi_connected = 1,
        .mqtt_connected = 1,
        .fault_flags = B1_FAULT_WIFI_LOSS,
        .metric_0_100 = 100
    };

    B1_Telemetry decoded = {0};
    uint8_t payload[B1_TELEMETRY_BYTES] = {0};
    uint8_t command_byte = 0;
    uint8_t decoded_command = 0;
    int result;

    TEST_INIT();

    tm_printf((UB*)
        "\n[BUDDY1] CR1 minimum-bit representation tests\n");

    /* B1-C01: binary payload occupies 3 bytes. */
    TEST_CHECK(
        "B1-C01",
        sizeof(payload) == 3u,
        "Telemetry payload is 3 bytes"
    );

    /* B1-C02: valid input packs successfully. */
    result = b1_telemetry_pack(&original, payload);

    TEST_CHECK(
        "B1-C02",
        result == 0,
        "Telemetry packing succeeds"
    );

    /* B1-C03: exact binary layout. */
    TEST_CHECK(
        "B1-C03",
        payload[0] == 0x1Du &&
        payload[1] == 0x82u &&
        payload[2] == 0x0Cu,
        "Telemetry bit layout is correct"
    );

    /* B1-C04: unpacking restores the values. */
    result = b1_telemetry_unpack(payload, &decoded);

    TEST_CHECK(
        "B1-C04",
        result == 0 &&
        decoded.controller_state ==
            original.controller_state &&
        decoded.wifi_connected ==
            original.wifi_connected &&
        decoded.mqtt_connected ==
            original.mqtt_connected &&
        decoded.fault_flags ==
            original.fault_flags &&
        decoded.metric_0_100 ==
            original.metric_0_100,
        "Telemetry round-trip preserves values"
    );

    /* B1-C05: reject invalid controller state. */
    original.controller_state = 8;

    TEST_CHECK(
        "B1-C05",
        b1_telemetry_pack(&original, payload) != 0,
        "Invalid controller state rejected"
    );

    /* B1-C06: reject invalid measurement. */
    original.controller_state = 5;
    original.metric_0_100 = 101;

    TEST_CHECK(
        "B1-C06",
        b1_telemetry_pack(&original, payload) != 0,
        "Out-of-range measurement rejected"
    );

    /* B1-C07: encode a compact command. */
    result = b1_command_pack(3, &command_byte);

    TEST_CHECK(
        "B1-C07",
        result == 0 && command_byte == 3,
        "Command encoded using 3 bits"
    );

    /* B1-C08: decode the command. */
    result = b1_command_unpack(
        command_byte,
        &decoded_command
    );

    TEST_CHECK(
        "B1-C08",
        result == 0 && decoded_command == 3,
        "Command decoded correctly"
    );

    /* B1-C09: reject invalid command ID. */
    TEST_CHECK(
        "B1-C09",
        b1_command_pack(8, &command_byte) != 0,
        "Invalid command rejected"
    );

    /* B1-C10: reject invalid reserved bits. */
    TEST_CHECK(
        "B1-C10",
        b1_command_unpack(0x80u, &decoded_command) != 0,
        "Reserved command bits rejected"
    );

    TEST_SUMMARY("buddy1_cr1");
}

#endif /* APP_TEST */
