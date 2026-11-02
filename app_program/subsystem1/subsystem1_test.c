
/*
 * subsystem1_test.c
 *
 * INF2004 Autonomous Robot Car
 * Buddy 1 - Combined Subsystem Tests
 *
 * Wi-Fi: B1-W01
 * CR1: B1-C01 to B1-C10
 * MQTT payload: B1-M01 to B1-M03
 * Barcode mapping: B1-P01 to B1-P10
 * Shared mapping API: B1-S01 to B1-S06
 */

#include "subsystem1.h"
#include "buddy1_cr1.h"
#include "buddy1_mqtt.h"
#include "buddy1_mapping.h"
#include "../test_report.h"

#if TM_NET_LWIP
#include "cyw43_utk.h"
#include "mqtt_utk.h"
#endif

#if APP_TEST

EXPORT void subsystem1_test(void)
{
    TEST_INIT();

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

    B1_Telemetry mqtt_sample = {
        .controller_state = 0,
        .wifi_connected = 1,
        .mqtt_connected = 1,
        .fault_flags = 0,
        .metric_0_100 = 0
    };

    B1_Telemetry mqtt_decoded = {0};
    uint8_t mqtt_payload[B1_TELEMETRY_BYTES] = {0};

    /*
     * Example mappings only.
     * Actual mappings are received through MQTT.
     */
    uint8_t mapping_payload[B1_MAPPING_BYTES] = {
        'A', B1_MOVE_LEFT,
        'F', B1_MOVE_RIGHT,
        'K', B1_MOVE_FORWARD,
        'Z', B1_MOVE_UTURN
    };

    uint8_t invalid_payload[B1_MAPPING_BYTES];
    B1_Mapping mapping = {0};
    B1_Mapping saved_mapping = {0};
    B1_Movement movement = B1_MOVE_FORWARD;

    INT result;
    size_t i;
    size_t j;
    INT all_letters_valid = 1;

    tm_printf((UB*)
        "\n==================================\n"
        " BUDDY 1 COMBINED TESTS\n"
        " Wi-Fi / CR1 / MQTT / Mapping\n"
        "==================================\n");

    /*
     * SECTION 1: Wi-Fi connectivity
     */
#if TM_NET_LWIP
    {
        T_CYW43_UTK_STATUS status = {0};
        INT connected = 0;
        INT attempt;

        for (attempt = 0; attempt < 90; attempt++)
        {
            cyw43_utk_get_status(&status);

            if (status.init_complete &&
                status.ready &&
                status.ip_addr != 0)
            {
                connected = 1;
                break;
            }

            tk_dly_tsk(1000);
        }

        TEST_CHECK(
            "B1-W01",
            connected == 1,
            "Wi-Fi connected with IPv4 address"
        );
    }
#else
    tm_printf((UB*)
        "[BUDDY1] Wi-Fi test skipped\n");
#endif

    /*
     * SECTION 2: CR1 binary payload tests
     */
    tm_printf((UB*)
        "\n[BUDDY1] CR1 tests\n");

    TEST_CHECK(
        "B1-C01",
        sizeof(payload) == 3u,
        "Telemetry payload occupies 3 bytes"
    );

    result = b1_telemetry_pack(&original, payload);

    TEST_CHECK(
        "B1-C02",
        result == 0,
        "Telemetry packing succeeds"
    );

    TEST_CHECK(
        "B1-C03",
        payload[0] == 0x1Du &&
        payload[1] == 0x82u &&
        payload[2] == 0x0Cu,
        "Telemetry binary layout correct"
    );

    result = b1_telemetry_unpack(
        payload,
        &decoded
    );

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
        "Telemetry round-trip correct"
    );

    original.controller_state = 8;

    TEST_CHECK(
        "B1-C05",
        b1_telemetry_pack(
            &original,
            payload) != 0,
        "Invalid controller state rejected"
    );

    original.controller_state = 5;
    original.metric_0_100 = 101;

    TEST_CHECK(
        "B1-C06",
        b1_telemetry_pack(
            &original,
            payload) != 0,
        "Invalid measurement rejected"
    );

    result = b1_command_pack(
        3,
        &command_byte
    );

    TEST_CHECK(
        "B1-C07",
        result == 0 &&
        command_byte == 3u,
        "Command packed into 3 bits"
    );

    result = b1_command_unpack(
        command_byte,
        &decoded_command
    );

    TEST_CHECK(
        "B1-C08",
        result == 0 &&
        decoded_command == 3u,
        "Command decoded correctly"
    );

    TEST_CHECK(
        "B1-C09",
        b1_command_pack(
            8,
            &command_byte) != 0,
        "Invalid command rejected"
    );

    TEST_CHECK(
        "B1-C10",
        b1_command_unpack(
            0x80u,
            &decoded_command) != 0,
        "Reserved command bits rejected"
    );

    /*
     * SECTION 3: MQTT binary payload tests
     */
    tm_printf((UB*)
        "\n[BUDDY1] MQTT payload tests\n");

    result = buddy1_mqtt_prepare_telemetry(
        &mqtt_sample,
        mqtt_payload
    );

    TEST_CHECK(
        "B1-M01",
        result == 0 &&
        mqtt_payload[0] == 0x18u &&
        mqtt_payload[1] == 0x00u &&
        mqtt_payload[2] == 0x00u,
        "MQTT binary payload prepared"
    );

    TEST_CHECK(
        "B1-M02",
        sizeof(mqtt_payload) == 3u &&
        B1_TELEMETRY_BYTES == 3u,
        "MQTT payload size is 3 bytes"
    );

    result = b1_telemetry_unpack(
        mqtt_payload,
        &mqtt_decoded
    );

    TEST_CHECK(
        "B1-M03",
        result == 0 &&
        mqtt_decoded.controller_state == 0 &&
        mqtt_decoded.wifi_connected == 1 &&
        mqtt_decoded.mqtt_connected == 1 &&
        mqtt_decoded.fault_flags == 0 &&
        mqtt_decoded.metric_0_100 == 0,
        "MQTT telemetry decoded correctly"
    );

    /*
     * SECTION 4: Runtime barcode mapping
     */
    tm_printf((UB*)
        "\n[BUDDY1] Barcode mapping tests\n");

    result = b1_mapping_decode(
        mapping_payload,
        B1_MAPPING_BYTES,
        &mapping
    );

    TEST_CHECK(
        "B1-P01",
        result == 0,
        "Valid four-entry mapping accepted"
    );

    result = b1_mapping_lookup(
        &mapping,
        'F',
        &movement
    );

    TEST_CHECK(
        "B1-P02",
        result == 0 &&
        movement == B1_MOVE_RIGHT,
        "Mapped barcode lookup correct"
    );

    TEST_CHECK(
        "B1-P03",
        b1_mapping_lookup(
            &mapping,
            'Q',
            &movement) != 0,
        "Unmapped barcode rejected"
    );

    /*
     * Verify support for all A-Z characters.
     */
    for (i = 0; i < 26u; i++)
    {
        uint8_t letter_payload[B1_MAPPING_BYTES];

        for (j = 0; j < B1_MAPPING_COUNT; j++)
        {
            letter_payload[j * 2u] =
                (uint8_t)('A' + ((i + j) % 26u));

            letter_payload[j * 2u + 1u] =
                (uint8_t)j;
        }

        if (b1_mapping_decode(
                letter_payload,
                sizeof(letter_payload),
                &mapping) != 0)
        {
            all_letters_valid = 0;
            break;
        }

        if (b1_mapping_lookup(
                &mapping,
                (char)('A' + i),
                &movement) != 0 ||
            movement != B1_MOVE_FORWARD)
        {
            all_letters_valid = 0;
            break;
        }
    }

    TEST_CHECK(
        "B1-P04",
        all_letters_valid == 1,
        "Runtime mapping supports A-Z"
    );

    /*
     * Restore known valid mapping.
     */
    b1_mapping_decode(
        mapping_payload,
        B1_MAPPING_BYTES,
        &mapping
    );

    saved_mapping = mapping;

    /*
     * Invalid character.
     */
    for (i = 0; i < B1_MAPPING_BYTES; i++)
    {
        invalid_payload[i] = mapping_payload[i];
    }

    invalid_payload[0] = (uint8_t)'1';

    TEST_CHECK(
        "B1-P05",
        b1_mapping_decode(
            invalid_payload,
            B1_MAPPING_BYTES,
            &mapping) != 0 &&
        mapping.entries[0].character ==
            saved_mapping.entries[0].character,
        "Invalid character rejected safely"
    );

    /*
     * Duplicate barcode character.
     */
    invalid_payload[0] = (uint8_t)'A';
    invalid_payload[2] = (uint8_t)'A';

    TEST_CHECK(
        "B1-P06",
        b1_mapping_decode(
            invalid_payload,
            B1_MAPPING_BYTES,
            &mapping) != 0,
        "Duplicate barcode rejected"
    );

    /*
     * Duplicate movement.
     */
    for (i = 0; i < B1_MAPPING_BYTES; i++)
    {
        invalid_payload[i] = mapping_payload[i];
    }

    invalid_payload[1] = B1_MOVE_FORWARD;
    invalid_payload[3] = B1_MOVE_FORWARD;

    TEST_CHECK(
        "B1-P07",
        b1_mapping_decode(
            invalid_payload,
            B1_MAPPING_BYTES,
            &mapping) != 0,
        "Duplicate movement rejected"
    );

    /*
     * Invalid movement ID.
     */
    invalid_payload[3] = 4u;

    TEST_CHECK(
        "B1-P08",
        b1_mapping_decode(
            invalid_payload,
            B1_MAPPING_BYTES,
            &mapping) != 0,
        "Invalid movement ID rejected"
    );

    /*
     * Incorrect payload length.
     */
    TEST_CHECK(
        "B1-P09",
        b1_mapping_decode(
            mapping_payload,
            B1_MAPPING_BYTES - 1u,
            &mapping) != 0,
        "Incorrect mapping size rejected"
    );

    /*
     * Null input handling.
     */
    TEST_CHECK(
        "B1-P10",
        b1_mapping_decode(
            NULL,
            B1_MAPPING_BYTES,
            &mapping) != 0 &&
        b1_mapping_lookup(
            NULL,
            'A',
            &movement) != 0,
        "Null mapping arguments rejected"
    );

    /*
     * SECTION 5: Shared mapping API tests.
     *
     * The mapping store must already be
     * initialized by subsystem1_init().
     */
    tm_printf((UB*)
        "\n[BUDDY1] Shared mapping API tests\n");

    result = b1_mapping_decode(
        mapping_payload,
        B1_MAPPING_BYTES,
        &mapping
    );

    TEST_CHECK(
        "B1-S01",
        result == 0,
        "Valid mapping decoded for shared store"
    );

    result = b1_mapping_store_latest(&mapping);

    TEST_CHECK(
        "B1-S02",
        result == 0,
        "Shared mapping stored"
    );

    result = b1_mapping_get_latest(&saved_mapping);

    TEST_CHECK(
        "B1-S03",
        result == 0 &&
        saved_mapping.entries[0].character == 'A' &&
        saved_mapping.entries[1].character == 'F' &&
        saved_mapping.entries[2].character == 'K' &&
        saved_mapping.entries[3].character == 'Z',
        "Complete shared mapping snapshot retrieved"
    );

    result = b1_mapping_get_movement(
        'F',
        &movement
    );

    TEST_CHECK(
        "B1-S04",
        result == 0 &&
        movement == B1_MOVE_RIGHT,
        "VehicleController-style API maps F to RIGHT"
    );

    TEST_CHECK(
        "B1-S05",
        b1_mapping_get_movement(
            'Q',
            &movement) != 0,
        "Unmapped barcode not executed"
    );

    TEST_CHECK(
        "B1-S06",
        b1_mapping_get_latest(0) != 0 &&
        b1_mapping_get_movement(
            'F',
            0) != 0,
        "Null destination rejected"
    );

#if TM_NET_LWIP
    /*
     * READ-ONLY FR3 and CR1 producer API tests.
     * NEVER auto-clear a real safety latch during tests.
     */
    {
        B1_FR3_Status fr3 = {0};
        B1_Telemetry live = {0};
        INT status_rc = b1_fr3_get_status(&fr3);

        TEST_CHECK("B1-F01",
            status_rc == 0 &&
            fr3.safety_stop_required ==
                (uint8_t)(!fr3.healthy || fr3.link_lost_latched),
            "FR3 status snapshot consistency");

        TEST_CHECK("B1-F02",
            b1_fr3_get_status(0) != 0,
            "FR3 null output safely rejected");

        TEST_CHECK("B1-T01",
            b1_mqtt_set_robot_status(8u, 0u, 0u) != 0 &&
            b1_mqtt_set_robot_status(0u, 0u, 101u) != 0,
            "Live telemetry rejects invalid fields");

        TEST_CHECK("B1-T02",
            b1_mqtt_get_robot_status(0) != 0,
            "Live telemetry null output rejected");

        /*
         * Do not seed the live status with invented test data.
         * VehicleController must provide actual values.
         */
        (void)live;
    }
#endif

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
