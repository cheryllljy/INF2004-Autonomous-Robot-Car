
/*
 * buddy1_mqtt_test.c
 *
 * Buddy 1 MQTT binary payload unit tests.
 *
 * B1-M01: Encode telemetry successfully
 * B1-M02: Verify the 3-byte payload size
 * B1-M03: Verify telemetry round-trip
 */

#include "subsystem1.h"
#include "buddy1_mqtt.h"
#include "../test_report.h"

#if APP_TEST

void buddy1_mqtt_test(void)
{
    B1_Telemetry original = { .controller_state = 0,
                              .wifi_connected = 1,
                              .mqtt_connected = 1,
                              .fault_flags = 0,
                              .metric_0_100 = 0 };

    B1_Telemetry decoded = { 0 };

    uint8_t payload[B1_TELEMETRY_BYTES] = { 0 };

    int32_t pack_result;
    int32_t unpack_result;

    TEST_INIT();

    tm_printf((UB *)"\n[BUDDY1] MQTT binary payload tests\n");

    /* B1-M01: Encode binary telemetry */
    pack_result = buddy1_mqtt_prepare_telemetry(&original, payload);

    TEST_CHECK("B1-M01",
               pack_result == 0 && payload[0] == 0x18u && payload[1] == 0x00u &&
                   payload[2] == 0x00u,
               "MQTT binary payload prepared");

    /* B1-M02: Payload must be 3 bytes */
    TEST_CHECK("B1-M02", sizeof(payload) == 3u && B1_TELEMETRY_BYTES == 3u,
               "MQTT binary payload size is 3 bytes");

    /* B1-M03: Decode and compare telemetry */
    unpack_result = b1_telemetry_unpack(payload, &decoded);

    TEST_CHECK("B1-M03",
               unpack_result == 0 &&
                   decoded.controller_state == original.controller_state &&
                   decoded.wifi_connected == original.wifi_connected &&
                   decoded.mqtt_connected == original.mqtt_connected &&
                   decoded.fault_flags == original.fault_flags &&
                   decoded.metric_0_100 == original.metric_0_100,
               "MQTT telemetry decoded correctly");

    TEST_SUMMARY("buddy1_mqtt");
}

#endif /* APP_TEST */
