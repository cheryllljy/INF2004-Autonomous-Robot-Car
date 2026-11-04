
/*
 * INF2004 - Buddy 1 MQTT communication
 *
 * Wi-Fi is owned by cyw43_utk_task.
 * All lwIP/MQTT operations run in that network task.
 *
 * FR2: command validation, sequence protection,
 *      four-entry queue and priority STOP mailbox.
 *
 * FR3: Wi-Fi/MQTT fault detection, safety latch,
 *      controller-authorized reset request.
 *
 * CR1: publish real controller data only.
 *
 * Motor control remains VehicleController's job.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/timeouts.h"

#include "cyw43.h"
#include "cyw43_utk.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"

#include <tm/tmonitor.h>
#include <string.h>

#include "../../../../app_program/subsystem1/buddy1_cr1.h"
#include "../../../../app_program/subsystem1/buddy1_mqtt.h"
#include "../../../../app_program/subsystem1/buddy1_mapping.h"
#include "../../../../app_program/subsystem1/buddy1_fr2.h"

#define HEARTBEAT_MS 5000u
#define MQTT_RETRY_MS 3000u
#define MQTT_KEEPALIVE_S 60u

#define COMMAND_TOPIC "inf2004/robot/command"
#define PROBE_TOPIC "inf2004/robot/status/probe"

#define RX_TOPIC_MAX 64u
#define RX_DATA_MAX 128u

#define FR3_CHECK_MS 100u
#define FR3_PROBE_MS 1000u
#define FR3_TIMEOUT_US 3000000ull

#define STATUS_WIFI 0x01u
#define STATUS_MQTT 0x02u
#define STATUS_HEALTHY 0x04u
#define STATUS_LATCHED 0x08u

#define STATUS_REASON_SHIFT 4u
#define STATUS_REASON_MASK 0x30u

#define ROBOT_STATUS_VALID (1u << 18)

extern uint64_t time_us_64(void);

static mqtt_client_t *client = NULL;
static ip_addr_t broker_addr;

static u8_t connecting = 0;
static u8_t retry_pending = 0;

static char rx_topic[RX_TOPIC_MAX];
static u8_t rx_data[RX_DATA_MAX];
static u16_t rx_len = 0;
static u8_t rx_truncated = 0;

/*
 * Normal FR2 commands:
 * one network producer, one controller consumer.
 */
static B1_FR2_Command fr2_queue[B1_FR2_QUEUE_SIZE];

static uint32_t fr2_head = 0;
static uint32_t fr2_tail = 0;

static uint8_t fr2_sequence_ready = 0;
static uint8_t fr2_last_sequence = 0;

/*
 * Priority STOP mailbox.
 *
 * Bits 0..7: STOP sequence
 * Bits 8..31: generation
 *
 * Network producer writes fr2_stop_word.
 * Controller consumer writes fr2_stop_ack.
 *
 * A newer STOP supersedes an older STOP.
 * No compare-exchange library is required.
 */
static uint32_t fr2_stop_word = 0;
static uint32_t fr2_stop_ack = 0;
static uint32_t fr2_stop_generation = 0;

/*
 * Latest accepted STOP sequence.
 * Allows the consumer to discard queued
 * commands older than a priority STOP.
 */
static uint32_t fr2_last_stop_word = 0;

/*
 * Controller-provided real telemetry snapshot.
 */
static uint32_t robot_status_word = 0;

/*
 * FR3 public status.
 * Only network task writes this word.
 */
static uint32_t fr3_status_word = 0;

/*
 * Controller requests an FR3 latch reset.
 * Only network task processes requests.
 */
static uint32_t fr3_reset_request = 0;
static uint32_t fr3_reset_ack = 0;

static u8_t fr3_monitor_started = 0;
static u8_t fr3_probe_pending = 0;

static uint64_t fr3_last_ack_us = 0;
static uint64_t fr3_last_probe_us = 0;
static uint64_t fr3_loss_start_us = 0;

static uint8_t fr3_loss_active = 0;
static uint8_t fr3_latched = 0;
static uint8_t fr3_loss_reason = B1_FR3_REASON_NONE;
static uint8_t fr3_reported_reason = B1_FR3_REASON_NONE;

static void heartbeat_timeout(void *arg);
static void test_log_timeout(void *arg);
static void retry_timeout(void *arg);
static void fr3_monitor_timeout(void *arg);

static void publish_done_cb(void *arg, err_t result);
static void probe_done_cb(void *arg, err_t result);
static void subscribe_done_cb(void *arg, err_t result);

static void incoming_publish_cb(
    void *arg,
    const char *topic,
    u32_t total_length
);

static void incoming_data_cb(
    void *arg,
    const u8_t *data,
    u16_t len,
    u8_t flags
);

static void connection_cb(
    mqtt_client_t *c,
    void *arg,
    mqtt_connection_status_t status
);

/* --------------------------------------------------------
 * Wi-Fi status
 * -------------------------------------------------------- */

static uint8_t wifi_is_up(void)
{
    T_CYW43_UTK_STATUS status = {0};

    cyw43_utk_get_status(&status);

    return (
        status.link_status == CYW43_LINK_UP &&
        status.ip_addr != 0u
    ) ? 1u : 0u;
}

/* --------------------------------------------------------
 * FR2: Priority STOP handling
 * -------------------------------------------------------- */

/*
 * Network-task-only operation.
 *
 * A STOP is never placed in the normal queue.
 * This prevents CMD_BUSY from discarding STOP.
 */
static void fr2_post_priority_stop(uint8_t sequence)
{
    uint32_t word;

    fr2_stop_generation++;

    if (fr2_stop_generation == 0u ||
        fr2_stop_generation > 0xFFFFFFu)
    {
        fr2_stop_generation = 1u;
    }

    word = (fr2_stop_generation << 8) |
           (uint32_t)sequence;

    __atomic_store_n(
        &fr2_last_stop_word,
        word,
        __ATOMIC_RELEASE
    );

    __atomic_store_n(
        &fr2_stop_word,
        word,
        __ATOMIC_RELEASE
    );

    tm_printf((UB*)
        "[BUDDY1] FR2 PRIORITY STOP "
        "accepted seq=%u\n",
        (unsigned)sequence);
}

/*
 * Returns whether an ordinary queued command
 * was accepted before the latest STOP.
 */
static int fr2_command_before_stop(uint8_t sequence)
{
    uint32_t stop_word;
    uint8_t stop_sequence;
    uint8_t difference;

    stop_word = __atomic_load_n(
        &fr2_last_stop_word,
        __ATOMIC_ACQUIRE
    );

    if (stop_word == 0u)
    {
        return 0;
    }

    stop_sequence = (uint8_t)(stop_word & 0xFFu);

    difference = (uint8_t)(
        sequence - stop_sequence
    );

    /*
     * Equal or older sequence numbers
     * must not execute after STOP.
     */
    return (
        difference == 0u ||
        difference >= 128u
    ) ? 1 : 0;
}

/*
 * VehicleController calls this to retrieve
 * one validated command.
 *
 * Priority STOP is delivered first.
 *
 * Older queued START/RESET commands are skipped
 * after a newer STOP has been accepted.
 */
int b1_fr2_pop(B1_FR2_Command *output)
{
    uint32_t stop_word;
    uint32_t stop_ack;
    uint32_t head;
    uint32_t tail;

    if (output == 0)
    {
        return -1;
    }

    stop_word = __atomic_load_n(
        &fr2_stop_word,
        __ATOMIC_ACQUIRE
    );

    stop_ack = __atomic_load_n(
        &fr2_stop_ack,
        __ATOMIC_ACQUIRE
    );

    if (stop_word != 0u &&
        stop_word != stop_ack)
    {
        output->command = B1_FR2_STOP;
        output->sequence =
            (uint8_t)(stop_word & 0xFFu);

        __atomic_store_n(
            &fr2_stop_ack,
            stop_word,
            __ATOMIC_RELEASE
        );

        return 0;
    }

    /*
     * Read the normal SPSC queue.
     * Skip commands invalidated by STOP.
     */
    for (;;)
    {
        tail = __atomic_load_n(
            &fr2_tail,
            __ATOMIC_RELAXED
        );

        head = __atomic_load_n(
            &fr2_head,
            __ATOMIC_ACQUIRE
        );

        if (tail == head)
        {
            return -1;
        }

        *output = fr2_queue[
            tail % B1_FR2_QUEUE_SIZE
        ];

        __atomic_store_n(
            &fr2_tail,
            tail + 1u,
            __ATOMIC_RELEASE
        );

        if (!fr2_command_before_stop(
                output->sequence))
        {
            return 0;
        }
    }
}

uint32_t b1_fr2_pending(void)
{
    uint32_t head = __atomic_load_n(
        &fr2_head,
        __ATOMIC_ACQUIRE
    );

    uint32_t tail = __atomic_load_n(
        &fr2_tail,
        __ATOMIC_ACQUIRE
    );

    uint32_t stop = __atomic_load_n(
        &fr2_stop_word,
        __ATOMIC_ACQUIRE
    );

    uint32_t ack = __atomic_load_n(
        &fr2_stop_ack,
        __ATOMIC_ACQUIRE
    );

    return (head - tail) +
        ((stop != 0u && stop != ack) ? 1u : 0u);
}

static int fr2_enqueue(
    const B1_FR2_Command *command
)
{
    uint32_t head;
    uint32_t tail;

    if (command == 0)
    {
        return -1;
    }

    head = __atomic_load_n(
        &fr2_head,
        __ATOMIC_RELAXED
    );

    tail = __atomic_load_n(
        &fr2_tail,
        __ATOMIC_ACQUIRE
    );

    if ((head - tail) >= B1_FR2_QUEUE_SIZE)
    {
        return -1;
    }

    fr2_queue[head % B1_FR2_QUEUE_SIZE] =
        *command;

    __atomic_store_n(
        &fr2_head,
        head + 1u,
        __ATOMIC_RELEASE
    );

    return 0;
}

static void fr2_receive_command(
    const u8_t *payload,
    u16_t length
)
{
    B1_FR2_Command command;
    uint8_t difference;

    if (payload == 0 ||
        length != B1_FR2_MESSAGE_BYTES)
    {
        tm_printf((UB*)
            "[BUDDY1] FR2 invalid length\n");
        return;
    }

    if (payload[0] != B1_FR2_VERSION)
    {
        tm_printf((UB*)
            "[BUDDY1] FR2 invalid version: %u\n",
            (unsigned)payload[0]);
        return;
    }

    if (payload[1] != B1_FR2_START &&
        payload[1] != B1_FR2_STOP &&
        payload[1] != B1_FR2_RESET)
    {
        tm_printf((UB*)
            "[BUDDY1] FR2 invalid command: %u\n",
            (unsigned)payload[1]);
        return;
    }

    if (fr2_sequence_ready)
    {
        difference = (uint8_t)(
            payload[2] - fr2_last_sequence
        );

        if (difference == 0u ||
            difference >= 128u)
        {
            tm_printf((UB*)
                "[BUDDY1] FR2 duplicate/stale "
                "sequence rejected: %u\n",
                (unsigned)payload[2]);
            return;
        }
    }

    command.command = payload[1];
    command.sequence = payload[2];

    /*
     * STOP bypasses the regular four-slot queue.
     */
    if (command.command == B1_FR2_STOP)
    {
        fr2_post_priority_stop(command.sequence);
    }
    else
    {
        if (fr2_enqueue(&command) != 0)
        {
            tm_printf((UB*)
                "[BUDDY1] FR2 CMD_BUSY "
                "queue full; command rejected\n");
            return;
        }

        tm_printf((UB*)
            "[BUDDY1] FR2 queued command=%u seq=%u\n",
            (unsigned)command.command,
            (unsigned)command.sequence);
    }

    fr2_last_sequence = command.sequence;
    fr2_sequence_ready = 1u;
}

/* --------------------------------------------------------
 * CR1: Real robot data interface
 * -------------------------------------------------------- */

int b1_mqtt_set_robot_status(
    uint8_t controller_state,
    uint8_t fault_flags,
    uint8_t metric_0_100
)
{
    uint32_t encoded;

    if (controller_state > 7u ||
        metric_0_100 > 100u)
    {
        return -1;
    }

    encoded =
        ROBOT_STATUS_VALID |
        (uint32_t)controller_state |
        ((uint32_t)fault_flags << 3) |
        ((uint32_t)metric_0_100 << 11);

    __atomic_store_n(
        &robot_status_word,
        encoded,
        __ATOMIC_RELEASE
    );

    return 0;
}

int b1_mqtt_get_robot_status(
    B1_Telemetry *output
)
{
    uint32_t encoded;

    if (output == 0)
    {
        return -1;
    }

    encoded = __atomic_load_n(
        &robot_status_word,
        __ATOMIC_ACQUIRE
    );

    if ((encoded & ROBOT_STATUS_VALID) == 0u)
    {
        return -1;
    }

    output->controller_state =
        (uint8_t)(encoded & 7u);

    output->fault_flags =
        (uint8_t)((encoded >> 3) & 0xFFu);

    output->metric_0_100 =
        (uint8_t)((encoded >> 11) & 0x7Fu);

    output->wifi_connected = 0u;
    output->mqtt_connected = 0u;

    return 0;
}

/* --------------------------------------------------------
 * FR3: Communication safety state
 * -------------------------------------------------------- */

int b1_fr3_get_status(B1_FR3_Status *output)
{
    uint32_t word;

    if (output == 0)
    {
        return -1;
    }

    word = __atomic_load_n(
        &fr3_status_word,
        __ATOMIC_ACQUIRE
    );

    output->wifi_up =
        (word & STATUS_WIFI) != 0u;

    output->mqtt_up =
        (word & STATUS_MQTT) != 0u;

    output->healthy =
        (word & STATUS_HEALTHY) != 0u;

    output->link_lost_latched =
        (word & STATUS_LATCHED) != 0u;

    output->loss_reason = (uint8_t)(
        (word & STATUS_REASON_MASK) >>
        STATUS_REASON_SHIFT
    );

    output->safety_stop_required =
        !output->healthy ||
        output->link_lost_latched;

    return 0;
}

/*
 * VehicleController requests an authorized reset.
 *
 * This does NOT clear the latch immediately.
 * The network task must confirm healthy status.
 */
int b1_fr3_controller_reset(void)
{
    uint32_t word = __atomic_load_n(
        &fr3_status_word,
        __ATOMIC_ACQUIRE
    );

    if ((word & STATUS_HEALTHY) == 0u)
    {
        return -1;
    }

    __atomic_store_n(
        &fr3_reset_request,
        1u,
        __ATOMIC_RELEASE
    );

    return 0;
}

static void fr3_publish_status(
    uint8_t wifi_up,
    uint8_t mqtt_up,
    uint8_t healthy
)
{
    uint32_t word = 0;

    if (wifi_up)
    {
        word |= STATUS_WIFI;
    }

    if (mqtt_up)
    {
        word |= STATUS_MQTT;
    }

    if (healthy)
    {
        word |= STATUS_HEALTHY;
    }

    if (fr3_latched)
    {
        word |= STATUS_LATCHED;
    }

    word |= (
        ((uint32_t)fr3_loss_reason <<
            STATUS_REASON_SHIFT) &
        STATUS_REASON_MASK
    );

    __atomic_store_n(
        &fr3_status_word,
        word,
        __ATOMIC_RELEASE
    );
}

static void fr3_process_reset(uint8_t healthy)
{
    uint32_t requested;

    requested = __atomic_load_n(
        &fr3_reset_request,
        __ATOMIC_ACQUIRE
    );

    if (!requested)
    {
        return;
    }

    __atomic_store_n(
        &fr3_reset_request,
        0u,
        __ATOMIC_RELEASE
    );

    if (!healthy)
    {
        tm_printf((UB*)
            "[BUDDY1] FR3 reset rejected: "
            "communication unhealthy\n");
        return;
    }

    fr3_latched = 0u;
    fr3_loss_reason = B1_FR3_REASON_NONE;
    fr3_reported_reason = B1_FR3_REASON_NONE;

    __atomic_store_n(
        &fr3_reset_ack,
        1u,
        __ATOMIC_RELEASE
    );

    tm_printf((UB*)
        "[BUDDY1] FR3 controller reset accepted\n");
}

static void probe_done_cb(
    void *arg,
    err_t result
)
{
    (void)arg;

    fr3_probe_pending = 0;

    if (result == ERR_OK)
    {
        fr3_last_ack_us = time_us_64();
    }
    else
    {
        tm_printf((UB*)
            "[BUDDY1] FR3 probe failed: %d\n",
            (int)result);
    }
}

static void fr3_monitor_timeout(void *arg)
{
    uint64_t now;
    uint64_t loss_age;

    uint8_t wifi_up;
    uint8_t mqtt_up;
    uint8_t healthy;
    uint8_t reason;

    err_t result;

    (void)arg;

    now = time_us_64();
    wifi_up = wifi_is_up();

    mqtt_up = (
        wifi_up &&
        client != NULL &&
        mqtt_client_is_connected(client)
    ) ? 1u : 0u;

    if (mqtt_up &&
        !fr3_probe_pending &&
        (now - fr3_last_probe_us) >=
            (uint64_t)FR3_PROBE_MS * 1000ull)
    {
        const u8_t probe_payload = B1_FR2_VERSION;

        result = mqtt_publish(
            client,
            PROBE_TOPIC,
            &probe_payload,
            1,
            1,
            0,
            probe_done_cb,
            NULL
        );

        fr3_last_probe_us = now;

        if (result == ERR_OK)
        {
            fr3_probe_pending = 1;
        }
    }

    healthy = (
        wifi_up &&
        mqtt_up &&
        fr3_last_ack_us != 0ull &&
        (now - fr3_last_ack_us) <
            FR3_TIMEOUT_US
    ) ? 1u : 0u;

    reason = !wifi_up ?
        B1_FR3_REASON_WIFI :
        B1_FR3_REASON_MQTT;

    if (!healthy)
    {
        if (!fr3_loss_active)
        {
            fr3_loss_active = 1u;
            fr3_loss_start_us = now;

            tm_printf((UB*)
                "[BUDDY1] FR3 communication "
                "unhealthy; monitoring\n");
        }

        loss_age = now - fr3_loss_start_us;

        /*
         * A silent broker must be measured
         * from the last successful PUBACK.
         */
        if (reason == B1_FR3_REASON_MQTT &&
            wifi_up &&
            mqtt_up &&
            fr3_last_ack_us != 0ull)
        {
            loss_age = now - fr3_last_ack_us;
        }

        if (loss_age >= FR3_TIMEOUT_US)
        {
            fr3_latched = 1u;

            if (fr3_reported_reason != reason)
            {
                fr3_loss_reason = reason;
                fr3_reported_reason = reason;

                tm_printf((UB*)
                    "[BUDDY1] FR3 LINK_LOST "
                    "latched, reason=%u\n",
                    (unsigned)reason);

                tm_printf((UB*)
                    "[BUDDY1] FR3 VehicleController "
                    "STOP REQUIRED\n");
            }
        }
    }
    else if (fr3_loss_active)
    {
        fr3_loss_active = 0u;

        tm_printf((UB*)
            "[BUDDY1] FR3 communication restored\n");

        tm_printf((UB*)
            "[BUDDY1] FR3 recovery: "
            "latch=%u reason=%u\n",
            (unsigned)fr3_latched,
            (unsigned)fr3_loss_reason);
    }

    fr3_process_reset(healthy);

    fr3_publish_status(
        wifi_up,
        mqtt_up,
        healthy
    );

    sys_timeout(
        FR3_CHECK_MS,
        fr3_monitor_timeout,
        NULL
    );
}

static void fr3_monitor_start(void)
{
    if (fr3_monitor_started)
    {
        return;
    }

    fr3_monitor_started = 1u;
    fr3_last_ack_us = 0;
    fr3_last_probe_us = 0;

    sys_timeout(
        FR3_CHECK_MS,
        fr3_monitor_timeout,
        NULL
    );

    tm_printf((UB*)
        "[BUDDY1] FR3 monitor started\n");
}

/* --------------------------------------------------------
 * MQTT connection management
 * -------------------------------------------------------- */

static void schedule_retry(void)
{
    if (retry_pending)
    {
        return;
    }

    retry_pending = 1u;

    tm_printf((UB*)
        "[BUDDY1] MQTT retry scheduled in 3 seconds\n");

    sys_timeout(
        MQTT_RETRY_MS,
        retry_timeout,
        NULL
    );
}

static void retry_timeout(void *arg)
{
    (void)arg;

    retry_pending = 0u;
    connecting = 0u;

    mqtt_utk_start();
}

static void publish_done_cb(
    void *arg,
    err_t result
)
{
    (void)arg;

    if (result == ERR_OK)
    {
        tm_printf((UB*)
            "[BUDDY1] MQTT publish completed\n");
    }
    else
    {
        tm_printf((UB*)
            "[BUDDY1] MQTT publish failed: %d\n",
            (int)result);
    }
}

static void subscribe_done_cb(
    void *arg,
    err_t result
)
{
    const char *topic = (const char *)arg;

    if (result == ERR_OK)
    {
        tm_printf((UB*)
            "[BUDDY1] Subscribed to %s\n",
            topic);
    }
    else
    {
        tm_printf((UB*)
            "[BUDDY1] Subscription failed: "
            "%s, err=%d\n",
            topic,
            (int)result);
    }
}

/* --------------------------------------------------------
 * CR1 outgoing telemetry
 * -------------------------------------------------------- */

static void heartbeat_timeout(void *arg)
{
    B1_Telemetry telemetry = {0};
    B1_FR3_Status link = {0};
    u8_t payload[B1_TELEMETRY_BYTES];
    err_t result;

    static uint8_t waiting_reported = 0u;

    (void)arg;

    if (!wifi_is_up() ||
        client == NULL ||
        !mqtt_client_is_connected(client))
    {
        goto next_heartbeat;
    }

    /*
     * Never construct a fake controller snapshot.
     */
    if (b1_mqtt_get_robot_status(&telemetry) != 0)
    {
        if (!waiting_reported)
        {
            waiting_reported = 1u;

            tm_printf((UB*)
                "[BUDDY1] Robot telemetry awaiting "
                "VehicleController data\n");
        }

        goto next_heartbeat;
    }

    waiting_reported = 0u;

    if (b1_fr3_get_status(&link) != 0)
    {
        goto next_heartbeat;
    }

    telemetry.wifi_connected = link.wifi_up;
    telemetry.mqtt_connected = link.mqtt_up;

    if (buddy1_mqtt_prepare_telemetry(
            &telemetry,
            payload) != 0)
    {
        tm_printf((UB*)
            "[BUDDY1] Telemetry packing failed\n");

        goto next_heartbeat;
    }

    result = mqtt_publish(
        client,
        MQTT_TOPIC,
        payload,
        B1_TELEMETRY_BYTES,
        0,
        0,
        publish_done_cb,
        NULL
    );

    if (result == ERR_OK)
    {
        tm_printf((UB*)
            "[BUDDY1] TX live telemetry: "
            "%02X %02X %02X\n",
            (unsigned)payload[0],
            (unsigned)payload[1],
            (unsigned)payload[2]);
    }
    else
    {
        tm_printf((UB*)
            "[BUDDY1] MQTT publish queue error: %d\n",
            (int)result);
    }

next_heartbeat:

    sys_timeout(
        HEARTBEAT_MS,
        heartbeat_timeout,
        NULL
    );
}

/* --------------------------------------------------------
 * MQTT incoming messages
 * -------------------------------------------------------- */

static void incoming_publish_cb(
    void *arg,
    const char *topic,
    u32_t total_length
)
{
    u32_t i;

    (void)arg;

    for (i = 0;
         i < RX_TOPIC_MAX - 1u &&
         topic[i] != '\0';
         i++)
    {
        rx_topic[i] = topic[i];
    }

    rx_topic[i] = '\0';

    rx_len = 0;
    rx_truncated = 0;

    tm_printf((UB*)
        "[BUDDY1] MQTT incoming topic: "
        "%s, bytes=%u\n",
        rx_topic,
        (unsigned)total_length);
}

static void incoming_data_cb(
    void *arg,
    const u8_t *data,
    u16_t len,
    u8_t flags
)
{
    u16_t i;

    (void)arg;

    for (i = 0; i < len; i++)
    {
        if (rx_len < RX_DATA_MAX)
        {
            rx_data[rx_len++] = data[i];
        }
        else
        {
            rx_truncated = 1u;
        }
    }

    if ((flags & MQTT_DATA_FLAG_LAST) == 0)
    {
        return;
    }

    tm_printf((UB*)
        "[BUDDY1] MQTT received complete payload: "
        "topic=%s, bytes=%u\n",
        rx_topic,
        (unsigned)rx_len);

    if (rx_truncated)
    {
        tm_printf((UB*)
            "[BUDDY1] MQTT RX truncated; discarded\n");
        return;
    }

    if (strcmp(rx_topic, COMMAND_TOPIC) == 0)
    {
        if (rx_len == B1_MAPPING_BYTES)
        {
            B1_Mapping candidate = {0};

            if (b1_mapping_decode(
                    rx_data,
                    (uint32_t)rx_len,
                    &candidate) == 0)
            {
                uint32_t index;

                if (b1_mapping_store_latest(
                        &candidate) == 0)
                {
                    tm_printf((UB*)
                        "[BUDDY1] Shared mapping "
                        "stored successfully\n");

                    tm_printf((UB*)
                        "[BUDDY1] MQTT mapping accepted\n");

                    for (index = 0;
                         index < B1_MAPPING_COUNT;
                         index++)
                    {
                        tm_printf((UB*)
                            "[BUDDY1] Mapping %c -> %u\n",
                            candidate.entries[index].character,
                            (unsigned)
                                candidate.entries[index].movement);
                    }
                }
                else
                {
                    tm_printf((UB*)
                        "[BUDDY1] Mapping store failed\n");
                }
            }
            else
            {
                tm_printf((UB*)
                    "[BUDDY1] Invalid MQTT mapping rejected\n");
            }
        }
        else if (rx_len == B1_FR2_MESSAGE_BYTES)
        {
            fr2_receive_command(
                rx_data,
                rx_len
            );
        }
        else if (rx_len == B1_COMMAND_BYTES)
        {
            uint8_t command = 0;

            if (b1_command_unpack(
                    rx_data[0],
                    &command) == 0)
            {
                tm_printf((UB*)
                    "[BUDDY1] Legacy command=%u "
                    "(not forwarded)\n",
                    (unsigned)command);
            }
            else
            {
                tm_printf((UB*)
                    "[BUDDY1] Invalid legacy command\n");
            }
        }
        else
        {
            tm_printf((UB*)
                "[BUDDY1] Unsupported "
                "command length: %u\n",
                (unsigned)rx_len);
        }
    }
    else if (strcmp(rx_topic, MQTT_TOPIC) == 0)
    {
        B1_Telemetry decoded = {0};

        if (rx_len == B1_TELEMETRY_BYTES &&
            b1_telemetry_unpack(
                rx_data,
                &decoded) == 0)
        {
            tm_printf((UB*)
                "[BUDDY1] RX binary telemetry: "
                "state=%u wifi=%u mqtt=%u "
                "faults=%u metric=%u\n",
                (unsigned)decoded.controller_state,
                (unsigned)decoded.wifi_connected,
                (unsigned)decoded.mqtt_connected,
                (unsigned)decoded.fault_flags,
                (unsigned)decoded.metric_0_100);
        }
        else
        {
            tm_printf((UB*)
                "[BUDDY1] Invalid telemetry "
                "(length=%u)\n",
                (unsigned)rx_len);
        }
    }
    else
    {
        tm_printf((UB*)
            "[BUDDY1] Unknown MQTT topic: %s\n",
            rx_topic);
    }
}

static void connection_cb(
    mqtt_client_t *c,
    void *arg,
    mqtt_connection_status_t status
)
{
    err_t result;

    (void)c;
    (void)arg;

    connecting = 0u;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        retry_pending = 0u;

        sys_untimeout(
            retry_timeout,
            NULL
        );

        fr3_probe_pending = 0u;
        fr3_last_ack_us = time_us_64();
        fr3_last_probe_us = 0;

        tm_printf((UB*)
            "[BUDDY1] MQTT connected to "
            MQTT_BROKER_IP "\n");

        mqtt_set_inpub_callback(
            client,
            incoming_publish_cb,
            incoming_data_cb,
            NULL
        );

        result = mqtt_subscribe(
            client,
            MQTT_TOPIC,
            0,
            subscribe_done_cb,
            (void *)MQTT_TOPIC
        );

        if (result != ERR_OK)
        {
            tm_printf((UB*)
                "[BUDDY1] Telemetry subscribe "
                "failed: %d\n",
                (int)result);
        }

        result = mqtt_subscribe(
            client,
            COMMAND_TOPIC,
            0,
            subscribe_done_cb,
            (void *)COMMAND_TOPIC
        );

        if (result != ERR_OK)
        {
            tm_printf((UB*)
                "[BUDDY1] Command subscribe "
                "failed: %d\n",
                (int)result);
        }

        sys_untimeout(
            heartbeat_timeout,
            NULL
        );

        sys_timeout(
            HEARTBEAT_MS,
            heartbeat_timeout,
            NULL
        );
    }
    else
    {
        tm_printf((UB*)
            "[BUDDY1] MQTT disconnected, "
            "status=%d\n",
            (int)status);

        fr3_probe_pending = 0u;

        sys_untimeout(
            heartbeat_timeout,
            NULL
        );

        schedule_retry();
    }
}

/* --------------------------------------------------------
 * Test lines over MQTT (added 2026-11-04 by Buddy 3)
 *
 * MQTT builds.  tm_com.c keeps every console line that
 * starts with "[TEST]" or "[VC"; this timer publishes them on
 * TEST_LOG_TOPIC, so the test run can be watched with
 * mosquitto_sub while the car is unplugged.  QoS 0.  A line
 * that does not fit in lwIP's MQTT buffer yet stays queued
 * for the next tick.
 * -------------------------------------------------------- */

#define TEST_LOG_TOPIC "inf2004/robot/test"
#define TEST_LOG_MS 20u
#define TEST_LOG_BURST 8u
#define TEST_LOG_LINE_MAX 160
#define TEST_LOG_MAX_IN_FLIGHT 3u   /* of MQTT_REQ_MAX_IN_FLIGHT (8): the rest
                                       stays free for telemetry and the FR3
                                       probe, so test output can never starve
                                       the link monitor */

extern int tm_mqtt_log_peek(unsigned char *buf, int max);
extern void tm_mqtt_log_drop(void);
extern volatile unsigned long tm_mqtt_log_dropped;

static unsigned int test_log_in_flight = 0u;

static void test_log_sent_cb(void *arg, err_t result)
{
    (void)arg;
    (void)result;

    if (test_log_in_flight > 0u)
    {
        test_log_in_flight--;
    }
}

static err_t test_log_publish(const unsigned char *data, int len)
{
    err_t result;

    if (test_log_in_flight >= TEST_LOG_MAX_IN_FLIGHT)
    {
        return ERR_MEM;
    }

    result = mqtt_publish(client, TEST_LOG_TOPIC, data, (u16_t)len,
                          0, 0, test_log_sent_cb, NULL);

    if (result == ERR_OK)
    {
        test_log_in_flight++;
    }

    return result;
}

static void test_log_timeout(void *arg)
{
    static unsigned long dropped_sent = 0u;
    unsigned char line[TEST_LOG_LINE_MAX + 48];
    unsigned int n;
    int len;

    (void)arg;

    if (client == NULL ||
        !mqtt_client_is_connected(client))
    {
        /* a dropped connection never calls back for QoS 0 */
        test_log_in_flight = 0u;
    }
    else
    {
        if (tm_mqtt_log_dropped != dropped_sent)
        {
            int k = 0;
            unsigned long v = tm_mqtt_log_dropped;
            char digits[12];
            int d = 0;
            const char *head = "[TEST] (MQTT log: total lines dropped ";

            while (head[k] != '\0') { line[k] = (unsigned char)head[k]; k++; }
            do { digits[d++] = (char)('0' + (v % 10u)); v /= 10u; } while (v != 0u && d < 11);
            while (d > 0) { line[k++] = (unsigned char)digits[--d]; }
            line[k++] = ')';

            if (test_log_publish(line, k) == ERR_OK)
            {
                dropped_sent = tm_mqtt_log_dropped;
            }
        }

        for (n = 0u; n < TEST_LOG_BURST; n++)
        {
            len = tm_mqtt_log_peek(line, TEST_LOG_LINE_MAX);

            if (len <= 0)
            {
                break;
            }

            if (test_log_publish(line, len) != ERR_OK)
            {
                break;  /* buffer full: retry this line next tick */
            }

            tm_mqtt_log_drop();
        }
    }

    sys_timeout(TEST_LOG_MS, test_log_timeout, NULL);
}

void mqtt_utk_start(void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t result;

    fr3_monitor_start();

    {
        static u8_t test_log_started = 0u;

        if (!test_log_started)
        {
            test_log_started = 1u;
            sys_timeout(TEST_LOG_MS, test_log_timeout, NULL);
        }
    }

    if (connecting || retry_pending)
    {
        return;
    }

    if (client != NULL &&
        mqtt_client_is_connected(client))
    {
        return;
    }

    if (client == NULL)
    {
        client = mqtt_client_new();

        if (client == NULL)
        {
            tm_printf((UB*)
                "[BUDDY1] MQTT client allocation failed\n");

            schedule_retry();
            return;
        }
    }

    if (!ip4addr_aton(
            MQTT_BROKER_IP,
            ip_2_ip4(&broker_addr)))
    {
        tm_printf((UB*)
            "[BUDDY1] Invalid MQTT broker IP\n");

        schedule_retry();
        return;
    }

    IP_SET_TYPE(
        &broker_addr,
        IPADDR_TYPE_V4
    );

    client_info.client_id = MQTT_CLIENT_ID;
    client_info.keep_alive = MQTT_KEEPALIVE_S;

    connecting = 1u;

    tm_printf((UB*)
        "[BUDDY1] Connecting to MQTT broker "
        MQTT_BROKER_IP "...\n");

    result = mqtt_client_connect(
        client,
        &broker_addr,
        MQTT_BROKER_PORT,
        connection_cb,
        NULL,
        &client_info
    );

    if (result != ERR_OK)
    {
        connecting = 0u;

        tm_printf((UB*)
            "[BUDDY1] MQTT connect request "
            "failed: %d\n",
            (int)result);

        schedule_retry();
    }
}
