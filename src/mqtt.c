#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"

#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include "mqtt.h"

/*
 * Buddy 1 - MQTT Communication
 *
 * WiFi initialisation and connection are handled separately by wifi.c.
 * This module assumes that WiFi is already connected before mqtt_init()
 * is called.
 */

/* MQTT broker configuration. */
#define MQTT_BROKER_HOST "broker.emqx.io"
#define MQTT_BROKER_PORT       1883
#define MQTT_KEEP_ALIVE_S      60

/* Used to generate a unique MQTT client ID. */
#define MQTT_DEVICE_NAME       "inf2004-pico-"

/* Topic used to receive commands from the MQTT broker. */
#define MQTT_COMMAND_TOPIC     "inf2004/robot/command"

/* Maximum command payload we will store. */
#define MQTT_COMMAND_MAX_LEN   64

/*
 * MQTT connection state.
 */
static mqtt_client_t *mqtt_client = NULL;
static ip_addr_t mqtt_broker_address;

static volatile bool mqtt_connected = false;
static volatile bool mqtt_connection_finished = false;
/* Buffer used while receiving an MQTT command. */
static char mqtt_command_buffer[MQTT_COMMAND_MAX_LEN];
static u16_t mqtt_command_length = 0;

/**
 * @brief Called when subscription to the command topic completes.
 */
static void mqtt_subscribe_callback(void *arg, err_t result)
{
    (void)arg;

    if (result == ERR_OK)
    {
        printf(
            "[Buddy 1] Subscribed to command topic: %s\n",
            MQTT_COMMAND_TOPIC
        );
    }
    else
    {
        printf(
            "[Buddy 1] MQTT command subscription failed. Error: %d\n",
            (int)result
        );
    }
}


/**
 * @brief Called when a new MQTT message starts arriving.
 */
static void mqtt_incoming_publish_callback(
    void *arg,
    const char *topic,
    u32_t total_length)
{
    (void)arg;
    (void)total_length;

    /* Start a fresh command. */
    mqtt_command_length = 0;
    mqtt_command_buffer[0] = '\0';

    printf(
        "[Buddy 1] MQTT message received on topic: %s\n",
        topic
    );
}


/**
 * @brief Called when MQTT payload data is received.
 */
static void mqtt_incoming_data_callback(
    void *arg,
    const u8_t *data,
    u16_t len,
    u8_t flags)
{
    (void)arg;

    /*
     * Copy the received payload safely into our command buffer.
     */
    if (mqtt_command_length < (MQTT_COMMAND_MAX_LEN - 1U))
    {
        u16_t available =
            (u16_t)((MQTT_COMMAND_MAX_LEN - 1U) - mqtt_command_length);

        u16_t copy_length = len;

        if (copy_length > available)
        {
            copy_length = available;
        }

        memcpy(
            &mqtt_command_buffer[mqtt_command_length],
            data,
            copy_length
        );

        mqtt_command_length =
            (u16_t)(mqtt_command_length + copy_length);

        mqtt_command_buffer[mqtt_command_length] = '\0';
    }

    /*
     * Print the command after the complete MQTT payload has arrived.
     */
    if ((flags & MQTT_DATA_FLAG_LAST) != 0U)
    {
        printf(
            "[Buddy 1] MQTT command received: %s\n",
            mqtt_command_buffer
        );
    }
}

/**
 * @brief Callback executed when the MQTT connection status changes.
 */
static void mqtt_connection_callback(
    mqtt_client_t *client,
    void *arg,
    mqtt_connection_status_t status)
{
    (void)arg;

    mqtt_connection_finished = true;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        err_t subscribe_result;

        mqtt_connected = true;

        printf("[Buddy 1] MQTT connected successfully!\n");

        /*
         * Tell lwIP which functions should handle incoming
         * MQTT messages and payload data.
         */
        mqtt_set_inpub_callback(
            client,
            mqtt_incoming_publish_callback,
            mqtt_incoming_data_callback,
            NULL
        );

        /*
         * Subscribe to the robot command topic.
         */
        subscribe_result = mqtt_subscribe(
            client,
            MQTT_COMMAND_TOPIC,
            0,                  /* QoS 0 */
            mqtt_subscribe_callback,
            NULL
        );

        if (subscribe_result != ERR_OK)
        {
            printf(
                "[Buddy 1] Failed to start command subscription. Error: %d\n",
                (int)subscribe_result
            );
        }
    }
    else
{
    mqtt_connected = false;

    printf(
        "[Buddy 1] MQTT disconnected. Status: %d\n",
        (int)status
    );
}
}

/**
 * @brief Callback executed after an MQTT publish attempt completes.
 */
static void mqtt_publish_callback(void *arg, err_t result)
{
    (void)arg;

    if (result == ERR_OK)
    {
        printf("[Buddy 1] MQTT message published successfully!\n");
    }
    else
    {
        printf(
            "[Buddy 1] MQTT publish failed. Error: %d\n",
            (int)result
        );
    }
}


/**
 * @brief Start the connection to the MQTT broker.
 */
static int mqtt_start_connection(void)
{
    struct mqtt_connect_client_info_t client_info;
    pico_unique_board_id_t board_id;

    static char client_id[40];

    err_t result;

    /*
     * Create a unique client ID so multiple Pico W boards do not use
     * exactly the same MQTT client name.
     */
    pico_get_unique_board_id(&board_id);

    snprintf(
        client_id,
        sizeof(client_id),
        MQTT_DEVICE_NAME "%02x%02x%02x%02x",
        board_id.id[0],
        board_id.id[1],
        board_id.id[2],
        board_id.id[3]
    );

    printf("[Buddy 1] MQTT client ID: %s\n", client_id);

    /*
     * Initialise all MQTT connection information to zero first.
     */
    memset(&client_info, 0, sizeof(client_info));

    client_info.client_id = client_id;
    client_info.keep_alive = MQTT_KEEP_ALIVE_S;

    /*
     * Public test broker does not require username/password.
     */
    client_info.client_user = NULL;
    client_info.client_pass = NULL;

/*
 * Create the MQTT client only once.
 * During reconnection we reuse the existing client instead of
 * allocating a new one every time.
 */
if (mqtt_client == NULL)
{
    cyw43_arch_lwip_begin();

    mqtt_client = mqtt_client_new();

    cyw43_arch_lwip_end();

    if (mqtt_client == NULL)
    {
        printf("[Buddy 1] Failed to create MQTT client.\n");
        return -1;
    }
}

    printf(
        "[Buddy 1] Connecting to MQTT broker at %s:%d...\n",
        ipaddr_ntoa(&mqtt_broker_address),
        MQTT_BROKER_PORT
    );

    mqtt_connected = false;
    mqtt_connection_finished = false;

    cyw43_arch_lwip_begin();

    result = mqtt_client_connect(
        mqtt_client,
        &mqtt_broker_address,
        MQTT_BROKER_PORT,
        mqtt_connection_callback,
        NULL,
        &client_info
    );

    cyw43_arch_lwip_end();

    if (result != ERR_OK)
    {
        printf(
            "[Buddy 1] Failed to start MQTT connection: %d\n",
            result
        );

        return (int)result;
    }

    return 0;
}




/**
 * @brief DNS callback used when the broker hostname has been resolved.
 */
static void mqtt_dns_callback(
    const char *hostname,
    const ip_addr_t *ipaddr,
    void *arg)
{
    (void)hostname;
    (void)arg;

    if (ipaddr == NULL)
    {
        printf("[Buddy 1] MQTT broker DNS lookup failed.\n");

        mqtt_connection_finished = true;
        mqtt_connected = false;

        return;
    }

    mqtt_broker_address = *ipaddr;

    printf(
        "[Buddy 1] MQTT broker resolved to %s\n",
        ipaddr_ntoa(&mqtt_broker_address)
    );

    if (mqtt_start_connection() != 0)
    {
        mqtt_connection_finished = true;
        mqtt_connected = false;
    }
}


/**
 * @brief Initialise MQTT and begin connecting to the broker.
 *
 * WiFi must already be connected before this function is called.
 */
int mqtt_init(void)
{
    err_t result;

    printf("\n[Buddy 1] MQTT connection test\n");
    printf(
        "[Buddy 1] Resolving broker: %s\n",
        MQTT_BROKER_HOST
    );

    mqtt_connected = false;
    mqtt_connection_finished = false;

    /*
     * Resolve the broker hostname.
     */
    cyw43_arch_lwip_begin();

    result = dns_gethostbyname(
        MQTT_BROKER_HOST,
        &mqtt_broker_address,
        mqtt_dns_callback,
        NULL
    );

    cyw43_arch_lwip_end();

    /*
     * ERR_OK means the address was already available, normally because
     * it was found in the DNS cache.
     */
    if (result == ERR_OK)
    {
        printf(
            "[Buddy 1] MQTT broker resolved to %s\n",
            ipaddr_ntoa(&mqtt_broker_address)
        );

        return mqtt_start_connection();
    }

    /*
     * ERR_INPROGRESS means DNS resolution is happening asynchronously.
     * mqtt_dns_callback() will continue the connection process.
     */
    if (result == ERR_INPROGRESS)
    {
        printf("[Buddy 1] Waiting for DNS response...\n");
        return 0;
    }

    printf(
        "[Buddy 1] Failed to start DNS lookup: %d\n",
        result
    );

    mqtt_connection_finished = true;

    return (int)result;
}


/**
 * @brief Check whether the MQTT client is connected.
 */
int mqtt_is_connected(void)
{
    return mqtt_connected ? 1 : 0;
}


/**
 * @brief Check whether the MQTT connection attempt has completed.
 */
int mqtt_connection_complete(void)
{
    return mqtt_connection_finished ? 1 : 0;
}

/**
 * @brief Publish a message to an MQTT topic.
 *
 * @param topic   MQTT topic to publish to.
 * @param message Message payload to send.
 *
 * @return 0 if the publish request was accepted,
 *         non-zero if an error occurred.
 */
int mqtt_publish_message(const char *topic, const char *message)
{
    err_t result;

    if (topic == NULL || message == NULL)
    {
        printf("[Buddy 1] MQTT publish failed: invalid topic/message.\n");
        return -1;
    }

    if (mqtt_client == NULL || !mqtt_connected)
    {
        printf("[Buddy 1] MQTT publish failed: client not connected.\n");
        return -1;
    }

    printf(
        "[Buddy 1] Publishing \"%s\" to \"%s\"...\n",
        message,
        topic
    );

    cyw43_arch_lwip_begin();

    result = mqtt_publish(
        mqtt_client,
        topic,
        message,
        (u16_t)strlen(message),
        0,                      /* QoS 0 */
        0,                      /* Retain = false */
        mqtt_publish_callback,
        NULL
    );

    cyw43_arch_lwip_end();

    if (result != ERR_OK)
    {
        printf(
            "[Buddy 1] MQTT publish request failed. Error: %d\n",
            (int)result
        );

        return (int)result;
    }

    return 0;
}

/**
 * @brief Attempt to reconnect to the MQTT broker.
 *
 * WiFi must already be connected before this function is called.
 *
 * @return 0 if already connected or if the reconnection attempt
 *         starts successfully, non-zero otherwise.
 */
int mqtt_reconnect(void)
{
    if (mqtt_connected)
    {
        return 0;
    }

    /*
     * Do not start another connection while an existing
     * connection attempt is still in progress.
     */
    if (!mqtt_connection_finished)
    {
        return 0;
    }

    printf("[Buddy 1] Attempting MQTT reconnection...\n");

    return mqtt_init();
}