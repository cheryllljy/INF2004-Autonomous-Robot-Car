#ifndef MQTT_H
#define MQTT_H

/**
 * @brief Begin the MQTT broker connection.
 *
 * WiFi must already be connected before this function is called.
 *
 * @return 0 if the connection process starts successfully,
 *         non-zero if an immediate error occurs.
 */
int mqtt_init(void);
int mqtt_reconnect(void);

/**
 * @brief Check whether MQTT is connected.
 *
 * @return 1 if connected, 0 otherwise.
 */
int mqtt_is_connected(void);

/**
 * @brief Check whether the current MQTT connection attempt has completed.
 *
 * @return 1 if completed, 0 if still waiting.
 */
int mqtt_connection_complete(void);

int mqtt_publish_message(
    const char *topic,
    const char *message
);




#endif /* MQTT_H */