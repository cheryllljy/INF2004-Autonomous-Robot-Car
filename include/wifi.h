#ifndef WIFI_H
#define WIFI_H

/**
 * @brief Initialise the Pico W WiFi hardware.
 *
 * @return 0 if successful, non-zero if initialisation fails.
 */
int wifi_init(void);

/**
 * @brief Connect the Pico W to a WiFi network.
 *
 * @param ssid Chloeee
 * @param password Chloeee27~21
 *
 * @return 0 if connected successfully, non-zero otherwise.
 */
int wifi_connect(const char *ssid, const char *password);

/**
 * @brief Check whether the Pico W is currently connected to WiFi.
 *
 * @return 1 if connected, 0 otherwise.
 */
int wifi_is_connected(void);

/**
 * @brief Shut down the WiFi subsystem.
 */
void wifi_deinit(void);

#endif /* WIFI_H */