
/*
 * Buddy 1 - Wi-Fi and MQTT configuration
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

/* Wi-Fi credentials */
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

/* MQTT configuration */
#define MQTT_BROKER_IP   "192.168.1.36"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"

#endif /* WIFI_SECRETS_H */
