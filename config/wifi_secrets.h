
/*
 * Buddy 1 - Wi-Fi and MQTT configuration
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

// /* Wi-Fi credentials */
// #define WIFI_SSID     "Chloeee"
// #define WIFI_PASSWORD "Chloeee27~21"

/* Wi-Fi credentials */
#define WIFI_SSID     "Ice Bear"
#define WIFI_PASSWORD "icebear123"

/* MQTT configuration */
// #define MQTT_BROKER_IP   "10.190.182.12" /*IpConfig after connecting laptop to wifi.Both car/laptop must be on same wifi*/
#define MQTT_BROKER_IP   "172.20.10.12"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "inf2004-buddy1"
#define MQTT_TOPIC       "inf2004/robot/telemetry"


#endif /* WIFI_SECRETS_H */
