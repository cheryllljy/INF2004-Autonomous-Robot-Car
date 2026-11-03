
/*
 * Buddy 1 - Wi-Fi and MQTT configuration
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

/* Wi-Fi credentials */
#define WIFI_SSID     "Your_WiFi_SSID"
#define WIFI_PASSWORD "Your_WiFi_Password"

/* MQTT configuration */
#define MQTT_BROKER_IP   "Your Laptop IP Address" /*IpConfig after connecting laptop to wifi.Both car/laptop must be on same wifi*/
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "inf2004-buddy1"
#define MQTT_TOPIC       "inf2004/robot/telemetry"


#endif /* WIFI_SECRETS_H */
