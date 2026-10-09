#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "wifi.h"


int wifi_init(void)
{
    int result;

    printf("Initialising WiFi...\n");

    result = cyw43_arch_init();

    if (result != 0)
    {
        printf("WiFi initialisation failed: %d\n", result);
        return result;
    }

    
    cyw43_arch_enable_sta_mode();

    printf("WiFi initialised successfully.\n");

    return 0;
}

int wifi_is_connected(void)
{
    int link_status = cyw43_tcpip_link_status(
        &cyw43_state,
        CYW43_ITF_STA
    );

    return (link_status == CYW43_LINK_UP) ? 1 : 0;
}

int wifi_connect(const char *ssid, const char *password)
{
    int result;

    printf("Connecting to WiFi: %s\n", ssid);

    result = cyw43_arch_wifi_connect_timeout_ms(
        ssid,
        password,
        CYW43_AUTH_WPA2_AES_PSK,
        30000
    );

    if (result != 0)
    {
        printf("WiFi connection failed: %d\n", result);
        return result;
    }

    printf("WiFi connected successfully!\n");

    return 0;
}


void wifi_deinit(void)
{
    cyw43_arch_deinit();

    printf("WiFi deinitialised.\n");
}