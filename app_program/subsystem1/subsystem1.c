
/*
 * subsystem1.c
 *
 * INF2004 Autonomous Robot Car
 * Subsystem 1 - Buddy 1
 *
 * Responsibilities:
 * - Initialize shared barcode mapping storage
 *
 * Wi-Fi and MQTT networking are handled by
 * the existing cyw43_utk_task.
 */

#include "subsystem1.h"
#include "buddy1_mapping.h"

/*
 * Initialize Subsystem 1.
 */
ER subsystem1_init(void)
{
    if (b1_mapping_store_init() != 0)
    {
        tm_printf((UB *)"[BUDDY1] Mapping store initialization failed\n");

        return E_SYS;
    }

    tm_printf((UB *)"[BUDDY1] Mapping store initialized\n");

    return E_OK;
}

/*
 * Start Subsystem 1.
 *
 * Wi-Fi and MQTT run independently inside
 * the network-owning task.
 */
ER subsystem1_start(void)
{
    return E_OK;
}
