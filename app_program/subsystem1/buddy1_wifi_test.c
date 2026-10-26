
/*
 * buddy1_wifi_test.c
 *
 * Buddy 1 - Wi-Fi integration test
 * Uses the team's micro T-Kernel Wi-Fi service.
 */

#include "subsystem1.h"
#include "cyw43_utk.h"

#if APP_TEST && TM_NET_LWIP

/*
 * Return 1 when Wi-Fi is connected
 * and an IP address has been assigned.
 */
EXPORT INT buddy1_wifi_test(void)
{
    T_CYW43_UTK_STATUS status = {0};
    INT attempt;

    tm_printf((UB*)
        "\n[BUDDY1] Starting Wi-Fi integration test...\n");

    /* Wait up to approximately 30 seconds. */
    
    for (attempt = 0; attempt < 90; attempt++)
    {
        cyw43_utk_get_status(&status);

        if (status.init_complete &&
            status.ready &&
            status.ip_addr != 0)

        {
            tm_printf((UB*)
                "[BUDDY1] PASS: Wi-Fi connected\n");

            tm_printf((UB*)
                "[BUDDY1] PASS: IP address assigned\n");

            return 1;
        }

        tk_dly_tsk(1000);
    }

    tm_printf((UB*)
        "[BUDDY1] FAIL: Wi-Fi connection unavailable\n");

    tm_printf((UB*)
        "[BUDDY1] Link status: %d\n",
        (int)status.link_status);

    return 0;
}

#endif
