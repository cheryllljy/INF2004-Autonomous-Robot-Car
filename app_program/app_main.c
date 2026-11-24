/*
 *      app_main.c
 *      usermain() is the program's entry point, called from the kernel's
 *      initial task at priority 1.
 *
 *      On the dual-core build the other core may start a new task at once,
 *      so usermain() works in two phases:
 *        1. every subsystem creates its kernel objects  (subsystemN_init)
 *        2. every subsystem starts its tasks             (subsystemN_start)
 *      and then puts the initial task to sleep forever.
 *      If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *      Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "vehicle_controller/vehicle_controller.h"
#if defined(APP_MQTT) && APP_MQTT && APP_TEST
/* b1_fr3_get_status */
#include "../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
#endif

#ifndef APP_TEST_MQTT_WAIT_S
/* MQTT test build: wait this long for the broker */
#define APP_TEST_MQTT_WAIT_S 30
#endif
#ifndef APP_TEST_START_DELAY_S
#define APP_TEST_START_DELAY_S 5 /* seconds before the test run starts */
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
    T_CTSK ctsk = { 0 };
    ID tid;
    ER er;

    ctsk.task = entry;
    ctsk.itskpri = pri;           /* initial priority (1 = highest) */
    ctsk.stksz = TASK_STACK_SIZE; /* kernel allocates the stack */
    ctsk.tskatr = TA_HLNG | TA_RNG3; /* runs at user protection level */
#if TK_SUPPORT_SMP
    if (0 != prc)
    {
        ctsk.tskatr |= TA_ASSPRC; /* honour the affinity below */
        ctsk.assprc = prc;
    }
#else
    (void)prc; /* single-core build: no affinity */
#endif

    /* The new task starts in the DORMANT state. */
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, name);
        return tid;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    er = tk_sta_tsk(tid, 0);
    CHECK(er, name);
    return tid;
}

INT usermain(void)
{
    T_RVER ver;

    tk_ref_ver(&ver);
    tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                    " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
    tm_printf((UB *)"single-core build\n");
#endif

    /* Phase 1: every subsystem creates its kernel objects */
    /* S1: barcode mapping store, used by the VC */
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem5_init(), "subsystem5_init");
#if VC_ENABLE
    CHECK(vc_init(), "vc_init"); /* after the subsystems it reads */
#endif

    /* Phase 2: every subsystem starts its tasks */
    CHECK(subsystem1_start(), "subsystem1_start");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem5_start(), "subsystem5_start");
#if VC_ENABLE
    CHECK(vc_start(), "vc_start");
#endif

    tm_printf((UB *)"all subsystems started\n\n");

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn while
       the system is running.  Tests may call tk_dly_tsk(). */
#if defined(APP_MQTT) && APP_MQTT
    /* [TEST] lines are also published on inf2004/robot/test.  Wait for
       the broker so the start of the run is not missed; after
       APP_TEST_MQTT_WAIT_S run anyway (lines are kept until it connects). */
    {
        B1_FR3_Status link;
        INT s;

        for (s = 0; s < APP_TEST_MQTT_WAIT_S; s++)
        {
            if (b1_fr3_get_status(&link) == 0 && link.mqtt_up)
            {
                break;
            }
            tm_printf((UB *)"[TEST] waiting for MQTT broker (%d s)\n", s);
            tk_dly_tsk(1000);
        }
        tm_printf((UB *)"[TEST] MQTT %s\n",
                  (s < APP_TEST_MQTT_WAIT_S)
                      ? "connected: test lines go to inf2004/robot/test"
                      : "not connected yet: running anyway");
    }
#endif
    /* Give time to open the serial monitor before the first result */
    {
        INT n;

        for (n = APP_TEST_START_DELAY_S; n > 0; n--)
        {
            tm_printf((UB *)"[TEST] starting in %d s\n", n);
            tk_dly_tsk(1000);
        }
    }
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
