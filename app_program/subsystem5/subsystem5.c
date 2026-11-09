/*
 *      subsystem5.c
 *      Subsystem 5 -- start-up and the S5 task (nyx, 2026-11-07). See
 *      subsystem5.h.
 */

#include "subsystem5.h"

#include "s5_cal.h"
#include "s5_config.h"
#include "s5_hw.h"
#include "s5_port.h"
#include "s5_sensor.h"

static ID g_s5_tid;

/* nyx, 2026-11-07 (review A1): the S5 task is woken every S5_POLL_MS by
 * this cyclic handler, like Buddy 3's. tk_dly_tsk() after each poll
 * would wait for the second 10 ms tick (the kernel adds a tick to every
 * relative wait), so the polls would be 20 ms apart. */
static void s5_cyc(void *p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_s5_tid);
}

void s5_poll_wait(void)
{
    /* the timeout keeps the polls coming (20 ms apart) without the
     * cyclic handler */
    (void)tk_slp_tsk(S5_POLL_MS);
}

static void s5_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

#if S5_CAL_BUILD
    s5_cal_run(); /* never returns */
#endif
    for (;;)
    {
        s5_sensor_poll();
        s5_poll_wait();
    }
}

ER subsystem5_init(void)
{
    (void)s5_hw_init();
    s5_sensor_init();
    s5_avoid_init();
    S5_LOG("[VC-S5] ready: HC-SR04P TRIG GP%d ECHO GP%d, servo GP%d, "
           "stops for objects within %d mm%s\n",
           (int)S5_TRIG_GPIO, (int)S5_ECHO_GPIO, (int)S5_SERVO_GPIO,
           (int)S5_DETECT_MM,
           (0 != S5_CAL_BUILD) ? " (CALIBRATION BUILD)" : "");
    return E_OK;
}

ER subsystem5_start(void)
{
    const ID tid = app_start_task(s5_task, S5_TASK_PRI, 0U, "s5_task");
    T_CCYC cyc = (T_CCYC){ 0 };
    ID cid;

    if (tid < E_OK)
    {
        return tid;
    }
    g_s5_tid = tid;
    cyc.cycatr = TA_HLNG | TA_STA;
    cyc.cychdr = (FP)s5_cyc;
    cyc.cyctim = S5_POLL_MS;
    cyc.cycphs = S5_POLL_MS;
    cid = tk_cre_cyc(&cyc);
    if (cid < E_OK)
    {
        S5_LOG("[VC-S5] no cyclic handler (error %d): polling every 20 ms "
               "instead of %d\n",
               (int)cid, (int)S5_POLL_MS);
    }
    return E_OK;
}
