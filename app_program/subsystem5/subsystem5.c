/*
 *      subsystem5.c
 *      Subsystem 5 -- start-up and the S5 task. See
 *      subsystem5.h.
 */

#include "subsystem5.h"

#include "s5_cal.h"
#include "s5_config.h"
#include "s5_hw.h"
#include "s5_port.h"
#include "s5_scan.h"
#include "s5_sensor.h"

static ID g_s5_tid;

/* The S5 task is woken every S5_POLL_MS by this cyclic handler.
 * tk_dly_tsk() after each poll would wait for the second 10 ms tick (the
 * kernel adds a tick to every relative wait), so polls would be 20 ms
 * apart. */
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

#if (0 != S5_AUTO_TRIM) && !S5_CAL_BUILD
/* one standing sweep, as the profile; false: it did not finish */
static bool s5_trim_sweep(s5_profile_t *p_pr)
{
    static s5_sweep_t sw;
    const s5_view_t view = { 0.0f, 0.0f, 0.0f };
    const uint32_t seq = s5_sensor_scan();
    const uint32_t t0 = s5_hw_us();

    while (!s5_sensor_scan_get(seq, &sw))
    {
        if (s5_trim_aborted() ||
            ((s5_hw_us() - t0) > ((uint32_t)S5_SCAN_TIMEOUT_MS * 1000U)))
        {
            return false;
        }
        s5_sensor_poll();
        s5_poll_wait();
    }
    s5_scan_profile(sw.s, sw.n, &view, p_pr);
    return true;
}

/* S5_AUTO_TRIM: an object straight ahead at power-up corrects the servo
 * trim for this run */
static void s5_auto_trim(void)
{
    static s5_profile_t pr;
    const uint32_t t0 = s5_hw_us();
    int16_t c1 = 0;
    int16_t c2 = 0;
    bool b_ok;

    s5_sensor_set_trimming(true);
    while (((s5_hw_us() - t0) < ((uint32_t)S5_AUTO_TRIM_WAIT_MS * 1000U)) &&
           !s5_trim_aborted())
    {
        s5_sensor_poll();
        s5_poll_wait();
    }
    b_ok = s5_trim_sweep(&pr) && s5_scan_trim_from(&pr, &c1) &&
           s5_trim_sweep(&pr) && s5_scan_trim_from(&pr, &c2);
    /* the two at the same trim: then the correction (applied once, after) */
    s5_sensor_guard();
    if (s5_trim_aborted())
    {
        S5_LOG("[VC-S5] auto trim: line following started first, skipped: "
               "keeping S5_SERVO_TRIM_DDEG %d\n",
               (int)S5_SERVO_TRIM_DDEG);
        b_ok = false;
        c1 = 0;
        c2 = 0x7FFF; /* not taken */
    }
    else if (b_ok && ((c1 - c2) <= S5_AUTO_TRIM_AGREE_DDEG) &&
        ((c2 - c1) <= S5_AUTO_TRIM_AGREE_DDEG))
    {
        const int32_t c = ((int32_t)c1 + (int32_t)c2) / 2;

        s5_hw_trim_adj(c);
        S5_LOG("[VC-S5] auto trim: the object ahead (%d mm) reads %d (0.1 "
               "deg, + = left) off centre: servo trim %d -> %d for this "
               "run\n",
               (int)pr.face_mm, (int)c, (int)S5_SERVO_TRIM_DDEG,
               (int)s5_hw_trim_now());
    }
    else if (b_ok)
    {
        S5_LOG("[VC-S5] auto trim: the two sweeps disagree (%d / %d, 0.1 "
               "deg): keeping S5_SERVO_TRIM_DDEG %d\n",
               (int)c1, (int)c2, (int)S5_SERVO_TRIM_DDEG);
    }
    else if (0x7FFF != c2)
    {
        S5_LOG("[VC-S5] auto trim: no object straight ahead %d..%d mm (or an "
               "edge past the sweep): keeping S5_SERVO_TRIM_DDEG %d\n",
               (int)S5_AUTO_TRIM_NEAR_MM, (int)S5_AUTO_TRIM_FAR_MM,
               (int)S5_SERVO_TRIM_DDEG);
    }
    s5_sensor_set_trimming(false);
}
#endif

static void s5_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

#if S5_CAL_BUILD
    s5_cal_run(); /* never returns */
#endif
#if (0 != S5_AUTO_TRIM) && !S5_CAL_BUILD
    s5_auto_trim();
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
           (int)S5_STOP_MM,
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
