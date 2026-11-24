/*
 *      subsystem4.c
 *      Subsystem 4 -- IMU and monitor: creates objects and starts its tasks.
 */

#include "subsystem4.h"

ID mbf_summary; /* IF-03 */
ID flg_events;  /* IF-03 */
ID alm_monitor;
ID mtx_imu;

static s4_imu_snapshot_t s4_imu_snapshot;

ER subsystem4_init(void)
{
    T_CFLG cflg = { 0 };
    T_CMBF cmbf = { 0 };
    T_CALM calm = { 0 };
    T_CMTX cmtx = { 0 };

    cmtx.mtxatr = TA_INHERIT;
    mtx_imu = tk_cre_mtx(&cmtx);
    if (mtx_imu < E_OK)
    {
        return mtx_imu;
    }

    cflg.flgatr = TA_TFIFO | TA_WMUL; /* allow several waiters */
    cflg.iflgptn = 0;
    flg_events = tk_cre_flg(&cflg);
    if (flg_events < E_OK)
    {
        return flg_events;
    }

    /* Message buffer: short text lines from subsystem 3 (copied). */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = 4 * S4_SUMMARY_MAX; /* total buffer size, bytes */
    cmbf.maxmsz = S4_SUMMARY_MAX;    /* largest single message */
    mbf_summary = tk_cre_mbf(&cmbf);
    if (mbf_summary < E_OK)
    {
        return mbf_summary;
    }

    /* One-shot alarm; the monitor task re-arms it on every loop. */
    calm.almatr = TA_HLNG;
    calm.almhdr = (FP)monitor_alarm_handler;
    alm_monitor = tk_cre_alm(&calm);
    return (alm_monitor < E_OK) ? alm_monitor : E_OK;
}

ER subsystem4_publish_imu_snapshot(const s4_imu_snapshot_t *p_snapshot)
{
    ER er;

    if (NULL == p_snapshot)
    {
        return E_PAR;
    }
    er = tk_loc_mtx(mtx_imu, TMO_FEVR);
    if (er < E_OK)
    {
        return er;
    }
    s4_imu_snapshot = *p_snapshot;
    return tk_unl_mtx(mtx_imu);
}

ER subsystem4_get_imu_snapshot(s4_imu_snapshot_t *p_snapshot)
{
    ER er;

    if (NULL == p_snapshot)
    {
        return E_PAR;
    }
    er = tk_loc_mtx(mtx_imu, TMO_FEVR);
    if (er < E_OK)
    {
        return er;
    }
    *p_snapshot = s4_imu_snapshot;
    return tk_unl_mtx(mtx_imu);
}

ER subsystem4_start(void)
{
    ID tid = app_start_task(imu_task, S4_PRI_IMU, 0, "imu_task");
    if (tid < E_OK)
    {
        return tid;
    }
#if APP_TEST
    /* template example, kept for its tests only */
    tid = app_start_task(monitor_task, S4_PRI_MONITOR, 0, "monitor_task");
#endif
    return (tid < E_OK) ? tid : E_OK;
}
