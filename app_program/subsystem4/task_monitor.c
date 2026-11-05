/*
 *	task_monitor.c
 *	Subsystem 4 -- the monitor task and its alarm handler.
 *
 *	APIs shown: tk_sta_alm, tk_stp_alm, alarm handler, tk_wai_flg,
 *	            tk_rcv_mbf, tk_loc_mtx/tk_unl_mtx, tk_ref_sem, tk_ref_mpf,
 *	            tk_ref_tsk, tk_sus_tsk, tk_rsm_tsk, tk_chg_pri,
 *	            tk_get_tid, tk_get_tim
 */

#include "subsystem4.h"
#include "../subsystem2/subsystem2.h"	/* sem_slots, mpf_samples (read only) */
#include "../subsystem3/subsystem3.h"	/* IF-04: app_stats, mtx_stats */

EXPORT UW	s4_lines_received;

/*
 * Alarm handler: one-shot, interrupt context (same rules as a cyclic
 * handler: keep it short, never wait).
 */
EXPORT void monitor_alarm_handler(void *exinf)
{
	(void)exinf;
	tk_set_flg(flg_events, FLG_ALARM);
}

LOCAL ER publish_imu_sample(const imu_vector_t *p_accel,
			    imu_status_t accel_status,
			    const imu_vector_t *p_mag,
			    imu_status_t mag_status)
{
	s4_imu_snapshot_t snapshot = {0};
	SYSTIM now;
	ER er;

	er = tk_get_tim(&now);
	if (er < E_OK) {
		tm_printf((UB*)"monitor: failed to timestamp IMU sample, error=%d\n",
			  (INT)er);
		return er;
	}
	if (p_accel != NULL) {
		snapshot.accel = *p_accel;
	}
	if (p_mag != NULL) {
		snapshot.mag = *p_mag;
	}
	snapshot.accel_status = accel_status;
	snapshot.mag_status = mag_status;
	snapshot.timestamp_ms = now.lo;
	snapshot.has_sample = true;

	er = subsystem4_publish_imu_snapshot(&snapshot);
	if (er < E_OK) {
		tm_printf((UB*)"monitor: failed to publish IMU snapshot, error=%d\n",
			  (INT)er);
	}
	return er;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	imu_vector_t accel = {0};
	imu_vector_t mag = {0};
	imu_status_t init_status;
	imu_status_t accel_status;
	imu_status_t mag_status;
	INT consecutive_fails = 0;

	(void)stacd;
	(void)exinf;

#if APP_TEST
	/* Keep I2C idle while the hardware barcode tests are running. */
	tk_dly_tsk(30000);
#endif

	init_status = imu_init();
	if (init_status == IMU_OK) {
		tm_printf((UB*)"monitor: IMU ready at 0x%02X\n",
			  (UINT)imu_accel_address());
		imu_calibrate();
	} else {
		tm_printf((UB*)"monitor: IMU init failed, status=%d\n",
			  (INT)init_status);
		(void)publish_imu_sample(NULL, init_status, NULL, init_status);
	}

	for (;;) {
		if (init_status != IMU_OK) {
			tk_dly_tsk(500);
			init_status = imu_reset();
			if (init_status == IMU_OK) {
				tm_printf((UB*)"monitor: IMU recovered at 0x%02X\n",
					  (UINT)imu_accel_address());
				imu_calibrate();
				consecutive_fails = 0;
			} else {
				(void)publish_imu_sample(NULL, init_status,
							 NULL, init_status);
				continue;
			}
		}

		accel_status = imu_read_accel(&accel);
		mag_status = imu_read_mag(&mag);
		(void)publish_imu_sample(&accel, accel_status, &mag, mag_status);

		if ((accel_status == IMU_OK) && (mag_status == IMU_OK)) {
			consecutive_fails = 0;
		} else if (++consecutive_fails >= 10) {
			tm_printf((UB*)"monitor: IMU read failures, resetting sensor\n");
			init_status = imu_reset();
			consecutive_fails = 0;
			if (init_status == IMU_OK) {
				imu_calibrate();
			}
		}

		tk_dly_tsk(S4_IMU_SAMPLE_MS);
	}
}

EXPORT void monitor_task(INT stacd, void *exinf)
{
	UINT			flgptn;
	UB				text[S4_SUMMARY_MAX + 1];
	INT				len;
	APP_STATS		snap;
	T_RSEM			rsem;
	T_RMPF			rmpf;
	SYSTIM			now;
	s4_imu_snapshot_t imu_snapshot;
	ER				er;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"monitor: I am task %d\n", tk_get_tid());

	while (1) {
		/* 1. Arm the one-shot alarm, then wait for EITHER bit.
		      TWF_ORW = wake on any of the bits, TWF_CLR = clear them
		      on wake-up.  The extra timeout is just a safety net. */
		tk_sta_alm(alm_monitor, S4_ALARM_MS);
		er = tk_wai_flg(flg_events, FLG_ALARM | FLG_BATCH, TWF_ORW | TWF_CLR, &flgptn, S4_ALARM_MS * 2);
		tk_stp_alm(alm_monitor);	/* cancel it if a batch came first */

		if (er == E_TMOUT) 
		{
			tm_printf((UB*)"monitor: nothing at all happened?\n");
			continue;
		}
		
		if (flgptn & FLG_ALARM) 
		{
			tm_printf((UB*)"monitor: alarm -- no batch for %d ms\n", S4_ALARM_MS);
		}

		/* 2. Drain the message buffer without waiting (TMO_POL).
		      tk_rcv_mbf returns the message length, or an error. */
		while ((len = tk_rcv_mbf(mbf_summary, text, TMO_POL)) > 0) 
		{ 
			text[len] = '\0';
			s4_lines_received++;
			tm_printf((UB*)"monitor: batch  %s\n", text);
		}

		/* 3. Copy the stats under subsystem 3's mutex, print outside
		      it: hold a mutex for as short a time as possible. */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		snap = app_stats;
		tk_unl_mtx(mtx_stats);

		tk_get_tim(&now);
		tk_ref_sem(sem_slots, &rsem);
		tk_ref_mpf(mpf_samples, &rmpf);
		tm_printf((UB*)"monitor: t=%u ms  samples=%u  min=%d max=%d avg=%d"
			  "  free slots=%d  free blocks=%d\n",
			  now.lo, snap.received, snap.min, snap.max,
			  snap.received ? (INT)(snap.sum / (W)snap.received) : 0,
			  rsem.semcnt, (INT)rmpf.frbcnt);

		er = subsystem4_get_imu_snapshot(&imu_snapshot);
		if (er < E_OK) {
			tm_printf((UB*)"monitor: failed to copy IMU snapshot, error=%d\n",
				  (INT)er);
		} else if (imu_snapshot.has_sample) {
			if ((imu_snapshot.accel_status == IMU_OK) &&
			    (imu_snapshot.mag_status == IMU_OK)) {
				tm_printf((UB*)"monitor: IMU accel=(%d,%d,%d) "
					  "mag=(%d,%d,%d)\n",
					  (INT)imu_snapshot.accel.x,
					  (INT)imu_snapshot.accel.y,
					  (INT)imu_snapshot.accel.z,
					  (INT)imu_snapshot.mag.x,
					  (INT)imu_snapshot.mag.y,
					  (INT)imu_snapshot.mag.z);
			} else {
				tm_printf((UB*)"monitor: IMU read failed, "
					  "accel=%d mag=%d\n",
					  (INT)imu_snapshot.accel_status,
					  (INT)imu_snapshot.mag_status);
			}
		}
	}
}