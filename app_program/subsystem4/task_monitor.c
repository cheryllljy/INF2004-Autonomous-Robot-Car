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
#include "../subsystem1/subsystem1.h"	/* IF-05: tid_led */
#include "../subsystem2/subsystem2.h"	/* sem_slots, mpf_samples (read only) */
#include "../subsystem3/subsystem3.h"	/* IF-04: app_stats, mtx_stats */
#include "imu.h"

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

LOCAL const char *state_name(UINT tskstat)
{
	switch (tskstat) {
	case TTS_RUN:	return "RUN";
	case TTS_RDY:	return "READY";
	case TTS_WAI:	return "WAIT";
	case TTS_SUS:	return "SUSPEND";
	case TTS_WAS:	return "WAIT+SUSPEND";
	case TTS_DMT:	return "DORMANT";
	default:		return "?";
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
	T_RTSK			rtsk;
	SYSTIM			now;
	imu_vector_t	accel;
	imu_vector_t	mag;
	imu_status_t	init_status;
	imu_status_t	accel_status;
	imu_status_t	mag_status;
	UW				reports = 0;
	BOOL			led_paused = FALSE;
	ER				er;
	int 			consecutive_fails = 0;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"monitor: I am task %d\n", tk_get_tid());

	init_status = imu_init();
	if (init_status == IMU_OK) 
	{
		tm_printf((UB*)"monitor: IMU ready at 0x%02X\n", (UINT)imu_accel_address());
	} 
	
	else 
	{
		tm_printf((UB*)"monitor: IMU init failed, status=%d\n", (INT)init_status);
	}

	/* Wait for ALL the startup tests to finish.
	 * The B3-H04 and B3-H05 barcode tests each take 15s and hammer I2C. */
	tk_dly_tsk(30000);
	
	if (init_status == IMU_OK) 
	{
		imu_calibrate();
	}

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

		/* The monitor task is the sole owner of I2C0 and the IMU. */
		/* ---- IMU recovery path ---- */
		if (init_status != IMU_OK) {
			init_status = imu_reset();
			if (init_status == IMU_OK) {
				tm_printf((UB*)"monitor: IMU recovered at 0x%02X\n",
					  (UINT)imu_accel_address());
				tk_dly_tsk(500);
				imu_calibrate();
			} else {
				tm_printf((UB*)"monitor: IMU reset failed, status=%d\n",
					  (INT)init_status);
			}
		}

		if (init_status == IMU_OK) {
			accel_status = imu_read_accel(&accel);
			mag_status   = imu_read_mag(&mag);

			if (accel_status == IMU_OK && mag_status == IMU_OK) {
				consecutive_fails = 0;
				tm_printf((UB*)"monitor: IMU accel=(%d,%d,%d) "
					  "mag=(%d,%d,%d)\n",
					  (INT)accel.x, (INT)accel.y, (INT)accel.z,
					  (INT)mag.x, (INT)mag.y, (INT)mag.z);
			} else {
				consecutive_fails++;
				if (consecutive_fails >= 10) {
					tm_printf((UB*)"monitor: IMU DEAD, resetting\n");
					init_status = IMU_I2C_ERROR;
					consecutive_fails = 0;
				}
			}
		}
	}
}