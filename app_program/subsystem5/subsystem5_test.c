/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Results go in testing/subsystem5/TEST_LOG.md.
 *
 *	S5-U01..U06  CR1 packing, run on the real compiler and CPU
 *	S5-U07       geometry still gives the same answer from packed data
 *	S5-U08       profile build time on the Cortex-M0+ (CR1 performance)
 *	S5-U09..U11  the running task: alive, sensor answering, no overruns
 *	S5-U12..U14  v3.0: refined centre, watch word, calibration status
 *
 *	The tests run in the initial task while the obstacle task runs, so
 *	they never touch the servo or sonar themselves (one owning task).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_calib.h"
#include "s5_config.h"
#include "../test_report.h"

#if APP_TEST

/* Narrow post 300 mm ahead; the same vector as host test G2. */
static void s5t_post_sweep(s5_sample_t *p, uint8_t *p_n)
{
	static const int16_t  b[7] = { -60, -30, -10, 0, 10, 30, 60 };
	static const uint16_t r[7] = { OBSTACLE_RANGE_CLEAR,
		OBSTACLE_RANGE_CLEAR, OBSTACLE_RANGE_CLEAR, 250U,
		OBSTACLE_RANGE_CLEAR, OBSTACLE_RANGE_CLEAR,
		OBSTACLE_RANGE_CLEAR };
	uint8_t i;

	for (i = 0U; i < 7U; i++) {
		p[i] = s5_sample_pack(b[i], r[i]);
	}
	*p_n = 7U;
}

EXPORT void subsystem5_test(void)
{
	TEST_INIT();

	/* ---- CR1 packing ------------------------------------------- */
	{
		bool    ok = true;
		int16_t b;

		for (b = -70; b <= 70; b = (int16_t)(b + 10)) {
			const s5_sample_t s1 = s5_sample_pack(b, 30U);
			const s5_sample_t s2 = s5_sample_pack(b, 2500U);
			const s5_sample_t s3 = s5_sample_pack(b,
						OBSTACLE_RANGE_CLEAR);
			const s5_sample_t s4 = s5_sample_pack(b,
						OBSTACLE_RANGE_UNKNOWN);

			ok = ok && (s5_sample_bearing(s1) == b)
				&& (s5_sample_range(s1) == 30U)
				&& (s5_sample_range(s2) == 2500U)
				&& (s5_sample_range(s3) == OBSTACLE_RANGE_CLEAR)
				&& (s5_sample_range(s4) == OBSTACLE_RANGE_UNKNOWN);
		}
		TEST_CHECK("S5-U01", ok,
			   "sample pack/unpack: 15 bearings x 30, 2500, CLEAR, UNKNOWN");
	}

	{
		const s5_front_t w = s5_front_pack(245U,
				(uint32_t)ULTRASONIC_OK, 131070U);

		TEST_CHECK("S5-U02",
			   (s5_front_range(w) == 245U) &&
			   (s5_age_ms(s5_front_stamp(w), 131072U + 100U) == 102U),
			   "front word: range survives, 17-bit stamp age is wrap-safe");
	}

	{
		s5_sample_t               s[7];
		uint8_t                   n;
		obstacle_profile_t        p;
		obstacle_profile_t        q;
		obstacle_profile_packed_t pk;

		s5t_post_sweep(s, &n);
		obstacle_profile_build(s, n, &p);
		p.timestamp_ms = 1234U;
		obstacle_profile_pack(&p, &pk);
		obstacle_profile_unpack(&pk, &q);

		TEST_CHECK("S5-U03",
			   (q.valid == p.valid) &&
			   (q.nearest_mm == p.nearest_mm) &&
			   (q.bearing_deg == p.bearing_deg) &&
			   (q.width_mm == p.width_mm) &&
			   (q.gap_left_mm == p.gap_left_mm) &&
			   (q.gap_right_mm == p.gap_right_mm) &&
			   (q.swing_left_mm == p.swing_left_mm) &&
			   (q.swing_right_mm == p.swing_right_mm) &&
			   (q.confidence_pct == p.confidence_pct) &&
			   (q.action == p.action) &&
			   (q.timestamp_ms == 1234U),
			   "profile pack/unpack round trip, every field");
	}

	{
		obstacle_health_t        h = { 0 };
		obstacle_health_t        g;
		obstacle_health_packed_t pk;

		h.state           = OBSTACLE_ST_WAIT_STOP;
		h.fault           = OBSTACLE_FAULT_ASSIST_TIMEOUT;
		h.estop_requested = true;
		h.reverses        = 2U;
		h.last_status     = (uint8_t)ULTRASONIC_ECHO_STUCK;
		h.sensor_faults   = 255U;
		h.scans_completed = 65535U;
		h.build_us_max    = 777U;
		obstacle_health_pack(&h, &pk);
		obstacle_health_unpack(&pk, &g);

		TEST_CHECK("S5-U04",
			   (g.state == h.state) && (g.fault == h.fault) &&
			   g.estop_requested && (g.reverses == 2U) &&
			   (g.last_status == h.last_status) &&
			   (g.sensor_faults == 255U) &&
			   (g.scans_completed == 65535U) &&
			   (g.build_us_max == 777U),
			   "health pack/unpack round trip at field limits");
	}

	TEST_CHECK("S5-U05",
		   (sizeof(s5_sample_t) == 2U) &&
		   (sizeof(obstacle_profile_packed_t) == 16U) &&
		   (sizeof(obstacle_health_packed_t) == 8U) &&
		   (sizeof(ultrasonic_result_t) == 4U),
		   "packed sizes: sample 2 B, profile 16 B, health 8 B, ping 4 B");

	{
		uint8_t buf[S5_TLM_LEN];
		uint32_t n = obstacle_get_telemetry(buf, (uint32_t)sizeof(buf));

		TEST_CHECK("S5-U06", (n == S5_TLM_LEN) && (S5_TLM_LEN == 33U),
			   "telemetry packet is 33 bytes (v3.0: + watch word)");
	}

	/* ---- geometry and timing ------------------------------------ */
	{
		s5_sample_t        s[7];
		uint8_t            n;
		obstacle_profile_t p;
		uint32_t           t0;
		uint32_t           dt;

		s5t_post_sweep(s, &n);

		t0 = drv_ultrasonic_time_us();
		obstacle_profile_build(s, n, &p);
		dt = drv_ultrasonic_time_us() - t0;

		TEST_CHECK("S5-U07", p.valid &&
			   ((p.action == OBSTACLE_ACT_BYPASS_LEFT) ||
			    (p.action == OBSTACLE_ACT_BYPASS_RIGHT)),
			   "narrow post from packed samples -> bypass");

		tm_printf((UB*)"[INFO] S5 profile build %d us (7 samples)\n",
			  (INT)dt);
		TEST_CHECK("S5-U08", dt < 1000U,
			   "profile build under 1 ms (under 0.1 % of a scan)");
	}

	/* ---- the running task ---------------------------------------- */
	{
		obstacle_health_t  h;
		ultrasonic_stats_t st;

		tk_dly_tsk(1500);	/* let ~20 guard pings happen */
		obstacle_get_health(&h);
		drv_ultrasonic_get_stats(&st);

		tm_printf((UB*)"[INFO] S5 state %d fault %d pings %d"
			  " no_response %d no_echo %d front %d mm\n",
			  (INT)h.state, (INT)h.fault, (INT)st.pings,
			  (INT)st.no_response, (INT)st.no_echo,
			  (INT)obstacle_get_front_mm());

		TEST_CHECK("S5-U09", (h.state != OBSTACLE_ST_INIT) &&
			   (st.pings >= 10U),
			   "obstacle task running, >= 10 pings in 1.5 s");
		/* Only a SENSOR fault means the sonar is broken. On the bench
		   there is no VehicleController to call obstacle_resume(), so
		   an object in front ends in ASSIST_TIMEOUT by design. */
		TEST_CHECK("S5-U10", h.fault != OBSTACLE_FAULT_SENSOR,
			   "sensor answering (no SENSOR fault; needs sonar wired)");
		TEST_CHECK("S5-U11", h.deadline_overruns == 0U,
			   "no missed 60 ms guard ticks");
	}

	/* ---- v3.0 ------------------------------------------------------ */
	{
		/* An 80 mm box square on, 280 mm ahead, seen by a 7.5 deg beam
		   on the 10 deg grid, edges probed to 2.5 deg: same vector as
		   host test V2. The refined centre must be straight ahead. */
		static const int16_t  b[7] = { -30, -20, -10, 0, 10, 20, 30 };
		static const uint16_t r[7] = { OBSTACLE_RANGE_CLEAR,
			OBSTACLE_RANGE_CLEAR, 284U, 280U, 284U,
			OBSTACLE_RANGE_CLEAR, OBSTACLE_RANGE_CLEAR };
		const s5_geom_cal_t    cal = { 75, 10, 30, OBSTACLE_TRIGGER_MM };
		const s5_edge_probes_t pr  = { { true, 150, 175 },
					       { true, -150, -175 } };
		s5_sample_t        s[7];
		obstacle_profile_t p;
		uint8_t            i;

		for (i = 0U; i < 7U; i++) {
			s[i] = s5_sample_pack(b[i], r[i]);
		}
		obstacle_profile_build(s, 7U, &p);
		obstacle_profile_refine(&p, &cal, &pr);

		tm_printf((UB*)"[INFO] S5 refined: centre %d deg, width %d mm,"
			  " action %d\n", (INT)p.bearing_deg, (INT)p.width_mm,
			  (INT)p.action);
		TEST_CHECK("S5-U12", (p.bearing_deg == 0) &&
			   (p.width_mm >= 60U) && (p.width_mm <= 100U),
			   "v3.0 refined centre 0 deg, width 60..100 mm");
	}

	{
		const s5_watch_t w = s5_watch_pack(true, S5_WATCH_CLEARING,
						   250U, 140U, 280U);

		TEST_CHECK("S5-U13", s5_watch_left(w) &&
			   (s5_watch_phase(w) == S5_WATCH_CLEARING) &&
			   (s5_watch_reverse_mm(w) == 250U) &&
			   (s5_watch_lateral_mm(w) == 140U) &&
			   (s5_watch_range(w) == 280U),
			   "v3.0 watch word round trip on the ARM compiler");
	}

	{
		obstacle_health_t h;

		obstacle_get_health(&h);
		tm_printf((UB*)"[INFO] S5 calibration: %s (trim %d, backlash %d,"
			  " beam %d in 0.1 deg; range x%d/4096, offset %d mm)\n",
			  h.calibrated ? "DONE" : "NOT DONE - run S5_CAL_BUILD",
			  (INT)S5_CAL_SERVO_TRIM_DDEG,
			  (INT)S5_CAL_SERVO_BACKLASH_DDEG,
			  (INT)S5_CAL_BEAM_HALF_DDEG, (INT)S5_CAL_RANGE_SCALE_Q12,
			  (INT)S5_CAL_RANGE_OFFSET_MM);
		TEST_CHECK("S5-U14", h.calibrated == (S5_CAL_DONE != 0),
			   "health reports the calibration status");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
