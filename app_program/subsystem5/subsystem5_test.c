/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Results go in testing/subsystem5/TEST_LOG.md.
 *
 *	S5-U01..U06  CR1 packing, run on the real compiler and CPU
 *	S5-U07       geometry still gives the same answer from packed data
 *	S5-U08       profile build time on the Cortex-M0+ (CR1 performance)
 *	S5-U09..U11  the running task: alive, sensor answering, no overruns
 *
 *	The tests run in the initial task while the obstacle task runs, so
 *	they never touch the servo or sonar themselves (one owning task).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
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

		TEST_CHECK("S5-U06", n == S5_TLM_LEN,
			   "telemetry packet is 29 bytes");
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
			   "profile build under 1 ms (0.07 % of a 1.4 s scan)");
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
		TEST_CHECK("S5-U10", h.fault == OBSTACLE_FAULT_NONE,
			   "sensor answering (no fault; needs sonar wired)");
		TEST_CHECK("S5-U11", h.deadline_overruns == 0U,
			   "no missed 60 ms guard ticks");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
