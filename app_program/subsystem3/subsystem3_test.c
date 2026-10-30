/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 *
 *	Buddy 3 (line following + Code 39 barcodes) test groups:
 *	  B3-U..  logic tests, no hardware needed (pin map, Code 39, line rules)
 *	  B3-I..  the running Buddy 3 task (period, failsafe, mode gate)
 *	  B3-H..  the IR sensors IR_R/IR_L/IR_B on GP26/27/28 (robot on track)
 *	B3_TEST_HW = 0 skips the B3-H tests, e.g. on a bare Pico.
 *	This file plays the vehicle controller: it alone calls
 *	buddy3_set_context() during the test run.
 */

#include "subsystem3.h"
#include "buddy3.h"
#include "buddy3_team_config.h"
#include "b3_port.h"
#if B3_USE_B2_ODOMETRY
#include "../subsystem2/motion_control.h"
#endif
#include "../test_report.h"

#if APP_TEST

#ifndef B3_TEST_HW
#define B3_TEST_HW		1	/* 1 = run the B3-H sensor tests */
#endif
#define B3_TEST_PLACE_MS	5000	/* time to place the robot for a B3-H test */
#define B3_TEST_SAMPLES		100	/* sensor samples per B3-H test, 10 ms apart */
#define B3_TEST_DRIVE_MM	300	/* B3-H06: distance Buddy 2 drives over the barcode */
#define B3_TEST_DRIVE_PLACE_MS	10000	/* B3-H06: time to place the car on the floor */
#define B3_TEST_DRIVE_MAX_MS	8000	/* B3-H06: give up on the move after this */
#define B3_TEST_SCAN_MS		15000	/* time allowed to push the robot over a barcode */
#define B3_ADC_MID		2048	/* half of the 12-bit ADC range */

/* TEMPORARY sensor setup aid: 1 = after B3-H00, print the three IR sensors
   live (forever, the remaining tests do not run) so the height and pot can
   be adjusted while watching the serial monitor.  Set back to 0 after. */
#ifndef B3_TEST_LIVE_IR
#define B3_TEST_LIVE_IR		0
#endif
/* Which B3-H sensor tests run:
   0 = all
   1 = line only:    H01..H03 sensors, H07 steering, H08 drive along a line
   2 = barcode only: H04..H06
   3 = combined drive only: H09 (follow the line AND decode a barcode) */
#ifndef B3_TEST_SELECT
#define B3_TEST_SELECT		3
#endif
#define B3_RUN_LINE_TESTS	((B3_TEST_SELECT == 0) || (B3_TEST_SELECT == 1))
#define B3_RUN_BARCODE_TESTS	((B3_TEST_SELECT == 0) || (B3_TEST_SELECT == 2))
#define B3_RUN_COMBINED_TEST	((B3_TEST_SELECT == 0) || (B3_TEST_SELECT == 3))
#define B3_TEST_FOLLOW_MM	400	/* B3-H08/H09: drive this far along the line */
#define B3_TEST_FOLLOW_MS	8000	/* B3-H08/H09: give up if not there by then */
#define B3_TEST_FOLLOW_OFF_MS	1000	/* B3-H08: fail if one sensor stays on the line longer */
/* 1 = the live readout prints IR_B (barcode) only */
#ifndef B3_LIVE_IR_B_ONLY
#define B3_LIVE_IR_B_ONLY	0
#endif
#define B3_LIVE_FIRST		(B3_LIVE_IR_B_ONLY ? 2 : 0)
#define B3_LIVE_WINDOW		20	/* samples per printed line, 10 ms apart */

LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */

/* Build the 29 run widths of "*<symbol>*" at the given narrow/wide widths */
LOCAL BOOL make_frame(const char frame[3], UW narrow, UW wide,
		      uint32_t widths[B3_BARCODE_RUNS])
{
	uint16_t	pattern;
	INT		c, bit, out = 0;

	for (c = 0; c < 3; c++) {
		if (!b3_code39_pattern(frame[c], &pattern)) {
			return FALSE;
		}
		for (bit = 8; bit >= 0; bit--) {
			widths[out++] = (pattern & (1U << bit)) ? wide : narrow;
		}
		if (c < 2) {
			widths[out++] = narrow;		/* inter-character gap */
		}
	}
	return TRUE;
}

LOCAL void reverse_frame(uint32_t widths[B3_BARCODE_RUNS])
{
	INT		i;
	uint32_t	t;

	for (i = 0; i < B3_BARCODE_RUNS / 2; i++) {
		t = widths[i];
		widths[i] = widths[B3_BARCODE_RUNS - 1 - i];
		widths[B3_BARCODE_RUNS - 1 - i] = t;
	}
}

/* Count the wide (1) bits of a 9-bit Code 39 pattern */
LOCAL INT wide_count(uint16_t pattern)
{
	INT	n = 0;

	while (pattern != 0U) {
		n += pattern & 1U;
		pattern >>= 1;
	}
	return n;
}

/* Feed one frame to the barcode state machine as timed edges, then the
   trailing white quiet zone.  Returns TRUE and the result on a decode. */
LOCAL BOOL scan_frame(const uint32_t times[B3_BARCODE_RUNS], UW quiet_us,
		      b3_barcode_result_t *p_result)
{
	b3_barcode_t	scan;
	b3_edge_t	edge = {0};
	INT		i;

	edge.time_us = 1000000U - quiet_us;	/* white before the bars */
	b3_barcode_init(&scan, edge);
	edge.time_us = 1000000U;
	edge.black = true;
	if (b3_barcode_edge(&scan, edge, p_result)) {
		return FALSE;
	}
	for (i = 0; i < B3_BARCODE_RUNS; i++) {
		edge.time_us += times[i];
		edge.black = !edge.black;
		if (b3_barcode_edge(&scan, edge, p_result)) {
			return FALSE;		/* must not finish before the quiet zone */
		}
	}
	edge.time_us += quiet_us;		/* white after the bars */
	return b3_barcode_poll(&scan, edge, p_result) ? TRUE : FALSE;
}

/* Hand Buddy 3 a fresh controller context */
LOCAL void set_context(BOOL follow, BOOL scan)
{
	b3_context_t	ctx = {0};

	ctx.time_us        = b3_port_time_us();
	ctx.follow_enabled = follow ? true : false;
	ctx.scan_enabled   = scan ? true : false;
	buddy3_set_context(&ctx);
}

/* Keep the context fresh for ms milliseconds (10 ms steps) */
LOCAL void run_with_context(INT ms, BOOL follow, BOOL scan)
{
	for (; ms > 0; ms -= 10) {
		set_context(follow, scan);
		tk_dly_tsk(10);
	}
}

#if B3_TEST_HW
/* Read one ADC channel.  Uses Buddy 3's polled read (b3_port_adc_read)
   rather than the kernel "adca" driver, so it cannot collide with the
   IR_B analog sampler when B3_BARCODE_ANALOG is 1.  -1 = ADC busy. */
LOCAL INT adc_read(UW ch)
{
	return (INT)b3_port_adc_read(ch);
}

LOCAL void wait_for_placement(const char *id, const char *what)
{
	tm_printf((UB*)"[TEST] %s place the robot: %s (%d s)\n",
		  id, what, B3_TEST_PLACE_MS / 1000);
	run_with_context(B3_TEST_PLACE_MS, FALSE, FALSE);
}

/* Sample the IR sensors B3_TEST_SAMPLES times; count black readings and
   readings where the ADC level disagrees with the digital level */
typedef struct {
	INT	black[3];	/* IR_R, IR_L, IR_B */
	INT	mismatch[3];
	INT	adc_min[3];
	INT	adc_max[3];
	INT	centred;	/* line result B3_LINE_CENTRED */
} IR_COUNTS;

LOCAL void sample_ir(IR_COUNTS *p)
{
	static const UW	ch[3] = {B3_IR_R_ADC_CH, B3_IR_L_ADC_CH,
				 B3_IR_B_ADC_CH};
	static const UW	pin[3] = {B3_IR_R_GPIO, B3_IR_L_GPIO,
				  B3_IR_B_GPIO};
	b3_ir_sample_t	s;
	b3_snapshot_t	snap;
	uint32_t	pins;
	BOOL		black[3];
	INT		i, k, a;

	for (k = 0; k < 3; k++) {
		p->black[k] = p->mismatch[k] = 0;
		p->adc_min[k] = 4095;
		p->adc_max[k] = 0;
	}
	p->centred = 0;

	for (i = 0; i < B3_TEST_SAMPLES; i++) {
		s = b3_ir_read();
		pins = b3_port_gpio_get_all();
		black[0] = s.right_black;
		black[1] = s.left_black;
		black[2] = s.barcode.black;
		for (k = 0; k < 3; k++) {
			p->black[k] += black[k] ? 1 : 0;
			a = adc_read(ch[k]);
			if (a < p->adc_min[k]) p->adc_min[k] = a;
			if (a > p->adc_max[k]) p->adc_max[k] = a;
#if B3_BARCODE_ANALOG
			/* IR_B is AO on an analog pad: no digital level */
			if (k == 2) {
				continue;
			}
#endif
			/* digital high should read above mid-scale on the ADC */
			if ((a >= B3_ADC_MID) != (((pins >> pin[k]) & 1U) != 0U)) {
				p->mismatch[k]++;
			}
		}
		buddy3_get_snapshot(&snap);
		if (snap.line.state == B3_LINE_CENTRED) {
			p->centred++;
		}
		set_context(FALSE, FALSE);
		tk_dly_tsk(10);
	}
}

#if B3_BARCODE_ANALOG
/* IR_B analog levels as the adaptive threshold sees them (black is high,
   0..4095, whatever the wiring polarity) */
LOCAL void print_analog(const char *id)
{
	b3_ir_analog_stats_t	a;

	b3_ir_analog_stats(&a);
	tm_printf((UB*)"[TEST] %s   IR_B analog: now %u (%s)  white %u  black %u"
		  "  contrast %u (min %u)  edge at %u +/- %u  near misses %u"
		  "  samples %u\n", id, (unsigned)a.level,
		  a.black_now ? "black" : "white", (unsigned)a.white,
		  (unsigned)a.black, (unsigned)(a.black - a.white),
		  (unsigned)B3_ANALOG_MIN_CONTRAST, (unsigned)a.threshold,
		  (unsigned)a.hysteresis, (unsigned)a.near_misses,
		  (unsigned)a.samples);
}
#endif

LOCAL void print_ir(const char *id, const IR_COUNTS *p)
{
	static const char *const name[3] = {"IR_R (GP26)", "IR_L (GP27)",
					    "IR_B (GP28)"};
	INT	k;

	for (k = 0; k < 3; k++) {
		tm_printf((UB*)"[TEST] %s   %s black %d/%d  ADC %d..%d"
			  "  ADC/digital mismatch %d\n", id, name[k],
			  p->black[k], B3_TEST_SAMPLES,
			  p->adc_min[k], p->adc_max[k], p->mismatch[k]);
	}
#if B3_BARCODE_ANALOG
	print_analog(id);
#endif
}
#endif /* B3_TEST_HW */

/* ---------------------------------------------------------------------
 *  Buddy 3 tests
 * --------------------------------------------------------------------- */

LOCAL void buddy3_unit_tests(INT *p_pass, INT *p_fail)
{
	static const char data[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
	uint32_t		w[B3_BARCODE_RUNS];
	b3_code39_result_t	res;
	b3_barcode_result_t	scan;
	b3_line_t		line;
	b3_line_result_t	r;
	uint16_t		pattern;
	char			frame[3];
	INT			i, bad;
	BOOL			ok;
	INT test_pass_ = *p_pass, test_fail_ = *p_fail;

	/* B3-U01: pin map IR_R/IR_L/IR_B = GP26/27/28 (ADC0..2) */
	TEST_CHECK("B3-U01",
		   B3_IR_R_GPIO == 26U && B3_IR_L_GPIO == 27U &&
		   B3_IR_B_GPIO == 28U && B3_IR_R_ADC_CH == 0U &&
		   B3_IR_L_ADC_CH == 1U && B3_IR_B_ADC_CH == 2U,
		   "IR pins: IR_R GP26, IR_L GP27, IR_B GP28");

	/* B3-U02: every Code 39 symbol has 9 elements, exactly 3 wide */
	bad = 0;
	for (i = 0; data[i] != '\0'; i++) {
		if (!b3_code39_pattern(data[i], &pattern) ||
		    wide_count(pattern) != 3 || pattern > 0x1FFU) {
			bad++;
		}
	}
	if (!b3_code39_pattern('*', &pattern) || pattern != 0x094U) {
		bad++;
	}
	TEST_CHECK("B3-U02", bad == 0, "Code 39 table: 43 symbols + '*', 3 of 9 wide");

	/* B3-U03: "*X*" decodes to X for all 43 symbols, both directions */
	bad = 0;
	frame[0] = frame[2] = '*';
	for (i = 0; data[i] != '\0'; i++) {
		frame[1] = data[i];
		ok = make_frame(frame, 1000U, 3000U, w) &&
		     b3_code39_decode(w, &res) &&
		     res.symbol == data[i] && !res.reversed;
		reverse_frame(w);
		ok = ok && b3_code39_decode(w, &res) &&
		     res.symbol == data[i] && res.reversed;
		bad += ok ? 0 : 1;
	}
	TEST_CHECK("B3-U03", bad == 0, "decode all 43 symbols, forward and reversed");

	/* B3-U04: start/stop must both be '*' and the data must not be '*' */
	frame[0] = 'A'; frame[1] = 'B'; frame[2] = '*';
	ok = make_frame(frame, 1000U, 3000U, w) && !b3_code39_decode(w, &res);
	frame[0] = '*'; frame[1] = 'B'; frame[2] = 'A';
	ok = ok && make_frame(frame, 1000U, 3000U, w) && !b3_code39_decode(w, &res);
	frame[0] = '*'; frame[1] = '*'; frame[2] = '*';
	ok = ok && make_frame(frame, 1000U, 3000U, w) && !b3_code39_decode(w, &res);
	TEST_CHECK("B3-U04", ok, "reject missing start/stop '*' and '*' as data");

	/* B3-U05: wide:narrow ratio checked (school code is ~3:1) */
	frame[0] = '*'; frame[1] = 'A'; frame[2] = '*';
	ok = make_frame(frame, 1000U, 1200U, w) && !b3_code39_decode(w, &res);
	ok = ok && make_frame(frame, 1000U, 6000U, w) && !b3_code39_decode(w, &res);
	ok = ok && make_frame(frame, 1000U, 2500U, w) && b3_code39_decode(w, &res);
	TEST_CHECK("B3-U05", ok, "reject 1.2:1 and 6:1, accept 2.5:1 bars");

	/* B3-U06: timed edges through the scanner, 3 mm / 9 mm bars at
	   100 mm/s = 30 ms / 90 ms, with 400 ms white either side */
	ok = make_frame(frame, 30000U, 90000U, w) &&
	     scan_frame(w, 400000U, &scan) && scan.symbol == 'A';
	TEST_CHECK("B3-U06", ok, "edge stream '*A*' at 100 mm/s decodes to A");

	/* B3-U07: no decode without a white quiet zone before the bars */
	ok = make_frame(frame, 30000U, 90000U, w) && !scan_frame(w, 60000U, &scan);
	TEST_CHECK("B3-U07", ok, "reject a frame with a short quiet zone");

	/* B3-U12/U13: any printed size decodes (ratio-based).  1 mm / 3 mm
	   bars and 15 mm / 45 mm bars at 100 mm/s, each with a quiet zone of
	   10 narrow widths; also a 2:1 barcode */
	ok = make_frame(frame, 10000U, 30000U, w) &&
	     scan_frame(w, 120000U, &scan) && scan.symbol == 'A';
	ok = ok && make_frame(frame, 150000U, 450000U, w) &&
	     scan_frame(w, 1600000U, &scan) && scan.symbol == 'A';
	TEST_CHECK("B3-U12", ok, "1 mm and 15 mm narrow bars both decode to A");
	ok = make_frame(frame, 30000U, 60000U, w) &&
	     scan_frame(w, 400000U, &scan) && scan.symbol == 'A';
	TEST_CHECK("B3-U13", ok, "2:1 wide:narrow barcode decodes to A");

	/* B3-U08..U11: line rules, straddle layout (IR_L/IR_R either side of
	   the line, white = on track; 10 ms steps) */
	b3_line_init(&line, 0U);
	r = b3_line_step(&line, false, false, 0U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, false, false, 10000U, B3_BASE_SPEED_MM_S);
	TEST_CHECK("B3-U08",
		   r.state == B3_LINE_CENTRED && r.error == 0 &&
		   r.left_mm_s == r.right_mm_s && r.left_mm_s > 0 && !r.stop,
		   "IR_L and IR_R white: on track, straight, equal wheel speeds");

	r = b3_line_step(&line, true, false, 20000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, true, false, 30000U, B3_BASE_SPEED_MM_S);
	TEST_CHECK("B3-U09",
		   r.state == B3_LINE_LEFT && r.error < 0 &&
		   r.left_mm_s < r.right_mm_s && r.left_mm_s >= 0,
		   "IR_L black (drifted right): steer left, no wheel reverses");

	r = b3_line_step(&line, false, true, 40000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, false, true, 50000U, B3_BASE_SPEED_MM_S);
	TEST_CHECK("B3-U10",
		   r.state == B3_LINE_RIGHT && r.error > 0 &&
		   r.right_mm_s < r.left_mm_s && r.right_mm_s >= 0,
		   "IR_R black (drifted left): steer right, no wheel reverses");

	r = b3_line_step(&line, true, true, 60000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, true, true, 70000U, B3_BASE_SPEED_MM_S);
	ok = (r.state == B3_LINE_CROSSING) && !r.stop &&
	     r.left_mm_s == r.right_mm_s;
	r = b3_line_step(&line, false, false, 80000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, false, false, 90000U, B3_BASE_SPEED_MM_S);
	ok = ok && (r.state == B3_LINE_CENTRED) && !r.stop;
	TEST_CHECK("B3-U11", ok, "both black: crossing, straight; back to white: on track");

	*p_pass = test_pass_;
	*p_fail = test_fail_;
}

LOCAL void buddy3_task_tests(INT *p_pass, INT *p_fail)
{
	b3_snapshot_t	a, b;
	INT test_pass_ = *p_pass, test_fail_ = *p_fail;

	/* B3-I00: the microsecond timer behind every barcode time stamp runs
	   at 1 MHz (TIMERAWL against the kernel clock, over 1 s) */
	{
		UW t0, t1;

		t0 = b3_port_time_us();
		tk_dly_tsk(1000);
		t1 = b3_port_time_us();
		tm_printf((UB*)"[TEST] B3-I00   TIMERAWL advanced %u us in 1000 ms\n",
			  (unsigned)(t1 - t0));
		TEST_CHECK("B3-I00", (t1 - t0) >= 980000U && (t1 - t0) <= 1020000U,
			   "microsecond timer runs at 1 MHz");
	}

	/* B3-I01: the Buddy 3 task keeps publishing snapshots */
	buddy3_get_snapshot(&a);
	tk_dly_tsk(200);
	buddy3_get_snapshot(&b);
	TEST_CHECK("B3-I01", b.time_us != a.time_us && b.task_error == 0,
		   "Buddy 3 task is running");

	/* B3-I02: with no controller context the outputs are failsafe */
	TEST_CHECK("B3-I02",
		   b.stop_required && !b.control_active &&
		   b.line.left_mm_s == 0 && b.line.right_mm_s == 0,
		   "no controller context: stop requested, zero speeds");

	/* B3-I03: mode gate.  Outside line-follow mode, no steering output */
	run_with_context(200, FALSE, TRUE);
	buddy3_get_snapshot(&b);
	TEST_CHECK("B3-I03",
		   !b.control_active &&
		   b.line.left_mm_s == 0 && b.line.right_mm_s == 0,
		   "follow disabled: line error never reaches the wheels");

	/* B3-I04: line-follow mode is reported as active */
	run_with_context(200, TRUE, TRUE);
	buddy3_get_snapshot(&b);
	TEST_CHECK("B3-I04", b.control_active, "follow enabled: control active");

	/* B3-I05: a stale context stops the output again */
	tk_dly_tsk(300);	/* > B3_FEEDBACK_MAX_AGE_US with no update */
	buddy3_get_snapshot(&b);
	TEST_CHECK("B3-I05",
		   b.stop_required && b.line.left_mm_s == 0 &&
		   b.line.right_mm_s == 0,
		   "stale controller context: stop requested, zero speeds");

	/* B3-I06: sensors are sampled at least every 10 ms (NFR7).  The
	   longest gap over a 1 s window must stay within one extra tick.
	   The window starts here so the CPU-heavy unit tests above (which run
	   at priority 1 on the same core) are not counted. */
	buddy3_test_reset_max_gap();
	run_with_context(1000, FALSE, FALSE);
	buddy3_get_snapshot(&b);
	tm_printf((UB*)"[TEST] B3-I06   longest service gap %u us (period %d ms)\n",
		  (unsigned)b.max_service_gap_us, B3_SERVICE_DELAY_MS);
	TEST_CHECK("B3-I06", b.max_service_gap_us <= 2U * B3_SERVICE_DELAY_MS * 1000U,
		   "IR sampling period within 20 ms worst case");

	*p_pass = test_pass_;
	*p_fail = test_fail_;
}

#if B3_TEST_HW
/* Print the raw IR_B edge log (test build only) and return the number
   of edges.  Each line shows the colour that ENDED at that edge and how
   long it lasted; in analog mode also the level that element reached
   (black is high), and any near miss: a dip that crossed the threshold
   but not the hysteresis, which is what a missed thin bar looks like.
   Printed after the capture so USB output cannot delay the Buddy 3 task
   while it is scanning. */
LOCAL UW print_edge_log(const char *id)
{
	static uint32_t	t[B3_EDGE_LOG_SIZE];
	static uint8_t	ev[B3_EDGE_LOG_SIZE];
	static uint16_t	pk[B3_EDGE_LOG_SIZE];
	UW		total, n, i, d, dmin, dmax, edges, last, misses;

	total = b3_ir_edge_log_read(t, ev, pk, B3_EDGE_LOG_SIZE);
	n = (total < B3_EDGE_LOG_SIZE) ? total : B3_EDGE_LOG_SIZE;
	dmin = 0xFFFFFFFFU;
	dmax = 0U;
	edges = 0U;
	misses = 0U;
	last = 0U;
	for (i = 0; i < n; i++) {
		if (ev[i] == 0U) {
			misses++;
			tm_printf((UB*)"[TEST] %s      near miss, reached %u\n",
				  id, (unsigned)pk[i]);
			tk_dly_tsk(2);
			continue;
		}
		edges++;
		if (edges == 1U) {
			last = i;
			continue;
		}
		d = t[i] - t[last];
		last = i;
		if (d < dmin) dmin = d;
		if (d > dmax) dmax = d;
		/* rise = pin went high = now black (IR_B black is high),
		   so the element that just ended was white */
		tm_printf((UB*)"[TEST] %s   %2u %s ended, %6u us", id,
			  (unsigned)(edges - 1U),
			  (ev[i] == B3_GPIO_EDGE_RISE) ? "white" :
			  (ev[i] == B3_GPIO_EDGE_FALL) ? "black" : "BOTH ",
			  (unsigned)d);
#if B3_BARCODE_ANALOG
		tm_printf((UB*)", reached %4u", (unsigned)pk[i]);
#endif
		tm_printf((UB*)"%s\n",
			  (d < B3_MIN_EDGE_US) ? "  <- under B3_MIN_EDGE_US" :
			  (d > B3_MAX_ELEMENT_US) ? "  <- over B3_MAX_ELEMENT_US" : "");
		tk_dly_tsk(2);	/* let the USB console drain */
	}
	tm_printf((UB*)"[TEST] %s   %u edges, %u near misses (%u of %u entries"
		  " logged), shortest %u us, longest %u us\n", id,
		  (unsigned)edges, (unsigned)misses, (unsigned)n,
		  (unsigned)total, (unsigned)((edges > 1U) ? dmin : 0U),
		  (unsigned)dmax);
#if B3_BARCODE_ANALOG
	print_analog(id);
#endif
	return edges;
}

#if B3_TEST_LIVE_IR
/* Print one line every B3_LIVE_WINDOW x 10 ms: for each sensor the colour
   now, how many of the window's samples read black, and the ADC range on
   its pin.  A sensor that flickers (e.g. black 7/20) is sitting right on
   its threshold.  IR_B also shows the edges counted in that window. */
LOCAL void live_ir(void)
{
	static const UW	ch[3] = {B3_IR_R_ADC_CH, B3_IR_L_ADC_CH,
				 B3_IR_B_ADC_CH};
	static const char *const name[3] = {"IR_R", "IR_L", "IR_B"};
	b3_ir_sample_t	smp;
	BOOL		black[3];
	INT		nblack[3], amin[3], amax[3];
	INT		i, k, a;
	UW		edges;

	tm_printf((UB*)"[LIVE] IR sensor readout (B3_TEST_LIVE_IR 1 in"
		  " subsystem3_test.c; set 0 to run the tests)\n");
	tm_printf((UB*)"[LIVE] colour now, black samples / %d, ADC min..max\n",
		  B3_LIVE_WINDOW);
	for (;;) {
		for (k = 0; k < 3; k++) {
			nblack[k] = 0;
			amin[k] = 4095;
			amax[k] = 0;
		}
		b3_ir_edge_log_reset();
		for (i = 0; i < B3_LIVE_WINDOW; i++) {
			smp = b3_ir_read();
			black[0] = smp.right_black;
			black[1] = smp.left_black;
			black[2] = smp.barcode.black;
			for (k = 0; k < 3; k++) {
				nblack[k] += black[k] ? 1 : 0;
				a = adc_read(ch[k]);
				if (a < amin[k]) amin[k] = a;
				if (a > amax[k]) amax[k] = a;
			}
			set_context(FALSE, FALSE);
			tk_dly_tsk(10);
		}
		edges = b3_ir_edge_log_read(NULL, NULL, NULL, 0U);
		for (k = B3_LIVE_FIRST; k < 3; k++) {
			tm_printf((UB*)"[LIVE] %s %s %2d/%d %4d..%4d   ",
				  name[k], black[k] ? "BLACK" : "white",
				  nblack[k], B3_LIVE_WINDOW, amin[k], amax[k]);
		}
		tm_printf((UB*)"IR_B edges %u\n", (unsigned)edges);
	}
}
#endif

/* B3-H07 helper: follow mode on, motors untouched.  Count samples where
   Buddy 3 reports the expected state and steers that way (the wheel on
   that side slower). */
LOCAL INT count_steer(b3_line_state_t want)
{
	b3_snapshot_t	snap;
	INT		i, n = 0;

	for (i = 0; i < B3_TEST_SAMPLES; i++) {
		set_context(TRUE, FALSE);
		tk_dly_tsk(10);
		buddy3_get_snapshot(&snap);
		if ((snap.line.state == want) &&
		    ((want == B3_LINE_LEFT) ?
		     (snap.line.left_mm_s < snap.line.right_mm_s) :
		     (snap.line.right_mm_s < snap.line.left_mm_s))) {
			n++;
		}
	}
	set_context(FALSE, FALSE);
	return n;
}

/* B3-H08: Buddy 3 steers, Buddy 2 drives.  This test plays the vehicle
   controller: every 10 ms it passes Buddy 2's measured speeds to Buddy 3
   and Buddy 3's wheel speeds to motion_set_wheel_speeds(). */
LOCAL void follow_line_test(const char *id, BOOL scan, INT *p_pass,
			     INT *p_fail)
{
	INT test_pass_ = *p_pass, test_fail_ = *p_fail;
#if !B3_USE_B2_ODOMETRY
	(void)scan;
	tm_printf((UB*)"[TEST] %s skipped: needs Buddy 2 (B3_USE_B2_ODOMETRY 1)\n", id);
#elif MOTION_DEMO
	(void)scan;
	tm_printf((UB*)"[TEST] %s skipped: set MOTION_DEMO to 0 in"
		  " motion_control.h\n", id);
#else
	motion_status_t	ms;
	b3_context_t	ctx;
	b3_snapshot_t	snap;
	INT		t, n = 0, on = 0, left = 0, right = 0, cross = 0;
	INT		corrections = 0, off_ms = 0, off_max = 0;
	int16_t		sent_l = 0, sent_r = 0;
	INT		since_send = 0;
	b3_line_state_t	prev = B3_LINE_CENTRED;
	BOOL		stopped = FALSE;
	UW		seq;

	tm_printf((UB*)"[TEST] %s place the car on the floor, line BETWEEN"
		  " IR_L and IR_R, %d mm of line ahead%s (%d s)\n", id,
		  B3_TEST_FOLLOW_MM + 100,
		  scan ? ", and the barcode in IR_B's path" : "",
		  B3_TEST_DRIVE_PLACE_MS / 1000);
	run_with_context(B3_TEST_DRIVE_PLACE_MS, FALSE, scan);
	buddy3_get_snapshot(&snap);
	seq = snap.last_barcode.sequence;
	b3_ir_edge_log_reset();
	(void)motion_reset_trip();
	tk_dly_tsk(50);			/* let Buddy 2 apply the trip reset */

	for (t = 0; t < B3_TEST_FOLLOW_MS; t += 10) {
		motion_get_status(&ms);
		if (ms.distance_trip_mm >= B3_TEST_FOLLOW_MM) {
			break;
		}
		ctx = (b3_context_t){0};
		ctx.time_us = b3_port_time_us();
		ctx.follow_enabled = true;
		ctx.scan_enabled = scan ? true : false;
		ctx.motion_valid = true;
		ctx.left_measured_mm_s = ms.speed_l_mmps;
		ctx.right_measured_mm_s = ms.speed_r_mmps;
		buddy3_set_context(&ctx);
		buddy3_get_snapshot(&snap);
		if (snap.stop_required || (ms.state == MOTION_STATE_FAULT)) {
			stopped = TRUE;
			break;
		}
		/* Send on change, and refresh every 100 ms */
		if ((snap.line.left_mm_s != sent_l) ||
		    (snap.line.right_mm_s != sent_r) || (since_send >= 100)) {
			if (motion_set_wheel_speeds(snap.line.left_mm_s,
						    snap.line.right_mm_s) == MOTION_OK) {
				sent_l = snap.line.left_mm_s;
				sent_r = snap.line.right_mm_s;
				since_send = 0;
			}
		}
		since_send += 10;

		n++;
		switch (snap.line.state) {
		case B3_LINE_LEFT:	left++;		break;
		case B3_LINE_RIGHT:	right++;	break;
		case B3_LINE_CROSSING:	cross++;	break;
		default:		on++;		break;
		}
		if (((snap.line.state == B3_LINE_LEFT) ||
		     (snap.line.state == B3_LINE_RIGHT)) &&
		    (snap.line.state != prev)) {
			corrections++;
		}
		if ((snap.line.state == B3_LINE_LEFT) ||
		    (snap.line.state == B3_LINE_RIGHT)) {
			off_ms += 10;
			if (off_ms > off_max) off_max = off_ms;
		} else {
			off_ms = 0;
		}
		prev = snap.line.state;
		tk_dly_tsk(10);
	}
	(void)motion_stop();
	run_with_context(500, FALSE, scan);	/* trailing quiet zone */
	motion_get_status(&ms);
	buddy3_get_snapshot(&snap);

	tm_printf((UB*)"[TEST] %s   %d ms: on track %d, IR_L on line %d,"
		  " IR_R on line %d, crossing %d (of %d samples)\n",
		  id, t, on, left, right, cross, n);
	tm_printf((UB*)"[TEST] %s   %d corrections, longest on one side %d ms,"
		  " drove %d mm (L %d R %d total), fault %d%s\n", id,
		  corrections, off_max, (int)ms.distance_trip_mm,
		  (int)ms.dist_l_mm, (int)ms.dist_r_mm,
		  (int)ms.fault, stopped ? ", STOPPED EARLY" : "");
	if (!scan) {
		TEST_CHECK("B3-H08", !stopped && (t < B3_TEST_FOLLOW_MS) &&
			   (off_max < B3_TEST_FOLLOW_OFF_MS) && (on >= n / 2),
			   "drives along the line: drifts are corrected within 1 s");
	} else {
		tm_printf((UB*)"[TEST] %s   symbol '%c'%s  rejected scans %u\n", id,
			  (snap.last_barcode.sequence != seq) ?
				  snap.last_barcode.barcode.symbol : '-',
			  snap.last_barcode.barcode.reversed ? " (reversed)" : "",
			  (unsigned)snap.rejected_scans);
		(void)print_edge_log(id);
		TEST_CHECK("B3-H09", !stopped && (off_max < B3_TEST_FOLLOW_OFF_MS) &&
			   (snap.last_barcode.sequence != seq),
			   "follows the line and decodes the barcode on the way");
	}
#endif
	*p_pass = test_pass_;
	*p_fail = test_fail_;
}

LOCAL void buddy3_hw_tests(INT *p_pass, INT *p_fail)
{
	IR_COUNTS	c;
	b3_snapshot_t	snap;
	UW		seq;
	INT		ms, k;
	BOOL		ok;
	INT test_pass_ = *p_pass, test_fail_ = *p_fail;

	k = adc_read(B3_IR_R_ADC_CH);
	TEST_CHECK("B3-H00", k >= 0, "ADC reads (IR_R on ADC0)");
	if (k < 0) {
		*p_pass = test_pass_;
		*p_fail = test_fail_;
		return;
	}
#if B3_TEST_LIVE_IR
	live_ir();
#endif
#if B3_BARCODE_ANALOG
	tm_printf((UB*)"[TEST] B3-H00   IR_B is analog: wire the module's AO"
		  " (not DO) to GP28\n");
#endif

#if !B3_RUN_LINE_TESTS
	(void)c;
	(void)ok;
	tm_printf((UB*)"[TEST] B3-H01..H03, H07, H08 skipped (B3_TEST_SELECT)\n");
#else
	/* B3-H01: all sensors on white.  Each DO reads stable white and the
	   ADC agrees with the digital level (sensor self-check) */
	wait_for_placement("B3-H01", "all three sensors over WHITE");
	sample_ir(&c);
	print_ir("B3-H01", &c);
	ok = TRUE;
	for (k = 0; k < 3; k++) {
		ok = ok && c.black[k] <= B3_TEST_SAMPLES / 20 &&
		     c.mismatch[k] <= B3_TEST_SAMPLES / 20;
	}
	TEST_CHECK("B3-H01", ok, "white: every sensor reads white, ADC agrees");

	/* B3-H02: all sensors on black (e.g. a black card or junction) */
	wait_for_placement("B3-H02", "all three sensors over BLACK");
	sample_ir(&c);
	print_ir("B3-H02", &c);
	ok = TRUE;
	for (k = 0; k < 3; k++) {
		ok = ok && c.black[k] >= B3_TEST_SAMPLES - B3_TEST_SAMPLES / 20 &&
		     c.mismatch[k] <= B3_TEST_SAMPLES / 20;
	}
	TEST_CHECK("B3-H02", ok, "black: every sensor reads black, ADC agrees");

	/* B3-H03: robot centred on the line */
	wait_for_placement("B3-H03", "line BETWEEN IR_L and IR_R (both white)");
	sample_ir(&c);
	print_ir("B3-H03", &c);
	tm_printf((UB*)"[TEST] B3-H03   line state CENTRED %d/%d\n",
		  c.centred, B3_TEST_SAMPLES);
	TEST_CHECK("B3-H03", c.centred >= B3_TEST_SAMPLES - B3_TEST_SAMPLES / 10,
		   "line between the sensors: line state CENTRED");

	/* B3-H07: steering response, motors off.  Car shifted so the line
	   is under one sensor: Buddy 3 must steer back towards it. */
	wait_for_placement("B3-H07", "line under IR_L only (car shifted right)");
	k = count_steer(B3_LINE_LEFT);
	tm_printf((UB*)"[TEST] B3-H07   IR_L on line: steer LEFT %d/%d\n",
		  k, B3_TEST_SAMPLES);
	ok = k >= B3_TEST_SAMPLES - B3_TEST_SAMPLES / 10;
	wait_for_placement("B3-H07", "line under IR_R only (car shifted left)");
	k = count_steer(B3_LINE_RIGHT);
	tm_printf((UB*)"[TEST] B3-H07   IR_R on line: steer RIGHT %d/%d\n",
		  k, B3_TEST_SAMPLES);
	ok = ok && (k >= B3_TEST_SAMPLES - B3_TEST_SAMPLES / 10);
	TEST_CHECK("B3-H07", ok, "drift onto the line: steer back towards it");

	follow_line_test("B3-H08", FALSE, &test_pass_, &test_fail_);
#endif /* B3_RUN_LINE_TESTS */

#if !B3_RUN_BARCODE_TESTS
	(void)snap;
	(void)seq;
	(void)ms;
	tm_printf((UB*)"[TEST] B3-H04..H06 skipped (B3_TEST_SELECT)\n");
#else

	/* B3-H04: push the robot by hand over a barcode.  Any symbol that
	   decodes has passed the '*' start/stop and width checks. */
	buddy3_get_snapshot(&snap);
	seq = snap.last_barcode.sequence;
	tm_printf((UB*)"[TEST] B3-H04 push the robot over a barcode, IR_B"
		  " over the bars (%d s)\n", B3_TEST_SCAN_MS / 1000);
	for (ms = 0; ms < B3_TEST_SCAN_MS; ms += 10) {
		set_context(FALSE, TRUE);
		tk_dly_tsk(10);
		buddy3_get_snapshot(&snap);
		if (snap.last_barcode.sequence != seq) {
			break;
		}
	}
	tm_printf((UB*)"[TEST] B3-H04   symbol '%c'%s  rejected scans %u"
		  "  edge faults %u\n",
		  (snap.last_barcode.sequence != seq) ?
			  snap.last_barcode.barcode.symbol : '-',
		  snap.last_barcode.barcode.reversed ? " (reversed)" : "",
		  (unsigned)snap.rejected_scans, (unsigned)snap.edge_faults);
	TEST_CHECK("B3-H04", snap.last_barcode.sequence != seq,
		   "a real barcode decodes on IR_B");

	/* B3-H05: raw edge log.  Push the robot over the barcode again; every
	   IR_B interrupt is recorded by the ISR (before any filtering) and
	   printed afterwards.  One Code 39 symbol "*X*" is 29 bars and gaps,
	   so a clean pass gives about 30 edges. */
	tm_printf((UB*)"[TEST] B3-H05 push over the barcode again,"
		  " one steady pass (%d s)\n", B3_TEST_SCAN_MS / 1000);
	b3_ir_edge_log_reset();
	run_with_context(B3_TEST_SCAN_MS, FALSE, TRUE);
	TEST_CHECK("B3-H05", print_edge_log("B3-H05") >= 30U,
		   "at least 30 IR_B edges in one barcode pass");

#if B3_USE_B2_ODOMETRY
	/* B3-H06: Buddy 2 drives the car straight over the barcode at
	   B3_SCAN_SPEED_MM_S while Buddy 3 scans with encoder positions.
	   Needs subsystem2 initialised and started, and MOTION_DEMO 0 so the
	   demo does not drive the car at the same time. */
#if MOTION_DEMO
	tm_printf((UB*)"[TEST] B3-H06 skipped: set MOTION_DEMO to 0 in"
		  " motion_control.h\n");
#else
	{
		motion_status_t	ms;
		motion_err_t	er;
		b3_context_t	ctx;
		INT		t;
		BOOL		moving = FALSE;
		INT		vmax_l = 0, vmax_r = 0;

		tm_printf((UB*)"[TEST] B3-H06 place the car on the floor, IR_B about"
			  " 50 mm before the barcode, %d mm clear ahead (%d s)\n",
			  B3_TEST_DRIVE_MM, B3_TEST_DRIVE_PLACE_MS / 1000);
		run_with_context(B3_TEST_DRIVE_PLACE_MS, FALSE, TRUE);
		buddy3_get_snapshot(&snap);
		seq = snap.last_barcode.sequence;
		b3_ir_edge_log_reset();

		(void)motion_reset_trip();
		er = motion_move(B3_TEST_DRIVE_MM, B3_SCAN_SPEED_MM_S);
		for (t = 0; (er == MOTION_OK) && (t < B3_TEST_DRIVE_MAX_MS); t += 10) {
			motion_get_status(&ms);
			ctx = (b3_context_t){0};
			ctx.time_us = b3_port_time_us();
			ctx.scan_enabled = true;
			ctx.motion_valid = true;
			ctx.left_measured_mm_s = ms.speed_l_mmps;
			ctx.right_measured_mm_s = ms.speed_r_mmps;
			buddy3_set_context(&ctx);
			if (ms.speed_l_mmps > vmax_l) vmax_l = ms.speed_l_mmps;
			if (ms.speed_r_mmps > vmax_r) vmax_r = ms.speed_r_mmps;
			if (ms.state == MOTION_STATE_DRIVING) {
				moving = TRUE;
			} else if (moving && (ms.state != MOTION_STATE_DRIVING)) {
				break;		/* move finished (or faulted) */
			}
			tk_dly_tsk(10);
		}
		run_with_context(500, FALSE, TRUE);	/* trailing quiet zone */
		(void)motion_stop();

		motion_get_status(&ms);
		buddy3_get_snapshot(&snap);
		tm_printf((UB*)"[TEST] B3-H06   move err %d result %d, %d mm in %d ms;"
			  " symbol '%c'%s  rejected scans %u\n",
			  (int)er, (int)ms.last_result, (int)ms.distance_trip_mm, (int)t,
			  (snap.last_barcode.sequence != seq) ?
				  snap.last_barcode.barcode.symbol : '-',
			  snap.last_barcode.barcode.reversed ? " (reversed)" : "",
			  (unsigned)snap.rejected_scans);
		/* Per-wheel check (Buddy 2's encoders): a wheel that drags while
		   its distance still matches the other one means that encoder
		   reports movement the wheel is not making. */
		tm_printf((UB*)"[TEST] B3-H06   wheels: left %d mm (peak %d mm/s),"
			  " right %d mm (peak %d mm/s), heading %d deg, fault %d\n",
			  (int)ms.dist_l_mm, vmax_l, (int)ms.dist_r_mm, vmax_r,
			  (int)ms.heading_deg, (int)ms.fault);
		(void)print_edge_log("B3-H06");
		TEST_CHECK("B3-H06", (er == MOTION_OK) &&
			   (snap.last_barcode.sequence != seq),
			   "barcode decodes while Buddy 2 drives over it");
	}
#endif /* MOTION_DEMO */
#endif /* B3_USE_B2_ODOMETRY */
#endif /* B3_RUN_BARCODE_TESTS */

#if B3_RUN_COMBINED_TEST
	/* B3-H09: IR_L/IR_R keep the car on the line while IR_B decodes the
	   barcode it passes. */
	follow_line_test("B3-H09", TRUE, &test_pass_, &test_fail_);
#endif

	*p_pass = test_pass_;
	*p_fail = test_fail_;
}
#endif /* B3_TEST_HW */

EXPORT void subsystem3_test(void)
{
	APP_STATS	before, after;

	TEST_INIT();

	/* INT-01 (integration, IF-01): samples from subsystem 2 arrive.
	   At one sample per 300 ms, 2 s should bring at least 5. */
	before = read_stats();
	tk_dly_tsk(2000);
	after = read_stats();
	TEST_CHECK("INT-01", after.received >= before.received + 5,
		   "samples flow from subsystem 2 to subsystem 3");

	/* S3-U01: the statistics are self-consistent */
	TEST_CHECK("S3-U01",
		   after.received > 0 && after.min <= after.max &&
		   after.min >= 0 && after.max <= 100,
		   "min <= max, both within 0..100");

	/* Buddy 3: line following and barcodes */
	buddy3_unit_tests(&test_pass_, &test_fail_);
#if B3_TEAM_AUTOSTART
	buddy3_task_tests(&test_pass_, &test_fail_);
#if B3_TEST_HW
	buddy3_hw_tests(&test_pass_, &test_fail_);
#endif
	/* Leave Buddy 3 failsafe: the context goes stale from here on */
#else
	tm_printf((UB*)"[TEST] B3-I/B3-H skipped: B3_TEAM_AUTOSTART is 0\n");
#endif

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
