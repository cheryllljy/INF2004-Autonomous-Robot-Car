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
#include "../test_report.h"

#if APP_TEST

#ifndef B3_TEST_HW
#define B3_TEST_HW		1	/* 1 = run the B3-H sensor tests */
#endif
#define B3_TEST_PLACE_MS	5000	/* time to place the robot for a B3-H test */
#define B3_TEST_SAMPLES		100	/* sensor samples per B3-H test, 10 ms apart */
#define B3_TEST_SCAN_MS		15000	/* time allowed to push the robot over a barcode */
#define B3_ADC_MID		2048	/* half of the 12-bit ADC range */

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
/* Read one ADC channel through the kernel ADC driver ("adca") */
LOCAL INT adc_read(ID dd, UW ch)
{
	UW	val = 0;
	SZ	asz = 0;
	ER	er;

	er = tk_srea_dev(dd, (W)ch, &val, 1, &asz);
	return (er < E_OK) ? (INT)er : (INT)(val & 0x0FFF);
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

LOCAL void sample_ir(ID dd, IR_COUNTS *p)
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
			a = adc_read(dd, ch[k]);
			if (a < p->adc_min[k]) p->adc_min[k] = a;
			if (a > p->adc_max[k]) p->adc_max[k] = a;
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

	/* B3-U08..U11: line rules (IR_L/IR_R sensors, 10 ms steps) */
	b3_line_init(&line, 0U);
	r = b3_line_step(&line, true, true, 0U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, true, true, 10000U, B3_BASE_SPEED_MM_S);
	TEST_CHECK("B3-U08",
		   r.state == B3_LINE_CENTRED && r.error == 0 &&
		   r.left_mm_s == r.right_mm_s && !r.stop,
		   "IR_L and IR_R black: centred, equal wheel speeds");

	r = b3_line_step(&line, true, false, 20000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, true, false, 30000U, B3_BASE_SPEED_MM_S);
	TEST_CHECK("B3-U09",
		   r.state == B3_LINE_LEFT && r.error < 0 &&
		   r.left_mm_s < r.right_mm_s && r.left_mm_s >= 0,
		   "line under IR_L: steer left, no wheel reverses");

	r = b3_line_step(&line, false, true, 40000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, false, true, 50000U, B3_BASE_SPEED_MM_S);
	TEST_CHECK("B3-U10",
		   r.state == B3_LINE_RIGHT && r.error > 0 &&
		   r.right_mm_s < r.left_mm_s && r.right_mm_s >= 0,
		   "line under IR_R: steer right, no wheel reverses");

	r = b3_line_step(&line, false, false, 60000U, B3_BASE_SPEED_MM_S);
	r = b3_line_step(&line, false, false, 70000U, B3_BASE_SPEED_MM_S);
	ok = (r.state == B3_LINE_LOST) && !r.stop &&
	     r.left_mm_s <= B3_RECOVERY_SPEED_MM_S + B3_CORRECTION_MM_S;
	r = b3_line_step(&line, false, false,
			 70000U + B3_LINE_LOST_GRACE_US, B3_BASE_SPEED_MM_S);
	ok = ok && r.stop && r.recovery_needed;
	TEST_CHECK("B3-U11", ok, "line lost: slow search, then stop after the grace time");

	*p_pass = test_pass_;
	*p_fail = test_fail_;
}

LOCAL void buddy3_task_tests(INT *p_pass, INT *p_fail)
{
	b3_snapshot_t	a, b;
	INT test_pass_ = *p_pass, test_fail_ = *p_fail;

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
	   longest gap seen so far must stay within one extra tick. */
	tm_printf((UB*)"[TEST] B3-I06   longest service gap %u us (period %d ms)\n",
		  (unsigned)b.max_service_gap_us, B3_SERVICE_DELAY_MS);
	TEST_CHECK("B3-I06", b.max_service_gap_us <= 2U * B3_SERVICE_DELAY_MS * 1000U,
		   "IR sampling period within 20 ms worst case");

	*p_pass = test_pass_;
	*p_fail = test_fail_;
}

#if B3_TEST_HW
LOCAL void buddy3_hw_tests(INT *p_pass, INT *p_fail)
{
	IR_COUNTS	c;
	b3_snapshot_t	snap;
	UW		seq;
	ID		dd;
	INT		ms, k;
	BOOL		ok;
	INT test_pass_ = *p_pass, test_fail_ = *p_fail;

	dd = tk_opn_dev((UB*)"adca", TD_READ);
	TEST_CHECK("B3-H00", dd >= E_OK, "ADC driver opens (adca)");
	if (dd < E_OK) {
		*p_pass = test_pass_;
		*p_fail = test_fail_;
		return;
	}

	/* B3-H01: all sensors on white.  Each DO reads stable white and the
	   ADC agrees with the digital level (sensor self-check) */
	wait_for_placement("B3-H01", "all three sensors over WHITE");
	sample_ir(dd, &c);
	print_ir("B3-H01", &c);
	ok = TRUE;
	for (k = 0; k < 3; k++) {
		ok = ok && c.black[k] <= B3_TEST_SAMPLES / 20 &&
		     c.mismatch[k] <= B3_TEST_SAMPLES / 20;
	}
	TEST_CHECK("B3-H01", ok, "white: every sensor reads white, ADC agrees");

	/* B3-H02: all sensors on black (e.g. a black card or junction) */
	wait_for_placement("B3-H02", "all three sensors over BLACK");
	sample_ir(dd, &c);
	print_ir("B3-H02", &c);
	ok = TRUE;
	for (k = 0; k < 3; k++) {
		ok = ok && c.black[k] >= B3_TEST_SAMPLES - B3_TEST_SAMPLES / 20 &&
		     c.mismatch[k] <= B3_TEST_SAMPLES / 20;
	}
	TEST_CHECK("B3-H02", ok, "black: every sensor reads black, ADC agrees");

	/* B3-H03: robot centred on the line */
	wait_for_placement("B3-H03", "centred on the black line");
	sample_ir(dd, &c);
	print_ir("B3-H03", &c);
	tm_printf((UB*)"[TEST] B3-H03   line state CENTRED %d/%d\n",
		  c.centred, B3_TEST_SAMPLES);
	TEST_CHECK("B3-H03", c.centred >= B3_TEST_SAMPLES - B3_TEST_SAMPLES / 10,
		   "on the line: line state CENTRED");

	(void)tk_cls_dev(dd, 0);

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
