/**
 * @file    s5_calibrate.c
 * @brief   v3.0 guided calibration of the servo, the beam and the range.
 *
 * Built always, started only when S5_CAL_BUILD = 1 (s5_config.h), in
 * place of the obstacle task: it owns the same pins. Follow the serial
 * monitor; the full guide with photos-worth of detail is
 * testing/subsystem5/CALIBRATION.md.
 *
 *   STEP 1  servo trim, backlash, beam half-angle
 *           A thin post (pen, straw) on the car's centreline, 300 mm in
 *           front of the sensor face. The servo creeps 1 deg at a time
 *           from 60 to 120 deg and back, twice. The post answers over an
 *           arc; the middle of the arc is "straight ahead" in servo
 *           units. Coming from the right and from the left the middles
 *           differ by twice the backlash. Half the arc, less the post's
 *           own angular size, is the beam half-angle.
 *   STEP 2  range scale and offset
 *           A flat board square to the sensor at 100, 200, 300, 500 mm
 *           (tape measure from the sensor face). 20 pings each; a least
 *           squares line through the four medians gives scale + offset.
 *           Each point is checked against NFR15 (+/-10 mm).
 *   STEP 3  verify
 *           The new values are switched on live. Post back at 300 mm:
 *           the servo arc must now be centred on 90 deg from both sides,
 *           and the real S5 sweep + geometry must put the post at 0 deg.
 *
 * Then it prints a block for s5_calib.h, again every 15 s.
 *
 * Not on the control path, so printf and 64-bit maths are fine here.
 */

#include "subsystem5.h"

#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_calib.h"
#include "s5_config.h"

#define CAL_POST_MM             (300)   /* step 1 and 3 post distance   */
#define CAL_POST_WINDOW_MM      (100)   /* a hit is within +/- this     */
#define CAL_SWEEP_LO_DDEG       (DRV_SERVO_MIN_DEG * 10)
#define CAL_SWEEP_HI_DDEG       (DRV_SERVO_MAX_DEG * 10)
#define CAL_STEP_DDEG           (10)    /* 1 deg                         */
#define CAL_PASSES              (2)
#define CAL_SETUP_S             (20)    /* time to place things          */
#define CAL_RANGE_PINGS         (20U)
#define CAL_RANGE_POINTS        (4U)
#define CAL_NFR15_MM            (10)

static const int32_t g_range_true_mm[CAL_RANGE_POINTS] = { 100, 200, 300, 500 };

static s5_cal_result_t g_result;    /* written by the task, read by tests */

EXPORT void s5_cal_get_result(s5_cal_result_t *p_out)
{
    if (p_out != NULL)
    {
        *p_out = g_result;
    }
}

typedef struct
{
    int32_t first_ddeg;   /* first commanded angle that heard the post */
    int32_t last_ddeg;    /* last one                                  */
    bool    found;
} cal_arc_t;

/*=====================================================================*/
/* Helpers                                                             */
/*=====================================================================*/

static void cal_countdown(const char *p_what, INT seconds)
{
    INT n;

    tm_printf((UB *)"\n>>> %s\n", p_what);

    for (n = seconds; n > 0; n--)
    {
        if (((n % 5) == 0) || (n <= 3))
        {
            tm_printf((UB *)"    starting in %d s\n", n);
        }
        tk_dly_tsk(1000);
    }
}

/** One ping; raw (uncalibrated) mm if valid, else -1. */
static int32_t cal_ping_raw(void)
{
    ultrasonic_result_t       res;
    const ultrasonic_status_t st = drv_ultrasonic_ping(&res);
    int32_t                   mm = -1;

    if (st == ULTRASONIC_OK)
    {
        mm = (int32_t)drv_us_raw_mm(drv_us_result_echo_us(res));
    }

    return mm;
}

/** One ping; calibrated mm if valid, else -1. */
static int32_t cal_ping_mm(void)
{
    ultrasonic_result_t       res;
    const ultrasonic_status_t st = drv_ultrasonic_ping(&res);

    return (st == ULTRASONIC_OK) ? (int32_t)drv_us_result_mm(res) : -1;
}

static void cal_sort(int32_t *p, uint32_t n)
{
    uint32_t a;

    for (a = 1U; a < n; a++)
    {
        const int32_t key = p[a];
        uint32_t      b   = a;

        while ((b > 0U) && (p[b - 1U] > key))
        {
            p[b] = p[b - 1U];
            b--;
        }
        p[b] = key;
    }
}

/** Integer rounding division for signed values. */
static int32_t cal_div_round(int64_t num, int64_t den)
{
    if (den < 0)
    {
        num = -num;
        den = -den;
    }

    return (int32_t)((num >= 0) ? ((num + (den / 2)) / den)
                                : ((num - (den / 2)) / den));
}

/**
 * Creep the servo from `from` to `to` (0.1 deg, physical when raw,
 * logical otherwise) and record where the post answers.
 */
static cal_arc_t cal_sweep(int32_t from, int32_t to, bool raw)
{
    cal_arc_t     arc  = { 0, 0, false };
    const int32_t step = (to > from) ? CAL_STEP_DDEG : -CAL_STEP_DDEG;
    int32_t       a    = from;
    bool          done = false;

    /* Park at the start and let it settle fully. */
    if (raw)
    {
        (void)drv_servo_set_raw_ddeg((int16_t)from);
    }
    else
    {
        (void)drv_servo_set_angle((int16_t)(from / 10));
    }
    tk_dly_tsk(400);

    while (!done)
    {
        int32_t mm;

        if (raw)
        {
            (void)drv_servo_set_raw_ddeg((int16_t)a);
        }
        else
        {
            (void)drv_servo_set_angle((int16_t)(a / 10));
        }
        tk_dly_tsk(40);     /* 1 deg: ~2 ms travel + ringing */

        mm = raw ? cal_ping_raw() : cal_ping_mm();

        if ((mm > 0) && (mm > (CAL_POST_MM - CAL_POST_WINDOW_MM)) &&
            (mm < (CAL_POST_MM + CAL_POST_WINDOW_MM)))
        {
            if (!arc.found)
            {
                arc.first_ddeg = a;
                arc.found      = true;
            }
            arc.last_ddeg = a;
        }

        if (a == to)
        {
            done = true;
        }
        else
        {
            a += step;
        }
    }

    return arc;
}

static int32_t cal_arc_mid(const cal_arc_t *p)
{
    return (p->first_ddeg + p->last_ddeg) / 2;
}

static int32_t cal_arc_width(const cal_arc_t *p)
{
    const int32_t w = p->last_ddeg - p->first_ddeg;

    return (w < 0) ? -w : w;
}

/** Median of N calibrated-or-raw pings, and their spread (max - min). */
static bool cal_range_point(bool raw, int32_t *p_median, int32_t *p_spread)
{
    int32_t  v[CAL_RANGE_PINGS];
    uint32_t n = 0U;
    uint32_t i;

    for (i = 0U; i < CAL_RANGE_PINGS; i++)
    {
        const int32_t mm = raw ? cal_ping_raw() : cal_ping_mm();

        if (mm > 0)
        {
            v[n] = mm;
            n++;
        }
    }

    if (n >= (CAL_RANGE_PINGS / 2U))
    {
        cal_sort(v, n);
        *p_median = v[n / 2U];
        /* Spread of the middle 80 %: one stray echo is not a fail. */
        *p_spread = v[n - 1U - (n / 10U)] - v[n / 10U];
    }

    return (n >= (CAL_RANGE_PINGS / 2U));
}

/*=====================================================================*/
/* The task                                                            */
/*=====================================================================*/

EXPORT void s5_calibrate_task(INT stacd, void *exinf)
{
    cal_arc_t up[CAL_PASSES];
    cal_arc_t dn[CAL_PASSES];
    int32_t   c_up = 0;
    int32_t   c_dn = 0;
    int32_t   w_sum = 0;
    int32_t   trim;
    int32_t   backlash;
    int32_t   beam;
    int32_t   post_half;
    int32_t   raw_med[CAL_RANGE_POINTS];
    int32_t   scale_q12 = 4096;
    int32_t   offset_mm = 0;
    bool      step1_ok = true;
    bool      step2_ok = true;
    bool      step3_ok = true;
    INT       i;

    (void)stacd;
    (void)exinf;

    if (!drv_servo_init() || !drv_ultrasonic_init())
    {
        tm_printf((UB *)"S5 CAL: driver init failed\n");
        for (;;)
        {
            tk_dly_tsk(1000);
        }
    }

    tm_printf((UB *)"\n=== S5 CALIBRATION v3.0 (servo GP15, sonar GP16/17) ===\n"
              "Guide: testing/subsystem5/CALIBRATION.md\n"
              "Keep the car still and the area around the post empty.\n");

    /*---------------------------------------------------------------*/
    /* STEP 1: trim, backlash, beam                                  */
    /*---------------------------------------------------------------*/
    cal_countdown("STEP 1/3: put a THIN post (pen) on the car's centre line, "
                  "300 mm from the sensor face", CAL_SETUP_S);

    for (i = 0; i < CAL_PASSES; i++)
    {
        up[i] = cal_sweep(CAL_SWEEP_LO_DDEG, CAL_SWEEP_HI_DDEG, true);
        dn[i] = cal_sweep(CAL_SWEEP_HI_DDEG, CAL_SWEEP_LO_DDEG, true);

        tm_printf((UB *)"  pass %d  up: %s %d..%d   down: %s %d..%d (0.1 deg)\n",
                  i + 1,
                  up[i].found ? "post" : "NONE", (INT)up[i].first_ddeg,
                  (INT)up[i].last_ddeg,
                  dn[i].found ? "post" : "NONE", (INT)dn[i].first_ddeg,
                  (INT)dn[i].last_ddeg);

        step1_ok = step1_ok && up[i].found && dn[i].found;
    }

    if (step1_ok)
    {
        for (i = 0; i < CAL_PASSES; i++)
        {
            c_up  += cal_arc_mid(&up[i]);
            c_dn  += cal_arc_mid(&dn[i]);
            w_sum += cal_arc_width(&up[i]) + cal_arc_width(&dn[i]);
        }
        c_up /= CAL_PASSES;
        c_dn /= CAL_PASSES;
        w_sum /= (2 * CAL_PASSES);

        trim     = ((c_up + c_dn) / 2) - 900;
        backlash = (c_up - c_dn) / 2;
        /* The post's own half-width seen from 300 mm, in 0.1 deg:
         * (D/2)/d rad * 573. */
        post_half = cal_div_round((int64_t)S5_CAL_POST_DIAMETER_MM * 573,
                                  (int64_t)2 * CAL_POST_MM);
        beam      = (w_sum / 2) - post_half;

        if (backlash < 0)
        {
            tm_printf((UB *)"  note: negative backlash %d, using 0\n",
                      (INT)backlash);
            backlash = 0;
        }

        if ((trim > 300) || (trim < -300) || (backlash > 80) ||
            (beam < 20) || (beam > 300))
        {
            tm_printf((UB *)"  STEP 1 FAIL: trim %d, backlash %d, beam %d "
                      "out of range. Post too wide, not centred, or "
                      "something else in the beam?\n",
                      (INT)trim, (INT)backlash, (INT)beam);
            step1_ok = false;
        }
    }
    else
    {
        tm_printf((UB *)"  STEP 1 FAIL: post not heard on every sweep\n");
    }

    if (!step1_ok)
    {
        trim     = (int32_t)S5_CAL_SERVO_TRIM_DDEG;
        backlash = (int32_t)S5_CAL_SERVO_BACKLASH_DDEG;
        beam     = (int32_t)S5_CAL_BEAM_HALF_DDEG;
    }
    else
    {
        tm_printf((UB *)"  straight ahead = %d (0.1 deg) -> trim %d, "
                  "backlash %d, beam half-angle %d (0.1 deg)\n",
                  (INT)(900 + trim), (INT)trim, (INT)backlash, (INT)beam);
    }

    drv_servo_set_cal((int16_t)trim, (int16_t)backlash);

    /*---------------------------------------------------------------*/
    /* STEP 2: range                                                 */
    /*---------------------------------------------------------------*/
    (void)drv_servo_set_angle((int16_t)DRV_SERVO_CENTRE_DEG);
    tk_dly_tsk(400);

    for (i = 0; i < (INT)CAL_RANGE_POINTS; i++)
    {
        int32_t spread = 0;

        tm_printf((UB *)"\n>>> STEP 2/3: flat board, square to the sensor, "
                  "%d mm from the sensor face\n", (INT)g_range_true_mm[i]);
        cal_countdown("place the board now", CAL_SETUP_S);

        if (!cal_range_point(true, &raw_med[i], &spread))
        {
            tm_printf((UB *)"  no stable echo at %d mm\n",
                      (INT)g_range_true_mm[i]);
            step2_ok = false;
            raw_med[i] = g_range_true_mm[i];
        }
        else
        {
            tm_printf((UB *)"  true %d mm: raw median %d mm (error %d), "
                      "spread %d mm\n", (INT)g_range_true_mm[i],
                      (INT)raw_med[i], (INT)(raw_med[i] - g_range_true_mm[i]),
                      (INT)spread);

            if (spread > CAL_NFR15_MM)
            {
                tm_printf((UB *)"  WARNING: spread over %d mm; board not "
                          "square or something moving\n", CAL_NFR15_MM);
            }
        }
    }

    if (step2_ok)
    {
        /* Least squares true = a * raw + b, exact in 64-bit integers. */
        int64_t sx = 0;
        int64_t sy = 0;
        int64_t sxx = 0;
        int64_t sxy = 0;
        const int64_t n = (int64_t)CAL_RANGE_POINTS;
        int64_t den;

        for (i = 0; i < (INT)CAL_RANGE_POINTS; i++)
        {
            sx  += raw_med[i];
            sy  += g_range_true_mm[i];
            sxx += (int64_t)raw_med[i] * raw_med[i];
            sxy += (int64_t)raw_med[i] * g_range_true_mm[i];
        }

        den = (n * sxx) - (sx * sx);

        if (den != 0)
        {
            scale_q12 = cal_div_round(((n * sxy) - (sx * sy)) * 4096, den);
            offset_mm = cal_div_round((sy * 4096) - ((int64_t)scale_q12 * sx),
                                      n * 4096);
        }

        if ((scale_q12 < 3686) || (scale_q12 > 4506) ||
            (offset_mm < -100) || (offset_mm > 100))
        {
            tm_printf((UB *)"  STEP 2 FAIL: scale %d/4096, offset %d mm is "
                      "not a sensor error. Re-measure the distances.\n",
                      (INT)scale_q12, (INT)offset_mm);
            step2_ok  = false;
            scale_q12 = (int32_t)S5_CAL_RANGE_SCALE_Q12;
            offset_mm = (int32_t)S5_CAL_RANGE_OFFSET_MM;
        }
    }

    drv_us_set_cal(scale_q12, offset_mm);

    if (step2_ok)
    {
        tm_printf((UB *)"  fit: true = raw * %d / 4096 + (%d) mm\n",
                  (INT)scale_q12, (INT)offset_mm);

        for (i = 0; i < (INT)CAL_RANGE_POINTS; i++)
        {
            const int32_t fit = ((raw_med[i] * scale_q12) / 4096) + offset_mm;
            const int32_t err = fit - g_range_true_mm[i];
            const bool    ok  = (err <= CAL_NFR15_MM) && (err >= -CAL_NFR15_MM);

            tm_printf((UB *)"  %d mm -> %d mm after fit (error %d) %s\n",
                      (INT)g_range_true_mm[i], (INT)fit, (INT)err,
                      ok ? "PASS NFR15" : "FAIL NFR15");
            step2_ok = step2_ok && ok;
        }
    }

    /*---------------------------------------------------------------*/
    /* STEP 3: verify with the new values switched on                */
    /*---------------------------------------------------------------*/
    cal_countdown("STEP 3/3: post back on the centre line at 300 mm "
                  "(verify)", CAL_SETUP_S);
    {
        const cal_arc_t vu = cal_sweep(CAL_SWEEP_LO_DDEG, CAL_SWEEP_HI_DDEG,
                                       false);
        const cal_arc_t vd = cal_sweep(CAL_SWEEP_HI_DDEG, CAL_SWEEP_LO_DDEG,
                                       false);
        static const int8_t b[7] = { -30, -20, -10, 0, 10, 20, 30 };
        s5_sample_t         samples[7];
        obstacle_profile_t  p;
        const s5_geom_cal_t gc = { (int16_t)beam, 10,
                                   (int16_t)S5_SWEEP_LIMIT_DEG,
                                   (uint16_t)OBSTACLE_TRIGGER_MM };
        uint8_t             k;

        if (vu.found && vd.found)
        {
            const int32_t eu = cal_arc_mid(&vu) - 900;
            const int32_t ed = cal_arc_mid(&vd) - 900;

            tm_printf((UB *)"  servo: post centred at %d (from right) and "
                      "%d (from left), 0.1 deg off straight\n",
                      (INT)eu, (INT)ed);
            /* Whole-degree command steps: within 1.5 deg is the best
             * the 1 deg sweep can show. */
            step3_ok = (eu <= 15) && (eu >= -15) && (ed <= 15) && (ed >= -15);
        }
        else
        {
            tm_printf((UB *)"  servo: post not heard\n");
            step3_ok = false;
        }

        for (k = 0U; k < 7U; k++)
        {
            const int16_t from = drv_servo_get_angle();
            const int16_t to   = (int16_t)(DRV_SERVO_CENTRE_DEG + b[k]);
            ultrasonic_result_t res;
            ultrasonic_status_t st;
            uint16_t            r;

            (void)drv_servo_set_angle(to);
            tk_dly_tsk((RELTIM)drv_servo_settle_ms(from, to));
            st = drv_ultrasonic_ping(&res);
            r  = (st == ULTRASONIC_OK) ? drv_us_result_mm(res)
                                       : OBSTACLE_RANGE_CLEAR;
            samples[k] = s5_sample_pack(b[k], r);
        }

        obstacle_profile_build(samples, 7U, &p);
        obstacle_profile_refine(&p, &gc, NULL);

        tm_printf((UB *)"  S5 sweep: nearest %d mm, centre %d deg, width "
                  "%d mm (post %d mm)\n", (INT)p.nearest_mm,
                  (INT)p.bearing_deg, (INT)p.width_mm,
                  (INT)S5_CAL_POST_DIAMETER_MM);

        step3_ok = step3_ok && p.valid &&
                   (p.bearing_deg <= 2) && (p.bearing_deg >= -2);
        g_result.verify_bearing_deg = p.bearing_deg;
    }

    (void)drv_servo_set_angle((int16_t)DRV_SERVO_CENTRE_DEG);

    g_result.step1_ok       = step1_ok;
    g_result.step2_ok       = step2_ok;
    g_result.step3_ok       = step3_ok;
    g_result.trim_ddeg      = (int16_t)trim;
    g_result.backlash_ddeg  = (int16_t)backlash;
    g_result.beam_half_ddeg = (int16_t)beam;
    g_result.scale_q12      = scale_q12;
    g_result.offset_mm      = offset_mm;
    g_result.finished       = true;

    /*---------------------------------------------------------------*/
    /* Result, repeated so a monitor opened late still sees it       */
    /*---------------------------------------------------------------*/
    for (;;)
    {
        tm_printf((UB *)"\n=== S5 CALIBRATION RESULT: step1 %s, step2 %s, "
                  "step3 %s ===\n", step1_ok ? "PASS" : "FAIL",
                  step2_ok ? "PASS" : "FAIL", step3_ok ? "PASS" : "FAIL");
        tm_printf((UB *)"---- paste into app_program/subsystem5/s5_calib.h ----\n");
        tm_printf((UB *)"#define S5_CAL_DONE                 (%d)\n",
                  (step1_ok && step2_ok) ? 1 : 0);
        tm_printf((UB *)"#define S5_CAL_SERVO_TRIM_DDEG      (%d)\n", (INT)trim);
        tm_printf((UB *)"#define S5_CAL_SERVO_BACKLASH_DDEG  (%d)\n",
                  (INT)backlash);
        tm_printf((UB *)"#define S5_CAL_RANGE_SCALE_Q12      (%d)\n",
                  (INT)scale_q12);
        tm_printf((UB *)"#define S5_CAL_RANGE_OFFSET_MM      (%d)\n",
                  (INT)offset_mm);
        tm_printf((UB *)"#define S5_CAL_BEAM_HALF_DDEG       (%d)\n", (INT)beam);
        tm_printf((UB *)"/* S5_CAL_SENSOR_ARM_MM: measure with a ruler */\n");
        tm_printf((UB *)"---- then set S5_CAL_BUILD to 0, build, flash ----\n");
        tk_dly_tsk(15000);
    }
}
