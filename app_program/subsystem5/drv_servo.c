/**
 * @file    drv_servo.c
 * @brief   RC servo on PWM slice 7 channel B (GP15), register level.
 *
 * Timing: 50 Hz frame, 500..2500 us pulse. The slice counts at exactly
 * 1 MHz (divider = SYSCLK in MHz), so one count is one microsecond and
 * the compare register is loaded with the pulse width directly.
 * No floating point.
 *
 * CPU cost: drv_servo_set_angle() is one multiply, one divide and one
 * register write. It never blocks.
 */

#include "drv_servo.h"

#include "drv_pwm_s5.h"
#include "s5_calib.h"
#include "s5_config.h"
#include "s5_pack.h"

#include <bsp/libbsp.h>

/*=====================================================================*/
/* Tunable constants                                                   */
/*=====================================================================*/

/** Set to 1 if increasing angle sweeps the wrong way.
 *  nyx, 2026-11-06: check after re-mounting the sensor. Put a box at the
 *  front RIGHT of the car: the "[VC-OBS] scan" line must show a negative
 *  centre bearing (+ = left). If it is positive, set this to 1 (or build
 *  with -DDRV_SERVO_INVERT=1), then re-run calibration step 1. */
#ifndef DRV_SERVO_INVERT
#define DRV_SERVO_INVERT (0)
#endif

/** 20000 counts of 1 us = 50 Hz. */
#define DRV_SERVO_PERIOD_US (20000U)

/** Pulse at 0 and 180 deg. Calibrate per servo. */
#define DRV_SERVO_PULSE_MIN_US (500U)
#define DRV_SERVO_PULSE_MAX_US (2500U)
#define DRV_SERVO_SPAN_DEG     (180)

/**
 * SG90 datasheet: 0.1 s / 60 deg unloaded at 4.8 V. The sonar bracket
 * loads it, so 140 ms / 60 deg is the worst case.
 * TODO(measure): re-run bench test A5b on the power bank.
 */
#define DRV_SERVO_TRAVEL_MS_PER_60 (140U)

/** Fixed allowance for overshoot ringing after the horn arrives. */
#define DRV_SERVO_DAMPING_MS (25U)

/* The pin must really be slice 7 channel B, or the registers below
 * drive the wrong output. */
S5_STATIC_ASSERT(S5_PWM_SLICE_OF(S5_SERVO_GPIO) == S5_SERVO_PWM_SLICE,
                 servo_on_slice_7);
S5_STATIC_ASSERT(S5_PWM_IS_CHAN_B(S5_SERVO_GPIO), servo_on_channel_b);

/*=====================================================================*/
/* Module state (owned by the obstacle task only)                      */
/*=====================================================================*/

static int16_t g_angle_deg = DRV_SERVO_CENTRE_DEG;
static bool g_ready = false;

/** v3.0 backlash: direction of the last move, +1 left, -1 right. The
 *  correction keeps its sign while the horn holds still, because the
 *  gears stay where the last move left them. */
static int8_t g_last_dir = 0;

/** Live calibration. Start from s5_calib.h; only the calibration build
 *  changes them (drv_servo_set_cal), to verify new values at once. */
static int16_t g_trim_ddeg = (int16_t)S5_CAL_SERVO_TRIM_DDEG;
static int16_t g_backlash_ddeg = (int16_t)S5_CAL_SERVO_BACKLASH_DDEG;

/* A calibration that would push the 60..120 deg window past the
 * physical 0..180 range is a typo, not a measurement. */
S5_STATIC_ASSERT((S5_CAL_SERVO_TRIM_DDEG <= 300) &&
                     (S5_CAL_SERVO_TRIM_DDEG >= -300),
                 servo_trim_sane);
S5_STATIC_ASSERT((S5_CAL_SERVO_BACKLASH_DDEG >= 0) &&
                     (S5_CAL_SERVO_BACKLASH_DDEG <= 80),
                 servo_backlash_sane);

/*=====================================================================*/
/* Private helpers                                                     */
/*=====================================================================*/

/** Pulse for a PHYSICAL angle in 0.1 deg (no trim, no backlash). One
 *  count is 1 us = 0.09 deg, so 0.1 deg steps are real. */
static UW drv_servo_ddeg_to_us(int32_t ddeg)
{
    int32_t effective = ddeg;
    int32_t span_us;

    if (effective < 0)
    {
        effective = 0;
    }
    else if (effective > ((int32_t)DRV_SERVO_SPAN_DEG * 10))
    {
        effective = (int32_t)DRV_SERVO_SPAN_DEG * 10;
    }
    else
    {
        /* In range. */
    }

#if (DRV_SERVO_INVERT != 0)
    effective = ((int32_t)DRV_SERVO_SPAN_DEG * 10) - effective;
#endif

    span_us = (int32_t)DRV_SERVO_PULSE_MAX_US - (int32_t)DRV_SERVO_PULSE_MIN_US;

    return (UW)((int32_t)DRV_SERVO_PULSE_MIN_US +
                ((effective * span_us) / ((int32_t)DRV_SERVO_SPAN_DEG * 10)));
}

/** Logical angle -> physical command with the live calibration. */
static int32_t drv_servo_corrected_ddeg(int32_t angle_ddeg, int8_t dir)
{
    return drv_servo_correct_ddeg(angle_ddeg, dir, g_trim_ddeg,
                                  g_backlash_ddeg);
}

/** Last logical command in 0.1 deg, for the direction of travel. */
static int32_t g_logical_ddeg = (int32_t)DRV_SERVO_CENTRE_DEG * 10;

/*=====================================================================*/
/* Public interface                                                    */
/*=====================================================================*/

bool drv_servo_init(void)
{
    const UW slice = (UW)S5_SERVO_PWM_SLICE;

    s5_pwm_block_ready();

    /* Slice off while it is configured. */
    out_w(S5_PWM_CSR(slice), 0U);

    /* 1 us per count: integer divider = SYSCLK (MHz), no fraction, so
     * the pulse width cannot drift. */
    out_w(S5_PWM_DIV(slice), S5_PWM_DIV_1MHZ << S5_PWM_DIV_INT_SHIFT);
    out_w(S5_PWM_CTR(slice), 0U);

    (void)pwm_set_wrap((UINT)S5_SERVO_GPIO, (UW)(DRV_SERVO_PERIOD_US - 1U));

    g_angle_deg = (int16_t)DRV_SERVO_CENTRE_DEG;
    g_last_dir = 0;
    g_logical_ddeg = (int32_t)g_angle_deg * 10;
    (void)pwm_set_cc((UINT)S5_SERVO_GPIO,
                     drv_servo_ddeg_to_us(
                         drv_servo_corrected_ddeg(g_logical_ddeg, g_last_dir)));

    /* Route the pin to the PWM block only after the compare value is
     * set, so the servo never sees a garbage first pulse. */
    (void)pwm_set_pin((UINT)S5_SERVO_GPIO);
    (void)pwm_set_enabled((UINT)S5_SERVO_GPIO, TRUE);

    g_ready = true;

    return g_ready;
}

bool drv_servo_set_angle_ddeg(int16_t ddeg)
{
    bool exact = false;
    int32_t c = (int32_t)ddeg;

    if (c < ((int32_t)DRV_SERVO_MIN_DEG * 10))
    {
        c = (int32_t)DRV_SERVO_MIN_DEG * 10;
    }
    else if (c > ((int32_t)DRV_SERVO_MAX_DEG * 10))
    {
        c = (int32_t)DRV_SERVO_MAX_DEG * 10;
    }
    else
    {
        /* Inside the limits. */
    }

    if (g_ready)
    {
        if (c > g_logical_ddeg)
        {
            g_last_dir = 1;
        }
        else if (c < g_logical_ddeg)
        {
            g_last_dir = -1;
        }
        else
        {
            /* Same angle: the gears have not moved. */
        }

        (void)pwm_set_cc(
            (UINT)S5_SERVO_GPIO,
            drv_servo_ddeg_to_us(drv_servo_corrected_ddeg(c, g_last_dir)));
        g_logical_ddeg = c;
        g_angle_deg = (int16_t)((c + 5) / 10);
        exact = (c == (int32_t)ddeg);
    }

    return exact;
}

bool drv_servo_set_angle(int16_t angle_deg)
{
    int32_t d = (int32_t)angle_deg * 10;
    const bool in_range = (d >= ((int32_t)DRV_SERVO_MIN_DEG * 10)) &&
                          (d <= ((int32_t)DRV_SERVO_MAX_DEG * 10));

    /* Keep the 0.1 deg value inside int16 whatever the caller sent;
     * set_angle_ddeg clamps to the bracket limits. */
    if (d < 0)
    {
        d = 0;
    }
    if (d > 1800)
    {
        d = 1800;
    }

    return drv_servo_set_angle_ddeg((int16_t)d) && in_range;
}

bool drv_servo_set_raw_ddeg(int16_t ddeg)
{
    bool exact = false;
    int16_t c = ddeg;

    /* Same bracket limits as the logical command. */
    if (c < (int16_t)(DRV_SERVO_MIN_DEG * 10))
    {
        c = (int16_t)(DRV_SERVO_MIN_DEG * 10);
    }
    else if (c > (int16_t)(DRV_SERVO_MAX_DEG * 10))
    {
        c = (int16_t)(DRV_SERVO_MAX_DEG * 10);
    }
    else
    {
        /* Inside the limits. */
    }

    if (g_ready)
    {
        (void)pwm_set_cc((UINT)S5_SERVO_GPIO, drv_servo_ddeg_to_us(c));
        g_angle_deg = (int16_t)(c / 10);
        g_logical_ddeg = (int32_t)c - (int32_t)g_trim_ddeg;
        g_last_dir = 0;
        exact = (c == ddeg);
    }

    return exact;
}

int16_t drv_servo_get_angle(void)
{
    return g_angle_deg;
}

/* nyx, 2026-11-06: for the smooth sweep (obstacle_task.c). */
int16_t drv_servo_get_angle_ddeg(void)
{
    return (int16_t)g_logical_ddeg;
}

uint32_t drv_servo_settle_ms(int16_t from_deg, int16_t to_deg)
{
    int32_t delta = (int32_t)to_deg - (int32_t)from_deg;

    if (delta < 0)
    {
        delta = -delta;
    }

    return (((uint32_t)delta * DRV_SERVO_TRAVEL_MS_PER_60) / 60U) +
           DRV_SERVO_DAMPING_MS;
}

void drv_servo_release(void)
{
    if (g_ready)
    {
        /* Compare 0 holds the output low: no pulse, servo goes limp. */
        (void)pwm_set_cc((UINT)S5_SERVO_GPIO, 0U);
    }
}

void drv_servo_set_cal(int16_t trim_ddeg, int16_t backlash_ddeg)
{
    int16_t t = trim_ddeg;
    int16_t b = backlash_ddeg;

    /* Same sanity window as the compile-time checks. */
    if (t > 300)
    {
        t = 300;
    }
    if (t < -300)
    {
        t = -300;
    }
    if (b < 0)
    {
        b = 0;
    }
    if (b > 80)
    {
        b = 80;
    }

    g_trim_ddeg = t;
    g_backlash_ddeg = b;
}
