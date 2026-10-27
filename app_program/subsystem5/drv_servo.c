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
#include "s5_config.h"
#include "s5_pack.h"

#include <bsp/libbsp.h>

/*=====================================================================*/
/* Tunable constants                                                   */
/*=====================================================================*/

/** Set to 1 if increasing angle sweeps the wrong way. */
#define DRV_SERVO_INVERT            (0)

/** 20000 counts of 1 us = 50 Hz. */
#define DRV_SERVO_PERIOD_US         (20000U)

/** Pulse at 0 and 180 deg. Calibrate per servo. */
#define DRV_SERVO_PULSE_MIN_US      (500U)
#define DRV_SERVO_PULSE_MAX_US      (2500U)
#define DRV_SERVO_SPAN_DEG          (180)

/**
 * SG90 datasheet: 0.1 s / 60 deg unloaded at 4.8 V. The sonar bracket
 * loads it, so 140 ms / 60 deg is the worst case.
 * TODO(measure): re-run bench test A5b on the power bank.
 */
#define DRV_SERVO_TRAVEL_MS_PER_60  (140U)

/** Fixed allowance for overshoot ringing after the horn arrives. */
#define DRV_SERVO_DAMPING_MS        (25U)

/* The pin must really be slice 7 channel B, or the registers below
 * drive the wrong output. */
S5_STATIC_ASSERT(S5_PWM_SLICE_OF(S5_SERVO_GPIO) == S5_SERVO_PWM_SLICE,
                 servo_on_slice_7);
S5_STATIC_ASSERT(S5_PWM_IS_CHAN_B(S5_SERVO_GPIO), servo_on_channel_b);

/*=====================================================================*/
/* Module state (owned by the obstacle task only)                      */
/*=====================================================================*/

static int16_t g_angle_deg = DRV_SERVO_CENTRE_DEG;
static bool    g_ready     = false;

/*=====================================================================*/
/* Private helpers                                                     */
/*=====================================================================*/

static UW drv_servo_angle_to_us(int16_t angle_deg)
{
    int32_t effective = (int32_t)angle_deg;
    int32_t span_us;

#if (DRV_SERVO_INVERT != 0)
    effective = (int32_t)DRV_SERVO_SPAN_DEG - effective;
#endif

    span_us = (int32_t)DRV_SERVO_PULSE_MAX_US -
              (int32_t)DRV_SERVO_PULSE_MIN_US;

    return (UW)((int32_t)DRV_SERVO_PULSE_MIN_US +
                ((effective * span_us) / (int32_t)DRV_SERVO_SPAN_DEG));
}

static int16_t drv_servo_clamp(int16_t angle_deg)
{
    int16_t result = angle_deg;

    if (result < (int16_t)DRV_SERVO_MIN_DEG)
    {
        result = (int16_t)DRV_SERVO_MIN_DEG;
    }
    else if (result > (int16_t)DRV_SERVO_MAX_DEG)
    {
        result = (int16_t)DRV_SERVO_MAX_DEG;
    }
    else
    {
        /* Inside the limits. */
    }

    return result;
}

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
    (void)pwm_set_cc((UINT)S5_SERVO_GPIO, drv_servo_angle_to_us(g_angle_deg));

    /* Route the pin to the PWM block only after the compare value is
     * set, so the servo never sees a garbage first pulse. */
    (void)pwm_set_pin((UINT)S5_SERVO_GPIO);
    (void)pwm_set_enabled((UINT)S5_SERVO_GPIO, TRUE);

    g_ready = true;

    return g_ready;
}

bool drv_servo_set_angle(int16_t angle_deg)
{
    bool          exact   = false;
    const int16_t clamped = drv_servo_clamp(angle_deg);

    if (g_ready)
    {
        (void)pwm_set_cc((UINT)S5_SERVO_GPIO, drv_servo_angle_to_us(clamped));
        g_angle_deg = clamped;
        exact       = (clamped == angle_deg);
    }

    return exact;
}

int16_t drv_servo_get_angle(void)
{
    return g_angle_deg;
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
