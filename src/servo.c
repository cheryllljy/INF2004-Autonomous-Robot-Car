/**
 * @file    servo.c
 * @brief   RC servo driver on one RP2040 PWM channel.
 *
 * Timing:  50 Hz frame, 500..2500 us pulse. The PWM counter is clocked
 *          at exactly 1 MHz so one counter tick equals one microsecond
 *          and the level register can be loaded with the pulse width
 *          directly. No floating point anywhere.
 *
 * CPU cost: servo_set_angle() is one integer multiply, one divide and
 *           one register write. Measured < 3 us. It never blocks, so a
 *           caller holding a lock is never delayed by this driver.
 */

#include "servo.h"

#include "board_config.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

/*=====================================================================*/
/* Tunable constants                                                   */
/*=====================================================================*/

/** Set to 1 if increasing angle sweeps the wrong way. */
#define SERVO_INVERT                (0)

/** PWM counter tick rate. 1 tick == 1 us. */
#define SERVO_TICK_HZ               (1000000UL)

/** Frame length in ticks. 20000 us == 50 Hz. */
#define SERVO_PERIOD_TICKS          (20000U)

/** Pulse width at SERVO_ANGLE 0 and 180. Calibrate per servo. */
#define SERVO_PULSE_MIN_US          (500U)
#define SERVO_PULSE_MAX_US          (2500U)

/** Full angular span the pulse range maps onto. */
#define SERVO_SPAN_DEG              (180)

/**
 * Travel rate from the SG90 datasheet is 0.1 s / 60 deg unloaded at
 * 4.8 V. A sensor bracket loads it, so 140 ms / 60 deg is used as the
 * worst case. Re-measure this for your own servo (README, test S2).
 */
#define SERVO_TRAVEL_MS_PER_60DEG   (140U)

/** Fixed allowance for overshoot ringing after the horn arrives. */
#define SERVO_DAMPING_MS            (25U)

/*=====================================================================*/
/* Module state                                                        */
/*=====================================================================*/

static uint     g_slice     = 0U;
static uint     g_channel   = 0U;
static int16_t  g_angle_deg = SERVO_ANGLE_CENTRE_DEG;
static bool     g_ready     = false;

/*=====================================================================*/
/* Private helpers                                                     */
/*=====================================================================*/

/**
 * @brief   Convert a clamped angle to a pulse width in microseconds.
 * @param   angle_deg  Angle already inside the travel limits.
 * @return  Pulse width, microseconds.
 */
static uint16_t servo_angle_to_us(int16_t angle_deg)
{
    int32_t  effective = (int32_t)angle_deg;
    int32_t  span_us;
    int32_t  pulse_us;

#if (SERVO_INVERT != 0)
    effective = (int32_t)SERVO_SPAN_DEG - effective;
#endif

    span_us  = (int32_t)SERVO_PULSE_MAX_US - (int32_t)SERVO_PULSE_MIN_US;
    pulse_us = (int32_t)SERVO_PULSE_MIN_US +
               ((effective * span_us) / (int32_t)SERVO_SPAN_DEG);

    return (uint16_t)pulse_us;
}

/**
 * @brief   Clamp an angle to the mechanical travel limits.
 */
static int16_t servo_clamp(int16_t angle_deg)
{
    int16_t result = angle_deg;

    if (result < (int16_t)SERVO_ANGLE_MIN_DEG)
    {
        result = (int16_t)SERVO_ANGLE_MIN_DEG;
    }
    else if (result > (int16_t)SERVO_ANGLE_MAX_DEG)
    {
        result = (int16_t)SERVO_ANGLE_MAX_DEG;
    }
    else
    {
        /* Already inside the limits. */
    }

    return result;
}

/*=====================================================================*/
/* Public interface                                                    */
/*=====================================================================*/

bool servo_init(void)
{
    bool     ok      = false;
    uint32_t sys_hz  = clock_get_hz(clk_sys);
    uint32_t divider = sys_hz / SERVO_TICK_HZ;

    /* A non-integer divider would make the pulse width drift, so the
     * failure is reported rather than silently accepted. */
    if ((divider >= 1U) && (divider <= 255U) &&
        ((divider * SERVO_TICK_HZ) == sys_hz))
    {
        pwm_config cfg = pwm_get_default_config();

        gpio_set_function(BOARD_SERVO_GPIO, GPIO_FUNC_PWM);
        g_slice   = pwm_gpio_to_slice_num(BOARD_SERVO_GPIO);
        g_channel = pwm_gpio_to_channel(BOARD_SERVO_GPIO);

        pwm_config_set_clkdiv_int(&cfg, divider);
        pwm_config_set_wrap(&cfg, (uint16_t)(SERVO_PERIOD_TICKS - 1U));
        pwm_init(g_slice, &cfg, false);

        g_angle_deg = (int16_t)SERVO_ANGLE_CENTRE_DEG;
        pwm_set_chan_level(g_slice, g_channel,
                           servo_angle_to_us(g_angle_deg));
        pwm_set_enabled(g_slice, true);

        g_ready = true;
        ok      = true;
    }

    return ok;
}

bool servo_set_angle(int16_t angle_deg)
{
    bool    exact   = false;
    int16_t clamped = servo_clamp(angle_deg);

    if (g_ready)
    {
        pwm_set_chan_level(g_slice, g_channel,
                           servo_angle_to_us(clamped));
        g_angle_deg = clamped;
        exact       = (clamped == angle_deg);
    }

    return exact;
}

int16_t servo_get_angle(void)
{
    return g_angle_deg;
}

uint32_t servo_settle_ms(int16_t from_deg, int16_t to_deg)
{
    int32_t  delta = (int32_t)to_deg - (int32_t)from_deg;
    uint32_t travel_ms;

    if (delta < 0)
    {
        delta = -delta;
    }

    travel_ms = ((uint32_t)delta * SERVO_TRAVEL_MS_PER_60DEG) / 60U;

    return travel_ms + SERVO_DAMPING_MS;
}

void servo_release(void)
{
    if (g_ready)
    {
        /* Level 0 holds the output low, so no pulse is produced and
         * the servo stops driving. The PWM slice stays configured. */
        pwm_set_chan_level(g_slice, g_channel, 0U);
    }
}
