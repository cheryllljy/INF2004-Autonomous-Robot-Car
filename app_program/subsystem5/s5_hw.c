/*
 *      s5_hw.c
 *      Subsystem 5 -- HC-SR04P and servo, register level. See s5_hw.h.
 *
 *      This port does not link the Pico SDK runtime, so the PWM block is
 *      driven through the micro T-Kernel register map (sys/machine.h) and
 *      the BSP's pwm_set_* helpers, the same way Buddy 2 does.
 */

#include "s5_hw.h"

#include <tk/tkernel.h>
#include <sys/machine.h>
#include <bsp/libbsp.h>

#include "s5_config.h"
#include "../subsystem4/subsystem4.h"

/* One PWM slice's registers (RP2040 datasheet 4.5.3). */
#define S5_SLICE(s)     (PWM_BASE + (0x14U * (UW)(s)))
#define S5_PWM_CSR(s)   (S5_SLICE(s) + PWM_CHx_CSR)
#define S5_PWM_DIV(s)   (S5_SLICE(s) + PWM_CHx_DIV)
#define S5_PWM_CTR(s)   (S5_SLICE(s) + PWM_CHx_CTR)
#define S5_PWM_TOP(s)   (S5_SLICE(s) + PWM_CHx_TOP)
#define S5_CSR_EN       (1UL << 0)
#define S5_CSR_DIVMODE  (4U)         /* shift */
#define S5_DIVMODE_GATE (1UL)        /* count only while B is high */
#define S5_DIV_INT      (4U)         /* DIV: 8.4 fixed point */
#define S5_DIV_1MHZ     ((UW)SYSCLK) /* SYSCLK in MHz: 1 count per us */

#define S5_SERVO_FRAME_US (20000U) /* 50 Hz */
#define S5_TRIG_SPIN      (400U)   /* ~15 us at 125 MHz */

/* The TIMER counts one per watchdog tick, and nothing but the Pico SDK
 * runtime (not linked) sets that tick up. Buddy 3's b3_port_init() starts
 * it at 1 MHz from the 12 MHz crystal; s5_hw_init() does the same if it
 * finds it stopped, so S5 never depends on Buddy 3 being built in. */
#define S5_WATCHDOG_TICK         (0x4005802CU)
#define S5_WATCHDOG_TICK_ENABLE  (1UL << 9)
#define S5_WATCHDOG_TICK_RUNNING (1UL << 10)

_Static_assert(((S5_SERVO_GPIO >> 1) & 7U) == S5_SERVO_PWM_SLICE,
               "servo pin must be on PWM slice 7");
_Static_assert(((S5_ECHO_GPIO >> 1) & 7U) == S5_ECHO_PWM_SLICE,
               "echo pin must be on PWM slice 0");
_Static_assert((S5_ECHO_GPIO & 1U) == 1U,
               "echo pin must be a channel B (gate input) pin");

static bool g_b_ready = false;

static void s5_hw_pwm_out_of_reset(void)
{
    /* idempotent: Buddy 2's motors use the same block */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* a few clock cycles */
    }
}

static void s5_hw_timer_start(void)
{
    if (0U != (in_w(RESETS_RESET) & RESETS_RESET_TIMER))
    {
        clr_w(RESETS_RESET, RESETS_RESET_TIMER);
        while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
        {
            /* a few clock cycles */
        }
    }
    if (0U == (in_w(S5_WATCHDOG_TICK) & S5_WATCHDOG_TICK_RUNNING))
    {
        out_w(S5_WATCHDOG_TICK, S5_WATCHDOG_TICK_ENABLE | (UW)XOSC_MHz);
    }
}

bool s5_hw_init(void)
{
    const UW echo = (UW)S5_ECHO_PWM_SLICE;
    const UW servo = (UW)S5_SERVO_PWM_SLICE;

    s5_hw_timer_start();
    s5_hw_pwm_out_of_reset();

    /* TRIG: output, low */
    (void)gpio_set_pin(S5_TRIG_GPIO, GPIO_MODE_OUT);
    (void)gpio_set_val(S5_TRIG_GPIO, 0U);

    /* ECHO pad: input on, output driver OFF (the PWM block must never
     * drive the level converter), Schmitt trigger, weak pull-down so a
     * missing sensor reads low (DEAD), not floating */
    set_w(GPIO(S5_ECHO_GPIO), GPIO_IE | GPIO_OD | GPIO_SHEMITT | GPIO_PDE);
    clr_w(GPIO(S5_ECHO_GPIO), GPIO_PUE);
    out_w(GPIO_CTRL(S5_ECHO_GPIO), GPIO_CTRL_FUNCSEL_PWM);

    /* slice 0: gated by B, 1 count per us, full 16 bits (65 ms) */
    out_w(S5_PWM_CSR(echo), S5_DIVMODE_GATE << S5_CSR_DIVMODE);
    out_w(S5_PWM_DIV(echo), S5_DIV_1MHZ << S5_DIV_INT);
    out_w(S5_PWM_TOP(echo), 0xFFFFU);
    out_w(S5_PWM_CTR(echo), 0U);

    /* slice 7: 1 MHz, 20 ms frame; the pulse is set before the pin is
     * routed, so the servo never sees a garbage first pulse */
    out_w(S5_PWM_CSR(servo), 0U);
    out_w(S5_PWM_DIV(servo), S5_DIV_1MHZ << S5_DIV_INT);
    out_w(S5_PWM_CTR(servo), 0U);
    (void)pwm_set_wrap(S5_SERVO_GPIO, (UW)(S5_SERVO_FRAME_US - 1U));
    g_b_ready = true;
    s5_hw_servo(0);
    (void)pwm_set_pin(S5_SERVO_GPIO);
    (void)pwm_set_enabled(S5_SERVO_GPIO, TRUE);
    return true;
}

uint32_t s5_hw_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

void s5_hw_servo(int32_t ddeg)
{
    int32_t a = ddeg;
    int32_t phys;
    const int32_t span =
        (int32_t)S5_SERVO_PULSE_MAX_US - (int32_t)S5_SERVO_PULSE_MIN_US;

    if (a > S5_SERVO_LIMIT_DDEG)
    {
        a = S5_SERVO_LIMIT_DDEG;
    }
    if (a < -S5_SERVO_LIMIT_DDEG)
    {
        a = -S5_SERVO_LIMIT_DDEG;
    }
    a += S5_SERVO_TRIM_DDEG;
#if (S5_SERVO_INVERT != 0)
    phys = 900 - a;
#else
    phys = 900 + a;
#endif
    if (g_b_ready)
    {
        (void)pwm_set_cc(S5_SERVO_GPIO, (UW)((int32_t)S5_SERVO_PULSE_MIN_US +
                                             ((phys * span) / 1800)));
    }
}

bool s5_hw_fire(void)
{
    const UW slice = (UW)S5_ECHO_PWM_SLICE;
    volatile uint32_t spin;

    if (0U != gpio_get_val(S5_ECHO_GPIO))
    {
        return false; /* still answering the last ping */
    }
    /* zero and arm the gated counter */
    clr_w(S5_PWM_CSR(slice), S5_CSR_EN);
    out_w(S5_PWM_CTR(slice), 0U);
    set_w(S5_PWM_CSR(slice), S5_CSR_EN);

    (void)gpio_set_val(S5_TRIG_GPIO, 1U);
    for (spin = 0U; spin < S5_TRIG_SPIN; spin++)
    {
        /* >= 10 us; being preempted only makes it longer, which the
         * sensor accepts */
    }
    (void)gpio_set_val(S5_TRIG_GPIO, 0U);
    return true;
}

s5_echo_t s5_hw_echo(uint32_t *p_us)
{
    const UW slice = (UW)S5_ECHO_PWM_SLICE;
    const bool b_high = (0U != gpio_get_val(S5_ECHO_GPIO));
    uint32_t count;

    clr_w(S5_PWM_CSR(slice), S5_CSR_EN); /* freeze */
    count = (uint32_t)(in_w(S5_PWM_CTR(slice)) & 0xFFFFU);
    *p_us = count;
    if (b_high)
    {
        return S5_ECHO_NONE; /* still waiting: nothing within range */
    }
    if (0U == count)
    {
        return S5_ECHO_DEAD; /* ECHO never rose */
    }
    return S5_ECHO_OK;
}

bool s5_hw_mag(int32_t *p_x, int32_t *p_y)
{
    s4_imu_snapshot_t snap;
    SYSTIM now;

    if ((subsystem4_get_imu_snapshot(&snap) < E_OK) || !snap.has_sample ||
        (IMU_OK != snap.mag_status) || (tk_get_tim(&now) < E_OK) ||
        ((uint32_t)(now.lo - snap.timestamp_ms) > (uint32_t)S5_MAG_AGE_MS))
    {
        return false;
    }
    *p_x = (int32_t)snap.mag.x;
    *p_y = (int32_t)snap.mag.y;
    return true;
}
