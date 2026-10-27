/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04 driver. ECHO pulse width measured by PWM hardware,
 *          no GPIO interrupt, no busy-wait on ECHO.
 *
 * Why no interrupt (v2)
 * ---------------------
 * The RP2040 has ONE GPIO interrupt line per core (IO_IRQ_BANK0) and
 * the kernel allows one handler on it. Buddy 3's barcode reader already
 * installs that handler (B3_INSTALL_IO_BANK0_HANDLER = 1). A second
 * tk_def_int() on the same line would silently replace it and kill
 * barcode decoding (the shared-dispatcher risk in the tutor feedback).
 *
 * Instead GP17 is the B input of PWM slice 0, in "level" divider mode:
 * the slice counter advances once per microsecond ONLY while ECHO is
 * high. After the echo ends the counter simply holds the pulse width.
 * The task triggers, sleeps in the kernel, then reads one register.
 *
 *   - no ISR at all, so no ISR/task shared data and no CR1 race
 *   - no contention on IO_IRQ_BANK0 with Buddy 2 / Buddy 3
 *   - 1 us resolution (0.17 mm), same as the old edge timestamps
 *   - CPU per ping: a few register accesses plus one kernel delay
 *
 * Cost: the echo is read after a fixed window instead of at the falling
 * edge. With the 10 ms kernel tick the window ends 20-30 ms after the
 * trigger, so a guard ping now takes ~25 ms of wall time (the task is
 * blocked, not running). The guard period stays 60 ms.
 *
 * Failure modes
 * -------------
 *   ECHO high before trigger      -> ECHO_STUCK (fault), do not fire
 *   counter 0, ECHO low at read   -> NO_RESPONSE (fault): ECHO never rose
 *   ECHO still high at read       -> NO_ECHO: sensor alive, nothing
 *                                    within ~3.4 m, path clear
 *   width < 30 mm / > 2500 mm     -> TOO_CLOSE / TOO_FAR
 */

#include "drv_ultrasonic.h"

#include "drv_pwm_s5.h"
#include "s5_config.h"

#include <bsp/libbsp.h>

/*=====================================================================*/
/* Tunable constants                                                   */
/*=====================================================================*/

/** Trigger pulse. Datasheet minimum is 10 us. */
#define DRV_US_TRIG_US              (12U)

/**
 * Read the counter no earlier than this after the trigger. 20 ms of
 * flight is 3.4 m, past the 2.5 m valid limit, so every valid echo has
 * finished by then.
 */
#define DRV_US_READ_WINDOW_US       (20000U)

/**
 * Minimum trigger-to-trigger spacing (datasheet 60 ms). Firing sooner
 * makes the old echo look like a near object. Up to 1 ms early is
 * accepted so tick jitter on the 60 ms guard period does not push
 * every ping a whole 10 ms tick late.
 */
#define DRV_US_QUIET_US             (60000U)
#define DRV_US_QUIET_SLACK_US       (1000U)

/** mm = us * 343 / 2000 (speed of sound ~343 m/s at 20 degC). */
#define DRV_US_SPEED_MM_PER_MS      (343U)

/** Kernel tick, used to turn a remaining time into a delay. */
#define DRV_US_TICK_MS              (10U)

S5_STATIC_ASSERT(S5_PWM_SLICE_OF(S5_ECHO_GPIO) == S5_ECHO_PWM_SLICE,
                 echo_on_slice_0);
S5_STATIC_ASSERT(S5_PWM_IS_CHAN_B(S5_ECHO_GPIO), echo_on_channel_b);

/*=====================================================================*/
/* Module state (owned by the obstacle task only)                      */
/*=====================================================================*/

static uint32_t           g_last_trig_us = 0U;
static bool               g_fired_once   = false;
static bool               g_ready        = false;
static ultrasonic_stats_t g_stats        = { 0U, 0U, 0U, 0U };

/*=====================================================================*/
/* Private helpers                                                     */
/*=====================================================================*/

static void drv_us_count(uint16_t *p_counter)
{
    if (*p_counter < 0xFFFFU)
    {
        (*p_counter)++;
    }
}

static uint16_t drv_us_to_mm(uint32_t echo_us)
{
    uint32_t mm = (echo_us * DRV_US_SPEED_MM_PER_MS) / 2000U;

    if (mm > 0xFFFDU)
    {
        mm = 0xFFFDU;
    }

    return (uint16_t)mm;
}

static bool drv_us_echo_high(void)
{
    return (gpio_get_val((UINT)S5_ECHO_GPIO) != 0U);
}

/** Sleep (in the kernel) until at least `target_us` after `since_us`. */
static void drv_us_sleep_until(uint32_t since_us, uint32_t target_us)
{
    uint32_t elapsed = (uint32_t)s5_time_us() - since_us;

    while (elapsed < target_us)
    {
        const uint32_t remain_ms = ((target_us - elapsed) + 999U) / 1000U;

        (void)tk_dly_tsk((RELTIM)((remain_ms < DRV_US_TICK_MS) ?
                                  DRV_US_TICK_MS : remain_ms));
        elapsed = (uint32_t)s5_time_us() - since_us;
    }
}

/** Zero the slice counter and arm it. Counts only while ECHO is high. */
static void drv_us_arm_counter(void)
{
    const UW slice = (UW)S5_ECHO_PWM_SLICE;

    clr_w(S5_PWM_CSR(slice), S5_PWM_CSR_EN);
    out_w(S5_PWM_CTR(slice), 0U);
    set_w(S5_PWM_CSR(slice), S5_PWM_CSR_EN);
}

static uint32_t drv_us_read_counter(void)
{
    const UW slice = (UW)S5_ECHO_PWM_SLICE;
    UW       count;

    clr_w(S5_PWM_CSR(slice), S5_PWM_CSR_EN);  /* freeze */
    count = in_w(S5_PWM_CTR(slice)) & 0xFFFFU;

    return (uint32_t)count;
}

static void drv_us_fire(void)
{
    uint32_t t0;

    (void)gpio_set_val((UINT)S5_TRIG_GPIO, 1U);
    t0 = (uint32_t)s5_time_us();
    while (((uint32_t)s5_time_us() - t0) < DRV_US_TRIG_US)
    {
        /* 12 us. Interrupts stay enabled: preemption only lengthens
         * the pulse, which the sensor tolerates. */
    }
    (void)gpio_set_val((UINT)S5_TRIG_GPIO, 0U);
}

static ultrasonic_status_t drv_us_classify(uint32_t echo_us, uint16_t *p_mm)
{
    ultrasonic_status_t status = ULTRASONIC_OK;
    const uint16_t      mm     = drv_us_to_mm(echo_us);

    if (mm < (uint16_t)DRV_US_MIN_VALID_MM)
    {
        status = ULTRASONIC_TOO_CLOSE;
        *p_mm  = (uint16_t)DRV_US_MIN_VALID_MM;
    }
    else if (mm > (uint16_t)DRV_US_MAX_VALID_MM)
    {
        status = ULTRASONIC_TOO_FAR;
        *p_mm  = (uint16_t)S5_RANGE_UNKNOWN_MM;
    }
    else
    {
        *p_mm = mm;
    }

    return status;
}

/*=====================================================================*/
/* Public interface                                                    */
/*=====================================================================*/

bool drv_ultrasonic_init(void)
{
    const UW slice = (UW)S5_ECHO_PWM_SLICE;
    const UW pad   = GPIO(S5_ECHO_GPIO);

    s5_pwm_block_ready();

    /* TRIG: plain output, low. */
    (void)gpio_set_pin((UINT)S5_TRIG_GPIO, GPIO_MODE_OUT);
    (void)gpio_set_val((UINT)S5_TRIG_GPIO, 0U);

    /* ECHO pad: input enabled, OUTPUT DISABLED at the pad (the PWM block
     * must never drive the level converter), Schmitt trigger on, weak
     * pull-down so a missing sensor reads low (-> NO_RESPONSE) rather
     * than floating. */
    set_w(pad, GPIO_IE | GPIO_OD | GPIO_SHEMITT | GPIO_PDE);
    clr_w(pad, GPIO_PUE);
    out_w(GPIO_CTRL(S5_ECHO_GPIO), GPIO_CTRL_FUNCSEL_PWM);

    /* Slice 0: level-gated divider, 1 count per us while B is high,
     * full 16-bit range (65 ms, above the 38 ms HC-SR04 no-echo pulse). */
    out_w(S5_PWM_CSR(slice),
          S5_PWM_DIVMODE_LEVEL << S5_PWM_CSR_DIVMODE_SHIFT);
    out_w(S5_PWM_DIV(slice), S5_PWM_DIV_1MHZ << S5_PWM_DIV_INT_SHIFT);
    out_w(S5_PWM_TOP(slice), 0xFFFFU);
    out_w(S5_PWM_CTR(slice), 0U);

    g_fired_once = false;
    g_ready      = true;

    return g_ready;
}

ultrasonic_status_t drv_ultrasonic_ping(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t status = ULTRASONIC_BAD_PARAM;
    uint16_t            mm     = (uint16_t)S5_RANGE_UNKNOWN_MM;
    uint32_t            echo   = 0U;

    if (p_result != NULL)
    {
        if (!g_ready)
        {
            status = ULTRASONIC_NOT_READY;
        }
        else
        {
            if (g_fired_once)
            {
                drv_us_sleep_until(g_last_trig_us,
                                   DRV_US_QUIET_US - DRV_US_QUIET_SLACK_US);
            }

            if (drv_us_echo_high())
            {
                /* Previous burst still ringing, or the converter's LV
                 * side is pulled up with the sensor unpowered. */
                drv_us_count(&g_stats.rejects);
                status = ULTRASONIC_ECHO_STUCK;
            }
            else
            {
                bool still_high;

                drv_us_arm_counter();
                g_last_trig_us = (uint32_t)s5_time_us();
                g_fired_once   = true;
                drv_us_fire();

                drv_us_sleep_until(g_last_trig_us, DRV_US_READ_WINDOW_US);

                still_high = drv_us_echo_high();
                echo       = drv_us_read_counter();
                drv_us_count(&g_stats.pings);

                if (still_high)
                {
                    /* ECHO rose and is still up 20+ ms later: the sensor
                     * is alive and heard nothing within 3.4 m. */
                    drv_us_count(&g_stats.no_echo);
                    status = ULTRASONIC_NO_ECHO;
                }
                else if (echo == 0U)
                {
                    /* ECHO never rose. An empty track cannot cause this. */
                    drv_us_count(&g_stats.no_response);
                    status = ULTRASONIC_NO_RESPONSE;
                }
                else
                {
                    status = drv_us_classify(echo, &mm);

                    if (status != ULTRASONIC_OK)
                    {
                        drv_us_count(&g_stats.rejects);
                    }
                }
            }
        }

        *p_result = drv_us_result_pack(mm, echo, status);
    }

    return status;
}

ultrasonic_status_t drv_ultrasonic_ping_median(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t status = ULTRASONIC_BAD_PARAM;

    if (p_result != NULL)
    {
        uint16_t valid[DRV_US_MEDIAN_SAMPLES];
        uint8_t  n         = 0U;
        uint8_t  faults    = 0U;
        uint8_t  too_close = 0U;
        uint8_t  i;
        uint16_t mm = (uint16_t)S5_RANGE_UNKNOWN_MM;

        for (i = 0U; i < (uint8_t)DRV_US_MEDIAN_SAMPLES; i++)
        {
            ultrasonic_result_t       one;
            const ultrasonic_status_t st = drv_ultrasonic_ping(&one);

            if (st == ULTRASONIC_OK)
            {
                valid[n] = drv_us_result_mm(one);
                n++;
            }
            else if (st == ULTRASONIC_TOO_CLOSE)
            {
                too_close++;
            }
            else if ((st == ULTRASONIC_NO_ECHO) || (st == ULTRASONIC_TOO_FAR))
            {
                /* Answered, nothing in range. */
            }
            else
            {
                faults++;
            }
        }

        if (n >= 2U)
        {
            uint8_t a;

            /* Insertion sort, n <= 3. */
            for (a = 1U; a < n; a++)
            {
                const uint16_t key = valid[a];
                uint8_t        b   = a;

                while ((b > 0U) && (valid[b - 1U] > key))
                {
                    valid[b] = valid[b - 1U];
                    b--;
                }
                valid[b] = key;
            }

            mm     = valid[n / 2U];
            status = ULTRASONIC_OK;
        }
        else if (faults > 0U)
        {
            /* v1 reported NO_ECHO here, which a caller reads as a clear
             * path. A failed sensor must never look like free space. */
            status = ULTRASONIC_NO_RESPONSE;
        }
        else if (too_close > 0U)
        {
            mm     = (uint16_t)DRV_US_MIN_VALID_MM;
            status = ULTRASONIC_TOO_CLOSE;
        }
        else
        {
            status = ULTRASONIC_NO_ECHO;
        }

        *p_result = drv_us_result_pack(mm, 0U, status);
    }

    return status;
}

void drv_ultrasonic_get_stats(ultrasonic_stats_t *p_stats)
{
    if (p_stats != NULL)
    {
        *p_stats = g_stats;
    }
}

uint32_t drv_ultrasonic_time_us(void)
{
    return (uint32_t)s5_time_us();
}
