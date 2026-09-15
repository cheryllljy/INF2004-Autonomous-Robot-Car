/**
 * @file    ultrasonic.c
 * @brief   HC-SR04 driver: edge-interrupt capture, kernel wait, no spin.
 *
 * Why it is built this way
 * ------------------------
 * The textbook HC-SR04 routine busy-waits on the ECHO pin. Under an
 * RTOS that burns up to 25 ms of CPU per ping at the priority of the
 * measuring task, which starves the motor PID and the line follower.
 * Here the ISR only timestamps two edges and signals a semaphore; the
 * task is blocked in the kernel for the whole flight time.
 *
 * Interrupt budget
 * ----------------
 *   2 interrupts per ping, ~40 instructions each  -> < 2 us total
 *   1 ping per 60 ms in the guard state           -> < 0.01 % CPU
 *
 * Failure modes handled
 * ---------------------
 *   no echo at all          -> kernel timeout, reported as NO_ECHO
 *   ECHO stuck high         -> detected before triggering
 *   missed rising edge      -> state machine ignores the lone fall
 *   32-bit microsecond wrap -> unsigned subtraction is wrap correct
 */

#include "ultrasonic.h"

#include "board_config.h"

#include <tk/tkernel.h>

#include "hardware/gpio.h"
#include "hardware/structs/timer.h"
#include "pico/stdlib.h"

/*=====================================================================*/
/* Tunable constants                                                   */
/*=====================================================================*/

/** Trigger pulse width. Datasheet minimum is 10 us. */
#define ULTRASONIC_TRIG_US          (12U)

/**
 * Kernel timeout waiting for the echo. 25 ms of flight is about 4.2 m,
 * well past the useful range of the sensor.
 */
#define ULTRASONIC_TIMEOUT_MS       (25)

/**
 * Minimum trigger-to-trigger spacing. The datasheet asks for 60 ms so
 * that the previous burst has decayed; firing sooner makes an old echo
 * look like a near object. This is the hard limit on scan speed.
 */
#define ULTRASONIC_QUIET_MS         (60U)
#define ULTRASONIC_QUIET_US         (ULTRASONIC_QUIET_MS * 1000U)

/**
 * Speed of sound in mm per millisecond at about 20 degC. Rises by
 * roughly 0.6 mm/ms per degC; at 35 degC the error is about 3 %.
 */
#define ULTRASONIC_SPEED_MM_PER_MS  (343U)

/*=====================================================================*/
/* Module state                                                        */
/*=====================================================================*/

/** Edge capture states. Touched by the ISR, hence volatile. */
typedef enum
{
    CAP_IDLE = 0,
    CAP_WAIT_RISE,
    CAP_WAIT_FALL,
    CAP_DONE
} cap_state_t;

static volatile cap_state_t g_cap_state = CAP_IDLE;
static volatile uint32_t    g_rise_us   = 0U;
static volatile uint32_t    g_width_us  = 0U;
static volatile uint32_t    g_spurious  = 0U;

static ID                  g_echo_sem  = 0;
static uint32_t            g_last_trig_us = 0U;
static ultrasonic_stats_t  g_stats = { 0U, 0U, 0U, 0U };
static bool                g_ready = false;

/*=====================================================================*/
/* Interrupt service routine                                           */
/*=====================================================================*/

/**
 * @brief   ECHO edge handler. Runs in interrupt context.
 *
 * Contract: timestamp, update the state machine, signal. Nothing else.
 * No printf, no division, no floating point, no kernel call that can
 * block.
 */
static void ultrasonic_echo_isr(uint gpio, uint32_t events)
{
    /* Raw timer read, no latching pair needed for the low word. */
    const uint32_t now_us = timer_hw->timerawl;

    if (gpio == (uint)BOARD_SONAR_ECHO_GPIO)
    {
        /* Both edges can be reported in one call for a very short
         * pulse, so they are tested in order rather than exclusively. */
        if ((events & (uint32_t)GPIO_IRQ_EDGE_RISE) != 0U)
        {
            if (g_cap_state == CAP_WAIT_RISE)
            {
                g_rise_us   = now_us;
                g_cap_state = CAP_WAIT_FALL;
            }
            else
            {
                g_spurious++;
            }
        }

        if ((events & (uint32_t)GPIO_IRQ_EDGE_FALL) != 0U)
        {
            if (g_cap_state == CAP_WAIT_FALL)
            {
                g_width_us  = now_us - g_rise_us;
                g_cap_state = CAP_DONE;
                (void)tk_sig_sem(g_echo_sem, 1);
            }
            else
            {
                g_spurious++;
            }
        }
    }
}

/*=====================================================================*/
/* Private helpers                                                     */
/*=====================================================================*/

/**
 * @brief   Convert an echo pulse width to a one-way distance.
 * @param   echo_us  Pulse width, microseconds (<= 30000 by timeout).
 * @return  Distance in millimetres.
 *
 * mm = us * 343 / 2000. Worst case numerator 30000 * 343 = 1.03e7,
 * which fits a uint32_t with three decades to spare. No float.
 */
static uint16_t ultrasonic_us_to_mm(uint32_t echo_us)
{
    uint32_t mm = (echo_us * ULTRASONIC_SPEED_MM_PER_MS) / 2000U;

    if (mm > 0xFFFEU)
    {
        mm = 0xFFFEU;
    }

    return (uint16_t)mm;
}

/**
 * @brief   Sleep out whatever is left of the mandatory quiet period.
 */
static void ultrasonic_wait_quiet(void)
{
    const uint32_t elapsed_us = timer_hw->timerawl - g_last_trig_us;

    if (elapsed_us < ULTRASONIC_QUIET_US)
    {
        const uint32_t remain_us = ULTRASONIC_QUIET_US - elapsed_us;

        /* Round up to the next whole tick; tk_dly_tsk resolution is
         * 1 ms, and a short delay must never round down to zero. */
        (void)tk_dly_tsk((RELTIM)((remain_us + 999U) / 1000U));
    }
}

/**
 * @brief   Drain any stale signal so a late echo cannot satisfy the
 *          next wait.
 */
static void ultrasonic_drain_sem(void)
{
    while (tk_wai_sem(g_echo_sem, 1, TMO_POL) == E_OK)
    {
        /* Discard. */
    }
}

/**
 * @brief   Emit the trigger burst.
 *
 * Preemption during the pulse only makes it longer, which the sensor
 * tolerates, so interrupts are deliberately left enabled here. If the
 * task is preempted after the pulse the echo is still captured by the
 * ISR and latched in the counting semaphore.
 */
static void ultrasonic_fire(void)
{
    g_cap_state    = CAP_WAIT_RISE;
    g_last_trig_us = timer_hw->timerawl;

    gpio_put(BOARD_SONAR_TRIG_GPIO, 1);
    busy_wait_us_32(ULTRASONIC_TRIG_US);
    gpio_put(BOARD_SONAR_TRIG_GPIO, 0);
}

/**
 * @brief   Classify a captured pulse width.
 */
static ultrasonic_status_t ultrasonic_classify(uint32_t echo_us,
                                               uint16_t *p_mm)
{
    ultrasonic_status_t status = ULTRASONIC_OK;
    const uint16_t      mm     = ultrasonic_us_to_mm(echo_us);

    if (mm < (uint16_t)ULTRASONIC_MIN_VALID_MM)
    {
        status = ULTRASONIC_TOO_CLOSE;
        *p_mm  = (uint16_t)ULTRASONIC_MIN_VALID_MM;
    }
    else if (mm > (uint16_t)ULTRASONIC_MAX_VALID_MM)
    {
        status = ULTRASONIC_TOO_FAR;
        *p_mm  = ULTRASONIC_RANGE_NONE;
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

bool ultrasonic_init(void)
{
    T_CSEM csem;

    csem.exinf   = NULL;
    csem.sematr  = TA_TFIFO | TA_FIRST;
    csem.isemcnt = 0;
    csem.maxsem  = 1;

    g_echo_sem = tk_cre_sem(&csem);

    if (g_echo_sem > 0)
    {
        gpio_init(BOARD_SONAR_TRIG_GPIO);
        gpio_set_dir(BOARD_SONAR_TRIG_GPIO, GPIO_OUT);
        gpio_put(BOARD_SONAR_TRIG_GPIO, 0);

        gpio_init(BOARD_SONAR_ECHO_GPIO);
        gpio_set_dir(BOARD_SONAR_ECHO_GPIO, GPIO_IN);
        gpio_pull_down(BOARD_SONAR_ECHO_GPIO);

        /* See README "Interrupts and the kernel" before changing this
         * line: the handler must be reachable from the kernel's
         * interrupt entry/exit for the woken task to be dispatched
         * immediately rather than at the next tick. */
        gpio_set_irq_enabled_with_callback(
            BOARD_SONAR_ECHO_GPIO,
            (uint32_t)GPIO_IRQ_EDGE_RISE | (uint32_t)GPIO_IRQ_EDGE_FALL,
            false,
            &ultrasonic_echo_isr);

        g_cap_state = CAP_IDLE;
        g_ready     = true;
    }

    return g_ready;
}

ultrasonic_status_t ultrasonic_ping(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t status = ULTRASONIC_BAD_PARAM;

    if (p_result != NULL)
    {
        p_result->distance_mm = ULTRASONIC_RANGE_NONE;
        p_result->echo_us     = 0U;

        if (!g_ready)
        {
            status = ULTRASONIC_NOT_READY;
        }
        else if (gpio_get(BOARD_SONAR_ECHO_GPIO) != 0)
        {
            /* Previous burst still ringing, or the sensor is unpowered
             * and the pin is floating high. Either way, do not fire. */
            g_stats.reject_count++;
            status = ULTRASONIC_ECHO_STUCK;
        }
        else
        {
            ER er;

            ultrasonic_wait_quiet();
            ultrasonic_drain_sem();

            gpio_set_irq_enabled(
                BOARD_SONAR_ECHO_GPIO,
                (uint32_t)GPIO_IRQ_EDGE_RISE |
                (uint32_t)GPIO_IRQ_EDGE_FALL,
                true);

            ultrasonic_fire();

            er = tk_wai_sem(g_echo_sem, 1, (TMO)ULTRASONIC_TIMEOUT_MS);

            gpio_set_irq_enabled(
                BOARD_SONAR_ECHO_GPIO,
                (uint32_t)GPIO_IRQ_EDGE_RISE |
                (uint32_t)GPIO_IRQ_EDGE_FALL,
                false);

            g_stats.ping_count++;

            if (er == E_OK)
            {
                const uint32_t echo_us = g_width_us;

                p_result->echo_us = echo_us;
                status = ultrasonic_classify(echo_us,
                                             &p_result->distance_mm);

                if (status != ULTRASONIC_OK)
                {
                    g_stats.reject_count++;
                }
            }
            else
            {
                g_stats.timeout_count++;
                status = ULTRASONIC_NO_ECHO;
            }

            g_cap_state     = CAP_IDLE;
            g_stats.spurious_edges = g_spurious;
        }

        p_result->status = status;
    }

    return status;
}

ultrasonic_status_t ultrasonic_ping_median(ultrasonic_result_t *p_result)
{
    ultrasonic_status_t status = ULTRASONIC_BAD_PARAM;

    if (p_result != NULL)
    {
        uint16_t valid[ULTRASONIC_MEDIAN_SAMPLES];
        uint8_t  n = 0U;
        uint8_t  i;

        for (i = 0U; i < (uint8_t)ULTRASONIC_MEDIAN_SAMPLES; i++)
        {
            ultrasonic_result_t one;

            if (ultrasonic_ping(&one) == ULTRASONIC_OK)
            {
                valid[n] = one.distance_mm;
                n++;
            }
        }

        if (n >= 2U)
        {
            uint8_t a;
            uint8_t b;

            /* Insertion sort: n is at most 3, so this is cheaper and
             * more predictable than calling into a library sort. */
            for (a = 1U; a < n; a++)
            {
                const uint16_t key = valid[a];

                b = a;
                while ((b > 0U) && (valid[b - 1U] > key))
                {
                    valid[b] = valid[b - 1U];
                    b--;
                }
                valid[b] = key;
            }

            p_result->distance_mm = valid[n / 2U];
            p_result->echo_us     = 0U;
            status                = ULTRASONIC_OK;
        }
        else
        {
            p_result->distance_mm = ULTRASONIC_RANGE_NONE;
            p_result->echo_us     = 0U;
            status                = ULTRASONIC_NO_ECHO;
        }

        p_result->status = status;
    }

    return status;
}

void ultrasonic_get_stats(ultrasonic_stats_t *p_stats)
{
    if (p_stats != NULL)
    {
        *p_stats = g_stats;
    }
}
