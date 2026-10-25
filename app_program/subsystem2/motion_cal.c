/*
 * motion_cal.c - Buddy 2 calibration firmware, Step 3 (motor bring-up), v7.
 *
 * Compiled to NOTHING unless MOTION_CALIBRATION_BUILD is 1.
 *
 * Order of the steps (GP20 starts each one; the prompt repeats every ~3 s):
 *   0. Hand test (motors OFF): turn each wheel by hand 10 turns and see how
 *      many encoder ticks come out. Shows whether the encoder signal is clean.
 *   1. Dead-zone sweep, each motor, each input (A-driven, B-driven):
 *      raises the PWM duty in steps until the encoder sees plausible movement
 *      ("start"), then lowers it until the wheel stops ("stop").
 *   2. Coast vs brake test, each motor: cuts power with both inputs LOW
 *      (coast) and then both HIGH (brake) and reports ticks and ms to stop.
 *
 * Encoders are POLLED in a tight loop (tens of kHz) during each measurement,
 * so fast edges are not missed (a 1 ms poll aliases on a high-resolution
 * encoder). Edges closer together than CAL_MIN_EDGE_US are ignored as chatter.
 * Both counts are printed: "ticks" (filtered) and "raw" (every transition).
 * The task runs at LOW priority, so the busy polling only uses spare CPU time.
 * This does NOT test the real EncoderISR path.
 *
 * PWM is set up with RP2040 registers, so no Pico SDK library is needed.
 *
 * SAFETY: lift the car so the wheels are free for the first runs. Pressing
 * GP20 while a routine is running aborts it and coasts the motor.
 *
 * NOT compiled against the real kernel/BSP.
 */
#include <stdint.h>
#include <stdbool.h>
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hw.h"

#if MOTION_CALIBRATION_BUILD

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

/* ---------- Tunables for the routines ---------- */
#define CAL_STEP_PM         (10U)       /* duty step, permille                  */
#define CAL_SWEEP_MAX_PM    (600U)      /* give up above this duty              */
#define CAL_HOLD_MS         (800U)      /* time at each duty step               */
#define CAL_MIN_TICKS       (3U)        /* ticks in one step = "it is moving"   */
#define CAL_KICK_PM         (100U)      /* run this far above start, then step down */
#define CAL_REPEATS         (3U)        /* dead-zone runs per motor and input   */
#define CAL_STOP_REPEATS    (1U)        /* coast/brake runs per motor           */
#define CAL_MIN_EDGE_US     (100U)      /* after an accepted edge, ignore transitions for this long (chatter).
                                           Keep it BELOW the shortest real edge spacing:
                                           2000 us capped the count at ~500 ticks/s. */
#define CAL_RUN_ISRCHECK    (1U)        /* 1 = ISR counter vs polled count (both motors) */
#define CAL_RUN_HAND_TEST   (0U)        /* 1 = run this part, 0 = skip it       */
#define CAL_RUN_DEADZONE    (0U)
#define CAL_RUN_STOPTEST    (0U)
#define CAL_BINS            (100U)      /* stop test: 10 ms bins = 1 s after the cut */
#define CAL_ABORT_US        (30000U)    /* GP20 must stay pressed this long to abort (ignores noise spikes) */
#define CAL_TEST_RUN_PM     (500U)      /* duty used before the coast/brake cut */
#define CAL_SPINUP_MS       (1500U)
#define CAL_IDLE_MS         (300U)      /* no tick for this long = stopped      */
#define CAL_STOP_TIMEOUT_MS (5000U)
#define CAL_TEST_BRAKE      (1U)        /* 0 = skip the both-HIGH brake test.
                                           Set 0 if the driver datasheet says
                                           both inputs HIGH is not allowed.     */
#define CAL_PIN_BUTTON      (20U)       /* GP20, active low (Week 6 pin table)  */
#define CAL_SYS_CLK_HZ      (125000000U)/* TODO: confirm the kernel's clk_sys   */
/* Task priority is chosen in motion_cal_init(): as low as the kernel allows. */
#define CAL_TASK_STACK      (4096U)

/* ---------- RP2040 registers (see the RP2040 datasheet) ----------
 * CAL_ prefix on purpose: the kernel's sysdef.h already defines macros with the
 * plain names (as bare addresses), and redefining them caused warnings. */
#define CAL_REG32(addr)         (*(volatile uint32_t *)(addr))

#define CAL_RESETS_BASE         (0x4000C000U)
#define CAL_RESETS_RESET        CAL_REG32(CAL_RESETS_BASE + 0x00U)
#define CAL_RESETS_RESET_DONE   CAL_REG32(CAL_RESETS_BASE + 0x08U)
#define CAL_RESET_IO_BANK0      (1U << 5)
#define CAL_RESET_PADS_BANK0    (1U << 8)
#define CAL_RESET_PWM           (1U << 14)

#define CAL_IO_BANK0_BASE       (0x40014000U)
#define CAL_GPIO_CTRL(pin)      CAL_REG32(CAL_IO_BANK0_BASE + (8U * (pin)) + 4U)
#define CAL_FUNCSEL_PWM         (4U)
#define CAL_FUNCSEL_SIO         (5U)

#define CAL_PWM_BASE            (0x40050000U)
#define CAL_PWM_SLICE_STRIDE    (0x14U)
#define CAL_PWM_CSR(s)          CAL_REG32(CAL_PWM_BASE + ((s) * CAL_PWM_SLICE_STRIDE) + 0x00U)
#define CAL_PWM_DIV(s)          CAL_REG32(CAL_PWM_BASE + ((s) * CAL_PWM_SLICE_STRIDE) + 0x04U)
#define CAL_PWM_CC(s)           CAL_REG32(CAL_PWM_BASE + ((s) * CAL_PWM_SLICE_STRIDE) + 0x0CU)
#define CAL_PWM_TOP(s)          CAL_REG32(CAL_PWM_BASE + ((s) * CAL_PWM_SLICE_STRIDE) + 0x10U)

#define CAL_SIO_GPIO_IN         CAL_REG32(0xD0000004U)
#define CAL_TIMER_RAWL          CAL_REG32(0x40054028U)  /* free-running 1 MHz counter */

/* ---------- Left / right to M1 / M2 ---------- */
#if MOTION_LEFT_IS_M1
#define CAL_PIN_L_A         (PIN_M1_A)
#define CAL_PIN_L_B         (PIN_M1_B)
#define CAL_PIN_R_A         (PIN_M2_A)
#define CAL_PIN_R_B         (PIN_M2_B)
#else
#define CAL_PIN_L_A         (PIN_M2_A)
#define CAL_PIN_L_B         (PIN_M2_B)
#define CAL_PIN_R_A         (PIN_M1_A)
#define CAL_PIN_R_B         (PIN_M1_B)
#endif

typedef enum { CAL_MOTOR_LEFT = 0, CAL_MOTOR_RIGHT = 1 } cal_motor_t;

/* A: PWM on input A, B low.  B: PWM on input B, A low.
 * COAST: both low.  BRAKE: both high. Which of A / B is "forward" is one of
 * the things this firmware helps you find out: note the wheel direction. */
typedef enum { CAL_MODE_A = 0, CAL_MODE_B, CAL_MODE_COAST, CAL_MODE_BRAKE } cal_mode_t;

static const uint32_t s_pin_a[2] = { CAL_PIN_L_A, CAL_PIN_R_A };
static const uint32_t s_pin_b[2] = { CAL_PIN_L_B, CAL_PIN_R_B };

static uint32_t s_ticks[2];         /* filtered edge counts, [0] = left, [1] = right */
static uint32_t s_raw[2];           /* every transition seen                         */
static uint32_t s_prev[2];
static uint32_t s_last_edge_us[2];

static const char *cal_motor_name(cal_motor_t m)
{
    return (m == CAL_MOTOR_LEFT) ? "LEFT" : "RIGHT";
}

static const char *cal_mode_name(cal_mode_t mode)
{
    return (mode == CAL_MODE_A) ? "A-driven" : "B-driven";
}

/* ---------- PWM ---------- */

static void cal_pwm_set_level(uint32_t pin, uint32_t level)
{
    uint32_t slice = (pin >> 1) & 7U;
    uint32_t cc = CAL_PWM_CC(slice);

    if ((pin & 1U) == 0U)                   /* even GPIO = channel A (low half) */
    {
        cc = (cc & 0xFFFF0000U) | (level & 0xFFFFU);
    }
    else                                    /* odd GPIO = channel B (high half) */
    {
        cc = (cc & 0x0000FFFFU) | ((level & 0xFFFFU) << 16);
    }
    CAL_PWM_CC(slice) = cc;
}

/* duty in permille 0..MOTOR_PWM_MAX. TOP = MOTOR_PWM_MAX - 1, so a level of
 * MOTOR_PWM_MAX is "always high" and 0 is "always low". */
static void cal_motor_drive(cal_motor_t motor, cal_mode_t mode, uint32_t duty_pm)
{
    uint32_t level_a = 0U;
    uint32_t level_b = 0U;

    if (duty_pm > (uint32_t)MOTOR_PWM_MAX)
    {
        duty_pm = (uint32_t)MOTOR_PWM_MAX;
    }

    switch (mode)
    {
    case CAL_MODE_A:
        level_a = duty_pm;
        break;
    case CAL_MODE_B:
        level_b = duty_pm;
        break;
    case CAL_MODE_BRAKE:
        level_a = (uint32_t)MOTOR_PWM_MAX;
        level_b = (uint32_t)MOTOR_PWM_MAX;
        break;
    case CAL_MODE_COAST:
    default:
        break;
    }
    cal_pwm_set_level(s_pin_a[motor], level_a);
    cal_pwm_set_level(s_pin_b[motor], level_b);
}

static bool cal_hw_init(void)
{
    const uint32_t reset_mask = CAL_RESET_IO_BANK0 | CAL_RESET_PADS_BANK0 | CAL_RESET_PWM;
    const uint32_t pins[4] = { CAL_PIN_L_A, CAL_PIN_L_B, CAL_PIN_R_A, CAL_PIN_R_B };
    const uint32_t div16 = (CAL_SYS_CLK_HZ * 16U) /
                           (MOTOR_PWM_FREQ_HZ * (uint32_t)MOTOR_PWM_MAX);  /* 4.4 fixed point */
    uint32_t guard = 0U;
    uint32_t i;

    CAL_RESETS_RESET &= ~reset_mask;                    /* release PWM / GPIO blocks */
    while (((CAL_RESETS_RESET_DONE & reset_mask) != reset_mask) && (guard < 1000000U))
    {
        guard++;                                    /* bounded wait, never hangs */
    }
    if ((CAL_RESETS_RESET_DONE & reset_mask) != reset_mask)
    {
        return false;
    }

    for (i = 0U; i < 4U; i++)
    {
        uint32_t slice = (pins[i] >> 1) & 7U;
        CAL_PWM_CSR(slice) = 0U;
        CAL_PWM_DIV(slice) = div16;
        CAL_PWM_TOP(slice) = (uint32_t)MOTOR_PWM_MAX - 1U;
        CAL_PWM_CC(slice)  = 0U;
        CAL_GPIO_CTRL(pins[i]) = CAL_FUNCSEL_PWM;
    }
    for (i = 0U; i < 4U; i++)
    {
        CAL_PWM_CSR((pins[i] >> 1) & 7U) = 1U;          /* enable slice */
    }

    CAL_GPIO_CTRL(PIN_ENC_L)      = CAL_FUNCSEL_SIO;        /* plain inputs, polled */
    CAL_GPIO_CTRL(PIN_ENC_R)      = CAL_FUNCSEL_SIO;
    CAL_GPIO_CTRL(CAL_PIN_BUTTON) = CAL_FUNCSEL_SIO;

    s_prev[0] = (CAL_SIO_GPIO_IN >> PIN_ENC_L) & 1U;
    s_prev[1] = (CAL_SIO_GPIO_IN >> PIN_ENC_R) & 1U;
    for (i = 0U; i < 2U; i++)
    {
        s_ticks[i] = 0U;
        s_raw[i] = 0U;
        s_last_edge_us[i] = 0U;
    }

    cal_motor_drive(CAL_MOTOR_LEFT,  CAL_MODE_COAST, 0U);
    cal_motor_drive(CAL_MOTOR_RIGHT, CAL_MODE_COAST, 0U);
    return true;
}

/* ---------- Encoders (polled, chatter-filtered) and button ---------- */

/* Read the pins once, count encoder edges, and return true if the button
 * (GP20, active low) is pressed. Call it in a tight loop. */
static bool cal_poll(void)
{
    const uint32_t in = CAL_SIO_GPIO_IN;
    uint32_t level[2];
    uint32_t i;

    level[0] = (in >> PIN_ENC_L) & 1U;
    level[1] = (in >> PIN_ENC_R) & 1U;
    for (i = 0U; i < 2U; i++)
    {
        if (level[i] != s_prev[i])
        {
            const uint32_t now = CAL_TIMER_RAWL;

            s_prev[i] = level[i];
            s_raw[i]++;
            if ((now - s_last_edge_us[i]) >= CAL_MIN_EDGE_US)
            {
                s_last_edge_us[i] = now;
                if ((ENCODER_EDGES_PER_SLOT == 2) || (level[i] == 1U))
                {
                    s_ticks[i]++;
                }
            }
        }
    }
    return (((in >> CAL_PIN_BUTTON) & 1U) == 0U);
}

static bool cal_button_pressed(void)
{
    return (((CAL_SIO_GPIO_IN >> CAL_PIN_BUTTON) & 1U) == 0U);
}

/* Poll the encoders and the abort button for ms milliseconds.
 * Returns true if the button was pressed (abort). */
static bool cal_hold(uint32_t ms)
{
    const uint32_t start = CAL_TIMER_RAWL;
    const uint32_t limit = ms * 1000U;
    uint32_t low_since = 0U;
    bool low = false;

    while ((CAL_TIMER_RAWL - start) < limit)
    {
        if (cal_poll())                         /* button reads pressed */
        {
            const uint32_t now = CAL_TIMER_RAWL;

            if (!low)
            {
                low = true;
                low_since = now;
            }
            else if ((now - low_since) >= CAL_ABORT_US)
            {
                return true;                    /* held long enough: a real press */
            }
        }
        else
        {
            low = false;
        }
    }
    return false;
}

/* Print the prompt and wait for GP20. The prompt repeats every ~3 s, so it is
 * not lost if the USB terminal connected after it was first printed. */
static void cal_prompt(const char *title, const char *motor, const char *what,
                       uint32_t run, uint32_t repeats)
{
    bool pressed = false;
    uint32_t i;

    while (cal_button_pressed())            /* let go of a previous press */
    {
        (void)tk_dly_tsk(10);
    }

    while (!pressed)
    {
        tm_printf((UB *)"\n[%s %s %s] run %u of %u. Press GP20 to start.\n",
                  title, motor, what, (unsigned int)run, (unsigned int)repeats);
        for (i = 0U; (i < 300U) && !pressed; i++)
        {
            if (cal_button_pressed())
            {
                pressed = true;
            }
            else
            {
                (void)tk_dly_tsk(10);
            }
        }
    }
    (void)tk_dly_tsk(50);                   /* debounce */
    while (cal_button_pressed())
    {
        (void)tk_dly_tsk(10);
    }
}

static void cal_abort(cal_motor_t motor)
{
    cal_motor_drive(motor, CAL_MODE_COAST, 0U);
    tm_printf((UB *)"  ABORTED (GP20 pressed), motor off\n");
}

/* ---------- Routine ISR: does the interrupt counter in motion_hw.c keep up? ----------
 * Both motors run at fixed duties for 1 s each. "polled" is this firmware's own
 * tight-loop count (known good); "ISR" is the counter MotionControl will use.
 * They must agree. The expected value is the floor model 5 * (duty - 50). */

static void cal_isr_check(void)
{
    static const uint32_t duties[5] = { 100U, 200U, 300U, 400U, 500U };
    uint32_t i;

    if (!motion_hw_init())
    {
        tm_printf((UB *)"ISR check: motion_hw_init() failed\n");
        return;
    }
    {
        int32_t def_st;
        int32_t en_st;
        uint32_t inte0;
        uint32_t calls;

        motion_hw_irq_status(&def_st, &en_st, &inte0, &calls);
        tm_printf((UB *)"ISR setup: tk_def_int returned %d (0 = OK, -41 = someone else already owns the interrupt), INTE0 = 0x%x\n",
                  (int)def_st, (unsigned int)inte0);
        (void)en_st;
    }
    tm_printf((UB *)"ISR check: both motors at a fixed duty, 1 s each (wheels lifted!)\n");

    for (i = 0U; i < 5U; i++)
    {
        const uint32_t pl0 = s_ticks[0];
        const uint32_t pr0 = s_ticks[1];
        const uint32_t il0 = motion_hw_ticks(MOTION_SIDE_LEFT);
        const uint32_t ir0 = motion_hw_ticks(MOTION_SIDE_RIGHT);

        const uint32_t t_run = CAL_TIMER_RAWL;

        cal_motor_drive(CAL_MOTOR_LEFT,  CAL_MODE_A, duties[i]);
        cal_motor_drive(CAL_MOTOR_RIGHT, CAL_MODE_A, duties[i]);
        if (cal_hold(1000U))
        {
            cal_abort(CAL_MOTOR_LEFT);
            cal_abort(CAL_MOTOR_RIGHT);
            tm_printf((UB *)"  (aborted %u ms into duty %u; GP20 was held low for 30 ms)\n",
                      (unsigned int)((CAL_TIMER_RAWL - t_run) / 1000U), (unsigned int)duties[i]);
            return;
        }
        tm_printf((UB *)"  duty %u: polled L %u R %u | ISR L %u R %u | expected about %u\n",
                  (unsigned int)duties[i],
                  (unsigned int)(s_ticks[0] - pl0), (unsigned int)(s_ticks[1] - pr0),
                  (unsigned int)(motion_hw_ticks(MOTION_SIDE_LEFT) - il0),
                  (unsigned int)(motion_hw_ticks(MOTION_SIDE_RIGHT) - ir0),
                  (unsigned int)(5U * (duties[i] - 50U)));
    }
    cal_motor_drive(CAL_MOTOR_LEFT,  CAL_MODE_COAST, 0U);
    cal_motor_drive(CAL_MOTOR_RIGHT, CAL_MODE_COAST, 0U);
    {
        int32_t def_st;
        int32_t en_st;
        uint32_t inte0;
        uint32_t calls;

        motion_hw_irq_status(&def_st, &en_st, &inte0, &calls);
        tm_printf((UB *)"ISR handler ran %u times in total\n", (unsigned int)calls);
    }
    tm_printf((UB *)"RESULT ISR check done. ISR should match polled; if ISR is far lower, the interrupt path loses edges.\n");
}

/* ---------- Routine 0: hand test (motors off) ----------
 * One wheel at a time. Nothing is printed while counting (a blocked console
 * would stop the polling and lose edges). Turn the wheel exactly 10 turns,
 * slowly, then press GP20. There is no time limit. */

static void cal_hand_test(cal_motor_t motor)
{
    const uint32_t own = (motor == CAL_MOTOR_LEFT) ? 0U : 1U;

    s_ticks[0] = 0U;
    s_ticks[1] = 0U;
    s_raw[0] = 0U;
    s_raw[1] = 0U;

    while (!cal_poll())
    {
        /* count until GP20 is pressed */
    }
    tm_printf((UB *)"RESULT hand test %s: %u ticks (raw %u). If you turned it exactly 10 times: %u ticks per turn.\n",
              cal_motor_name(motor), (unsigned int)s_ticks[own], (unsigned int)s_raw[own],
              (unsigned int)(s_ticks[own] / 10U));
    tm_printf((UB *)"  other wheel counted %u (should be about 0)\n",
              (unsigned int)s_ticks[1U - own]);
}

/* ---------- Routine 1: dead-zone sweep ---------- */

static void cal_sweep(cal_motor_t motor, cal_mode_t mode)
{
    const uint32_t other = (motor == CAL_MOTOR_LEFT) ? 1U : 0U;
    uint32_t duty;
    uint32_t run_pm;
    uint32_t start_pm = 0U;
    uint32_t stop_pm = 0U;
    bool found = false;

    tm_printf((UB *)"Sweep %s motor, %s: raising duty (permille)\n",
              cal_motor_name(motor), cal_mode_name(mode));

    for (duty = CAL_STEP_PM; (duty <= CAL_SWEEP_MAX_PM) && !found; duty += CAL_STEP_PM)
    {
        const uint32_t own0 = s_ticks[motor];
        const uint32_t raw0 = s_raw[motor];
        const uint32_t oth0 = s_ticks[other];
        uint32_t own;
        uint32_t oth;

        cal_motor_drive(motor, mode, duty);
        if (cal_hold(CAL_HOLD_MS))
        {
            cal_abort(motor);
            return;
        }
        own = s_ticks[motor] - own0;
        oth = s_ticks[other] - oth0;
        tm_printf((UB *)"  duty %u: ticks %u (raw %u, %u ticks/s), other wheel %u\n",
                  (unsigned int)duty, (unsigned int)own,
                  (unsigned int)(s_raw[motor] - raw0),
                  (unsigned int)((own * 1000U) / CAL_HOLD_MS), (unsigned int)oth);
        if (own >= CAL_MIN_TICKS)
        {
            found = true;
            start_pm = duty;
        }
    }

    if (!found)
    {
        cal_motor_drive(motor, CAL_MODE_COAST, 0U);
        tm_printf((UB *)"  NO movement up to %u pm: check wiring / encoder / battery\n",
                  (unsigned int)CAL_SWEEP_MAX_PM);
        return;
    }

    /* Now lower the duty until the wheel stops (the "stop" value is normally
     * lower than the "start" value: the difference is the hysteresis). */
    run_pm = start_pm + CAL_KICK_PM;
    if (run_pm > (uint32_t)MOTOR_PWM_MAX)
    {
        run_pm = (uint32_t)MOTOR_PWM_MAX;
    }
    cal_motor_drive(motor, mode, run_pm);
    if (cal_hold(1000U))
    {
        cal_abort(motor);
        return;
    }

    tm_printf((UB *)"  lowering duty from %u\n", (unsigned int)run_pm);
    duty = run_pm;
    while (duty >= CAL_STEP_PM)
    {
        const uint32_t own0 = s_ticks[motor];
        const uint32_t raw0 = s_raw[motor];
        uint32_t own;

        cal_motor_drive(motor, mode, duty);
        if (cal_hold(CAL_HOLD_MS))
        {
            cal_abort(motor);
            return;
        }
        own = s_ticks[motor] - own0;
        tm_printf((UB *)"  duty %u: ticks %u (raw %u, %u ticks/s)\n",
                  (unsigned int)duty, (unsigned int)own,
                  (unsigned int)(s_raw[motor] - raw0),
                  (unsigned int)((own * 1000U) / CAL_HOLD_MS));
        if (own < CAL_MIN_TICKS)
        {
            stop_pm = duty;
            break;
        }
        duty -= CAL_STEP_PM;
    }

    cal_motor_drive(motor, CAL_MODE_COAST, 0U);
    tm_printf((UB *)"RESULT %s %s: start %u pm, stop %u pm\n",
              cal_motor_name(motor), cal_mode_name(mode),
              (unsigned int)start_pm, (unsigned int)stop_pm);
}

/* ---------- Routine 2: coast vs brake ---------- */

static uint32_t s_cum[CAL_BINS];        /* ticks since the cut, at the end of each 10 ms bin */

static void cal_stop_test(cal_motor_t motor, cal_mode_t stop_mode, const char *label)
{
    uint32_t mark;
    uint32_t before;
    uint32_t ticks_cut;
    uint32_t t_cut;
    uint32_t bin = 0U;
    uint32_t row;
    uint32_t col;

    cal_motor_drive(motor, CAL_MODE_A, CAL_TEST_RUN_PM);
    if (cal_hold(CAL_SPINUP_MS - 100U))
    {
        cal_abort(motor);
        return;
    }
    mark = s_ticks[motor];
    if (cal_hold(100U))                         /* speed just before the cut */
    {
        cal_abort(motor);
        return;
    }
    before = (s_ticks[motor] - mark) * 10U;     /* ticks per second */

    ticks_cut = s_ticks[motor];
    t_cut = CAL_TIMER_RAWL;
    cal_motor_drive(motor, stop_mode, 0U);      /* cut power */

    while (bin < CAL_BINS)                      /* record only, print afterwards */
    {
        (void)cal_poll();
        if ((CAL_TIMER_RAWL - t_cut) >= ((bin + 1U) * 10000U))
        {
            s_cum[bin] = s_ticks[motor] - ticks_cut;
            bin++;
        }
    }

    cal_motor_drive(motor, CAL_MODE_COAST, 0U);
    tm_printf((UB *)"RESULT %s %s: %u ticks/s before the cut, %u ticks in the next second\n",
              cal_motor_name(motor), label, (unsigned int)before, (unsigned int)s_cum[CAL_BINS - 1U]);
    tm_printf((UB *)"  ticks per 10 ms after the cut, 100 ms per row:\n");
    for (row = 0U; row < (CAL_BINS / 10U); row++)
    {
        tm_printf((UB *)"  %4u ms:", (unsigned int)(row * 100U));
        for (col = 0U; col < 10U; col++)
        {
            const uint32_t b = (row * 10U) + col;
            const uint32_t prev = (b == 0U) ? 0U : s_cum[b - 1U];

            tm_printf((UB *)" %u", (unsigned int)(s_cum[b] - prev));
        }
        tm_printf((UB *)"\n");
    }
}

/* ---------- Task ---------- */

static void cal_task(INT stacd, void *exinf)
{
    uint32_t m;
    uint32_t run;

    (void)stacd;
    (void)exinf;
    (void)m;                                /* unused when only the ISR check is enabled */
    (void)run;

    tm_printf((UB *)"cal: task running\n");
    (void)tk_dly_tsk(3000);                 /* give the USB console time to connect */
    if (!cal_hw_init())
    {
        tm_printf((UB *)"cal: reset release timed out - stopping\n");
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
    tm_printf((UB *)"cal: hardware ready\n");
    tm_printf((UB *)"\n=== Buddy 2 calibration firmware v8b ===\n");
    tm_printf((UB *)"PWM %u Hz assuming %u Hz system clock. Press GP20 to start each step,\n",
              (unsigned int)MOTOR_PWM_FREQ_HZ, (unsigned int)CAL_SYS_CLK_HZ);
    tm_printf((UB *)"and again during a run to abort it. Lift the wheels for the first runs.\n");

#if CAL_RUN_ISRCHECK
    cal_prompt("ISR check", "BOTH", "wheels lifted: both motors run about 6 s", 1U, 1U);
    cal_isr_check();
#endif

#if CAL_RUN_HAND_TEST
    for (m = 0U; m < 2U; m++)
    {
        cal_prompt("Hand test", cal_motor_name((cal_motor_t)m),
                   "10 turns by hand slowly, GP20 again when done", 1U, 1U);
        cal_hand_test((cal_motor_t)m);
    }
#endif

#if CAL_RUN_DEADZONE
    for (m = 0U; m < 2U; m++)
    {
        cal_mode_t mode;
        for (mode = CAL_MODE_A; mode <= CAL_MODE_B; mode = (cal_mode_t)((uint32_t)mode + 1U))
        {
            for (run = 1U; run <= CAL_REPEATS; run++)
            {
                cal_prompt("Dead-zone", cal_motor_name((cal_motor_t)m), cal_mode_name(mode),
                           run, CAL_REPEATS);
                cal_sweep((cal_motor_t)m, mode);
            }
        }
    }
#endif

#if CAL_RUN_STOPTEST
    for (m = 0U; m < 2U; m++)
    {
        for (run = 1U; run <= CAL_STOP_REPEATS; run++)
        {
            cal_prompt("Coast vs brake", cal_motor_name((cal_motor_t)m), "", run, CAL_STOP_REPEATS);
            cal_stop_test((cal_motor_t)m, CAL_MODE_COAST, "coast (both low)");
            (void)tk_dly_tsk(2000);
#if CAL_TEST_BRAKE
            cal_stop_test((cal_motor_t)m, CAL_MODE_BRAKE, "brake (both high)");
#endif
        }
    }
#endif

    tm_printf((UB *)"\nAll calibration steps finished.\n");
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

static ID s_cal_task_id;

void motion_cal_init(void)
{
    /* The kernel refuses a priority above its configured maximum with E_PAR.
     * Busy polling must not starve other tasks, so try the lowest priority
     * first and step towards the higher ones until one is accepted. */
    static const PRI try_pri[] = { 100, 60, 40, 30, 20, 16, 14 };
    T_CTSK ctsk = {0};
    uint32_t i;

    ctsk.tskatr = TA_HLNG | TA_RNG0;
    ctsk.task   = cal_task;
    ctsk.stksz  = CAL_TASK_STACK;
    for (i = 0U; i < (sizeof(try_pri) / sizeof(try_pri[0])); i++)
    {
        ctsk.itskpri = try_pri[i];
        s_cal_task_id = tk_cre_tsk(&ctsk);
        if (s_cal_task_id != E_PAR)
        {
            break;
        }
    }
    tm_printf((UB *)"motion_cal: init, task id %d, priority %d\n",
              (int)s_cal_task_id, (int)ctsk.itskpri);
}

void motion_cal_start(void)
{
    if (s_cal_task_id > 0)
    {
        (void)tk_sta_tsk(s_cal_task_id, 0);
        tm_printf((UB *)"motion_cal: start\n");
    }
    else
    {
        tm_printf((UB *)"motion_cal: task creation failed (%d)\n", (int)s_cal_task_id);
    }
}

#endif /* MOTION_CALIBRATION_BUILD */
