/*
 * motion_hw.c - motor PWM (RP2040 registers, same method as motion_cal.c) and
 * encoder edge counting for MotionControl.
 *
 * Direction: the Robo Pico driver has two inputs per motor. PWM on the
 * "forward" input with the other low drives forward; swapped drives reverse;
 * both low = coast (this driver brakes WORSE with both high, see the
 * calibration results).
 *
 * The encoder interrupt goes through gpio_dispatch.c (one shared handler).
 */
#include <stdint.h>
#include <stdbool.h>
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hw.h"
#include "../subsystem3/gpio_dispatch.h"   /* shared dispatcher, owned by Buddy 3 */

#include <tk/tkernel.h>

/* MH_ prefix: the kernel's sysdef.h already uses the plain register names. */
#define MH_REG32(addr)          (*(volatile uint32_t *)(addr))

#define MH_RESETS_BASE          (0x4000C000U)
#define MH_RESETS_RESET         MH_REG32(MH_RESETS_BASE + 0x00U)
#define MH_RESETS_RESET_DONE    MH_REG32(MH_RESETS_BASE + 0x08U)
#define MH_RST_IO_BANK0         (1U << 5)
#define MH_RST_PADS_BANK0       (1U << 8)
#define MH_RST_PWM              (1U << 14)

#define MH_IO_BANK0_BASE        (0x40014000U)
#define MH_GPIO_CTRL(pin)       MH_REG32(MH_IO_BANK0_BASE + (8U * (pin)) + 4U)
#define MH_INTR0                MH_REG32(MH_IO_BANK0_BASE + 0x0F0U)   /* GPIO 0-7 raw/latched events */
#define MH_PROC0_INTE0          MH_REG32(MH_IO_BANK0_BASE + 0x100U)   /* GPIO 0-7 enable, core 0     */
#define MH_PROC0_INTS0          MH_REG32(MH_IO_BANK0_BASE + 0x120U)   /* GPIO 0-7 status, core 0     */
#define MH_FUNCSEL_PWM          (4U)
#define MH_FUNCSEL_SIO          (5U)

#define MH_PWM_BASE             (0x40050000U)
#define MH_PWM_SLICE_STRIDE     (0x14U)
#define MH_PWM_CSR(s)           MH_REG32(MH_PWM_BASE + ((s) * MH_PWM_SLICE_STRIDE) + 0x00U)
#define MH_PWM_DIV(s)           MH_REG32(MH_PWM_BASE + ((s) * MH_PWM_SLICE_STRIDE) + 0x04U)
#define MH_PWM_CC(s)            MH_REG32(MH_PWM_BASE + ((s) * MH_PWM_SLICE_STRIDE) + 0x0CU)
#define MH_PWM_TOP(s)           MH_REG32(MH_PWM_BASE + ((s) * MH_PWM_SLICE_STRIDE) + 0x10U)

#define MH_TIMER_RAWL           MH_REG32(0x40054028U)   /* free-running 1 MHz counter */
#define MH_SYS_CLK_HZ           (125000000U)            /* TODO: confirm the kernel's clk_sys */

/* Left / right to M1 / M2, and which input is "forward" for each wheel. */
#if MOTION_LEFT_IS_M1
#define MH_L_A                  (PIN_M1_A)
#define MH_L_B                  (PIN_M1_B)
#define MH_R_A                  (PIN_M2_A)
#define MH_R_B                  (PIN_M2_B)
#else
#define MH_L_A                  (PIN_M2_A)
#define MH_L_B                  (PIN_M2_B)
#define MH_R_A                  (PIN_M1_A)
#define MH_R_B                  (PIN_M1_B)
#endif

#if MOTOR_L_FORWARD_IS_A
#define MH_L_FWD                (MH_L_A)
#define MH_L_REV                (MH_L_B)
#else
#define MH_L_FWD                (MH_L_B)
#define MH_L_REV                (MH_L_A)
#endif
#if MOTOR_R_FORWARD_IS_A
#define MH_R_FWD                (MH_R_A)
#define MH_R_REV                (MH_R_B)
#else
#define MH_R_FWD                (MH_R_B)
#define MH_R_REV                (MH_R_A)
#endif

static const uint32_t s_pin_fwd[2] = { MH_L_FWD, MH_R_FWD };
static const uint32_t s_pin_rev[2] = { MH_L_REV, MH_R_REV };

static volatile uint32_t s_ticks[2];            /* written by the ISR only */
static volatile uint32_t s_last_edge_us[2];
static volatile uint32_t s_min_edge_ticks = ENCODER_MIN_EDGE_US;

/* ---------- Encoder edges ---------- */

/* Called for every edge on an encoder pin (by the shared GPIO dispatcher, or by
 * the local handler below). Counts and returns: nothing that can block. */
void motion_encoder_isr(uint32_t gpio)
{
    uint32_t side;
    uint32_t now;

    if (gpio == PIN_ENC_L)
    {
        side = MOTION_SIDE_LEFT;
    }
    else if (gpio == PIN_ENC_R)
    {
        side = MOTION_SIDE_RIGHT;
    }
    else
    {
        return;
    }

    now = MH_TIMER_RAWL;
    // if ((now - s_last_edge_us[side]) >= ENCODER_MIN_EDGE_US)
    if ((now - s_last_edge_us[side]) >= s_min_edge_ticks)
    {
        s_last_edge_us[side] = now;
        s_ticks[side]++;
    }
}

void motion_encoder_ticks_isr(int32_t *left, int32_t *right)
{
    *left  = (int32_t)s_ticks[MOTION_SIDE_LEFT];
    *right = (int32_t)s_ticks[MOTION_SIDE_RIGHT];
}

int32_t motion_ticks_to_mm(int32_t ticks)
{
    const float mm = (float)ticks * MM_PER_TICK;

    return (int32_t)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

uint32_t motion_hw_ticks(uint32_t side)
{
    return s_ticks[side];
}

void motion_hw_set_encoder_min_edge_ticks(uint32_t ticks)
{
    s_min_edge_ticks = (ticks == 0U) ? 1U : ticks;
}

uint32_t motion_hw_time_us(void)
{
    return MH_TIMER_RAWL;
}

void motion_hw_irq_status(int32_t *def_status, int32_t *enable_status,
                          uint32_t *inte0, uint32_t *handler_calls)
{
    *enable_status = 0;                 /* EnableInt() returns nothing in this kernel */
    gpio_dispatch_status(def_status, inte0, handler_calls);
}

/* ---------- Motor PWM ---------- */

static void mh_pwm_set_level(uint32_t pin, uint32_t level)
{
    const uint32_t slice = (pin >> 1) & 7U;
    uint32_t cc = MH_PWM_CC(slice);

    if ((pin & 1U) == 0U)                   /* even GPIO = channel A (low half) */
    {
        cc = (cc & 0xFFFF0000U) | (level & 0xFFFFU);
    }
    else                                    /* odd GPIO = channel B (high half) */
    {
        cc = (cc & 0x0000FFFFU) | ((level & 0xFFFFU) << 16);
    }
    MH_PWM_CC(slice) = cc;
}

void motion_hw_motor_set(uint32_t side, int32_t duty_pm)
{
    const uint32_t fwd = s_pin_fwd[side];
    const uint32_t rev = s_pin_rev[side];
    uint32_t mag = (duty_pm >= 0) ? (uint32_t)duty_pm : (uint32_t)(-duty_pm);

    if (mag > (uint32_t)MOTOR_PWM_MAX)
    {
        mag = (uint32_t)MOTOR_PWM_MAX;
    }

    if (mag == 0U)                          /* coast: both inputs low */
    {
        mh_pwm_set_level(fwd, 0U);
        mh_pwm_set_level(rev, 0U);
    }
    else if (duty_pm > 0)                   /* switch the idle input off first */
    {
        mh_pwm_set_level(rev, 0U);
        mh_pwm_set_level(fwd, mag);
    }
    else
    {
        mh_pwm_set_level(fwd, 0U);
        mh_pwm_set_level(rev, mag);
    }
}

bool motion_hw_init(void)
{
    const uint32_t reset_mask = MH_RST_IO_BANK0 | MH_RST_PADS_BANK0 | MH_RST_PWM;
    const uint32_t pins[4] = { MH_L_A, MH_L_B, MH_R_A, MH_R_B };
    const uint32_t div16 = (MH_SYS_CLK_HZ * 16U) /
                           (MOTOR_PWM_FREQ_HZ * (uint32_t)MOTOR_PWM_MAX);   /* 4.4 fixed point */
    uint32_t guard = 0U;
    uint32_t i;

    MH_RESETS_RESET &= ~reset_mask;
    while (((MH_RESETS_RESET_DONE & reset_mask) != reset_mask) && (guard < 1000000U))
    {
        guard++;
    }
    if ((MH_RESETS_RESET_DONE & reset_mask) != reset_mask)
    {
        return false;
    }

    for (i = 0U; i < 4U; i++)
    {
        const uint32_t slice = (pins[i] >> 1) & 7U;

        MH_PWM_CSR(slice) = 0U;
        MH_PWM_DIV(slice) = div16;
        MH_PWM_TOP(slice) = (uint32_t)MOTOR_PWM_MAX - 1U;
        MH_PWM_CC(slice)  = 0U;
        MH_GPIO_CTRL(pins[i]) = MH_FUNCSEL_PWM;
    }
    for (i = 0U; i < 4U; i++)
    {
        MH_PWM_CSR((pins[i] >> 1) & 7U) = 1U;
    }

    motion_hw_motor_set(MOTION_SIDE_LEFT, 0);
    motion_hw_motor_set(MOTION_SIDE_RIGHT, 0);

    MH_GPIO_CTRL(PIN_ENC_L) = MH_FUNCSEL_SIO;
    MH_GPIO_CTRL(PIN_ENC_R) = MH_FUNCSEL_SIO;
    (void)gpio_dispatch_register(PIN_ENC_L, GPIO_DISPATCH_EDGE_BOTH, motion_encoder_isr);
    (void)gpio_dispatch_register(PIN_ENC_R, GPIO_DISPATCH_EDGE_BOTH, motion_encoder_isr);
    return gpio_dispatch_start();       /* false if another handler already owns the interrupt */
}
