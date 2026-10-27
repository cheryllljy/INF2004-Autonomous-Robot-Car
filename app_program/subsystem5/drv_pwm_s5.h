/**
 * @file    drv_pwm_s5.h
 * @brief   Register helpers shared by the two subsystem 5 drivers.
 *
 * This port does not link the Pico SDK runtime, so the PWM block is
 * driven through the micro T-Kernel register map (sys/machine.h,
 * in_w / out_w / set_w / clr_w), the same way Buddy 3's port does.
 * Only drv_servo.c and drv_ultrasonic.c include this file.
 */
#ifndef DRV_PWM_S5_H
#define DRV_PWM_S5_H

#include <tk/tkernel.h>
#include <sys/machine.h>

/* Register offsets inside one slice (RP2040 datasheet 4.5.3). */
#define S5_PWM_SLICE_BASE(s)        (PWM_BASE + (0x14U * (UW)(s)))
#define S5_PWM_CSR(s)               (S5_PWM_SLICE_BASE(s) + PWM_CHx_CSR)
#define S5_PWM_DIV(s)               (S5_PWM_SLICE_BASE(s) + PWM_CHx_DIV)
#define S5_PWM_CTR(s)               (S5_PWM_SLICE_BASE(s) + PWM_CHx_CTR)
#define S5_PWM_CC(s)                (S5_PWM_SLICE_BASE(s) + PWM_CHx_CC)
#define S5_PWM_TOP(s)               (S5_PWM_SLICE_BASE(s) + PWM_CHx_TOP)

/* CSR fields */
#define S5_PWM_CSR_EN               (1UL << 0)
#define S5_PWM_CSR_DIVMODE_SHIFT    (4U)
#define S5_PWM_DIVMODE_FREE         (0UL)   /* count every divided clock */
#define S5_PWM_DIVMODE_LEVEL        (1UL)   /* count only while B is high */

/* DIV: 8.4 fixed point, integer part in bits 11..4 */
#define S5_PWM_DIV_INT_SHIFT        (4U)

/** Divider for 1 count per microsecond. SYSCLK is in MHz (125). */
#define S5_PWM_DIV_1MHZ             ((UW)SYSCLK)

/** Slice of a GPIO, and whether it is channel B (odd pin). */
#define S5_PWM_SLICE_OF(gpio)       (((gpio) >> 1) & 7U)
#define S5_PWM_IS_CHAN_B(gpio)      (((gpio) & 1U) != 0U)

/**
 * @brief   Take the PWM block out of reset.
 *
 * config.h has USE_PTMR = 0, so the kernel's startup leaves PWM in
 * reset. Clearing the bit through the atomic CLR alias is idempotent
 * and touches no other block, so it is safe whichever subsystem (S2
 * motors, S5 servo/sonar) gets here first. Recorded in TEAM.md.
 */
static inline void s5_pwm_block_ready(void)
{
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U)
    {
        /* Takes a few clock cycles. */
    }
}

/** Free-running microsecond counter (TIMER_TIMERAWL). Wraps every
 *  71 minutes; unsigned subtraction is wrap-correct. */
static inline UW s5_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

#endif /* DRV_PWM_S5_H */
