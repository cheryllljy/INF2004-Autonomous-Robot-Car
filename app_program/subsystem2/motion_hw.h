/*
 * motion_hw.h - low-level motor PWM and encoder counting for MotionControl.
 * Only motion_control.c should call these (Buddy 2 owns these pins).
 */
#ifndef MOTION_HW_H
#define MOTION_HW_H

#include <stdint.h>
#include <stdbool.h>

#define MOTION_SIDE_LEFT    (0U)
#define MOTION_SIDE_RIGHT   (1U)

/* Release the PWM/GPIO blocks, set up the motor PWM and the encoder interrupt. */
bool motion_hw_init(void);

/* duty in permille, signed: + = forward, - = reverse, 0 = coast (both inputs low). */
void motion_hw_motor_set(uint32_t side, int32_t duty_pm);

/* Unsigned count of encoder edges since boot (direction is not sensed). */
uint32_t motion_hw_ticks(uint32_t side);

/* Diagnostics for the encoder interrupt (from gpio_dispatch.c): what tk_def_int()
 * returned (0 = OK, -41 = another handler already owns the interrupt), the INTE0
 * register read back, and how many times the shared handler has run.
 * enable_status is always 0 (EnableInt() returns nothing in this kernel). */
void motion_hw_irq_status(int32_t *def_status, int32_t *enable_status,
                          uint32_t *inte0, uint32_t *handler_calls);

/* Free-running 1 MHz time stamp (low 32 bits). */
uint32_t motion_hw_time_us(void);

#endif /* MOTION_HW_H */
