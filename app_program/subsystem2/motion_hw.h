/*
 * motion_hw.h - low-level motor PWM and encoder counting for MotionControl.
 * Only motion_control.c should call these.
 */
#ifndef MOTION_HW_H
#define MOTION_HW_H

#include <stdint.h>
#include <stdbool.h>

#define MOTION_SIDE_LEFT  (0U)
#define MOTION_SIDE_RIGHT (1U)

/* Release the PWM/GPIO blocks, set up the motor PWM and the encoder interrupt.
 * Call it from subsystem init code (usermain, core 0), NOT from a task: the
 * interrupt has to be enabled from core 0. */
bool motion_hw_init(void);

/* duty in permille, signed: + = forward, - = reverse, 0 = coast. */
void motion_hw_motor_set(uint32_t side, int32_t duty_pm);

/* Unsigned count of encoder edges since boot (direction is not sensed). */
uint32_t motion_hw_ticks(uint32_t side);

/* Set the encoder ISR debounce threshold in raw TIMER ticks. The motion task
 * calibrates this against kernel time because TIMERAWL may not match its
 * nominal 1 MHz rate on every startup. */
void motion_hw_set_encoder_min_edge_ticks(uint32_t ticks);

/* Encoder interrupt diagnostics: tk_def_int() result (0 = OK, -41 = another
 * handler owns the interrupt), INTE0 read back, and shared handler run count.
 * enable_status is always 0 (EnableInt() returns nothing in this kernel). */
void motion_hw_irq_status(int32_t *def_status, int32_t *enable_status,
                          uint32_t *inte0, uint32_t *handler_calls);

/* Free-running 1 MHz time stamp (low 32 bits). */
uint32_t motion_hw_time_us(void);

#endif /* MOTION_HW_H */
