/*
 * gpio_dispatch.h - the team's ONE handler for the RP2040 GPIO interrupt.
 *
 * All GPIO pins share one interrupt (IO_IRQ_BANK0, number 13). Only one
 * handler can own it, so every subsystem registers its pin here instead of
 * calling tk_def_int() itself:
 *   Buddy 2: encoders GP2, GP3      (registered by motion_hw.c)
 *   Buddy 3: barcode sensor GP28    (registered below, calls b3_port_dispatch_barcode_irq())
 *   Buddy 5: ultrasonic echo GP17   (register it the same way)
 *
 * Callbacks run in INTERRUPT context: count or time-stamp and return. No
 * blocking calls, no printing.
 *
 * Needs B3_INSTALL_IO_BANK0_HANDLER set to 0 in b3_config.h, otherwise Buddy 3
 * installs a second handler on the same interrupt and one of them is lost.
 */
#ifndef GPIO_DISPATCH_H
#define GPIO_DISPATCH_H

#include <stdint.h>
#include <stdbool.h>

#define GPIO_DISPATCH_EDGE_LOW      (0x4U)
#define GPIO_DISPATCH_EDGE_HIGH     (0x8U)
#define GPIO_DISPATCH_EDGE_BOTH     (0xCU)

#define GPIO_DISPATCH_IRQ_NUMBER    (13U)   /* RP2040 IO_IRQ_BANK0                          */
#define GPIO_DISPATCH_IRQ_LEVEL     (1)     /* same level Buddy 3 uses                      */
#define GPIO_DISPATCH_MAX_PINS      (8U)

/* 1 = also route GP28 to Buddy 3's b3_port_dispatch_barcode_irq(). Set to 0 if
 * the Buddy 3 headers are not wanted in this build. */
#define GPIO_DISPATCH_BUDDY3        (1)

typedef void (*gpio_dispatch_cb_t)(uint32_t pin);

/* events != 0: the dispatcher enables these edge events for the pin, and clears
 *              them before calling cb (use for the encoders).
 * events == 0: the owner enables and clears its own events (Buddy 3 does this);
 *              cb is called whenever the pin shows any event. */
bool gpio_dispatch_register(uint32_t pin, uint32_t events, gpio_dispatch_cb_t cb);

/* Install the one handler (idempotent). Returns false if tk_def_int() failed,
 * for example because another handler already owns the interrupt. */
bool gpio_dispatch_start(void);

/* Diagnostics: tk_def_int() return value (0 = OK, -41 = already defined),
 * INTE0 read back, and how many times the handler has run. */
void gpio_dispatch_status(int32_t *def_status, uint32_t *inte0, uint32_t *handler_calls);

#endif /* GPIO_DISPATCH_H */
