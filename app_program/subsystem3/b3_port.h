#ifndef B3_PORT_H
#define B3_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/* RP2040 IO_BANK0 uses four interrupt bits per GPIO:
 * level-low, level-high, edge-low, edge-high. Buddy 3 only uses edges. */
#define B3_GPIO_EDGE_FALL  (1U << 2)
#define B3_GPIO_EDGE_RISE  (1U << 3)
#define B3_GPIO_EDGE_BOTH  (B3_GPIO_EDGE_FALL | B3_GPIO_EDGE_RISE)

typedef uint32_t b3_irq_state_t;

/* Initialise the synchronisation primitive used by Buddy 3.
 * On the supplied SMP port this uses a real cross-core spinlock. */
ER b3_port_init(void);
void b3_port_lock(b3_irq_state_t *p_state);
void b3_port_unlock(b3_irq_state_t state);

/* GPIO/time services used by the portable IR driver. */
ER b3_port_gpio_input_init(unsigned int pin, bool pull_up);
ER b3_port_gpio_output_init(unsigned int pin, bool initial_high);
uint32_t b3_port_gpio_get_all(void);
void b3_port_gpio_put(unsigned int pin, bool value);
uint32_t b3_port_time_us(void);

/* Barcode edge interrupt control. */
void b3_port_barcode_irq_enable(bool enabled);
uint32_t b3_port_barcode_irq_pending(void);
void b3_port_barcode_irq_ack(uint32_t events);

/* Standalone mode: install Buddy 3 as the IO_IRQ_BANK0 handler.
 * Integrated robot mode: set B3_INSTALL_IO_BANK0_HANDLER=0 and call
 * b3_port_dispatch_barcode_irq() from the team's one shared IO handler. */
ER b3_port_install_barcode_irq(void);
void b3_port_dispatch_barcode_irq(void);

#endif
