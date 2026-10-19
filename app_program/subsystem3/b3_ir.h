#ifndef B3_IR_H
#define B3_IR_H

#include "b3_barcode.h"
#include <tk/tkernel.h>

/* Optional Buddy 2 hook. Read a monotonic forward odometry counter.
 * Called with interrupts disabled OR from the GPIO ISR. Must not block,
 * call the RTOS, print, access I2C, or change GPIO configuration. */
typedef bool (*b3_position_reader_t)(uint32_t *p_ticks);

/* One snapshot of the three IR sensors: IR_L, IR_R (line) and IR_B
 * (barcode, with timestamp and odometry, if available). */
typedef struct
{
    bool left_black;
    bool right_black;
    b3_edge_t barcode;
} b3_ir_sample_t;

typedef enum
{
    B3_IR_EMPTY = 0,
    B3_IR_EDGE,
    B3_IR_DISCONTINUITY
} b3_ir_pop_status_t;

ER b3_ir_init(b3_position_reader_t reader);
void b3_ir_enable_barcode_irq(bool enabled);
/* Low-level queue delegate used by the port layer. Integrated applications
 * should normally call b3_port_dispatch_barcode_irq() from their shared
 * IO_BANK0 handler instead of calling this directly. */
void b3_ir_gpio_irq(unsigned int gpio, uint32_t events);
b3_ir_pop_status_t b3_ir_pop(b3_edge_t *p_edge);
void b3_ir_flush(void);
b3_ir_sample_t b3_ir_read(void);
/* Atomic queue-empty + GPIO snapshot for checking the trailing quiet zone. */
bool b3_ir_read_when_idle(b3_ir_sample_t *p_sample);
uint32_t b3_ir_fault_count(void);

#endif
