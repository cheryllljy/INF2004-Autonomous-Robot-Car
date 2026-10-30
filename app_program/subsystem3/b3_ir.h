#ifndef B3_IR_H
#define B3_IR_H

#include "b3_barcode.h"
#include "b3_config.h"
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
uint32_t b3_ir_glitch_count(void); /* coalesced IR_B interrupts dropped as glitches */

#if B3_BARCODE_ANALOG
/* One IR_B ADC reading (0..4095) taken at time_us, from the port's sampler
 * interrupt. Tracks the white and black levels, and queues an edge (timed
 * where the signal crossed the threshold) when the colour changes. */
void b3_ir_analog_sample(uint32_t time_us, uint32_t raw);

/* Levels on a black-is-high scale (0..4095), whatever the wiring polarity. */
typedef struct
{
    uint32_t samples;      /* sampler readings since b3_ir_init() */
    uint32_t near_misses;  /* crossed the threshold but not the hysteresis */
    uint16_t level;        /* latest reading */
    uint16_t white;        /* tracked white level */
    uint16_t black;        /* tracked black level */
    uint16_t threshold;    /* edge point */
    uint16_t hysteresis;   /* +/- band around the threshold */
    bool black_now;        /* current colour */
} b3_ir_analog_stats_t;

void b3_ir_analog_stats(b3_ir_analog_stats_t *p_stats);
#endif

#if defined(APP_TEST) && APP_TEST
/* Test build only: raw log of every barcode interrupt (or analog edge),
 * filled before any filtering. Used by tests B3-H05/H06 to see which edges
 * arrive. events is B3_GPIO_EDGE_RISE/FALL (electrical direction), or 0 for
 * an analog near miss: a dip that crossed the threshold but not the
 * hysteresis. peak is the analog level (black-is-high, 0..4095) furthest
 * from the threshold in the element that ended (the dip itself for a near
 * miss); 0 in DO mode. */
#define B3_EDGE_LOG_SIZE 128U
void b3_ir_edge_log_reset(void);
/* Copies up to max entries; returns how many entries were recorded in total
 * (can be more than B3_EDGE_LOG_SIZE). p_peak may be NULL. */
uint32_t b3_ir_edge_log_read(uint32_t *p_time_us, uint8_t *p_events,
                             uint16_t *p_peak, uint32_t max);
#endif

#endif
