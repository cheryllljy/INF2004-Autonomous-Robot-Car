/*
 *      s5_sensor.h
 *      Subsystem 5 -- the sensor task's side: servo, pings, the sweep
 *.
 *
 *      The S5 task calls s5_sensor_poll() every few milliseconds. It never
 *      blocks: a ping is fired, read S5_ECHO_READ_MS later, and the next
 *      one goes out S5_PING_PERIOD_MS after the last (one ping every 60 ms,
 *      NFR10). The servo only moves between pings, and a ping is only fired
 *      once the servo has had time to get where it was sent.
 *
 *      What it does is chosen by the VehicleController's task (through
 *      s5_avoid.c) with three requests:
 *        s5_sensor_guard()      sensor straight ahead; each ping says
 *                               whether something is within S5_DETECT_MM
 *                               (S5_DETECT_CONFIRM pings in a row)
 *        s5_sensor_look(angle)  sensor held at an angle (the bypass looks
 *                               at the object's corners with it)
 *        s5_sensor_scan()       one full sweep for the width; the samples
 *                               come back with s5_sensor_scan_get()
 *
 *      Between the two tasks: the request is one 32-bit word (one aligned
 *      store, one load); the latest ping is published under a sequence
 *      count, so a reader never sees half of one; a sweep's samples are
 *      written before its "done" number is (with a memory barrier: the two
 *      tasks may run on different cores).
 */

#ifndef S5_SENSOR_H
#define S5_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "s5_scan.h"

typedef struct
{
    uint32_t seq;      /* counts pings; 0 = none yet */
    uint32_t fired_us; /* s5_hw_us() when it was fired */
    int16_t ddeg;      /* servo angle it was fired at, + = left */
    uint16_t mm;       /* range; 0 = nothing within S5_RANGE_MAX_MM */
    bool b_valid;      /* false: the sensor gave no echo pulse at all */
    bool b_close;      /* guard: something within S5_DETECT_MM ahead */
} s5_ping_t;

typedef struct
{
    uint32_t seq; /* the s5_sensor_scan() it answers */
    uint32_t n;   /* samples, sorted right (-) to left (+) */
    s5_sample_t s[S5_SCAN_MAX];
} s5_sweep_t;

/* ---- S5 task ---------------------------------------------------------- */
void s5_sensor_init(void);
void s5_sensor_poll(void);

/* ---- any one other task (the VehicleController's) --------------------- */
void s5_sensor_guard(void);
void s5_sensor_look(int32_t ddeg);
/* Starts a sweep; returns its number for s5_sensor_scan_get(). */
uint32_t s5_sensor_scan(void);
/* True once sweep seq is complete; *p_out is then a copy of it. */
bool s5_sensor_scan_get(uint32_t seq, s5_sweep_t *p_out);
/* The latest ping. False if there has been none yet. */
bool s5_sensor_ping(s5_ping_t *p_out);
/* FR25: S5_DEAD_PINGS pings in a row without any echo pulse, or ECHO
 * stuck high. */
bool s5_sensor_dead(void);

#endif /* S5_SENSOR_H */
