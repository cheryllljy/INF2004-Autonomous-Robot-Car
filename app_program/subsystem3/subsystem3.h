/*
 *      subsystem3.h
 *      Subsystem 3 -- Consumer (statistics).  Student ID: <student ID>.
 *
 *      Public interface of subsystem 3.  Other subsystems include it as
 *      "../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER 16 /* priority band 16-17 (TEAM.md) */
/* report to subsystem 4 every N samples */
#define S3_BATCH_SIZE 5
/* summary lines in a row that found subsystem 4's message buffer full.
 * One is dropped by design (TMO_POL); this many means subsystem 4 has
 * stopped reading, which is a fault. */
#define S3_MBF_FULL_LIMIT 3

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
extern ID mbx_samples;

/* IF-04: statistics. Read them ONLY while holding mtx_stats:
 *   tk_loc_mtx(mtx_stats, TMO_FEVR); copy = app_stats; tk_unl_mtx(...);
 */
typedef struct
{
    UW received; /* samples received so far */
    INT min;
    INT max;
    W sum;
} APP_STATS;

extern ID mtx_stats;
extern APP_STATS app_stats;

/* Private to subsystem 3 */
extern void consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
