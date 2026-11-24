/*
 * cyc_check.c - bench check of the 20 ms cyclic handler -> task wake-up path.
 *
 * Measures the handler period and the wake-up latency of the woken task
 * (NFR5: 20 ms with jitter within +/-2 ms). Enable it by calling
 * cyc_check_init() / cyc_check_start() from subsystem2.c.
 */
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "cyc_check.h"

#define CYC_PERIOD_MS     (20U)
#define CYC_REPORT_CYCLES (250U) /* print about every 5 s */
/* same as MotionControl (1 = highest) */
#define CYC_TASK_PRIORITY (1)
#define CYC_TASK_STACK    (2048U)
/* RP2040 free-running 1 MHz timer, low 32 bits (TIMER base 0x40054000 + 0x28).
 * Read directly so no SDK timer library is needed. */
#define TIMER_RAWL (*(volatile uint32_t *)0x40054028U)

static ID s_task_id;
static ID s_cyc_id;
static volatile uint32_t s_handler_stamp_us;
static volatile uint32_t s_handler_count;

/* Runs in handler context: stamp, count, wake the task. Nothing else. */
static void cyc_handler(void *exinf)
{
    (void)exinf;
    s_handler_stamp_us = TIMER_RAWL;
    s_handler_count++;
    (void)tk_wup_tsk(s_task_id);
}

static void cyc_task(INT stacd, void *exinf)
{
    uint32_t last_stamp = 0U;
    uint32_t last_count = 0U;
    uint32_t period_min = UINT32_MAX;
    uint32_t period_max = 0U;
    uint32_t latency_max = 0U;
    uint32_t missed = 0U;
    uint32_t samples = 0U;
    bool have_last = false;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);

        uint32_t stamp = s_handler_stamp_us;
        uint32_t count = s_handler_count;
        /* handler -> task running */
        uint32_t latency = TIMER_RAWL - stamp;

        if (latency > latency_max)
        {
            latency_max = latency;
        }

        if (have_last)
        {
            uint32_t period = stamp - last_stamp; /* handler to handler */
            if (period < period_min)
            {
                period_min = period;
            }
            if (period > period_max)
            {
                period_max = period;
            }
            missed += (count - last_count) - 1U; /* wake-ups that piled up */
        }

        have_last = true;
        last_stamp = stamp;
        last_count = count;
        samples++;

        if (samples >= CYC_REPORT_CYCLES)
        {
            tm_printf((UB *)"cyc: period %u..%u us, max wake latency %u us, "
                            "missed %u\n",
                      (uint32_t)period_min, (uint32_t)period_max,
                      (uint32_t)latency_max, (uint32_t)missed);
            samples = 0U;
            period_min = UINT32_MAX;
            period_max = 0U;
            latency_max = 0U;
            missed = 0U;
            have_last = false; /* skip the cycle disturbed by printing */
        }
    }
}

void cyc_check_init(void)
{
    T_CTSK ctsk = { 0 };
    T_CCYC ccyc = { 0 };

    ctsk.tskatr = TA_HLNG | TA_RNG0;
    ctsk.task = cyc_task;
    ctsk.itskpri = CYC_TASK_PRIORITY;
    ctsk.stksz = CYC_TASK_STACK;
    s_task_id = tk_cre_tsk(&ctsk);

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = cyc_handler;
    ccyc.cyctim = CYC_PERIOD_MS;
    ccyc.cycphs = 0U;
    s_cyc_id = tk_cre_cyc(&ccyc);
}

void cyc_check_start(void)
{
    if ((s_task_id > 0) && (s_cyc_id > 0))
    {
        (void)tk_sta_tsk(s_task_id, 0);
        (void)tk_sta_cyc(s_cyc_id);
    }
    else
    {
        tm_printf((UB *)"cyc_check: object creation failed (task %d, cyc %d)\n",
                  (int32_t)s_task_id, (int32_t)s_cyc_id);
    }
}
