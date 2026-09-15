/**
 * @file    tk_shim.c
 * @brief   Implementation of the micro T-Kernel API subset.
 *
 * Two backends selected at compile time:
 *
 *   TKSHIM_HOST defined   virtual clock, no hardware. Time only
 *                         advances when the code under test asks to
 *                         wait, so a simulated run is instant and
 *                         perfectly repeatable.
 *
 *   otherwise             Pico SDK. Real timer, real interrupts,
 *                         cyclic handlers on the alarm pool.
 *
 * Neither backend is a scheduler. See shim/tk/tkernel.h for the
 * limits this places on what the bench numbers mean.
 */

#include <tk/tkernel.h>

#include <string.h>

#ifdef TKSHIM_HOST
#include <setjmp.h>
#else
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#endif

/*=====================================================================*/
/* Object tables. Static, small, no allocation.                        */
/*=====================================================================*/

#define SHIM_MAX_SEM        (4)
#define SHIM_MAX_MTX        (4)
#define SHIM_MAX_FLG        (4)
#define SHIM_MAX_CYC        (4)

typedef struct
{
    volatile INT count;
    INT          maxcnt;
    int          used;
} shim_sem_t;

typedef struct
{
    volatile UINT ptn;
    int           used;
} shim_flg_t;

typedef struct
{
    void  *exinf;
    void (*hdr)(void *exinf);
    RELTIM period_ms;
    int    running;
    int    used;
#ifdef TKSHIM_HOST
    uint32_t next_due_ms;
#else
    repeating_timer_t timer;
#endif
} shim_cyc_t;

static shim_sem_t g_sem[SHIM_MAX_SEM];
static shim_flg_t g_flg[SHIM_MAX_FLG];
static shim_cyc_t g_cyc[SHIM_MAX_CYC];
static int        g_mtx_used[SHIM_MAX_MTX];

static void (*g_task_entry)(INT, void *) = NULL;
static void  *g_task_exinf               = NULL;
static void (*g_idle_hook)(void)         = NULL;

/*=====================================================================*/
/* Time base                                                           */
/*=====================================================================*/

#ifdef TKSHIM_HOST

static uint32_t g_now_ms     = 0U;
static uint32_t g_budget_ms  = 60000U;
static int      g_stop       = 0;
static jmp_buf  g_exit_point;
static int      g_running    = 0;

uint32_t tkshim_now_ms(void)
{
    return g_now_ms;
}

void tkshim_set_time_budget_ms(uint32_t budget_ms)
{
    g_budget_ms = budget_ms;
}

void tkshim_stop(void)
{
    g_stop = 1;
}

/**
 * @brief   Advance virtual time by one millisecond, firing any cyclic
 *          handler that comes due.
 */
static void shim_tick_1ms(void)
{
    int i;

    g_now_ms++;

    for (i = 0; i < SHIM_MAX_CYC; i++)
    {
        if (g_cyc[i].used && g_cyc[i].running &&
            (g_now_ms >= g_cyc[i].next_due_ms))
        {
            g_cyc[i].next_due_ms = g_now_ms + g_cyc[i].period_ms;
            g_cyc[i].hdr(g_cyc[i].exinf);
        }
    }

    if (g_idle_hook != NULL)
    {
        g_idle_hook();
    }

    if (g_stop || (g_now_ms > g_budget_ms))
    {
        if (g_running)
        {
            longjmp(g_exit_point, 1);
        }
    }
}

static void shim_advance_ms(uint32_t ms)
{
    uint32_t i;

    for (i = 0U; i < ms; i++)
    {
        shim_tick_1ms();
    }
}

#else /* Pico SDK backend */

uint32_t tkshim_now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void tkshim_set_time_budget_ms(uint32_t budget_ms)
{
    (void)budget_ms;
}

void tkshim_stop(void)
{
    /* Nothing to do: the target build never leaves the task. */
}

static bool shim_timer_cb(repeating_timer_t *t)
{
    shim_cyc_t *c = (shim_cyc_t *)t->user_data;

    if ((c != NULL) && (c->hdr != NULL))
    {
        c->hdr(c->exinf);
    }

    return true;
}

#endif /* TKSHIM_HOST */

void tkshim_set_idle_hook(void (*hook)(void))
{
    g_idle_hook = hook;
}

/*=====================================================================*/
/* Semaphores                                                          */
/*=====================================================================*/

ID tk_cre_sem(const T_CSEM *pk_csem)
{
    ID  id = E_LIMIT;
    int i;

    if (pk_csem != NULL)
    {
        for (i = 0; (i < SHIM_MAX_SEM) && (id < 0); i++)
        {
            if (!g_sem[i].used)
            {
                g_sem[i].used   = 1;
                g_sem[i].count  = pk_csem->isemcnt;
                g_sem[i].maxcnt = pk_csem->maxsem;
                id              = i + 1;
            }
        }
    }

    return id;
}

ER tk_sig_sem(ID semid, INT cnt)
{
    ER er = E_ID;

    if ((semid >= 1) && (semid <= SHIM_MAX_SEM) && g_sem[semid - 1].used)
    {
        shim_sem_t *s = &g_sem[semid - 1];

#ifndef TKSHIM_HOST
        uint32_t irq = save_and_disable_interrupts();
#endif
        if ((s->count + cnt) > s->maxcnt)
        {
            er = E_QOVR;
        }
        else
        {
            s->count += cnt;
            er = E_OK;
        }
#ifndef TKSHIM_HOST
        restore_interrupts(irq);
#endif
    }

    return er;
}

ER tk_wai_sem(ID semid, INT cnt, TMO tmout)
{
    ER er = E_ID;

    if ((semid >= 1) && (semid <= SHIM_MAX_SEM) && g_sem[semid - 1].used)
    {
        shim_sem_t    *s       = &g_sem[semid - 1];
        const uint32_t started = tkshim_now_ms();
        int            waiting = 1;

        er = E_TMOUT;

        while (waiting)
        {
            int got = 0;

#ifndef TKSHIM_HOST
            uint32_t irq = save_and_disable_interrupts();
#endif
            if (s->count >= cnt)
            {
                s->count -= cnt;
                got = 1;
            }
#ifndef TKSHIM_HOST
            restore_interrupts(irq);
#endif

            if (got)
            {
                er      = E_OK;
                waiting = 0;
            }
            else if (tmout == TMO_POL)
            {
                waiting = 0;
            }
            else
            {
#ifdef TKSHIM_HOST
                shim_tick_1ms();
#else
                if (g_idle_hook != NULL)
                {
                    g_idle_hook();
                }
                tight_loop_contents();
#endif
                if ((tmout != TMO_FEVR) &&
                    ((tkshim_now_ms() - started) >= (uint32_t)tmout))
                {
                    waiting = 0;
                }
            }
        }
    }

    return er;
}

/*=====================================================================*/
/* Mutexes. One task, so locking is bookkeeping only.                  */
/*=====================================================================*/

ID tk_cre_mtx(const T_CMTX *pk_cmtx)
{
    ID  id = E_LIMIT;
    int i;

    if (pk_cmtx != NULL)
    {
        for (i = 0; (i < SHIM_MAX_MTX) && (id < 0); i++)
        {
            if (!g_mtx_used[i])
            {
                g_mtx_used[i] = 1;
                id            = i + 1;
            }
        }
    }

    return id;
}

ER tk_loc_mtx(ID mtxid, TMO tmout)
{
    (void)tmout;

    return ((mtxid >= 1) && (mtxid <= SHIM_MAX_MTX)) ? E_OK : E_ID;
}

ER tk_unl_mtx(ID mtxid)
{
    return ((mtxid >= 1) && (mtxid <= SHIM_MAX_MTX)) ? E_OK : E_ID;
}

/*=====================================================================*/
/* Event flags                                                         */
/*=====================================================================*/

ID tk_cre_flg(const T_CFLG *pk_cflg)
{
    ID  id = E_LIMIT;
    int i;

    if (pk_cflg != NULL)
    {
        for (i = 0; (i < SHIM_MAX_FLG) && (id < 0); i++)
        {
            if (!g_flg[i].used)
            {
                g_flg[i].used = 1;
                g_flg[i].ptn  = pk_cflg->iflgptn;
                id            = i + 1;
            }
        }
    }

    return id;
}

ER tk_set_flg(ID flgid, UINT setptn)
{
    ER er = E_ID;

    if ((flgid >= 1) && (flgid <= SHIM_MAX_FLG) && g_flg[flgid - 1].used)
    {
        g_flg[flgid - 1].ptn |= setptn;
        er = E_OK;
    }

    return er;
}

ER tk_clr_flg(ID flgid, UINT clrptn)
{
    ER er = E_ID;

    if ((flgid >= 1) && (flgid <= SHIM_MAX_FLG) && g_flg[flgid - 1].used)
    {
        g_flg[flgid - 1].ptn &= clrptn;
        er = E_OK;
    }

    return er;
}

ER tk_wai_flg(ID flgid, UINT waiptn, UINT wfmode,
              UINT *p_flgptn, TMO tmout)
{
    ER er = E_ID;

    if ((flgid >= 1) && (flgid <= SHIM_MAX_FLG) && g_flg[flgid - 1].used)
    {
        shim_flg_t    *f       = &g_flg[flgid - 1];
        const uint32_t started = tkshim_now_ms();
        int            waiting = 1;

        er = E_TMOUT;

        while (waiting)
        {
            const UINT cur = f->ptn;
            int        hit;

            if ((wfmode & TWF_ORW) != 0U)
            {
                hit = ((cur & waiptn) != 0U);
            }
            else
            {
                hit = ((cur & waiptn) == waiptn);
            }

            if (hit)
            {
                if (p_flgptn != NULL)
                {
                    *p_flgptn = cur;
                }

                if ((wfmode & TWF_BITCLR) != 0U)
                {
                    f->ptn &= ~waiptn;
                }
                else if ((wfmode & TWF_CLR) != 0U)
                {
                    f->ptn = 0U;
                }
                else
                {
                    /* Leave the pattern alone. */
                }

                er      = E_OK;
                waiting = 0;
            }
            else if (tmout == TMO_POL)
            {
                waiting = 0;
            }
            else
            {
#ifdef TKSHIM_HOST
                shim_tick_1ms();
#else
                if (g_idle_hook != NULL)
                {
                    g_idle_hook();
                }
                tight_loop_contents();
#endif
                if ((tmout != TMO_FEVR) &&
                    ((tkshim_now_ms() - started) >= (uint32_t)tmout))
                {
                    waiting = 0;
                }
            }
        }
    }

    return er;
}

/*=====================================================================*/
/* Cyclic handlers                                                     */
/*=====================================================================*/

ID tk_cre_cyc(const T_CCYC *pk_ccyc)
{
    ID  id = E_LIMIT;
    int i;

    if ((pk_ccyc != NULL) && (pk_ccyc->cychdr != NULL))
    {
        for (i = 0; (i < SHIM_MAX_CYC) && (id < 0); i++)
        {
            if (!g_cyc[i].used)
            {
                g_cyc[i].used      = 1;
                g_cyc[i].running   = 0;
                g_cyc[i].exinf     = pk_ccyc->exinf;
                g_cyc[i].hdr       = pk_ccyc->cychdr;
                g_cyc[i].period_ms = pk_ccyc->cyctim;
                id                 = i + 1;
            }
        }
    }

    return id;
}

ER tk_sta_cyc(ID cycid)
{
    ER er = E_ID;

    if ((cycid >= 1) && (cycid <= SHIM_MAX_CYC) && g_cyc[cycid - 1].used)
    {
        shim_cyc_t *c = &g_cyc[cycid - 1];

        if (!c->running)
        {
#ifdef TKSHIM_HOST
            c->next_due_ms = tkshim_now_ms() + c->period_ms;
#else
            (void)add_repeating_timer_ms((int32_t)c->period_ms,
                                         shim_timer_cb, c, &c->timer);
#endif
            c->running = 1;
        }

        er = E_OK;
    }

    return er;
}

ER tk_stp_cyc(ID cycid)
{
    ER er = E_ID;

    if ((cycid >= 1) && (cycid <= SHIM_MAX_CYC) && g_cyc[cycid - 1].used)
    {
        shim_cyc_t *c = &g_cyc[cycid - 1];

        if (c->running)
        {
#ifndef TKSHIM_HOST
            (void)cancel_repeating_timer(&c->timer);
#endif
            c->running = 0;
        }

        er = E_OK;
    }

    return er;
}

/*=====================================================================*/
/* Tasks. Exactly one is supported.                                    */
/*=====================================================================*/

ID tk_cre_tsk(const T_CTSK *pk_ctsk)
{
    ID id = E_LIMIT;

    if ((pk_ctsk != NULL) && (pk_ctsk->task != NULL) &&
        (g_task_entry == NULL))
    {
        g_task_entry = pk_ctsk->task;
        g_task_exinf = pk_ctsk->exinf;
        id           = 1;
    }

    return id;
}

ER tk_sta_tsk(ID tskid, INT stacd)
{
    (void)stacd;

    /* The task is not started here. tkshim_run() runs it on the main
     * stack, because there is no scheduler to switch to. */
    return (tskid == 1) ? E_OK : E_ID;
}

ER tk_dly_tsk(RELTIM dlytim)
{
#ifdef TKSHIM_HOST
    shim_advance_ms(dlytim);
#else
    const uint32_t started = tkshim_now_ms();

    while ((tkshim_now_ms() - started) < dlytim)
    {
        if (g_idle_hook != NULL)
        {
            g_idle_hook();
        }
        tight_loop_contents();
    }
#endif

    return E_OK;
}

ER tk_slp_tsk(TMO tmout)
{
    if (tmout == TMO_FEVR)
    {
        for (;;)
        {
#ifdef TKSHIM_HOST
            shim_tick_1ms();
#else
            if (g_idle_hook != NULL)
            {
                g_idle_hook();
            }
            tight_loop_contents();
#endif
        }
    }
    else if (tmout > 0)
    {
        (void)tk_dly_tsk((RELTIM)tmout);
    }
    else
    {
        /* TMO_POL: return immediately. */
    }

    return E_OK;
}

ER tk_get_otm(SYSTIM *pk_tim)
{
    ER er = E_PAR;

    if (pk_tim != NULL)
    {
        pk_tim->hi = 0;
        pk_tim->lo = (UW)tkshim_now_ms();
        er         = E_OK;
    }

    return er;
}

/*=====================================================================*/
/* Run the single task                                                 */
/*=====================================================================*/

void tkshim_run(void)
{
    if (g_task_entry != NULL)
    {
#ifdef TKSHIM_HOST
        g_running = 1;

        if (setjmp(g_exit_point) == 0)
        {
            g_task_entry(0, g_task_exinf);
        }

        g_running = 0;
#else
        g_task_entry(0, g_task_exinf);
#endif
    }
}
