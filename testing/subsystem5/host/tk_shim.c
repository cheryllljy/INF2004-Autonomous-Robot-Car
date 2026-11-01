/**
 * @file    tk_shim.c
 * @brief   HOST TEST ONLY. Virtual-clock implementation of the kernel
 *          calls in mock/tk/tkernel.h.
 *
 * Time advances 1 ms at a time, and only when the task under test
 * waits. Each simulated millisecond fires due cyclic handlers and then
 * the idle hook, which plays every other task (VehicleController, the
 * test observer). A run is instant and exactly repeatable.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SHIM_MAX_OBJ    (8)

typedef struct { INT count; INT maxcnt; int used; } shim_sem_t;
typedef struct { UINT ptn; int used; } shim_flg_t;
typedef struct
{
    void    *exinf;
    void   (*hdr)(void *exinf);
    RELTIM   period_ms;
    uint32_t next_due_ms;
    int      running;
    int      used;
} shim_cyc_t;

static shim_sem_t g_sem[SHIM_MAX_OBJ];
static shim_flg_t g_flg[SHIM_MAX_OBJ];
static shim_cyc_t g_cyc[SHIM_MAX_OBJ];
static int        g_mtx_count;

static uint32_t g_now_ms;
static uint32_t g_budget_ms = 60000U;
static int      g_stop;
static int      g_running;
static jmp_buf  g_exit_point;
static void   (*g_idle_hook)(void);

/*=====================================================================*/
/* Clock                                                               */
/*=====================================================================*/

static void shim_tick_1ms(void)
{
    int i;

    g_now_ms++;

    for (i = 0; i < SHIM_MAX_OBJ; i++)
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

    if ((g_stop || (g_now_ms > g_budget_ms)) && g_running)
    {
        longjmp(g_exit_point, 1);
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

void tkshim_reset(void)
{
    memset(g_sem, 0, sizeof(g_sem));
    memset(g_flg, 0, sizeof(g_flg));
    memset(g_cyc, 0, sizeof(g_cyc));
    g_mtx_count = 0;
    g_now_ms    = 0U;
    g_stop      = 0;
    g_running   = 0;
    g_idle_hook = NULL;
}

uint32_t tkshim_now_ms(void)            { return g_now_ms; }
void tkshim_set_time_budget_ms(uint32_t b) { g_budget_ms = b; }
void tkshim_stop(void)                  { g_stop = 1; }
void tkshim_set_idle_hook(void (*hook)(void)) { g_idle_hook = hook; }

void tkshim_run(tkshim_task_t entry)
{
    g_running = 1;

    if (setjmp(g_exit_point) == 0)
    {
        entry(0, NULL);
    }

    g_running = 0;
}

UINT tkshim_flag_pattern(ID flgid)
{
    return ((flgid > 0) && (flgid <= SHIM_MAX_OBJ)) ? g_flg[flgid - 1].ptn
                                                     : 0U;
}

/*=====================================================================*/
/* Semaphores                                                          */
/*=====================================================================*/

ID tk_cre_sem(const T_CSEM *pk)
{
    int i;

    for (i = 0; i < SHIM_MAX_OBJ; i++)
    {
        if (!g_sem[i].used)
        {
            g_sem[i].used   = 1;
            g_sem[i].count  = pk->isemcnt;
            g_sem[i].maxcnt = pk->maxsem;
            return (ID)(i + 1);
        }
    }

    return E_LIMIT;
}

ER tk_sig_sem(ID semid, INT cnt)
{
    shim_sem_t *s = &g_sem[semid - 1];

    if ((s->count + cnt) > s->maxcnt)
    {
        return E_QOVR;
    }

    s->count += cnt;

    return E_OK;
}

ER tk_wai_sem(ID semid, INT cnt, TMO tmout)
{
    shim_sem_t *s       = &g_sem[semid - 1];
    uint32_t    waited  = 0U;

    while (s->count < cnt)
    {
        if ((tmout == TMO_POL) ||
            ((tmout != TMO_FEVR) && (waited >= (uint32_t)tmout)))
        {
            return E_TMOUT;
        }

        shim_tick_1ms();
        waited++;
    }

    s->count -= cnt;

    return E_OK;
}

/*=====================================================================*/
/* Mutex: one task, so locking always succeeds at once                 */
/*=====================================================================*/

ID tk_cre_mtx(const T_CMTX *pk) { (void)pk; g_mtx_count++; return g_mtx_count; }
ER tk_loc_mtx(ID mtxid, TMO tmout) { (void)mtxid; (void)tmout; return E_OK; }
ER tk_unl_mtx(ID mtxid) { (void)mtxid; return E_OK; }

/*=====================================================================*/
/* Event flags                                                         */
/*=====================================================================*/

ID tk_cre_flg(const T_CFLG *pk)
{
    int i;

    for (i = 0; i < SHIM_MAX_OBJ; i++)
    {
        if (!g_flg[i].used)
        {
            g_flg[i].used = 1;
            g_flg[i].ptn  = pk->iflgptn;
            return (ID)(i + 1);
        }
    }

    return E_LIMIT;
}

ER tk_set_flg(ID flgid, UINT setptn)
{
    g_flg[flgid - 1].ptn |= setptn;

    return E_OK;
}

ER tk_clr_flg(ID flgid, UINT clrptn)
{
    g_flg[flgid - 1].ptn &= clrptn;     /* T-Kernel: pattern AND clrptn */

    return E_OK;
}

ER tk_wai_flg(ID flgid, UINT waiptn, UINT wfmode, UINT *p_flgptn, TMO tmout)
{
    shim_flg_t *f      = &g_flg[flgid - 1];
    uint32_t    waited = 0U;

    for (;;)
    {
        const UINT hit = f->ptn & waiptn;
        const int  ok  = ((wfmode & TWF_ORW) != 0U) ? (hit != 0U)
                                                    : (hit == waiptn);

        if (ok)
        {
            *p_flgptn = f->ptn;

            if ((wfmode & TWF_CLR) != 0U)
            {
                f->ptn = 0U;
            }
            else if ((wfmode & TWF_BITCLR) != 0U)
            {
                f->ptn &= ~waiptn;
            }
            else
            {
                /* Leave the pattern. */
            }

            return E_OK;
        }

        if ((tmout == TMO_POL) ||
            ((tmout != TMO_FEVR) && (waited >= (uint32_t)tmout)))
        {
            return E_TMOUT;
        }

        shim_tick_1ms();
        waited++;
    }
}

/*=====================================================================*/
/* Cyclic handlers, task delay, time                                   */
/*=====================================================================*/

ID tk_cre_cyc(const T_CCYC *pk)
{
    int i;

    for (i = 0; i < SHIM_MAX_OBJ; i++)
    {
        if (!g_cyc[i].used)
        {
            g_cyc[i].used      = 1;
            g_cyc[i].exinf     = pk->exinf;
            g_cyc[i].hdr       = (void (*)(void *))pk->cychdr;
            g_cyc[i].period_ms = pk->cyctim;
            g_cyc[i].running   = ((pk->cycatr & TA_STA) != 0U);
            g_cyc[i].next_due_ms = g_now_ms + pk->cycphs;
            return (ID)(i + 1);
        }
    }

    return E_LIMIT;
}

ER tk_sta_cyc(ID cycid)
{
    shim_cyc_t *c = &g_cyc[cycid - 1];

    c->running     = 1;
    c->next_due_ms = g_now_ms + c->period_ms;   /* no TA_PHS: restart */

    return E_OK;
}

ER tk_stp_cyc(ID cycid)
{
    g_cyc[cycid - 1].running = 0;

    return E_OK;
}

ER tk_dly_tsk(RELTIM dlytim)
{
    shim_advance_ms((uint32_t)dlytim);

    return E_OK;
}

ER tk_slp_tsk(TMO tmout)
{
    shim_advance_ms((tmout == TMO_FEVR) ? 1000000U : (uint32_t)tmout);

    return E_TMOUT;
}

ID tk_get_tid(void)
{
    return 1;
}

ER tk_get_otm(SYSTIM *pk_tim)
{
    pk_tim->hi = 0;
    pk_tim->lo = g_now_ms;

    return E_OK;
}

INT tm_printf(const UB *format, ...)
{
    va_list ap;
    int     n;

    va_start(ap, format);
    n = vprintf((const char *)format, ap);
    va_end(ap);

    return n;
}
