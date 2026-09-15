/**
 * @file    shim/tk/tkernel.h
 * @brief   Bring-up shim: the micro T-Kernel 3.0 API subset this
 *          subsystem uses, implemented on the Pico SDK or on a
 *          virtual clock for host simulation.
 *
 * WHY THIS EXISTS
 * ---------------
 * The final robot runs micro T-Kernel 3.0, but the kernel BSP is not
 * always integrated when you want to bring the sensor up on the bench.
 * This shim provides exactly the calls obstacle.c and ultrasonic.c
 * make, so the subsystem compiles and runs in three configurations
 * with ZERO source changes:
 *
 *   1. mtk3       real kernel, real hardware        (the deliverable)
 *   2. baremetal  this shim + Pico SDK, one task    (bench bring-up)
 *   3. host       this shim + virtual clock         (laptop testing)
 *
 * The include path decides which. CMake adds shim/ ahead of nothing
 * in baremetal mode, and the BSP's own include path replaces it in
 * mtk3 mode. src/obstacle.c and src/ultrasonic.c never change.
 *
 * IMPORTANT LIMITS OF THE SHIM
 * ----------------------------
 * - One task only. There is no scheduler and no preemption.
 * - tk_wai_sem() busy-waits (or advances virtual time on the host)
 *   instead of blocking. The CPU-cost argument for the interrupt
 *   driven driver therefore only holds under the real kernel. Do not
 *   quote shim timings as RTOS timings in the report.
 * - Mutexes are no-ops, which is correct only because there is one
 *   task and the ISR never takes one.
 *
 * It is a test harness, not a kernel. The graded build uses mtk3.
 */
#ifndef TKSHIM_TKERNEL_H
#define TKSHIM_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

/*=====================================================================*/
/* Types (names and widths match micro T-Kernel 3.0)                   */
/*=====================================================================*/

typedef int             INT;
typedef unsigned int    UINT;
typedef int             W;
typedef unsigned int    UW;
typedef int             ID;
typedef int             ER;
typedef int             PRI;
typedef unsigned int    ATR;
typedef int             TMO;
typedef unsigned int    RELTIM;
typedef unsigned int    SZ;
typedef int             BOOL;

typedef struct t_systim
{
    W  hi;
    UW lo;
} SYSTIM;

/*=====================================================================*/
/* Error codes and constants                                           */
/*=====================================================================*/

#define E_OK            (0)
#define E_PAR           (-17)
#define E_ID            (-18)
#define E_LIMIT         (-34)
#define E_QOVR          (-43)
#define E_TMOUT         (-50)

#define TMO_POL         (0)
#define TMO_FEVR        (-1)

#define TA_HLNG         (0x00000001U)
#define TA_RNG3         (0x00000300U)
#define TA_TFIFO        (0x00000000U)
#define TA_TPRI         (0x00000001U)
#define TA_FIRST        (0x00000000U)
#define TA_CNT          (0x00000002U)
#define TA_INHERIT      (0x00000002U)
#define TA_WSGL         (0x00000000U)
#define TA_WMUL         (0x00000008U)
#define TA_STA          (0x00000002U)
#define TA_PHS          (0x00000004U)

#define TWF_ANDW        (0x00000000U)
#define TWF_ORW         (0x00000001U)
#define TWF_CLR         (0x00000010U)
#define TWF_BITCLR      (0x00000020U)

/*=====================================================================*/
/* Creation packets                                                    */
/*=====================================================================*/

typedef struct t_csem
{
    void *exinf;
    ATR   sematr;
    INT   isemcnt;
    INT   maxsem;
} T_CSEM;

typedef struct t_cmtx
{
    void *exinf;
    ATR   mtxatr;
    PRI   ceilpri;
} T_CMTX;

typedef struct t_cflg
{
    void *exinf;
    ATR   flgatr;
    UINT  iflgptn;
} T_CFLG;

typedef struct t_ccyc
{
    void  *exinf;
    ATR    cycatr;
    void (*cychdr)(void *exinf);
    RELTIM cyctim;
    RELTIM cycphs;
} T_CCYC;

typedef struct t_ctsk
{
    void  *exinf;
    ATR    tskatr;
    void (*task)(INT stacd, void *exinf);
    PRI    itskpri;
    SZ     stksz;
} T_CTSK;

/*=====================================================================*/
/* API subset                                                          */
/*=====================================================================*/

ID tk_cre_sem(const T_CSEM *pk_csem);
ER tk_wai_sem(ID semid, INT cnt, TMO tmout);
ER tk_sig_sem(ID semid, INT cnt);

ID tk_cre_mtx(const T_CMTX *pk_cmtx);
ER tk_loc_mtx(ID mtxid, TMO tmout);
ER tk_unl_mtx(ID mtxid);

ID tk_cre_flg(const T_CFLG *pk_cflg);
ER tk_set_flg(ID flgid, UINT setptn);
ER tk_clr_flg(ID flgid, UINT clrptn);
ER tk_wai_flg(ID flgid, UINT waiptn, UINT wfmode,
              UINT *p_flgptn, TMO tmout);

ID tk_cre_cyc(const T_CCYC *pk_ccyc);
ER tk_sta_cyc(ID cycid);
ER tk_stp_cyc(ID cycid);

ID tk_cre_tsk(const T_CTSK *pk_ctsk);
ER tk_sta_tsk(ID tskid, INT stacd);
ER tk_dly_tsk(RELTIM dlytim);
ER tk_slp_tsk(TMO tmout);

ER tk_get_otm(SYSTIM *pk_tim);

/*=====================================================================*/
/* Shim-only control surface. None of this exists in the real kernel,  */
/* so it must appear only in main.c and in test/, never in src/        */
/* obstacle.c, ultrasonic.c, servo.c or obstacle_geometry.c.           */
/*=====================================================================*/

/** Hand the CPU to the single created task. Does not return in the
 *  baremetal build; returns on the host when the time budget runs
 *  out or tkshim_stop() is called. */
void tkshim_run(void);

/** Ask tkshim_run() to return. Host build only; ignored on target. */
void tkshim_stop(void);

/** Called repeatedly from inside the shim's wait loops. Use it to
 *  print monitoring output without adding a second task. */
void tkshim_set_idle_hook(void (*hook)(void));

/** Virtual (host) or real (target) milliseconds since start. */
uint32_t tkshim_now_ms(void);

/** Host build: stop advancing virtual time past this many ms. */
void tkshim_set_time_budget_ms(uint32_t budget_ms);

#endif /* TKSHIM_TKERNEL_H */
