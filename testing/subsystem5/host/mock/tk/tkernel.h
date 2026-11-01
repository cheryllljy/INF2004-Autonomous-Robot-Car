/**
 * @file    mock/tk/tkernel.h
 * @brief   HOST TEST ONLY. The micro T-Kernel API subset subsystem 5
 *          uses, backed by a virtual clock (tk_shim.c).
 *
 * Types and constants match include/tk/typedef.h and syscall.h of the
 * team repo, so app.h, subsystem5.h and obstacle_task.c compile here
 * unchanged. This is NOT a scheduler: one task runs as a plain function
 * and time only advances when it waits. That is enough to check state
 * transitions, the timing budget, and fault handling repeatably.
 */
#ifndef MOCK_TKERNEL_H
#define MOCK_TKERNEL_H

#include <stdint.h>

typedef signed char     B;
typedef short           H;
typedef int             W;
typedef unsigned char   UB;
typedef unsigned short  UH;
typedef unsigned int    UW;
typedef int             INT;
typedef unsigned int    UINT;
typedef int             ID;
typedef int             ER;
typedef int             PRI;
typedef unsigned int    ATR;
typedef int             TMO;
typedef unsigned int    RELTIM;
typedef long            SZ;
typedef unsigned int    BOOL;
typedef void          (*FP)();

#ifndef NULL
#define NULL            ((void *)0)    /* as include/tk/typedef.h */
#endif

#define CONST           const
#define LOCAL           static
#define EXPORT
#define IMPORT          extern
#define TRUE            (1)
#define FALSE           (0)

typedef struct systim
{
    W  hi;
    UW lo;
} SYSTIM;

#define E_OK            (0)
#define E_PAR           (-17)
#define E_ID            (-18)
#define E_CTX           (-25)
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

#define TP_PRC1         (0x00000001U)
#define TP_PRC2         (0x00000002U)

typedef struct { void *exinf; ATR sematr; INT isemcnt; INT maxsem; } T_CSEM;
typedef struct { void *exinf; ATR mtxatr; PRI ceilpri; } T_CMTX;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ATR cycatr; FP cychdr;
                 RELTIM cyctim; RELTIM cycphs; } T_CCYC;

ID tk_cre_sem(const T_CSEM *pk_csem);
ER tk_wai_sem(ID semid, INT cnt, TMO tmout);
ER tk_sig_sem(ID semid, INT cnt);

ID tk_cre_mtx(const T_CMTX *pk_cmtx);
ER tk_loc_mtx(ID mtxid, TMO tmout);
ER tk_unl_mtx(ID mtxid);

ID tk_cre_flg(const T_CFLG *pk_cflg);
ER tk_set_flg(ID flgid, UINT setptn);
ER tk_clr_flg(ID flgid, UINT clrptn);
ER tk_wai_flg(ID flgid, UINT waiptn, UINT wfmode, UINT *p_flgptn, TMO tmout);

ID tk_cre_cyc(const T_CCYC *pk_ccyc);
ER tk_sta_cyc(ID cycid);
ER tk_stp_cyc(ID cycid);

ER tk_dly_tsk(RELTIM dlytim);
ER tk_slp_tsk(TMO tmout);
ID tk_get_tid(void);
ER tk_get_otm(SYSTIM *pk_tim);

/* ---- shim control (host only) ---- */
typedef void (*tkshim_task_t)(INT stacd, void *exinf);

/** Run one task function until tkshim_stop() or the time budget. */
void     tkshim_run(tkshim_task_t entry);
void     tkshim_stop(void);
void     tkshim_reset(void);
uint32_t tkshim_now_ms(void);
void     tkshim_set_time_budget_ms(uint32_t budget_ms);
/** Called once per simulated millisecond (the "other tasks"). */
void     tkshim_set_idle_hook(void (*hook)(void));
/** Current pattern of a flag, for test observers. */
UINT     tkshim_flag_pattern(ID flgid);

#endif /* MOCK_TKERNEL_H */
