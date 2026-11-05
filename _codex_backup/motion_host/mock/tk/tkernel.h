/* Software-only motion regression boundary. No real kernel or GPIO access. */
#ifndef MOTION_HOST_TKERNEL_H
#define MOTION_HOST_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int INT;
typedef int ER;
typedef int ID;
typedef int SZ;
typedef int PRI;
typedef unsigned int UINT;
typedef uint32_t UW;
typedef unsigned char UB;
typedef void (*FP)();
typedef struct { uint32_t hi, lo; } SYSTIM;
typedef struct { UINT flgptn; } T_RFLG;
typedef struct { UINT mtxatr; } T_CMTX;
typedef struct { UINT mbfatr; SZ bufsz; INT maxmsz; } T_CMBF;
typedef struct { UINT flgatr; UINT iflgptn; } T_CFLG;
typedef struct { UINT tskatr; FP task; PRI itskpri; SZ stksz; } T_CTSK;
typedef struct { UINT cycatr; FP cychdr; uint32_t cyctim, cycphs; } T_CCYC;
typedef struct { UINT sematr; INT isemcnt, maxsem; } T_CSEM;
typedef struct { UINT mpfatr; INT mpfcnt; SZ blfsz; } T_CMPF;
#define EXPORT
#define TA_INHERIT 1U
#define TA_TFIFO 0U
#define TA_WMUL 2U
#define TA_HLNG 1U
#define TA_RNG0 0U
#define TA_RNG3 0x300U
#define TP_PRC2 2U
#define E_OK 0
#define E_SYS (-5)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TSK_SELF 0
ER tk_ref_flg(ID, T_RFLG *);
ER tk_set_flg(ID, UINT);
ER tk_clr_flg(ID, UINT);
ER tk_get_otm(SYSTIM *);
INT tk_rcv_mbf(ID, void *, INT);
ER tk_snd_mbf(ID, const void *, INT, INT);
ER tk_loc_mtx(ID, INT);
ER tk_unl_mtx(ID);
ER tk_slp_tsk(INT);
INT tk_can_wup(ID);
ER tk_wup_tsk(ID);
ID tk_cre_mtx(const T_CMTX *);
ID tk_cre_mbf(const T_CMBF *);
ID tk_cre_flg(const T_CFLG *);
ID tk_cre_tsk(const T_CTSK *);
ID tk_cre_cyc(const T_CCYC *);
ER tk_sta_tsk(ID, INT);
ER tk_sta_cyc(ID);
ID tk_cre_sem(const T_CSEM *);
ID tk_cre_mpf(const T_CMPF *);
#endif
