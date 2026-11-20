/* Host stub of the micro T-Kernel API, only what motion_control.c uses. */
#ifndef TK_STUB_H
#define TK_STUB_H
#include <stdint.h>
#include <string.h>

typedef int INT;
typedef unsigned int UINT;
typedef int ID;
typedef int ER;
typedef int TMO;
typedef int PRI;
typedef long SZ;
typedef unsigned int ATR;
typedef unsigned int RELTIM;
typedef unsigned char UB;
typedef void (*FP)();

typedef struct { int32_t hi; uint32_t lo; } SYSTIM;

#define E_OK 0
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define TA_HLNG 1U
#define TA_RNG0 0U
#define TA_TFIFO 0U
#define TA_WMUL 8U
#define TA_INHERIT 2U

typedef struct { void *exinf; ATR tskatr; void (*task)(INT, void *); PRI itskpri; SZ stksz; } T_CTSK;
typedef struct { void *exinf; ATR cycatr; void (*cychdr)(void *); RELTIM cyctim; RELTIM cycphs; } T_CCYC;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; } T_CMBF;
typedef struct { void *exinf; ATR mtxatr; PRI ceilpri; } T_CMTX;
typedef struct { void *exinf; ID wtsk; UINT flgptn; } T_RFLG;

ER tk_get_otm(SYSTIM *t);
ER tk_cre_tsk(const T_CTSK *p);
ER tk_sta_tsk(ID id, INT s);
ER tk_cre_cyc(const T_CCYC *p);
ER tk_sta_cyc(ID id);
ER tk_cre_flg(const T_CFLG *p);
ER tk_set_flg(ID id, UINT p);
ER tk_clr_flg(ID id, UINT p);
ER tk_ref_flg(ID id, T_RFLG *r);
ER tk_cre_mbf(const T_CMBF *p);
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO t);
INT tk_rcv_mbf(ID id, void *m, TMO t);
ER tk_cre_mtx(const T_CMTX *p);
ER tk_loc_mtx(ID id, TMO t);
ER tk_unl_mtx(ID id);
ER tk_slp_tsk(TMO t);
INT tk_can_wup(ID id);
ER tk_wup_tsk(ID id);
#endif
