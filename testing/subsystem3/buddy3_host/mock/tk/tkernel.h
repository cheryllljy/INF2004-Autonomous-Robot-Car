/* Minimal API shape mirroring the supplied mtk3smp-rp2040 headers.
 * This is a host test double, not a kernel implementation. */
#ifndef MOCK_TKERNEL_H
#define MOCK_TKERNEL_H
#include <stdint.h>
#include <stddef.h>

typedef int INT;
typedef int ER;
typedef int ID;
typedef int PRI;
typedef int32_t SZ;
typedef uint32_t ATR;
typedef uint32_t UW;
typedef unsigned int UINT;
typedef unsigned char UB;
typedef void (*FP)();

#define EXPORT

typedef struct
{
    ATR intatr;
    FP inthdr;
} T_DINT;

typedef struct
{
    void *exinf;
    ATR tskatr;
    FP task;
    PRI itskpri;
    SZ stksz;
    void *bufptr;
    UW assprc;
} T_CTSK;

typedef uint32_t RELTIM;

typedef struct
{
    void *exinf;
    ATR cycatr;
    FP cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
} T_CCYC;

#define TA_STA      0x00000002UL
#define TA_HLNG     0x00000001UL
#define TA_RNG3     0x00000300UL
#define TA_ASSPRC   0x00010000UL
#define TP_PRC1     0x00000001UL
#define TK_SUPPORT_SMP 1
#define E_OK        0
#define E_CTX       (-25)
#define E_OBJ       (-41)
#define TMO_FEVR    (-1)
#define E_NOSPT     (-9)
#define E_IO        (-57)


/* RP2040/system-library symbols needed only for compile-checking the real port adapter. */
#define IO_BANK0_BASE   0x40014000U
#define PADS_BANK0_BASE 0x4001C000U
#define GPIO_IN         0xD0000004U
#define RESETS_RESET        0x4000C000U
#define RESETS_RESET_DONE   0x4000C008U
#define RESETS_RESET_TIMER  0x00200000U
#define RESETS_RESET_ADC    0x00000001U
#define XOSC_MHz            (12)
#define TIMER_TIMERAWL  0x40054028U
#define GPIO(n)         (PADS_BANK0_BASE + 0x4U + ((n) * 4U))
#define GPIO_IE         (1U << 6)
#define GPIO_PUE        (1U << 3)
#define GPIO_PDE        (1U << 2)
#define GPIO_SHEMITT    (1U << 1)

UW in_w(UW port);
void out_w(UW port, UW data);
void set_w(UW port, UW data);
void clr_w(UW port, UW data);
ER tk_def_int(UINT intno, const T_DINT *p_dint);
void EnableInt(UINT intno, INT level);
void ClearInt(UINT intno);
INT tk_get_prc(void);

ID tk_cre_tsk(const T_CTSK *p_create);
ER tk_sta_tsk(ID id, INT code);
ER tk_del_tsk(ID id);
ER tk_ter_tsk(ID id);
ER tk_wup_tsk(ID id);
ID tk_cre_cyc(const T_CCYC *p_create);
ER tk_dly_tsk(uint32_t milliseconds);
ER tk_slp_tsk(int32_t timeout);
void tk_ext_tsk(void);

#endif
