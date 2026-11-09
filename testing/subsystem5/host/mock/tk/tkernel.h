/**
 * @file    mock/tk/tkernel.h
 * @brief   HOST TEST ONLY. The micro T-Kernel types vc_turn.c needs to
 *          compile in the junction-turn test (nyx, 2026-11-07: the
 *          kernel shim went with the old subsystem 5).
 *
 * Types and constants match include/tk/typedef.h and syscall.h of the
 * team repo. Subsystem 5 itself needs none of this on the host: with
 * S5_HOST, s5_port.h logs through s5_sim.c and s5_sim.c replaces s5_hw.c.
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

#endif /* MOCK_TKERNEL_H */
