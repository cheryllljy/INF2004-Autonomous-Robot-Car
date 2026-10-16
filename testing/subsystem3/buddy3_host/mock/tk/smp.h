#ifndef MOCK_TK_SMP_H
#define MOCK_TK_SMP_H
#include <tk/tkernel.h>
typedef volatile UW T_SPLOCK;
ER InitSpinLock(T_SPLOCK *lock);
ER ISpinLock(T_SPLOCK *lock, UINT *intsts);
ER ISpinUnlock(T_SPLOCK *lock, UINT intsts);
#endif
