/* HOST TEST ONLY. tm_printf goes to stdout. */
#ifndef MOCK_TMONITOR_H
#define MOCK_TMONITOR_H

#include <tk/tkernel.h>

IMPORT INT tm_printf(const UB *format, ...);

#endif /* MOCK_TMONITOR_H */
