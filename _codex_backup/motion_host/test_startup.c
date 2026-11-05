/* Buddy 3 / Codex, 2026-11-05: real subsystem wrapper must propagate errors. */
#include <assert.h>
#include <stdio.h>
#include <tk/tkernel.h>
#define SUBSYSTEM2_H
#define S2_POOL_COUNT 4
#define S2_PRI_PRODUCER 14
typedef struct { unsigned char unused[32]; } SAMPLE_MSG;
void producer_task(INT code, void *data) { (void)code; (void)data; }
ID app_start_task(FP entry, PRI priority, UW core, const char *name);
#include "subsystem2.c"

static bool init_ok, start_ok;
static unsigned objects;
bool motion_init(void) { return init_ok; }
bool motion_start(void) { return start_ok; }
ID tk_cre_sem(const T_CSEM *p) { (void)p; ++objects; return 1; }
ID tk_cre_mpf(const T_CMPF *p) { (void)p; ++objects; return 2; }
ID app_start_task(FP entry, PRI priority, UW core, const char *name)
{ (void)entry; (void)priority; (void)core; (void)name; return 3; }

int main(void)
{
    init_ok = false;
    assert(subsystem2_init() == E_SYS && objects == 0U);
    init_ok = true;
    assert(subsystem2_init() == E_OK && objects == 2U);
    start_ok = false;
    assert(subsystem2_start() == E_SYS);
    start_ok = true;
    assert(subsystem2_start() == E_OK);
    puts("PASS S2-HOST-06: motion init/start failures propagate to usermain");
    return 0;
}
