/* Linux software model: RP2040 registers are private anonymous memory. */
#define _GNU_SOURCE
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/mman.h>
#include "motion_hw.c"

static unsigned register_calls;
static unsigned fail_registration;
static bool dispatcher_ok;
static unsigned start_calls;
static char diagnostic[180];

bool gpio_dispatch_register(uint32_t pin, uint32_t events, gpio_dispatch_cb_t callback)
{
    ++register_calls;
    assert(pin == ((register_calls == 1U) ? PIN_ENC_L : PIN_ENC_R));
    assert(events == GPIO_DISPATCH_EDGE_BOTH && callback == motion_encoder_isr);
    return register_calls != fail_registration;
}
bool gpio_dispatch_start(void) { ++start_calls; return dispatcher_ok; }
void gpio_dispatch_status(int32_t *status, uint32_t *inte, uint32_t *calls)
{ *status = 0; *inte = 0U; *calls = 0U; }
int tm_printf(const UB *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = vsnprintf(diagnostic, sizeof(diagnostic), (const char *)format, args);
    va_end(args);
    return result;
}

int main(void)
{
    void *registers = mmap((void *)0x40000000U, 0x60000U, PROT_READ | PROT_WRITE,
                          MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    assert(registers != MAP_FAILED);
    for (unsigned scenario = 0U; scenario < 4U; ++scenario)
    {
        memset(registers, 0, 0x60000U);
        MH_RESETS_RESET_DONE = MH_RST_IO_BANK0 | MH_RST_PADS_BANK0 | MH_RST_PWM;
        register_calls = start_calls = 0U;
        fail_registration = (scenario < 2U) ? scenario + 1U : 0U;
        dispatcher_ok = scenario != 2U;
        diagnostic[0] = '\0';
        assert(motion_hw_init() == (scenario == 3U));
        assert(MH_PWM_CC((MH_L_A >> 1) & 7U) == 0U);
        assert(MH_PWM_CC((MH_R_A >> 1) & 7U) == 0U);
        assert(start_calls == ((scenario < 2U) ? 0U : 1U));
        if (scenario == 0U) assert(strstr(diagnostic, "left encoder") != NULL);
        if (scenario == 1U) assert(strstr(diagnostic, "right encoder") != NULL);
        if (scenario == 2U) assert(strstr(diagnostic, "IRQ startup") != NULL);
    }
    assert(munmap(registers, 0x60000U) == 0);
    puts("PASS S2-HOST-01: either registration/shared IRQ failure aborts initialization with PWM zero");
    return 0;
}
