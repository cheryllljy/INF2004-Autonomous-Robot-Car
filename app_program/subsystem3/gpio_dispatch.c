/*
 * gpio_dispatch.c - one IO_IRQ_BANK0 handler shared by all subsystems.
 * See gpio_dispatch.h. Uses the core 0 (PROC0) interrupt registers.
 * Owner: Buddy 3 (moved from subsystem2/, code written by Buddy 2).
 */
#include <stdint.h>
#include <stdbool.h>
#include "gpio_dispatch.h"

#include <tk/tkernel.h>

#define GD_REG32(addr)      (*(volatile uint32_t *)(addr))
#define GD_SIO_CPUID        GD_REG32(0xD0000000U)   /* 0 on core 0, 1 on core 1 */
#define GD_IO_BANK0_BASE    (0x40014000U)
#define GD_INTR(reg)        GD_REG32(GD_IO_BANK0_BASE + 0x0F0U + (4U * (reg)))   /* latched edge events (write 1 to clear) */
#define GD_INTE(reg)        GD_REG32(GD_IO_BANK0_BASE + 0x100U + (4U * (reg)))   /* core 0 enable  */
#define GD_INTS(reg)        GD_REG32(GD_IO_BANK0_BASE + 0x120U + (4U * (reg)))   /* core 0 status  */

typedef struct
{
    uint32_t pin;
    uint32_t events;
    gpio_dispatch_cb_t cb;
} gd_entry_t;

static gd_entry_t s_tab[GPIO_DISPATCH_MAX_PINS];
static volatile uint32_t s_count;
static volatile uint32_t s_calls;
static int32_t s_def_status;
static bool s_started;

/* The one handler. Runs to completion, so two callbacks never overlap: if two
 * pins fire together, one run serves both; an edge that arrives during the run
 * re-pends the interrupt and the handler runs again. */
static void gd_handler(UINT intno)
{
    uint32_t ints[4];
    uint32_t i;

    (void)intno;
    s_calls++;
    for (i = 0U; i < 4U; i++)
    {
        ints[i] = GD_INTS(i);
    }

    for (i = 0U; i < s_count; i++)
    {
        const uint32_t pin = s_tab[i].pin;
        const uint32_t reg = pin >> 3;                 /* 8 pins per register */
        const uint32_t shift = 4U * (pin & 7U);        /* 4 event bits per pin */
        const uint32_t nibble = (ints[reg] >> shift) & 0xFU;

        if (nibble != 0U)
        {
            if (s_tab[i].events != 0U)
            {
                GD_INTR(reg) = (nibble & 0xCU) << shift;   /* clear the latched edge(s) first */
            }
            s_tab[i].cb(pin);
        }
    }
}

bool gpio_dispatch_register(uint32_t pin, uint32_t events, gpio_dispatch_cb_t cb)
{
    uint32_t i;
    const uint32_t n = s_count;
    const uint32_t reg = pin >> 3;
    const uint32_t shift = 4U * (pin & 7U);

    if ((pin > 29U) || (cb == 0) || (n >= GPIO_DISPATCH_MAX_PINS))
    {
        return false;
    }
    for (i = 0U; i < n; i++)
    {
        if (s_tab[i].pin == pin)
        {
            return false;                               /* already registered */
        }
    }

    s_tab[n].pin = pin;
    s_tab[n].events = events & 0xCU;
    s_tab[n].cb = cb;
    if (s_tab[n].events != 0U)
    {
        GD_INTR(reg) = s_tab[n].events << shift;        /* drop stale events */
        GD_INTE(reg) |= s_tab[n].events << shift;
    }
    s_count = n + 1U;                                   /* publish the entry last */
    return true;
}

bool gpio_dispatch_start(void)
{
    T_DINT dint = {0};

    if (s_started)
    {
        return true;
    }
    if (GD_SIO_CPUID != 0U)
    {
        s_def_status = GPIO_DISPATCH_ERR_WRONG_CORE;    /* NVIC enable would land on core 1 */
        return false;
    }

    dint.intatr = TA_HLNG;
    dint.inthdr = gd_handler;
    s_def_status = (int32_t)tk_def_int(GPIO_DISPATCH_IRQ_NUMBER, &dint);
    EnableInt(GPIO_DISPATCH_IRQ_NUMBER, GPIO_DISPATCH_IRQ_LEVEL);   /* returns nothing in this kernel */
    s_started = (s_def_status >= 0);
    return s_started;
}

void gpio_dispatch_status(int32_t *def_status, uint32_t *inte0, uint32_t *handler_calls)
{
    *def_status = s_def_status;
    *inte0 = GD_INTE(0U);
    *handler_calls = s_calls;
}
