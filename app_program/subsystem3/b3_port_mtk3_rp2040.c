/* Hardware/RTOS adapter for https://github.com/sirfonzie/mtk3smp-rp2040
 *
 * This file intentionally uses the micro T-Kernel BSP and register definitions
 * already present in that port. It does NOT require the Pico SDK runtime.
 */
#include "b3_port.h"
#include "b3_ir.h"
#include "b3_config.h"
#include <bsp/libbsp.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#define B3_IO_IRQ_BANK0             13U
#define B3_IO_IRQ_PRIORITY          1
#define B3_IO_INTR0_OFFSET          0x000000F0U
#define B3_IO_PROC0_INTE0_OFFSET    0x00000100U
#define B3_IO_PROC0_INTS0_OFFSET    0x00000120U
#define B3_GPIO_IRQ_BITS_PER_PIN    4U
#define B3_GPIO_IRQ_PINS_PER_REG    8U

#if TK_SUPPORT_SMP
static T_SPLOCK g_b3_lock;
#endif
static bool g_port_ready;

static uint32_t irq_reg_index(unsigned int pin)
{
    return pin / B3_GPIO_IRQ_PINS_PER_REG;
}

static uint32_t irq_shift(unsigned int pin)
{
    return (pin % B3_GPIO_IRQ_PINS_PER_REG) * B3_GPIO_IRQ_BITS_PER_PIN;
}

static uint32_t irq_hw_mask(unsigned int pin, uint32_t events)
{
    return (events & 0xFU) << irq_shift(pin);
}

ER b3_port_init(void)
{
    if (g_port_ready)
    {
        return E_OK;
    }
#if TK_SUPPORT_SMP
    /* The RP2040 port owns peripheral IRQs on processor 1 (core 0). Start the
     * subsystem there. The worker task is also pinned there below. */
    if (1 != tk_get_prc())
    {
        return E_CTX;
    }
    {
        ER status = InitSpinLock(&g_b3_lock);
        if (E_OK != status)
        {
            return status;
        }
    }
#endif
    g_port_ready = true;
    return E_OK;
}

void b3_port_lock(b3_irq_state_t *p_state)
{
#if TK_SUPPORT_SMP
    UINT intsts = 0U;
    (void)ISpinLock(&g_b3_lock, &intsts);
    *p_state = (uint32_t)intsts;
#else
    UW intsts;
    DI(intsts);
    *p_state = (uint32_t)intsts;
#endif
}

void b3_port_unlock(b3_irq_state_t state)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_b3_lock, (UINT)state);
#else
    EI((UW)state);
#endif
}

ER b3_port_gpio_input_init(unsigned int pin, bool pull_up)
{
    ER status = gpio_set_pin((UINT)pin, GPIO_MODE_IN);
    UW pad;
    if (E_OK != status)
    {
        return status;
    }
    pad = in_w(GPIO(pin));
    pad |= GPIO_IE | GPIO_SHEMITT;
    pad &= ~GPIO_PDE;
    if (pull_up)
    {
        pad |= GPIO_PUE;
    }
    else
    {
        pad &= ~GPIO_PUE;
    }
    out_w(GPIO(pin), pad);
    return E_OK;
}

/* Buddy3 source diagnostics use these pin numbers even when they are not
 * declared in the config file. In team mode with debug off, leave those
 * outputs alone so other students can use the pins safely. */
static bool b3_is_diagnostic_pin(unsigned int pin)
{
    return pin == 2U || pin == 3U || pin == 5U ||
           pin == 6U || pin == 16U || pin == 17U;
}

ER b3_port_gpio_output_init(unsigned int pin, bool initial_high)
{
#if !B3_DEBUG_ENABLED
    if (b3_is_diagnostic_pin(pin)) {
        return E_OK; /* do not configure teammates' GPIOs */
    }
#endif
    ER status = gpio_set_pin((UINT)pin, GPIO_MODE_OUT);
    if (E_OK != status)
    {
        return status;
    }
    return gpio_set_val((UINT)pin, initial_high ? 1U : 0U);
}

uint32_t b3_port_gpio_get_all(void)
{
    return (uint32_t)in_w(GPIO_IN);
}

void b3_port_gpio_put(unsigned int pin, bool value)
{
#if !B3_DEBUG_ENABLED
    if (b3_is_diagnostic_pin(pin)) {
        return; /* ignore Buddy 3's temporary LED probes */
    }
#endif
    (void)gpio_set_val((UINT)pin, value ? 1U : 0U);
}

uint32_t b3_port_time_us(void)
{
    /* RP2040 TIMER_TIMERAWL is the free-running lower 32-bit microsecond
     * counter. Unsigned subtraction remains correct across wraparound. */
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

void b3_port_barcode_irq_enable(bool enabled)
{
    UW reg = IO_BANK0_BASE + B3_IO_PROC0_INTE0_OFFSET +
             (irq_reg_index(B3_BARCODE_GPIO) * sizeof(UW));
    UW mask = (UW)irq_hw_mask(B3_BARCODE_GPIO, B3_GPIO_EDGE_BOTH);
    if (enabled)
    {
        set_w(reg, mask);
    }
    else
    {
        clr_w(reg, mask);
    }
}

uint32_t b3_port_barcode_irq_pending(void)
{
    UW reg = IO_BANK0_BASE + B3_IO_PROC0_INTS0_OFFSET +
             (irq_reg_index(B3_BARCODE_GPIO) * sizeof(UW));
    UW raw = in_w(reg);
    return (uint32_t)((raw >> irq_shift(B3_BARCODE_GPIO)) & B3_GPIO_EDGE_BOTH);
}

void b3_port_barcode_irq_ack(uint32_t events)
{
    UW reg = IO_BANK0_BASE + B3_IO_INTR0_OFFSET +
             (irq_reg_index(B3_BARCODE_GPIO) * sizeof(UW));
    /* INTR edge bits are write-one-to-clear. */
    out_w(reg, (UW)irq_hw_mask(B3_BARCODE_GPIO, events & B3_GPIO_EDGE_BOTH));
}

void b3_port_dispatch_barcode_irq(void)
{
    uint32_t events = b3_port_barcode_irq_pending();
    if (0U != events)
    {
        /* Capture/clear the hardware flags first; the delegate stays short and
         * records only timestamp/level/optional odometry into its queue. */
        b3_port_barcode_irq_ack(events);
        b3_ir_gpio_irq(B3_BARCODE_GPIO, events);
    }
}

static void b3_io_bank0_handler(UINT intno)
{
    (void)intno;
    b3_port_dispatch_barcode_irq();
    /* TA_HLNG handlers return normally. Do NOT call tk_ret_int(). */
}

ER b3_port_install_barcode_irq(void)
{
    T_DINT dint = {0};
    ER status;

#if TK_SUPPORT_SMP
    if (1 != tk_get_prc())
    {
        return E_CTX;
    }
#endif
    b3_port_barcode_irq_enable(false);
    b3_port_barcode_irq_ack(B3_GPIO_EDGE_BOTH);
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)b3_io_bank0_handler;
    status = tk_def_int(B3_IO_IRQ_BANK0, &dint);
    if (E_OK != status)
    {
        return status;
    }
    EnableInt(B3_IO_IRQ_BANK0, B3_IO_IRQ_PRIORITY);
    return E_OK;
}
