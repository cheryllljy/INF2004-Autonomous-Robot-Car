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

/* The TIMER block (TIMERAWL, used for every barcode timestamp) counts one
 * tick per watchdog tick. The kernel never sets the watchdog tick up (the
 * Pico SDK runtime normally does), so after some resets TIMERAWL stands
 * still or runs at the wrong rate. b3_port_init() starts it at 1 MHz:
 * clk_ref is the 12 MHz XOSC (XOSC_MHz, divider 1 in cpu_clock.c). */
#define B3_WATCHDOG_TICK            0x4005802CU
#define B3_WATCHDOG_TICK_ENABLE     (1U << 9)
#define B3_WATCHDOG_TICK_CYCLES     ((UW)XOSC_MHz)

/* Analog barcode input (B3_BARCODE_ANALOG): one TIMER alarm paces the
 * IR_B ADC reads. ADC registers as in device/adc/sysdepend/rp2040. */
#define B3_TIMER_ALARM(n)           (0x40054010U + ((n) * 4U))
#define B3_TIMER_ARMED              0x40054020U
#define B3_TIMER_INTR               0x40054034U
#define B3_TIMER_INTE               0x40054038U
#define B3_SAMPLER_MASK             (1U << B3_ANALOG_TIMER_ALARM)
#define B3_SAMPLER_MIN_LEAD_US      20U
#define B3_ADC_CS                   0x4004C000U
#define B3_ADC_RESULT               0x4004C004U
#define B3_ADC_FCS                  0x4004C008U
#define B3_ADC_FIFO                 0x4004C00CU
#define B3_ADC_INTE                 0x4004C018U
#define B3_ADC_CS_EN                (1U << 0)
#define B3_ADC_CS_START_ONCE        (1U << 2)
#define B3_ADC_CS_READY             (1U << 8)
#define B3_ADC_CS_AINSEL_POS        12U
#define B3_ADC_FCS_EMPTY            (1U << 8)
#define B3_ADC_READY_SPINS          1000U   /* a conversion is 2 us; bound the wait */
#define B3_IO_GPIO_CTRL(n)          (IO_BANK0_BASE + 0x04U + ((n) * 8U))
#define B3_IO_FUNCSEL_NULL          31U

#if TK_SUPPORT_SMP
static T_SPLOCK g_b3_lock;
#endif
static bool g_port_ready;
#if B3_BARCODE_ANALOG
static volatile bool g_sampler_on;
static uint32_t g_next_sample_us;
#endif

#if !B3_BARCODE_ANALOG
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
#endif

ER b3_port_init(void)
{
    if (g_port_ready)
    {
        return E_OK;
    }

    /* 1 MHz timebase for b3_port_time_us(): take TIMER out of reset and
     * run the watchdog tick at clk_ref / 12. Harmless if already done. */
    if (0U != (in_w(RESETS_RESET) & RESETS_RESET_TIMER))
    {
        clr_w(RESETS_RESET, RESETS_RESET_TIMER);
        while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
        {
        }
    }
    out_w(B3_WATCHDOG_TICK, B3_WATCHDOG_TICK_ENABLE | B3_WATCHDOG_TICK_CYCLES);
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

ER b3_port_gpio_output_init(unsigned int pin, bool initial_high)
{
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
#if B3_BARCODE_ANALOG
    if (enabled)
    {
        g_next_sample_us = b3_port_time_us() + B3_ANALOG_SAMPLE_US;
        g_sampler_on = true;
        out_w(B3_TIMER_INTR, B3_SAMPLER_MASK);
        set_w(B3_TIMER_INTE, B3_SAMPLER_MASK);
        out_w(B3_TIMER_ALARM(B3_ANALOG_TIMER_ALARM), g_next_sample_us);
    }
    else
    {
        g_sampler_on = false;
        clr_w(B3_TIMER_INTE, B3_SAMPLER_MASK);
        out_w(B3_TIMER_ARMED, B3_SAMPLER_MASK);     /* write 1 to disarm */
        out_w(B3_TIMER_INTR, B3_SAMPLER_MASK);
    }
#else
    UW reg = IO_BANK0_BASE + B3_IO_PROC0_INTE0_OFFSET +
             (irq_reg_index(B3_IR_B_GPIO) * sizeof(UW));
    UW mask = (UW)irq_hw_mask(B3_IR_B_GPIO, B3_GPIO_EDGE_BOTH);
    if (enabled)
    {
        set_w(reg, mask);
    }
    else
    {
        clr_w(reg, mask);
    }
#endif
}

uint32_t b3_port_barcode_irq_pending(void)
{
#if B3_BARCODE_ANALOG
    /* Samples are queued under the lock; nothing is ever left pending. */
    return 0U;
#else
    UW reg = IO_BANK0_BASE + B3_IO_PROC0_INTS0_OFFSET +
             (irq_reg_index(B3_IR_B_GPIO) * sizeof(UW));
    UW raw = in_w(reg);
    return (uint32_t)((raw >> irq_shift(B3_IR_B_GPIO)) & B3_GPIO_EDGE_BOTH);
#endif
}

void b3_port_barcode_irq_ack(uint32_t events)
{
#if B3_BARCODE_ANALOG
    (void)events;
#else
    UW reg = IO_BANK0_BASE + B3_IO_INTR0_OFFSET +
             (irq_reg_index(B3_IR_B_GPIO) * sizeof(UW));
    /* INTR edge bits are write-one-to-clear. */
    out_w(reg, (UW)irq_hw_mask(B3_IR_B_GPIO, events & B3_GPIO_EDGE_BOTH));
#endif
}

void b3_port_dispatch_barcode_irq(void)
{
    /* Always 0 in analog mode, so this does nothing there. */
    uint32_t events = b3_port_barcode_irq_pending();
    if (0U != events)
    {
        /* Capture/clear the hardware flags first; the delegate stays short and
         * records only timestamp/level/optional odometry into its queue. */
        b3_port_barcode_irq_ack(events);
        b3_ir_gpio_irq(B3_IR_B_GPIO, events);
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

ER b3_port_adc_input_init(unsigned int pin)
{
    UW spins;

    /* The kernel ADC driver normally releases the reset; do it here too
     * in case it is configured out. clk_adc is set up by cpu_clock.c. */
    if (0U != (in_w(RESETS_RESET) & RESETS_RESET_ADC))
    {
        clr_w(RESETS_RESET, RESETS_RESET_ADC);
        while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC))
        {
        }
    }
    /* Analog pad: no digital function, input buffer and pulls off. */
    out_w(B3_IO_GPIO_CTRL(pin), B3_IO_FUNCSEL_NULL);
    clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
    set_w(B3_ADC_CS, B3_ADC_CS_EN);
    for (spins = 0U; spins < B3_ADC_READY_SPINS; ++spins)
    {
        if (0U != (in_w(B3_ADC_CS) & B3_ADC_CS_READY))
        {
            return E_OK;
        }
    }
    return E_IO;
}

int32_t b3_port_adc_read(unsigned int channel)
{
    b3_irq_state_t saved;
    int32_t value = -1;
    UW spins;

    b3_port_lock(&saved);
    /* The "adca" driver enables the ADC interrupt only while its own
     * conversion is in flight. Skip this read rather than disturb it. */
    if (0U == in_w(B3_ADC_INTE))
    {
        for (spins = 0U; (spins < B3_ADC_READY_SPINS) &&
                         (0U == (in_w(B3_ADC_CS) & B3_ADC_CS_READY)); ++spins)
        {
        }
        out_w(B3_ADC_CS, ((UW)channel << B3_ADC_CS_AINSEL_POS) | B3_ADC_CS_EN);
        set_w(B3_ADC_CS, B3_ADC_CS_START_ONCE);
        for (spins = 0U; spins < B3_ADC_READY_SPINS; ++spins)
        {
            if (0U != (in_w(B3_ADC_CS) & B3_ADC_CS_READY))
            {
                value = (int32_t)(in_w(B3_ADC_RESULT) & 0x0FFFU);
                break;
            }
        }
        /* The driver leaves the FIFO on; drop this result from it so the
         * driver's next read does not pick it up. */
        while (0U == (in_w(B3_ADC_FCS) & B3_ADC_FCS_EMPTY))
        {
            (void)in_w(B3_ADC_FIFO);
        }
    }
    b3_port_unlock(saved);
    return value;
}

#if B3_BARCODE_ANALOG
static void b3_sampler_handler(UINT intno)
{
    uint32_t now;
    int32_t raw;

    out_w(B3_TIMER_INTR, B3_SAMPLER_MASK);
    ClearInt(intno);
    if (!g_sampler_on)
    {
        return;
    }
    /* Re-arm first, on a fixed grid. If an interrupt ran late, restart the
     * grid from now: an alarm set in the past would not fire for 71 min. */
    now = b3_port_time_us();
    g_next_sample_us += B3_ANALOG_SAMPLE_US;
    if ((int32_t)(g_next_sample_us - now) < (int32_t)B3_SAMPLER_MIN_LEAD_US)
    {
        g_next_sample_us = now + B3_ANALOG_SAMPLE_US;
    }
    out_w(B3_TIMER_ALARM(B3_ANALOG_TIMER_ALARM), g_next_sample_us);

    raw = b3_port_adc_read(B3_IR_B_ADC_CH);
    if (raw >= 0)
    {
        b3_ir_analog_sample(now, (uint32_t)raw);
    }
    /* TA_HLNG handlers return normally. Do NOT call tk_ret_int(). */
}
#endif

ER b3_port_install_barcode_sampler(void)
{
#if B3_BARCODE_ANALOG
    T_DINT dint = {0};
    ER status;

#if TK_SUPPORT_SMP
    if (1 != tk_get_prc())
    {
        return E_CTX;
    }
#endif
    b3_port_barcode_irq_enable(false);
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)b3_sampler_handler;
    status = tk_def_int(B3_ANALOG_TIMER_ALARM, &dint);
    if (E_OK != status)
    {
        return status;
    }
    ClearInt(B3_ANALOG_TIMER_ALARM);
    EnableInt(B3_ANALOG_TIMER_ALARM, B3_ANALOG_IRQ_PRIORITY);
    return E_OK;
#else
    return E_NOSPT;
#endif
}
