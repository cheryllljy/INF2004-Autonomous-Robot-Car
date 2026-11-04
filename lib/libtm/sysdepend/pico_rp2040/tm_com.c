/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.0 BSP
 *
 *    Copyright (C) 2021-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

/*
 *    tm_com.c
 *    T-Monitor Communication low-level device driver (RaspberryPi Pico / Pico W)
 *
 *    Two sinks, because they are good at different things:
 *
 *      UART0   available from libtm_init() onward, needs no host, no
 *              enumeration and no scheduler.  This is the only channel that
 *              works during early boot, inside handlers, and after a fault,
 *              so it stays compiled in as the early/panic mirror.
 *
 *      USB CDC the console you actually read.  Needs the scheduler (its
 *              service task drains the ring) and a host that has enumerated
 *              the device, so it says nothing about early boot.
 *
 *    Output goes to both when both are enabled.  Neither waits indefinitely
 *    on external I/O: the UART wait is bounded and the USB path only appends
 *    under a short SMP ring lock.  This matters -- the original driver spun
 *    forever on a UART flag, which turns any console problem into an
 *    unexplained hang.
 */

#include <tk/typedef.h>
#include <sys/sysdef.h>

#if USE_TMONITOR
#include "../../libtm.h"

#ifdef PICO_RP2040

/* UART register definition (Use UART0) */
#define UART_BASE              0x40034000

#define UART_DR		(*(_UW*)(UART_BASE+0x000))
#define UART_FR		(*(_UW*)(UART_BASE+0x018))
#define UART_IBRD	(*(_UW*)(UART_BASE+0x024))
#define UART_FBRD	(*(_UW*)(UART_BASE+0x028))
#define UART_LCR_H	(*(_UW*)(UART_BASE+0x02C))
#define UART_CR		(*(_UW*)(UART_BASE+0x030))

#define	FR_TXFF		(1<<5)	// Transmit FIFO full
#define	FR_RXFE		(1<<4)	// Receive FIFO empty

/*
 * Bound on the wait for FIFO space, in loop iterations.  Generous for a
 * 115200 baud character (~87 us) yet still finite, so a UART that is
 * unclocked or held in reset costs a bounded delay and a dropped byte
 * instead of hanging the system.
 */
#define UART_TX_SPIN_LIMIT	200000

#if TM_CONSOLE_UART
LOCAL void uart_snd_dat(const UB* buf, INT size)
{
	UB	*b;
	UW	spin;

	for( b = (UB *)buf; size > 0; size--, b++ ){
		spin = UART_TX_SPIN_LIMIT;
		while( (UART_FR & FR_TXFF) != 0 ){
			if( --spin == 0 ) return;	/* give up, drop the rest */
		}
		UART_DR = *b;
	}
}
#endif /* TM_CONSOLE_UART */

#if TM_CONSOLE_USB_CDC
IMPORT void tm_usb_console_put(const UB *buf, INT size);
#endif

/*
 * Test lines over MQTT (added 2026-11-04 by Buddy 3, for testing with the
 * car unplugged).  In a test build with the MQTT client (make TEST=1
 * MQTT=1 ...), every console line that starts with "[TEST]" is also kept
 * in a ring here.  mqtt_utk.c (network task) takes them out with
 * tm_mqtt_log_peek()/tm_mqtt_log_drop() and publishes each line on
 * inf2004/robot/test.  This side never waits and never calls lwIP, so it
 * is safe from any task, handler or core.  A full ring drops the line and
 * counts it in tm_mqtt_log_dropped.
 */
#if defined(APP_MQTT) && APP_MQTT && defined(APP_TEST) && APP_TEST
#include <tk/tkernel.h>
#include <tk/smp.h>

#define MLOG_PREFIX	"[TEST]"
#define MLOG_PREFIX_LEN	6
#define MLOG_LINE_MAX	160		/* longer lines are cut */
#define MLOG_RING_SIZE	16384U		/* power of two; records = length byte + text (~240 lines) */
#define MLOG_RING_MASK	(MLOG_RING_SIZE - 1U)

LOCAL UB	mlog_line[MLOG_LINE_MAX];
LOCAL UW	mlog_len;
LOCAL UB	mlog_ring[MLOG_RING_SIZE];
LOCAL UW	mlog_head, mlog_tail;
#if TK_SUPPORT_SMP
LOCAL T_SPLOCK	mlog_lock;		/* 0 = unlocked, so no init needed */
#endif
EXPORT volatile UW	tm_mqtt_log_dropped;

LOCAL void mlog_lock_enter(UINT *intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinLock(&mlog_lock, intsts);
#else
	DI(*intsts);
#endif
}

LOCAL void mlog_lock_leave(UINT intsts)
{
#if TK_SUPPORT_SMP
	(void)ISpinUnlock(&mlog_lock, intsts);
#else
	EI(intsts);
#endif
}

/* Lock held.  Keep the finished line if it is a test line. */
LOCAL void mlog_end_line(void)
{
	UW	i, used;

	for (i = 0; i < MLOG_PREFIX_LEN; i++) {
		if (i >= mlog_len || mlog_line[i] != (UB)MLOG_PREFIX[i]) {
			return;
		}
	}
	used = (mlog_head - mlog_tail) & MLOG_RING_MASK;
	if (used + mlog_len + 1U > MLOG_RING_MASK) {
		tm_mqtt_log_dropped++;
		return;
	}
	mlog_ring[mlog_head] = (UB)mlog_len;
	mlog_head = (mlog_head + 1U) & MLOG_RING_MASK;
	for (i = 0; i < mlog_len; i++) {
		mlog_ring[mlog_head] = mlog_line[i];
		mlog_head = (mlog_head + 1U) & MLOG_RING_MASK;
	}
}

LOCAL void mlog_put(const UB *buf, INT size)
{
	UINT	intsts;

	mlog_lock_enter(&intsts);
	for ( ; size > 0; size--, buf++) {
		if (*buf == '\n') {
			mlog_end_line();
			mlog_len = 0;
		} else if (*buf != '\r' && mlog_len < MLOG_LINE_MAX) {
			mlog_line[mlog_len++] = *buf;
		}
	}
	mlog_lock_leave(intsts);
}

/* Network task: copy the oldest line (no newline) without removing it.
   Returns its length, or 0 when there is none. */
EXPORT INT tm_mqtt_log_peek(UB *buf, INT max)
{
	UINT	intsts;
	UW	len, i, at;

	mlog_lock_enter(&intsts);
	if (mlog_head == mlog_tail) {
		mlog_lock_leave(intsts);
		return 0;
	}
	len = mlog_ring[mlog_tail];
	at = (mlog_tail + 1U) & MLOG_RING_MASK;
	for (i = 0; i < len && i < (UW)max; i++) {
		buf[i] = mlog_ring[at];
		at = (at + 1U) & MLOG_RING_MASK;
	}
	mlog_lock_leave(intsts);
	return (INT)i;
}

/* Network task: remove the line tm_mqtt_log_peek() returned. */
EXPORT void tm_mqtt_log_drop(void)
{
	UINT	intsts;

	mlog_lock_enter(&intsts);
	if (mlog_head != mlog_tail) {
		mlog_tail = (mlog_tail + 1U + mlog_ring[mlog_tail]) & MLOG_RING_MASK;
	}
	mlog_lock_leave(intsts);
}
#endif /* APP_MQTT && APP_TEST */

EXPORT	void	tm_snd_dat( const UB* buf, INT size )
{
#if defined(APP_MQTT) && APP_MQTT && defined(APP_TEST) && APP_TEST
	mlog_put(buf, size);
#endif
#if TM_CONSOLE_USB_CDC
	tm_usb_console_put(buf, size);
#endif
#if TM_CONSOLE_UART
	uart_snd_dat(buf, size);
#endif
}

EXPORT	void	tm_rcv_dat( UB* buf, INT size )
{
#if TM_CONSOLE_UART
	for( ; size > 0; size--, buf++ ){
		while ( (UART_FR & FR_RXFE) != 0 );
		*buf = UART_DR & 0xff;
	}
#else
	/* Console input is not implemented on the USB-only configuration. */
	for( ; size > 0; size--, buf++ ){
		*buf = 0;
	}
#endif
}

EXPORT	void	tm_com_init(void)
{
#if TM_CONSOLE_UART
	UART_IBRD	= 67;			/* Baud rate setting */
	UART_FBRD	= 52;
	UART_LCR_H	= 0x70;			/* Communication data format setting */
	UART_CR 	= (1<<9)|(1<<8)|(1<<0);	/* Communication enabled */
#endif
	/* The USB side needs the scheduler, so it is started later from
	   knl_start_device(); nothing to do here. */
}

#endif /* PICO_RP2040 */
#endif /* USE_TMONITOR */
