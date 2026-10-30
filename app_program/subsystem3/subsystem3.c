/*
 *	subsystem3.c
 *	Subsystem 3 -- Consumer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mtx
 */

#include "subsystem3.h"
#include "buddy3.h"
#include "buddy3_team_config.h"
#include "b3_config.h"
#include "b3_port.h"
#include "gpio_dispatch.h"
#if B3_USE_B2_ODOMETRY
#include "../subsystem2/motion_hw.h"
#endif

EXPORT ID		mbx_samples;	/* IF-01 */
EXPORT ID		mtx_stats;	/* IF-04 */
EXPORT APP_STATS	app_stats;

#if !B3_INSTALL_IO_BANK0_HANDLER && !B3_BARCODE_ANALOG
/* GP28 barcode edges, called by the shared GPIO dispatcher (interrupt context).
   Buddy 3 enables and acknowledges its own edge events. */
static void s3_barcode_irq(uint32_t pin)
{
	(void)pin;
	b3_port_dispatch_barcode_irq();
}
#endif

#if B3_USE_B2_ODOMETRY
/* Buddy 2's encoder count as Buddy 3's position reader. Called from the
   barcode edge callback, so it must not block: it only reads two words that
   the encoder callback (same handler) writes. Average of both wheels. */
static bool s3_read_b2_ticks(uint32_t *p_ticks)
{
	*p_ticks = (motion_hw_ticks(MOTION_SIDE_LEFT) +
		    motion_hw_ticks(MOTION_SIDE_RIGHT)) / 2U;
	return true;
}
#define S3_POSITION_READER	s3_read_b2_ticks
#else
#define S3_POSITION_READER	NULL
#endif

EXPORT ER subsystem3_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	/* Mailbox: passes a POINTER to a message (zero copy).  The message
	   memory must stay valid until the receiver is done with it, which
	   is why subsystem 2 takes it from its memory pool. */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;	/* FIFO waiters, FIFO messages */
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	/* Mutex: only one task at a time may touch app_stats.  TA_INHERIT
	   (priority inheritance) avoids priority inversion: a low-priority
	   holder is temporarily raised to the waiter's priority. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	if (mtx_stats < E_OK) {
		return mtx_stats;
	}

#if !B3_INSTALL_IO_BANK0_HANDLER
	/* The team's one GPIO interrupt handler (gpio_dispatch.c, owned by
	   Buddy 3). Runs here in usermain on core 0. If Buddy 2 already
	   started it in subsystem2_init(), this start call just returns true.
	   With B3_BARCODE_ANALOG, GP28 is an ADC input read by Buddy 3's own
	   TIMER sampler, so only Buddy 2's encoder pins use the dispatcher. */
#if !B3_BARCODE_ANALOG
	if (!gpio_dispatch_register(B3_IR_B_GPIO, 0U, s3_barcode_irq)) {
		return E_SYS;
	}
#endif
	if (!gpio_dispatch_start()) {
		return E_SYS;
	}
#endif
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");

	/* Buddy 3 is opt-in until its pins, IRQ ownership and vehicle interface are approved.
	 * No existing consumer/task behaviour is replaced by this hook. */
	#if B3_TEAM_AUTOSTART
		if (tid >= E_OK) {
			ER b3_status = buddy3_start(S3_POSITION_READER);
			if (b3_status < E_OK) {
				return b3_status;
			}
		}
	#endif

	return (tid < E_OK) ? tid : E_OK;
}
