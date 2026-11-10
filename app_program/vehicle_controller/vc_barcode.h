/*
 *      vc_barcode.h
 *      VehicleController -- back up and read a barcode slowly
 *.
 *
 *      The car line-follows at B3_BASE_SPEED_MM_S (100 mm/s). When IR_B
 *      starts reading a barcode (VC_BAR_DETECT_ELEMENTS elements: a bar, a
 *      gap and the next bar, so a crossing line, which is one black element,
 *      does not count), the VehicleController:
 *
 *        1. stops (Buddy 2 brakes),
 *        2. backs up straight past where the barcode started, plus
 *           VC_BAR_BACK_MM of white for the leading quiet zone,
 *        3. line-follows over the barcode at B3_SCAN_SPEED_MM_S (30 mm/s),
 *        4. goes back to B3_BASE_SPEED_MM_S once the letter is decoded.
 *
 *      Scanning is off while stopping and backing up, so the decoder never
 *      sees the bars backwards. If the slow pass fails (a reject, or no
 *      barcode within VC_BAR_SLOW_MAX_MM) the car speeds up again and does
 *      not back up for the next VC_BAR_REARM_MM, so a barcode it cannot read
 *      is not tried over and over.
 *
 *      Called only from vc_task, so nothing in here needs a lock.
 */

#ifndef VC_BARCODE_H
#define VC_BARCODE_H

#include "../app.h"
#include "../subsystem2/motion_control.h"
#include "../subsystem3/buddy3.h"

#define VC_BAR_DETECT_ELEMENTS 2   /* bar + gap measured: a barcode */
#define VC_BAR_BACK_MM         100 /* white past its start; was 100 */
#define VC_BAR_BACK_SPEED_MM_S 100
#define VC_BAR_SLOW_MAX_MM     400  /* slow pass gives up after this */
#define VC_BAR_REARM_MM        250  /* after a failed pass: no back-up */
#define VC_BAR_CMD_TIMEOUT_MS  1000 /* Buddy 2 never took the command */
#define VC_BAR_STEP_TIMEOUT_MS 8000 /* stop or back-up took too long */

/* Line following starts again (START, after a turn or a bypass). */
void vc_bar_reset(const motion_status_t *p_motion);

/* One LINE_FOLLOW pass. decoded: a letter came from Buddy 3 this pass;
 * rejected: a scan failed this pass. Returns true while it drives the
 * motors itself (stopping or backing up): then the VehicleController must
 * not send Buddy 3's wheel speeds. */
bool vc_bar_step(const motion_status_t *p_motion, const b3_snapshot_t *p_line,
                 bool decoded, bool rejected, int32_t dt_ms);

/* For Buddy 3's context. */
int16_t vc_bar_speed_mm_s(void); /* line speed now */
bool vc_bar_scan_allowed(void);  /* false while stopping / backing up */
bool vc_bar_busy(void);          /* stopping / backing up */

#endif /* VC_BARCODE_H */
