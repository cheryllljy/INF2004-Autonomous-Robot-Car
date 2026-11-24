/*
 *      vc_barcode.c
 *      VehicleController -- back up and read a barcode slowly.
 *      See vc_barcode.h.
 */

#include "vc_barcode.h"
#include "../subsystem4/subsystem4.h" /* hump and tilt */

/* On the hump (the IMU reports it, or the car is on the ramp or just
 * past it: vc_on_ramp()) Buddy 3's barcode scanning is off. IR_B is
 * 125 mm ahead of the wheels, so it reaches the ramp before the IMU can
 * feel it: after the first black the car drives on VC_BAR_WAIT_MM before
 * stopping, and a hump in that distance means it was not a barcode.
 * Tilt is not used: it read 5 to 10 deg at barcodes on flat ground
 * (braking and shaking), the same as on the ramp. */

typedef enum
{
    VB_ARMED = 0, /* fast; a barcode start triggers the back-up */
    VB_STOPPING,  /* braking */
    VB_BACKING,   /* reversing past the barcode's start */
    VB_SLOW,      /* reading it at B3_SCAN_SPEED_MM_S */
    VB_REARM      /* fast again after a failed pass, not armed yet */
} vb_phase_t;

typedef struct
{
    vb_phase_t phase;
    int32_t detect_mm; /* distance when the barcode was seen */
    int32_t back_mm;   /* how far to back up */
    int32_t from_mm;   /* start of the slow pass / of the re-arm */
    int32_t phase_ms;
    int32_t cmd_ms; /* since the move was sent */
    bool sent;      /* back-up move sent */
    bool busy_seen; /* Buddy 2 started it */
    bool waiting;   /* barcode seen, driving on VC_BAR_WAIT_MM first */
    bool scan_off;  /* on the hump: Buddy 3's scanning off */
    bool kept_slow; /* rejected before VC_BAR_SLOW_MIN_MM: still slow */
} vb_t;

static vb_t vb;

static int32_t vb_abs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Fast speed is not faster than the slow one: no back-up needed. */
static bool vb_enabled(void)
{
    return B3_BASE_SPEED_MM_S > B3_SCAN_SPEED_MM_S;
}

static void vb_goto(vb_phase_t next)
{
    vb.phase = next;
    vb.phase_ms = 0;
}

void vc_bar_reset(const motion_status_t *p_motion)
{
    vb = (vb_t){ 0 };
    vb.from_mm = p_motion->distance_total_mm;
    vb_goto(VB_ARMED);
}

/* the IMU reports the hump, or the car is on the ramp or just past it */
static bool vb_hump(void)
{
    return subsystem4_on_hump() || vc_on_ramp();
}

static void vb_stop_for_barcode(const motion_status_t *p_motion,
                                const char *why)
{
    vb.waiting = false;
    (void)motion_stop();
    tm_printf((UB *)"[VC] barcode %d mm back at %d mm/s, %s: stopping to "
                    "back up\n",
              (INT)vb_abs(p_motion->distance_total_mm - vb.detect_mm),
              (INT)((p_motion->speed_l_mmps + p_motion->speed_r_mmps) / 2),
              why);
    vb_goto(VB_STOPPING);
}

static void vb_start_slow(const motion_status_t *p_motion, const char *why)
{
    vb.from_mm = p_motion->distance_total_mm;
    vb.kept_slow = false;
    tm_printf((UB *)"[VC] barcode: %s, reading it at %d mm/s\n", why,
              (INT)B3_SCAN_SPEED_MM_S);
    vb_goto(VB_SLOW);
}

bool vc_bar_step(const motion_status_t *p_motion, const b3_snapshot_t *p_line,
                 bool decoded, bool rejected, int32_t dt_ms)
{
    const int32_t at = p_motion->distance_total_mm;

    const bool b_hump = vb_hump();

    vb.phase_ms += dt_ms;

    if (b_hump != vb.scan_off)
    {
        vb.scan_off = b_hump;
        tm_printf((UB *)"[VC] %s the hump: barcode scanning %s\n",
                  b_hump ? "on" : "off", b_hump ? "off" : "on again");
    }

    switch (vb.phase)
    {
        case VB_ARMED:
            if (vb.waiting)
            {
                if (b_hump)
                {
                    vb.waiting = false;
                    tm_printf((UB *)"[VC] the black under the barcode "
                                    "sensor %d mm back was the hump (side "
                                    "lift %d mm): not backing up\n",
                              (INT)vb_abs(at - vb.detect_mm),
                              (INT)subsystem4_side_lift_mm());
                }
                else if (decoded)
                {
                    vb.waiting = false;
                    tm_printf((UB *)"[VC] barcode read at %d mm/s: no "
                                    "back-up needed\n",
                              (INT)((p_motion->speed_l_mmps +
                                     p_motion->speed_r_mmps) /
                                    2));
                }
                else if (vb_abs(at - vb.detect_mm) >= VC_BAR_WAIT_MM)
                {
                    vb_stop_for_barcode(p_motion, "no hump");
                    return true;
                }
                else
                {
                    /* driving on */
                }
            }
            else if (vb_enabled() && p_line->scan_active && !b_hump &&
                     (p_line->scan_elements >= VC_BAR_DETECT_ELEMENTS) &&
                     !(p_line->line.left_black && p_line->line.right_black))
            {
                vb.detect_mm = at;
                vb.waiting = true;
                tm_printf((UB *)"[VC] barcode sensor black: driving %d mm "
                                "on in case it is the hump\n",
                          (INT)VC_BAR_WAIT_MM);
            }
            else
            {
                /* no barcode */
            }
            return false;

        case VB_STOPPING:
            /* No hump check while stopping and backing up: the car has
             * just driven VC_BAR_WAIT_MM over this ground with no hump,
             * and a lift spike while reversing would throw the read away.
             * The slow pass still checks. */
            if (MOTION_STATE_IDLE == p_motion->state)
            {
                /* back past where it was seen, which was a few mm into the
                 * barcode, plus the white for its leading quiet zone */
                vb.back_mm = vb_abs(at - vb.detect_mm) + VC_BAR_BACK_MM;
                vb.sent = false;
                vb.busy_seen = false;
                vb.cmd_ms = 0;
                tm_printf((UB *)"[VC] barcode: stopped %d mm past it, "
                                "backing up %d mm\n",
                          (INT)vb_abs(at - vb.detect_mm), (INT)vb.back_mm);
                vb_goto(VB_BACKING);
            }
            else if (MOTION_STATE_DRIVING == p_motion->state)
            {
                (void)motion_stop(); /* the first stop was refused */
            }
            else
            {
                /* STOPPING: wait */
            }
            if ((VB_STOPPING == vb.phase) &&
                (vb.phase_ms >= VC_BAR_STEP_TIMEOUT_MS))
            {
                vb_start_slow(p_motion, "stop took too long");
            }
            return true;

        case VB_BACKING:
            vb.cmd_ms += dt_ms;
            if (!vb.sent)
            {
                if (MOTION_OK ==
                    motion_move(-vb.back_mm, VC_BAR_BACK_SPEED_MM_S))
                {
                    vb.sent = true;
                    vb.cmd_ms = 0;
                }
            }
            else if (!vb.busy_seen)
            {
                if (MOTION_STATE_IDLE != p_motion->state)
                {
                    vb.busy_seen = true;
                }
                else if (vb.cmd_ms >= VC_BAR_CMD_TIMEOUT_MS)
                {
                    vb.sent = false; /* refused: send it again */
                }
                else
                {
                    /* not started yet */
                }
            }
            else if (MOTION_STATE_IDLE == p_motion->state)
            {
                vb_start_slow(p_motion, "backed up");
            }
            else
            {
                /* reversing */
            }
            if ((VB_BACKING == vb.phase) &&
                (vb.phase_ms >= VC_BAR_STEP_TIMEOUT_MS))
            {
                (void)motion_stop();
                vb_start_slow(p_motion, "back-up took too long");
            }
            return VB_BACKING == vb.phase;

        case VB_SLOW:
            if (b_hump)
            {
                tm_printf((UB *)"[VC] barcode: it was the hump, back to %d "
                                "mm/s\n",
                          (INT)B3_BASE_SPEED_MM_S);
                vb_goto(VB_ARMED);
            }
            else if (decoded)
            {
                tm_printf((UB *)"[VC] barcode read, back to %d mm/s\n",
                          (INT)B3_BASE_SPEED_MM_S);
                vb_goto(VB_ARMED);
            }
            else if (rejected &&
                     (vb_abs(at - vb.from_mm) < VC_BAR_SLOW_MIN_MM))
            {
                if (!vb.kept_slow)
                {
                    tm_printf((UB *)"[VC] barcode rejected %d mm into the "
                                    "slow pass: staying at %d mm/s to %d "
                                    "mm\n",
                              (INT)vb_abs(at - vb.from_mm),
                              (INT)B3_SCAN_SPEED_MM_S,
                              (INT)VC_BAR_SLOW_MIN_MM);
                }
                vb.kept_slow = true;
            }
            else if (rejected ||
                     (vb.kept_slow &&
                      (vb_abs(at - vb.from_mm) >= VC_BAR_SLOW_MIN_MM)) ||
                     (vb_abs(at - vb.from_mm) >=
                      (vb.back_mm + VC_BAR_SLOW_MAX_MM)))
            {
                tm_printf((UB *)"[VC] barcode not read at %d mm/s (%s), back "
                                "to %d mm/s\n",
                          (INT)B3_SCAN_SPEED_MM_S,
                          (rejected || vb.kept_slow) ? "rejected"
                                                     : "none found",
                          (INT)B3_BASE_SPEED_MM_S);
                vb.from_mm = at;
                vb_goto(VB_REARM);
            }
            else
            {
                /* reading */
            }
            return false;

        case VB_REARM:
            if (vb_abs(at - vb.from_mm) >= VC_BAR_REARM_MM)
            {
                vb_goto(VB_ARMED);
            }
            return false;

        default:
            vb_goto(VB_ARMED);
            return false;
    }
}

int16_t vc_bar_speed_mm_s(void)
{
    return ((VB_ARMED == vb.phase) || (VB_REARM == vb.phase))
               ? (int16_t)B3_BASE_SPEED_MM_S
               : (int16_t)B3_SCAN_SPEED_MM_S;
}

bool vc_bar_scan_allowed(void)
{
    return !vc_bar_busy() && !vb.scan_off;
}

bool vc_bar_waiting(void)
{
    return (VB_ARMED == vb.phase) && vb.waiting;
}

void vc_bar_back_now(const motion_status_t *p_motion)
{
    if (vc_bar_waiting())
    {
        vb_stop_for_barcode(p_motion, "junction reached first");
    }
}

bool vc_bar_busy(void)
{
    return (VB_STOPPING == vb.phase) || (VB_BACKING == vb.phase);
}
