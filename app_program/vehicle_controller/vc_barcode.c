/*
 *      vc_barcode.c
 *      VehicleController -- back up and read a barcode slowly
 *. See vc_barcode.h.
 */

#include "vc_barcode.h"

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

static void vb_start_slow(const motion_status_t *p_motion, const char *why)
{
    vb.from_mm = p_motion->distance_total_mm;
    tm_printf((UB *)"[VC] barcode: %s, reading it at %d mm/s\n", why,
              (INT)B3_SCAN_SPEED_MM_S);
    vb_goto(VB_SLOW);
}

bool vc_bar_step(const motion_status_t *p_motion, const b3_snapshot_t *p_line,
                 bool decoded, bool rejected, int32_t dt_ms)
{
    const int32_t at = p_motion->distance_total_mm;

    vb.phase_ms += dt_ms;

    switch (vb.phase)
    {
        case VB_ARMED:
            if (vb_enabled() && p_line->scan_active &&
                (p_line->scan_elements >= VC_BAR_DETECT_ELEMENTS) &&
                !(p_line->line.left_black && p_line->line.right_black))
            {
                vb.detect_mm = at;
                (void)motion_stop();
                tm_printf(
                    (UB *)"[VC] barcode ahead at %d mm/s: stopping "
                          "to back up\n",
                    (INT)((p_motion->speed_l_mmps + p_motion->speed_r_mmps) /
                          2));
                vb_goto(VB_STOPPING);
                return true;
            }
            return false;

        case VB_STOPPING:
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
            if (decoded)
            {
                tm_printf((UB *)"[VC] barcode read, back to %d mm/s\n",
                          (INT)B3_BASE_SPEED_MM_S);
                vb_goto(VB_ARMED);
            }
            else if (rejected || (vb_abs(at - vb.from_mm) >=
                                  (vb.back_mm + VC_BAR_SLOW_MAX_MM)))
            {
                tm_printf((UB *)"[VC] barcode not read at %d mm/s (%s), back "
                                "to %d mm/s\n",
                          (INT)B3_SCAN_SPEED_MM_S,
                          rejected ? "rejected" : "none found",
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
    return !vc_bar_busy();
}

bool vc_bar_busy(void)
{
    return (VB_STOPPING == vb.phase) || (VB_BACKING == vb.phase);
}
