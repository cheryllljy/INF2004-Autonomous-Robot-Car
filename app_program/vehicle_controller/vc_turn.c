/*
 *      vc_turn.c
 *      VehicleController -- the turn at a junction (nyx, 2026-11-06).
 *      See vc_turn.h for what it does and why.
 */

#include "vc_turn.h"
#include "../subsystem2/motion_config.h" /* TRACK_WIDTH_MM */

/* pi x 1000 */
#define VT_PI_1000 3142

typedef enum
{
    VT_IDLE = 0,
    VT_ARC,     /* LEFT/RIGHT: on the arc */
    VT_SLOWING, /* U-turn: easing down to VC_SOFT_STOP_MM_S */
    VT_PIVOT    /* U-turn: pivoting left on the spot */
} vt_phase_t;

typedef struct
{
    b3_nav_command_t turn;
    vt_phase_t phase;
    int32_t side; /* +1 left, -1 right */
    int32_t l0;   /* wheel distances at the start, mm */
    int32_t r0;
    int32_t speed_ums; /* U-turn: forward speed being eased down, um/s */
    int32_t pivot_ums; /* U-turn: pivot wheel speed, um/s */
    bool seen_lead;    /* the leading IR has seen the new line */
    int16_t sent_l;
    int16_t sent_r;
    int32_t since_send_ms;
} vt_t;

static vt_t vt;

static int32_t track_mm(void)
{
    return (int32_t)(TRACK_WIDTH_MM + 0.5f);
}

static const char *turn_text(b3_nav_command_t turn)
{
    return (B3_NAV_LEFT == turn)    ? "LEFT"
           : (B3_NAV_RIGHT == turn) ? "RIGHT"
                                    : "U-TURN";
}

/* integer square root */
static int32_t isqrt(int64_t n)
{
    int64_t x = 0;
    int64_t bit = (int64_t)1 << 62;

    if (n <= 0)
    {
        return 0;
    }
    while (bit > n)
    {
        bit >>= 2;
    }
    while (0 != bit)
    {
        if (n >= (x + bit))
        {
            n -= x + bit;
            x = (x >> 1) + bit;
        }
        else
        {
            x >>= 1;
        }
        bit >>= 2;
    }
    return (int32_t)x;
}

/* Send wheel speeds when they change, and again every 50 ms while Buddy 2
 * is not driving them yet (a command sent while it was STOPPING is
 * refused). */
static void wheels(const motion_status_t *p_motion, int16_t left, int16_t right)
{
    if ((left != vt.sent_l) || (right != vt.sent_r) ||
        ((MOTION_STATE_DRIVING != p_motion->state) && (vt.since_send_ms >= 50)))
    {
        if (MOTION_OK == motion_set_wheel_speeds(left, right))
        {
            vt.sent_l = left;
            vt.sent_r = right;
            vt.since_send_ms = 0;
        }
    }
}

/* Heading turned since the start towards side (+1 left), milliradians. */
static int32_t turned_mrad(const motion_status_t *p_motion, int32_t side)
{
    const int32_t dl = p_motion->dist_l_mm - vt.l0;
    const int32_t dr = p_motion->dist_r_mm - vt.r0;

    return (side * (dr - dl) * 1000) / track_mm();
}

static int32_t mrad_to_deg(int32_t mrad)
{
    return (mrad * 180) / VT_PI_1000;
}

static int32_t deg_to_mrad(int32_t deg)
{
    return (deg * VT_PI_1000) / 180;
}

/* The new line is centred between the IRs once the leading IR has seen
 * it and is white again, or the trailing IR sees it. */
static bool line_centred(bool lead_black, bool trail_black)
{
    if (lead_black)
    {
        vt.seen_lead = true;
    }
    return trail_black || (vt.seen_lead && !lead_black);
}

bool vc_turn_start(b3_nav_command_t turn, const motion_status_t *p_motion)
{
    vt = (vt_t){ 0 };
    vt.turn = turn;
    vt.l0 = p_motion->dist_l_mm;
    vt.r0 = p_motion->dist_r_mm;
    vt.sent_l = INT16_MIN;
    vt.sent_r = INT16_MIN;

    switch (turn)
    {
        case B3_NAV_LEFT:
        case B3_NAV_RIGHT:
        {
            const int32_t v = VC_CORNER_SPEED_MM_S;
            const int32_t half = track_mm() / 2;

            vt.side = (B3_NAV_LEFT == turn) ? 1 : -1;
            vt.phase = VT_ARC;
#if VC_CORNER_PIVOT
            (void)v;
            (void)half;
            tm_printf((UB *)"[VC] junction: %s, pivot on the %s wheel, the "
                            "other at %d mm/s, until IR_%s crosses the line\n",
                      turn_text(turn), (vt.side > 0) ? "left" : "right",
                      (INT)VC_CORNER_PIVOT_MM_S, (vt.side > 0) ? "L" : "R");
#else
            tm_printf(
                (UB *)"[VC] junction: arc %s, radius %d mm at %d mm/s "
                      "(inner wheel %d, outer %d mm/s)\n",
                turn_text(turn), (INT)VC_CORNER_RADIUS_MM, (INT)v,
                (INT)((v * (VC_CORNER_RADIUS_MM - half)) / VC_CORNER_RADIUS_MM),
                (INT)((v * (VC_CORNER_RADIUS_MM + half)) /
                      VC_CORNER_RADIUS_MM));
#endif
            break;
        }
        case B3_NAV_UTURN:
        {
            int32_t now = ((int32_t)p_motion->speed_l_mmps +
                           (int32_t)p_motion->speed_r_mmps) /
                          2;

            if (now < 0)
            {
                now = 0;
            }
            /* nyx, 2026-11-07: was 2 * VC_CORNER_MAX_MM_S; kept at 160 so
             * the slower arc does not make the U-turn brake at once */
            if (now > VC_UTURN_EASE_MAX_MM_S)
            {
                now = VC_UTURN_EASE_MAX_MM_S;
            }
            vt.side = 1;
            vt.speed_ums = now * 1000;
            vt.phase = VT_SLOWING;
            tm_printf((UB *)"[VC] junction: U-turn, easing to a stop from %d "
                            "mm/s\n",
                      (INT)now);
            break;
        }
        case B3_NAV_STRAIGHT:
        case B3_NAV_NONE:
        default:
            vt.phase = VT_IDLE;
            return false;
    }

    return true;
}

/* LEFT / RIGHT: one step on the arc. */
static vc_turn_result_t arc_step(const motion_status_t *p_motion,
                                 bool inner_black, bool outer_black)
{
    const int32_t w = track_mm();
    const int32_t r = VC_CORNER_RADIUS_MM;
    const int32_t v = VC_CORNER_SPEED_MM_S;
    const int32_t dl = p_motion->dist_l_mm - vt.l0;
    const int32_t dr = p_motion->dist_r_mm - vt.r0;
    const int32_t centre = (dl + dr) / 2;
    const int32_t longest = (dl > dr) ? dl : dr;
    const int32_t turned = turned_mrad(p_motion, vt.side);
    /* where the arc wants the heading after this much driving */
    const int32_t wanted = (centre * 1000) / r;
    const int32_t cap =
        (int32_t)(((int64_t)VT_PI_1000 * (r + (w / 2)) * VC_CORNER_CAP_PCT) /
                  (2 * 1000 * 100));
    const bool seeking = (turned >= deg_to_mrad(VC_CORNER_SEEK_DEG));
    const bool found = seeking && line_centred(inner_black, outer_black);
    const bool over =
        (turned >= deg_to_mrad(VC_CORNER_DEG + VC_CORNER_SEEK_MORE_DEG)) ||
        (longest >= cap);
    int32_t diff;
    int16_t inner_mm_s;
    int16_t outer_mm_s;

    if (found || over)
    {
        /* straight again; Buddy 3 takes over on the next pass */
        wheels(p_motion, (int16_t)v, (int16_t)v);
        tm_printf((UB *)"[VC] turn %s done: %s, turned %d deg, axle drove "
                        "%d mm (wheels %d / %d mm)\n",
                  turn_text(vt.turn),
                  found ? "line centred between the IRs"
                        : "line NOT found, Buddy 3 takes over anyway",
                  (INT)mrad_to_deg(turned), (INT)centre, (INT)dl, (INT)dr);
        vt.phase = VT_IDLE;
        return VC_TURN_DONE;
    }

#if VC_CORNER_PIVOT
    /* nyx, 2026-11-07: pivot on the inner wheel (see vc_turn.h) */
    if (centre < VC_CORNER_PREDRIVE_MM)
    {
        wheels(p_motion, (int16_t)v, (int16_t)v);
    }
    else if (vt.side > 0)
    {
        wheels(p_motion, 0, (int16_t)VC_CORNER_PIVOT_MM_S);
    }
    else
    {
        wheels(p_motion, (int16_t)VC_CORNER_PIVOT_MM_S, 0);
    }
    (void)wanted;
    return VC_TURN_RUNNING;
#endif

    /* Wheel speed difference for the arc, plus a correction that pulls
     * the heading back onto it. Past 90 deg (the line not found yet) the
     * arc tightens as far as the inner wheel allows (nyx, 2026-11-07). */
    diff = (w * v) / r;
    if (turned < deg_to_mrad(VC_CORNER_DEG))
    {
        diff += (w * VC_CORNER_GAIN_PER_S * (wanted - turned)) / 1000;
    }
    else
    {
        diff = 2 * v; /* limited to the inner wheel's minimum below */
    }
    if (diff < 0)
    {
        diff = 0;
    }
    /* the inner wheel keeps rolling (see "Slow wheels" in vc_turn.h) */
    if ((v > VC_ARC_MIN_WHEEL_MM_S) &&
        (diff > (2 * (v - VC_ARC_MIN_WHEEL_MM_S))))
    {
        diff = 2 * (v - VC_ARC_MIN_WHEEL_MM_S);
    }
    if (diff > (2 * v))
    {
        diff = 2 * v;
    }
    inner_mm_s = (int16_t)(v - (diff / 2));
    outer_mm_s = (int16_t)(v + (diff / 2));

    if (vt.side > 0)
    {
        wheels(p_motion, inner_mm_s, outer_mm_s);
    }
    else
    {
        wheels(p_motion, outer_mm_s, inner_mm_s);
    }

    return VC_TURN_RUNNING;
}

/* U-turn: ease down, then pivot left on the spot until the line is back
 * between the IRs. */
static vc_turn_result_t uturn_step(const motion_status_t *p_motion,
                                   bool left_black, bool right_black,
                                   int32_t dt_ms)
{
    const int32_t half = track_mm() / 2;
    vc_turn_result_t result = VC_TURN_RUNNING;

    if (VT_SLOWING == vt.phase)
    {
        vt.speed_ums -= VC_SOFT_DECEL_MMPS2 * dt_ms;
        if (vt.speed_ums > (VC_SOFT_STOP_MM_S * 1000))
        {
            const int16_t s = (int16_t)(vt.speed_ums / 1000);

            wheels(p_motion, s, s);
            return VC_TURN_RUNNING;
        }
        /* slow enough: the pivot starts from here, no full stop needed;
         * the right wheel stays at VC_MIN_WHEEL_MM_S, the left one
         * reverses */
        vt.l0 = p_motion->dist_l_mm;
        vt.r0 = p_motion->dist_r_mm;
        vt.pivot_ums = VC_MIN_WHEEL_MM_S * 1000;
        vt.phase = VT_PIVOT;
        tm_printf((UB *)"[VC] U-turn: pivoting left at up to %d deg/s\n",
                  (INT)VC_UTURN_RATE_DPS);
    }

    {
        const int32_t turned = turned_mrad(p_motion, 1);
        const int32_t seek_from = deg_to_mrad(180 - VC_UTURN_SEEK_DEG);
        /* wheel speeds (um/s) for the two pivot rates */
        const int32_t fast = (VC_UTURN_RATE_DPS * VT_PI_1000 * half) / 180;
        const int32_t slow = (VC_UTURN_SEEK_DPS * VT_PI_1000 * half) / 180;
        int32_t limit = fast;
        int32_t w;
        int16_t s;

        /* IR_L leads in a left pivot */
        if ((turned >= seek_from) && line_centred(left_black, right_black))
        {
            /* roll on slowly; not (0, 0): that is a stop */
            wheels(p_motion, VC_MIN_WHEEL_MM_S, VC_MIN_WHEEL_MM_S);
            tm_printf((UB *)"[VC] U-turn done: line centred between the "
                            "IRs, turned %d deg\n",
                      (INT)mrad_to_deg(turned));
            vt.phase = VT_IDLE;
            return VC_TURN_DONE_STOPPED;
        }
        if (turned >= deg_to_mrad(180 + VC_UTURN_OVER_DEG))
        {
            wheels(p_motion, VC_MIN_WHEEL_MM_S, VC_MIN_WHEEL_MM_S);
            tm_printf((UB *)"[VC] U-turn: line NOT found after %d deg, "
                            "Buddy 3 takes over anyway\n",
                      (INT)mrad_to_deg(turned));
            vt.phase = VT_IDLE;
            return VC_TURN_DONE_STOPPED;
        }

        if (turned >= seek_from)
        {
            limit = slow;
        }
        else
        {
            /* slow down in time for the seek part:
             * w^2 = slow^2 + 2 a (wheel arc left) */
            const int64_t left_um =
                ((int64_t)(seek_from - turned) * half); /* mrad x mm = um */
            const int32_t brake =
                isqrt(((int64_t)slow * slow) +
                      (2 * (int64_t)VC_PIVOT_ACCEL_MMPS2 * 1000 * left_um));

            if (brake < limit)
            {
                limit = brake;
            }
        }

        w = vt.pivot_ums + (VC_PIVOT_ACCEL_MMPS2 * dt_ms);
        if (w > limit)
        {
            w = limit;
        }
        if (w < (VC_MIN_WHEEL_MM_S * 1000))
        {
            w = VC_MIN_WHEEL_MM_S * 1000;
        }
        vt.pivot_ums = w;
        s = (int16_t)((w + 500) / 1000);
        wheels(p_motion, (int16_t)(-s), s);
    }

    return result;
}

vc_turn_result_t vc_turn_step(const motion_status_t *p_motion, bool left_black,
                              bool right_black, int32_t dt_ms)
{
    vt.since_send_ms += dt_ms;

    switch (vt.phase)
    {
        case VT_ARC:
            /* the inner IR is on the side of the turn */
            return (vt.side > 0) ? arc_step(p_motion, left_black, right_black)
                                 : arc_step(p_motion, right_black, left_black);
        case VT_SLOWING:
        case VT_PIVOT:
            return uturn_step(p_motion, left_black, right_black, dt_ms);
        case VT_IDLE:
        default:
            return VC_TURN_DONE;
    }
}

/* One wheel of the soft start: pct of its speed, but not below
 * VC_MIN_WHEEL_MM_S (or the wheel's own speed, if that is lower). */
static int16_t soft_wheel(int16_t speed, int32_t pct)
{
    const int32_t mag = (speed < 0) ? -(int32_t)speed : (int32_t)speed;
    int32_t out = (mag * pct) / 100;

    if (out < VC_MIN_WHEEL_MM_S)
    {
        out = (mag < VC_MIN_WHEEL_MM_S) ? mag : VC_MIN_WHEEL_MM_S;
    }
    return (int16_t)((speed < 0) ? -out : out);
}

void vc_turn_soft_start(int16_t *p_left, int16_t *p_right, int32_t elapsed_ms)
{
    int32_t pct;

    if ((elapsed_ms < 0) || (elapsed_ms >= VC_SOFT_START_MS))
    {
        return;
    }

    pct = VC_SOFT_START_PCT +
          (((100 - VC_SOFT_START_PCT) * elapsed_ms) / VC_SOFT_START_MS);

    *p_left = soft_wheel(*p_left, pct);
    *p_right = soft_wheel(*p_right, pct);
}
