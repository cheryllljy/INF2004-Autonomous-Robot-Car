/*
 *      vc_obstacle.c
 *      VehicleController -- the obstacle part (Buddy 5 hook, v3.0).
 *      See vc_obstacle.h for the manoeuvre. Runs inside vc_task only.
 *
 *      Every motion is one Buddy 2 command (motion_turn / motion_move),
 *      issued when MotionControl is IDLE and finished when it is IDLE again
 *      (DONE, or STOPPED after we cut a leg short). A leg is cut short with
 *      motion_stop(), which ramps down; the HARD_STOP cut is separate and
 *      instant.
 */

#include "vc_obstacle.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem5/s5_config.h" /* S5_REVERSE_MIN/MAX_MM (v3.1) */

/* ------------------------------------------------------------------ */
/*  State (vc_task only)                                              */
/* ------------------------------------------------------------------ */

typedef enum
{
    VO_CMD_NONE = 0,
    VO_CMD_TURN,
    VO_CMD_MOVE
} vo_cmd_t;

/* Why a leg was cut short (decides the next step). */
typedef enum
{
    VO_CUT_NONE = 0,
    VO_CUT_PASSED,
    VO_CUT_SIDE_CLOSE,
    VO_CUT_BLOCKED
} vo_cut_t;

typedef struct
{
    vc_obs_step_t step;
    int8_t side; /* +1 pass on the left, -1 right */

    /* one motion command in flight */
    vo_cmd_t cmd;
    int32_t p1;
    int32_t p2;
    bool issued;
    bool seen_busy;
    bool stopping;
    INT cmd_ms;
    INT cmd_limit_ms;
    int32_t cmd_origin_mm; /* distance when the move started */
    vo_cut_t cut;

    /* latched S5 events, consumed by the steps */
    bool ev_profiled;
    bool ev_cleared;
    bool ev_passed;
    bool ev_side_close;
    bool ev_fault;

    /* dead reckoning, 1/256 mm, frame = pose at detection */
    int32_t h0_deg;
    int32_t last_dist_mm;
    int32_t x_q;
    int32_t y_q;

    /* the plan */
    uint16_t nearest_mm;
    uint16_t swing_mm;
    int32_t out_deg;
    int32_t y_scan_q;     /* y when the profile was taken */
    int32_t pass_end_y_q; /* hard cap on the pass leg */
    int32_t pass_fallback_y_q;
    int32_t line_hit_dist_mm;
    bool line_hit;
    bool watching;
    bool interrupted; /* a HARD_STOP cut hit a leg */
    uint8_t stop_scans;
    uint8_t sidesteps;
    uint8_t replans;
    INT dwell_ms;

    INT step_ms;
    INT total_ms;
    INT quiet_ms; /* since the last S5 event */
    int32_t rejoin_mm;

    /* nyx, 2026-11-06 (v3.1) */
    bool remeasured;  /* backed off + rescanned already */
    uint8_t profiles; /* profiles seen this episode (log) */
    INT arc_send_ms;  /* since the last wheel speed update */
    int8_t ir_first;  /* IR that hit the line first: +1 left,
                                           -1 right, 0 none yet */
    bool ir_centred;  /* line passed between the IRs */

    /* nyx, 2026-11-06 (v3.2) */
    obstacle_profile_t first; /* first BYPASS profile of the episode */
    bool have_first;
    int32_t rev_y0_q; /* y when the slow back-off started */
    bool searching;   /* arc: dead reckoning says "there", the
                                  IRs have not seen the line yet */
    int32_t search_start_mm;
    int32_t enc_l0_mm; /* wheel distances at the start */
    int32_t enc_r0_mm;

    /* nyx, 2026-11-06 (v3.4) */
    int32_t h_deg;          /* heading at the last pose update (log) */
    int32_t in_start_mm;    /* distance when the way back started */
    int32_t in_cap_mm;      /* give up after driving this far back */
    int32_t line_window_mm; /* lines further out are another lane's */
    bool ignoring;          /* an IR is on another lane's line */
    uint8_t lines_ignored;
} vc_obs_t;

static vc_obs_t vo;
static bool g_cut; /* HARD_STOP motor cut active */
static INT g_still_ms;
static int32_t g_last_rejoin_mm;

#define VO_Q \
    256 /* pose in 1/256 mm: no truncation
                                           drift over a few thousand steps */
#define VO_DWELL_MS \
    150                      /* after a turn, let S5 take a fresh
                                           front ping before trusting it */
#define VO_CMD_START_MS 1000 /* MotionControl never went busy */

/* ------------------------------------------------------------------ */
/*  Small helpers                                                     */
/* ------------------------------------------------------------------ */

static int32_t vo_abs(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t vo_heading(const motion_status_t *m)
{
    return m->heading_deg - vo.h0_deg;
}

static int32_t vo_x_mm(void)
{
    return vo.x_q / VO_Q;
}

static void vo_goto(vc_obs_step_t next)
{
    if (next != vo.step)
    {
        tm_printf((UB *)"[VC-OBS] step %d -> %d  (x %d y %d mm)\n",
                  (INT)vo.step, (INT)next, (INT)vo_x_mm(),
                  (INT)(vo.y_q / VO_Q));
        vo.step = next;
        vo.step_ms = 0;
    }
}

static void vo_pose_update(const motion_status_t *m)
{
    const int32_t d = m->distance_total_mm;
    const int32_t ds = d - vo.last_dist_mm;
    const int32_t h = vo_heading(m) * 10; /* 0.1 deg */

    vo.last_dist_mm = d;
    vo.h_deg = h / 10; /* nyx, 2026-11-06 (v3.4): for the log */

    /* x is +left: heading +h (left) moves the car to +x. */
    vo.x_q += (ds * VO_Q * obstacle_sin_q15_ddeg(h)) / 32768;
    vo.y_q += (ds * VO_Q * obstacle_sin_q15_ddeg(900 - h)) / 32768;
}

static void vo_cmd_set(vo_cmd_t cmd, int32_t p1, int32_t p2)
{
    vo.cmd = cmd;
    vo.p1 = p1;
    vo.p2 = p2;
    vo.issued = false;
    vo.seen_busy = false;
    vo.stopping = false;
    vo.cmd_ms = 0;
    vo.cut = VO_CUT_NONE;

    if (VO_CMD_TURN == cmd)
    {
        vo.cmd_limit_ms = 2000 + ((vo_abs(p1) * 1000) / VC_OBS_TURN_DPS) * 2;
    }
    else
    {
        vo.cmd_limit_ms =
            2000 + ((vo_abs(p1) * 1000) / ((p2 > 0) ? p2 : 1)) * 2;
    }
}

/* Turn to an absolute heading in the detection frame. Under 2 deg is
   within Buddy 2's turn tolerance (NFR13): nothing to do. */
static void vo_turn_to(const motion_status_t *m, int32_t heading_deg)
{
    const int32_t a = heading_deg - vo_heading(m);

    if (vo_abs(a) < 2)
    {
        vo_cmd_set(VO_CMD_NONE, 0, 0);
    }
    else
    {
        vo_cmd_set(VO_CMD_TURN, a, VC_OBS_TURN_DPS);
    }
}

/* Cut the current leg short; the step learns why from vo.cut. */
static void vo_cmd_cut(vo_cut_t why)
{
    if (!vo.stopping && (VO_CMD_NONE != vo.cmd))
    {
        vo.stopping = true;
        vo.cut = why;
        (void)motion_stop();
    }
}

/*
 * Drive the command in flight. 0 = still going, 1 = finished (target
 * reached, or stopped by us), -1 = refused / failed / timed out.
 */
static INT vo_cmd_run(const motion_status_t *m, INT period_ms)
{
    INT r = 0;

    if (VO_CMD_NONE == vo.cmd)
    {
        return 1;
    }

    vo.cmd_ms += period_ms;

    if (!vo.issued)
    {
        if (MOTION_STATE_IDLE == m->state)
        {
            const motion_err_t er = (VO_CMD_TURN == vo.cmd)
                                        ? motion_turn(vo.p1, vo.p2)
                                        : motion_move(vo.p1, vo.p2);

            if (MOTION_OK == er)
            {
                vo.issued = true;
                vo.cmd_ms = 0;
                vo.cmd_origin_mm = m->distance_total_mm;
            }
        }
        if (!vo.issued && (vo.cmd_ms > VO_CMD_START_MS))
        {
            r = -1;
        }
    }
    else if (MOTION_STATE_IDLE != m->state)
    {
        vo.seen_busy = true;
        if (vo.cmd_ms > vo.cmd_limit_ms)
        {
            (void)motion_stop();
            r = -1;
        }
    }
    else if (vo.seen_busy || (vo.cmd_ms > VO_CMD_START_MS))
    {
        /* Back to IDLE. A move so short it never showed busy still
                   ends DONE. */
        r = ((MOTION_RESULT_DONE == m->last_result) ||
             (MOTION_RESULT_STOPPED == m->last_result))
                ? 1
                : -1;
    }
    else
    {
        /* Issued, MotionControl has not picked it up yet. */
    }

    if (0 != r)
    {
        if (r < 0)
        {
            tm_printf((UB *)"[VC-OBS] motion command %d (%d) failed,"
                            " result %d\n",
                      (INT)vo.cmd, (INT)vo.p1, (INT)m->last_result);
        }
        vo.cmd = VO_CMD_NONE;
    }

    return r;
}

static void vo_fail(const char *why)
{
    /* nyx, 2026-11-06 (v3.4): where the car was, for the log. */
    tm_printf((UB *)"[VC-OBS] FAIL: %s (x %d y %d mm from the detection "
                    "point, heading %d deg)\n",
              why, (INT)vo_x_mm(), (INT)(vo.y_q / VO_Q), (INT)vo.h_deg);
    if (vo.watching)
    {
        obstacle_watch_end();
        vo.watching = false;
    }
    (void)motion_stop();
    vo_goto(VC_OBS_FAILED);
}

/* Square up to the detection heading, then ask S5 for a fresh scan. */
static void vo_replan(const motion_status_t *m, const char *why)
{
    tm_printf((UB *)"[VC-OBS] replan: %s\n", why);
    if (vo.watching)
    {
        obstacle_watch_end();
        vo.watching = false;
    }
    vo.replans++;
    if (vo.replans > VC_OBS_MAX_REPLANS)
    {
        vo_fail("too many replans");
    }
    else
    {
        vo_turn_to(m, 0);
        vo_goto(VC_OBS_ALIGN_SCAN);
    }
}

/* Front range along the current heading (S5 keeps pinging ahead in
   ASSIST and on every second look in WATCH). Only something that lies
   on what is LEFT of this leg counts: on the way back to the line a
   wall beyond the line is in the beam but not in the way (sim C5). */
static bool vo_front_blocked(const motion_status_t *m, int32_t leg_left_mm)
{
    const uint16_t f = obstacle_get_front_mm();
    int32_t limit = VC_OBS_LEG_BLOCK_MM;
    int32_t left = leg_left_mm;

    if ((left < 0) && (VO_CMD_MOVE == vo.cmd) && vo.issued)
    {
        left = vo_abs(vo.p1) - vo_abs(m->distance_total_mm - vo.cmd_origin_mm);
    }
    if (left >= 0)
    {
        left = ((left > 0) ? left : 0) + VC_OBS_BLOCK_AHEAD_MM;
        if (left < limit)
        {
            limit = left;
        }
    }

    return (OBSTACLE_RANGE_CLEAR != f) && (OBSTACLE_RANGE_UNKNOWN != f) &&
           ((int32_t)f < limit);
}

/* ------------------------------------------------------------------ */
/*  Plan from a profile                                               */
/* ------------------------------------------------------------------ */

/* nyx, 2026-11-06: what S5 measured, so the width can be checked with a
   ruler. Width is the beam-corrected face width; a horizontally sweeping
   HC-SR04 cannot measure the obstacle's HEIGHT, so none is printed. */
static void vo_print_profile(const obstacle_profile_t *p)
{
    static const char *const acts[] = {
        "CONTINUE", "SLOW", "STOP", "BYPASS_LEFT", "BYPASS_RIGHT", "REVERSE"
    };

    vo.profiles++;
    tm_printf(
        (UB *)"[VC-OBS] scan %d: obstacle width %d mm, edges "
              "L %d R %d deg, centre %d deg, nearest %d mm, swing "
              "L %d R %d mm, %d%%, %s\n",
        (INT)vo.profiles, (INT)p->width_mm, (INT)p->left_edge_deg,
        (INT)p->right_edge_deg, (INT)p->bearing_deg, (INT)p->nearest_mm,
        (INT)p->swing_left_mm, (INT)p->swing_right_mm, (INT)p->confidence_pct,
        ((UINT)p->action < (sizeof(acts) / sizeof(acts[0]))) ? acts[p->action]
                                                             : "?");
}

/*
 * nyx, 2026-11-06: smallest out angle that still clears. The OUT leg must
 * move the car `lateral` sideways while it moves at most `forward`
 * towards the obstacle: lateral / tan(a) <= forward, i.e.
 * lateral * cos(a) <= forward * sin(a). A shallow angle keeps the car
 * near its lane and makes the turns small; a close obstacle gets a
 * steeper one. v3.0 used 45 deg, or 70 if the obstacle was close.
 */
static int32_t vo_out_angle(int32_t lateral_mm, int32_t forward_mm)
{
    int32_t a = VC_OBS_TURN_MIN_DEG;
    bool found = false;

    if (forward_mm > 0)
    {
        while (!found && (a < VC_OBS_TURN_STEEP_DEG))
        {
            const int32_t sn = obstacle_sin_q15_ddeg(a * 10);
            const int32_t cs = obstacle_sin_q15_ddeg(900 - (a * 10));

            if ((lateral_mm * cs) <= (forward_mm * sn))
            {
                found = true;
            }
            else
            {
                a++;
            }
        }
    }
    else
    {
        a = VC_OBS_TURN_STEEP_DEG;
    }

    return a;
}

static bool vo_plan_bypass(const motion_status_t *m,
                           const obstacle_profile_t *p)
{
    int32_t lateral;

    vo.side = (OBSTACLE_ACT_BYPASS_LEFT == p->action) ? 1 : -1;
    vo.have_first = false; /* v3.2: only for the back-off rescan */
    vo.nearest_mm = p->nearest_mm;
    vo.swing_mm = (vo.side > 0) ? p->swing_left_mm : p->swing_right_mm;
    vo.y_scan_q = vo.y_q;
    vo.sidesteps = 0U;

    lateral = (int32_t)vo.swing_mm + VC_OBS_EXTRA_MM;

    /* nyx, 2026-11-06: computed, not 45/70 (see vo_out_angle()). */
    vo.out_deg =
        vo_out_angle(lateral, (int32_t)vo.nearest_mm - VC_OBS_FRONT_MARGIN_MM);

    /* Pass leg ends, at the latest, past the deepest obstacle we
           accept; the fallback (no side data) is S5's v2 estimate. */
    vo.pass_end_y_q =
        vo.y_q + VO_Q * ((int32_t)vo.nearest_mm + VC_OBS_MAX_DEPTH_MM +
                         VC_OBS_PASS_TAIL_MM);
    vo.pass_fallback_y_q =
        vo.y_q + VO_Q * (int32_t)obstacle_pass_distance_mm(p);

    tm_printf((UB *)"[VC-OBS] bypass %s: nearest %d mm, swing %d mm,"
                    " out %d deg\n",
              (vo.side > 0) ? "LEFT" : "RIGHT", (INT)vo.nearest_mm,
              (INT)vo.swing_mm, (INT)vo.out_deg);

    vo_turn_to(m, vo.side * vo.out_deg);

    return (vo.swing_mm > 0U);
}

/*
 * nyx, 2026-11-06 (v3.2): the scan after the slow back-off did not give
 * a bypass (CONTINUE, STOP or SLOW). Obstacles on the course do not
 * move, so the first scan still holds: plan from it, with the nearest
 * point moved out by how far the car backed off (or the fresh front
 * range, if the sensor sees it straight ahead). Before v3.2 a CONTINUE
 * here ended the episode, the car drove forward, the guard fired again
 * and the car reversed again: the loop nyx saw.
 */
static bool vo_plan_from_first(const motion_status_t *m, const char *why)
{
    obstacle_profile_t p = vo.first;
    const uint16_t f = obstacle_get_front_mm();
    int32_t back = (vo.rev_y0_q - vo.y_q) / VO_Q;
    int32_t est;

    if (back < 0)
    {
        back = 0;
    }
    est = (int32_t)vo.first.nearest_mm + back;

    if ((OBSTACLE_RANGE_CLEAR != f) && (OBSTACLE_RANGE_UNKNOWN != f) &&
        ((int32_t)f < (est + 150)))
    {
        est = (int32_t)f;
    }
    if (est > 0xFFF0)
    {
        est = 0xFFF0;
    }
    p.nearest_mm = (uint16_t)est;

    tm_printf((UB *)"[VC-OBS] rescan said %s; keeping the first scan "
                    "(backed off %d mm, nearest now %d mm)\n",
              why, (INT)back, (INT)est);

    return vo_plan_bypass(m, &p);
}

/* Lateral target of the OUT leg, as a distance along the out heading. */
static int32_t vo_out_distance(void)
{
    const int32_t target = vo.side * ((int32_t)vo.swing_mm + VC_OBS_EXTRA_MM);
    const int32_t need = vo_abs(target - vo_x_mm());

    return (need * 32768) / obstacle_sin_q15_ddeg(vo.out_deg * 10);
}

static void vo_start_pass(const motion_status_t *m)
{
    int32_t cap = (vo.pass_end_y_q - vo.y_q) / VO_Q;

    (void)m;

    if (cap < 50)
    {
        cap = 50;
    }

    if (!vo.watching)
    {
        /* We pass on the left -> the obstacle is on our right. */
        obstacle_watch_start(vo.side < 0);
        vo.watching = true;
    }

    vo.ev_passed = false;
    vo.ev_side_close = false;
    vo_cmd_set(VO_CMD_MOVE, cap, VC_OBS_SPEED_MM_S);
    vo_goto(VC_OBS_PASS);
}

/* nyx, 2026-11-06 (v3.1): the way back is one arc driven with wheel
   speeds (VC_OBS_IN), not pivot + straight + pivot. */
static void vo_start_in(const motion_status_t *m)
{
    const int32_t off = vo_abs(vo_x_mm());
    int32_t window = ((int32_t)vo.swing_mm + VC_OBS_EXTRA_MM) / 2;

    if (vo.watching)
    {
        obstacle_watch_end();
        vo.watching = false;
    }
    vo_cmd_set(VO_CMD_NONE, 0, 0);
    vo.arc_send_ms = VC_OBS_ARC_SEND_MS; /* send at once */
    vo.ir_first = 0;
    vo.ir_centred = false;
    vo.line_hit = false;
    vo.searching = false;

    /* nyx, 2026-11-06 (v3.4): a line met on the way back while dead
     * reckoning still has the car more than this from its own line is
     * another lane's (the course has lanes). Half the bypass offset, at
     * least VC_OBS_LINE_WINDOW_MIN_MM. */
    if (window < VC_OBS_LINE_WINDOW_MIN_MM)
    {
        window = VC_OBS_LINE_WINDOW_MIN_MM;
    }
    vo.line_window_mm = window;
    vo.ignoring = false;
    vo.lines_ignored = 0U;

    /* nyx, 2026-11-06 (v3.4): give up by distance, not time, so a slow
     * car is not cut off: the longest path back at the shallowest
     * approach, the line search, and some slack. */
    vo.in_start_mm = m->distance_total_mm;
    vo.in_cap_mm =
        ((off * 32768) / obstacle_sin_q15_ddeg(VC_OBS_APPROACH_MIN_DEG * 10)) +
        VC_OBS_SEARCH_MM + VC_OBS_ARC_SLACK_MM;

    tm_printf((UB *)"[VC-OBS] back to the line: %d mm to the side, heading "
                    "%d deg; lines over %d mm from ours are other lanes\n",
              (INT)vo_x_mm(), (INT)vo_heading(m), (INT)window);
    vo_goto(VC_OBS_IN);
}

static int32_t vo_clamp(int32_t v, int32_t lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

/*
 * One 10 ms step of the arc. Returns 1 when the car is on the centre and
 * straight, 0 while it is still easing in, -1 if the line was not found
 * (nyx, 2026-11-06, v3.2).
 *
 *  - heading asked:  hd = -x / lookahead (rad), at most VC_OBS_ARC_MAX_DEG.
 *    Far from the line the car cuts back at up to 30 deg; as x shrinks so
 *    does hd, so it arrives straight instead of crossing the line.
 *  - steering:       right - left = 2 * gain * (hd - h); + turns left.
 *  - IR fix:         with the sensors straddling the line, the line is
 *    between them when the IR that went black first reads white again
 *    and the other has not gone black: the sensor midpoint is then on the
 *    centre, so x is reset from it (the IRs are VC_OBS_IR_AHEAD_MM ahead
 *    of the axle).
 */
static INT vo_arc_step(const motion_status_t *m, bool irl, bool irr,
                       INT period_ms)
{
    const int32_t h = vo_heading(m);
    int32_t x = vo_x_mm();
    /* nyx, 2026-11-06 (v3.4): where dead reckoning has the IRs. */
    const int32_t x_ir =
        x + ((VC_OBS_IR_AHEAD_MM * obstacle_sin_q15_ddeg(h * 10)) / 32768);
    int32_t hd;
    int32_t diff;
    int32_t speed;
    INT done = 0;

    if (!vo.ir_centred)
    {
        if ((0 == vo.ir_first) && vo.ignoring)
        {
            /* nyx, 2026-11-06 (v3.4): on another lane's line. */
            if (!irl && !irr)
            {
                vo.ignoring = false;
            }
        }
        else if ((0 == vo.ir_first) && (irl || irr) && !vo.searching &&
                 ((vo.side * x_ir) > vo.line_window_mm))
        {
            /* nyx, 2026-11-06 (v3.4): still far from our line by dead
             * reckoning, so this is the next lane's line (the course
             * has lanes; a wide detour crosses it). Driving on. While
             * searching, dead reckoning is already known to be off, so
             * the first line is taken. */
            vo.ignoring = true;
            vo.lines_ignored++;
            tm_printf((UB *)"[VC-OBS] crossed a line %d mm from ours by "
                            "dead reckoning: another lane, ignored\n",
                      (INT)x_ir);
        }
        else if (0 == vo.ir_first)
        {
            if (irl != irr)
            {
                vo.ir_first = irl ? 1 : -1;
                vo.line_hit = true;
                tm_printf((UB *)"[VC-OBS] IR_%s on the line, %d mm from "
                                "where dead reckoning has it (heading %d "
                                "deg)\n",
                          irl ? "L" : "R", (INT)x_ir, (INT)h);
                vo.line_hit_dist_mm = m->distance_total_mm;
                if (vo.searching)
                {
                    /* nyx, 2026-11-06 (v3.2): dead reckoning was
                     * wrong; the IRs are on the line now, so put
                     * the axle IR_AHEAD behind it. */
                    vo.searching = false;
                    vo.x_q = -((int32_t)VC_OBS_IR_AHEAD_MM * VO_Q *
                               obstacle_sin_q15_ddeg(h * 10)) /
                             32768;
                    x = vo_x_mm();
                    tm_printf((UB *)"[VC-OBS] found the line after "
                                    "searching %d mm\n",
                              (INT)(m->distance_total_mm - vo.search_start_mm));
                }
            }
        }
        else if (!irl && !irr)
        {
            vo.ir_centred = true;
            vo.x_q = -((int32_t)VC_OBS_IR_AHEAD_MM * VO_Q *
                       obstacle_sin_q15_ddeg(h * 10)) /
                     32768;
            x = vo_x_mm();
            tm_printf((UB *)"[VC-OBS] line between the IRs: offset "
                            "reset to %d mm (heading %d deg)\n",
                      (INT)x, (INT)h);
        }
        else if ((vo.ir_first > 0) ? irr : irl)
        {
            /* The other IR is on it too: crossed at an angle with
                           no white gap seen. Keep dead reckoning. */
            vo.ir_centred = true;
        }
        else
        {
            /* still on the first IR */
        }
    }

    /* nyx, 2026-11-06 (v3.2): never hand back to line following
     * before the IRs have seen the line. With the IRs straddling the
     * line, "both white" off the track looks exactly like "centred", so
     * the line follower would drive straight on. If dead reckoning says
     * we are back (or past) but no IR went black, keep closing in on
     * the line side at VC_OBS_SEARCH_DEG for at most VC_OBS_SEARCH_MM. */
    if (!vo.line_hit && ((vo.side * x) <= VC_OBS_SEARCH_X_MM))
    {
        if (!vo.searching)
        {
            vo.searching = true;
            vo.search_start_mm = m->distance_total_mm;
            tm_printf((UB *)"[VC-OBS] dead reckoning says %d mm from "
                            "the line but no IR has seen it: searching\n",
                      (INT)x);
        }
        hd = -vo.side * VC_OBS_SEARCH_DEG;
        if (vo_abs(m->distance_total_mm - vo.search_start_mm) >
            VC_OBS_SEARCH_MM)
        {
            done = -1;
        }
    }
    else
    {
        hd = vo_clamp((-x * 573) / (VC_OBS_ARC_LOOKAHEAD_MM * 10),
                      VC_OBS_ARC_MAX_DEG);

        /* nyx, 2026-11-06 (v3.2): until an IR has seen the line, cut
         * back at least VC_OBS_APPROACH_MIN_DEG. Dead reckoning drifts
         * (wheel slip in the pivot turns), and easing in as x -> 0 made
         * the car creep alongside a line it never reached. */
        if (!vo.line_hit && ((vo.side * hd) > -VC_OBS_APPROACH_MIN_DEG))
        {
            hd = -vo.side * VC_OBS_APPROACH_MIN_DEG;
        }
    }
    diff = vo_clamp((hd - h) * VC_OBS_ARC_GAIN, VC_OBS_ARC_DIFF_MM_S);

    /* nyx, 2026-11-06 (v3.4): slow down near the line, so the IRs see it
     * and Buddy 3 takes over gently. */
    speed =
        (vo.line_hit || vo.searching || ((vo.side * x_ir) < VC_OBS_ARC_NEAR_MM))
            ? VC_OBS_ARC_NEAR_SPEED_MM_S
            : VC_OBS_ARC_SPEED_MM_S;

    if (vo.line_hit && (vo_abs(x) <= VC_OBS_ARC_DONE_MM) &&
        (vo_abs(h) <= VC_OBS_ARC_DONE_DEG))
    {
        done = 1;
        diff = 0; /* hand over driving straight */
    }
    else if (vo.ir_centred && (irl != irr))
    {
        /* nyx, 2026-11-06 (v3.4): the line has been between the IRs and
         * is under one of them again. Buddy 3 steers towards it from
         * here; driving on by dead reckoning could carry the line out
         * past the far IR, and with the IRs straddling the line "both
         * white" off the track looks the same as centred. */
        done = 1;
        tm_printf((UB *)"[VC-OBS] line under IR_%s, Buddy 3 takes over "
                        "(heading %d deg)\n",
                  irl ? "L" : "R", (INT)h);
    }
    else
    {
        /* Easing in. */
    }

    vo.arc_send_ms += period_ms;
    if (done < 0)
    {
        /* Caller stops and fails. */
    }
    else if ((0 != done) || (vo.arc_send_ms >= VC_OBS_ARC_SEND_MS))
    {
        vo.arc_send_ms = 0;
        (void)motion_set_wheel_speeds((int32_t)(speed - diff),
                                      (int32_t)(speed + diff));
    }

    return done;
}

/* ------------------------------------------------------------------ */
/*  Public: vc_task hooks                                             */
/* ------------------------------------------------------------------ */

void vc_obs_begin(const motion_status_t *p_motion)
{
    const bool cut = g_cut;

    vo = (vc_obs_t){ 0 };
    vo.h0_deg = p_motion->heading_deg;
    vo.last_dist_mm = p_motion->distance_total_mm;
    vo.enc_l0_mm = p_motion->dist_l_mm; /* nyx, 2026-11-06 (v3.2) */
    vo.enc_r0_mm = p_motion->dist_r_mm;
    vo.step = VC_OBS_IDLE;
    g_cut = cut; /* a cut may already be on */
    vo_goto(VC_OBS_WAIT_SCAN);
}

void vc_obs_events(UINT s5_events)
{
    if (0U != s5_events)
    {
        vo.quiet_ms = 0;
    }
    if (s5_events & S5_EVT_PROFILED)
    {
        vo.ev_profiled = true;
    }
    if (s5_events & S5_EVT_CLEARED)
    {
        vo.ev_cleared = true;
    }
    if (s5_events & S5_EVT_PASSED)
    {
        vo.ev_passed = true;
    }
    if (s5_events & S5_EVT_SIDE_CLOSE)
    {
        vo.ev_side_close = true;
    }
    if (s5_events & S5_EVT_FAULT)
    {
        vo.ev_fault = true;
    }
}

void vc_obs_leave(void)
{
    if ((VC_OBS_IDLE != vo.step) && (VC_OBS_FINISHED != vo.step))
    {
        if (vo.watching)
        {
            obstacle_watch_end();
        }
        obstacle_resume(); /* S5 back to GUARD */
    }
    vo.watching = false;
    vo.cmd = VO_CMD_NONE;
    vo.step = VC_OBS_IDLE;
}

vc_obs_result_t vc_obs_step(const motion_status_t *p_motion, bool ir_left_black,
                            bool ir_right_black, INT period_ms)
{
    const motion_status_t *m = p_motion;
    vc_obs_result_t res = VC_OBS_RUNNING;
    INT r;

    obstacle_assist_kick(); /* S5 watchdog: VC is alive */
    vo_pose_update(m);
    vo.step_ms += period_ms;
    vo.total_ms += period_ms;
    vo.quiet_ms += period_ms;

    if (VC_OBS_FAILED == vo.step)
    {
        return VC_OBS_FAULT;
    }
    if (VC_OBS_FINISHED == vo.step)
    {
        return VC_OBS_DONE;
    }

    if (vo.total_ms > VC_OBS_TIMEOUT_MS)
    {
        vo_fail("obstacle episode timed out");
        return VC_OBS_FAULT;
    }
    if (vo.ev_fault)
    {
        /* S5 FAULT: its own handling (E-stop bit or latched mission
                   fault) is in read_obstacle(); the bypass is over. */
        /* Diagnostic (Buddy 3, 2026-11-04): which S5 fault, since
                   S5 itself prints nothing that reaches the MQTT log. */
        {
            obstacle_health_t h;
            static const char *const names[] = {
                "none", "SENSOR (no echo)",
                "STOP_TIMEOUT (car not still <= 5 mm/s within 1 s)",
                "REVERSE_LIMIT (third reverse)",
                "ASSIST_TIMEOUT (no VC request for 5 s)"
            };

            obstacle_get_health(&h);
            tm_printf(
                (UB *)"[VC-OBS] S5 fault %u: %s, S5 state %u\n", (UINT)h.fault,
                (h.fault < (sizeof(names) / sizeof(names[0]))) ? names[h.fault]
                                                               : "?",
                (UINT)h.state);
        }
        vo_fail("ObstacleScan fault");
        return VC_OBS_FAULT;
    }

    /* nyx, 2026-11-06 (v3.2): a dead encoder makes the heading and the
     * distances wrong, so every leg and the arc back go astray. */
    {
        const int32_t dl = vo_abs(m->dist_l_mm - vo.enc_l0_mm);
        const int32_t dr = vo_abs(m->dist_r_mm - vo.enc_r0_mm);

        if (((dl >= VC_OBS_ENC_MOVED_MM) && (dr <= VC_OBS_ENC_DEAD_MM)) ||
            ((dr >= VC_OBS_ENC_MOVED_MM) && (dl <= VC_OBS_ENC_DEAD_MM)))
        {
            tm_printf((UB *)"[VC-OBS] %s encoder reads %d mm while the "
                            "other reads %d mm\n",
                      (dr < dl) ? "RIGHT (GP3?)" : "LEFT",
                      (INT)((dr < dl) ? dr : dl), (INT)((dr < dl) ? dl : dr));
            vo_fail("encoder not counting, heading unknown");
            return VC_OBS_FAULT;
        }
    }

    if (g_cut)
    {
        /* HARD_STOP: motors cut. Wait for the release. */
        vo.interrupted = true;
        return res;
    }

    if (vo.interrupted)
    {
        vo.interrupted = false;
        vo.cmd = VO_CMD_NONE;
        if ((VC_OBS_WAIT_SCAN != vo.step) && (VC_OBS_REVERSE != vo.step) &&
            (VC_OBS_ALIGN_SCAN != vo.step))
        {
            vo_replan(m, "hard stop during a leg");
        }
    }

    switch (vo.step)
    {
        case VC_OBS_WAIT_SCAN:
            if (vo.ev_cleared && vo.remeasured && vo.have_first)
            {
                /* nyx, 2026-11-06 (v3.2) */
                vo.ev_cleared = false;
                vo.ev_profiled = false;
                if (vo_plan_from_first(m, "clear"))
                {
                    vo_goto(VC_OBS_TURN_OUT);
                }
                else
                {
                    vo_fail("bypass with no swing");
                }
            }
            else if (vo.ev_cleared)
            {
                vo.ev_cleared = false;
                obstacle_resume();
                tm_printf((UB *)"[VC-OBS] path clear\n");
                vo.rejoin_mm = vo_x_mm();
                g_last_rejoin_mm = vo.rejoin_mm;
                vo_goto(VC_OBS_FINISHED);
                res = VC_OBS_DONE;
            }
            else if (vo.ev_profiled)
            {
                obstacle_profile_t p;
                obstacle_watch_t w;

                vo.ev_profiled = false;
                (void)obstacle_get_profile(&p);
                obstacle_get_watch(&w);

                vo_print_profile(&p); /* nyx, 2026-11-06 */

                switch (p.action)
                {
                    case OBSTACLE_ACT_BYPASS_LEFT:
                    case OBSTACLE_ACT_BYPASS_RIGHT:
                        vo.stop_scans = 0U;
                        if (!vo.remeasured)
                        {
                            /* nyx, 2026-11-06: always back off
                                           slowly and scan again before the
                                           bypass; plan from the 2nd scan. */
                            int32_t back =
                                VC_OBS_REMEASURE_MM - (int32_t)p.nearest_mm;

                            if (back < (int32_t)S5_REVERSE_MIN_MM)
                            {
                                back = (int32_t)S5_REVERSE_MIN_MM;
                            }
                            if (back > (int32_t)S5_REVERSE_MAX_MM)
                            {
                                back = (int32_t)S5_REVERSE_MAX_MM;
                            }
                            vo.remeasured = true;
                            vo.first = p; /* nyx, 2026-11-06 (v3.2) */
                            vo.have_first = true;
                            vo.rev_y0_q = vo.y_q;
                            tm_printf((UB *)"[VC-OBS] reverse %d mm "
                                            "slowly, then scan again\n",
                                      (INT)back);
                            vo_cmd_set(VO_CMD_MOVE, -back,
                                       VC_OBS_REV_SPEED_MM_S);
                            vo_goto(VC_OBS_REVERSE);
                        }
                        else if (vo_plan_bypass(m, &p))
                        {
                            vo_goto(VC_OBS_TURN_OUT);
                        }
                        else
                        {
                            vo_fail("bypass with no swing");
                        }
                        break;
                    case OBSTACLE_ACT_REVERSE:
                        tm_printf((UB *)"[VC-OBS] reverse %d mm to "
                                        "remeasure (nearest %d mm)\n",
                                  (INT)w.reverse_mm, (INT)p.nearest_mm);
                        vo.remeasured = true; /* this IS the rescan */
                        vo.rev_y0_q = vo.y_q;
                        vo_cmd_set(VO_CMD_MOVE, -(int32_t)w.reverse_mm,
                                   VC_OBS_REV_SPEED_MM_S);
                        vo_goto(VC_OBS_REVERSE);
                        break;
                    case OBSTACLE_ACT_CONTINUE:
                        if (vo.remeasured && vo.have_first)
                        {
                            /* nyx, 2026-11-06 (v3.2) */
                            if (vo_plan_from_first(m, "CONTINUE"))
                            {
                                vo_goto(VC_OBS_TURN_OUT);
                            }
                            else
                            {
                                vo_fail("bypass with no swing");
                            }
                        }
                        else
                        {
                            obstacle_resume();
                            vo_goto(VC_OBS_FINISHED);
                            res = VC_OBS_DONE;
                        }
                        break;
                    default:
                        if (vo.remeasured && vo.have_first)
                        {
                            /* nyx, 2026-11-06 (v3.2): STOP / SLOW on
                             * the rescan: the first scan still holds. */
                            if (vo_plan_from_first(m, "STOP/SLOW"))
                            {
                                vo_goto(VC_OBS_TURN_OUT);
                            }
                            else
                            {
                                vo_fail("bypass with no swing");
                            }
                        }
                        else
                        {
                            obstacle_health_t h;

                            /* STOP / SLOW: S5 will look again by
                             * itself from GUARD; after a VC rescan it
                             * waits in ASSIST, so ask again (v3.2). */
                            vo.stop_scans++;
                            tm_printf((UB *)"[VC-OBS] profile says %d, "
                                            "waiting (%d)\n",
                                      (INT)p.action, (INT)vo.stop_scans);
                            obstacle_get_health(&h);
                            if (vo.stop_scans >= VC_OBS_MAX_STOP_SCANS)
                            {
                                vo_fail("no usable profile");
                            }
                            else if (OBSTACLE_ST_ASSIST == h.state)
                            {
                                obstacle_rescan();
                            }
                            else
                            {
                                /* S5 rescans from GUARD. */
                            }
                        }
                        break;
                }
            }
            else if ((vo.stop_scans > 0U) && (vo.quiet_ms > VC_OBS_GONE_MS))
            {
                const uint16_t f = obstacle_get_front_mm();

                if ((OBSTACLE_RANGE_CLEAR == f) ||
                    ((OBSTACLE_RANGE_UNKNOWN != f) &&
                     (f > (uint16_t)OBSTACLE_TRIGGER_MM)))
                {
                    tm_printf((UB *)"[VC-OBS] obstacle gone\n");
                    obstacle_resume();
                    vo_goto(VC_OBS_FINISHED);
                    res = VC_OBS_DONE;
                }
            }
            else
            {
                /* S5 is waiting for the stop, or sweeping. */
            }
            break;

        case VC_OBS_REVERSE:
            r = vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                obstacle_rescan();
                vo_goto(VC_OBS_WAIT_SCAN);
            }
            else if (r < 0)
            {
                vo_fail("reverse refused");
            }
            else
            {
                /* Backing off. */
            }
            break;

        case VC_OBS_ALIGN_SCAN:
            r = vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                vo.ev_profiled = false;
                vo.ev_cleared = false;
                obstacle_rescan();
                vo_goto(VC_OBS_WAIT_SCAN);
            }
            else if (r < 0)
            {
                vo_fail("could not square up");
            }
            else
            {
                /* Turning. */
            }
            break;

        case VC_OBS_TURN_OUT:
            r = (VO_CMD_NONE == vo.cmd) ? 1 : vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                vo.dwell_ms += period_ms;
            }
            if ((r > 0) && (vo.dwell_ms >= VO_DWELL_MS))
            {
                const int32_t d = vo_out_distance();
                const uint16_t f = obstacle_get_front_mm();

                vo.dwell_ms = 0;

                /* The sensor now looks along the out leg. */
                if ((OBSTACLE_RANGE_CLEAR != f) &&
                    (OBSTACLE_RANGE_UNKNOWN != f) &&
                    ((int32_t)f < (d + VC_OBS_FRONT_MARGIN_MM)))
                {
                    if (VC_OBS_TURN_STEEP_DEG != vo.out_deg)
                    {
                        /* Clipping the near corner: steeper. */
                        tm_printf((UB *)"[VC-OBS] out leg blocked "
                                        "at %d mm, turning out steeper\n",
                                  (INT)f);
                        vo.out_deg = VC_OBS_TURN_STEEP_DEG;
                        vo_turn_to(m, vo.side * vo.out_deg);
                    }
                    else
                    {
                        vo_replan(m, "out leg blocked");
                    }
                }
                else
                {
                    vo_cmd_set(VO_CMD_MOVE, d, VC_OBS_SPEED_MM_S);
                    vo_goto(VC_OBS_OUT);
                }
            }
            else if (r < 0)
            {
                vo_fail("turn out refused");
            }
            else
            {
                /* Turning. */
            }
            break;

        case VC_OBS_OUT:
            if (vo_front_blocked(m, -1))
            {
                vo_cmd_cut(VO_CUT_BLOCKED);
            }
            r = vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                if (VO_CUT_BLOCKED == vo.cut)
                {
                    vo_replan(m, "something ahead on the out leg");
                }
                else
                {
                    vo_turn_to(m, 0);
                    vo_goto(VC_OBS_TURN_PAR);
                }
            }
            else if (r < 0)
            {
                vo_fail("out leg refused");
            }
            else
            {
                /* Driving out. */
            }
            break;

        case VC_OBS_TURN_PAR:
            r = vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                vo_start_pass(m);
            }
            else if (r < 0)
            {
                vo_fail("turn parallel refused");
            }
            else
            {
                /* Turning. */
            }
            break;

        case VC_OBS_PASS:
            if (vo.ev_passed)
            {
                vo.ev_passed = false;
                vo.watching = false; /* S5 left WATCH by itself */
                vo_cmd_cut(VO_CUT_PASSED);
            }
            else if (vo.ev_side_close)
            {
                vo.ev_side_close = false;
                vo_cmd_cut(VO_CUT_SIDE_CLOSE);
            }
            else if (vo_front_blocked(m, -1))
            {
                vo_cmd_cut(VO_CUT_BLOCKED);
            }
            else
            {
                /* Driving past. */
            }

            r = vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                obstacle_watch_t w;

                obstacle_get_watch(&w);

                if (VO_CUT_PASSED == vo.cut)
                {
                    tm_printf((UB *)"[VC-OBS] passed (closest face"
                                    " %d mm from the sensor axis)\n",
                              (INT)w.lateral_mm);
                    vo_start_in(m);
                }
                else if (VO_CUT_SIDE_CLOSE == vo.cut)
                {
                    vo.sidesteps++;
                    if (vo.sidesteps > VC_OBS_MAX_SIDESTEPS)
                    {
                        vo_fail("kept drifting onto the obstacle");
                    }
                    else
                    {
                        tm_printf((UB *)"[VC-OBS] too close to the"
                                        " side (%d mm), steering away\n",
                                  (INT)w.lateral_mm);
                        vo_cmd_set(VO_CMD_TURN, vo.side * VC_OBS_SIDESTEP_DEG,
                                   VC_OBS_TURN_DPS);
                        vo_goto(VC_OBS_SIDESTEP);
                    }
                }
                else if (VO_CUT_BLOCKED == vo.cut)
                {
                    vo_replan(m, "something ahead on the pass leg");
                }
                else if ((S5_WATCH_BLIND == w.phase) &&
                         (vo.y_q >= vo.pass_fallback_y_q))
                {
                    /* Never saw it beside us: trust the profile's
                                   estimate, as v2 did. */
                    tm_printf((UB *)"[VC-OBS] no side data, using the"
                                    " profile pass distance\n");
                    vo_start_in(m);
                }
                else
                {
                    vo_fail("pass leg ended, obstacle still beside us");
                }
            }
            else if (r < 0)
            {
                vo_fail("pass leg refused");
            }
            else
            {
                /* Driving past. */
            }
            break;

        case VC_OBS_SIDESTEP:
            r = vo_cmd_run(m, period_ms);
            if (r > 0)
            {
                vo_start_pass(m);
            }
            else if (r < 0)
            {
                vo_fail("side step refused");
            }
            else
            {
                /* Turning away. */
            }
            break;

        case VC_OBS_IN:
            /* nyx, 2026-11-06 (v3.1): smooth arc back to the centre. */
            if (vo_front_blocked(
                    m, ((vo_abs(vo_x_mm()) * 32768) /
                        obstacle_sin_q15_ddeg(VC_OBS_ARC_MAX_DEG * 10)) +
                           VC_OBS_IR_AHEAD_MM))
            {
                /* Only what lies before the line counts. */
                (void)motion_stop();
                vo_replan(m, "something ahead on the way back");
            }
            else if ((vo.step_ms > VC_OBS_ARC_TIMEOUT_MS) ||
                     (vo_abs(m->distance_total_mm - vo.in_start_mm) >
                      vo.in_cap_mm))
            {
                /* nyx, 2026-11-06 (v3.4): by distance, and a long time
                 * cap only for a car that is not moving. */
                vo_fail("arc back to the line did not converge");
            }
            else
            {
                r = vo_arc_step(m, ir_left_black, ir_right_black, period_ms);
                if (r < 0)
                {
                    /* nyx, 2026-11-06 (v3.2): stop rather than drive
                     * off the track. */
                    vo_fail("line not found on the way back");
                }
                else if (0 != r)
                {
                    vo.rejoin_mm = vo_x_mm();
                    g_last_rejoin_mm = vo.rejoin_mm;
                    tm_printf((UB *)"[VC-OBS] rejoined: %d mm from the line"
                                    " (%s, tolerance %d), %d ms\n",
                              (INT)vo.rejoin_mm,
                              (vo_abs(vo.rejoin_mm) <= VC_OBS_REJOIN_TOL_MM)
                                  ? "PASS"
                                  : "FAIL",
                              (INT)VC_OBS_REJOIN_TOL_MM, (INT)vo.total_ms);
                    obstacle_resume();
                    vo_goto(VC_OBS_FINISHED);
                    res = VC_OBS_DONE;
                }
                else
                {
                    /* Easing back in. */
                }
            }
            break;

        case VC_OBS_TURN_IN: /* v3.0 pivot turns, not used since v3.1 */
        case VC_OBS_ALIGN:
        case VC_OBS_IDLE:
        case VC_OBS_FINISHED:
        case VC_OBS_FAILED:
        default:
            break;
    }

    if (VC_OBS_FAILED == vo.step)
    {
        res = VC_OBS_FAULT;
    }

    return res;
}

/* ------------------------------------------------------------------ */
/*  HARD_STOP motor cut                                               */
/* ------------------------------------------------------------------ */

void vc_obs_hard_stop(const motion_status_t *p_motion)
{
    /* Only a car driving INTO the obstacle needs the cut. Backing away
           from it (REVERSE) or turning on the spot next to it, S5 keeps
           reporting "under 120 mm" on every ping, and cutting then would
           pin the car in place. */
    const int32_t v =
        ((int32_t)p_motion->speed_l_mmps + (int32_t)p_motion->speed_r_mmps) / 2;

    if (!g_cut && (v > VC_OBS_STILL_MM_S))
    {
        tm_printf((UB *)"[VC-OBS] HARD STOP: motors cut (%d mm/s)\n", (INT)v);
        motion_estop_request(MOTION_ESTOP_OBSTACLE);
        g_cut = true;
        g_still_ms = 0;
    }
}

void vc_obs_service_cut(const motion_status_t *p_motion, INT period_ms)
{
    if (g_cut)
    {
        if ((vo_abs(p_motion->speed_l_mmps) < VC_OBS_STILL_MM_S) &&
            (vo_abs(p_motion->speed_r_mmps) < VC_OBS_STILL_MM_S))
        {
            g_still_ms += period_ms;
        }
        else
        {
            g_still_ms = 0;
        }

        if (g_still_ms >= VC_OBS_STILL_MS)
        {
            /* Stopped: release our bit only. MotionControl
                           refuses the reset while any other cause is set. */
            motion_estop_clear(MOTION_ESTOP_OBSTACLE);
            (void)motion_reset();
            g_cut = false;
            tm_printf((UB *)"[VC-OBS] hard stop released\n");
        }
    }
}

bool vc_obs_cut_active(void)
{
    return g_cut;
}

vc_obs_step_t vc_obs_get_step(void)
{
    return vo.step;
}

/* nyx, 2026-11-06: vo.side is set only when a bypass is planned */
bool vc_obs_bypassed(void)
{
    return 0 != vo.side;
}

int32_t vc_obs_last_rejoin_mm(void)
{
    return g_last_rejoin_mm;
}
