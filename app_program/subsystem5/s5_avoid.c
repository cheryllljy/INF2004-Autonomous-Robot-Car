/*
 *      s5_avoid.c
 *      Subsystem 5 -- the obstacle bypass, run in the VehicleController's
 *      task. See s5_avoid.h for the episode, s5_scan.h
 *      for the width and s5_path.h for the path.
 *
 *      Frames: while Buddy 3 follows the line, the car's pose is dead
 *      reckoned (s5_odom.c) and the IR pair's track is kept; at the
 *      obstacle a straight line is fitted through that track and the pose
 *      is moved into the line frame (x along the line, y to its left, 0 at
 *      the IR pair). Everything after that, the object, the path and where
 *      the IRs find the line again, is in that frame.
 */

#include "s5_avoid.h"

#include "s5_config.h"
#include "s5_hw.h"
#include "s5_math.h"
#include "s5_odom.h"
#include "s5_path.h"
#include "s5_port.h"
#include "s5_scan.h"
#include "s5_sensor.h"

#define S5_US_PER_MS   (1000U)
#define S5_STALE_US    ((uint32_t)S5_STALE_MS * S5_US_PER_MS)
#define S5_SILENT_US   ((uint32_t)S5_SILENT_MS * S5_US_PER_MS)
#define S5_FAR_N       (16U)   /* far corner pings kept */
#define S5_FAR_MIN     (6U)    /* ... needed for a fix */
#define S5_FAR_SAY_N   (8U)    /* print the fix once it has this many */
#define S5_FAR_SPAN_MM (60.0f) /* ... fired this far apart along it */
#define S5_FAR_FIT_MM  (12.0f) /* every ping within this of the fix */
/* a side further out than this: not this box's corner */
#define S5_FAR_SANE_MM   (150.0f)
#define S5_BISECT        (8U)    /* halvings when searching a distance */
#define S5_LOOK_TOL_DDEG (50)    /* a ping counts for a look within 5 deg */
#define S5_CROSS_MIN_MM  (12.0f) /* an IR crossing the tape moves at least */
#define S5_CROSS_MAX_MM  (40.0f) /* ... and at most this sideways */
/* mean square spread of the track, mm^2: under it (about 70 mm) use the
 * heading */
#define S5_FIT_SPREAD (400.0f)
/* a track point this far off the fit: the line bends there (Buddy 3's
 * zig-zag is within 6 mm) */
#define S5_FIT_RESID_MM (8.0f)
/* the car pointed this far off the fit at a track point: the line bends
 * there */
#define S5_FIT_TURN_DEG    (8.0f)
#define S5_REV_HEADING_MAX (0.17f) /* rad: backing up, steer at most 10 deg */
/* on the way back a ping within S5_DETECT_MM
 * that is this much shorter than anything of the object just passed could
 * answer at (its calibrated beam plus S5_KNOWN_BEAM_DDEG) is another
 * object: stop and start again from the sweep. */
#define S5_KNOWN_TOL_MM    (25.0f)
#define S5_KNOWN_BEAM_DDEG (30)
/* stopped further than this off the line's direction: straighten first
 * (on the spot, or backing up) */
#define S5_SQUARE_DEG (10.0f)
#define S5_SQUARE_MM  (100.0f) /* ... by this much */
#define S5_PLAN_DEG   (15.0f)  /* plan only from closer to straight */
/* straightening on the spot instead, when
 * a sweep hears nothing within S5_PIVOT_ROOM_MM (but the object just gone
 * round) and the car's outline stays S5_PIVOT_MARGIN_MM clear of that
 * object (checked every S5_PIVOT_STEP_DEG); wheels at +-S5_PIVOT_MM_S,
 * slower for the last S5_PIVOT_SLOW_DEG, done within S5_PIVOT_DONE_DEG */
#define S5_PIVOT_ROOM_MM   (200U)
#define S5_PIVOT_MARGIN_MM (15.0f)
#define S5_PIVOT_STEP_DEG  (2.0f)
#define S5_PIVOT_MM_S      (40)
/* 20 stalled on the car, but 30 turned past the line in
 * the simulator (C44, C53, C56); the kick below gets a stalled wheel
 * going instead */
#define S5_PIVOT_SLOW_MM_S (20)
/* turning on the spot, a wheel that gives no ticks for
 * S5_PIVOT_KICK_MS is asked for S5_PIVOT_KICK_MM_S (both wheels, the same
 * turn) until it moves. Buddy 2 starts a standing wheel at its breakaway
 * duty and builds up from there slowly at a low speed; on the car both
 * wheels stood still for 0.6 s at 20 mm/s and the bypass failed. */
#define S5_PIVOT_KICK_MS   (200U)
#define S5_PIVOT_KICK_MM_S (80)
#define S5_PIVOT_SLOW_DEG  (8.0f)
#define S5_PIVOT_DONE_DEG  (2.0f)
#define S5_PIVOT_MS        (4000U) /* longer: stop turning, sweep anyway */
/* backing up past the object just gone round: its outline checked this
 * often */
#define S5_BACK_STEP_MM (10.0f)
/* passing, the sensor looks across for the far corner;
 * this often it looks once along the pass lane instead (something there
 * that the object being passed cannot explain is another object). It
 * was 80, which let a box 40 mm behind this one, sticking into
 * the lane, slip between two looks */
#define S5_AHEAD_EVERY_MM (60.0f)
/* ... put off by up to this while the far corner is in view and not yet
 * fixed, so the look does not cost the fix (was 60) */
#define S5_AHEAD_HOLD_MM (30.0f)
/* ... and steeper than this to the line (on the ramp out), only an echo
 * from further on than the depth planned for counts (see s5_lane_counts()) */
#define S5_AIM_STEEP_DEG (20.0f)
/* ... and once the far corner is fitted, an echo from this much further
 * on is another object close behind (s5_far_behind()); the fit is good to
 * about 10 mm (was 80) */
#define S5_FAR_JUMP_MM (40.0f)
/* a look along the pass lane that heard
 * something maybe in it goes on through this many pings with no echo (a
 * dropout is not a clear lane) */
#define S5_MAYBE_GAPS (3U)
/* ... and other objects met in one bypass: one more is a fault */
#define S5_NEXT_MAX (4U)
/* handing back to Buddy 3. With one IR on our
 * line the car may be this far off its direction; with both IRs white an
 * IR must have seen our line within the last S5_FRESH_MM (the line may
 * bend after the object; dead reckoning alone cannot tell). */
#define S5_HANDOVER_DEG (10.0f)
#define S5_FRESH_MM     (200.0f)
/* picking the line up with the IRs on the way back. Both
 * IRs black together, or one and then the other within this much driving
 * (about 1.5 s at the bypass speeds), near where the line should be: the
 * car is crossing our line. It drives on until the axle is on the line,
 * then turns on the spot to face along it, stopping when the IRs have the
 * line between them. Dead reckoning only has to hold for the last
 * S5_IR_AHEAD_MM. */
#define S5_CAP_PAIR_MM  (150.0f)
#define S5_CAP_MIN_DEG  (3.0f)   /* heading across the line, at least */
#define S5_CAP_REACH_MM (250.0f) /* drive on at most this, then turn */
#define S5_CAP_OVER_DEG (25.0f)  /* turn at most this far past straight */
#define S5_CAP_MAX_DEG  (70.0f)  /* the IRs' timing: no steeper than this */
/* after the IRs crossed a line that is not ours (see
 * S5_CAP_LINE_DEG), both IRs white this far before they look again */
#define S5_CAP_REARM_MM (20.0f)
/* turning onto the line, an IR black counts only this
 * near where the line is (tape 9 mm either side, plus the encoders' error
 * over the turn) */
#define S5_CAP_GATE_MM (30.0f)
/* turning to the line's direction from where the two
 * IRs met it, the IRs count only this near straight (a junction's branch
 * beside the line goes under them earlier in the turn), and the turn
 * stops this far past straight if they have not ended it (1 deg: the
 * wheels run on about 3 deg more after it; 3 deg here and a 45 deg
 * branch at the handover took the car in the simulator, C56) */
#define S5_CAP_IR_DEG   (6.0f)
#ifndef S5_CAP_PAST_DEG
#define S5_CAP_PAST_DEG (1.0f)
#endif
/* the wheels run on after the drive onto the line
 * stops (13 mm in the simulator, the axle 8 mm past the line), so that
 * drive stops this much short of the line, along the way the car goes,
 * and "straight" for the turn is pointing the IRs back at the line from
 * where the axle ended up, up to S5_CAP_AIM_DEG */
#ifndef S5_CAP_COAST_MM
#define S5_CAP_COAST_MM (15.0f)
#endif
#ifndef S5_CAP_AIM_DEG
#define S5_CAP_AIM_DEG  (10.0f)
#endif
/* turning onto the line after a bypass, the IRs decide
 * (as in a line search): they count from this far before the direction
 * the crossing gave, and the turn stops this far past it. The IRs, 125 mm
 * ahead of the axle, meet a bend where its chord runs, asin(125 / 2R) off
 * that direction (R300: 12 deg, R200: 18 deg); a junction's branch at
 * 45 deg or more stays outside */
#ifndef S5_CAP_BEND_DEG
#define S5_CAP_BEND_DEG (25.0f)
#endif
/* after the IR on the far side has crossed the tape: a little more, to
 * centre the tape between the IRs */
#define S5_CAP_EXTRA_DEG                                                      \
    ((((float)(S5_IR_HALF_GAP_MM - S5_LINE_HALF_MM)) / (float)S5_IR_AHEAD_MM) \
     * S5_RAD2DEG_F)
/* a far corner fit counts only when the
 * pings saw it from bearings this far apart and it lies this far towards
 * the object from the sensor's track */
#define S5_FAR_SPREAD_DEG (8.0f)
#define S5_FAR_INSIDE_MM  (40.0f)
/* a corner fitted further inside the sweep's
 * edge than this is not this object's far corner on the passing side (a
 * box turned 30 deg puts it 80 mm in) but another object's, further
 * across: not used */
#define S5_FAR_INWARD_MM (100.0f)

typedef enum
{
    S5_PH_IDLE = 0, /* line following, watching */
    S5_PH_STOP,     /* standing still, then g.then */
    S5_PH_BACK,     /* backing up to g.back_x */
    S5_PH_TURN,     /* turning on the spot to the line's direction */
    S5_PH_SCAN,     /* sweeping */
    S5_PH_BYPASS,   /* on the path */
    S5_PH_SEEK,     /* path done, line not met yet */
    S5_PH_ALIGN,    /* settling on the line */
    S5_PH_CAPTURE,  /* the IRs crossed our line: onto it, face along it */
    S5_PH_BLOCKED,  /* no way round */
    S5_PH_SEARCH,   /* line search, straight on */
    S5_PH_OUT       /* turning out on the spot */
} s5_phase_t;

typedef enum
{
    S5_THEN_FACE = 0, /* look how close it is: back off or sweep */
    S5_THEN_SCAN,
    S5_THEN_BYPASS
} s5_then_t;

typedef enum
{
    S5_PICK_BLOCKED = 0,
    S5_PICK_LEFT,
    S5_PICK_RIGHT,
    S5_PICK_RESCAN
} s5_pick_t;

/* what the IRs had made of the line before a crossing,
 * put back when that crossing turns out to be a junction's branch */
typedef struct
{
    bool b_line;
    float line_y;
    float y_end;
    float line_seen;
    float line_win;
    bool b_ret_black;
} s5_line_snap_t;

typedef struct
{
    s5_phase_t ph;
    s5_then_t then;
    s5_odom_t odo;
    /* the IR pair's track while Buddy 3 follows (odometry frame) */
    float hx[S5_HIST_LEN];
    float hy[S5_HIST_LEN];
    float hpsi[S5_HIST_LEN]; /* the car's heading there */
    uint32_t h_n;
    uint32_t h_head;
    float h_dist;
    /* the latest ping read */
    s5_ping_t ping;
    bool b_ping;
    uint32_t used_ping; /* last ping used by the bypass */
    int32_t look_ddeg;  /* last LOOK asked for */
    /* episode */
    uint32_t ph_ms;
    uint32_t ep_ms;
    uint32_t still_ms;
    float ep_dist;
    float back_x;
    float back_from;
    s5_then_t back_then;
    float reversed;
    uint32_t rescans;
    uint32_t scan_seq;
    bool b_scan_early; /* scan_seq started at detection, car braking */
    s5_sweep_t sweep;
    s5_profile_t prof;
    int32_t side; /* +1 left, -1 right */
    s5_box_t box;
    s5_path_t path;
    /* the far corner: depth, and the side as measured passing it */
    float side_moved;       /* moved out by this since the sweep */
    float far_px[S5_FAR_N]; /* sensor where each ping was fired */
    float far_py[S5_FAR_N];
    float far_pr[S5_FAR_N]; /* range */
    uint32_t far_n;
    uint32_t far_head;
    bool b_depth;
    bool b_far_fit;      /* a far corner fit has been used */
    bool b_back_said;    /* the way back has started (logged) */
    float far_d;         /* the last fit's side error (-1: none yet) */
    float far_x;         /* ... and its corner x */
    float far_lo;        /* the last ping's "something at least this far" */
    float far_floor;     /* x_far never shallower than this (s5_far_behind) */
    float far_own;       /* ... and this object's own far end, as fitted */
    float own_min;       /* this object's own far corner fits: shallowest */
    float own_max;       /* ... and deepest (before anything behind it) */
    bool b_behind;       /* another object close behind: passing both */
    float ahead_at;      /* odometer: next look along the pass lane */
    bool b_ahead;        /* looking along it now */
    bool b_far_seen;     /* the last look across answered from the far corner */
    uint32_t maybe_n;    /* no-echo pings the look ahead still waits through */
    const char *p_where; /* where another object was met */
    /* the line on the way back */
    float line_y;
    float line_est;  /* where the line is (0, or where the IRs found it) */
    float line_seen; /* odometer when an IR last saw our line */
    bool b_line;
    bool b_ir_black[2];
    bool b_ir_ok[2];
    float ir_entry[2];
    float seek_from;
    float align_from;
    /* picking the line up with the IRs */
    int32_t cap_first;   /* IR that went black first (0 L, 1 R), -1 none */
    float cap_first_at;  /* ... odometer then */
    int32_t cap_dir;     /* the car crosses the line to +1 left, -1 right */
    uint32_t cap_step;   /* 0 drive on, 3 stop, 1 turn, 2 the last bit,
                          * 4 stop before Buddy 3 */
    const char *p_cap_why; /* ... why the turn ended */
    float cap_from;      /* odometer when the pick-up began */
    float cap_psi;       /* heading when the last bit of the turn began */
    bool b_cap_seen;     /* turning: the far-side IR has seen the tape */
    bool b_cap_wide;     /* ... both IRs black at once
                          * (a junction): turn by the crossing's angle */
    bool b_cap_back;     /* ... turning towards an IR black at the start */
    bool b_out_pivot;    /* the plan turns out on the
                          * spot (too close for a ramp) ... */
    float out_th;        /* ... to this heading (+ = left) ... */
    uint32_t out_step;   /* ... 0 turn, 1 stand, 2 drive out, 3 stand,
                          * 4 turn back, 5 stand */
    float out_from;      /* ... odometer where the drive out began */
    float cap_psi0;      /* ... heading then */
    float cap_x1;        /* where the first IR met the */
    float cap_y1;        /* tape (encoders' frame) */
    float cap_phi;       /* the crossed line's direction, encoders' frame */
    bool b_cap_phi;      /* ... known (one IR and then the other) */
    bool b_cap_skip;     /* the IRs are on a line that is not ours */
    float cap_skip_at;   /* ... odometer when an IR was last black on it */
    s5_line_snap_t cap_snap; /* the line as it was before this crossing */
    /* the compass (S5_MAG_*) */
    bool b_mag;          /* its heading of the line is known: in use */
    bool b_mag_fixed;    /* ... taken, or not, for this episode */
    float mag_s;         /* standing at the stop: sums of sin and cos of */
    float mag_c;         /* (compass - encoders) */
    uint32_t mag_n;
    int32_t mag_pct;     /* the field's strength, % of calibration */
    float mag_off;       /* the compass's heading of the line */
    float mag_corr;      /* taken out of the encoders' heading in all */
    float mag_err;       /* compass minus encoders, last sample */
    float mag_dist;      /* odometer and heading after the last pass */
    float mag_psi;
    uint32_t mag_skip;   /* samples not used (field off, too far apart) */
    int32_t cap_second;  /* the IR that went black second */
    float line_win;      /* an IR black this near line_y is our line */
    bool b_ret_black;    /* an IR saw black on the way back */
    /* Buddy 5, 2026-11-16 */
    bool b_seek_ours;    /* looking on with no black seen on the way back:
                          * the first line met is ours, whatever direction
                          * the encoders give it */
    uint8_t seek_stage;  /* 0: at S5_SEEK_DEG, 1: at S5_SEEK2_DEG */
    /* an episode begun on the way back from the last one */
    bool b_rejoin;   /* off the line: go back to it if nothing is in the way */
    bool b_passed;   /* the car must not back into ... */
    s5_box_t passed; /* ... the object it has just gone round */
    bool b_passed_fit;   /* its far corner was fitted, from ... */
    float passed_fit_lo; /* ... here (x) */
    float passed_fit_hi; /* ... to here */
    uint32_t n_next;     /* other objects met in this bypass */
    bool b_turn_check;   /* this sweep decides: turn on the spot? */
    /* wheels */
    int16_t cmd_l;
    int16_t cmd_r;
    int16_t prev_l;
    int16_t prev_r;
    uint32_t dead_l_ms;
    uint32_t dead_r_ms;
    uint32_t wheel_tries; /* stalled wheel: stops and goes again, per episode */
    uint32_t rest_ms;     /* ... standing still that much longer */
    uint32_t dt_ms;       /* this pass's time step */
    int32_t last_tl;
    int32_t last_tr;
    /* no way round */
    bool b_blocked;
    bool b_block_said;
    uint32_t clear_ms;
    uint32_t block_ms;
    /* misc */
    uint32_t rand;
    uint32_t init_us;
    bool b_fault_said;
    /* line search (s5_search_begin()) */
    bool b_search;       /* this episode is a line search */
    bool b_search_first; /* ... its first pass */
    bool b_search_held;  /* ... standing: something ahead */
    float search_from;   /* odometer when it began */
    float search_on[2];  /* odometer when each IR last went black */
} s5_av_t;

static s5_av_t g;

/* ===================================================================== */
/*  Small helpers                                                         */
/* ===================================================================== */

static uint32_t s5_rand(void)
{
    uint32_t x = g.rand;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g.rand = (0U != x) ? x : 0x2545F491U;
    return g.rand;
}

static int32_t s5_mm(float v)
{
    return s5_roundf(v);
}

static int32_t s5_deg(float rad)
{
    return s5_roundf(rad * S5_RAD2DEG_F);
}

static const char *s5_side_name(int32_t side)
{
    return (side > 0) ? "LEFT" : "RIGHT";
}

/* The latest ping (the last one read, if this read collided). */
static bool s5_get_ping(s5_ping_t *p_out)
{
    s5_ping_t p;

    if (s5_sensor_ping(&p))
    {
        g.ping = p;
        g.b_ping = true;
    }
    *p_out = g.ping;
    return g.b_ping;
}

static bool s5_fresh(const s5_ping_t *p_p)
{
    return (s5_hw_us() - p_p->fired_us) <= S5_STALE_US;
}

/* A new ping since the bypass last used one, fired within 5 deg of the
 * angle it asked for. */
static bool s5_new_ping(s5_ping_t *p_out, int32_t want_ddeg)
{
    int32_t d;

    if (!s5_get_ping(p_out) || (p_out->seq == g.used_ping))
    {
        return false;
    }
    g.used_ping = p_out->seq;
    d = (int32_t)p_out->ddeg - want_ddeg;
    return p_out->b_valid && (d <= S5_LOOK_TOL_DDEG) &&
           (d >= -S5_LOOK_TOL_DDEG) && s5_fresh(p_out);
}

static void s5_look(int32_t ddeg)
{
    int32_t a = ddeg;

    if (a > S5_SERVO_LIMIT_DDEG)
    {
        a = S5_SERVO_LIMIT_DDEG;
    }
    if (a < -S5_SERVO_LIMIT_R_DDEG)
    {
        a = -S5_SERVO_LIMIT_R_DDEG;
    }
    g.look_ddeg = a;
    s5_sensor_look(a);
}

static void s5_mag_fix(void);

static void s5_enter(s5_phase_t ph)
{
    if ((S5_PH_STOP == g.ph) && (S5_PH_STOP != ph) && !g.b_mag_fixed)
    {
        s5_mag_fix(); /* the car is about to move */
    }
    if ((S5_PH_STOP != ph) && (S5_PH_SCAN != ph))
    {
        g.b_scan_early = false; /* the car moves first: sweep again */
    }
    g.ph = ph;
    g.ph_ms = 0U;
    g.still_ms = 0U;
}

static float s5_edge(void)
{
    return (g.side > 0) ? g.box.y_left : g.box.y_right;
}

/* ===================================================================== */
/*  Line following: where the line runs                                   */
/* ===================================================================== */

void s5_avoid_init(void)
{
    g = (s5_av_t){ 0 };
    s5_odom_init(&g.odo);
    g.init_us = s5_hw_us();
    g.rand = g.init_us | 1U;
    s5_sensor_guard();
}

void s5_avoid_track(const motion_status_t *p_m, bool b_following)
{
    s5_odom_update(&g.odo, p_m->ticks_l, p_m->ticks_r);
    if (!b_following)
    {
        g.h_n = 0U;
        return;
    }
    if ((0U == g.h_n) || ((g.odo.dist - g.h_dist) >= (float)S5_HIST_STEP_MM))
    {
        float x;
        float y;

        s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, 0.0f, &x, &y);
        g.hx[g.h_head] = x;
        g.hy[g.h_head] = y;
        g.hpsi[g.h_head] = g.odo.psi;
        g.h_head = (g.h_head + 1U) % S5_HIST_LEN;
        if (g.h_n < S5_HIST_LEN)
        {
            g.h_n++;
        }
        g.h_dist = g.odo.dist;
    }
}

/* Principal-axis fit through the newest m points of the IR pair's
 * track: mean (mx, my), direction th (the way the car drives) and the
 * furthest point from the fit. False: they spread too little. */
static bool s5_fit_newest(uint32_t m, float *p_mx, float *p_my, float *p_th,
                          float *p_worst)
{
    const float n = (float)m;
    float mx = 0.0f;
    float my = 0.0f;
    float sxx = 0.0f;
    float syy = 0.0f;
    float sxy = 0.0f;
    float th;
    float worst = 0.0f;
    uint32_t i;

    for (i = 0U; i < m; i++)
    {
        const uint32_t k = (g.h_head + S5_HIST_LEN - 1U - i) % S5_HIST_LEN;

        mx += g.hx[k];
        my += g.hy[k];
    }
    mx /= n;
    my /= n;
    for (i = 0U; i < m; i++)
    {
        const uint32_t k = (g.h_head + S5_HIST_LEN - 1U - i) % S5_HIST_LEN;
        const float dx = g.hx[k] - mx;
        const float dy = g.hy[k] - my;

        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    if (((sxx + syy) / n) < S5_FIT_SPREAD)
    {
        return false;
    }
    th = 0.5f * s5_atan2f(2.0f * sxy, sxx - syy);
    if (s5_cosf(th - g.odo.psi) < 0.0f)
    {
        th = s5_wrapf(th + S5_PI_F); /* the way the car drives */
    }
    for (i = 0U; i < m; i++)
    {
        const uint32_t k = (g.h_head + S5_HIST_LEN - 1U - i) % S5_HIST_LEN;
        const float off = s5_absf(((g.hy[k] - my) * s5_cosf(th)) -
                                  ((g.hx[k] - mx) * s5_sinf(th)));

        worst = (off > worst) ? off : worst;
    }
    *p_mx = mx;
    *p_my = my;
    *p_th = th;
    *p_worst = worst;
    return true;
}

/* The line: a straight fit through the IR pair's track (its direction
 * by principal axes), through the IR pair now. If the track bends (a
 * point further than S5_FIT_RESID_MM off the fit, or the car pointed
 * more than S5_FIT_TURN_DEG off it at the oldest point), the oldest
 * points are left out until it is straight, so a bend just before the
 * object does not tilt the line. Too short a track: the car's heading. */
static void s5_fit_line(float *p_ox, float *p_oy, float *p_th, uint32_t *p_n,
                        float *p_worst)
{
    float irx;
    float iry;
    uint32_t m = g.h_n;

    s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, 0.0f, &irx, &iry);
    *p_ox = irx;
    *p_oy = iry;
    *p_th = g.odo.psi;
    *p_n = 0U;
    *p_worst = 0.0f;
    while (m >= S5_HIST_MIN)
    {
        float mx;
        float my;
        float th;
        float worst;
        bool b_bend;

        if (!s5_fit_newest(m, &mx, &my, &th, &worst))
        {
            break;
        }
        {
            /* the oldest point used */
            const uint32_t k = (g.h_head + S5_HIST_LEN - m) % S5_HIST_LEN;

            b_bend = (worst > S5_FIT_RESID_MM) ||
                     (s5_absf(s5_wrapf(g.hpsi[k] - th)) >
                      (S5_FIT_TURN_DEG * S5_DEG2RAD_F));
        }
        if (!b_bend || (m < (S5_HIST_MIN + 2U)))
        {
            const float ux = s5_cosf(th);
            const float uy = s5_sinf(th);
            const float t = ((irx - mx) * ux) + ((iry - my) * uy);

            *p_ox = mx + (t * ux);
            *p_oy = my + (t * uy);
            *p_th = th;
            *p_n = m;
            *p_worst = worst;
            break;
        }
        /* the line bends: leave out the oldest (an eighth at a time) */
        m -= ((m / 8U) > 2U) ? (m / 8U) : 2U;
    }
}

bool s5_avoid_check(bool b_armed)
{
#if S5_CAL_BUILD
    (void)b_armed;
    return false; /* the calibration drives the sensor itself */
#else
    s5_ping_t p;

    if (!b_armed || (S5_PH_IDLE != g.ph) || !s5_get_ping(&p))
    {
        return false;
    }
    return p.b_close && (0 == p.ddeg) && s5_fresh(&p);
#endif
}

bool s5_avoid_sensor_fault(void)
{
    const char *p_why = (const char *)0;
    s5_ping_t p;

    if (s5_sensor_dead())
    {
        p_why = "the HC-SR04 gives no echo pulse at all (5 V, GND, TRIG GP16,"
                " ECHO GP17 via the level shifter?)";
    }
    else if (s5_get_ping(&p))
    {
        if ((s5_hw_us() - p.fired_us) > S5_SILENT_US)
        {
            p_why = "no new ping for 1 s (S5 task stuck?)";
        }
    }
    else if ((s5_hw_us() - g.init_us) > S5_SILENT_US)
    {
        p_why = "no ping at all since start (S5 task not running?)";
    }
    else
    {
        /* starting up */
    }
    if ((const char *)0 == p_why)
    {
        g.b_fault_said = false;
        return false;
    }
    if (!g.b_fault_said)
    {
        g.b_fault_said = true;
        S5_LOG("[VC-S5] sensor fault (FR25): %s\n", p_why);
    }
    return true;
}

/* ===================================================================== */
/*  Compass                                             */
/* ===================================================================== */

/* The compass's heading of the car (rad, + = left), false if it is not
 * calibrated, has no fresh sample, or the field's strength is off (iron
 * nearby bends it too). */
static bool s5_mag_heading(float *p_h, int32_t *p_pct)
{
    int32_t x;
    int32_t y;
    float dx;
    float dy;
    float r;

    if ((0 == S5_MAG_SIGN) || (S5_MAG_R <= 0) || !s5_hw_mag(&x, &y))
    {
        return false;
    }
    dx = (float)(x - (int32_t)S5_MAG_X0);
    dy = (float)(y - (int32_t)S5_MAG_Y0);
    r = s5_sqrtf((dx * dx) + (dy * dy));
    *p_pct = s5_roundf((100.0f * r) / (float)S5_MAG_R);
    if (s5_absf(r - (float)S5_MAG_R) >
        (((float)S5_MAG_R * (float)S5_MAG_TOL_PCT) / 100.0f))
    {
        return false;
    }
    *p_h = (float)S5_MAG_SIGN * s5_atan2f(dy, dx);
    return true;
}

/* Leaving the first stop of an episode: the compass's heading of the line
 * from the samples taken standing there. */
static void s5_mag_fix(void)
{
    static bool b_said;

    g.b_mag_fixed = true;
    if (0 == S5_MAG_SIGN)
    {
        if (!b_said)
        {
            b_said = true;
            S5_LOG("[VC-S5] compass not calibrated (S5_MAG_SIGN 0): "
                   "encoders only; calibration step 4 (make S5CAL=2) "
                   "gives the numbers\n");
        }
    }
    else if (g.mag_n >= 5U)
    {
        g.mag_off = s5_atan2f(g.mag_s, g.mag_c);
        g.b_mag = true;
        g.mag_dist = g.odo.dist;
        g.mag_psi = g.odo.psi;
        S5_LOG("[VC-S5] compass: heading taken at the stop (%d samples, "
               "field %d percent of the calibration's)%s\n",
               (int)g.mag_n, (int)g.mag_pct,
               (0 != S5_MAG_USE) ? "" : "; logged only (S5_MAG_USE 0)");
    }
    else
    {
        S5_LOG("[VC-S5] compass: no good sample at the stop (field %d percent "
               "of the calibration's): encoders only\n",
               (int)g.mag_pct);
    }
}

/* Every pass of an episode: standing at the first stop, collect; after
 * that, pull the encoders' heading towards the compass's. */
static void s5_mag_step(uint32_t dt)
{
    float h;
    int32_t pct = 0;

    if (S5_PH_IDLE == g.ph)
    {
        return;
    }
    if (!s5_mag_heading(&h, &pct))
    {
        if (0 != pct)
        {
            g.mag_pct = pct;
        }
        if (g.b_mag)
        {
            g.mag_skip++;
        }
        return;
    }
    if (!g.b_mag_fixed)
    {
        const float d = s5_wrapf(h - g.odo.psi);

        g.mag_s += s5_sinf(d);
        g.mag_c += s5_cosf(d);
        g.mag_n++;
        g.mag_pct = pct;
        return;
    }
    if (g.b_mag)
    {
        const float err = s5_wrapf((h - g.mag_off) - g.odo.psi);
        /* how far the wheels went since the last pass, turning on the
         * spot included: the encoders drift only with that, so the
         * compass may pull at most S5_MAG_RATE_DEG per 100 mm of it (iron
         * the car drives past bends the field slowly too) */
        const float went =
            (g.odo.dist - g.mag_dist) +
            (s5_absf(s5_wrapf(g.odo.psi - g.mag_psi)) * S5_MAG_ARM_MM);
        const float most =
            (went * (float)S5_MAG_RATE_DEG * S5_DEG2RAD_F) / 100.0f;
        float k = (float)dt / (float)S5_MAG_TAU_MS;
        float c;

        g.mag_dist = g.odo.dist;
        g.mag_psi = g.odo.psi;
        if (s5_absf(err) > ((float)S5_MAG_MAX_DEG * S5_DEG2RAD_F))
        {
            g.mag_skip++;
            return;
        }
        g.mag_err = err;
        if (s5_absf(err) <= ((float)S5_MAG_DEAD_DEG * S5_DEG2RAD_F))
        {
            return;
        }
        k = (k > 1.0f) ? 1.0f : k;
        c = k * ((err > 0.0f)
                     ? (err - ((float)S5_MAG_DEAD_DEG * S5_DEG2RAD_F))
                     : (err + ((float)S5_MAG_DEAD_DEG * S5_DEG2RAD_F)));
        c = (c > most) ? most : ((c < -most) ? -most : c);
        if (0 != S5_MAG_USE)
        {
            s5_odom_set_pose(&g.odo, g.odo.x, g.odo.y, g.odo.psi + c);
            g.mag_psi = g.odo.psi;
            g.mag_corr += c;
        }
    }
}

/* One log line: what the compass did so far. */
static void s5_mag_say(void)
{
    if ((0 != S5_MAG_SIGN) && g.b_mag_fixed && (g.mag_n >= 5U))
    {
        S5_LOG("[VC-S5] compass: %d deg taken out of the encoders' heading "
               "so far, now %d deg from it (%d samples skipped)\n",
               (int)s5_deg(-g.mag_corr), (int)s5_deg(g.mag_err),
               (int)g.mag_skip);
    }
}

/* ===================================================================== */
/*  Episode                                                               */
/* ===================================================================== */

void s5_avoid_begin(const motion_status_t *p_m)
{
    float ox;
    float oy;
    float th;
    float worst;
    uint32_t n;
    s5_ping_t p;

    s5_odom_update(&g.odo, p_m->ticks_l, p_m->ticks_r);
    s5_fit_line(&ox, &oy, &th, &n, &worst);
    {
        /* the pose, now in the line frame */
        const float dx = g.odo.x - ox;
        const float dy = g.odo.y - oy;
        const float c = s5_cosf(th);
        const float s = s5_sinf(th);

        s5_odom_set_pose(&g.odo, (dx * c) + (dy * s), (dy * c) - (dx * s),
                         g.odo.psi - th);
    }
    g.h_n = 0U;
    g.ep_ms = 0U;
    g.ep_dist = g.odo.dist;
    g.b_mag = false;
    g.b_mag_fixed = false;
    g.mag_s = 0.0f;
    g.mag_c = 0.0f;
    g.mag_n = 0U;
    g.mag_pct = 0;
    g.mag_corr = 0.0f;
    g.mag_err = 0.0f;
    g.mag_skip = 0U;
    g.reversed = 0.0f;
    g.rescans = 0U;
    g.b_blocked = false;
    g.b_block_said = false;
    g.block_ms = 0U;
    g.line_est = 0.0f;
    g.b_rejoin = false;
    g.b_passed = false;
    g.n_next = 0U;
    g.b_turn_check = false;
    g.prev_l = 0;
    g.prev_r = 0;
    g.dead_l_ms = 0U;
    g.dead_r_ms = 0U;
    g.wheel_tries = 0U;
    g.rest_ms = 0U;
    g.last_tl = p_m->ticks_l;
    g.last_tr = p_m->ticks_r;
    g.rand ^= s5_hw_us();
    g.b_search = false;
    g.then = S5_THEN_FACE;
    s5_enter(S5_PH_STOP);
    (void)s5_get_ping(&p);
    /* NFR9/NFR11: the sweep starts now, while the car brakes (it rolls
     * on about 20 mm, and the first bearings are the far left ones) */
    g.b_scan_early = (0 != S5_SCAN_WHILE_STOPPING);
    if (g.b_scan_early)
    {
        g.scan_seq = s5_sensor_scan();
    }
    else
    {
        s5_look(S5_SERVO_LIMIT_DDEG);
    }
    S5_LOG("[VC-S5] object %d mm ahead: stopping (line from the last %d "
           "mm, within %d mm; car %d deg to it)\n",
           (int)p.mm, (int)(n * (uint32_t)S5_HIST_STEP_MM), (int)s5_mm(worst),
           (int)s5_deg(g.odo.psi));
}

void s5_avoid_abort(void)
{
    if (S5_PH_IDLE != g.ph)
    {
        S5_LOG("[VC-S5] %s stopped by the VehicleController\n",
               g.b_search ? "line search" : "bypass");
    }
    g.ph = S5_PH_IDLE;
    g.b_blocked = false;
    s5_sensor_guard();
}

bool s5_avoid_blocked(void)
{
    return g.b_blocked;
}

static void s5_mag_say(void);

static s5_avoid_result_t s5_fail(const char *p_why)
{
    s5_mag_say();
    S5_LOG("[VC-S5] %s failed: %s\n", g.b_search ? "line search" : "bypass",
           p_why);
    g.ph = S5_PH_IDLE;
    g.b_blocked = false;
    g.cmd_l = 0;
    g.cmd_r = 0;
    s5_sensor_guard();
    return S5_AVOID_FAULT;
}

static s5_avoid_result_t s5_done(const char *p_why)
{
    s5_mag_say();
    S5_LOG("[VC-S5] %s: line following again (%d mm, %d.%d s)\n", p_why,
           (int)s5_mm(g.odo.dist - g.ep_dist), (int)(g.ep_ms / 1000U),
           (int)((g.ep_ms % 1000U) / 100U));
    g.ph = S5_PH_IDLE;
    g.b_blocked = false;
    g.cmd_l = 0;
    g.cmd_r = 0;
    g.h_n = 0U;
    s5_sensor_guard();
    return S5_AVOID_DONE;
}

static void s5_block(const char *p_why)
{
    if (!g.b_block_said)
    {
        g.b_block_said = true;
        g.block_ms = 0U;
        S5_LOG("[VC-S5] no way round (%s): waiting here with the obstacle "
               "fault bit 0x40 set, no E-stop\n",
               p_why);
    }
    else
    {
        /* said again after every look, for
         * whoever starts watching later */
        S5_LOG("[VC-S5] still no way round after %d s: fault bit 0x40, no "
               "E-stop\n",
               (int)(g.block_ms / 1000U));
    }
    g.b_blocked = true;
    g.clear_ms = 0U;
    g.ep_ms = 0U; /* the 60 s start again when a way opens */
    s5_sensor_guard();
    s5_enter(S5_PH_BLOCKED);
}

static void s5_scan_start(void)
{
    if (!g.b_scan_early)
    {
        g.scan_seq = s5_sensor_scan();
    }
    g.b_scan_early = false;
    s5_enter(S5_PH_SCAN);
}

/* Would the car come within S5_RAMP_MARGIN_MM of the object it has just
 * gone round if it backed up d along its heading? */
static bool s5_back_hits(float d)
{
    return g.b_passed &&
           s5_path_car_hits(g.odo.x - (d * s5_cosf(g.odo.psi)),
                            g.odo.y - (d * s5_sinf(g.odo.psi)), g.odo.psi,
                            &g.passed, (float)S5_RAMP_MARGIN_MM);
}

/* How far the car may still back up: the reverse limit, and never into
 * the object it has just gone round (checked every S5_BACK_STEP_MM). */
static float s5_back_room(void)
{
    const float left = (float)S5_REVERSE_MAX_MM - g.reversed;
    float d = 0.0f;

    if (!g.b_passed)
    {
        return left;
    }
    while (((d + S5_BACK_STEP_MM) <= left) &&
           !s5_back_hits(d + S5_BACK_STEP_MM))
    {
        d += S5_BACK_STEP_MM;
    }
    return d;
}

/* Back up by d (as far as s5_back_room() allows), stop, then 'then'. */
static void s5_back_by(float d, s5_then_t then)
{
    float dd = d;
    const float left = s5_back_room();

    if (dd > left)
    {
        dd = left;
    }
    g.back_then = then;
    if (dd < 5.0f)
    {
        g.then = then;
        s5_enter(S5_PH_STOP);
        return;
    }
    g.back_from = g.odo.x;
    g.back_x = g.odo.x - dd;
    s5_enter(S5_PH_BACK);
}

/* ===================================================================== */
/*  Planning                                                              */
/* ===================================================================== */

static bool s5_out_clear(float x0)
{
    s5_path_t p = g.path;

    p.x0 = x0;
    p.x_back = x0 + p.l_out + 100000.0f; /* no way back in this check */
    return s5_path_clear(&p, &g.box, (float)S5_RAMP_MARGIN_MM, x0,
                         x0 + p.l_out) &&
           (!g.b_passed ||
            s5_path_clear(&p, &g.passed, S5_PIVOT_MARGIN_MM, x0, x0 + p.l_out));
}

static bool s5_in_clear(float x_back)
{
    s5_path_t p = g.path;

    p.x_back = x_back;
    return s5_path_clear(&p, &g.box, (float)S5_RAMP_MARGIN_MM, x_back,
                         x_back + p.l_in) &&
           (!g.b_passed || s5_path_clear(&p, &g.passed, S5_PIVOT_MARGIN_MM,
                                         x_back, x_back + p.l_in));
}

/* The earliest the way back can start without touching the object, and
 * never behind the car (its axle at x_now). */
static void s5_plan_back(float x_now)
{
    const float far = (g.b_passed && (g.passed.x_far > g.box.x_far))
                          ? g.passed.x_far
                          : g.box.x_far;
    float lo = g.path.x0 + g.path.l_out;
    float hi = far + (float)(S5_RAMP_MARGIN_MM + S5_CAR_REAR_MM);
    uint32_t i;

    if (lo < (x_now + 1.0f))
    {
        lo = x_now + 1.0f;
    }
    if ((lo >= hi) || s5_in_clear(lo))
    {
        g.path.x_back = lo;
        return;
    }
    for (i = 0U; i < S5_BISECT; i++)
    {
        const float mid = 0.5f * (lo + hi);

        if (s5_in_clear(mid))
        {
            hi = mid;
        }
        else
        {
            lo = mid;
        }
    }
    g.path.x_back = hi;
}

/* the car's outline at (x, y), heading psi, clear of
 * the object (margin) and of the one just gone round? */
static bool s5_out_free(float x, float y, float psi, float margin)
{
    return !s5_path_car_hits(x, y, psi, &g.box, margin) &&
           (!g.b_passed ||
            !s5_path_car_hits(x, y, psi, &g.passed, S5_PIVOT_MARGIN_MM));
}

/* turning out on the spot to heading th (+ = left),
 * straight out until the axle is at the pass (g.path.a), turning back
 * on the spot there: clear of the object all the way? *p_x1: where the
 * axle reaches the pass. */
static bool s5_pivot_out_clear(float th, float *p_x1)
{
    const float x = g.odo.x;
    const float y = g.odo.y;
    const float run = s5_absf(g.path.a - y) / s5_absf(s5_sinf(th));
    const float x1 = x + (run * s5_cosf(th));
    const uint32_t n_turn =
        (uint32_t)(s5_absf(th) / (S5_PIVOT_STEP_DEG * S5_DEG2RAD_F)) + 1U;
    const uint32_t n_run = (uint32_t)(run / 10.0f) + 1U;
    uint32_t i;

    for (i = 0U; i <= n_turn; i++)
    {
        const float a = (th * (float)i) / (float)n_turn;

        if (!s5_out_free(x, y, a, S5_PIVOT_MARGIN_MM) ||
            !s5_out_free(x1, g.path.a, a, S5_PIVOT_MARGIN_MM))
        {
            return false;
        }
    }
    for (i = 0U; i <= n_run; i++)
    {
        const float d = (run * (float)i) / (float)n_run;

        if (!s5_out_free(x + (d * s5_cosf(th)), y + (d * s5_sinf(th)), th,
                         (float)S5_RAMP_MARGIN_MM))
        {
            return false;
        }
    }
    *p_x1 = x1;
    return true;
}

/* The path round on this side. False: not even backing up all the way
 * left gives room to turn out. */
static bool s5_plan(int32_t side)
{
    const float x_here = g.odo.x;
    const float budget = s5_back_room();
    float x0 = x_here;

    g.side = side;
    g.path.y0 = g.odo.y;
    g.path.a = s5_scan_offset(s5_edge(), side);
    if (((float)side * (g.odo.y - g.path.a)) > 0.0f)
    {
        /* already further out on that side (on the way back from the
         * last object): stay there, never turn in across the object's
         * front */
        g.path.a = g.odo.y;
    }
    g.path.l_out = s5_path_ramp_len(g.path.a - g.path.y0);
    g.path.y_end = g.line_est;
    /* back in more sharply (S5_IN_*: shorter, less for
     * the encoders to drift over; the IRs pick the line up) */
    g.path.l_in = s5_path_ramp_len_in(g.path.a - g.line_est);
    if (!s5_out_clear(x_here))
    {
        /* no room for the gentle ramp out from here: the
         * gentlest that fits, as steep as S5_STEEP_HEADING_DEG, before
         * backing up at all (from 30 cm the car should just go round) */
        const float l_soft = g.path.l_out;
        float lo_l = s5_path_ramp_len_steep(g.path.a - g.path.y0); /* fits? */
        float hi_l = l_soft;                                     /* no */
        uint32_t k;

        g.path.l_out = lo_l;
        if (s5_out_clear(x_here))
        {
            for (k = 0U; k < S5_BISECT; k++)
            {
                const float mid = 0.5f * (lo_l + hi_l);

                g.path.l_out = mid;
                if (s5_out_clear(x_here))
                {
                    lo_l = mid;
                }
                else
                {
                    hi_l = mid;
                }
            }
            g.path.l_out = lo_l;
        }
    }
    g.b_out_pivot = false;
    if (!s5_out_clear(x_here))
    {
        /* no backing up for room to turn out: turn out
         * on the spot, by as little as clears the object */
        int32_t deg;
        float x1 = x_here;

        for (deg = S5_OUT_PIVOT_MIN_DEG; (deg <= 90) && !g.b_out_pivot;
             deg += S5_OUT_PIVOT_STEP_DEG)
        {
            const float th = (float)side * (float)deg * S5_DEG2RAD_F;

            if (s5_pivot_out_clear(th, &x1))
            {
                g.b_out_pivot = true;
                g.out_th = th;
            }
        }
        if (g.b_out_pivot)
        {
            /* the rest of the path from where the turn back ends */
            g.path.x0 = x1;
            g.path.y0 = g.path.a;
            g.path.l_out = 1.0f;
            s5_plan_back(x1);
            return !g.b_passed ||
                   s5_path_clear(&g.path, &g.passed, S5_PIVOT_MARGIN_MM, x1,
                                 s5_path_end(&g.path));
        }
    }
    if (!s5_out_clear(x_here))
    {
        /* not even turning out on the spot: no way round from here, so
         * back up */
        float lo = x_here - budget; /* clear, if anything is */
        float hi = x_here;          /* not clear */
        uint32_t i;

        if ((budget <= 0.0f) || !s5_out_clear(lo))
        {
            return false;
        }
        for (i = 0U; i < S5_BISECT; i++)
        {
            const float mid = 0.5f * (lo + hi);

            if (s5_out_clear(mid))
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }
        x0 = lo;
    }
    g.path.x0 = x0;
    s5_plan_back(x0); /* the car will be at x0 when it sets off */
    /* the whole path, out, along and
     * back in, clear of the object just gone round too (the ramp out was
     * checked above; a path planned after backing up past it can still
     * run alongside or back across it) */
    return !g.b_passed || s5_path_clear(&g.path, &g.passed, S5_PIVOT_MARGIN_MM,
                                        x0, s5_path_end(&g.path));
}

/* b_maybe: a side the edge of an echo's beam reaches counts as blocked */
static s5_pick_t s5_pick_sides(bool b_maybe)
{
    const s5_profile_t *p = &g.prof;
    const bool b_blk_l = p->b_left_blocked || (b_maybe && p->b_left_maybe);
    const bool b_blk_r = p->b_right_blocked || (b_maybe && p->b_right_maybe);
    const float ext_l = p->y_left;   /* how far it reaches left */
    const float ext_r = -p->y_right; /* ... and right */
    const float room = (float)(S5_CAR_HALF_WIDTH_MM + S5_CLEARANCE_MM);
    /* an edge hidden by the object just gone round: not that way, and no
     * point looking again */
    const bool b_ok_l = !b_blk_l && !p->b_left_hidden &&
                        ((ext_l + room) <= (float)S5_MAX_OFFSET_MM);
    const bool b_ok_r = !b_blk_r && !p->b_right_hidden &&
                        ((ext_r + room) <= (float)S5_MAX_OFFSET_MM);
    const bool b_more_l = !p->b_left_hidden && (g.rescans < S5_MAX_RESCANS);
    const bool b_more_r = !p->b_right_hidden && (g.rescans < S5_MAX_RESCANS);
    /* the shorter way round. From the line that is the
     * side the object reaches less far to (cost: out and back, twice the
     * offset); a car still off the line after the last object (b_rejoin)
     * also counts the way across. An edge outside the sweep gives a lower
     * bound, so does its cost. */
    const float yc = g.b_rejoin ? (g.odo.y - g.line_est) : 0.0f;
    const float a_l = ext_l + room;
    const float a_r = ext_r + room;
    const float cost_l = s5_absf(a_l - yc) + a_l;
    const float cost_r = s5_absf(a_r + yc) + a_r;
    const float tie = 2.0f * (float)S5_TIE_MM;

    if (!p->b_left_seen && !p->b_right_seen)
    {
        return (b_more_l || b_more_r) ? S5_PICK_RESCAN : S5_PICK_BLOCKED;
    }
    /* no backing up to see the other edge while one side
     * is open (it may be the longer way round) */
    if (!p->b_left_seen)
    {
        if (b_ok_r)
        {
            return S5_PICK_RIGHT;
        }
        return b_more_l ? S5_PICK_RESCAN : S5_PICK_BLOCKED;
    }
    if (!p->b_right_seen)
    {
        if (b_ok_l)
        {
            return S5_PICK_LEFT;
        }
        return b_more_r ? S5_PICK_RESCAN : S5_PICK_BLOCKED;
    }
    if (b_ok_l && b_ok_r)
    {
        if (s5_absf(cost_l - cost_r) <= tie)
        {
            const bool b_left = (0U != (s5_rand() & 0x100U));

            S5_LOG("[VC-S5] both sides the same (%d / %d mm): %s at random\n",
                   (int)s5_mm(ext_l), (int)s5_mm(ext_r),
                   b_left ? "LEFT" : "RIGHT");
            return b_left ? S5_PICK_LEFT : S5_PICK_RIGHT;
        }
        return (cost_l < cost_r) ? S5_PICK_LEFT : S5_PICK_RIGHT;
    }
    if (b_ok_l)
    {
        return S5_PICK_LEFT;
    }
    if (b_ok_r)
    {
        return S5_PICK_RIGHT;
    }
    return S5_PICK_BLOCKED;
}

/* a side where only the edge of an echo's beam reaches
 * the pass lane is used when the other side is blocked outright (on the
 * track something 55-65 cm out beside the box ruled out both sides).
 * Both sides only maybe blocked (walls either side): wait as before. */
static s5_pick_t s5_pick(void)
{
    const s5_profile_t *p = &g.prof;
    s5_pick_t pick = s5_pick_sides(true);
    /* the car went the long way round the box all
     * day: the sonar hears the floor 54-71 cm out on every bearing the
     * box does not cover, and the edge of that echo's beam made the short
     * side "maybe blocked". At the first object (not after another one),
     * a side only maybe blocked by an echo at least S5_FLOOR_MIN_MM away
     * is taken when it is the shorter way by more than S5_TIE_MM and the
     * other side is clear. A nearer echo (a 2nd box close behind) still
     * rules it out; both sides maybe (walls, C10/C11) still waits. */
    const bool b_first = !g.b_passed && !g.b_rejoin;
    const float ext_l = p->y_left;
    const float ext_r = -p->y_right;
    const bool b_l_short = b_first && (S5_PICK_RIGHT == pick) &&
                           p->b_left_maybe && !p->b_left_blocked &&
                           !p->b_right_maybe &&
                           (p->block_l_mm >= (uint16_t)S5_FLOOR_MIN_MM) &&
                           ((ext_r - ext_l) > (float)S5_TIE_MM);
    const bool b_r_short = b_first && (S5_PICK_LEFT == pick) &&
                           p->b_right_maybe && !p->b_right_blocked &&
                           !p->b_left_maybe &&
                           (p->block_r_mm >= (uint16_t)S5_FLOOR_MIN_MM) &&
                           ((ext_l - ext_r) > (float)S5_TIE_MM);
    /* the same when the other edge is past the sweep (the box heard to
     * the sweep's end reaches at least that far round): no side counted
     * open, so the car backed off to look again from further away, where
     * the floor still blocked the short side. Only with the object's face
     * nearer than such echoes (else the "object" may be walls, C11). */
    const bool b_none =
        ((S5_PICK_RESCAN == pick) || (S5_PICK_BLOCKED == pick)) &&
        (p->face_mm < (uint16_t)S5_FLOOR_MIN_MM);
    const bool b_l_past = b_first && b_none && p->b_left_seen &&
                          !p->b_right_seen && p->b_left_maybe &&
                          !p->b_left_blocked &&
                          (p->block_l_mm >= (uint16_t)S5_FLOOR_MIN_MM) &&
                          ((ext_r - ext_l) > (float)S5_TIE_MM);
    const bool b_r_past = b_first && b_none && p->b_right_seen &&
                          !p->b_left_seen && p->b_right_maybe &&
                          !p->b_right_blocked &&
                          (p->block_r_mm >= (uint16_t)S5_FLOOR_MIN_MM) &&
                          ((ext_l - ext_r) > (float)S5_TIE_MM);

    if (b_l_past || b_r_past)
    {
        const s5_pick_t shorter = s5_pick_sides(false);

        if (((S5_PICK_LEFT == shorter) && b_l_past) ||
            ((S5_PICK_RIGHT == shorter) && b_r_past))
        {
            const bool b_left = (S5_PICK_LEFT == shorter);

            S5_LOG("[VC-S5] going %s anyway, the shorter way (%d / at least "
                   "%d mm): the other edge is past the sweep, only a far "
                   "echo (floor?) at this side's beam edge\n",
                   b_left ? "LEFT" : "RIGHT",
                   (int)s5_mm(b_left ? ext_l : ext_r),
                   (int)s5_mm(b_left ? ext_r : ext_l));
            pick = shorter;
        }
    }
    if (b_l_short || b_r_short)
    {
        const s5_pick_t shorter = s5_pick_sides(false);

        if (((S5_PICK_LEFT == shorter) && b_l_short) ||
            ((S5_PICK_RIGHT == shorter) && b_r_short))
        {
            S5_LOG("[VC-S5] going %s anyway, the shorter way (%d / %d mm): "
                   "only a far echo (floor?) at its beam edge\n",
                   (S5_PICK_LEFT == shorter) ? "LEFT" : "RIGHT",
                   (int)s5_mm(ext_l), (int)s5_mm(ext_r));
            pick = shorter;
        }
    }
    if ((S5_PICK_BLOCKED == pick) &&
        ((p->b_left_maybe && p->b_right_blocked) ||
         (p->b_right_maybe && p->b_left_blocked)))
    {
        pick = s5_pick_sides(false);
        if ((S5_PICK_LEFT == pick) || (S5_PICK_RIGHT == pick))
        {
            S5_LOG("[VC-S5] no side clear: going %s, where an echo is in "
                   "the pass lane only at the edge of its beam; watching "
                   "the lane\n",
                   (S5_PICK_LEFT == pick) ? "LEFT" : "RIGHT");
        }
    }
    return pick;
}

/* append v as decimal text at p_buf[at], return the new length (stops
 * short of max, which leaves room for the terminating NUL) */
static uint32_t s5_put_int(char *p_buf, uint32_t at, uint32_t max, int32_t v)
{
    char digits[11];
    uint32_t n = 0U;
    uint32_t u = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;

    if ((v < 0) && ((at + 1U) < max))
    {
        p_buf[at] = '-';
        at++;
    }
    do
    {
        digits[n] = (char)('0' + (char)(u % 10U));
        n++;
        u /= 10U;
    } while ((u > 0U) && (n < sizeof(digits)));
    while ((n > 0U) && ((at + 1U) < max))
    {
        n--;
        p_buf[at] = digits[n];
        at++;
    }
    return at;
}

/* every ping of the sweep, right to left, as deg:mm (0: no echo, x: left
 * out, the last object is that way), S5_PINGS_PER_LINE to a line */
#define S5_PINGS_PER_LINE (13U)

static void s5_say_pings(void)
{
    char text[S5_PINGS_PER_LINE * 10U];
    uint32_t first;

    for (first = 0U; first < g.sweep.n; first += S5_PINGS_PER_LINE)
    {
        uint32_t i;
        uint32_t at = 0U;
        const uint32_t last = ((first + S5_PINGS_PER_LINE) < g.sweep.n)
                                  ? (first + S5_PINGS_PER_LINE)
                                  : g.sweep.n;

        for (i = first; i < last; i++)
        {
            const s5_sample_t *p_s = &g.sweep.s[i];

            at = s5_put_int(text, at, sizeof(text),
                            (int32_t)p_s->ddeg / 10);
            if ((at + 3U) < sizeof(text))
            {
                text[at] = ':';
                at++;
                if (S5_MM_UNKNOWN == p_s->mm)
                {
                    text[at] = 'x';
                    at++;
                }
                else
                {
                    at = s5_put_int(text, at, sizeof(text),
                                    (int32_t)p_s->mm);
                }
            }
            if ((at + 1U) < sizeof(text))
            {
                text[at] = ' ';
                at++;
            }
        }
        if ((at > 0U) && (' ' == text[at - 1U]))
        {
            at--;
        }
        text[at] = '\0';
        S5_LOG("[VC-S5] pings %d-%d of %d (deg:mm): %s\n", (int)first + 1,
               (int)last, (int)g.sweep.n, text);
    }
}

static void s5_say_sweep(void)
{
    const s5_profile_t *p = &g.prof;

    s5_say_pings();

    S5_LOG("[VC-S5] sweep: %d pings, face %d mm, echoes from %d to %d deg, "
           "beam %d deg\n",
           (int)g.sweep.n, (int)p->face_mm, (int)(p->right_ddeg / 10),
           (int)(p->left_ddeg / 10), (int)(p->beam_ddeg / 10));
    S5_LOG("[VC-S5] object %d mm wide: %d mm left and %d mm right of the "
           "line%s%s%s%s%s%s\n",
           (int)s5_mm(p->width), (int)s5_mm(p->y_left), (int)s5_mm(-p->y_right),
           p->b_left_seen     ? ""
           : p->b_left_hidden ? ", left edge hidden by the last object"
                              : ", left edge past the sweep",
           p->b_right_seen     ? ""
           : p->b_right_hidden ? ", right edge hidden by the last object"
                               : ", right edge past the sweep",
           p->b_left_blocked ? ", left side blocked" : "",
           p->b_left_maybe ? ", left maybe blocked" : "",
           p->b_right_blocked ? ", right side blocked" : "",
           p->b_right_maybe ? ", right maybe blocked" : "");
    /* Buddy 5, 2026-11-16: how the edges were got, with both results, to
     * hold against a tape measure */
    if (p->b_by_range)
    {
        S5_LOG("[VC-S5] edges by range: face %d mm, corners %d mm (left) "
               "and %d mm (right) away; by angle it would be %d mm left and "
               "%d mm right\n",
               (int)p->face_q_mm, (int)p->edge_l_mm, (int)p->edge_r_mm,
               (int)s5_mm(p->ya_left), (int)s5_mm(-p->ya_right));
    }
    else
    {
        S5_LOG("[VC-S5] edges by angle (the sweep was not taken from the "
               "line, straight)\n");
    }
    /* what blocked it */
    if (p->b_left_blocked || p->b_left_maybe)
    {
        S5_LOG("[VC-S5] left side %s by an echo at %d deg, %d mm\n",
               p->b_left_blocked ? "blocked" : "maybe blocked (beam edge)",
               (int)(p->block_l_ddeg / 10), (int)p->block_l_mm);
    }
    if (p->b_right_blocked || p->b_right_maybe)
    {
        S5_LOG("[VC-S5] right side %s by an echo at %d deg, %d mm\n",
               p->b_right_blocked ? "blocked" : "maybe blocked (beam edge)",
               (int)(p->block_r_ddeg / 10), (int)p->block_r_mm);
    }
}

static void s5_start_bypass(void);

/* Nothing to go round any more. Normally the car is on the line and Buddy
 * 3 takes over; an episode begun on the way back from the last one
 * (b_rejoin) first goes back onto the line along a ramp, as a bypass
 * ends. */
static s5_avoid_result_t s5_end_here(const char *p_why)
{
    float dy;

    if (!g.b_rejoin)
    {
        return s5_done(p_why);
    }
    dy = g.odo.y - g.line_est;
    S5_LOG("[VC-S5] %s: back to the line from %d mm off it\n", p_why,
           (int)s5_mm(dy));
    g.b_rejoin = false;
    g.b_blocked = false;
    g.side = (dy >= 0.0f) ? 1 : -1; /* the side the car is on */
    g.box = g.passed;               /* the ramp back must clear it */
    g.path.x0 = g.odo.x;
    g.path.l_out = 0.0f;
    g.path.y0 = g.odo.y;
    g.path.a = g.odo.y;
    g.path.y_end = g.line_est;
    g.path.l_in = s5_path_ramp_len_in(dy);
    s5_plan_back(g.odo.x); /* at once, or once past it */
    s5_start_bypass();
    g.b_back_said = true;        /* no new object: no far corner to tell */
    g.own_min = g.passed_fit_lo; /* the same object: its fit stands */
    g.own_max = g.passed_fit_hi;
    return S5_AVOID_RUNNING;
}

/* a sweep after another object met on the
 * way back may still see a corner of the object just gone round. Its far
 * corners are where they were fitted passing it, else anywhere from the
 * shallowest it can be to S5_DEPTH_DEFAULT_MM (never as far as something
 * else heard further on pushed it: that may be the new object). Those
 * returns are not the new object, and not clear either (S5_MM_UNKNOWN:
 * an edge next to them is not seen). */
static void s5_drop_passed(const s5_view_t *p_v)
{
    const s5_box_t *p_b = &g.passed;
    const float x_lo = g.b_passed_fit ? (g.passed_fit_lo - S5_KNOWN_TOL_MM)
                                      : (p_b->x_near + (float)S5_DEPTH_MIN_MM);
    const float x_hi =
        g.b_passed_fit
            ? (g.passed_fit_hi + S5_KNOWN_TOL_MM)
            : (p_b->x_near + (float)S5_DEPTH_DEFAULT_MM + S5_KNOWN_TOL_MM);
    uint32_t dropped = 0U;
    uint32_t i;

    if (!g.b_passed)
    {
        return;
    }
    for (i = 0U; i < g.sweep.n; i++)
    {
        s5_sample_t *p_s = &g.sweep.s[i];
        float look;
        float half;
        float mm;

        if ((0U == p_s->mm) || (S5_MM_UNKNOWN == p_s->mm))
        {
            continue;
        }
        look = p_v->psi + ((float)p_s->ddeg * 0.1f * S5_DEG2RAD_F);
        half = (float)(s5_scan_beam_ddeg(p_s->mm) + S5_KNOWN_BEAM_DDEG) * 0.1f *
               S5_DEG2RAD_F;
        mm = (float)p_s->mm;
        if (s5_scan_corner_echo(p_v->x, p_v->y, look, half, mm, p_b->y_left,
                                p_b->x_near, p_b->x_near, S5_KNOWN_TOL_MM) ||
            s5_scan_corner_echo(p_v->x, p_v->y, look, half, mm, p_b->y_right,
                                p_b->x_near, p_b->x_near, S5_KNOWN_TOL_MM) ||
            s5_scan_corner_echo(p_v->x, p_v->y, look, half, mm, p_b->y_left,
                                x_lo, x_hi, S5_KNOWN_TOL_MM) ||
            s5_scan_corner_echo(p_v->x, p_v->y, look, half, mm, p_b->y_right,
                                x_lo, x_hi, S5_KNOWN_TOL_MM))
        {
            p_s->mm = S5_MM_UNKNOWN;
            dropped++;
        }
    }
    if (0U != dropped)
    {
        S5_LOG("[VC-S5] sweep: %d returns from the object just passed left "
               "out\n",
               (int)dropped);
    }
}

/* Room to turn on the spot: nothing but the object just gone round
 * (left out of the sweep) answered within S5_PIVOT_ROOM_MM. The car's
 * outline swings through less than 130 mm of the sensor wherever the
 * sweep looks. */
static bool s5_turn_room(void)
{
    uint32_t i;

    for (i = 0U; i < g.sweep.n; i++)
    {
        const uint16_t mm = g.sweep.s[i].mm;

        if ((0U != mm) && (S5_MM_UNKNOWN != mm) &&
            (mm < (uint16_t)S5_PIVOT_ROOM_MM))
        {
            return false;
        }
    }
    return true;
}

/* The sweep is in: what is it, which way round, is there room. */
static s5_avoid_result_t s5_decide(void)
{
    s5_view_t view;
    s5_pick_t pick;
    int32_t side;

    s5_odom_point(&g.odo, (float)S5_SONAR_AHEAD_MM, 0.0f, &view.x, &view.y);
    view.psi = g.odo.psi;
    s5_drop_passed(&view);
    if (g.b_turn_check)
    {
        g.b_turn_check = false;
        if (s5_turn_room())
        {
            S5_LOG("[VC-S5] the car is %d deg to the line: turning on the "
                   "spot to straighten first\n",
                   (int)s5_deg(g.odo.psi));
            s5_enter(S5_PH_TURN);
            return S5_AVOID_RUNNING;
        }
        S5_LOG("[VC-S5] the car is %d deg to the line, something within %d "
               "mm: backing up to straighten first\n",
               (int)s5_deg(g.odo.psi), (int)S5_PIVOT_ROOM_MM);
        s5_back_by(S5_SQUARE_MM, S5_THEN_SCAN);
        return S5_AVOID_RUNNING;
    }
    s5_scan_profile(g.sweep.s, g.sweep.n, &view, &g.prof);
    if (!g.prof.b_found)
    {
        if (g.b_blocked)
        {
            return s5_end_here("the object has gone");
        }
        return s5_end_here("nothing in front after all");
    }
    s5_say_sweep();
    if (s5_absf(g.odo.psi) > (S5_PLAN_DEG * S5_DEG2RAD_F))
    {
        /* only after another object on the way back, when backing up
         * could not straighten the car (the last object is behind it) */
        s5_block("the car is too far off the line's direction to plan");
        return S5_AVOID_RUNNING;
    }
    {
        uint32_t i;

        for (i = 0U; i < g.sweep.n; i++)
        {
            g.rand ^= (uint32_t)g.sweep.s[i].mm << (i & 15U);
        }
    }
    g.box.x_near = g.prof.x_face;
    g.box.x_far = g.prof.x_face + (float)S5_DEPTH_DEFAULT_MM;
    g.box.y_left = g.prof.y_left;
    g.box.y_right = g.prof.y_right;
    g.b_depth = false;

    pick = s5_pick();
    if (S5_PICK_RESCAN == pick)
    {
        g.rescans++;
        /* only with no side open (s5_pick_sides()) */
        S5_LOG("[VC-S5] an edge is outside the sweep and no side is open: "
               "backing off %d mm to look again\n",
               (int)S5_RESCAN_BACK_MM);
        s5_back_by((float)S5_RESCAN_BACK_MM, S5_THEN_SCAN);
        return S5_AVOID_RUNNING;
    }
    if (S5_PICK_BLOCKED == pick)
    {
        /* no way round open from here: back up and look
         * again (the only backing up for room), then wait */
        if ((g.rescans < S5_MAX_RESCANS) &&
            (s5_back_room() >= (float)S5_RESCAN_BACK_MM))
        {
            g.rescans++;
            S5_LOG("[VC-S5] no room on either side from here: backing up "
                   "%d mm to look again\n",
                   (int)S5_RESCAN_BACK_MM);
            s5_back_by((float)S5_RESCAN_BACK_MM, S5_THEN_SCAN);
            return S5_AVOID_RUNNING;
        }
        s5_block("no room on either side");
        return S5_AVOID_RUNNING;
    }
    side = (S5_PICK_LEFT == pick) ? 1 : -1;
    if (!s5_plan(side))
    {
        const bool b_other_ok =
            (side > 0) ? (g.prof.b_right_seen && !g.prof.b_right_blocked &&
                          !g.prof.b_right_maybe)
                       : (g.prof.b_left_seen && !g.prof.b_left_blocked &&
                          !g.prof.b_left_maybe);

        if (b_other_ok && s5_plan(-side))
        {
            side = -side;
        }
        else
        {
            s5_block("too close to turn out, even after backing up");
            return S5_AVOID_RUNNING;
        }
    }
    g.b_blocked = false;
    if (g.b_out_pivot)
    {

        S5_LOG("[VC-S5] going %s: axle %d mm off the line; too close for a "
               "ramp: turn %d deg on the spot, out %d mm, turn back; back "
               "in from %d mm past the face\n",
               s5_side_name(side), (int)s5_mm(g.path.a),
               (int)s5_deg(s5_absf(g.out_th)),
               (int)s5_mm(s5_absf(g.path.a - g.odo.y) /
                          s5_absf(s5_sinf(g.out_th))),
               (int)s5_mm(g.path.x_back - g.box.x_near));
        g.out_step = 0U;
        s5_enter(S5_PH_OUT);
        return S5_AVOID_RUNNING;
    }
    S5_LOG("[VC-S5] going %s: axle %d mm off the line, ramps %d / %d mm, "
           "back in from %d mm past the face\n",
           s5_side_name(side), (int)s5_mm(g.path.a), (int)s5_mm(g.path.l_out),
           (int)s5_mm(g.path.l_in), (int)s5_mm(g.path.x_back - g.box.x_near));
    if (g.path.x0 < (g.odo.x - 5.0f))
    {
        S5_LOG("[VC-S5] no room to turn out even on the spot: backing up "
               "%d mm first\n",
               (int)s5_mm(g.odo.x - g.path.x0));
        g.back_then = S5_THEN_BYPASS;
        g.back_from = g.odo.x;
        g.back_x = g.path.x0;
        s5_enter(S5_PH_BACK);
        return S5_AVOID_RUNNING;
    }
    s5_start_bypass();
    return S5_AVOID_RUNNING;
}

/* ===================================================================== */
/*  Driving                                                               */
/* ===================================================================== */

static void s5_start_bypass(void)
{
    g.used_ping = g.ping.seq;
    g.side_moved = 0.0f;
    g.far_n = 0U;
    g.far_head = 0U;
    g.b_far_fit = false;
    g.b_back_said = false;
    g.far_lo = -1.0e9f;
    g.far_floor = -1.0e9f;
    g.b_behind = false;
    g.far_d = -1.0f;
    g.far_x = 0.0f;
    g.own_min = 1.0e9f;
    g.own_max = -1.0e9f;
    g.ahead_at = g.odo.dist;
    g.b_ahead = false;
    g.b_far_seen = false;
    g.maybe_n = 0U;
    g.line_y = g.line_est;
    g.line_seen = -1.0e9f;
    g.b_line = false;
    g.b_ir_black[0] = false;
    g.b_ir_black[1] = false;
    g.cap_first = -1;
    g.b_cap_skip = false;
    g.line_win = (float)S5_LINE_WINDOW_MM;
    g.b_ret_black = false;
    g.b_seek_ours = false; /* Buddy 5, 2026-11-16 */
    g.seek_stage = 0U;
    g.path.y_end = g.line_est;
    if (g.path.x0 < g.odo.x)
    {
        g.path.x0 = g.odo.x; /* crept on after the plan: turn out here */
    }
    s5_look(0);
    s5_enter(S5_PH_BYPASS);
}

static void s5_wheels(float v, float k)
{
    s5_path_wheels(v, k, &g.cmd_l, &g.cmd_r);
}

/* Backing up: heading held along the line, drifting back onto it; off
 * the line on purpose beside the object just gone round (b_rejoin), only
 * straightening. */
static void s5_reverse_step(void)
{
    const float s0 = (float)S5_TRACK_DIST_MM;
    const float v = -(float)S5_REVERSE_SPEED_MM_S;
    const float want =
        g.b_rejoin ? 0.0f
                   : s5_clampf(s5_atanf((g.odo.y - g.line_est) / (2.0f * s0)),
                               -S5_REV_HEADING_MAX, S5_REV_HEADING_MAX);
    const float rate = -((2.0f * (float)S5_REVERSE_SPEED_MM_S) / s0) *
                       s5_sinf(g.odo.psi - want);

    s5_wheels(v, rate / v);
}

/* Move the pass further out: the side is d closer than the sweep said
 * (never back in). */
static void s5_move_out(float d, const char *p_what)
{
    const float dd = (d > (float)S5_SIDE_MAX_MM) ? (float)S5_SIDE_MAX_MM : d;

    if (dd > (g.side_moved + 2.0f))
    {
        const float extra = dd - g.side_moved;
        const float s = (float)g.side;

        g.side_moved = dd;
        g.path.a += s * extra;
        if (g.side > 0)
        {
            g.box.y_left += extra;
        }
        else
        {
            g.box.y_right -= extra;
        }
        g.path.l_in = s5_path_ramp_len_in(g.path.a - g.path.y_end);
        if (g.odo.x < g.path.x_back)
        {
            s5_plan_back(g.odo.x);
        }
        S5_LOG("[VC-S5] %s %d mm closer than planned: %d mm further "
               "out\n",
               p_what, (int)s5_mm(d), (int)s5_mm(extra));
    }
}

/* The far corner from the pings at it: the point whose distances from
 * where the sensor was best match the ranges (least squares by
 * Gauss-Newton, from the corner as planned). The sensor moves along the
 * box while the corner answers at 20..60 deg, so both how deep the box
 * is and how far its side is from the car come out of it, whatever the
 * beam width and whatever the encoders got wrong since the sweep. */
static bool s5_far_solve(uint32_t skip, float *p_x, float *p_y,
                         uint32_t *p_worst)
{
    const uint32_t n = (g.far_n < S5_FAR_N) ? g.far_n : S5_FAR_N;
    float x = g.box.x_far;
    float y = s5_edge();
    float lo = 1.0e9f;
    float hi = -1.0e9f;
    float worst = -1.0f;
    uint32_t it;
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        if (i != skip)
        {
            lo = (g.far_px[i] < lo) ? g.far_px[i] : lo;
            hi = (g.far_px[i] > hi) ? g.far_px[i] : hi;
        }
    }
    if ((n < S5_FAR_MIN) || ((hi - lo) < S5_FAR_SPAN_MM))
    {
        return false;
    }
    for (it = 0U; it < 8U; it++)
    {
        float a11 = 0.0f;
        float a12 = 0.0f;
        float a22 = 0.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float det;
        float sx;
        float sy;

        for (i = 0U; i < n; i++)
        {
            const float dx = x - g.far_px[i];
            const float dy = y - g.far_py[i];
            const float d = s5_sqrtf((dx * dx) + (dy * dy));

            if ((i != skip) && (d > 1.0f))
            {
                const float jx = dx / d;
                const float jy = dy / d;
                const float res = d - g.far_pr[i];

                a11 += jx * jx;
                a12 += jx * jy;
                a22 += jy * jy;
                b1 += jx * res;
                b2 += jy * res;
            }
        }
        det = (a11 * a22) - (a12 * a12);
        if (det < 1.0e-3f)
        {
            return false; /* the bearings hardly changed */
        }
        sx = ((a22 * b1) - (a12 * b2)) / det;
        sy = ((a11 * b2) - (a12 * b1)) / det;
        x -= sx;
        y -= sy;
        if ((s5_absf(sx) + s5_absf(sy)) < 0.5f)
        {
            break;
        }
    }
    for (i = 0U; i < n; i++)
    {
        if (i != skip)
        {
            const float dx = x - g.far_px[i];
            const float dy = y - g.far_py[i];
            const float res =
                s5_absf(s5_sqrtf((dx * dx) + (dy * dy)) - g.far_pr[i]);

            if (res > worst)
            {
                worst = res;
                *p_worst = i;
            }
        }
    }
    *p_x = x;
    *p_y = y;
    return worst <= S5_FAR_FIT_MM;
}

/* the far corner was fitted at far_x, and
 * something answers from well past it, at about x: another object close
 * behind. Its side cannot be seen (only corners and faces square on
 * answer), so the pass goes on past both: from here the far end is taken
 * at least S5_DEPTH_DEFAULT_MM past x, further if a deeper corner shows. */
static void s5_far_behind(float x)
{
    const float x_far = x + (float)S5_DEPTH_DEFAULT_MM;

    g.b_behind = true;
    g.far_own = g.box.x_far;
    S5_LOG("[VC-S5] another object just behind this one (%d mm past its far "
           "corner): going on past both\n",
           (int)s5_mm(x - g.far_x));
    g.far_floor = x_far;
    if (x_far > g.box.x_far)
    {
        g.box.x_far = x_far;
        s5_plan_back(g.odo.x);
    }
}

/* Whatever answered a ping across is at least this far along the line:
 * its range along the edge of the beam nearest square-on. Two pings in a row
 * past the object's far side
 * push the far side on, so the way back never starts before something
 * the sensor has heard, however deep the object is and whether or not
 * its corner can be fixed. */
static void s5_far_bound(float xf, int16_t ddeg, uint16_t mm)
{
    const float half = (float)(s5_scan_beam_ddeg(mm) + S5_KNOWN_BEAM_DDEG) *
                       0.1f * S5_DEG2RAD_F;
    const float look = g.odo.psi + ((float)ddeg * 0.1f * S5_DEG2RAD_F);
    const float c1 = s5_cosf(look - half);
    const float c2 = s5_cosf(look + half);
    const float x_lo = xf + ((float)mm * ((c1 < c2) ? c1 : c2));

    if (g.odo.x >= g.path.x_back)
    {
        return; /* on the way back already */
    }
    if (g.b_far_fit && !g.b_behind && (x_lo > (g.far_x + S5_FAR_JUMP_MM)))
    {
        /* past the far corner fitted, two pings in a row: another
         * object; it is no further than the beam's nearest edge to
         * straight ahead puts it */
        if (g.far_lo > (g.far_x + S5_FAR_JUMP_MM))
        {
            const float c_hi = ((look - half) < 0.0f) && ((look + half) > 0.0f)
                                   ? 1.0f
                                   : ((c1 > c2) ? c1 : c2);

            s5_far_behind(xf + ((float)mm * c_hi));
        }
        g.far_lo = x_lo;
        return;
    }
    if (x_lo > (g.box.x_far + 5.0f))
    {
        if (g.far_lo > (g.box.x_far + 5.0f))
        {
            g.box.x_far = (x_lo < g.far_lo) ? x_lo : g.far_lo;
            /* never undone by a shallower fit */
            g.far_floor =
                (g.box.x_far > g.far_floor) ? g.box.x_far : g.far_floor;
            s5_plan_back(g.odo.x);
        }
        g.far_lo = x_lo;
    }
    else
    {
        g.far_lo = -1.0e9f;
    }
}

/* How far apart, rad, the bearings were from where the pings were fired
 * to the fitted corner (x, y), leaving out ping 'skip'. Pings fired from
 * nearly the same bearing fix the range but not the corner. */
static float s5_far_spread(uint32_t skip, float x, float y)
{
    const uint32_t n = (g.far_n < S5_FAR_N) ? g.far_n : S5_FAR_N;
    float lo = 10.0f;
    float hi = -10.0f;
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        if (i != skip)
        {
            const float b = s5_atan2f(y - g.far_py[i], x - g.far_px[i]);

            lo = (b < lo) ? b : lo;
            hi = (b > hi) ? b : hi;
        }
    }
    return (hi > lo) ? (hi - lo) : 0.0f;
}

/* A ping at the far corner, sensor at (xf, yf) when it was fired. */
static void s5_far_ping(float xf, float yf, float r)
{
    const float s = (float)g.side;
    uint32_t skip = S5_FAR_N; /* none */
    uint32_t worst = 0U;
    float x;
    float y;
    bool b_ok;

    g.far_px[g.far_head] = xf;
    g.far_py[g.far_head] = yf;
    g.far_pr[g.far_head] = r;
    g.far_head = (g.far_head + 1U) % S5_FAR_N;
    if (g.far_n < S5_FAR_N)
    {
        g.far_n++;
    }
    if (g.odo.x >= g.path.x_back)
    {
        return; /* on the way back already */
    }
    b_ok = s5_far_solve(skip, &x, &y, &worst);
    if (!b_ok && (g.far_n > S5_FAR_MIN))
    {
        skip = worst;
        b_ok = s5_far_solve(skip, &x, &y, &worst); /* one odd ping out */
    }
    /* a fit counts only when the pings
     * saw the corner from bearings S5_FAR_SPREAD_DEG apart and it lies
     * well towards the object from the sensor's track; one deeper than
     * S5_DEPTH_MAX_MM is taken as that deep, not thrown away */
    if (!b_ok || ((s * (yf - y)) < S5_FAR_INSIDE_MM) ||
        (x < (g.box.x_near + (0.5f * (float)S5_DEPTH_MIN_MM))) ||
        (s5_far_spread(skip, x, y) < (S5_FAR_SPREAD_DEG * S5_DEG2RAD_F)))
    {
        return; /* not this box's corner, or not seen well enough yet */
    }
    {
        /* how much further out than planned the side is (the sweep's
         * edge, before any move out) */
        const float d = s * (y - s5_edge()) + g.side_moved;
        const float x_far = s5_clampf(x, g.box.x_near + (float)S5_DEPTH_MIN_MM,
                                      g.box.x_near + (float)S5_DEPTH_MAX_MM);

        if (d < -S5_FAR_INWARD_MM)
        {
            return; /* another object's corner, further across */
        }

        /* the side: further out only when two fits in a row say so, by
         * the smaller of the two */
        if ((d > (float)S5_SIDE_TOL_MM) && (d < (float)S5_FAR_SANE_MM) &&
            (g.far_d > (float)S5_SIDE_TOL_MM))
        {
            s5_move_out((d < g.far_d) ? d : g.far_d, "side (far corner)");
        }
        if (g.b_far_fit && !g.b_behind && (x_far > (g.far_x + S5_FAR_JUMP_MM)))
        {
            s5_far_behind(x_far); /* another object's corner */
            g.far_x = x_far;
            return;
        }
        g.far_d = d;
        if (x_far > (g.box.x_far + 5.0f))
        {
            g.box.x_far = x_far; /* deeper: at once */
            s5_plan_back(g.odo.x);
        }
        else if ((x_far < (g.box.x_far - 5.0f)) && g.b_far_fit &&
                 (s5_absf(x_far - g.far_x) <= 15.0f))
        {
            /* shallower, so the way back earlier: only when two fits in
             * a row agree, and never shallower than any fit of this
             * corner so far (fits that
             * creep, one step at a time, towards the corner of something
             * close behind must not pull the way back in), something
             * heard further on (s5_far_bound()) or another object close
             * behind allows */
            float xn = (g.own_max > x_far) ? g.own_max : x_far;

            xn = (xn > g.far_floor) ? xn : g.far_floor;
            if (xn < g.box.x_far)
            {
                g.box.x_far = xn;
                s5_plan_back(g.odo.x);
            }
        }
        else
        {
            /* no change */
        }
        g.far_x = x_far;
        g.b_far_fit = true;
        if (!g.b_behind)
        {
            g.own_min = (x_far < g.own_min) ? x_far : g.own_min;
            g.own_max = (x_far > g.own_max) ? x_far : g.own_max;
        }
        if (!g.b_depth && (g.far_n >= S5_FAR_SAY_N))
        {
            g.b_depth = true;
            S5_LOG("[VC-S5] far corner: object %d mm deep, its side %d mm "
                   "from the sweep's; back in from %d mm past the face\n",
                   (int)s5_mm(g.box.x_far - g.box.x_near), (int)s5_mm(d),
                   (int)s5_mm(g.path.x_back - g.box.x_near));
        }
    }
}

/* Where a ray from (xs, ys) along (ux, uy) enters the slab lo..hi of one
 * axis (o, u: that axis's start and direction): narrows t0..t1. */
static bool s5_slab(float o, float u, float lo, float hi, float *p_t0,
                    float *p_t1)
{
    if (s5_absf(u) < 1.0e-6f)
    {
        return (o >= lo) && (o <= hi);
    }
    {
        float ta = (lo - o) / u;
        float tb = (hi - o) / u;

        if (ta > tb)
        {
            const float t = ta;

            ta = tb;
            tb = t;
        }
        *p_t0 = (ta > *p_t0) ? ta : *p_t0;
        *p_t1 = (tb < *p_t1) ? tb : *p_t1;
    }
    return *p_t0 <= *p_t1;
}

/* How far along a ray from (xs, ys) at angle a it first meets the box;
 * -1: never. */
static float s5_ray_box(const s5_box_t *p_b, float xs, float ys, float a)
{
    float t0 = 0.0f;
    float t1 = 1.0e9f;

    if (!s5_slab(xs, s5_cosf(a), p_b->x_near, p_b->x_far, &t0, &t1) ||
        !s5_slab(ys, s5_sinf(a), p_b->y_right, p_b->y_left, &t0, &t1))
    {
        return -1.0f;
    }
    return t0;
}

/* The shortest range at which the box could answer a ping fired from
 * (xs, ys) towards dir, beam half-angle half: its nearest point inside
 * the beam (the nearest point of all if that is inside, else the first
 * point an edge of the beam meets). -1: no part of it is in the beam. */
static float s5_box_range(const s5_box_t *p_b, float xs, float ys, float dir,
                          float half)
{
    const float px = s5_clampf(xs, p_b->x_near, p_b->x_far);
    const float py = s5_clampf(ys, p_b->y_right, p_b->y_left);
    const float d = s5_sqrtf(((px - xs) * (px - xs)) + ((py - ys) * (py - ys)));
    float best = -1.0f;
    float t;

    if (d < 1.0f)
    {
        return 0.0f;
    }
    if (s5_absf(s5_wrapf(s5_atan2f(py - ys, px - xs) - dir)) <= half)
    {
        return d;
    }
    t = s5_ray_box(p_b, xs, ys, dir - half);
    if (t >= 0.0f)
    {
        best = t;
    }
    t = s5_ray_box(p_b, xs, ys, dir + half);
    if ((t >= 0.0f) && ((best < 0.0f) || (t < best)))
    {
        best = t;
    }
    return best;
}

/* What a fresh ping p within S5_DETECT_MM, on the way back or along the
 * pass lane, may have come from. */
typedef enum
{
    S5_FROM_KNOWN = 0, /* the object being gone round, as far as known (at
                          the ping's calibrated beam + S5_KNOWN_BEAM_DDEG) */
    S5_FROM_MAYBE,     /* its far corner, while its depth is not measured
                          (any depth up to S5_DEPTH_MAX_MM), or something
                          else in the pass lane: look again */
    S5_FROM_OTHER      /* another object */
} s5_from_t;

static s5_from_t s5_echo_from(const motion_status_t *p_m, const s5_ping_t *p_p)
{
    /* where the sensor was when the ping went out, and its beam */
    const float back = 0.5f * (float)(p_m->speed_l_mmps + p_m->speed_r_mmps) *
                       (float)(s5_hw_us() - p_p->fired_us) * 1.0e-6f;
    const float half =
        (float)(s5_scan_beam_ddeg(p_p->mm) + S5_KNOWN_BEAM_DDEG) * 0.1f *
        S5_DEG2RAD_F;
    const float dir = g.odo.psi + ((float)p_p->ddeg * 0.1f * S5_DEG2RAD_F);
    const float mm = (float)p_p->mm;
    s5_box_t own = g.box;
    float xs;
    float ys;
    float r_box;

    if (g.b_behind)
    {
        /* passing both: only the part measured is known; the rest of the
         * one behind is a guess and explains nothing (it may stick out
         * into the pass lane further than this one) */
        own.x_far = g.far_own;
    }
    s5_odom_point(&g.odo, (float)S5_SONAR_AHEAD_MM, 0.0f, &xs, &ys);
    xs -= back * s5_cosf(g.odo.psi);
    ys -= back * s5_sinf(g.odo.psi);
    r_box = s5_box_range(&own, xs, ys, dir, half);
    if ((r_box >= 0.0f) && (mm >= (r_box - S5_KNOWN_TOL_MM)))
    {
        return S5_FROM_KNOWN;
    }
    if (!g.b_far_fit &&
        s5_scan_corner_echo(xs, ys, dir, half, mm, s5_edge(),
                            g.box.x_near + (float)S5_DEPTH_MIN_MM,
                            g.box.x_near + (float)S5_DEPTH_MAX_MM,
                            S5_KNOWN_TOL_MM))
    {
        return S5_FROM_MAYBE;
    }
    return S5_FROM_OTHER;
}

/* On the way back, the sensor looks straight ahead. True: another
 * object within S5_DETECT_MM. The depth the car turned in for is all the
 * object may have: anything past it is in the way. */
static bool s5_next_ahead(const motion_status_t *p_m)
{
    s5_ping_t p;

    s5_look(0);
    return s5_new_ping(&p, 0) && (0U != p.mm) &&
           (p.mm <= (uint16_t)S5_DETECT_MM) &&
           (S5_FROM_KNOWN != s5_echo_from(p_m, &p));
}

/* An echo the object being passed cannot explain, heard looking along the
 * pass lane: does it stop the car? On
 * the steep part of the ramp out the sensor is still close to the
 * object's side, and the look takes in the corner a box turned towards
 * the pass lane sticks out there; that corner is no further on than the
 * depth planned for. Only an echo from further on counts then. The far
 * corner watch moves the pass out for the corner, and the looks once the
 * car is straighter (S5_AIM_STEEP_DEG) see the whole lane. */
static bool s5_lane_counts(const s5_ping_t *p_p)
{
    float xs;
    float ys;

    if (s5_absf(g.odo.psi) <= (S5_AIM_STEEP_DEG * S5_DEG2RAD_F))
    {
        return true;
    }
    s5_odom_point(&g.odo, (float)S5_SONAR_AHEAD_MM, 0.0f, &xs, &ys);
    return (xs + (float)p_p->mm) > g.box.x_far; /* as far on as it can be */
}

/* Passing: one look along the pass lane every S5_AHEAD_EVERY_MM, and at
 * every ping while what answers may be either this object's far corner
 * or something in the lane (it is the corner only while it is more than
 * about S5_DETECT_MM - 40 mm away). True: another object within
 * S5_DETECT_MM in it. *p_b_busy: looking ahead now (the far corner watch
 * waits). */
static bool s5_lane_ahead(const motion_status_t *p_m, bool *p_b_busy)
{
    s5_ping_t p;

    *p_b_busy = false;
    if (!g.b_ahead && ((g.odo.dist < g.ahead_at) ||
                       (g.b_far_seen && !g.b_far_fit &&
                        (g.odo.dist < (g.ahead_at + S5_AHEAD_HOLD_MM)))))
    {
        return false;
    }
    g.b_ahead = true;
    *p_b_busy = true;
    /* along the lane, whatever the car's heading (on the ramp out the car
     * points well off it) */
    s5_look(-s5_roundf(g.odo.psi * S5_RAD2DEG_F * 10.0f));
    if (!s5_new_ping(&p, g.look_ddeg))
    {
        return false;
    }
    if ((0U != p.mm) && (p.mm <= (uint16_t)S5_DETECT_MM))
    {
        const s5_from_t e = s5_echo_from(p_m, &p);

        if ((S5_FROM_OTHER == e) && s5_lane_counts(&p))
        {
            return true;
        }
        if (S5_FROM_MAYBE == e)
        {
            g.maybe_n = S5_MAYBE_GAPS;
            return false; /* look again at the next ping */
        }
    }
    else if ((0U == p.mm) && (0U != g.maybe_n))
    {
        g.maybe_n--;
        return false; /* a dropout while following it: look again */
    }
    else
    {
        /* clear, or further than S5_DETECT_MM */
    }
    g.maybe_n = 0U;
    g.b_ahead = false;
    g.ahead_at = g.odo.dist + S5_AHEAD_EVERY_MM;
    return false;
}

/* Another object on the way back or in the pass lane: stop and start again from
 * the sweep, in the same line
 * frame. The line is where the IRs found it, the car never backs into
 * the object it is going round (g.passed), and if there turns out to be
 * nothing to go round it goes back onto the line (b_rejoin). */
static s5_avoid_result_t s5_next_object(void)
{
    S5_LOG("[VC-S5] another object %s (%d mm away): stopping\n", g.p_where,
           (int)g.ping.mm);
    if (g.n_next >= S5_NEXT_MAX)
    {
        /* the episode's time and
         * reverse limits run on across objects; this ends a car that
         * keeps meeting them */
        return s5_fail("another object again: S5_NEXT_MAX in one bypass");
    }
    g.n_next++;
    if (g.b_line)
    {
        g.line_est = g.line_y;
    }
    g.passed = g.box;
    if (g.b_behind)
    {
        g.passed.x_far = g.far_own; /* not the one behind it */
    }
    g.b_passed_fit = (g.own_max > g.own_min - 1.0f);
    g.passed_fit_lo = g.own_min;
    g.passed_fit_hi = g.own_max;
    g.b_passed = true;
    g.b_rejoin = true;
    g.rescans = 0U;
    g.b_blocked = false;
    g.b_block_said = false;
    g.then = S5_THEN_FACE;
    s5_enter(S5_PH_STOP);
    return S5_AVOID_RUNNING;
}

/* The sensor during the bypass: straight ahead while turning out and on
 * the way back, across at the far corner while passing, with a look
 * along the pass lane every S5_AHEAD_EVERY_MM. False: another object
 * ahead (g.p_where says where).
 *
 * There is no near-corner check: while the
 * sensor is inside the edge the face answers before the corner does, and
 * once it is outside the edge the car is heading out so steeply that the
 * corner is past the servo's 60 deg. The near corner's gap is the sweep's
 * (calibration step 3) plus S5_CLEARANCE_MM; the far corner below
 * corrects the rest of the pass and the way back in. */
static bool s5_bypass_sensor(const motion_status_t *p_m)
{
    const float s = (float)g.side;
    float xs;
    float ys;
    s5_ping_t p;

    s5_odom_point(&g.odo, (float)S5_SONAR_AHEAD_MM, 0.0f, &xs, &ys);

    if (g.odo.x >= g.path.x_back)
    {
        if (!g.b_back_said)
        {
            g.b_back_said = true;
            if (!g.b_far_fit)
            {
                S5_LOG("[VC-S5] far corner not fixed: back in after %d mm "
                       "of object (S5_DEPTH_DEFAULT_MM %d)\n",
                       (int)s5_mm(g.box.x_far - g.box.x_near),
                       (int)S5_DEPTH_DEFAULT_MM);
            }
        }
        g.p_where = "ahead on the way back";
        return !s5_next_ahead(p_m);
    }

    if (xs >= (g.box.x_near - 60.0f))
    {
        /* passing: look across for the far corner; only once the
         * sensor is past the face, so the near corner cannot answer */
        const int32_t want = -g.side * S5_WATCH_DDEG;
        bool b_busy;

        g.p_where = "ahead in the pass lane";
        if (s5_lane_ahead(p_m, &b_busy))
        {
            return false;
        }
        if (b_busy)
        {
            return true;
        }
        s5_look(want);
        if (s5_new_ping(&p, want))
        {
            g.b_far_seen = false; /* nothing across, or far off */
            if ((0U != p.mm) && (p.mm <= (uint16_t)S5_WATCH_RANGE_MM))
            {
                /* where the sensor was when the ping went out */
                const float back =
                    0.5f * (float)(p_m->speed_l_mmps + p_m->speed_r_mmps) *
                    (float)(s5_hw_us() - p.fired_us) * 1.0e-6f;
                const float xf = xs - (back * s5_cosf(g.odo.psi));
                const float yf = ys - (back * s5_sinf(g.odo.psi));
                const float half =
                    (float)(s5_scan_beam_ddeg(p.mm) + S5_KNOWN_BEAM_DDEG) *
                    0.1f * S5_DEG2RAD_F;
                const float look =
                    g.odo.psi + ((float)p.ddeg * 0.1f * S5_DEG2RAD_F);

                /* from the far corner unless the near
                 * corner could have answered (it can only while the
                 * sensor is short of the face): a box turned towards the
                 * pass lane shows its far corner well before that */
                g.b_far_seen = ((s * (yf - s5_edge())) > 0.0f) &&
                               !s5_scan_corner_echo(
                                   xf, yf, look, half, (float)p.mm, s5_edge(),
                                   g.box.x_near, g.box.x_near, S5_KNOWN_TOL_MM);
                if (g.b_far_seen)
                {
                    s5_far_bound(xf, p.ddeg, p.mm);
                    s5_far_ping(xf, yf, (float)p.mm);
                }
            }
        }
        return true;
    }

    s5_look(0); /* turning out */
    return true;
}

static void s5_line_at(float y, const char *p_how)
{
    if (!g.b_line)
    {
        S5_LOG("[VC-S5] IRs found the line (%s), %d mm from where the "
               "encoders put it\n",
               p_how, (int)s5_mm(y));
    }
    g.line_y = y;
    g.line_seen = g.odo.dist;
    g.b_line = true;
    g.path.y_end = y;
    g.line_win = (float)S5_LINE_WINDOW_MM;
}

/* The IRs on the way back. dir: the way the car moves across the line
 * (+1 left, -1 right; 0 = only full crossings count). */
static void s5_ir_watch(bool b_ir_l, bool b_ir_r, int32_t dir)
{
    uint32_t k;

    for (k = 0U; k < 2U; k++)
    {
        const bool b_black = (0U == k) ? b_ir_l : b_ir_r;
        const float side =
            (0U == k) ? (float)S5_IR_HALF_GAP_MM : -(float)S5_IR_HALF_GAP_MM;
        float xk;
        float yk;

        s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, side, &xk, &yk);
        if (b_black && !g.b_ir_black[k])
        {
            g.b_ir_black[k] = true;
            g.ir_entry[k] = yk;
            /* nothing counts while the IRs are on a line
             * that s5_cap_watch() found is not ours */
            if (!g.b_cap_skip)
            {
                g.b_ret_black = true;
            }
            g.b_ir_ok[k] = !g.b_cap_skip &&
                           (s5_absf(yk - g.line_y) <= g.line_win);
            if (g.b_ir_ok[k] && g.b_line)
            {
                g.line_seen = g.odo.dist;
            }
            if (g.b_ir_ok[k] && (0 != dir) && !g.b_line)
            {
                /* it went in over the near edge */
                s5_line_at(yk + ((float)dir * (float)S5_LINE_HALF_MM), "edge");
            }
        }
        else if (!b_black && g.b_ir_black[k])
        {
            const float span = s5_absf(yk - g.ir_entry[k]);

            g.b_ir_black[k] = false;
            if (g.b_ir_ok[k] && g.b_line)
            {
                g.line_seen = g.odo.dist; /* it left our line */
            }
            if (g.b_ir_ok[k] && (span >= S5_CROSS_MIN_MM) &&
                (span <= S5_CROSS_MAX_MM))
            {
                s5_line_at(0.5f * (yk + g.ir_entry[k]), "crossed");
            }
        }
        else
        {
            /* no change */
        }
    }
}

/* the line as the IRs have it now, before a crossing
 * that may turn out not to be ours */
static void s5_cap_snap(void)
{
    g.cap_snap.b_line = g.b_line;
    g.cap_snap.line_y = g.line_y;
    g.cap_snap.y_end = g.path.y_end;
    g.cap_snap.line_seen = g.line_seen;
    g.cap_snap.line_win = g.line_win;
    g.cap_snap.b_ret_black = g.b_ret_black;
}

/* the IRs crossed a line that is not ours (a junction's
 * branch): forget what they made of it and drive on; look again once
 * both IRs are white */
static void s5_cap_reject(void)
{
    g.b_line = g.cap_snap.b_line;
    g.line_y = g.cap_snap.line_y;
    g.path.y_end = g.cap_snap.y_end;
    g.line_seen = g.cap_snap.line_seen;
    g.line_win = g.cap_snap.line_win;
    g.b_ret_black = g.cap_snap.b_ret_black;
    g.b_ir_ok[0] = false;
    g.b_ir_ok[1] = false;
    g.cap_first = -1;
    g.b_cap_skip = true;
    g.cap_skip_at = g.odo.dist;
}

/* the IRs on the way back, for the pick-up. True: the
 * car is crossing our line (one IR and then the other within
 * S5_CAP_PAIR_MM, each near where the line should be, and the two places
 * they met it in a line along ours; or both IRs black at once with the
 * car crossing steeply), and g.cap_dir / g.line_y say which way and
 * where. A line at an angle to ours (a junction's branch) is left alone.
 * Call before s5_ir_watch(): it reads the IR states that one keeps. */
static bool s5_cap_watch(bool b_ir_l, bool b_ir_r)
{
    const float psi = g.odo.psi;
    const bool b_steep = (s5_absf(psi) >= (S5_CAP_MIN_DEG * S5_DEG2RAD_F));
    const float win = (g.line_win > (float)S5_CAP_WINDOW_MM)
                          ? g.line_win
                          : (float)S5_CAP_WINDOW_MM;
    bool b_new[2];
    float xk[2];
    float yk[2];
    uint32_t k;

    if (g.b_cap_skip)
    {
        if (b_ir_l || b_ir_r)
        {
            g.cap_skip_at = g.odo.dist;
        }
        else if ((g.odo.dist - g.cap_skip_at) >= S5_CAP_REARM_MM)
        {
            g.b_cap_skip = false;
        }
        else
        {
            /* still near it */
        }
        return false;
    }
    if ((g.cap_first >= 0) &&
        ((g.odo.dist - g.cap_first_at) > S5_CAP_PAIR_MM))
    {
        g.cap_first = -1; /* too long ago: a mark, not our line */
    }
    for (k = 0U; k < 2U; k++)
    {
        const bool b_black = (0U == k) ? b_ir_l : b_ir_r;
        const float side =
            (0U == k) ? (float)S5_IR_HALF_GAP_MM : -(float)S5_IR_HALF_GAP_MM;

        s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, side, &xk[k], &yk[k]);
        b_new[k] = b_black && !g.b_ir_black[k] &&
                   (s5_absf(yk[k] - g.line_y) <= win);
    }
    g.cap_dir = (psi > 0.0f) ? 1 : -1;
    for (k = 0U; k < 2U; k++)
    {
        const uint32_t o = 1U - k; /* the other IR */
        const bool b_other = (0U == o) ? b_ir_l : b_ir_r;

        if (b_new[k])
        {
            if (g.cap_first < 0)
            {
                s5_cap_snap();
            }
            if ((g.cap_first >= 0) && ((uint32_t)g.cap_first == o))
            {
                /* the second IR met the tape's near edge. Where the two met
                 * it gives the line's direction without the encoders'
                 * heading, also while the car turns */
                float dx = xk[k] - g.cap_x1;
                float dy = yk[k] - g.cap_y1;

                if (dx < 0.0f)
                {
                    dx = -dx;
                    dy = -dy;
                }
                g.cap_phi = s5_atan2f(dy, dx);
                /* Buddy 5, 2026-11-16: not while looking on with no black
                 * seen on the way back (b_seek_ours). The encoders' heading
                 * is what has gone wrong by then, and this direction is
                 * worked out from it: on the track the car's own line was
                 * turned down here as a junction */
                if (!g.b_seek_ours &&
                    (s5_absf(g.cap_phi) >
                     ((float)S5_CAP_LINE_DEG * S5_DEG2RAD_F)))
                {
                    S5_LOG("[VC-S5] the IRs crossed a line %d mm from where "
                           "the encoders put ours, at %d deg to it: a "
                           "junction, not our line; driving on\n",
                           (int)s5_mm(yk[k] - g.cap_snap.line_y),
                           (int)s5_deg(g.cap_phi));
                    s5_cap_reject();
                    return false;
                }
                if (!b_steep)
                {
                    /* along the line: the handover deals with it */
                    g.cap_first = -1;
                    return false;
                }
                g.line_y =
                    yk[k] + ((float)g.cap_dir * (float)S5_LINE_HALF_MM);
                g.cap_second = (int32_t)k;
                g.b_cap_phi = true;
                return true;
            }
            if (b_other)
            {
                /* both black, where the other one met it not seen: on a
                 * line along ours that takes a steep crossing (25 mm between
                 * the IRs, 18 mm tape) */
                float xc;
                float yc;

                s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, 0.0f, &xc, &yc);
                /* Buddy 5, 2026-11-16: as above */
                if (!g.b_seek_ours &&
                    (s5_absf(psi) < ((float)S5_CAP_BOTH_DEG * S5_DEG2RAD_F)))
                {
                    S5_LOG("[VC-S5] both IRs black at once %d mm from where "
                           "the encoders put the line, car %d deg to it: a "
                           "line across ours (junction); driving on\n",
                           (int)s5_mm(yc - g.cap_snap.line_y),
                           (int)s5_deg(psi));
                    s5_cap_reject();
                    return false;
                }
                g.line_y = yc;
                g.b_cap_phi = false;
                return true;
            }
            g.cap_first = (int32_t)k;
            g.cap_first_at = g.odo.dist;
            g.cap_x1 = xk[k];
            g.cap_y1 = yk[k];
        }
    }
    return false;
}

/* line search. Something within S5_STOP_MM ahead
 * stops the car until it has gone: there is no line yet to plan a way
 * round it against. True: standing. */
static bool s5_search_held(void)
{
    s5_ping_t p;
    bool b_near;

    s5_look(0);
    b_near = s5_get_ping(&p) && p.b_valid && s5_fresh(&p) &&
             (p.ddeg <= S5_LOOK_TOL_DDEG) && (p.ddeg >= -S5_LOOK_TOL_DDEG) &&
             (0U != p.mm) && (p.mm <= (uint16_t)S5_STOP_MM);
    if (b_near && !g.b_search_held)
    {
        S5_LOG("[VC-S5] line search: something %d mm ahead; standing until "
               "it has gone\n",
               (int)p.mm);
    }
    else if (!b_near && g.b_search_held)
    {
        S5_LOG("[VC-S5] line search: the way ahead is clear again\n");
    }
    else
    {
        /* no change */
    }
    g.b_search_held = b_near;
    return b_near;
}

static s5_avoid_result_t s5_do_capture(const motion_status_t *p_m,
                                       bool b_ir_l, bool b_ir_r);

static s5_avoid_result_t s5_cap_start(const motion_status_t *p_m, bool b_ir_l,
                                      bool b_ir_r)
{
    /* a line search has the line only from the IRs and
     * says so itself (s5_search_found()) */
    const float max_th =
        (g.b_search ? 90.0f : S5_CAP_MAX_DEG) * S5_DEG2RAD_F;

    if (!g.b_search)
    {
        S5_LOG("[VC-S5] both IRs crossed the line (%d mm from where the "
               "encoders put it, %d deg to it): onto it and facing along "
               "it\n",
               (int)s5_mm(g.line_y), (int)s5_deg(g.odo.psi));
    }
    if (g.b_cap_phi)
    {
        /* where the two IRs met the tape says which way
         * the line really runs; the encoders' heading may have drifted on
         * the way round. Turn the encoders' frame to it, then drive on and
         * turn by that. */
        float th = s5_wrapf(g.odo.psi - g.cap_phi);
        float xk;
        float yk;
        const float side = (0 == g.cap_second) ? (float)S5_IR_HALF_GAP_MM
                                               : -(float)S5_IR_HALF_GAP_MM;

        th = (th > max_th) ? max_th : th;
        th = (th < -max_th) ? -max_th : th;
        if (!g.b_search)
        {
            s5_mag_say();
            S5_LOG("[VC-S5] by where the two IRs met it, the car is %d deg "
                   "to the line (the encoders said %d)\n",
                   (int)s5_deg(th), (int)s5_deg(g.odo.psi));
        }
        g.cap_dir = (th > 0.0f) ? 1 : -1;
        /* from here the IRs have the line; the compass
         * (maybe bent by iron here) is not used for the turn onto it */
        g.b_mag = false;
        s5_odom_set_pose(&g.odo, g.odo.x, g.odo.y, th);
        s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, side, &xk, &yk);
        g.line_y = yk + ((float)g.cap_dir * (float)S5_LINE_HALF_MM);
    }
    g.b_line = true;
    g.line_seen = g.odo.dist;
    g.path.y_end = g.line_y;
    g.cap_step = 0U;
    g.cap_from = g.odo.dist;
    g.b_cap_seen = false;
    g.b_cap_wide = false;
    g.b_cap_back = false;
    s5_enter(S5_PH_CAPTURE);
    return s5_do_capture(p_m, b_ir_l, b_ir_r);
}

/* both wheels under S5_STILL_MM_S for S5_STILL_MS (g.still_ms counts) */
static bool s5_standing(const motion_status_t *p_m)
{
    const bool b_still = (p_m->speed_l_mmps < S5_STILL_MM_S) &&
                         (p_m->speed_l_mmps > -S5_STILL_MM_S) &&
                         (p_m->speed_r_mmps < S5_STILL_MM_S) &&
                         (p_m->speed_r_mmps > -S5_STILL_MM_S);

    g.still_ms = b_still ? (g.still_ms + g.dt_ms) : 0U;
    return (g.still_ms >= (uint32_t)S5_STILL_MS);
}

/* the turn onto the line has ended: stand still, then
 * Buddy 3 takes over. Handed over while still turning, the car on the
 * track went on turning past the line (Buddy 2 lets the wheel that was
 * going backwards coast down before it drives it forwards) */
static s5_avoid_result_t s5_cap_end(const char *p_why)
{
    g.p_cap_why = p_why;
    g.cap_step = 4U;
    s5_enter(S5_PH_CAPTURE); /* the stop's own clock */
    return S5_AVOID_RUNNING;  /* wheels 0 */
}

/* turning onto the line: "straight" is pointing the IRs back at the line
 * from where the axle ended up, up to S5_CAP_AIM_DEG (rad) */
static float s5_cap_aim(void)
{
    const float lim = S5_CAP_AIM_DEG * S5_DEG2RAD_F;
    float aim = s5_atan2f(g.line_y - g.odo.y, (float)S5_IR_AHEAD_MM);

    aim = (aim > lim) ? lim : aim;
    aim = (aim < -lim) ? -lim : aim;
    return aim;
}

static s5_avoid_result_t s5_do_capture(const motion_status_t *p_m,
                                       bool b_ir_l, bool b_ir_r)
{
    const float psi = g.odo.psi;
    /* the IR on the side the car came from meets the tape first as the
     * car turns to face along the line; the other one means turned past.
     * only black where our line is counts (on the track
     * a junction's branch beside it stopped the turn early) */
    const float near_side = (g.cap_dir > 0) ? -(float)S5_IR_HALF_GAP_MM
                                            : (float)S5_IR_HALF_GAP_MM;
    float xn;
    float yn;
    float xf;
    float yf;
    bool b_near;
    bool b_far;
    int16_t w;

    s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, near_side, &xn, &yn);
    s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, -near_side, &xf, &yf);
    b_near = ((g.cap_dir > 0) ? b_ir_r : b_ir_l) &&
             (s5_absf(yn - g.line_y) <= S5_CAP_GATE_MM);
    b_far = ((g.cap_dir > 0) ? b_ir_l : b_ir_r) &&
            (s5_absf(yf - g.line_y) <= S5_CAP_GATE_MM);
    if (g.b_search || (g.b_cap_phi && !g.b_cap_wide))
    {
        /* the line may be a bend, which is not where the
         * straight line through the crossing runs: the IRs themselves
         * decide (S5_CAP_BEND_DEG) */
        b_near = (g.cap_dir > 0) ? b_ir_r : b_ir_l;
        b_far = (g.cap_dir > 0) ? b_ir_l : b_ir_r;
    }

    if (0U == g.cap_step)
    {
        /* drive on, straight, until the axle is on the line (the wheels
         * run on the last S5_CAP_COAST_MM) */
        g.p_where = "ahead on the way back";
        if (g.b_search)
        {
            if (s5_search_held())
            {
                return S5_AVOID_RUNNING;
            }
        }
        else if (s5_next_ahead(p_m))
        {
            return s5_next_object();
        }
        else
        {
            /* nothing in the way */
        }
        if ((((float)g.cap_dir * (g.odo.y - g.line_y)) <
             -(S5_CAP_COAST_MM * s5_absf(s5_sinf(g.odo.psi)))) &&
            ((g.odo.dist - g.cap_from) < S5_CAP_REACH_MM))
        {
            s5_wheels((float)S5_SLOW_MM_S, 0.0f);
            return S5_AVOID_RUNNING;
        }
        S5_LOG("[VC-S5] axle on the line (%d mm on): turning on the spot "
               "to face along it\n",
               (int)s5_mm(g.odo.dist - g.cap_from));
        g.cap_step = 3U;
        s5_enter(S5_PH_CAPTURE); /* the stop's own clock */
    }
    if (3U == g.cap_step)
    {
        /* stand still first. On the car the wheel that
         * has to reverse straight from driving on gave no ticks for 0.6 s
         * (Buddy 2 lets it coast down before it turns it the other way);
         * the turn from standing at the start of a bypass never did. */
        if (!s5_standing(p_m) && (g.ph_ms < (uint32_t)S5_STOP_WAIT_MS))
        {
            return S5_AVOID_RUNNING;
        }
        g.cap_step = 1U;
        g.b_ir_black[0] = b_ir_l;
        g.b_ir_black[1] = b_ir_r;
        s5_enter(S5_PH_CAPTURE); /* the turn's own clock */
        if (!g.b_search && g.b_cap_phi && (b_ir_l != b_ir_r) &&
            (((float)g.cap_dir * (psi - s5_cap_aim())) <=
             (S5_CAP_BEND_DEG * S5_DEG2RAD_F)))
        {
            /* an IR is on the line already, and the car
             * is near its direction: the line bends that way. Turn
             * towards that IR until the tape is between the IRs, as a
             * line search does (host test C66) */
            const int32_t dir = b_ir_l ? -1 : 1;

            if (dir != g.cap_dir)
            {
                S5_LOG("[VC-S5] IR_%c is on the line already (it bends "
                       "that way): turning %s until the tape is between "
                       "the IRs\n",
                       b_ir_l ? 'L' : 'R', s5_side_name(b_ir_l ? 1 : -1));
                g.cap_dir = dir;
                g.b_cap_back = true;
                g.cap_psi0 = psi;
                g.b_cap_seen = true;
                return S5_AVOID_RUNNING; /* near and far swapped */
            }
            g.b_cap_seen = true;
        }
    }
    if (4U == g.cap_step)
    {
        if (!s5_standing(p_m) && (g.ph_ms < (uint32_t)S5_STOP_WAIT_MS))
        {
            return S5_AVOID_RUNNING;
        }
        return s5_done(g.p_cap_why);
    }
    if ((1U == g.cap_step) &&
        (g.b_search || (g.b_cap_phi && !g.b_cap_wide)))
    {
        /* still to turn to the direction the crossing gave (a search) or
         * to pointing the IRs back at the line from where the axle is */
        float left = (float)g.cap_dir * psi;
        float past = (float)S5_SEARCH_PAST_DEG * S5_DEG2RAD_F;

        if (g.b_cap_back)
        {
            /* turned back towards the line: at most S5_CAP_BEND_DEG */
            left = (float)g.cap_dir * (psi - g.cap_psi0);
            past = S5_CAP_BEND_DEG * S5_DEG2RAD_F;
        }
        else if (!g.b_search)
        {
            left = (float)g.cap_dir * (psi - s5_cap_aim());
            past = S5_CAP_BEND_DEG * S5_DEG2RAD_F;
            if (left > past)
            {
                /* not near the line's direction yet: a junction's branch */
                b_near = false;
                b_far = false;
            }
            else if (b_far && (b_near || !g.b_cap_seen))
            {
                /* black wider than the IRs are apart, or
                 * under the far IR before the near one met any: not our
                 * line crossed, a junction (host tests C53, C54) */
                S5_LOG("[VC-S5] IR_%c black %s while turning onto the "
                       "line: a junction there, turning to where the IRs "
                       "had the line run\n",
                       (g.cap_dir > 0) ? 'L' : 'R',
                       b_near ? "with the other" : "before the other");
                g.b_cap_wide = true;
                g.b_cap_seen = false;
                b_near = false;
                b_far = false;
            }
            else
            {
                /* the IRs count */
            }
        }
        /* the turn onto the line (a
         * search's, and a bypass's) stops on the IRs, not at an angle.
         * The IR on the side the car turns towards going black means the
         * line is there; the turn
         * stops when the other IR goes black, or once the tape is between
         * them and the car has turned S5_CAP_EXTRA_DEG more (which allows
         * for the wheels running on). The IRs, 125 mm ahead of the axle,
         * meet a bend where its chord runs, up to asin(125 / 2R) off the
         * direction worked out from the crossing (R300: 12 deg, R200:
         * 18 deg): turning to that direction, as the bypass did, lost
         * every bend (host tests Q32..Q46, C61..C68). The direction only
         * limits the turn (S5_SEARCH_PAST_DEG or S5_CAP_BEND_DEG past
         * it). */
        if (b_near)
        {
            g.b_cap_seen = true;
        }
        if (b_far)
        {
            return s5_cap_end(g.b_search
                                  ? "back on the line (line search: the first "
                                    "IR met the tape, the turn stopped at the "
                                    "other)"
                                  : "back on the line (the first IR met the "
                                    "tape, the turn stopped at the other)");
        }
        if (g.b_cap_seen && !b_near)
        {
            /* the tape is between the IRs now: a little more */
            g.cap_step = 2U;
            g.cap_psi = psi;
        }
        else if ((left < -past) ||
                 (g.ph_ms > (g.b_search ? (uint32_t)S5_SEARCH_PIVOT_MS
                                        : S5_PIVOT_MS)))
        {
            return s5_cap_end(g.b_search
                                  ? "back on the line (line search: the turn "
                                    "did not find the tape, Buddy 3 looks)"
                                  : "back on the line (the turn did not find "
                                    "the tape, Buddy 3 looks)");
        }
        else
        {
            /* still turning */
        }
    }
    else if ((1U == g.cap_step) && g.b_cap_phi)
    {
        /* the line's direction is known from where the
         * two IRs met it, and the encoders' frame turned to it. At a
         * junction the IRs see its branch while the car turns, so they
         * count only near straight (or near pointing back at the line,
         * S5_CAP_AIM_DEG), and the encoders stop the turn a little past
         * it if they have not. only once both IRs were
         * black at once (g.b_cap_wide); else the IRs decide, above */
        const float left = (float)g.cap_dir * (psi - s5_cap_aim());

        if (left <= (S5_CAP_IR_DEG * S5_DEG2RAD_F))
        {
            if (b_near)
            {
                g.b_cap_seen = true;
            }
            if (b_far && !b_near)
            {
                return s5_cap_end("back on the line (picked up by the IRs, "
                                  "the tape under the far IR)");
            }
            if (g.b_cap_seen && !b_near)
            {
                /* the tape is between the IRs now: a little more */
                g.cap_step = 2U;
                g.cap_psi = psi;
            }
        }
        if ((1U == g.cap_step) &&
            (left <= -(S5_CAP_PAST_DEG * S5_DEG2RAD_F)))
        {
            return s5_cap_end("back on the line (picked up by the IRs, turned "
                              "to where they had it run)");
        }
        if ((1U == g.cap_step) &&
            (g.ph_ms > (g.b_search ? (uint32_t)S5_SEARCH_PIVOT_MS
                                   : S5_PIVOT_MS)))
        {
            return s5_cap_end("back on the line (picked up by the IRs; the "
                              "turn took too long, Buddy 3 looks)");
        }
    }
    else if (1U == g.cap_step)
    {
        if (b_near)
        {
            g.b_cap_seen = true;
        }
        if (b_far && !b_near)
        {
            return s5_cap_end("back on the line (picked up by the IRs, the "
                              "tape under the far IR)");
        }
        if (g.b_cap_seen && !b_near)
        {
            /* the tape is between the IRs now: a little more */
            g.cap_step = 2U;
            g.cap_psi = psi;
        }
        else if (((float)g.cap_dir * psi) <
                     -(S5_CAP_OVER_DEG * S5_DEG2RAD_F) ||
                 (g.ph_ms > S5_PIVOT_MS))
        {
            return s5_cap_end("back on the line (picked up by the IRs; the "
                              "turn did not find the tape, Buddy 3 looks)");
        }
        else
        {
            /* still turning */
        }
    }
    if (2U == g.cap_step)
    {
        if ((s5_absf(psi - g.cap_psi) >= (S5_CAP_EXTRA_DEG * S5_DEG2RAD_F)) ||
            b_far)
        {
            return s5_cap_end("back on the line (picked up by the IRs)");
        }
    }
    /* a line search turns up to 90 deg, so it slows
     * down further out (S5_SEARCH_SLOW_DEG): a car that turns 10 % more
     * than the encoders say was still at full speed when it got there */
    w = ((2U == g.cap_step) || g.b_cap_seen ||
         (g.b_cap_phi &&
          (s5_absf(psi) < ((g.b_search ? (float)S5_SEARCH_SLOW_DEG
                                       : S5_PIVOT_SLOW_DEG) *
                           S5_DEG2RAD_F))))
            ? (int16_t)S5_PIVOT_SLOW_MM_S
            : (int16_t)S5_PIVOT_MM_S;
    /* turning towards straight: psi > 0 (crossing to the left) turns
     * right, left wheel forward */
    g.cmd_l = (g.cap_dir > 0) ? w : (int16_t)(-w);
    g.cmd_r = (g.cap_dir > 0) ? (int16_t)(-w) : w;
    return S5_AVOID_RUNNING;
}

/* line search. Both IRs have met a line: IR k just now
 * (on the tape's near edge), the other one where g.cap_x1 / cap_y1 say
 * (b_square: both at once, or it is too near square to tell which way it
 * runs). Turn the encoders' frame to the line and do the bypass's
 * pick-up: axle onto it, stand, turn on the spot to face along it. */
static s5_avoid_result_t s5_search_found(const motion_status_t *p_m,
                                         bool b_ir_l, bool b_ir_r, uint32_t k,
                                         bool b_square)
{
    const float psi = g.odo.psi;
    const float square = (90.0f - (float)S5_SEARCH_SQUARE_DEG) * S5_DEG2RAD_F;
    float rel = 0.0f; /* the line's direction to the car's heading */
    float apart = 0.0f;

    if (!b_square)
    {
        const float side =
            (0U == k) ? (float)S5_IR_HALF_GAP_MM : -(float)S5_IR_HALF_GAP_MM;
        float xk;
        float yk;
        float dx;
        float dy;

        s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, side, &xk, &yk);
        dx = xk - g.cap_x1;
        dy = yk - g.cap_y1;
        /* the way along the line that is forward for the car */
        if (((dx * s5_cosf(psi)) + (dy * s5_sinf(psi))) < 0.0f)
        {
            dx = -dx;
            dy = -dy;
        }
        apart = g.odo.dist - g.cap_first_at;
        rel = s5_wrapf(s5_atan2f(dy, dx) - psi);
        b_square = (s5_absf(rel) >= square);
    }
    if (b_square)
    {
        rel = (float)S5_SEARCH_TURN * 90.0f * S5_DEG2RAD_F;
        S5_LOG("[VC-S5] line search: both IRs met a line straight across "
               "(%d mm apart): onto it, then turning %s to face along it "
               "(S5_SEARCH_TURN)\n",
               (int)s5_mm(apart), s5_side_name((int32_t)S5_SEARCH_TURN));
    }
    else
    {
        S5_LOG("[VC-S5] line search: IR_%c then IR_%c met a line %d mm "
               "apart; it runs %d deg to the car: onto it, then turning %s "
               "to face along it\n",
               (0U == k) ? 'R' : 'L', (0U == k) ? 'L' : 'R',
               (int)s5_mm(apart), (int)s5_deg(s5_absf(rel)),
               s5_side_name((rel > 0.0f) ? 1 : -1));
    }
    g.cap_phi = s5_wrapf(psi + rel);
    g.b_cap_phi = true;
    g.cap_second = (int32_t)k;
    return s5_cap_start(p_m, b_ir_l, b_ir_r);
}

/* line search, the straight run. Watches where each IR
 * goes black:
 *   both at once                    a line straight across
 *   one, then the other within      a line at an angle: its direction
 *     S5_SEARCH_PAIR_MM             from where the two met it
 *   one, black for                  a line along the car (or the car
 *     S5_SEARCH_ALONG_MM            started on one): Buddy 3 follows
 *   one, white again, and not the   a mark, not a line: on
 *     other within PAIR
 */
static s5_avoid_result_t s5_do_search(bool b_ir_l, bool b_ir_r,
                                      const motion_status_t *p_m)
{
    const bool b_black[2] = { b_ir_l, b_ir_r };
    bool b_new[2];
    s5_ref_t ref;
    uint32_t k;

    for (k = 0U; k < 2U; k++)
    {
        b_new[k] = b_black[k] && !g.b_ir_black[k];
        if (b_new[k])
        {
            g.search_on[k] = g.odo.dist;
        }
        g.b_ir_black[k] = b_black[k];
    }
    if (g.b_search_first)
    {
        g.b_search_first = false;
        if (b_ir_l != b_ir_r)
        {
            /* started with one IR on a line. Turn on
             * the spot towards that IR until the tape is between the IRs
             * (s5_do_capture()), so Buddy 3 starts with the
             * line in the middle: on a bend the line was soon past the IR
             * and Buddy 3, seeing both white, went straight on (host test
             * Q22, IRs 25 mm apart) */
            S5_LOG("[VC-S5] line search: IR_%c is on a line already: "
                   "turning %s on the spot until the tape is between the "
                   "IRs\n",
                   b_ir_l ? 'L' : 'R', s5_side_name(b_ir_l ? 1 : -1));
            g.cap_dir = b_ir_l ? -1 : 1; /* > 0 turns right */
            g.b_mag = false;
            s5_odom_set_pose(&g.odo, g.odo.x, g.odo.y, 0.0f);
            g.line_y = 0.0f;
            g.b_line = true;
            g.line_seen = g.odo.dist;
            g.path.y_end = g.line_y;
            g.cap_from = g.odo.dist;
            g.cap_step = 3U; /* stand, then turn */
            g.b_cap_seen = true;
            s5_enter(S5_PH_CAPTURE);
            return s5_do_capture(p_m, b_ir_l, b_ir_r);
        }
    }
    if (b_new[0] && b_new[1])
    {
        return s5_search_found(p_m, b_ir_l, b_ir_r, 0U, true);
    }
    for (k = 0U; k < 2U; k++)
    {
        const uint32_t o = 1U - k;

        if (!b_new[k])
        {
            continue;
        }
        if ((g.cap_first >= 0) && ((uint32_t)g.cap_first == o))
        {
            return s5_search_found(p_m, b_ir_l, b_ir_r, k, false);
        }
        if (b_black[o])
        {
            /* the other one is black, and its start was not seen */
            return s5_search_found(p_m, b_ir_l, b_ir_r, k, true);
        }
        if (g.cap_first < 0)
        {
            const float side = (0U == k) ? (float)S5_IR_HALF_GAP_MM
                                         : -(float)S5_IR_HALF_GAP_MM;

            g.cap_first = (int32_t)k;
            g.cap_first_at = g.odo.dist;
            s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, side, &g.cap_x1,
                          &g.cap_y1);
        }
    }
    if (g.cap_first >= 0)
    {
        const uint32_t f = (uint32_t)g.cap_first;

        if (b_black[f] &&
            ((g.odo.dist - g.search_on[f]) >= (float)S5_SEARCH_ALONG_MM))
        {
            S5_LOG("[VC-S5] line search: IR_%c black for %d mm and IR_%c "
                   "never: a line along the car\n",
                   (0U == f) ? 'L' : 'R', (int)S5_SEARCH_ALONG_MM,
                   (0U == f) ? 'R' : 'L');
            return s5_done("line search: on a line");
        }
        if ((g.odo.dist - g.cap_first_at) > (float)S5_SEARCH_PAIR_MM)
        {
            S5_LOG("[VC-S5] line search: only IR_%c saw black (%d mm "
                   "back): a mark, not a line; on\n",
                   (0U == f) ? 'L' : 'R',
                   (int)s5_mm(g.odo.dist - g.cap_first_at));
            g.cap_first = -1;
        }
    }
    if ((g.cap_first < 0) &&
        ((g.odo.dist - g.search_from) > (float)S5_SEARCH_MAX_MM))
    {
        return s5_fail("no line within S5_SEARCH_MAX_MM of the start");
    }
    if (s5_search_held())
    {
        return S5_AVOID_RUNNING;
    }
    /* straight on: hold the heading it started with */
    ref.y = g.odo.y;
    ref.slope = 0.0f;
    ref.curv = 0.0f;
    s5_wheels((float)S5_SEARCH_MM_S,
              s5_path_steer(&ref, g.odo.y, g.odo.psi, (float)S5_ALIGN_DIST_MM));
    return S5_AVOID_RUNNING;
}

void s5_search_begin(const motion_status_t *p_m)
{
    s5_odom_update(&g.odo, p_m->ticks_l, p_m->ticks_r);
    s5_odom_set_pose(&g.odo, 0.0f, 0.0f, 0.0f);
    g.h_n = 0U;
    g.ep_ms = 0U;
    g.ep_dist = g.odo.dist;
    g.b_mag = false;
    g.b_mag_fixed = true; /* no compass in a search */
    g.mag_n = 0U;
    g.mag_corr = 0.0f;
    g.mag_err = 0.0f;
    g.mag_skip = 0U;
    g.reversed = 0.0f;
    g.rescans = 0U;
    g.b_blocked = false;
    g.b_block_said = false;
    g.block_ms = 0U;
    g.line_est = 0.0f;
    g.b_rejoin = false;
    g.b_passed = false;
    g.n_next = 0U;
    g.b_turn_check = false;
    g.prev_l = 0;
    g.prev_r = 0;
    g.dead_l_ms = 0U;
    g.dead_r_ms = 0U;
    g.wheel_tries = 0U;
    g.rest_ms = 0U;
    g.last_tl = p_m->ticks_l;
    g.last_tr = p_m->ticks_r;
    g.b_line = false;
    g.line_y = 0.0f;
    g.line_win = (float)S5_LINE_WINDOW_MM;
    g.b_ir_black[0] = false;
    g.b_ir_black[1] = false;
    g.b_ir_ok[0] = false;
    g.b_ir_ok[1] = false;
    g.cap_first = -1;
    g.b_cap_phi = false;
    g.b_cap_skip = false;
    g.b_search = true;
    g.b_search_first = true;
    g.b_search_held = false;
    g.search_from = g.odo.dist;
    s5_sensor_guard();
    s5_look(0);
    S5_LOG("[VC-S5] line search: straight on at %d mm/s until the IRs meet "
           "a line (at most %d mm), then onto it and along it\n",
           (int)S5_SEARCH_MM_S, (int)S5_SEARCH_MAX_MM);
    s5_enter(S5_PH_SEARCH);
}

/* Can the car turn on the spot from its heading to the line's direction
 * without coming within S5_PIVOT_MARGIN_MM of the object it has just gone
 * round? */
static bool s5_pivot_clear(void)
{
    const float step = S5_PIVOT_STEP_DEG * S5_DEG2RAD_F;
    float a = g.odo.psi;

    if (!g.b_passed)
    {
        return true;
    }
    for (;;)
    {
        if (s5_path_car_hits(g.odo.x, g.odo.y, a, &g.passed,
                             S5_PIVOT_MARGIN_MM))
        {
            return false;
        }
        if (s5_absf(a) <= step)
        {
            break;
        }
        a += (a > 0.0f) ? -step : step;
    }
    return !s5_path_car_hits(g.odo.x, g.odo.y, 0.0f, &g.passed,
                             S5_PIVOT_MARGIN_MM);
}

/* Turning on the spot to the line's direction, then the sweep. */
static s5_avoid_result_t s5_do_turn(void)
{
    const float psi = g.odo.psi;
    const float ap = s5_absf(psi);
    int16_t w;

    if ((ap <= (S5_PIVOT_DONE_DEG * S5_DEG2RAD_F)) || (g.ph_ms > S5_PIVOT_MS))
    {
        /* straight now: look how close the object is (back off if under
         * S5_SCAN_MIN_MM) before the sweep; turned too long: sweep */
        g.then = (ap <= (S5_PIVOT_DONE_DEG * S5_DEG2RAD_F)) ? S5_THEN_FACE
                                                            : S5_THEN_SCAN;
        s5_enter(S5_PH_STOP);
        return S5_AVOID_RUNNING;
    }
    w = (ap > (S5_PIVOT_SLOW_DEG * S5_DEG2RAD_F)) ? (int16_t)S5_PIVOT_MM_S
                                                  : (int16_t)S5_PIVOT_SLOW_MM_S;
    /* psi < 0: turn left (right wheel forward) */
    g.cmd_l = (psi < 0.0f) ? (int16_t)(-w) : w;
    g.cmd_r = (psi < 0.0f) ? w : (int16_t)(-w);
    return S5_AVOID_RUNNING;
}

/* too close for a ramp out (s5_plan()): turn on the
 * spot to g.out_th, stand, drive straight out until the axle is at the
 * pass, stand, turn back on the spot to along the line, stand, then
 * follow the rest of the path. Stands between, as before the turn onto
 * the line: Buddy 2 lets a wheel that changes direction coast down
 * first. */
static s5_avoid_result_t s5_do_out(const motion_status_t *p_m)
{
    const float psi = g.odo.psi;

    if ((1U == g.out_step) || (3U == g.out_step) || (5U == g.out_step))
    {
        if (!s5_standing(p_m) && (g.ph_ms < (uint32_t)S5_STOP_WAIT_MS))
        {
            return S5_AVOID_RUNNING; /* wheels 0 */
        }
        if (5U == g.out_step)
        {
            /* the pass from here: the ramp is only what is left over */
            g.path.x0 = g.odo.x;
            g.path.y0 = g.odo.y;
            g.path.l_out = s5_path_ramp_len(g.path.a - g.odo.y);
            s5_plan_back(g.odo.x);
            s5_start_bypass();
            return S5_AVOID_RUNNING;
        }
        g.out_step++;
        g.out_from = g.odo.dist;
        s5_enter(S5_PH_OUT);
        return S5_AVOID_RUNNING;
    }
    if (2U == g.out_step)
    {
        const float s0 = (float)S5_TRACK_DIST_MM;
        const float rate = ((2.0f * (float)S5_SLOW_MM_S) / s0) *
                           s5_sinf(g.out_th - psi);

        /* at the pass, or (the encoders' heading off) 1.5 times as far
         * as planned: (S5_MAX_OFFSET_MM / sin 45) */
        if ((((float)g.side * (g.odo.y - g.path.a)) >= 0.0f) ||
            ((g.odo.dist - g.out_from) >
             (1.5f * (float)S5_MAX_OFFSET_MM * 1.42f)))
        {
            g.out_step = 3U;
            s5_enter(S5_PH_OUT);
            return S5_AVOID_RUNNING;
        }
        s5_wheels((float)S5_SLOW_MM_S, rate / (float)S5_SLOW_MM_S);
        return S5_AVOID_RUNNING;
    }
    {
        /* 0: turning out, 4: turning back */
        const float want = (0U == g.out_step) ? g.out_th : 0.0f;
        const float err = s5_wrapf(want - psi); /* + = turn left */
        const float ae = s5_absf(err);
        int16_t w;

        if ((ae <= (S5_PIVOT_DONE_DEG * S5_DEG2RAD_F)) ||
            (g.ph_ms > S5_PIVOT_MS))
        {
            g.out_step++;
            s5_enter(S5_PH_OUT);
            return S5_AVOID_RUNNING;
        }
        w = (ae > (S5_PIVOT_SLOW_DEG * S5_DEG2RAD_F))
                ? (int16_t)S5_PIVOT_MM_S
                : (int16_t)S5_PIVOT_SLOW_MM_S;
        g.cmd_l = (err > 0.0f) ? (int16_t)(-w) : w;
        g.cmd_r = (err > 0.0f) ? w : (int16_t)(-w);
    }
    return S5_AVOID_RUNNING;
}

static s5_avoid_result_t s5_do_stop(const motion_status_t *p_m, uint32_t dt)
{
    const bool b_still = (p_m->speed_l_mmps < S5_STILL_MM_S) &&
                         (p_m->speed_l_mmps > -S5_STILL_MM_S) &&
                         (p_m->speed_r_mmps < S5_STILL_MM_S) &&
                         (p_m->speed_r_mmps > -S5_STILL_MM_S);
    s5_ping_t p;

    g.still_ms = b_still ? (g.still_ms + dt) : 0U;
    if ((g.still_ms < (uint32_t)S5_STILL_MS) &&
        (g.ph_ms < (uint32_t)S5_STOP_WAIT_MS))
    {
        return S5_AVOID_RUNNING;
    }
    switch (g.then)
    {
        case S5_THEN_FACE:
        {
            float d = 0.0f;

            /* however close it is, the sweep is from
             * here (it backed off to S5_SCAN_DIST_MM under S5_SCAN_MIN_MM);
             * the car backs up only when no way round is open */
            if (s5_get_ping(&p) && (0 == p.ddeg) && p.b_valid && s5_fresh(&p) &&
                (0U != p.mm) && (p.mm < (uint16_t)S5_SCAN_MIN_MM))
            {
                S5_LOG("[VC-S5] only %d mm away: sweeping from here\n",
                       (int)p.mm);
            }
            if (s5_absf(g.odo.psi) > (S5_SQUARE_DEG * S5_DEG2RAD_F))
            {
                /* stopped on the way back from the last object, turned
                 * towards the line: turn on the spot if there is room,
                 * else back up, which straightens the car too. Room: the
                 * object just gone round clear of the car's outline all
                 * the way round, and the sweep (s5_decide()) hears
                 * nothing else within S5_PIVOT_ROOM_MM (one ping ahead missed a
                 * box turned
                 * towards the car's front corner) */
                if ((d <= 0.0f) && s5_pivot_clear())
                {
                    g.b_turn_check = true;
                    s5_scan_start();
                    break;
                }
                S5_LOG("[VC-S5] the car is %d deg to the line: backing up "
                       "to straighten first\n",
                       (int)s5_deg(g.odo.psi));
                d = (d > S5_SQUARE_MM) ? d : S5_SQUARE_MM;
            }
            if (d > 0.0f)
            {
                s5_back_by(d, S5_THEN_SCAN);
            }
            else
            {
                s5_scan_start();
            }
            break;
        }
        case S5_THEN_BYPASS:
            s5_start_bypass();
            break;
        case S5_THEN_SCAN:
        default:
            s5_scan_start();
            break;
    }
    return S5_AVOID_RUNNING;
}

static s5_avoid_result_t s5_do_back(void)
{
    if ((g.odo.x <= g.back_x) || s5_back_hits(S5_BACK_STEP_MM))
    {
        g.reversed += g.back_from - g.odo.x;
        g.then = g.back_then;
        s5_enter(S5_PH_STOP);
        return S5_AVOID_RUNNING;
    }
    if ((g.reversed + (g.back_from - g.odo.x)) >
        ((float)S5_REVERSE_MAX_MM + 20.0f))
    {
        return s5_fail("backed up the most it may (S5_REVERSE_MAX_MM)");
    }
    s5_reverse_step();
    return S5_AVOID_RUNNING;
}

static s5_avoid_result_t s5_do_scan(void)
{
    if (s5_sensor_scan_get(g.scan_seq, &g.sweep))
    {
        s5_sensor_guard();
        return s5_decide();
    }
    if (g.ph_ms > (uint32_t)S5_SCAN_TIMEOUT_MS)
    {
        return s5_fail("the sweep did not finish");
    }
    return S5_AVOID_RUNNING;
}

static void s5_enter_end(void)
{
    if (g.b_line && (g.cap_first < 0))
    {
        g.align_from = g.odo.dist;
        s5_enter(S5_PH_ALIGN);
    }
    else if (g.b_line)
    {
        /* one IR on a line, the other not yet: on until
         * it is (or it turns out to be a junction's branch) */
        g.seek_from = g.odo.dist;
        s5_enter(S5_PH_SEEK);
    }
    else
    {
        S5_LOG("[VC-S5] the line is not where the encoders put it: "
               "looking on at %d deg\n",
               (int)S5_SEEK_DEG);
        if (!g.b_ret_black)
        {
            /* no IR has seen black on the way back, so
             * the car has not crossed our line yet: the first line ahead
             * is ours, however far the encoders have drifted */
            S5_LOG("[VC-S5] no black on the way back: the first line "
                   "ahead is ours\n");
            g.line_win = (float)S5_SEEK_WINDOW_MM;
            g.b_seek_ours = true; /* Buddy 5, 2026-11-16 */
        }
        g.seek_stage = 0U;
        g.seek_from = g.odo.dist;
        s5_enter(S5_PH_SEEK);
    }
}

static s5_avoid_result_t s5_do_seek(const motion_status_t *p_m, bool b_ir_l,
                                    bool b_ir_r);
static s5_avoid_result_t s5_do_align(const motion_status_t *p_m, bool b_ir_l,
                                     bool b_ir_r);

static s5_avoid_result_t s5_do_bypass(const motion_status_t *p_m, bool b_ir_l,
                                      bool b_ir_r)
{
    s5_ref_t ref;
    float v = (float)S5_SPEED_MM_S;

    if (!s5_bypass_sensor(p_m))
    {
        return s5_next_object();
    }
    if (g.odo.x >= g.path.x_back)
    {
        if (s5_cap_watch(b_ir_l, b_ir_r))
        {
            return s5_cap_start(p_m, b_ir_l, b_ir_r);
        }
        s5_ir_watch(b_ir_l, b_ir_r, -g.side);
    }
    if (g.odo.x >= s5_path_end(&g.path))
    {
        /* the next phase drives this pass already: a pass with no wheel
         * speeds would be a (0, 0), and Buddy 2 brakes to a stop */
        s5_enter_end();
        return (S5_PH_ALIGN == g.ph) ? s5_do_align(p_m, b_ir_l, b_ir_r)
                                     : s5_do_seek(p_m, b_ir_l, b_ir_r);
    }
    s5_path_eval(&g.path, g.odo.x, &ref);
    if (g.odo.x > (s5_path_end(&g.path) - 100.0f))
    {
        v = (float)S5_SLOW_MM_S;
    }
    s5_wheels(v,
              s5_path_steer(&ref, g.odo.y, g.odo.psi, (float)S5_TRACK_DIST_MM));
    return S5_AVOID_RUNNING;
}

static s5_avoid_result_t s5_do_seek(const motion_status_t *p_m, bool b_ir_l,
                                    bool b_ir_r)
{
    s5_ref_t ref;

    if (!s5_bypass_sensor(p_m))
    {
        return s5_next_object();
    }
    if (s5_cap_watch(b_ir_l, b_ir_r))
    {
        return s5_cap_start(p_m, b_ir_l, b_ir_r);
    }
    s5_ir_watch(b_ir_l, b_ir_r, -g.side);
    /* one IR on the line, at an angle to it: straight on
     * until the other one crosses too (the pick-up), as the encoders'
     * idea of the line's direction may be well off by now */
    if (g.b_line && (g.cap_first < 0))
    {
        g.align_from = g.odo.dist;
        s5_enter(S5_PH_ALIGN);
        return s5_do_align(p_m, b_ir_l, b_ir_r);
    }
    /* Buddy 5, 2026-11-16: a second, steeper try before giving up */
    if (!g.b_line && (0U == g.seek_stage) && (S5_SEEK2_MM > 0) &&
        ((g.odo.dist - g.seek_from) > (float)S5_SEEK_MAX_MM))
    {
        S5_LOG("[VC-S5] no line after %d mm at %d deg: turning in to %d deg "
               "for %d mm more%s\n",
               (int)S5_SEEK_MAX_MM, (int)S5_SEEK_DEG, (int)S5_SEEK2_DEG,
               (int)S5_SEEK2_MM,
               g.b_ret_black ? "" : " (the first line met is ours)");
        g.seek_stage = 1U;
        g.seek_from = g.odo.dist;
        if (!g.b_ret_black)
        {
            g.b_seek_ours = true;
            g.line_win = (float)S5_SEEK2_WINDOW_MM;
        }
    }
    if (!g.b_line &&
        ((g.odo.dist - g.seek_from) >
         (float)((0U == g.seek_stage) ? S5_SEEK_MAX_MM : S5_SEEK2_MM)))
    {
        return s5_fail("the line was not found (FR24)");
    }
    ref.y = g.odo.y; /* heading only */
    ref.slope = -(float)g.side *
                s5_tanf((float)((0U == g.seek_stage) ? S5_SEEK_DEG
                                                     : S5_SEEK2_DEG) *
                        S5_DEG2RAD_F);
    ref.curv = 0.0f;
    s5_wheels((float)S5_SLOW_MM_S,
              s5_path_steer(&ref, g.odo.y, g.odo.psi, (float)S5_ALIGN_DIST_MM));
    return S5_AVOID_RUNNING;
}

static s5_avoid_result_t s5_do_align(const motion_status_t *p_m, bool b_ir_l,
                                     bool b_ir_r)
{
    s5_ref_t ref;
    float xc;
    float yc;
    const float psi = s5_absf(g.odo.psi);
    bool b_cap;

    g.p_where = "ahead on the way back";
    if (s5_next_ahead(p_m))
    {
        return s5_next_object();
    }
    b_cap = s5_cap_watch(b_ir_l, b_ir_r);
    if (!g.b_line)
    {
        /* that was a junction's branch: look on */
        S5_LOG("[VC-S5] the line is not where the encoders put it: "
               "looking on at %d deg\n",
               (int)S5_SEEK_DEG);
        g.seek_from = g.odo.dist;
        s5_enter(S5_PH_SEEK);
        return s5_do_seek(p_m, b_ir_l, b_ir_r);
    }
    s5_ir_watch(b_ir_l, b_ir_r, 0);
    s5_odom_point(&g.odo, (float)S5_IR_AHEAD_MM, 0.0f, &xc, &yc);
    /* Buddy 3 takes over as soon as one IR
     * is on our line and the car is near its direction (it steers by the
     * IRs from there, also where the line bends), or with both IRs white
     * once the encoders say the pair is centred and an IR saw our line
     * within the last S5_FRESH_MM. Never on dead reckoning alone. */
    if ((b_ir_l != b_ir_r) && g.b_ir_ok[b_ir_l ? 0U : 1U] &&
        (psi <= (S5_HANDOVER_DEG * S5_DEG2RAD_F)))
    {
        return s5_done("back on the line");
    }
    if (!b_ir_l && !b_ir_r &&
        (s5_absf(yc - g.line_y) <= (float)S5_ALIGN_TOL_MM) &&
        (psi <= ((float)S5_ALIGN_TOL_DEG * S5_DEG2RAD_F)) &&
        ((g.odo.dist - g.line_seen) <= S5_FRESH_MM))
    {
        return s5_done("back on the line");
    }
    if ((g.odo.dist - g.align_from) >= (float)S5_ALIGN_MAX_MM)
    {
        return s5_fail("lost the line after the bypass: the IRs do not see "
                       "it where it should be (FR24)");
    }
    if (b_cap)
    {
        return s5_cap_start(p_m, b_ir_l, b_ir_r);
    }
    ref.y = g.line_y;
    ref.slope = 0.0f;
    ref.curv = 0.0f;
    s5_wheels((float)S5_SLOW_MM_S,
              s5_path_steer(&ref, g.odo.y, g.odo.psi, (float)S5_ALIGN_DIST_MM));
    return S5_AVOID_RUNNING;
}

static s5_avoid_result_t s5_do_blocked(uint32_t dt)
{
    s5_ping_t p;

    if (s5_get_ping(&p) && (0 == p.ddeg) && p.b_valid && s5_fresh(&p))
    {
        if ((0U == p.mm) || (p.mm > (uint16_t)S5_CLEAR_MM))
        {
            g.clear_ms += dt;
        }
        else
        {
            g.clear_ms = 0U;
        }
    }
    if (g.clear_ms >= (uint32_t)S5_CLEAR_MS)
    {
        return s5_end_here("the way ahead is clear");
    }
    if (g.ph_ms >= (uint32_t)S5_BLOCKED_SCAN_MS)
    {
        g.rescans = S5_MAX_RESCANS; /* look again from here */
        s5_scan_start();
    }
    return S5_AVOID_RUNNING;
}

/* what the bypass was doing, for the log */
static const char *s5_ph_name(void)
{
    const char *p_name;

    switch (g.ph)
    {
        case S5_PH_STOP:
            p_name = "standing";
            break;
        case S5_PH_BACK:
            p_name = "backing up";
            break;
        case S5_PH_TURN:
            p_name = "turning on the spot to straighten";
            break;
        case S5_PH_SCAN:
            p_name = "sweeping";
            break;
        case S5_PH_BYPASS:
            p_name = "going round";
            break;
        case S5_PH_SEEK:
            p_name = "looking for the line";
            break;
        case S5_PH_ALIGN:
            p_name = "settling on the line";
            break;
        case S5_PH_CAPTURE:
            p_name = (0U == g.cap_step)   ? "driving onto the line"
                     : (3U == g.cap_step) ? "stopping on the line"
                                          : "turning on the spot onto the line";
            break;
        case S5_PH_BLOCKED:
            p_name = "waiting (no way round)";
            break;
        case S5_PH_SEARCH:
            p_name = "looking for a line (line search)";
            break;
        case S5_PH_OUT:
            p_name = "turning out on the spot";
            break;
        default:
            p_name = "idle";
            break;
    }
    return p_name;
}

/* A wheel asked to turn that gives no ticks (FR: dead encoder). */
static bool s5_wheel_dead(const motion_status_t *p_m, uint32_t dt)
{
    const bool b_drive = (MOTION_STATE_DRIVING == p_m->state);

    if (b_drive &&
        ((g.prev_l >= S5_MIN_WHEEL_MM_S) || (g.prev_l <= -S5_MIN_WHEEL_MM_S)) &&
        (p_m->ticks_l == g.last_tl))
    {
        g.dead_l_ms += dt;
    }
    else
    {
        g.dead_l_ms = 0U;
    }
    if (b_drive &&
        ((g.prev_r >= S5_MIN_WHEEL_MM_S) || (g.prev_r <= -S5_MIN_WHEEL_MM_S)) &&
        (p_m->ticks_r == g.last_tr))
    {
        g.dead_r_ms += dt;
    }
    else
    {
        g.dead_r_ms = 0U;
    }
    g.last_tl = p_m->ticks_l;
    g.last_tr = p_m->ticks_r;
    return (g.dead_l_ms >= (uint32_t)S5_DEAD_WHEEL_MS) ||
           (g.dead_r_ms >= (uint32_t)S5_DEAD_WHEEL_MS);
}

s5_avoid_result_t s5_avoid_step(const motion_status_t *p_m, bool b_ir_l,
                                bool b_ir_r, int32_t dt_ms, int16_t *p_l,
                                int16_t *p_r)
{
    const uint32_t dt = (dt_ms > 0) ? (uint32_t)dt_ms : 1U;
    s5_avoid_result_t res = S5_AVOID_RUNNING;
    bool b_pivot;

    s5_odom_update(&g.odo, p_m->ticks_l, p_m->ticks_r);
    g.dt_ms = dt;
    s5_mag_step(dt);
    g.ph_ms += dt;
    /* the straight run of a line search has its own
     * limit (S5_SEARCH_MAX_MM) */
    if ((S5_PH_BLOCKED != g.ph) && (S5_PH_SEARCH != g.ph))
    {
        g.ep_ms += dt;
    }
    if (g.b_blocked)
    {
        g.block_ms += dt;
    }
    g.cmd_l = 0;
    g.cmd_r = 0;

    if (S5_PH_IDLE == g.ph)
    {
        res = S5_AVOID_DONE; /* not in an episode */
    }
    else if (s5_wheel_dead(p_m, dt))
    {
        /* which wheel, how fast it was asked to go, and
         * what the car was doing */
        S5_LOG("[VC-S5] wheels asked for L %d / R %d mm/s while %s; no "
               "ticks for L %d / R %d ms (%d deg, %d mm on)\n",
               (int)g.prev_l, (int)g.prev_r, s5_ph_name(),
               (int)g.dead_l_ms, (int)g.dead_r_ms, (int)s5_deg(g.odo.psi),
               (int)s5_mm(g.odo.dist - g.ep_dist));
        if (g.wheel_tries < (uint32_t)S5_WHEEL_TRIES)
        {
            /* a wheel that stalled (a turn on the spot from a slow
             * speed) moves again from standing; a dead encoder fails
             * again */
            g.wheel_tries++;
            g.dead_l_ms = 0U;
            g.dead_r_ms = 0U;
            g.rest_ms = (uint32_t)S5_WHEEL_REST_MS;
            S5_LOG("[VC-S5] stalled wheel? standing %d ms and going on "
                   "(try %d of %d)\n",
                   (int)S5_WHEEL_REST_MS, (int)g.wheel_tries,
                   (int)S5_WHEEL_TRIES);
        }
        else
        {
            res = s5_fail((g.dead_l_ms >= (uint32_t)S5_DEAD_WHEEL_MS)
                              ? "left wheel gives no encoder ticks"
                              : "right wheel gives no encoder ticks");
        }
    }
    else if (g.rest_ms > 0U)
    {
        g.rest_ms = (g.rest_ms > dt) ? (g.rest_ms - dt) : 0U; /* wheels 0 */
    }
    else if (g.ep_ms > (uint32_t)S5_EPISODE_MS)
    {
        res = s5_fail("took longer than S5_EPISODE_MS");
    }
    else
    {
        switch (g.ph)
        {
            case S5_PH_STOP:
                res = s5_do_stop(p_m, dt);
                break;
            case S5_PH_BACK:
                res = s5_do_back();
                break;
            case S5_PH_TURN:
                res = s5_do_turn();
                break;
            case S5_PH_SCAN:
                res = s5_do_scan();
                break;
            case S5_PH_BYPASS:
                res = s5_do_bypass(p_m, b_ir_l, b_ir_r);
                break;
            case S5_PH_SEEK:
                res = s5_do_seek(p_m, b_ir_l, b_ir_r);
                break;
            case S5_PH_ALIGN:
                res = s5_do_align(p_m, b_ir_l, b_ir_r);
                break;
            case S5_PH_CAPTURE:
                res = s5_do_capture(p_m, b_ir_l, b_ir_r);
                break;
            case S5_PH_BLOCKED:
                res = s5_do_blocked(dt);
                break;
            case S5_PH_SEARCH:
                res = s5_do_search(b_ir_l, b_ir_r, p_m);
                break;
            case S5_PH_OUT:
                res = s5_do_out(p_m);
                break;
            case S5_PH_IDLE:
            default:
                res = S5_AVOID_DONE;
                break;
        }
    }
    b_pivot = ((g.cmd_l > 0) && (g.cmd_r < 0)) ||
              ((g.cmd_l < 0) && (g.cmd_r > 0));
    if (b_pivot && ((g.dead_l_ms >= S5_PIVOT_KICK_MS) ||
                    (g.dead_r_ms >= S5_PIVOT_KICK_MS)))
    {
        const int16_t k = (int16_t)S5_PIVOT_KICK_MM_S;

        g.cmd_l = (g.cmd_l > 0) ? ((g.cmd_l > k) ? g.cmd_l : k)
                                : ((g.cmd_l < -k) ? g.cmd_l : (int16_t)(-k));
        g.cmd_r = (g.cmd_r > 0) ? ((g.cmd_r > k) ? g.cmd_r : k)
                                : ((g.cmd_r < -k) ? g.cmd_r : (int16_t)(-k));
    }
    g.prev_l = g.cmd_l;
    g.prev_r = g.cmd_r;
    *p_l = g.cmd_l;
    *p_r = g.cmd_r;
    return res;
}
