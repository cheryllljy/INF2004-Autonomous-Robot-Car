/**
 * @file    test_car.c
 * @brief   HOST TEST (S5-H07). 2-D closed-loop simulation of the whole
 *          obstacle episode: the REAL obstacle_task.c (S5) and the REAL
 *          vc_obstacle.c (VehicleController's obstacle part) drive a
 *          simulated car round box obstacles and back onto the line.
 *
 * What is real and what is simulated
 *   real       obstacle_task.c, obstacle_geometry.c, s5_pack.c (S5)
 *              vc_obstacle.c (bypass, reverse, rejoin, hard-stop cut)
 *   mirrored   the few lines of vehicle_controller.c that route S5
 *              events to vc_obstacle (read_obstacle(), the OBSTACLE case
 *              of act()): copied below, because the real file also pulls
 *              in Buddy 1's MQTT and Buddy 3's tasks
 *   simulated  the car (motion_mock.c: Buddy 2's API on a diff-drive
 *              model with optional encoder / track errors), the HC-SR04
 *              (beam cone ray-cast from the servo pivot on the moving
 *              car), the servo (optional trim / backlash), Buddy 3's IR
 *              pair straddling an 18 mm line, a simple line follower
 *              up to the obstacle
 *   real too   nyx, 2026-11-06: after VC hands back, Buddy 3's own line
 *              logic (b3_line.c) follows at its base speed for
 *              AFTER_MM, and the car must still be on the line: a
 *              handover with the line outside the IR pair looks "centred"
 *              to the straddle layout and the car drives straight off
 *
 * Every millisecond the car footprint is checked against every
 * obstacle: any overlap is a collision and fails the scenario. The
 * closest approach and the rejoin error (axle to line, at the moment VC
 * hands back to line following) are reported.
 *
 *   ./test_car            all scenarios
 *   ./test_car N [v]      scenario N, v = print the VC log
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "subsystem5.h"
#include "s5_calib.h"
#include "s5_config.h"
#include "sim_hal.h"
#include "motion_mock.h"
#include "vc_obstacle.h"
#include "b3_line.h"

#define PI              (3.14159265358979323846)

/* Chassis (axle-centred, mm). v3.1: nyx's car is about 180 x 130 mm.
 * nyx, 2026-11-06: the IRs are 125 mm ahead of the axle, so the front is
 * at least that far forward (was 110 / -70).
 * TODO(measure) the axle and servo positions on the real car. */
#define CAR_FRONT       (135.0)
#define CAR_REAR        (-45.0)
#define CAR_HALF_W      (65.0)
#define PIVOT_AHEAD     (80.0)      /* servo axis ahead of the axle */
#define IR_AHEAD        ((double)VC_OBS_IR_AHEAD_MM)
#define IR_SPREAD       (10.0)      /* each IR 10 mm off the centre line */
#define LINE_HALF       (9.0)       /* 18 mm tape */
#define CRUISE          (200)       /* line following, mm/s */

#define MAX_RECTS       (6)

/* nyx, 2026-11-06: after the handover Buddy 3 follows for this far; the
 * car must then be on the line (axle within AFTER_X_MM, heading within
 * AFTER_DEG, line between or under the IRs). */
#define AFTER_MM        (400.0)
#define AFTER_X_MM      (30.0)
#define AFTER_DEG       (15.0)

typedef struct { double x0, x1, y0, y1; } rect_t;

typedef struct
{
    const char *name;
    rect_t      r[MAX_RECTS];
    int         n;
    double      drop_at_s;      /* >0: r[0] appears this late, ...    */
    double      drop_gap_mm;    /* ...this far ahead of the sensor    */
    double      enc_scale;
    double      track_err;
    int         servo_err;      /* 1: trim -3.5, backlash 1.5, beam 12 */
    int         expect_fault;   /* 1: must end in a fault, still safe;
                                   2: either outcome, as long as safe */
    int         dead_enc;       /* v3.2: right encoder reads 0         */
    double      gap_y0;         /* v3.2: no line from here...          */
    double      gap_y1;         /* ...to here (0, 0: none)             */
    double      beam_deg;       /* nyx, 2026-11-06: true beam half-angle
                                   (0: the datasheet 7.5); a flat box face
                                   answers ~22 deg off-axis on the car */
    double      lane_mm;        /* nyx, 2026-11-06: other lanes' lines this
                                   far left and right (0: none) */
    int         cal_beam;       /* nyx, 2026-11-06: 1 = only in the build
                                   with the step 4 beam (test_car_cal) */
    double      ir_ahead_mm;    /* nyx, 2026-11-06: where the IRs really
                                   are (0: VC_OBS_IR_AHEAD_MM, which is
                                   still a TODO(measure) on the car) */
} scen_t;

/* Boxes are written centre x (+ = left), near face y, width, depth. */
#define BOX(cx, y, w, d) { (cx) - (w) / 2.0, (cx) + (w) / 2.0, (y), (y) + (d) }

static const scen_t g_scen[] =
{
    { "C1 box 100x150 on the line",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C2 box 100x150, 35 mm left of the line",
      { BOX(35, 800, 100, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C3 wide box 300x150 (fills the +/-30 deg view)",
      { BOX(0, 800, 300, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C4 long box 100x450 (v2's guessed depth was 150)",
      { BOX(0, 800, 100, 450) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C5 box on the line, wall 230 mm to the left",
      { BOX(0, 800, 100, 150), { 200, 260, 300, 1600 } }, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C6 box dropped 90 mm in front while driving",
      { BOX(0, 0, 100, 150) }, 1, 2.0, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C7 boxed in: box + walls both sides",
      { BOX(0, 800, 120, 150), { 150, 210, 300, 1600 }, { -210, -150, 300, 1600 } },
      3, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
    { "C8 C1 with encoders 2 % long, track 3 % wide",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0.02, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C9 C2 on a robot with servo trim/backlash and a 12 deg beam",
      { BOX(35, 800, 100, 150) }, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C10 second box in the left lane (v3.1 sees it, passes right)",
      { BOX(0, 800, 100, 150), BOX(210, 1300, 120, 100) }, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 },
    { "C11 C1 calibrated robot, 12 deg beam, odometry errors",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, -0.02, -0.03, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* v3.1: C12 and C13 sit in the first box's shadow, so no scan sees
     * them. C12 is found beside the pass lane (side step); C13 is found
     * ahead on the arc back (stop, rescan, replan). */
    { "C12 small box hidden behind the first, beside the lanes",
      { BOX(0, 800, 100, 150), BOX(50, 1300, 60, 60) }, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 },
    { "C13 second box on the line, hidden behind the first",
      { BOX(0, 800, 100, 150), BOX(0, 1450, 100, 150) }, 2, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 },
    /* nyx, 2026-11-06 (v3.2) */
    { "C14 C1 with odometry reading 25 % short",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, -0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C15 C1 with the right encoder dead (must stop, not wander)",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0 },
    { "C16 C1, no line where the car comes back (must stop, not drive on)",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, 0, 0, 1, 0, 1000, 4000, 0, 0, 0, 0 },
    { "C17 C1, turns slip: heading reads 12 % over (finds the line)",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, 0.12, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { "C18 C1, heading reads 12 % under (finds the line)",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, -0.12, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* nyx, 2026-11-06: the course's 21 cm box with the real flat-face
     * beam (~22 deg) and S5 still on the datasheet 7.5 deg: S5 sees it
     * about 35 cm wide, so the detour is far wider than needed. */
    { "C19 nyx's 210 mm box, 22 deg beam, beam not calibrated",
      { BOX(0, 800, 210, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22.0, 0, 0, 0 },
    /* nyx, 2026-11-06: the course has lanes. The wide detour of C19 goes
     * past the next lane's line; coming back, that line is crossed first
     * and must not be taken for the car's own. */
    { "C20 C19 with other lanes' lines 250 mm left and right",
      { BOX(0, 800, 210, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22.0, 250.0, 0, 0 },
    { "C21 C20 with the beam calibrated by step 4 (narrower detour)",
      { BOX(0, 800, 210, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22.0, 250.0, 1, 0 },
    { "C22 C1, IRs 150 mm ahead of the axle (VC assumes 125)",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 150.0 },
    { "C23 C1, IRs 100 mm ahead of the axle (VC assumes 125)",
      { BOX(0, 800, 100, 150) }, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100.0 },
};

#define N_SCEN ((int)(sizeof(g_scen) / sizeof(g_scen[0])))

static const scen_t *g_s;

/* nyx, 2026-11-06: built with the step 4 beam (S5_CAL_BEAM_HALF_DDEG
 * for the 22 deg flat-face beam), only the cal_beam scenarios run. */
#define CAL_BEAM_BUILD (S5_CAL_BEAM_HALF_DDEG >= 200)

/*=====================================================================*/
/* World                                                               */
/*=====================================================================*/

static rect_t g_rect[MAX_RECTS];
static int    g_nrect;
static int    g_dropped;

static void fwd(double psi, double *fx, double *fy)
{
    *fx = sin(psi * PI / 180.0);
    *fy = cos(psi * PI / 180.0);
}

static void body_to_world(double f, double l, double *wx, double *wy)
{
    double x, y, psi, fx, fy;

    mm_get_truth(&x, &y, &psi);
    fwd(psi, &fx, &fy);
    /* left unit vector in the x-left frame: (cos psi, -sin psi) */
    *wx = x + f * fx + l * fy;
    *wy = y + f * fy - l * fx;
}

/** Ray from the servo pivot; bearing relative to the sensor axis. */
static double car_ray(double bearing_deg)
{
    double px, py, x, y, psi, dx, dy;
    double best = 0.0;
    int i;

    mm_get_truth(&x, &y, &psi);
    body_to_world(PIVOT_AHEAD, 0.0, &px, &py);
    fwd(psi + bearing_deg, &dx, &dy);

    for (i = 0; i < g_nrect; i++)
    {
        const rect_t *r = &g_rect[i];
        double t0 = -1e9, t1 = 1e9;

        if (fabs(dx) < 1e-12)
        {
            if ((px < r->x0) || (px > r->x1)) { continue; }
        }
        else
        {
            double a = (r->x0 - px) / dx, b = (r->x1 - px) / dx;
            if (a > b) { double t = a; a = b; b = t; }
            if (a > t0) { t0 = a; }
            if (b < t1) { t1 = b; }
        }
        if (fabs(dy) < 1e-12)
        {
            if ((py < r->y0) || (py > r->y1)) { continue; }
        }
        else
        {
            double a = (r->y0 - py) / dy, b = (r->y1 - py) / dy;
            if (a > b) { double t = a; a = b; b = t; }
            if (a > t0) { t0 = a; }
            if (b < t1) { t1 = b; }
        }
        if ((t1 >= t0) && (t1 > 0.0))
        {
            const double t = (t0 > 0.0) ? t0 : 0.001;

            if ((t < 2500.0) && ((best == 0.0) || (t < best)))
            {
                best = t;
            }
        }
    }

    return best;
}

static double rect_dist(const rect_t *r, double x, double y)
{
    const double dx = (x < r->x0) ? (r->x0 - x) : ((x > r->x1) ? (x - r->x1) : 0.0);
    const double dy = (y < r->y0) ? (r->y0 - y) : ((y > r->y1) ? (y - r->y1) : 0.0);

    return sqrt(dx * dx + dy * dy);
}

static int    g_collided;
static double g_min_clear;
static char   g_collide_at[96];

static void check_footprint(void)
{
    double f, l;
    int i;

    for (f = CAR_REAR; f <= CAR_FRONT + 0.1; f += 10.0)
    {
        for (l = -CAR_HALF_W; l <= CAR_HALF_W + 0.1; l += 10.0)
        {
            double wx, wy;

            /* outline only */
            if ((f > CAR_REAR + 0.1) && (f < CAR_FRONT - 0.1) &&
                (l > -CAR_HALF_W + 0.1) && (l < CAR_HALF_W - 0.1))
            {
                continue;
            }
            body_to_world(f, l, &wx, &wy);
            for (i = 0; i < g_nrect; i++)
            {
                const double d = rect_dist(&g_rect[i], wx, wy);

                if ((d <= 0.0) && !g_collided)
                {
                    double cx, cy, cpsi;

                    mm_get_truth(&cx, &cy, &cpsi);
                    (void)snprintf(g_collide_at, sizeof(g_collide_at),
                                   "COLLISION at %.2f s, axle x %.0f y %.0f mm,"
                                   " heading %+.1f deg",
                                   tkshim_now_ms() / 1000.0, cx, cy, cpsi);
                    g_collided = 1;
                }
                if (d < g_min_clear) { g_min_clear = d; }
            }
        }
    }
}

static double g_gap_y0, g_gap_y1;

static int ir_black(double lateral)
{
    double wx, wy;

    body_to_world((g_s->ir_ahead_mm > 0.0) ? g_s->ir_ahead_mm : IR_AHEAD,
                  lateral, &wx, &wy);
    if ((g_gap_y1 > g_gap_y0) && (wy >= g_gap_y0) && (wy <= g_gap_y1))
    {
        return 0;   /* v3.2: line missing here */
    }
    if ((g_s->lane_mm > 0.0) && (fabs(fabs(wx) - g_s->lane_mm) < LINE_HALF))
    {
        return 1;   /* nyx, 2026-11-06: another lane's line */
    }
    return fabs(wx) < LINE_HALF;
}

static bool car_odometry(int32_t *p_speed, int32_t *p_dist)
{
    motion_status_t m;

    motion_get_status(&m);
    /* as vc_task publishes it: the faster wheel, and the mean distance */
    *p_speed = (abs(m.speed_l_mmps) > abs(m.speed_r_mmps)) ? m.speed_l_mmps
                                                           : m.speed_r_mmps;
    *p_dist = m.distance_total_mm;
    return true;
}

/*=====================================================================*/
/* VehicleController stand-in                                          */
/*=====================================================================*/

typedef enum { H_FOLLOW, H_OBSTACLE, H_FAULT } h_state_t;

static h_state_t g_h;
static int       g_done_count;
static uint32_t  g_done_ms;
static uint32_t  g_fault_ms;
static double    g_rejoin_x, g_rejoin_psi, g_rejoin_y;
static int16_t   g_sent_l, g_sent_r;
static int       g_since_send;
static int       g_last_err;
static int       g_reverses;
static int       g_hard_stops;
static int       g_steps_seen[VC_OBS_STEP_COUNT];
/* nyx, 2026-11-06: Buddy 3's line logic after the handover */
static int        g_b3;           /* 1 once VC handed back */
static b3_line_t  g_b3_line;
static double     g_after_y0;     /* truth y at the handover */
static double     g_after_d0;     /* truth path length at the handover */
static double     g_path;         /* truth path length driven */
static double     g_after_max_x;  /* worst |axle x| while following */
static int        g_after_done;
static double     g_after_x, g_after_psi, g_after_irmid;

static void h_drive(int16_t l, int16_t r)
{
    if ((l != g_sent_l) || (r != g_sent_r) || (g_since_send >= 100))
    {
        if (motion_set_wheel_speeds(l, r) == MOTION_OK)
        {
            g_sent_l = l;
            g_sent_r = r;
            g_since_send = 0;
        }
    }
}

/* mirrors vehicle_controller.c read_obstacle() */
static void h_read_obstacle(const motion_status_t *m)
{
    UINT ptn = 0U;

    if (tk_wai_flg(flg_obstacle, S5_EVT_ALL, TWF_ORW | TWF_BITCLR, &ptn,
                   TMO_POL) < E_OK)
    {
        return;
    }

    if ((ptn & S5_EVT_HARD_STOP) && (g_h != H_FAULT))
    {
        if (!vc_obs_cut_active())
        {
            vc_obs_hard_stop(m);
            if (vc_obs_cut_active()) { g_hard_stops++; }
        }
    }

    if ((ptn & (S5_EVT_DETECTED | S5_EVT_HARD_STOP)) && (g_h == H_FOLLOW))
    {
        (void)motion_stop();
        g_sent_l = 0;
        g_sent_r = 0;
        g_h = H_OBSTACLE;
        vc_obs_begin(m);
    }

    if (g_h == H_OBSTACLE)
    {
        vc_obs_events(ptn);
    }
    else if (ptn & S5_EVT_PROFILED)
    {
        obstacle_resume();      /* parked (E-stop): nothing to bypass */
    }

    if (ptn & S5_EVT_FAULT)
    {
        obstacle_health_t hl;

        obstacle_get_health(&hl);
        if ((hl.fault != OBSTACLE_FAULT_NONE) && (hl.fault != OBSTACLE_FAULT_SENSOR))
        {
            if (g_h == H_OBSTACLE) { vc_obs_leave(); }
            (void)motion_stop();
            g_h = H_FAULT;
            g_fault_ms = tkshim_now_ms();
        }
    }
}

static void h_step(void)
{
    motion_status_t m;
    const int irl = ir_black(IR_SPREAD);
    const int irr = ir_black(-IR_SPREAD);

    /* nyx, 2026-11-06: Buddy 3's line logic runs on every pass, as on
     * the car, so its state (a latched sharp turn) carries over into the
     * handover; its wheel speeds are used only after it. */
    const b3_line_result_t lr =
        b3_line_step(&g_b3_line, irl != 0, irr != 0, tkshim_now_ms() * 1000U,
                     (int16_t)B3_BASE_SPEED_MM_S);

    motion_get_status(&m);
    vc_obs_service_cut(&m, 10);
    h_read_obstacle(&m);
    g_since_send += 10;

    switch (g_h)
    {
    case H_FOLLOW:
        if (g_b3 && !vc_obs_cut_active() && (m.state != MOTION_STATE_ESTOP))
        {
            h_drive(lr.left_mm_s, lr.right_mm_s);
        }
        else if (!vc_obs_cut_active() && (m.state != MOTION_STATE_ESTOP))
        {
            /* Buddy 3 stand-in: IR straddle, gentle steering. */
            if (irl && !irr)      { g_last_err = -1; }
            else if (irr && !irl) { g_last_err = 1; }
            else if (!irl && !irr){ g_last_err = 0; }
            h_drive((int16_t)(CRUISE + 40 * g_last_err),
                    (int16_t)(CRUISE - 40 * g_last_err));
        }
        break;
    case H_OBSTACLE:
    {
        const vc_obs_step_t before = vc_obs_get_step();
        const vc_obs_result_t r = vc_obs_step(&m, irl, irr, 10);
        const vc_obs_step_t after = vc_obs_get_step();

        if ((after != before) && (after < VC_OBS_STEP_COUNT))
        {
            g_steps_seen[after]++;
            if (after == VC_OBS_REVERSE) { g_reverses++; }
        }
        if (r == VC_OBS_DONE)
        {
            mm_get_truth(&g_rejoin_x, &g_rejoin_y, &g_rejoin_psi);
            g_done_count++;
            g_b3 = 1;
            g_after_y0 = g_rejoin_y;
            g_after_d0 = g_path;
            g_after_max_x = fabs(g_rejoin_x);
            g_after_done = 0;
            g_done_ms = tkshim_now_ms();
            g_h = H_FOLLOW;
            g_sent_l = 0;
            g_sent_r = 0;
            g_since_send = 1000;
        }
        else if (r == VC_OBS_FAULT)
        {
            vc_obs_leave();
            (void)motion_stop();
            g_h = H_FAULT;
            g_fault_ms = tkshim_now_ms();
        }
        else
        {
            /* running */
        }
        break;
    }
    case H_FAULT:
    default:
        break;
    }
}

/*=====================================================================*/
/* Run one scenario                                                    */
/*=====================================================================*/

static uint32_t g_ms;
static char     g_log[16384];
static size_t   g_log_len;

static void capture(const char *text)
{
    if ((strncmp(text, "[VC-OBS]", 8) == 0) || (strncmp(text, "S5:", 3) == 0))
    {
        int n = snprintf(g_log + g_log_len, sizeof(g_log) - g_log_len,
                         "    %5.2f s  %s", tkshim_now_ms() / 1000.0, text);
        if ((n > 0) && ((size_t)n < sizeof(g_log) - g_log_len))
        {
            g_log_len += (size_t)n;
        }
    }
}

static void world_tick(void)
{
    double ox, oy, opsi;

    g_ms++;
    mm_get_truth(&ox, &oy, &opsi);
    mm_step_1ms();
    {
        double nx, ny, npsi;

        mm_get_truth(&nx, &ny, &npsi);
        g_path += sqrt(((nx - ox) * (nx - ox)) + ((ny - oy) * (ny - oy)));
        if (g_b3 && (g_h == H_FOLLOW) && (fabs(nx) > g_after_max_x))
        {
            g_after_max_x = fabs(nx);
        }
    }

    if ((g_s->drop_at_s > 0.0) && !g_dropped &&
        (g_ms >= (uint32_t)(g_s->drop_at_s * 1000.0)))
    {
        /* Drop r[0] right in front of the sensor (a person's hand). */
        double px, py;
        const double w = g_s->r[0].x1 - g_s->r[0].x0;
        const double d = g_s->r[0].y1 - g_s->r[0].y0;

        body_to_world(PIVOT_AHEAD + g_s->drop_gap_mm, 0.0, &px, &py);
        g_rect[0].x0 = px - w / 2.0;
        g_rect[0].x1 = px + w / 2.0;
        g_rect[0].y0 = py;
        g_rect[0].y1 = py + d;
        g_nrect = 1;
        g_dropped = 1;
    }

    check_footprint();

    if ((g_ms % 10U) == 0U)
    {
        h_step();
    }

    /* nyx, 2026-11-06: follow with Buddy 3 for AFTER_MM, then judge. */
    if ((g_done_count > 0) && (g_h == H_FOLLOW) &&
        ((g_path - g_after_d0) >= AFTER_MM))
    {
        double wx, wy;

        mm_get_truth(&g_after_x, &wy, &g_after_psi);
        body_to_world((g_s->ir_ahead_mm > 0.0) ? g_s->ir_ahead_mm : IR_AHEAD,
                      0.0, &wx, &wy);
        g_after_irmid = wx;
        g_after_done = 1;
        tkshim_stop();
    }
    if ((g_h == H_FAULT) && ((tkshim_now_ms() - g_fault_ms) > 1500U))
    {
        tkshim_stop();
    }
}

static int run(int idx, int verbose)
{
    const scen_t *s = &g_scen[idx];
    int ok;
    const char *why = "";

    g_s = s;
    tkshim_reset();
    sim_counters_reset();
    sim_world_clear();
    sim_set_ray_fn(car_ray);
    if (s->servo_err)
    {
        sim_set_servo_error(-35, 15);
        sim_set_beam_half_deg(12.0);
    }
    if (s->beam_deg > 0.0)
    {
        sim_set_beam_half_deg(s->beam_deg); /* nyx, 2026-11-06 */
    }
    sim_set_noise_mm(3);
    mm_reset(0.0, 0.0, 0.0);
    mm_set_errors(s->enc_scale, s->track_err);
    mm_set_dead_right(s->dead_enc);
    g_gap_y0 = s->gap_y0;
    g_gap_y1 = s->gap_y1;

    memcpy(g_rect, s->r, sizeof(g_rect));
    g_nrect = (s->drop_at_s > 0.0) ? 0 : s->n;
    g_dropped = 0;
    g_collided = 0;
    g_collide_at[0] = '\0';
    g_min_clear = 1e9;
    g_h = H_FOLLOW;
    g_done_count = 0;
    g_sent_l = g_sent_r = 0;
    g_since_send = 1000;
    g_last_err = 0;
    g_reverses = 0;
    g_hard_stops = 0;
    memset(g_steps_seen, 0, sizeof(g_steps_seen));
    g_b3 = 0;
    b3_line_init(&g_b3_line, 0U);
    g_path = 0.0;
    g_after_done = 0;
    g_ms = 0;
    g_log_len = 0;
    g_log[0] = '\0';

    (void)obstacle_create_objects();
    obstacle_register_odometry(car_odometry);

    tkshim_set_quiet(1);
    tkshim_set_print_hook(capture);
    tkshim_set_idle_hook(world_tick);
    /* v3.2: slower scans; nyx, 2026-11-06: + Buddy 3 following AFTER_MM
     * at its base speed after the handover */
    tkshim_set_time_budget_ms(150000U);
    tkshim_run(obstacle_task);

    if (g_collided)
    {
        ok = 0; why = g_collide_at;
    }
    else if ((s->expect_fault == 1) ||
             ((s->expect_fault == 2) && (g_done_count == 0)))
    {
        ok = (g_h == H_FAULT) && (fabs(mm_speed_true()) < 1.0);
        why = ok ? "" : "expected a safe stop in FAULT";
    }
    else if (g_done_count == 0)
    {
        ok = 0; why = "never got back to line following";
    }
    else
    {
        /* The rear of the car must be past every obstacle that sat on
         * the line, or "rejoined" just means "gave up and drove on". */
        double far_end = 0.0;
        int k;

        for (k = 0; k < g_nrect; k++)
        {
            if ((g_rect[k].x0 < CAR_HALF_W) && (g_rect[k].x1 > -CAR_HALF_W) &&
                (g_rect[k].y1 > far_end))
            {
                far_end = g_rect[k].y1;
            }
        }
        ok = (fabs(g_rejoin_x) <= 50.0) && ((g_rejoin_y + CAR_REAR) > far_end);
        why = (fabs(g_rejoin_x) > 50.0) ? "rejoin outside +/-50 mm"
                                         : "rejoined before passing the obstacle";
        /* nyx, 2026-11-06: and Buddy 3 must keep it on the line. */
        if (ok)
        {
            const int on_line =
                g_after_done && (fabs(g_after_x) <= AFTER_X_MM) &&
                (fabs(g_after_psi) <= AFTER_DEG) &&
                (fabs(g_after_irmid) <= (IR_SPREAD + LINE_HALF));

            if (!on_line)
            {
                ok = 0;
                why = g_after_done ? "lost the line after the handover"
                                   : "Buddy 3 never drove the follow distance";
            }
        }
        if ((g_rejoin_y + CAR_REAR) > far_end)
        {
            printf("    rear %.0f mm past the obstacle's far end\n",
                   g_rejoin_y + CAR_REAR - far_end);
        }
    }

    printf("%-58s %s\n", s->name, ok ? "PASS" : "FAIL");
    if (g_h == H_FAULT)
    {
        printf("    safe stop in FAULT after %.1f s, closest approach %.0f mm, reverses %d\n",
               g_fault_ms / 1000.0, g_min_clear, g_reverses);
    }
    else if (g_done_count > 0)
    {
        printf("    rejoined at y %.0f mm: axle %+.0f mm from the line, heading %+.1f deg;"
               " closest approach %.0f mm; reverses %d, hard stops %d, side steps %d;"
               " episode %.1f s\n",
               g_rejoin_y, g_rejoin_x, g_rejoin_psi, g_min_clear, g_reverses,
               g_hard_stops, g_steps_seen[VC_OBS_SIDESTEP], g_done_ms / 1000.0);
        printf("    Buddy 3 %.0f mm later: axle %+.0f mm, heading %+.1f deg,"
               " IR middle %+.0f mm from the line (worst axle %.0f mm)\n",
               AFTER_MM, g_after_x, g_after_psi, g_after_irmid, g_after_max_x);
    }
    if (!ok)
    {
        printf("    %s\n", why);
    }
    if (verbose || !ok)
    {
        fputs(g_log, stdout);
    }

    return ok;
}

int main(int argc, char **argv)
{
    int failures = 0;
    int i;

    printf("S5-H07 2-D bypass simulation (%s, beam half-angle %d)\n",
           (S5_CAL_DONE != 0) ? "calibration compiled in" : "no calibration",
           (int)S5_CAL_BEAM_HALF_DDEG);

    if (argc > 1)
    {
        i = atoi(argv[1]) - 1;
        if ((i < 0) || (i >= N_SCEN))
        {
            return EXIT_FAILURE;
        }
        return run(i, (argc > 2)) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    {
        int n = 0;

        for (i = 0; i < N_SCEN; i++)
        {
            if ((g_scen[i].cal_beam != 0) != (CAL_BEAM_BUILD != 0))
            {
                continue; /* nyx, 2026-11-06: the other build runs it */
            }
            n++;
            if (!run(i, 0))
            {
                failures++;
            }
        }

        printf("\n%d scenarios, %d failures\n", n, failures);
    }

    return (failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
