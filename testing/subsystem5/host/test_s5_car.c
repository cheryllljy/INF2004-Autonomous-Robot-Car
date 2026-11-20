/**
 * @file    test_s5_car.c
 * @brief   HOST TEST (S5-H21). 2-D simulation of the
 *          whole obstacle episode on a simulated car.
 *
 *   real       s5_sensor.c (the S5 task), s5_avoid.c, s5_scan.c,
 *              s5_path.c, s5_odom.c, s5_math.c; Buddy 3's b3_line.c
 *   simulated  the world, the HC-SR04P on its servo and the IR pair
 *              (s5_sim.c), Buddy 2 with its 20 ms goal, ramp and wheel lag
 *              (motion_mock.c), the VehicleController's obstacle part
 *              (read_obstacle(), act(), drive(), as in
 *              vehicle_controller.c), the S5 task every 10 ms (tick),
 *              Buddy 3 every 10 ms, the VehicleController every 20 ms
 *
 * Each scenario says what must happen: round (on the side it names, or
 * either), blocked (wait with the fault bit, no E-stop, go on once the way
 * opens) or a fault. Going round passes only if the car never touches a
 * box, stays at least 10 mm from it, hands back to Buddy 3 with the line
 * between the IRs, and then follows the SAME line (not another lane) for
 * the rest of the track.
 *
 *   ./test_s5_car            all scenarios
 *   ./test_s5_car ID [v]     one scenario (e.g. C1); v = log and trace
 *   ./test_s5_car -s N       N random seeds per scenario (default 3)
 *   ./test_s5_car -s N -k K ID   only seed K of N (to look at one run)
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "b3_line.h"
#include "motion_control.h"
#include "motion_mock.h"
#include "s5_avoid.h"
#include "s5_config.h"
#include "s5_hw.h"
#include "s5_sensor.h"
#include "s5_sim.h"

#define PI        (3.14159265358979323846)
#define VC_MS     (20U)
#define REFRESH   (100U) /* VC_SPEED_REFRESH_MS */
#define AFTER_MM  (1000.0)
#define LIMIT_MS  (120000U)

typedef enum
{
    X_ROUND = 0, /* goes round and back onto the line */
    X_BLOCKED,   /* waits with the fault bit; goes on if the way opens */
    X_ECHO,      /* FR25 sensor fault (E-stop) */
    X_OBST,      /* the bypass gives up (VC_FAULT_OBSTACLE) */
    X_CLEAR      /* stops, finds nothing, follows the line again */
} expect_t;

typedef struct
{
    double face; /* near face, mm along the line from the start */
    double y;    /* centre, mm left of the line */
    double w;    /* across */
    double d;    /* along */
    double rot;  /* deg */
    double appear_x; /* not there until the car's axle reaches this */
    double gone_ms;  /* taken away this long after the car first stops */
} box_t;

typedef struct
{
    const char *id;
    const char *name;
    expect_t expect;
    int side; /* +1 left, -1 right, 0 either */
    box_t box[4];
    int n_box;
    double lanes;      /* other lanes' lines this far either side */
    double enc_err;    /* encoders read long by this fraction */
    double track_err;  /* the real track is wider by this fraction */
    double beam;       /* real beam half-angle, deg (0: 22) */
    double servo_trim; /* real servo error, deg */
    double servo_dps;  /* 0: 450 */
    double no_echo_ms; /* 0: 38 */
    double track_deg;  /* the track turned in the odometry frame */
    double bend_r;     /* our line bends (R, deg) just before the box */
    double bend_deg;
    double start_y;    /* the car starts this far left of the line */
    double start_th;   /* ... turned this much */
    double sonar_left; /* real sensor this far left of the centre */
    int dead_sonar;
    int dead_right;    /* right encoder dead from the start of the bypass */
    double drop;       /* -1: no dropouts */
    double noise;      /* 0: 2 mm */
    int n_obst;        /* obstacles on the line, one after the other (0: 1) */
    double rev_max;    /* > 0: back up no more than this */
    double lr_err;     /* left encoder long by this / 2,
                        * right short by this / 2 */
    double junc_x;     /* > 0: a junction's branch leaves
                        * our line here (track frame) ... */
    double junc_deg;   /* ... at this angle to it (+ = to the left) ... */
    double junc_len;   /* ... this long (0: 400) */
    int no_mag;        /* no compass (IMU unplugged) */
    double mag_err;    /* compass error after calibration, deg (0: 2) */
    double mag_iron;   /* iron near the box pulls the field, counts */
    double shift;      /* our line moves this far left
                        * (S-bend from the box's face on), so dead
                        * reckoning has it that far off on the way back */
    double turn_x;     /* > 0: our line bends from here
                        * (track frame) ... */
    double turn_r;     /* ... with this radius ... */
    double turn_deg;   /* ... this far (+ = to the left), then straight */
} scen_t;

#define BOX(f, y, w, d) { (f), (y), (w), (d), 0.0, 0.0, 0.0 }

static const scen_t g_scen[] =
{
    { "C1", "21x16 cm box on the line (track), 2 m ahead",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1 },
    { "C2", "box 60 mm right of the line: left is shorter",
      X_ROUND, 1, { BOX(900, -60, 210, 160) }, 1 },
    { "C3", "box 60 mm left of the line: right is shorter",
      X_ROUND, -1, { BOX(900, 60, 210, 160) }, 1 },
    { "C4", "box 25 mm right: left is shorter by 50 mm",
      X_ROUND, 1, { BOX(900, -25, 210, 160) }, 1 },
    { "C5", "narrow 10x10 cm box",
      X_ROUND, 0, { BOX(900, 0, 100, 100) }, 1 },
    { "C6", "wide 30 cm box",
      X_ROUND, 0, { BOX(900, 0, 300, 160) }, 1 },
    { "C7", "deep box, 21 x 40 cm",
      X_ROUND, 0, { BOX(900, 0, 210, 400) }, 1 },
    { "C8", "box put down 100 mm in front of the car",
      X_ROUND, 0, { { 900, 0, 210, 160, 0, 900 - 135 - 100 - 1, 0 } }, 1 },
    { "C9", "box put down 50 mm in front of the car",
      X_ROUND, 0, { { 900, 0, 210, 160, 0, 900 - 135 - 50 - 1, 0 } }, 1 },
    { "C10", "walls both sides: no way round, wait, left wall taken away",
      X_BLOCKED, 1,
      { BOX(900, 0, 210, 160), { 850, 330, 200, 400, 0, 0, 15000 },
        BOX(850, -330, 200, 400) }, 3 },
    { "C11", "walls both sides: wait, then the box is taken away",
      X_BLOCKED, 0,
      { { 900, 0, 210, 160, 0, 0, 4000 }, BOX(850, 330, 200, 400),
        BOX(850, -330, 200, 400) }, 3 },
    { "C12", "second box beside it on the left: goes right",
      X_ROUND, -1, { BOX(900, -15, 210, 160), BOX(1000, 290, 160, 160) },
      2 },
    { "C13", "lanes 250 mm either side: back onto OUR line",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 250.0 },
    { "C14", "encoders 3 % long, real track 5 % wider",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.03, 0.05 },
    { "C15", "encoders 3 % short, real track 5 % narrower, lanes 250",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 250.0, -0.03, -0.05 },
    { "C16", "real beam 19 deg (calibration says 22)",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 19.0 },
    { "C17", "real beam 25 deg (calibration says 22)",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 25.0 },
    { "C18", "servo 4 deg off to the left, slow servo (250 deg/s)",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 4.0,
      250.0 },
    { "C19", "no-echo pulse 150 ms (slow HC-SR04P variant)",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 150.0 },
    { "C20", "track at 37 deg to the odometry frame",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 37.0 },
    { "C21", "box right after a R500 left bend",
      X_ROUND, 0, { BOX(380, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 500.0, 40.0 },
    { "C22", "box turned 15 deg",
      X_ROUND, 0, { { 900, 0, 210, 160, 15.0, 0, 0 } }, 1 },
    { "C23", "car starts 10 mm off the line, 8 deg to it",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 10.0, -8.0 },
    { "C24", "sensor mounted 10 mm left of the centre",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0 },
    { "C25", "two boxes 1.4 m apart",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(2300, 20, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C26", "no sensor (ECHO never rises): FR25 fault",
      X_ECHO, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1 },
    { "C27", "right encoder dies at the obstacle: bypass gives up",
      X_OBST, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1 },
    { "C28", "box taken away while the car stands: line following again",
      X_CLEAR, 0, { { 900, 0, 210, 160, 0, 0, 300 } }, 1 },
    { "C29", "noisy sensor: 5 mm noise, 10 % dropouts",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.10, 5.0 },
    /* Buddy 5, 2026-11-19: the 9 cm object. side = the shorter way */
    { .id = "C80", .name = "9 cm box on the line",
      .expect = X_ROUND, .side = 0, .box = { BOX(900, 0, 90, 90) }, .n_box = 1 },
    { .id = "C81", .name = "9 cm box 20 mm right, beam 15",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -20, 90, 90) },
      .n_box = 1, .beam = 15.0 },
    { .id = "C82", .name = "9 cm box 20 mm left, beam 15",
      .expect = X_ROUND, .side = -1, .box = { BOX(900, 20, 90, 90) },
      .n_box = 1, .beam = 15.0 },
    { .id = "C83", .name = "9 cm box 30 mm right, beam 30",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -30, 90, 90) },
      .n_box = 1, .beam = 30.0 },
    { .id = "C84", .name = "9 cm box 30 mm left, beam 30",
      .expect = X_ROUND, .side = -1, .box = { BOX(900, 30, 90, 90) },
      .n_box = 1, .beam = 30.0 },
    { .id = "C85", .name = "9 cm box 40 mm right, beam 15, servo 4 deg off",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -40, 90, 90) },
      .n_box = 1, .beam = 15.0, .servo_trim = -4.0 },
    { .id = "C86", .name = "9 cm box 40 mm left, beam 15, servo 4 deg off",
      .expect = X_ROUND, .side = -1, .box = { BOX(900, 40, 90, 90) },
      .n_box = 1, .beam = 15.0, .servo_trim = -4.0 },
    { .id = "C87", .name = "9 cm box 30 mm right, beam 15, servo 10 deg off",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -30, 90, 90) },
      .n_box = 1, .beam = 15.0, .servo_trim = -10.0 },
    { .id = "C88", .name = "9 cm box 30 mm left, beam 15, servo 10 deg off",
      .expect = X_ROUND, .side = -1, .box = { BOX(900, 30, 90, 90) },
      .n_box = 1, .beam = 15.0, .servo_trim = -10.0 },
    { "C30", "box 120 mm right: line clear of it on the left",
      X_ROUND, 1, { BOX(900, -120, 210, 160) }, 1 },
    /* the next box close behind the first,
     * met on the way back to the line */
    { "C31", "second box 56 cm behind the first",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1620, 0, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C32", "second box 75 cm behind the first, 30 mm left",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1810, 30, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C33", "second box 40 cm behind the first",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1460, 0, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C34", "second box 30 cm behind the first",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1360, 0, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C35", "second box 30 cm behind, 60 mm left: in the left pass lane",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1360, 60, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C36", "second box 40 cm behind, 100 mm right",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1460, -100, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C37", "second box 40 cm behind, 30 cm wide",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1460, 0, 300, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C38", "second box 40 cm behind, 40 cm deep",
      X_ROUND, 0, { BOX(900, 0, 210, 160), BOX(1460, 0, 210, 400) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C39", "three boxes, gaps 50 and 30 cm, 30 mm left / right",
      X_ROUND, 0,
      { BOX(900, 0, 210, 160), BOX(1560, 30, 210, 160),
        BOX(2020, -30, 210, 160) },
      3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 3 },
    /* layouts from the independent
     * review that hit a box before the fixes */
    { "C40", "second box 15 cm behind, 50 mm into the left pass lane",
      X_ROUND, 0, { BOX(900, -20, 210, 160), BOX(1210, 50, 210, 160) }, 2,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    { "C41", "3 cm post put down in the left pass lane as the car turns out",
      X_ROUND, 0,
      { BOX(900, -20, 210, 160), { 1160, 200, 30, 30, 0.0, 800.0, 0.0 } },
      2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
      0, 0.0, 0.0, 2 },
    /* the car stops 30 cm from the box and goes round
     * without backing up (a steeper ramp out where the gentle one has no
     * room); the car turns 10 % more than the encoders say, so dead
     * reckoning puts the line some 50-70 mm off on the way back and the
     * IRs pick it up */
    { .id = "C42", .name = "21x16 cm box, car stops 30 cm short: no backing up",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .rev_max = 1.0 },
    { .id = "C43", .name = "16x21 cm box (16 cm face): no backing up",
      .expect = X_ROUND, .box = { BOX(900, 0, 160, 210) }, .n_box = 1,
      .rev_max = 1.0 },
    { .id = "C44", .name = "real track 10 % narrower (turns 10 % more): IRs pick "
                           "the line up",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .track_err = -0.10 },
    { .id = "C45", .name = "real track 10 % wider (turns 10 % less): IRs pick "
                           "the line up",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .track_err = 0.10 },
    { .id = "C46", .name = "line 70 mm further left behind the box: IRs pick "
                           "it up",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .shift = 70.0 },
    { .id = "C47", .name = "line 70 mm further right behind the box: IRs pick "
                           "it up",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .shift = -70.0 },
    /* a track run: the car ended ~5 cm beside the line, no IR saw
     * black, and the search at 12 deg did not reach it */
    { .id = "C48", .name = "left encoder 0.5 % long, right 0.5 % short: ends "
                           "~6 cm left of the line",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = 0.01 },
    { .id = "C49", .name = "left encoder 0.5 % short, right 0.5 % long: ends "
                           "~6 cm right of the line",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = -0.01 },
    { .id = "C50", .name = "left encoder 0.75 % long, right 0.75 % short: "
                           "ends ~10 cm left",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = 0.015 },
    { .id = "C51", .name = "left encoder 0.75 % short, right 0.75 % long: "
                           "ends ~10 cm right",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = -0.015 },
    /* track runs 2026-11-08: a junction just behind the box. The
     * encoders put our line 7-12 cm off; the IRs took a junction's branch
     * 5 cm beside our line for it, or the branch stopped the turn onto
     * the line early. The branch: tape across our line, from it */
    { .id = "C52", .name = "junction's branch across the way back (going "
                           "left): the IRs leave it alone",
      .expect = X_ROUND, .box = { BOX(900, -60, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .junc_x = 1200.0, .junc_deg = 90.0 },
    { .id = "C53", .name = "junction where the car turns onto the line "
                           "(going left): the turn is not cut short",
      .expect = X_ROUND, .box = { BOX(900, -60, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .junc_x = 1450.0, .junc_deg = 90.0 },
    { .id = "C54", .name = "junction where the car turns onto the line "
                           "(going right): the turn is not cut short",
      .expect = X_ROUND, .box = { BOX(900, 60, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .junc_x = 1400.0, .junc_deg = 90.0 },
    { .id = "C55", .name = "junction's branch on the side the car comes "
                           "back from (going right): left alone",
      .expect = X_ROUND, .box = { BOX(900, 60, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .junc_x = 1200.0, .junc_deg = -90.0 },
    { .id = "C56", .name = "junction's branch at 45 deg across the way back "
                           "(going left): left alone",
      .expect = X_ROUND, .box = { BOX(900, -60, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .junc_x = 1400.0, .junc_deg = 45.0 },
    /* the compass (Buddy 4's magnetometer). Every
     * scenario has it unless it says otherwise */
    { .id = "C57", .name = "encoders 2 % apart (left long): the compass "
                           "keeps the heading",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = 0.02 },
    { .id = "C58", .name = "no compass (IMU unplugged), encoders 1 % apart: "
                           "encoders and IRs only",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .no_mag = 1 },
    { .id = "C59", .name = "compass up to 6 deg off with heading (poor "
                           "calibration), encoders 1 % apart",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .mag_err = 6.0 },
    { .id = "C60", .name = "iron in the box pulls the field 30 % near it: "
                           "the compass is not used there",
      .expect = X_ROUND, .box = { BOX(900, 0, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .mag_iron = 750.0 },
    /* our line bends where the car comes back to it
     * (the car comes back about 45 deg to it, the IRs crossing it near
     * x 1360): the turn onto the line stops on the IRs (as
     * the line search), not at the direction the crossing gave */
    { .id = "C61", .name = "line bends left R300 behind the box, car comes "
                           "back from the left",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -60, 210, 160) },
      .n_box = 1, .turn_x = 1300.0, .turn_r = 300.0, .turn_deg = 90.0 },
    { .id = "C62", .name = "line bends right R300 behind the box, car "
                           "comes back from the left",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -60, 210, 160) },
      .n_box = 1, .turn_x = 1300.0, .turn_r = 300.0, .turn_deg = -90.0 },
    { .id = "C63", .name = "line bends left R300 behind the box, car comes "
                           "back from the right",
      .expect = X_ROUND, .side = -1, .box = { BOX(900, 60, 210, 160) },
      .n_box = 1, .turn_x = 1300.0, .turn_r = 300.0, .turn_deg = 90.0 },
    { .id = "C64", .name = "line bends right R300 behind the box, car "
                           "comes back from the right",
      .expect = X_ROUND, .side = -1, .box = { BOX(900, 60, 210, 160) },
      .n_box = 1, .turn_x = 1300.0, .turn_r = 300.0, .turn_deg = -90.0 },
    { .id = "C65", .name = "line bends left R200 behind the box, car comes "
                           "back from the left",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -60, 210, 160) },
      .n_box = 1, .turn_x = 1250.0, .turn_r = 200.0, .turn_deg = 90.0 },
    { .id = "C66", .name = "line bends left R500 from the box's back, car "
                           "comes back from the left, encoders 1 % apart",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -60, 210, 160) },
      .n_box = 1, .lr_err = 0.01, .turn_x = 1100.0, .turn_r = 500.0,
      .turn_deg = 60.0 },
    { .id = "C67", .name = "line bends right R500 from the box's back, car "
                           "comes back from the left, encoders 1 % apart",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -60, 210, 160) },
      .n_box = 1, .lr_err = -0.01, .turn_x = 1100.0, .turn_r = 500.0,
      .turn_deg = -60.0 },
    /* too close for a ramp out, the car turns out on
     * the spot instead of backing up; it backs up only when no way round
     * is open (edges it cannot see, both sides blocked) */
    { .id = "C68", .name = "box put down 150 mm in front: turns out on the "
                           "spot, no backing up",
      .expect = X_ROUND,
      .box = { { 900, 0, 210, 160, 0, 900 - 135 - 150 - 1, 0 } },
      .n_box = 1, .rev_max = 1.0 },
    { .id = "C69", .name = "box put down 150 mm in front, 40 mm left: no "
                           "backing up",
      .expect = X_ROUND, .side = -1,
      .box = { { 900, 40, 210, 160, 0, 900 - 135 - 150 - 1, 0 } },
      .n_box = 1, .rev_max = 1.0 },
    { .id = "C70", .name = "25 cm wide box put down 200 mm in front, "
                           "encoders 1 % apart: no backing up",
      .expect = X_ROUND,
      .box = { { 900, 0, 250, 160, 0, 900 - 135 - 200 - 1, 0 } },
      .n_box = 1, .lr_err = 0.01, .rev_max = 1.0 },
    /* shown only (the calibration build measures the beam): 6 deg off */
    { "L1", "LIMIT: real beam 16 deg (calibration says 22)",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 16.0 },
    { "L2", "LIMIT: real beam 28 deg (calibration says 22)",
      X_ROUND, 0, { BOX(900, 0, 210, 160) }, 1, 0.0, 0.0, 0.0, 28.0 },
    /* shown only: the IRs cross our line right where a branch meets it,
     * one IR on each, and the pair looks like neither */
    { .id = "L3", .name = "LIMIT: crossing right where a junction's branch "
                          "meets our line",
      .expect = X_ROUND, .box = { BOX(900, 60, 210, 160) }, .n_box = 1,
      .lr_err = 0.01, .junc_x = 1300.0, .junc_deg = -90.0 },
    /* shown only: a R200 bend away from the car right
     * where it comes back; one IR meets the line's edge, the bend takes
     * it away before the other does, and the car can be handed over by
     * the encoders off the line (2 of 10) */
    { .id = "L4", .name = "LIMIT: line bends right R200 behind the box, "
                          "car comes back from the left",
      .expect = X_ROUND, .side = 1, .box = { BOX(900, -60, 210, 160) },
      .n_box = 1, .turn_x = 1250.0, .turn_r = 200.0, .turn_deg = -90.0 },
};

#define N_SCEN ((int)(sizeof(g_scen) / sizeof(g_scen[0])))

/*=====================================================================*/
/* The VehicleController's obstacle part                                 */
/*=====================================================================*/

typedef enum
{
    V_LINE = 0,
    V_OBST,
    V_FAULT
} vstate_t;

typedef struct
{
    vstate_t state;
    int16_t sent_l;
    int16_t sent_r;
    uint32_t since_send_ms;
    int fault; /* 1 FR25 sensor, 2 obstacle */
    b3_line_t line;
    b3_line_result_t snap;
} vc_t;

static vc_t g_vc;

static void drive(int16_t l, int16_t r)
{
    if ((l != g_vc.sent_l) || (r != g_vc.sent_r) ||
        (g_vc.since_send_ms >= REFRESH))
    {
        if (MOTION_OK == motion_set_wheel_speeds(l, r))
        {
            g_vc.sent_l = l;
            g_vc.sent_r = r;
            g_vc.since_send_ms = 0U;
        }
    }
}

static void stop_motors(void)
{
    (void)motion_stop();
    g_vc.sent_l = 0;
    g_vc.sent_r = 0;
}

/* one vc_task pass: read_motion, read_obstacle, exchange_line, act */
static int vc_pass(void)
{
    motion_status_t m;
    int done = 0;

    motion_get_status(&m);
    g_vc.since_send_ms += VC_MS;
    if (V_FAULT == g_vc.state)
    {
        return 0;
    }
    /* read_obstacle() */
    if (((V_LINE == g_vc.state) || (V_OBST == g_vc.state)) &&
        s5_avoid_sensor_fault())
    {
        stop_motors();
        g_vc.state = V_FAULT;
        g_vc.fault = 1;
        return 0;
    }
    if ((V_LINE == g_vc.state) && s5_avoid_check(true))
    {
        stop_motors();
        g_vc.state = V_OBST;
        s5_avoid_begin(&m);
    }
    /* act() */
    if (V_OBST != g_vc.state)
    {
        s5_avoid_track(&m, V_LINE == g_vc.state);
    }
    if (V_LINE == g_vc.state)
    {
        drive(g_vc.snap.left_mm_s, g_vc.snap.right_mm_s);
    }
    else
    {
        int16_t l = 0;
        int16_t r = 0;

        switch (s5_avoid_step(&m, g_vc.snap.left_black,
                              g_vc.snap.right_black, (int32_t)VC_MS, &l,
                              &r))
        {
            case S5_AVOID_DONE:
                g_vc.sent_l = INT16_MIN;
                g_vc.sent_r = INT16_MIN;
                g_vc.state = V_LINE;
                done = 1;
                break;
            case S5_AVOID_FAULT:
                stop_motors();
                g_vc.state = V_FAULT;
                g_vc.fault = 2;
                break;
            case S5_AVOID_RUNNING:
            default:
                drive(l, r);
                break;
        }
    }
    return done;
}

/*=====================================================================*/
/* A run                                                                 */
/*=====================================================================*/

typedef struct
{
    int dones;
    int side;          /* from the log: +1 LEFT, -1 RIGHT */
    int fault;
    double min_clear;
    double max_y;      /* axle, furthest from the line, mm */
    double off_done;   /* IR centre to our line at the handover */
    double max_after;  /* |IR centre to the line| after the handover */
    int lane_end;
    double width;      /* logged width (first sweep) */
    int blocked_seen;
    double stop_face;  /* sensor to the face when the car stopped */
    double ep_s;
    int min_wheel;
    int stopped;
    int longest;
    double rev;        /* backed up in the episode, mm */
} res_t;

static int g_verbose;
/* S5 task poll, ms: S5_POLL_MS from the cyclic handler; -p 20 is the
 * fallback (tk_slp_tsk timing out at the second tick) */
static uint32_t g_poll_ms = S5_POLL_MS;

static void place(const scen_t *s, double *p_bx, double *p_by,
                  double *p_th)
{
    /* where the straight with the box starts, its direction */
    sim_track_start();
    if (s->bend_r > 0.0)
    {
        sim_track_straight(400.0);
        sim_track_arc(s->bend_r, s->bend_deg);
        {
            const double a = s->bend_deg * PI / 180.0;

            *p_bx = s->bend_r * sin(a);
            *p_by = s->bend_r * (1.0 - cos(a));
            *p_th = a;
        }
        sim_track_straight(5000.0);
    }
    else if (s->turn_r > 0.0)
    {
        /* the line bends behind the box */
        sim_track_straight(s->turn_x + 400.0); /* starts at -400 */
        sim_track_arc(s->turn_r, s->turn_deg);
        sim_track_straight(5000.0);
        *p_bx = 0.0;
        *p_by = 0.0;
        *p_th = 0.0;
    }
    else if (0.0 != s->shift)
    {
        /* two R600 arcs, from the box's face on */
        const double r = 600.0;
        const double th =
            acos(1.0 - (fabs(s->shift) / (2.0 * r))) * 180.0 / PI;
        const double sg = (s->shift > 0.0) ? 1.0 : -1.0;

        sim_track_straight(s->box[0].face);
        sim_track_arc(r, sg * th);
        sim_track_arc(r, -sg * th);
        sim_track_straight(5000.0);
        *p_bx = 0.0;
        *p_by = 0.0;
        *p_th = 0.0;
    }
    else
    {
        sim_track_straight(6400.0);
        *p_bx = 0.0;
        *p_by = 0.0;
        *p_th = 0.0;
    }
}

/* along a straight track: the far side of the last object */
static double last_far(const scen_t *s)
{
    double far = 0.0;
    int k;

    for (k = 0; k < s->n_box; k++)
    {
        const double f = s->box[k].face + s->box[k].d;

        far = (f > far) ? f : far;
    }
    return far;
}

static res_t run(const scen_t *s, uint32_t seed)
{
    res_t o;
    double bx;
    double by;
    double bth;
    uint32_t t = 0U;
    uint32_t first_stop_ms = 0U;
    double after_mm = -1.0;
    double ep_start = 0.0;
    int k;
    int in_ep = 0;

    memset(&o, 0, sizeof o);
    o.min_clear = 1e9;
    o.min_wheel = 0x7FFF;

    sim_reset(seed);
    sim_log_verbose(g_verbose);
    g_sim.track_deg = s->track_deg;
    if (s->beam > 0.0)
    {
        g_sim.beam_deg = s->beam;
    }
    g_sim.servo_trim_deg = s->servo_trim;
    if (s->servo_dps > 0.0)
    {
        g_sim.servo_dps = s->servo_dps;
    }
    if (s->no_echo_ms > 0.0)
    {
        g_sim.no_echo_ms = s->no_echo_ms;
    }
    if (s->noise > 0.0)
    {
        g_sim.noise_mm = s->noise;
    }
    if (s->drop > 0.0)
    {
        g_sim.drop = s->drop;
    }
    g_sim.sonar_left = s->sonar_left;
    g_sim.dead = s->dead_sonar;
    if (s->lanes > 0.0)
    {
        g_sim.lane[1] = s->lanes;
        g_sim.lane[2] = -s->lanes;
        g_sim.n_lane = 3;
    }
    place(s, &bx, &by, &bth);
    if (0 != s->no_mag)
    {
        g_sim.mag_on = 0;
    }
    if (s->mag_err > 0.0)
    {
        g_sim.mag_err_deg = s->mag_err;
    }
    if (s->mag_iron > 0.0)
    {
        g_sim.mag_iron = s->mag_iron;
        g_sim.mag_iron_mm = 300.0;
    }
    if (s->junc_x > 0.0)
    {
        const double a = s->junc_deg * PI / 180.0;
        const double len = (s->junc_len > 0.0) ? s->junc_len : 400.0;

        g_sim.seg[0][0] = s->junc_x;
        g_sim.seg[0][1] = 0.0;
        g_sim.seg[0][2] = s->junc_x + (len * cos(a));
        g_sim.seg[0][3] = len * sin(a);
        g_sim.n_seg = 1;
    }
    g_sim.n_box = s->n_box;
    for (k = 0; k < s->n_box; k++)
    {
        const box_t *b = &s->box[k];
        const double along = b->face + b->d / 2.0;

        g_sim.box[k].cx = bx + along * cos(bth) - b->y * sin(bth);
        g_sim.box[k].cy = by + along * sin(bth) + b->y * cos(bth);
        g_sim.box[k].w = b->w;
        g_sim.box[k].d = b->d;
        g_sim.box[k].rot_deg = bth * 180.0 / PI + b->rot;
        g_sim.box[k].on = (b->appear_x <= 0.0);
    }

    sim_car_place(-150.0, s->start_y, s->start_th);
    mm_set_errors(s->enc_err, s->track_err);
    mm_set_lr_error(s->lr_err);
    mm_set_dead_right(0);
    mm_set_lag(60, (double)B3_BASE_SPEED_MM_S);
    (void)mm_min_wheel_cmd(1);

    memset(&g_vc, 0, sizeof g_vc);
    g_vc.sent_l = INT16_MIN;
    g_vc.sent_r = INT16_MIN;
    b3_line_init(&g_vc.line, sim_now_us());
    (void)s5_hw_init();
    s5_sensor_init();
    s5_avoid_init();

    while (t < LIMIT_MS)
    {
        double x;
        double y;
        double th;

        sim_car_pose(&x, &y, &th);

        /* boxes put down or taken away */
        for (k = 0; k < s->n_box; k++)
        {
            const box_t *b = &s->box[k];

            if ((b->appear_x > 0.0) && !g_sim.box[k].on && (x >= b->appear_x))
            {
                g_sim.box[k].on = 1;
            }
            if ((b->gone_ms > 0.0) && (0U != first_stop_ms) &&
                g_sim.box[k].on && (t >= first_stop_ms + (uint32_t)b->gone_ms))
            {
                g_sim.box[k].on = 0;
                if (g_verbose)
                {
                    (void)printf("    %7.3f  [SIM] box %d taken away\n",
                                 t / 1000.0, k + 1);
                }
            }
        }

        /* S5 task */
        if (3U == (t % g_poll_ms))
        {
            s5_sensor_poll();
        }
        /* Buddy 3's task */
        if (0U == (t % 10U))
        {
            g_vc.snap = b3_line_step(&g_vc.line, sim_ir_black(true),
                                     sim_ir_black(false), sim_now_us(),
                                     (int16_t)B3_BASE_SPEED_MM_S);
            if (V_LINE != g_vc.state)
            {
                b3_line_hold(&g_vc.line); /* ctx.steer_hold */
            }
        }
        /* vc_task */
        if (5U == (t % VC_MS))
        {
            const vstate_t was = g_vc.state;

            if (vc_pass())
            {
                o.dones++;
                {
                    double off;

                    (void)sim_ir_lane(&off);
                    o.off_done = off;
                }
                o.ep_s = (t - ep_start) / 1000.0;
                after_mm = 0.0;
                in_ep = 0;
            }
            if ((V_LINE == was) && (V_OBST == g_vc.state))
            {
                in_ep = 1;
                after_mm = -1.0; /* the next handover starts it again */
                o.max_after = 0.0;
                ep_start = (double)t;
                if (0U == first_stop_ms)
                {
                    first_stop_ms = t;
                }
                if (0.0 == o.stop_face)
                {
                    double sx;
                    double sy;

                    sim_body(70.0, 0.0, &sx, &sy);
                    o.stop_face = (s->box[0].face) - sx; /* straight only */
                }
                if (s->dead_right)
                {
                    mm_set_dead_right(1);
                }
            }
            if (s5_avoid_blocked())
            {
                o.blocked_seen = 1;
            }
        }

        mm_step_1ms();
        sim_step_1ms();
        t++;

        if (0U == (t % 5U))
        {
            const double c = sim_clearance();
            double off;
            const int lane = sim_ir_lane(&off);

            if (c < o.min_clear)
            {
                o.min_clear = c;
                if (g_verbose && (c < 25.0))
                {
                    (void)printf("    %7.3f  [SIM] %.0f mm from a box, car x %.0f "
                                 "y %.0f th %.1f\n",
                                 t / 1000.0, c, x, y, th * 180.0 / PI);
                }
            }
            if (in_ep && (mm_speed_true() < 0.0))
            {
                o.rev += -mm_speed_true() * 0.005;
            }
            if (in_ep && (fabs(y) > o.max_y))
            {
                o.max_y = fabs(y);
            }
            if (after_mm >= 300.0)
            {
                if ((0 != lane) || (fabs(off) > o.max_after))
                {
                    o.max_after = (0 != lane) ? 999.0 : fabs(off);
                }
            }
            o.lane_end = lane;
        }
        if (after_mm >= 0.0)
        {
            after_mm += fabs(mm_speed_true()) * 0.001;
            /* an object met on the way back from the
             * last one is gone round in the same OBSTACLE state, so with
             * several objects (straight track) the run ends
             * once the car is past the last of them */
            if ((after_mm >= AFTER_MM) &&
                ((s->n_obst > 1) ? (x > last_far(s))
                                 : (o.dones >= 1)))
            {
                break;
            }
        }
        if ((V_FAULT == g_vc.state) && (0U == o.stopped))
        {
            o.stopped = (int)t;
        }
        if ((0 != o.stopped) && (t > (uint32_t)o.stopped + 1500U))
        {
            break;
        }
        if (g_verbose >= 2 && (0U == (t % 100U)))
        {
            motion_status_t ms;

            motion_get_status(&ms);
            (void)printf("    %7.3f  car x %7.1f y %6.1f th %6.1f  servo %6.1f"
                         "  cmd %4d %4d  st %d  b2 %d v %d %d  ir %d%d\n",
                         t / 1000.0, x, y, th * 180.0 / PI, sim_servo_deg(),
                         g_vc.sent_l, g_vc.sent_r, (int)g_vc.state,
                         (int)ms.state, ms.speed_l_mmps, ms.speed_r_mmps,
                         sim_ir_black(true), sim_ir_black(false));
        }
    }
    o.fault = g_vc.fault;
    o.min_wheel = mm_min_wheel_cmd(0);
    {
        const int i = sim_log_find("going ", 0);
        const int w = sim_log_find(" mm wide:", 0);

        if (i >= 0)
        {
            o.side = (NULL != strstr(sim_log_line(i), "LEFT")) ? 1 : -1;
        }
        if (w >= 0)
        {
            const char *p = strstr(sim_log_line(w), "object ");

            o.width = (NULL != p) ? atof(p + 7) : 0.0;
        }
    }
    o.longest = sim_log_longest();
    return o;
}

static int judge(const scen_t *s, const res_t *o, char *p_why, size_t n)
{
    p_why[0] = '\0';
    if (o->longest > 159)
    {
        (void)snprintf(p_why, n, "a log line has %d characters", o->longest);
        return 0;
    }
    if (o->min_clear < 0.0)
    {
        (void)snprintf(p_why, n, "HIT the box");
        return 0;
    }
    switch (s->expect)
    {
        case X_ECHO:
            if (1 != o->fault)
            {
                (void)snprintf(p_why, n, "no FR25 sensor fault");
                return 0;
            }
            return 1;
        case X_OBST:
            if (2 != o->fault)
            {
                (void)snprintf(p_why, n, "no obstacle fault");
                return 0;
            }
            return 1;
        case X_CLEAR:
            if ((0 != o->fault) || (o->dones < 1) || (o->max_after > 24.0))
            {
                (void)snprintf(p_why, n, "did not go on along the line");
                return 0;
            }
            return 1;
        case X_BLOCKED:
            if (!o->blocked_seen)
            {
                (void)snprintf(p_why, n, "never showed blocked");
                return 0;
            }
            break;
        case X_ROUND:
        default:
            break;
    }
    if (0 != o->fault)
    {
        (void)snprintf(p_why, n, "fault %d", o->fault);
        return 0;
    }
    if (o->dones < 1)
    {
        (void)snprintf(p_why, n, "never back on the line");
        return 0;
    }
    if (o->min_clear < 10.0)
    {
        (void)snprintf(p_why, n, "only %.0f mm from the box", o->min_clear);
        return 0;
    }
    if ((0 != s->side) && (o->side != s->side))
    {
        (void)snprintf(p_why, n, "went the wrong way");
        return 0;
    }
    if ((s->rev_max > 0.0) && (o->rev > s->rev_max))
    {
        (void)snprintf(p_why, n, "backed up %.0f mm", o->rev);
        return 0;
    }
    if (fabs(o->off_done) > 12.0)
    {
        (void)snprintf(p_why, n, "handed over %.0f mm off the line",
                       o->off_done);
        return 0;
    }
    if ((0 != o->lane_end) || (o->max_after > 24.0))
    {
        (void)snprintf(p_why, n, "lost the line after the handover");
        return 0;
    }
    return 1;
}

static int g_only_seed = 0; /* -k: run only this seed */

int main(int argc, char **argv)
{
    int seeds = 3;
    const char *p_only = NULL;
    int failed = 0;
    int runs = 0;
    int i;

    for (i = 1; i < argc; i++)
    {
        if ((0 == strcmp(argv[i], "-s")) && (i + 1 < argc))
        {
            seeds = atoi(argv[++i]);
        }
        else if ((0 == strcmp(argv[i], "-p")) && (i + 1 < argc))
        {
            g_poll_ms = (uint32_t)atoi(argv[++i]);
            g_poll_ms = (g_poll_ms < 5U) ? 5U : g_poll_ms;
        }
        else if ((0 == strcmp(argv[i], "-k")) && (i + 1 < argc))
        {
            g_only_seed = atoi(argv[++i]);
        }
        else if (0 == strcmp(argv[i], "v"))
        {
            g_verbose = 1;
        }
        else if (0 == strcmp(argv[i], "vv"))
        {
            g_verbose = 2;
        }
        else
        {
            p_only = argv[i];
        }
    }
    if (NULL != p_only)
    {
        seeds = (seeds > 0) ? seeds : 1;
    }
    (void)printf("S5 task polls every %u ms\n", (unsigned)g_poll_ms);
    (void)printf("%-4s %-58s %5s %4s %6s %5s %5s %6s %4s %6s %5s\n", "",
                 "scenario", "side", "wide", "clear", "max y", "hand", "after",
                 "rev", "time", "ok");
    for (i = 0; i < N_SCEN; i++)
    {
        const scen_t *s = &g_scen[i];
        int seed;
        int lefts = 0;
        int rights = 0;

        if ((NULL != p_only) && (0 != strcmp(p_only, s->id)))
        {
            continue;
        }
        for (seed = 1; seed <= seeds; seed++)
        {
            char why[96];
            res_t o;
            int ok;

            if ((0 != g_only_seed) && (seed != g_only_seed))
            {
                continue;
            }
            o = run(s, (uint32_t)(seed * 7919 + i));
            ok = judge(s, &o, why, sizeof why);

            runs++;
            lefts += (1 == o.side);
            rights += (-1 == o.side);
            if (!ok && ('L' != s->id[0]))
            {
                failed++;
            }
            (void)printf("%-4s %-58.58s %5s %4.0f %6.0f %5.0f %5.0f %6.1f %4.0f "
                         "%5.1fs %s %s\n",
                         s->id, s->name,
                         (1 == o.side) ? "LEFT" : ((-1 == o.side) ? "RIGHT"
                                                                   : "-"),
                         o.width, (o.min_clear > 1e8) ? -1.0 : o.min_clear,
                         o.max_y, o.off_done, o.max_after, o.rev, o.ep_s,
                         ok ? "PASS" : (('L' == s->id[0]) ? "LIMIT" : "FAIL"),
                         why);
        }
        (void)lefts;
        (void)rights;
    }
    (void)printf("\n%d runs, %d failed\n", runs, failed);
    return (0 == failed) ? 0 : 1;
}
