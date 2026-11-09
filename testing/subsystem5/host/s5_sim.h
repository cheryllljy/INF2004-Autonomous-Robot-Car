/**
 * @file    s5_sim.h
 * @brief   HOST TEST ONLY (nyx, 2026-11-07). The world subsystem 5 sees
 *          in the host tests, and the s5_hw.h functions working on it.
 *
 *   simulated  boxes; our line and other lanes' lines (18 mm tape); the
 *              HC-SR04P on the SG90 (servo slew, trim, echo cone, corner
 *              echoes, noise, dropouts, the no-echo timeout pulse, a dead
 *              sensor); the IR pair; the microsecond clock (starts 3 s
 *              before it wraps, so every time comparison is tested
 *              across the wrap)
 *   real       everything above s5_hw.h (s5_sensor.c, s5_scan.c, ...)
 *
 * Frames: the track frame has X along our line (where it starts), Y to
 * its left, mm, angles + = anticlockwise. The car's pose comes from
 * motion_mock.c, whose frame is the track frame turned by track_deg
 * (mock x = left, mock y = forward).
 *
 * Sonar model: a flat face answers when the sensor looks within beam_deg
 * of square to it, with the perpendicular distance; a corner answers when
 * it is within corner_frac x beam_deg of where the sensor looks, with its
 * distance; the nearest answer wins; a box hidden behind another does not
 * answer.
 */
#ifndef S5_SIM_H
#define S5_SIM_H

#include <stdbool.h>
#include <stdint.h>

#define SIM_MAX_BOX  (8)
#define SIM_MAX_LANE (4)
#define SIM_MAX_PTS  (4000)
#define SIM_MAX_SEG  (4)

typedef struct
{
    double cx;      /* centre, track frame, mm */
    double cy;
    double w;       /* across the line (Y), mm */
    double d;       /* along the line (X), mm */
    double rot_deg; /* turned, + = anticlockwise */
    int    on;
} sim_box_t;

typedef struct
{
    /* sonar */
    double beam_deg;     /* half-angle within which a face answers */
    double corner_frac;  /* corners answer within corner_frac x beam */
    double noise_mm;     /* range noise, sd */
    double drop;         /* chance that a ping hears nothing */
    double temp_c;       /* real air temperature */
    double range_off_mm; /* the sensor reads this much long */
    double sonar_ahead;  /* real sensor face, mm ahead of the axle */
    double sonar_left;   /* ... and to the left of the centre line */
    double no_echo_ms;   /* ECHO stays high this long when nothing answers */
    int    dead;         /* ECHO never rises */
    /* servo */
    double servo_dps;      /* slew rate, deg/s */
    double servo_trim_deg; /* real angle minus the angle asked */
    int    servo_invert;   /* the servo turns the other way */
    /* IR pair */
    double ir_ahead;
    double ir_half;
    /* track: our line is the polyline (track frame); lanes are copies of
     * it moved sideways by lane[k] mm (lane 0 = ours, offset 0) */
    double lane[SIM_MAX_LANE];
    int    n_lane;
    /* nyx, 2026-11-08: straight tape pieces (a junction's branch), track
     * frame, from (x0, y0) to (x1, y1) */
    double seg[SIM_MAX_SEG][4];
    int    n_seg;
    /* nyx, 2026-11-08: the compass. The earth's field (mag_r counts)
     * points mag_dir_deg in the track frame; seen from the car it turns
     * the other way (so S5_MAG_SIGN -1); hard-iron offset mag_x0 / y0;
     * a heading error of mag_err_deg x sin(2 x heading) left after
     * calibration; mag_iron: within mag_iron_mm of the first box's
     * centre the field is pulled by that many counts along X */
    int    mag_on;
    double mag_r;
    double mag_dir_deg;
    double mag_x0;
    double mag_y0;
    double mag_err_deg;
    double mag_iron;
    double mag_iron_mm;
    double track_deg;
    sim_box_t box[SIM_MAX_BOX];
    int    n_box;
} sim_world_t;

extern sim_world_t g_sim;

/* defaults, a straight line 6 m long, no boxes, seeded noise */
void     sim_reset(uint32_t seed);
/* our line: turtle from (0, 0) heading along X; then straights and arcs */
void     sim_track_start(void);
void     sim_track_straight(double len);
void     sim_track_arc(double radius, double angle_deg); /* + = left */

void     sim_step_1ms(void); /* clock and servo */
uint32_t sim_now_us(void);
uint32_t sim_ms(void);

/* the car (motion_mock.c truth), track frame */
void     sim_car_place(double x, double y, double th_deg);
void     sim_car_pose(double *p_x, double *p_y, double *p_th_rad);
void     sim_body(double ahead, double left, double *p_x, double *p_y);

bool     sim_ir_black(bool b_left);
/* nearest lane to the IR pair's centre and the distance to it (signed,
 * + = the IR centre is left of that lane) */
int      sim_ir_lane(double *p_off);
/* car outline (front 135, rear 45, half-width 65) to the nearest box;
 * < 0 = they overlap */
double   sim_clearance(void);
double   sim_servo_deg(void);
/* what the sensor would hear now (0 = nothing), mm, without noise */
double   sim_sonar_now(void);
uint32_t sim_pings(void);

/* S5_LOG lines (s5_port.h with S5_HOST) */
void        sim_log_verbose(int verbose);
int         sim_log_count(void);
const char *sim_log_line(int i);
int         sim_log_find(const char *p_text, int from); /* index or -1 */
int         sim_log_longest(void);

#endif /* S5_SIM_H */
