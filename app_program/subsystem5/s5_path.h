/*
 *      s5_path.h
 *      Subsystem 5 -- the path round the object and how the car follows
 *      it. Pure: no kernel, the host tests use the same code.
 *
 *      Everything is in the line frame: x along the line (driving
 *      direction), y to its left, origin where the IR pair was when the
 *      object was seen. The path gives the axle's y for each x:
 *
 *          y0 ......                                    ...... y_end
 *                   \  ramp out            ramp back  /
 *                    \______________________________/
 *                    a (the pass, beside the object)
 *
 *         x0    x0+l_out                   x_back   x_back+l_in
 *
 *      Each ramp is a minimum-jerk curve, y = start + (end - start) S(t),
 *      S(t) = 10t^3 - 15t^4 + 6t^5, t = 0..1 along the ramp: the heading
 *      and the curvature are both 0 where a ramp starts and ends, so the
 *      wheels never get a step change, and the steepest heading and the
 *      tightest radius are known in advance (s5_path_ramp_len()).
 *
 *      Following it: the car's distance from the path and its heading error
 *      die out together over S5_TRACK_DIST_MM (critically damped), on top
 *      of the path's own curvature.
 */

#ifndef S5_PATH_H
#define S5_PATH_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x0;     /* ramp out starts */
    float l_out;  /* its length */
    float y0;     /* the axle's y before it */
    float a;      /* the pass: axle's y beside the object */
    float x_back; /* ramp back starts */
    float l_in;   /* its length */
    float y_end;  /* the line's y (0 until the IRs find it) */
} s5_path_t;

typedef struct
{
    float y;     /* axle y the path wants here */
    float slope; /* dy/dx */
    float curv;  /* curvature, 1/mm, + = turning left */
} s5_ref_t;

/* The object's footprint, line frame. */
typedef struct
{
    float x_near;
    float x_far;
    float y_right;
    float y_left;
} s5_box_t;

/* Shortest ramp for a sideways move of dy: no steeper than
 * S5_MAX_HEADING_DEG, no tighter than S5_MIN_RADIUS_MM. */
float s5_path_ramp_len(float dy);

/* The steepest ramp allowed (S5_STEEP_HEADING_DEG, S5_STEEP_RADIUS_MM),
 * for turning out where the gentle one has no room. */
float s5_path_ramp_len_steep(float dy);
/* the ramp back in (S5_IN_HEADING_DEG, S5_IN_RADIUS_MM) */
float s5_path_ramp_len_in(float dy);

void s5_path_eval(const s5_path_t *p_p, float x, s5_ref_t *p_ref);

/* Where the ramp back ends. */
float s5_path_end(const s5_path_t *p_p);

/* Does the whole car (S5_CAR_* outline) stay at least margin away from
 * the box while its axle follows the path from x_from to x_to? */
bool s5_path_clear(const s5_path_t *p_p, const s5_box_t *p_box, float margin,
                   float x_from, float x_to);

/* Does the car (S5_CAR_* outline) with its axle at (x, y), heading psi,
 * come within margin of the box? */
bool s5_path_car_hits(float x, float y, float psi, const s5_box_t *p_box,
                      float margin);

/* Curvature to ask for, 1/mm: the path's own plus the correction for a
 * car at (y, psi) at this x; errors die out over s0 mm. Clamped to
 * S5_MAX_CURV_PPM. */
float s5_path_steer(const s5_ref_t *p_ref, float y, float psi, float s0);

/* Wheel speeds for speed v (mm/s, < 0 = backwards) on curvature curv
 * (per mm of path). A moving wheel never gets less than
 * S5_MIN_WHEEL_MM_S. */
void s5_path_wheels(float v, float curv, int16_t *p_l, int16_t *p_r);

#endif /* S5_PATH_H */
