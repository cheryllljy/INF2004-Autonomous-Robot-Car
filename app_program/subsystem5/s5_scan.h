/*
 *      s5_scan.h
 *      Subsystem 5 -- what one sweep says about the object. Pure
 *      functions: no kernel, no hardware, so the host tests run the same
 *      code.
 *
 *      How the width is measured
 *      -------------------------
 *      The car stands still and the sensor sweeps from 60 deg left to
 *      60 deg right in 6 deg steps, then halves the step twice at each
 *      edge (1.5 deg). The nearest return near the middle is the face
 *      (FACE). A sample is part of the object when it comes back no more
 *      than S5_EDGE_DEPTH_MM behind the face; the object's samples around
 *      the face form one run, and where that run stops on each side is
 *      where the echoes stop.
 *
 *      The HC-SR04's beam is wide, so the echoes go on past each corner
 *      by the beam half-angle (BEAM, about 22 deg for a flat box). The
 *      corner is therefore BEAM inside where the echoes stop, and it sits
 *      on the face, so
 *
 *          corner lateral = FACE x tan(heading + bearing of the corner)
 *
 *      in the line frame (the face is taken square to the line, as a box
 *      put on the track is). BEAM comes from calibration step 3 with a
 *      box of known width, at two distances; between them it is
 *      interpolated. Width = left corner - right corner.
 *
 *      Edges by range (S5_EDGE_BY_RANGE)
 *      ---------------------------------
 *      The angle method above needs BEAM and the servo's angle to be
 *      right, and on the car neither is: BEAM changes with the object and
 *      the distance (8..33 deg), and the servo is 5-10 deg off (its zero,
 *      the sweep's lag, the wires holding it back on the right). 10 deg
 *      at 280 mm is 50 mm of error.
 *
 *      The ranges do not have that problem. While the sensor looks at the
 *      face they all read FACE (the nearest point, straight in front).
 *      Turned towards a corner, the last echoes before they stop come
 *      from the corner itself, so they read the distance to the corner,
 *      EDGE, a little longer than FACE. Pythagoras:
 *
 *          corner, sideways from the sonar = sqrt(EDGE^2 - FACE^2)
 *
 *      No angle and no BEAM in it.
 *
 *      FACE is the mean of the flat middle of the object's ranges (those
 *      within S5_EDGE_JUMP_MM of their lower quartile). EDGE is the mean
 *      of the two longest ranges from the face out to the last echo that
 *      can be trusted: one with no jump to the echo next to it nor to the
 *      next one at least S5_EDGE_PAIR_DDEG further in (an
 *      echo that jumps is a mix with whatever is behind the object). A few
 *      mm are added to each side (S5_EDGE_PAD_MM2) so that noise does not
 *      put a corner inside the real one.
 *
 *      What range cannot do, and what happens then:
 *      - a corner under S5_EDGE_MIN_MM from the sonar's axis, or the near
 *        corner of a box that stands turned: that side by angle, as before
 *        (at least half of S5_EDGE_MIN_MM out);
 *      - echoes right to the end of the sweep on a side (a wall, another
 *        object): no side closer in than by angle;
 *      - a sweep from off the line or not straight (after going round
 *        another object the sonar can be out beside this one), or nothing
 *        straight ahead: all by angle, as before.
 *      Both results are logged after every sweep.
 *
 *      Room on each side: anything else the sweep saw beyond an edge
 *      (another box, a wall) answered from somewhere within BEAM of the
 *      angle it was seen at (the object's own, nearer echoes can hide
 *      where it starts). If any such place lies where the car would drive
 *      round on that side, that side is blocked.
 */

#ifndef S5_SCAN_H
#define S5_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#define S5_SCAN_MAX (40U)

typedef struct
{
    int16_t ddeg; /* servo angle, car frame, + = left */
    uint16_t mm;  /* 0 = nothing came back (clear) */
} s5_sample_t;

/* A return left out because it may have come from the object just gone
 * round. Neither the object nor clear: an edge next to one is not seen. */
#define S5_MM_UNKNOWN (0xFFFFU)

/* Where the sonar was for the sweep, in the line frame. */
typedef struct
{
    float x;   /* mm */
    float y;   /* mm, + = left of the line */
    float psi; /* car heading against the line, rad, + = left */
} s5_view_t;

typedef struct
{
    bool b_found;        /* an object ahead (a face within the cone) */
    uint16_t face_mm;    /* sonar to the face */
    bool b_left_seen;    /* the left edge is inside the sweep */
    bool b_right_seen;   /* the right edge is inside the sweep */
    int16_t left_ddeg;   /* car frame: where the echoes stop, left */
    int16_t right_ddeg;  /* ... and right */
    int16_t beam_ddeg;   /* beam half-angle used */
    float x_face;        /* line frame, mm */
    float y_left;        /* left corner (line frame, + = left) */
    float y_right;       /* right corner */
    float width;         /* y_left - y_right */
    /* how the corners were placed (for the log) */
    bool b_by_range;     /* true: by range; false: by angle */
    bool b_narrow;       /* placed by the middle of its echoes */
    float y_mid;         /* ... that middle, line frame (mm) */
    bool b_limited;      /* S5_OBJ_MIN_MM / MAX_MM applied */
    uint16_t face_q_mm;  /* FACE used by the range method */
    uint16_t edge_l_mm;  /* EDGE, left and right */
    uint16_t edge_r_mm;
    float ya_left;       /* the angle method's corners (line frame), */
    float ya_right;      /* before S5_WIDTH_TRIM_MM */
    bool b_left_blocked; /* something else where the car would pass */
    bool b_right_blocked;
    /* Only the edge of the beam of an echo reaches where the car would
     * pass (its own direction is outside the pass lane): that side is
     * used only when no side is clear. */
    bool b_left_maybe;
    bool b_right_maybe;
    /* That edge is next to returns left out (S5_MM_UNKNOWN): the object
     * just gone round is that way, and the edge is not seen. */
    bool b_left_hidden;
    bool b_right_hidden;
    /* the echo that blocked a side (for the log) */
    int16_t block_l_ddeg;
    uint16_t block_l_mm;
    int16_t block_r_ddeg;
    uint16_t block_r_mm;
} s5_profile_t;

/* Nearest return in the face cone, 0 = none. */
uint16_t s5_scan_face(const s5_sample_t *p_s, uint32_t n);

/* Is a return of mm part of the object whose face is face_mm away? */
bool s5_scan_is_object(uint16_t mm, uint16_t face_mm);
/* Echoes up to mm behind the face count as the object's (at most
 * S5_EDGE_DEPTH_MM; 0: S5_EDGE_DEPTH_MM again). */
void s5_scan_set_depth(uint16_t mm);

/* Where the object's echoes stop on each side. *p_*_obj: outermost angle
 * that still answered; *p_*_clr: next angle out that did not (only valid
 * when the function sets *p_*_seen). Samples sorted by angle. */
void s5_scan_edges(const s5_sample_t *p_s, uint32_t n, uint16_t face_mm,
                   int16_t *p_left_obj, int16_t *p_left_clr, bool *p_left_seen,
                   int16_t *p_right_obj, int16_t *p_right_clr,
                   bool *p_right_seen);

/* Beam half-angle for a face this far away (calibration step 3). */
int16_t s5_scan_beam_ddeg(uint16_t face_mm);

/* Sort by angle, right (-) to left (+). */
void s5_scan_sort(s5_sample_t *p_s, uint32_t n);

/* S5_AUTO_TRIM. True and *p_corr (0.1 deg, add to the servo trim) if
 * this sweep, taken standing with an object straight ahead, is one to
 * trim by. */
bool s5_scan_trim_from(const s5_profile_t *p_pr, int16_t *p_corr);

/* S5_TWO_DIST. Width (mm) and half beam (rad) from two sweeps d1 < d2
 * from the face, echoes spanning span1, span2 (rad), the object's middle
 * yc to the side of the sonar (mm). False: no sensible answer. */
bool s5_scan_two_width(float d1, float span1, float d2, float span2,
                       float yc, float *p_w, float *p_beam);

/* The whole profile. Samples sorted by angle. */
void s5_scan_profile(const s5_sample_t *p_s, uint32_t n,
                     const s5_view_t *p_view, s5_profile_t *p_out);

/* Lateral offset the axle needs to pass the edge at y_edge on side
 * (+1 left, -1 right). */
float s5_scan_offset(float y_edge, int32_t side);

/* Could a return of mm, looked for along 'look' (line frame, rad) with a
 * beam half-angle 'half', have come from a corner at y = y_c lying
 * somewhere from x_lo to x_hi in front of the sonar at (vx, vy)? Range
 * within tol. Used for corners whose depth is not known exactly. */
bool s5_scan_corner_echo(float vx, float vy, float look, float half, float mm,
                         float y_c, float x_lo, float x_hi, float tol);

#endif /* S5_SCAN_H */
