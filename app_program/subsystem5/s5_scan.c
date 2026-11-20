/*
 *      s5_scan.c
 *      Subsystem 5 -- what one sweep says about the object. See s5_scan.h for
 *      the method.
 */

#include "s5_scan.h"

#include "s5_config.h"
#include "s5_math.h"

/* returns further than this are scenery, not something in the way */
#define S5_CORRIDOR_RANGE_MM (900U)
/* the strip beside the object the car drives through, plus this */
#define S5_CORRIDOR_MARGIN_MM (20.0f)
/* how far past the face the corridor is checked */
#define S5_CORRIDOR_AHEAD_MM (400.0f)
/* never place anything further round than this */
#define S5_BEARING_MAX_DDEG (850)
/* the object's own far corner (sonar beside the object): range and beam
 * tolerance */
#define S5_OWN_TOL_MM    (20.0f)
#define S5_OWN_BEAM_DDEG (30)

static float s5_ddeg_to_rad(int32_t ddeg)
{
    return (float)ddeg * (S5_DEG2RAD_F / 10.0f);
}

uint16_t s5_scan_face(const s5_sample_t *p_s, uint32_t n)
{
    uint16_t face = 0U;
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        const int32_t a = (int32_t)p_s[i].ddeg;

        if ((0U != p_s[i].mm) && (S5_MM_UNKNOWN != p_s[i].mm) &&
            (a <= S5_FACE_CONE_DDEG) && (a >= -S5_FACE_CONE_DDEG) &&
            ((0U == face) || (p_s[i].mm < face)))
        {
            face = p_s[i].mm;
        }
    }
    return face;
}

/* Buddy 5, 2026-11-19: how far behind the face an echo is still the
 * object's: S5_EDGE_DEPTH_MM, or less for one sweep (s5_scan_set_depth()) */
static uint16_t g_depth_mm = (uint16_t)S5_EDGE_DEPTH_MM;

void s5_scan_set_depth(uint16_t mm)
{
    g_depth_mm = ((0U == mm) || (mm > (uint16_t)S5_EDGE_DEPTH_MM))
                     ? (uint16_t)S5_EDGE_DEPTH_MM
                     : mm;
}

bool s5_scan_is_object(uint16_t mm, uint16_t face_mm)
{
    return (0U != mm) && (S5_MM_UNKNOWN != mm) &&
           ((uint32_t)mm <= ((uint32_t)face_mm + (uint32_t)g_depth_mm));
}

/* index of the face sample: the nearest return in the cone, the one
 * closest to straight ahead if several are equal */
static uint32_t s5_face_index(const s5_sample_t *p_s, uint32_t n,
                              uint16_t face_mm)
{
    uint32_t best = n;
    int32_t best_abs = 0x7FFF;
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        int32_t a = (int32_t)p_s[i].ddeg;

        if (a < 0)
        {
            a = -a;
        }
        if ((p_s[i].mm == face_mm) && (a <= S5_FACE_CONE_DDEG) &&
            (a < best_abs))
        {
            best = i;
            best_abs = a;
        }
    }
    return best;
}

/* Walk out from the face in direction dir (+1 = up the array = left)
 * while the samples belong to the object. One sample with no echo at
 * all between two of the object's is a dropout, not a gap. Returns the
 * index of the outermost object sample; *p_clr is the first sample past
 * it, or n (or -1) if the run reaches the end of the sweep. */
static int32_t s5_walk(const s5_sample_t *p_s, int32_t n, int32_t i0,
                       int32_t dir, uint16_t face_mm, int32_t *p_clr)
{
    int32_t last = i0;
    int32_t j = i0 + dir;

    while ((j >= 0) && (j < n))
    {
        if (s5_scan_is_object(p_s[j].mm, face_mm))
        {
            last = j;
            j += dir;
        }
        else if ((0U == p_s[j].mm) && ((j + dir) >= 0) && ((j + dir) < n) &&
                 s5_scan_is_object(p_s[j + dir].mm, face_mm))
        {
            last = j + dir;
            j += 2 * dir;
        }
        else
        {
            break;
        }
    }
    *p_clr = j;
    return last;
}

void s5_scan_edges(const s5_sample_t *p_s, uint32_t n, uint16_t face_mm,
                   int16_t *p_left_obj, int16_t *p_left_clr, bool *p_left_seen,
                   int16_t *p_right_obj, int16_t *p_right_clr,
                   bool *p_right_seen)
{
    const uint32_t i0 = s5_face_index(p_s, n, face_mm);
    int32_t clr;
    int32_t last;

    *p_left_seen = false;
    *p_right_seen = false;
    *p_left_obj = 0;
    *p_left_clr = 0;
    *p_right_obj = 0;
    *p_right_clr = 0;
    if (i0 >= n)
    {
        return;
    }

    /* an edge next to a return left out (S5_MM_UNKNOWN) is not seen: the
     * object may go on behind it */
    last = s5_walk(p_s, (int32_t)n, (int32_t)i0, 1, face_mm, &clr);
    *p_left_obj = p_s[last].ddeg;
    if ((clr < (int32_t)n) && (S5_MM_UNKNOWN != p_s[clr].mm))
    {
        *p_left_clr = p_s[clr].ddeg;
        *p_left_seen = true;
    }

    last = s5_walk(p_s, (int32_t)n, (int32_t)i0, -1, face_mm, &clr);
    *p_right_obj = p_s[last].ddeg;
    if ((clr >= 0) && (S5_MM_UNKNOWN != p_s[clr].mm))
    {
        *p_right_clr = p_s[clr].ddeg;
        *p_right_seen = true;
    }
}

int16_t s5_scan_beam_ddeg(uint16_t face_mm)
{
    const int32_t near_mm = S5_BEAM_NEAR_MM;
    const int32_t far_mm = S5_BEAM_FAR_MM;
    int32_t d = (int32_t)face_mm;

    if (d < near_mm)
    {
        d = near_mm;
    }
    if (d > far_mm)
    {
        d = far_mm;
    }
    return (int16_t)(S5_BEAM_NEAR_DDEG +
                     (((S5_BEAM_FAR_DDEG - S5_BEAM_NEAR_DDEG) * (d - near_mm)) /
                      (far_mm - near_mm)));
}

void s5_scan_sort(s5_sample_t *p_s, uint32_t n)
{
    uint32_t i;

    for (i = 1U; i < n; i++)
    {
        const s5_sample_t key = p_s[i];
        uint32_t j = i;

        while ((j > 0U) && (p_s[j - 1U].ddeg > key.ddeg))
        {
            p_s[j] = p_s[j - 1U];
            j--;
        }
        p_s[j] = key;
    }
}

float s5_scan_offset(float y_edge, int32_t side)
{
    return y_edge +
           ((float)side * (float)(S5_CAR_HALF_WIDTH_MM + S5_CLEARANCE_MM));
}

bool s5_scan_corner_echo(float vx, float vy, float look, float half, float mm,
                         float y_c, float x_lo, float x_hi, float tol)
{
    const float dy = y_c - vy;
    const float ady = s5_absf(dy);
    const float r_lo = mm - tol;
    const float r_hi = mm + tol;
    float lo;
    float hi;
    float d_lo;
    float d_hi;

    if (r_hi <= ady)
    {
        return false;
    }
    /* x where the corner would be that far away (in front only: a sweep
     * never sees behind the sonar) */
    lo = vx + ((r_lo > ady) ? s5_sqrtf((r_lo * r_lo) - (dy * dy)) : 0.0f);
    hi = vx + s5_sqrtf((r_hi * r_hi) - (dy * dy));
    lo = (lo > x_lo) ? lo : x_lo;
    hi = (hi < x_hi) ? hi : x_hi;
    if (lo > hi)
    {
        return false;
    }
    /* the bearing changes monotonically from lo to hi */
    d_lo = s5_wrapf(s5_atan2f(dy, lo - vx) - look);
    d_hi = s5_wrapf(s5_atan2f(dy, hi - vx) - look);
    if (d_lo > d_hi)
    {
        const float t = d_lo;

        d_lo = d_hi;
        d_hi = t;
    }
    return (d_lo <= half) && (d_hi >= -half);
}

/* with the sonar out beside this side of
 * the object (as after going round another one), it can see along the
 * object's flank to its far corner, which answers from further than the
 * face, past the edge. Such a return is the object itself, not something
 * in the way. */
static bool s5_own_far_corner(const s5_sample_t *p_s, int32_t side,
                              int16_t beam, const s5_view_t *p_view,
                              float y_edge, float x_face)
{
    const float half = s5_ddeg_to_rad((int32_t)beam + S5_OWN_BEAM_DDEG);

    if (((float)side * (p_view->y - y_edge)) <= 0.0f)
    {
        return false; /* the sonar is not out beside this side */
    }
    return s5_scan_corner_echo(
        p_view->x, p_view->y, p_view->psi + s5_ddeg_to_rad(p_s->ddeg), half,
        (float)p_s->mm, y_edge, x_face + (float)S5_DEPTH_MIN_MM,
        x_face + (float)S5_DEPTH_MAX_MM, S5_OWN_TOL_MM);
}

/* Does anything else the sweep saw past the object's edge on this side
 * lie where the car would drive round it? A return at angle a came from
 * somewhere within the beam of a, at that range; where exactly is not
 * known (the object's own echoes can hide where the other one starts), so
 * every point of that arc is tried: any of them in the car's way blocks
 * the side. */
/* 2 = an echo whose own direction is in the pass lane,
 * 1 = an echo that reaches it only within its beam (+-beam, it may be
 * further out), 0 = clear. *p_at: the sample (the first one at the level
 * returned). */
static int32_t s5_side_blocked(const s5_sample_t *p_s, int32_t n,
                               int32_t first, int32_t side, int16_t beam,
                               const s5_view_t *p_view, float y_edge,
                               float y_own, float x_face, int32_t *p_at)
{
    const float pass = s5_scan_offset(y_edge, side);
    const float band_in = y_edge + ((float)side * 10.0f);
    const float band_out =
        pass +
        ((float)side * ((float)S5_CAR_HALF_WIDTH_MM + S5_CORRIDOR_MARGIN_MM));
    const float lo = (side > 0) ? band_in : band_out;
    const float hi = (side > 0) ? band_out : band_in;
    int32_t i;
    int32_t level = 0;

    for (i = first; (i >= 0) && (i < n); i += side)
    {
        int32_t j;

        if ((0U == p_s[i].mm) || (p_s[i].mm > S5_CORRIDOR_RANGE_MM) ||
            s5_own_far_corner(&p_s[i], side, beam, p_view, y_own, x_face))
        {
            continue;
        }
        for (j = -2; j <= 2; j++)
        {
            int32_t a = (int32_t)p_s[i].ddeg + ((j * (int32_t)beam) / 2);
            float th;
            float px;
            float py;

            if (a > S5_BEARING_MAX_DDEG)
            {
                a = S5_BEARING_MAX_DDEG;
            }
            if (a < -S5_BEARING_MAX_DDEG)
            {
                a = -S5_BEARING_MAX_DDEG;
            }
            th = p_view->psi + s5_ddeg_to_rad(a);
            px = p_view->x + ((float)p_s[i].mm * s5_cosf(th));
            py = p_view->y + ((float)p_s[i].mm * s5_sinf(th));
            if ((py >= lo) && (py <= hi) && (px >= p_view->x) &&
                (px <= (x_face + S5_CORRIDOR_AHEAD_MM)))
            {
                if (0 == j)
                {
                    *p_at = i;
                    return 2;
                }
                if (0 == level)
                {
                    *p_at = i;
                    level = 1;
                }
            }
        }
    }
    return level;
}

/* the next object echo inwards (towards the face sample i0) from i that
 * is at least S5_EDGE_PAIR_DDEG away (face_mm 0: the very next one) */
static int32_t s5_edge_inner(const s5_sample_t *p_s, int32_t i, int32_t dir,
                             int32_t i0, uint16_t face_mm)
{
    int32_t in = i - dir;

    while (in != i0)
    {
        int32_t da = (int32_t)p_s[i].ddeg - (int32_t)p_s[in].ddeg;

        da = (da < 0) ? -da : da;
        if ((0U == face_mm) ||
            (s5_scan_is_object(p_s[in].mm, face_mm) &&
             (da >= S5_EDGE_PAIR_DDEG)))
        {
            break;
        }
        in -= dir; /* a dropout, or too close to tell */
    }
    return in;
}

/* two echoes with no jump between them, for the angle between them */
static bool s5_edge_agree(const s5_sample_t *p_a, const s5_sample_t *p_b)
{
    int32_t da = (int32_t)p_a->ddeg - (int32_t)p_b->ddeg;
    float limit;
    float rise;

    da = (da < 0) ? -da : da;
    limit = (float)S5_EDGE_JUMP_MM +
            (S5_EDGE_STEP_MM_PER_DEG * 0.1f * (float)da);
    rise = (float)p_a->mm - (float)p_b->mm;
    return (rise <= limit) && (rise >= -limit);
}

/* Buddy 5, 2026-11-16: the range method (s5_scan.h). FACE = lower
 * quartile of the object's ranges, EDGE on each side = mean of the two
 * outermost neighbouring ranges that agree. Returns false
 * if there is nothing to work with. */
static bool s5_range_edges(const s5_sample_t *p_s, uint32_t n,
                           uint16_t face_mm, uint16_t *p_face_q,
                           uint16_t *p_edge_l, uint16_t *p_edge_r)
{
    const uint32_t i0 = s5_face_index(p_s, n, face_mm);
    uint16_t v[S5_SCAN_MAX];
    uint32_t m = 0U;
    int32_t clr;
    int32_t i_l;
    int32_t i_r;
    int32_t dir;
    int32_t i;

    if ((i0 >= n) || (n > S5_SCAN_MAX))
    {
        return false;
    }
    i_l = s5_walk(p_s, (int32_t)n, (int32_t)i0, 1, face_mm, &clr);
    i_r = s5_walk(p_s, (int32_t)n, (int32_t)i0, -1, face_mm, &clr);

    /* FACE: lower quartile of the run (insertion sort, at most 40) */
    for (i = i_r; i <= i_l; i++)
    {
        if (s5_scan_is_object(p_s[i].mm, face_mm))
        {
            uint32_t j = m;

            while ((j > 0U) && (v[j - 1U] > p_s[i].mm))
            {
                v[j] = v[j - 1U];
                j--;
            }
            v[j] = p_s[i].mm;
            m++;
        }
    }
    if (0U == m)
    {
        return false;
    }
    /* ... then the mean of the echoes within S5_EDGE_JUMP_MM of it (the
     * flat of the face): a single short reading would put every corner
     * further out than it is */
    {
        const uint32_t q = v[(m - 1U) / 4U];
        uint32_t sum = 0U;
        uint32_t cnt = 0U;
        uint32_t j;

        for (j = 0U; j < m; j++)
        {
            if ((v[j] + (uint32_t)S5_EDGE_JUMP_MM >= q) &&
                (v[j] <= q + (uint32_t)S5_EDGE_JUMP_MM))
            {
                sum += v[j];
                cnt++;
            }
        }
        *p_face_q = (uint16_t)((sum + (cnt / 2U)) / cnt);
    }

    /* EDGE: from the outermost echo inwards, the first echo that agrees
     * (no jump) with the echo next to it and with the next one at least
     * S5_EDGE_PAIR_DDEG further in. Echoes that jump are a mix with whatever is behind the object,
     * heard as the beam leaves the corner: left out. The partner has to
     * be a few degrees away because the two extra pings of a halving
     * are 1..3 deg apart and always agree with each other (real sweeps:
     * 283 at 18 deg, then 369 and 369 at 24 and 25). */
    for (dir = 1; dir >= -1; dir -= 2)
    {
        int32_t out = (dir > 0) ? i_l : i_r;
        uint32_t edge = p_s[i0].mm;

        while (out != (int32_t)i0)
        {
            const int32_t in = s5_edge_inner(p_s, out, dir, (int32_t)i0,
                                             face_mm);

            if (s5_edge_agree(&p_s[out], &p_s[in]) &&
                s5_edge_agree(&p_s[out],
                              &p_s[s5_edge_inner(p_s, out, dir, (int32_t)i0,
                                                 0U)]))
            {
                /* the two furthest of the echoes kept, averaged: the
                 * range grows towards a corner (a shorter last echo is
                 * noise), and the single furthest reads long by the
                 * noise */
                uint32_t top = edge;
                uint32_t next = edge;
                int32_t k;
#if (0 != S5_EDGE_KNEE)
                /* Buddy 5, 2026-11-20: a box's echoes go face, then a
                 * step to its corner, then flat. Ranges that go on
                 * growing over two steps in a row past S5_EDGE_GROW_MM +
                 * S5_EDGE_GROW_PER_DEG per degree are something sloping
                 * away beside it (people round the track: 171 181 197 213
                 * mm at -20..-45 deg for a 13 cm box from 167): the corner
                 * is where that growth starts */
                {
                    int32_t prev = -1;
                    int32_t prev2 = -1;
                    bool b_grew = false;

                    for (k = (int32_t)i0; k != (out + dir); k += dir)
                    {
                        if (!s5_scan_is_object(p_s[k].mm, face_mm))
                        {
                            continue;
                        }
                        if (prev >= 0)
                        {
                            int32_t da = (int32_t)p_s[k].ddeg -
                                         (int32_t)p_s[prev].ddeg;
                            const int32_t dr = (int32_t)p_s[k].mm -
                                               (int32_t)p_s[prev].mm;
                            bool b_grow;

                            da = (da < 0) ? -da : da;
                            b_grow = (dr > ((int32_t)S5_EDGE_GROW_MM +
                                            (((int32_t)S5_EDGE_GROW_PER_DDEG *
                                              da) /
                                             100)));
                            if (b_grow && b_grew && (prev2 >= 0))
                            {
                                out = prev2;
                                break;
                            }
                            b_grew = b_grow;
                        }
                        prev2 = prev;
                        prev = k;
                    }
                }
#endif

                for (k = (int32_t)i0; k != (out + dir); k += dir)
                {
                    if (!s5_scan_is_object(p_s[k].mm, face_mm))
                    {
                        continue;
                    }
                    if (p_s[k].mm > top)
                    {
                        next = top;
                        top = p_s[k].mm;
                    }
                    else if (p_s[k].mm > next)
                    {
                        next = p_s[k].mm;
                    }
                    else
                    {
                        /* shorter than both */
                    }
                }
                edge = (top + next + 1U) / 2U;
                break;
            }
            out = s5_edge_inner(p_s, out, dir, (int32_t)i0, 0U);
        }
        if (dir > 0)
        {
            *p_edge_l = (uint16_t)edge;
        }
        else
        {
            *p_edge_r = (uint16_t)edge;
        }
    }
    return true;
}

/* the object is straight ahead: the ping nearest 0 deg is one of its
 * echoes */
static bool s5_range_ahead(const s5_sample_t *p_s, uint32_t n,
                           uint16_t face_mm)
{
    uint32_t best = 0U;
    int32_t best_a = 0x7FFFFFFF;
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        int32_t a = (int32_t)p_s[i].ddeg;

        a = (a < 0) ? -a : a;
        if (a < best_a)
        {
            best_a = a;
            best = i;
        }
    }
    return (n > 0U) && s5_scan_is_object(p_s[best].mm, face_mm);
}

/* sideways distance of a corner EDGE away on a face FACE away */
static float s5_range_side(uint16_t edge_mm, uint16_t face_mm)
{
    const float e = (float)edge_mm;
    const float f = (float)face_mm;

    float side = (e > f) ? s5_sqrtf((e * e) - (f * f)) : 0.0f;

    /* 2..3 mm of noise in EDGE - FACE moves a near corner a lot (15 mm at
     * 65 mm out, 7 mm at 150): that much is added, so that the noise does
     * not put the corner inside the real one (real sweeps of the 210 mm
     * box read 196 mm without it) */
    if (side >= (float)S5_EDGE_MIN_MM)
    {
        const float pad = (float)S5_EDGE_PAD_MM2 / side;

        side += (pad < (float)S5_EDGE_PAD_MAX_MM) ? pad
                                                  : (float)S5_EDGE_PAD_MAX_MM;
    }
    return side;
}

void s5_scan_profile(const s5_sample_t *p_s, uint32_t n,
                     const s5_view_t *p_view, s5_profile_t *p_out)
{
    int16_t l_obj;
    int16_t l_clr;
    int16_t r_obj;
    int16_t r_clr;
    bool b_l;
    bool b_r;
    int32_t l_obs;
    int32_t r_obs;
    int32_t l_true;
    int32_t r_true;
    int32_t beam;
    float face;

    p_out->b_found = false;
    p_out->b_left_blocked = false;
    p_out->b_right_blocked = false;
    p_out->b_left_maybe = false;
    p_out->b_right_maybe = false;
    p_out->b_left_hidden = false;
    p_out->b_right_hidden = false;
    p_out->block_l_ddeg = 0;
    p_out->block_l_mm = 0U;
    p_out->block_r_ddeg = 0;
    p_out->block_r_mm = 0U;
    p_out->face_mm = s5_scan_face(p_s, n);
    if (0U == p_out->face_mm)
    {
        return;
    }
    p_out->b_found = true;
    face = (float)p_out->face_mm;

    s5_scan_edges(p_s, n, p_out->face_mm, &l_obj, &l_clr, &b_l, &r_obj, &r_clr,
                  &b_r);
    p_out->b_left_seen = b_l;
    p_out->b_right_seen = b_r;
    beam = (int32_t)s5_scan_beam_ddeg(p_out->face_mm);
    p_out->beam_ddeg = (int16_t)beam;

    /* where the echoes stop: halfway to the first angle that was clear
     * (an edge outside the sweep: at the sweep's end, a lower bound) */
    l_obs = b_l ? (((int32_t)l_obj + (int32_t)l_clr) / 2) : (int32_t)l_obj;
    r_obs = b_r ? (((int32_t)r_obj + (int32_t)r_clr) / 2) : (int32_t)r_obj;
    p_out->left_ddeg = (int16_t)l_obs;
    p_out->right_ddeg = (int16_t)r_obs;

    /* the corners are a beam half-angle inside that */
    l_true = l_obs - beam;
    r_true = r_obs + beam;
    if (l_true < r_true)
    {
        /* narrower than the beam can tell apart: a thin post */
        l_true = (l_obs + r_obs) / 2;
        r_true = l_true;
    }

    {
        /* the face's nearest point: straight in front of the sonar while
         * the sonar is between the corners; the near corner when it is
         * out beside the object (after going round another one) */
        const float brg_l = p_view->psi + s5_ddeg_to_rad(l_true);
        const float brg_r = p_view->psi + s5_ddeg_to_rad(r_true);
        float dx = face;

        if (brg_l < 0.0f)
        {
            dx = face * s5_cosf(brg_l);
        }
        else if (brg_r > 0.0f)
        {
            dx = face * s5_cosf(brg_r);
        }
        else
        {
            /* in between */
        }
        p_out->x_face = p_view->x + dx;
        p_out->y_left = p_view->y + (dx * s5_tanf(brg_l));
        p_out->y_right = p_view->y + (dx * s5_tanf(brg_r));
        p_out->ya_left = p_out->y_left;
        p_out->ya_right = p_out->y_right;
        p_out->b_by_range = false;
        p_out->face_q_mm = p_out->face_mm;
        p_out->edge_l_mm = 0U;
        p_out->edge_r_mm = 0U;
        p_out->b_narrow = false;
        p_out->y_mid = 0.0f;
        p_out->b_limited = false;
#if (0 != S5_EDGE_BY_RANGE)
        {
            /* Buddy 5, 2026-11-16: the corners by range (the face is
             * square to the line, so FACE is measured along the line
             * whatever the car's heading) */
            uint16_t fq = 0U;
            uint16_t el = 0U;
            uint16_t er = 0U;

            /* only from the line, looking along it (the first look at an
             * object), with the object straight ahead. From out beside
             * one, after going round another, the nearest echo is its
             * corner, not its face: by angle, as before. */
            if ((s5_absf(p_view->y) <= (float)S5_EDGE_ON_LINE_MM) &&
                (s5_absf(p_view->psi) <=
                 ((float)S5_EDGE_ON_LINE_DEG * S5_DEG2RAD_F)) &&
                s5_range_ahead(p_s, n, p_out->face_mm) &&
                s5_range_edges(p_s, n, p_out->face_mm, &fq, &el, &er))
            {
                float side_l = s5_range_side(el, fq);
                float side_r = s5_range_side(er, fq);
                const float raw_l = side_l; /* before any fallback */
                const float raw_r = side_r;

                p_out->b_by_range = true;
                p_out->face_q_mm = fq;
                p_out->edge_l_mm = el;
                p_out->edge_r_mm = er;
                p_out->x_face = p_view->x + (float)fq;

                /* the angle method's sides, trimmed as they are when
                 * used on their own */
                const float ang_l = (p_out->ya_left - p_view->y) -
                                    (0.5f * (float)S5_WIDTH_TRIM_MM);
                const float ang_r = (p_view->y - p_out->ya_right) -
                                    (0.5f * (float)S5_WIDTH_TRIM_MM);
                /* echoes all the way to the end of the sweep on a side: a
                 * wall or something else is next to the object, and may
                 * hide more on the other side too. Then no side is taken
                 * closer in than by angle. */
                const bool b_alone = p_out->b_left_seen &&
                                     p_out->b_right_seen;
                /* the least a side that range cannot tell is taken to be */
                /* Buddy 5, 2026-11-19: half the least object (S5_OBJ_MIN_MM),
                 * was half of S5_EDGE_MIN_MM (22 mm) */
                const float near = (float)S5_NARROW_HALF_MIN_MM;

                /* a corner closer to the sonar's axis than S5_EDGE_MIN_MM
                 * cannot be told from the face by range (it may even be
                 * on the other side of the axis), and neither can the near
                 * corner of a box that stands turned (its range is the
                 * shortest one): where the angle method says, as before,
                 * but at least half of S5_EDGE_MIN_MM out (so never more
                 * than that half too narrow on that side) */
                if (side_l < (float)S5_EDGE_MIN_MM)
                {
                    side_l = (ang_l > near) ? ang_l : near;
                }
                else if (!b_alone && (side_l < ang_l))
                {
                    side_l = ang_l;
                }
                else
                {
                    /* by range */
                }
                if (side_r < (float)S5_EDGE_MIN_MM)
                {
                    side_r = (ang_r > near) ? ang_r : near;
                }
                else if (!b_alone && (side_r < ang_r))
                {
                    side_r = ang_r;
                }
                else
                {
                    /* by range */
                }
                p_out->y_left = p_view->y + side_l;
                p_out->y_right = p_view->y - side_r;

                /* Buddy 5, 2026-11-19: a narrow object (a 9 cm one), both
                 * corners too near the sonar's axis to tell by range. Its
                 * echoes spread out by the beam the same on both sides,
                 * so the middle of where they start and stop is where the
                 * object is, whatever the beam (only the servo's trim
                 * counts, so S5_AUTO_TRIM). Half its width: the two sides
                 * by range, even if short, kept within
                 * S5_NARROW_HALF_MIN_MM .. S5_EDGE_MIN_MM. Not when one
                 * side is told by range (a wider object off centre): a
                 * servo error would move the middle and cut that object
                 * short on the other side. Before, the unresolved side came from
                 * the angle method (beam and servo both off on the car)
                 * and a 9 cm object 2..4 cm off the line went round the
                 * long way. */
                if ((0 != S5_NARROW_ON) && b_alone &&
                    (raw_l < (float)S5_EDGE_MIN_MM) &&
                    (raw_r < (float)S5_EDGE_MIN_MM))
                {
                    const float mid =
                        p_view->psi +
                        s5_ddeg_to_rad((l_obs + r_obs) / 2);
                    const float yc = p_view->y + ((float)fq * s5_tanf(mid));
                    float hw = 0.5f * (raw_l + raw_r);

                    if (hw > (float)S5_EDGE_MIN_MM)
                    {
                        hw = (float)S5_EDGE_MIN_MM;
                    }
                    if (hw < (float)S5_NARROW_HALF_MIN_MM)
                    {
                        hw = (float)S5_NARROW_HALF_MIN_MM;
                    }
                    p_out->b_narrow = true;
                    p_out->y_mid = yc;
                    if (raw_l < (float)S5_EDGE_MIN_MM)
                    {
                        p_out->y_left = yc + hw;
                    }
                    if (raw_r < (float)S5_EDGE_MIN_MM)
                    {
                        p_out->y_right = yc - hw;
                    }
                }
                /* Buddy 5, 2026-11-19: S5_OBJ_MIN_MM .. S5_OBJ_MAX_MM
                 * around the middle of the echoes */
                if (b_alone)
                {
                    const float mid =
                        p_view->psi +
                        s5_ddeg_to_rad((l_obs + r_obs) / 2);
                    const float ym = p_view->y + ((float)fq * s5_tanf(mid));
                    const float half_min = 0.5f * (float)S5_OBJ_MIN_MM;
                    const float half_max = 0.5f * (float)S5_OBJ_MAX_MM;
                    const float w = p_out->y_left - p_out->y_right;

                    if (w < (float)S5_OBJ_MIN_MM)
                    {
                        if (p_out->y_left < (ym + half_min))
                        {
                            p_out->y_left = ym + half_min;
                        }
                        if (p_out->y_right > (ym - half_min))
                        {
                            p_out->y_right = ym - half_min;
                        }
                        p_out->b_limited = true;
                    }
                    else if (w > (float)S5_OBJ_MAX_MM)
                    {
                        if (p_out->y_left > (ym + half_max))
                        {
                            p_out->y_left = ym + half_max;
                        }
                        if (p_out->y_right < (ym - half_max))
                        {
                            p_out->y_right = ym - half_max;
                        }
                        p_out->b_limited = true;
                    }
                    else
                    {
                        /* within the limits */
                    }
                }
            }
        }
#endif
    }
    if (!p_out->b_by_range)
    {
        /* each edge in by half of S5_WIDTH_TRIM_MM, never past the
         * middle */
        const float half_trim = 0.5f * (float)S5_WIDTH_TRIM_MM;
        const float mid = 0.5f * (p_out->y_left + p_out->y_right);

        p_out->y_left -= half_trim;
        p_out->y_right += half_trim;
        if (p_out->y_left < mid)
        {
            p_out->y_left = mid;
            p_out->y_right = mid;
        }
    }
    p_out->width = p_out->y_left - p_out->y_right;

    /* room on each side: the samples past each edge */
    {
        uint32_t i;
        int32_t first_l = (int32_t)n;
        int32_t first_r = -1;
        int32_t at_l = -1;
        int32_t at_r = -1;
        int32_t lev_l;
        int32_t lev_r;
        float y_own_l;
        float y_own_r;

        for (i = 0U; i < n; i++)
        {
            if ((int32_t)p_s[i].ddeg > (int32_t)l_obj)
            {
                first_l = (int32_t)i;
                break;
            }
        }
        for (i = n; i > 0U; i--)
        {
            if ((int32_t)p_s[i - 1U].ddeg < (int32_t)r_obj)
            {
                first_r = (int32_t)i - 1;
                break;
            }
        }
        p_out->b_left_hidden =
            (first_l < (int32_t)n) && (S5_MM_UNKNOWN == p_s[first_l].mm);
        p_out->b_right_hidden =
            (first_r >= 0) && (S5_MM_UNKNOWN == p_s[first_r].mm);
        /* Buddy 5, 2026-11-16: a side that range could not tell
         * (S5_EDGE_MIN_MM) may end on the far side of the sonar, where
         * the angle method puts it: the object's own far corner is looked
         * for from there */
        y_own_l = (p_out->b_by_range && (p_out->ya_left < p_out->y_left) &&
                   ((p_out->y_left - p_view->y) < (float)S5_EDGE_MIN_MM))
                      ? p_out->ya_left
                      : p_out->y_left;
        y_own_r = (p_out->b_by_range && (p_out->ya_right > p_out->y_right) &&
                   ((p_view->y - p_out->y_right) < (float)S5_EDGE_MIN_MM))
                      ? p_out->ya_right
                      : p_out->y_right;
        lev_l = s5_side_blocked(p_s, (int32_t)n, first_l, 1, (int16_t)beam,
                                p_view, p_out->y_left, y_own_l, p_out->x_face,
                                &at_l);
        lev_r = s5_side_blocked(p_s, (int32_t)n, first_r, -1, (int16_t)beam,
                                p_view, p_out->y_right, y_own_r, p_out->x_face,
                                &at_r);
#if (0 != S5_SIDE_BLOCK)
        p_out->b_left_blocked = (2 == lev_l);
        p_out->b_right_blocked = (2 == lev_r);
        p_out->b_left_maybe = (1 == lev_l);
        p_out->b_right_maybe = (1 == lev_r);
#else
        /* Buddy 5, 2026-11-19: S5_SIDE_BLOCK 0, nothing beside the object
         * blocks a side (on the track the echoes 30..37 cm away were not
         * objects, and the car backed up and measured worse) */
        (void)lev_l;
        (void)lev_r;
#endif
        p_out->block_l_ddeg = (at_l >= 0) ? p_s[at_l].ddeg : 0;
        p_out->block_l_mm = (at_l >= 0) ? p_s[at_l].mm : 0U;
        p_out->block_r_ddeg = (at_r >= 0) ? p_s[at_r].ddeg : 0;
        p_out->block_r_mm = (at_r >= 0) ? p_s[at_r].mm : 0U;
    }
}

/* Buddy 5, 2026-11-19: S5_AUTO_TRIM. A sweep taken with the car standing
 * still and an object straight ahead: true and *p_corr (0.1 deg, to add to
 * the trim) if it can be trusted for that: found, both edges inside the
 * sweep, the face S5_AUTO_TRIM_NEAR_MM .. S5_AUTO_TRIM_FAR_MM away, the
 * object S5_OBJ_MIN_MM .. S5_OBJ_MAX_MM wide by angle, the correction
 * under S5_AUTO_TRIM_MAX_DDEG. */
bool s5_scan_trim_from(const s5_profile_t *p_pr, int16_t *p_corr)
{
    int32_t c;
    float w;

    if (!p_pr->b_found || !p_pr->b_left_seen || !p_pr->b_right_seen ||
        (p_pr->face_mm < (uint16_t)S5_AUTO_TRIM_NEAR_MM) ||
        (p_pr->face_mm > (uint16_t)S5_AUTO_TRIM_FAR_MM))
    {
        return false;
    }
    c = ((int32_t)p_pr->left_ddeg + (int32_t)p_pr->right_ddeg) / 2;
    if ((c > S5_AUTO_TRIM_MAX_DDEG) || (c < -S5_AUTO_TRIM_MAX_DDEG))
    {
        return false;
    }
    w = p_pr->ya_left - p_pr->ya_right;
    if ((w < (0.5f * (float)S5_OBJ_MIN_MM)) ||
        (w > (2.0f * (float)S5_OBJ_MAX_MM)))
    {
        return false; /* a wall, or nothing much: not an object to trim by */
    }
    *p_corr = (int16_t)c;
    return true;
}

/* Buddy 5, 2026-11-19: S5_TWO_DIST. The angle a face of width w subtends,
 * its middle yc to the side, from d away. */
static float s5_two_span(float d, float yc, float w)
{
    return s5_atanf((yc + (0.5f * w)) / d) - s5_atanf((yc - (0.5f * w)) / d);
}

/* Width from two sweeps of the same object, d1 and d2 from its face (d1
 * the nearer), the echoes spanning span1 and span2 (rad). The echoes go on
 * past each corner by the same beam angle at both, so the beam drops out of
 * span1 - span2 and only w is left. True: *p_w (mm) and *p_beam (rad, half,
 * per side) if they make sense (the beam S5_TWO_BEAM_MIN_DEG ..
 * S5_TWO_BEAM_MAX_DEG, the width up to 2 sqrt(d1 d2), past which the
 * difference no longer grows with it). */
bool s5_scan_two_width(float d1, float span1, float d2, float span2,
                       float yc, float *p_w, float *p_beam)
{
    const float want = span1 - span2;
    float lo = 0.0f;
    float hi;
    float beam;
    uint32_t i;

    if ((d1 < 50.0f) || ((d2 - d1) < (float)S5_TWO_MIN_APART_MM))
    {
        return false;
    }
    hi = 2.0f * s5_sqrtf(d1 * d2);
    if (hi > 600.0f)
    {
        hi = 600.0f;
    }
    if ((want <= 0.0f) ||
        (want >= (s5_two_span(d1, yc, hi) - s5_two_span(d2, yc, hi))))
    {
        return false; /* no width gives that difference */
    }
    for (i = 0U; i < 30U; i++)
    {
        const float mid = 0.5f * (lo + hi);

        if ((s5_two_span(d1, yc, mid) - s5_two_span(d2, yc, mid)) < want)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    beam = 0.5f * (span1 - s5_two_span(d1, yc, lo));
    if ((beam < ((float)S5_TWO_BEAM_MIN_DEG * S5_DEG2RAD_F)) ||
        (beam > ((float)S5_TWO_BEAM_MAX_DEG * S5_DEG2RAD_F)))
    {
        return false;
    }
    *p_w = lo;
    *p_beam = beam;
    return true;
}
