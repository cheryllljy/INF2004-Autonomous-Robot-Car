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

bool s5_scan_is_object(uint16_t mm, uint16_t face_mm)
{
    return (0U != mm) && (S5_MM_UNKNOWN != mm) &&
           ((uint32_t)mm <= ((uint32_t)face_mm + (uint32_t)S5_EDGE_DEPTH_MM));
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
                               float x_face, int32_t *p_at)
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
            s5_own_far_corner(&p_s[i], side, beam, p_view, y_edge, x_face))
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
        lev_l = s5_side_blocked(p_s, (int32_t)n, first_l, 1, (int16_t)beam,
                                p_view, p_out->y_left, p_out->x_face, &at_l);
        lev_r = s5_side_blocked(p_s, (int32_t)n, first_r, -1, (int16_t)beam,
                                p_view, p_out->y_right, p_out->x_face, &at_r);
        p_out->b_left_blocked = (2 == lev_l);
        p_out->b_right_blocked = (2 == lev_r);
        p_out->b_left_maybe = (1 == lev_l);
        p_out->b_right_maybe = (1 == lev_r);
        p_out->block_l_ddeg = (at_l >= 0) ? p_s[at_l].ddeg : 0;
        p_out->block_l_mm = (at_l >= 0) ? p_s[at_l].mm : 0U;
        p_out->block_r_ddeg = (at_r >= 0) ? p_s[at_r].ddeg : 0;
        p_out->block_r_mm = (at_r >= 0) ? p_s[at_r].mm : 0U;
    }
}
