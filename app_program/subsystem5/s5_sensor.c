/*
 *      s5_sensor.c
 *      Subsystem 5 -- servo, pings and the sweep, run by the S5 task. See
 *      s5_sensor.h.
 */

#include "s5_sensor.h"

#include "s5_config.h"
#include "s5_hw.h"

/* ---------------------------------------------------------------------
 *  The request word (written by the VehicleController's task only):
 *      bits 30..31  mode
 *      bits 18..29  angle + 2048, 0.1 deg (LOOK)
 *      bits  0..17  sweep number (SCAN)
 * --------------------------------------------------------------------- */
#define S5_MODE_GUARD     (0U)
#define S5_MODE_LOOK      (1U)
#define S5_MODE_SCAN      (2U)
#define S5_REQ_MODE_SHIFT (30U)
#define S5_REQ_ANG_SHIFT  (18U)
#define S5_REQ_ANG_MASK   (0xFFFU)
#define S5_REQ_ANG_BIAS   (2048)
#define S5_REQ_SEQ_MASK   (0x3FFFFU)

#define S5_US_PER_MS (1000U)
/* coarse sweep: +LIMIT .. -LIMIT in STEP */
#define S5_COARSE_N \
    ((uint32_t)((2 * S5_SERVO_LIMIT_DDEG) / S5_SCAN_STEP_DDEG) + 1U)
/* ECHO high at every try to fire for this long: stuck, not answering */
#define S5_STUCK_US (500U * S5_US_PER_MS)
/* the servo's first move after power-up can be the whole way */
#define S5_BOOT_SETTLE_US (400U * S5_US_PER_MS)
/* LOOK: a new angle this close to where the servo is changes nothing */
#define S5_LOOK_DEADBAND_DDEG (15)
/* LOOK: an angle that creeps away by up to this moves the servo once per
 * ping: after the echo is in, it follows, and then it waits for the next
 * ping, which goes out where the servo is (each ping carries its angle).
 * Following every step would restart the settle each time and no ping
 * would ever go out. */
#define S5_LOOK_HOLD_DDEG (50)

_Static_assert(((S5_COARSE_N + (2U * (S5_SCAN_FINE + 1U))) <= S5_SCAN_MAX),
               "sweep does not fit S5_SCAN_MAX samples");

typedef enum
{
    S5_SW_NONE = 0, /* no sweep running */
    S5_SW_COARSE,
    S5_SW_FINE
} s5_sw_phase_t;

/* ---- shared with the VehicleController's task ------------------------ */
static volatile uint32_t g_req;       /* written by the VC's task */
static uint32_t g_scan_next;          /* the VC's task only */
static volatile uint32_t g_ping_lock; /* odd while the ping is written */
static s5_ping_t g_ping_pub;
static volatile uint32_t g_scan_done; /* last sweep published */
static s5_sweep_t g_sweep_pub;
static volatile bool g_b_dead;

/* ---- the S5 task's own --------------------------------------------- */
typedef struct
{
    int32_t servo;      /* angle last sent to the servo */
    uint32_t move_us;   /* when */
    uint32_t settle_us; /* how long it needs from then */
    uint32_t ring_us;   /* ... of which ringing down at the end */
    bool b_moved;       /* moved since the last ping went out */
    bool b_in_flight;   /* a ping fired, not read yet */
    uint32_t fire_us;
    int16_t fire_ddeg;
    uint32_t fire_sweep; /* the sweep it was fired for, 0 = none */
    bool b_fired;        /* any ping fired yet */
    uint32_t last_fire_us;
    uint32_t dead_run;
    bool b_refused;
    uint32_t refused_us;
    uint32_t close_run;
    uint32_t seq;
} s5_drv_t;

typedef struct
{
    uint32_t seq; /* sweep being done */
    s5_sw_phase_t phase;
    uint32_t idx; /* coarse step */
    uint32_t n;
    s5_sample_t s[S5_SCAN_MAX];
    uint16_t face_mm;
    uint32_t n_br;  /* edges to refine (right first, then left) */
    uint32_t br;    /* the one being refined */
    uint32_t level; /* halvings done on it */
    int16_t obj[2]; /* angle that still answered */
    int16_t clr[2]; /* angle that did not */
    int32_t probe;
    bool b_via;   /* going to probe + overshoot first */
    bool b_check; /* probe = the coarse sweep's first clear angle */
    bool b_again; /* the probe heard nothing once: ping it again */
} s5_sw_t;

static s5_drv_t g_d;
static s5_sw_t g_w;

/* ===================================================================== */
/*  VehicleController's side                                             */
/* ===================================================================== */

static void s5_request(uint32_t mode, int32_t ddeg, uint32_t seq)
{
    int32_t a = ddeg;

    if (a > (S5_REQ_ANG_BIAS - 1))
    {
        a = S5_REQ_ANG_BIAS - 1;
    }
    if (a < -S5_REQ_ANG_BIAS)
    {
        a = -S5_REQ_ANG_BIAS;
    }
    g_req = (mode << S5_REQ_MODE_SHIFT) |
            (((uint32_t)(a + S5_REQ_ANG_BIAS) & S5_REQ_ANG_MASK)
             << S5_REQ_ANG_SHIFT) |
            (seq & S5_REQ_SEQ_MASK);
}

void s5_sensor_guard(void)
{
    s5_request(S5_MODE_GUARD, 0, 0U);
}

void s5_sensor_look(int32_t ddeg)
{
    s5_request(S5_MODE_LOOK, ddeg, 0U);
}

uint32_t s5_sensor_scan(void)
{
    g_scan_next = (g_scan_next + 1U) & S5_REQ_SEQ_MASK;
    if (0U == g_scan_next)
    {
        g_scan_next = 1U;
    }
    s5_request(S5_MODE_SCAN, 0, g_scan_next);
    return g_scan_next;
}

bool s5_sensor_scan_get(uint32_t seq, s5_sweep_t *p_out)
{
    if (g_scan_done != seq)
    {
        return false;
    }
    __sync_synchronize(); /* the samples were written before the number */
    *p_out = g_sweep_pub;
    return true;
}

bool s5_sensor_ping(s5_ping_t *p_out)
{
    uint32_t tries;

    for (tries = 0U; tries < 4U; tries++)
    {
        const uint32_t before = g_ping_lock;

        __sync_synchronize();
        *p_out = g_ping_pub;
        __sync_synchronize();
        if ((0U == (before & 1U)) && (before == g_ping_lock))
        {
            return 0U != p_out->seq;
        }
    }
    return false; /* kept colliding with the writer: try next pass */
}

bool s5_sensor_dead(void)
{
    return g_b_dead;
}

/* ===================================================================== */
/*  S5 task's side                                                        */
/* ===================================================================== */

void s5_sensor_init(void)
{
    const uint32_t now = s5_hw_us();

    g_d = (s5_drv_t){ 0 };
    g_w = (s5_sw_t){ 0 };
    g_d.servo = 0;
    g_d.move_us = now;
    g_d.settle_us = S5_BOOT_SETTLE_US;
    g_d.ring_us = (uint32_t)S5_SERVO_SETTLE_MS * S5_US_PER_MS;
    g_req = 0U; /* guard */
    g_scan_next = 0U;
    g_scan_done = 0U;
    g_ping_lock = 0U;
    g_ping_pub = (s5_ping_t){ 0 }; /* seq 0: no ping yet */
    g_b_dead = false;
    s5_hw_servo(0);
}

static void s5_publish(const s5_ping_t *p_ping)
{
    g_ping_lock = g_ping_lock + 1U; /* odd: being written */
    __sync_synchronize();
    g_ping_pub = *p_ping;
    __sync_synchronize();
    g_ping_lock = g_ping_lock + 1U; /* even: complete */
}

/* Echo time to mm. Sound: 331.3 + 0.606 T m/s; there and back. */
static uint16_t s5_us_to_mm(uint32_t us)
{
    const uint32_t c =
        3313U + ((606U * (uint32_t)S5_AIR_TEMP_C) / 100U); /* 0.1 m/s */
    int32_t mm = (int32_t)((us * c) / 20000U) + S5_RANGE_OFFSET_MM;

    if (mm > S5_RANGE_MAX_MM)
    {
        return 0U; /* nothing that matters */
    }
    if (mm < S5_RANGE_MIN_MM)
    {
        mm = S5_RANGE_MIN_MM;
    }
    return (uint16_t)mm;
}

static bool s5_settled(uint32_t now)
{
    return (now - g_d.move_us) >= g_d.settle_us;
}

static void s5_move(int32_t ddeg, uint32_t now)
{
    int32_t d = ddeg - g_d.servo;
    uint32_t left_us = 0U;

    if (d < 0)
    {
        d = -d;
    }
    /* an unfinished move: its travel still has to be done (its ring-down
     * is replaced by this move's) */
    if (!s5_settled(now))
    {
        const uint32_t left = g_d.settle_us - (now - g_d.move_us);

        left_us = (left > g_d.ring_us) ? (left - g_d.ring_us) : 0U;
    }
    s5_hw_servo(ddeg);
    g_d.servo = ddeg;
    g_d.move_us = now;
    g_d.ring_us = (uint32_t)S5_SERVO_SETTLE_MS * S5_US_PER_MS;
    g_d.b_moved = true;
    g_d.settle_us = left_us +
                    ((((uint32_t)d * (uint32_t)S5_SERVO_MS_PER_60DEG) / 600U) *
                     S5_US_PER_MS) +
                    g_d.ring_us;
}

/* ---- sweep ----------------------------------------------------------- */

static int32_t s5_coarse_angle(uint32_t idx)
{
    return S5_SERVO_LIMIT_DDEG - ((int32_t)idx * S5_SCAN_STEP_DDEG);
}

static void s5_sweep_start(uint32_t seq)
{
    g_w.seq = seq;
    g_w.phase = S5_SW_COARSE;
    g_w.idx = 0U;
    g_w.n = 0U;
    g_w.n_br = 0U;
    g_w.br = 0U;
    g_w.level = 0U;
}

static void s5_sweep_finish(void)
{
    s5_scan_sort(g_w.s, g_w.n);
    g_sweep_pub.seq = g_w.seq;
    g_sweep_pub.n = g_w.n;
    {
        uint32_t i;

        for (i = 0U; i < g_w.n; i++)
        {
            g_sweep_pub.s[i] = g_w.s[i];
        }
    }
    __sync_synchronize(); /* samples first, then the number */
    g_scan_done = g_w.seq;
    g_w.phase = S5_SW_NONE;
}

static void s5_next_probe(void)
{
    if (g_w.b_check)
    {
        /* first ping the coarse sweep's first clear angle again: if that
         * was a dropout, the edge is further out */
        g_w.probe = g_w.clr[g_w.br];
    }
    else
    {
        g_w.probe = ((int32_t)g_w.obj[g_w.br] + (int32_t)g_w.clr[g_w.br]) / 2;
    }
    /* come at it from the left, as the coarse sweep did */
    g_w.b_via = (g_d.servo < g_w.probe);
}

/* The next bracket (or the end of the sweep). */
static void s5_next_bracket(void)
{
    g_w.level = 0U;
    g_w.b_check = true;
    g_w.b_again = false;
    g_w.br++;
    if (g_w.br >= g_w.n_br)
    {
        s5_sweep_finish();
        return;
    }
    s5_next_probe();
}

/* One sample per angle: a ping again at an angle replaces the one that
 * heard nothing there (an answer wins over a dropout). Returns what the
 * angle has now. */
static uint16_t s5_sweep_store(int16_t ddeg, uint16_t mm)
{
    uint32_t i;

    for (i = 0U; i < g_w.n; i++)
    {
        if (ddeg == g_w.s[i].ddeg)
        {
            if (0U == g_w.s[i].mm)
            {
                g_w.s[i].mm = mm;
            }
            return g_w.s[i].mm;
        }
    }
    if (g_w.n < S5_SCAN_MAX)
    {
        g_w.s[g_w.n].ddeg = ddeg;
        g_w.s[g_w.n].mm = mm;
        g_w.n++;
    }
    return mm;
}

static void s5_sweep_sample(int16_t ddeg, uint16_t mm_now)
{
    /* every ping at this angle so far: one answer is enough */
    const uint16_t mm = s5_sweep_store(ddeg, mm_now);

    if (S5_SW_COARSE == g_w.phase)
    {
        g_w.idx++;
        if (g_w.idx < S5_COARSE_N)
        {
            return;
        }
        /* coarse done: find the two edges to refine */
        s5_scan_sort(g_w.s, g_w.n);
        g_w.face_mm = s5_scan_face(g_w.s, g_w.n);
        if ((0U != g_w.face_mm) && (S5_SCAN_FINE > 0U))
        {
            int16_t l_obj;
            int16_t l_clr;
            int16_t r_obj;
            int16_t r_clr;
            bool b_l;
            bool b_r;

            s5_scan_edges(g_w.s, g_w.n, g_w.face_mm, &l_obj, &l_clr, &b_l,
                          &r_obj, &r_clr, &b_r);
            if (b_r)
            {
                g_w.obj[g_w.n_br] = r_obj;
                g_w.clr[g_w.n_br] = r_clr;
                g_w.n_br++;
            }
            if (b_l)
            {
                g_w.obj[g_w.n_br] = l_obj;
                g_w.clr[g_w.n_br] = l_clr;
                g_w.n_br++;
            }
        }
        if (0U == g_w.n_br)
        {
            s5_sweep_finish();
            return;
        }
        g_w.phase = S5_SW_FINE;
        g_w.br = 0U;
        g_w.level = 0U;
        g_w.b_check = true;
        g_w.b_again = false;
        s5_next_probe();
        return;
    }
    if (!s5_scan_is_object(mm, g_w.face_mm) && !g_w.b_again)
    {
        g_w.b_again = true; /* heard nothing: once more, same angle */
        return;
    }
    g_w.b_again = false;
    if (g_w.b_check)
    {
        g_w.b_check = false;
        if (s5_scan_is_object(mm, g_w.face_mm))
        {
            /* it was a dropout: the edge is a step further out */
            const int32_t out =
                (int32_t)g_w.clr[g_w.br] +
                (((int32_t)g_w.clr[g_w.br] > (int32_t)g_w.obj[g_w.br])
                     ? S5_SCAN_STEP_DDEG
                     : -S5_SCAN_STEP_DDEG);

            g_w.obj[g_w.br] = g_w.clr[g_w.br];
            if ((out > S5_SERVO_LIMIT_DDEG) || (out < -S5_SERVO_LIMIT_DDEG))
            {
                s5_next_bracket(); /* past the sweep: nothing to refine */
                return;
            }
            g_w.clr[g_w.br] = (int16_t)out;
            g_w.b_check = true; /* and check that one too */
        }
        s5_next_probe();
        return;
    }
    /* fine: halve the bracket */
    if (s5_scan_is_object(mm, g_w.face_mm))
    {
        g_w.obj[g_w.br] = (int16_t)g_w.probe;
    }
    else
    {
        g_w.clr[g_w.br] = (int16_t)g_w.probe;
    }
    g_w.level++;
    if (g_w.level >= S5_SCAN_FINE)
    {
        s5_next_bracket();
        return;
    }
    s5_next_probe();
}

/* Where the servo should be now; *p_b_ping: fire once it is there.
 * b_due: a ping could go out now (settled, none in flight, 60 ms up). */
static int32_t s5_target(uint32_t req, uint32_t now, bool b_due, bool *p_b_ping,
                         uint32_t *p_sweep)
{
    const uint32_t mode = req >> S5_REQ_MODE_SHIFT;
    const uint32_t seq = req & S5_REQ_SEQ_MASK;

    *p_b_ping = true;
    *p_sweep = 0U;
    if (S5_MODE_SCAN != mode)
    {
        g_w.phase = S5_SW_NONE; /* a sweep given up half way */
        g_w.seq = 0U;
        if (S5_MODE_LOOK == mode)
        {
            const int32_t a =
                (int32_t)((req >> S5_REQ_ANG_SHIFT) & S5_REQ_ANG_MASK) -
                S5_REQ_ANG_BIAS;
            const int32_t d =
                (a > g_d.servo) ? (a - g_d.servo) : (g_d.servo - a);

            if ((d <= S5_LOOK_DEADBAND_DDEG) ||
                ((d <= S5_LOOK_HOLD_DDEG) && (b_due || g_d.b_moved)))
            {
                return g_d.servo; /* ping here first, follow after */
            }
            return a;
        }
        return 0;
    }
    if ((seq != g_w.seq) && (seq != g_scan_done))
    {
        s5_sweep_start(seq);
    }
    if (S5_SW_COARSE == g_w.phase)
    {
        *p_sweep = g_w.seq;
        return s5_coarse_angle(g_w.idx);
    }
    if (S5_SW_FINE == g_w.phase)
    {
        if (g_w.b_via)
        {
            int32_t via = g_w.probe + S5_SCAN_OVERSHOOT_DDEG;

            if (via > S5_SERVO_LIMIT_DDEG)
            {
                via = S5_SERVO_LIMIT_DDEG; /* still left of the probe */
            }

            if ((g_d.servo == via) && s5_settled(now))
            {
                g_w.b_via = false;
            }
            else
            {
                *p_b_ping = false;
                return via;
            }
        }
        *p_sweep = g_w.seq;
        return g_w.probe;
    }
    return 0; /* done: straight ahead until told otherwise */
}

static void s5_read(uint32_t req)
{
    const uint32_t mode = req >> S5_REQ_MODE_SHIFT;
    s5_ping_t p = { 0 };
    uint32_t us = 0U;
    const s5_echo_t e = s5_hw_echo(&us);

    g_d.b_in_flight = false;
    p.seq = g_d.seq + 1U;
    g_d.seq = p.seq;
    p.fired_us = g_d.fire_us;
    p.ddeg = g_d.fire_ddeg;
    if (S5_ECHO_OK == e)
    {
        p.mm = s5_us_to_mm(us);
        p.b_valid = true;
    }
    else if (S5_ECHO_NONE == e)
    {
        p.mm = 0U; /* the sensor answered and heard nothing */
        p.b_valid = true;
    }
    else
    {
        p.b_valid = false;
    }

    if (p.b_valid)
    {
        g_d.dead_run = 0U;
    }
    else if (g_d.dead_run < 0xFFFFU)
    {
        g_d.dead_run++;
    }
    g_b_dead = (g_d.dead_run >= S5_DEAD_PINGS);

    /* guard: only straight-ahead pings count, a dead one changes nothing */
    if ((S5_MODE_GUARD == mode) && (0 == p.ddeg))
    {
        if (p.b_valid)
        {
            if ((0U != p.mm) && (p.mm <= (uint16_t)S5_DETECT_MM))
            {
                g_d.close_run++;
            }
            else
            {
                g_d.close_run = 0U;
            }
        }
    }
    else
    {
        g_d.close_run = 0U;
    }
    p.b_close = (g_d.close_run >= S5_DETECT_CONFIRM);
    s5_publish(&p);

    /* the sweep: only a ping fired for it, at the angle it wanted */
    if ((0U != g_d.fire_sweep) && (g_d.fire_sweep == g_w.seq) &&
        (S5_SW_NONE != g_w.phase) && p.b_valid)
    {
        const bool b_coarse = (S5_SW_COARSE == g_w.phase);
        const int32_t want = b_coarse ? s5_coarse_angle(g_w.idx) : g_w.probe;

        if ((int32_t)p.ddeg == want)
        {
            s5_sweep_sample(p.ddeg, p.mm);
        }
    }
}

void s5_sensor_poll(void)
{
    const uint32_t now = s5_hw_us();
    const uint32_t req = g_req;
    bool b_ping;
    bool b_due;
    uint32_t sweep;
    int32_t target;

    if (g_d.b_in_flight &&
        ((now - g_d.fire_us) >= ((uint32_t)S5_ECHO_READ_MS * S5_US_PER_MS)))
    {
        s5_read(req);
    }

    b_due = !g_d.b_in_flight && s5_settled(now) &&
            (!g_d.b_fired || ((now - g_d.last_fire_us) >=
                              ((uint32_t)S5_PING_PERIOD_MS * S5_US_PER_MS)));
    target = s5_target(req, now, b_due, &b_ping, &sweep);
    if (target > S5_SERVO_LIMIT_DDEG)
    {
        target = S5_SERVO_LIMIT_DDEG;
    }
    if (target < -S5_SERVO_LIMIT_DDEG)
    {
        target = -S5_SERVO_LIMIT_DDEG;
    }

    /* the servo moves only once the echo is in (the counter holds it) */
    if ((target != g_d.servo) &&
        (!g_d.b_in_flight ||
         ((now - g_d.fire_us) >= ((uint32_t)S5_ECHO_DONE_MS * S5_US_PER_MS))))
    {
        s5_move(target, now);
    }

    if (b_ping && b_due && (target == g_d.servo) && s5_settled(now))
    {
        if (s5_hw_fire())
        {
            g_d.b_in_flight = true;
            g_d.b_fired = true;
            g_d.fire_us = now;
            g_d.last_fire_us = now;
            g_d.fire_ddeg = (int16_t)g_d.servo;
            g_d.fire_sweep = sweep;
            g_d.b_moved = false;
            g_d.b_refused = false;
        }
        else if (!g_d.b_refused)
        {
            g_d.b_refused = true;
            g_d.refused_us = now;
        }
        else if ((now - g_d.refused_us) >= S5_STUCK_US)
        {
            g_b_dead = true; /* ECHO stuck high */
        }
        else
        {
            /* try again next poll */
        }
    }
}
