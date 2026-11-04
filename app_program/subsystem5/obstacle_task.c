/**
 * @file    obstacle_task.c
 * @brief   Subsystem 5 task: forward guard, wait-for-stop, sweep,
 *          publish, assist. One owner of the servo and the sonar, one
 *          explicit state machine.
 *
 * Design rules
 * ------------
 * 1. One task owns GP15/16/17 and PWM slices 0 and 7, so the sonar can
 *    never point sideways while another task thinks it looks ahead.
 * 2. Each state is one small function that returns the next state.
 * 3. No dynamic memory, no recursion, no VLAs, no printf on the control
 *    path. Every buffer is static and sized at compile time.
 * 4. Shared data, CR1 rules (see s5_pack.h):
 *      front word   one 32-bit word, ONE writer, ONE store: lock-free
 *      profile      packed, 16 B, copied under a priority-inheritance
 *                   mutex held for 4 word copies
 *      health       packed, 8 B, same mutex
 *      events       kernel event flag (atomic set/clear on both cores)
 *      overruns     8-bit counter, written only by the cyclic handler
 *    Private task state is a C bit-field struct (no other context
 *    touches it, so its read-modify-writes cannot race).
 *
 * States (10, stored in 4 bits)
 * -----------------------------
 *   GUARD      sensor ahead, one ping per 60 ms tick
 *   CONFIRM    one close return seen; a second one starts the scan
 *   WAIT_STOP  DETECTED raised; VehicleController stops the car. Sweep
 *              only once odometry speed = 0 for 100 ms. 1 s -> FAULT.
 *   COARSE     5-point sweep, one ping each
 *   FINE       7 points, 10 deg apart, round the nearest IN-PATH return
 *   PUBLISH    build, plan, publish, sensor back to straight ahead
 *   ASSIST     controller manoeuvres; S5 keeps pinging ahead. No word
 *              from VehicleController for 5 s -> FAULT (v3.0: VC kicks
 *              every loop, so a long bypass is fine, a dead VC is not)
 *   WATCH      v3.0, pass leg: alternate front ping and side ping at
 *              S5_WATCH_DEG towards the obstacle. Measures the gap
 *              (SIDE_CLOSE) and decides when the car is past it (PASSED)
 *              from the side data and odometry, not from a guess.
 *   FAULT      SENSOR: range UNKNOWN, E-stop cause bit set, retry every
 *              500 ms, 3 good pings -> GUARD (the E-stop stays latched
 *              until VehicleController clears it).
 *              STOP_TIMEOUT / REVERSE_LIMIT / ASSIST_TIMEOUT: latched,
 *              still pinging ahead, leave only on obstacle_reset_fault().
 */

#include "subsystem5.h"

#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_calib.h"
#include "s5_config.h"

/*=====================================================================*/
/* Task-private control state: C bit-fields, 31 bits in one word       */
/*=====================================================================*/

typedef struct
{
    unsigned int state       : 4;   /* obstacle_state_t (10 values)     */
    unsigned int fault       : 3;   /* obstacle_fault_t (5 values)      */
    unsigned int fault_run   : 4;   /* hard sensor errors in a row 0..10*/
    unsigned int good_run    : 2;   /* good pings while faulted 0..3    */
    unsigned int reverses    : 2;   /* REVERSE results in a row 0..2    */
    unsigned int count       : 5;   /* samples in g_samples 0..16       */
    unsigned int last_action : 3;   /* obstacle_action_t (6 values)     */
    unsigned int last_status : 3;   /* ultrasonic_status_t (8 values)   */
    unsigned int estop_req   : 1;   /* S5 set its E-stop cause bit      */
    unsigned int odo_ok      : 1;   /* last WaitStopped used odometry   */
    unsigned int has_profile : 1;
    unsigned int stopped     : 1;   /* WaitStopped: speed 0 seen        */
    unsigned int close_seen  : 1;   /* blind-zone logic armed           */
} s5_task_ctx_t;

S5_STATIC_ASSERT(sizeof(s5_task_ctx_t) == 4U, task_ctx_one_word);
S5_STATIC_ASSERT(S5_FAULT_LIMIT <= 15U, fault_run_fits_4_bits);
S5_STATIC_ASSERT(S5_RECOVER_SAMPLES <= 3U, good_run_fits_2_bits);
S5_STATIC_ASSERT(S5_MAX_REVERSES <= 3U, reverses_fit_2_bits);

/** v3.0 WATCH: task-private, one word. */
typedef struct
{
    unsigned int phase       : 3;   /* s5_watch_phase_t (6 values)      */
    unsigned int left        : 1;   /* obstacle is on the left          */
    unsigned int clear_looks : 2;   /* side looks in a row with nothing */
    unsigned int look_side   : 1;   /* next look is the side one        */
    unsigned int odo         : 1;   /* start distance taken             */
} s5_watch_ctx_t;

S5_STATIC_ASSERT(sizeof(s5_watch_ctx_t) == 4U, watch_ctx_one_word);
S5_STATIC_ASSERT(S5_WATCH_CLEAR_LOOKS <= 3U, clear_looks_fit_2_bits);

/** Geometry correction from s5_calib.h (v3.0). */
static const s5_geom_cal_t g_geom_cal =
{
    (int16_t)S5_CAL_BEAM_HALF_DDEG,
    (int16_t)S5_FINE_STEP_DEG,
    (int16_t)S5_SWEEP_LIMIT_DEG,
    (uint16_t)OBSTACLE_TRIGGER_MM
};

/** After a REVERSE the object is further than 300 mm on purpose: it is
 *  still the obstacle, so the rescan plans round anything up to just
 *  past where the car backed off to. */
static const s5_geom_cal_t g_geom_cal_rescan =
{
    (int16_t)S5_CAL_BEAM_HALF_DDEG,
    (int16_t)S5_FINE_STEP_DEG,
    (int16_t)S5_SWEEP_LIMIT_DEG,
    (uint16_t)(S5_REMEASURE_FAR_MM + 100U)
};

/*=====================================================================*/
/* Kernel objects                                                      */
/*=====================================================================*/

EXPORT ID flg_obstacle = 0;     /* published in subsystem5.h */

static ID g_tick_sem = 0;
static ID g_mtxid    = 0;
static ID g_cycid    = 0;

/*=====================================================================*/
/* Shared data                                                         */
/*=====================================================================*/

/** Front word. Written only by the obstacle task, with one store. */
static volatile s5_front_t g_front = 0U;

/** Packed profile and health, guarded by g_mtxid. */
static obstacle_profile_packed_t g_profile_pk;
static obstacle_health_packed_t  g_health_pk;

/** Missed guard ticks. Written only by the cyclic handler. */
static volatile uint8_t g_overruns = 0U;

/** v3.0 watch word. Written only by the obstacle task, with one store. */
static volatile s5_watch_t g_watch = 0U;

/** v3.0 VehicleController heartbeat. Written only by VC (one writer),
 *  read by the obstacle task: any change = "VC is alive". */
static volatile uint8_t g_kick = 0U;

/** Set once during subsystemN_init(), before the task exists. */
static s5_odometry_fn g_odometry  = NULL;
static ID             g_estop_flg = 0;
static UINT           g_estop_bit = 0U;

/*=====================================================================*/
/* Task-private data                                                   */
/*=====================================================================*/

static s5_task_ctx_t g_ctx;
static s5_sample_t   g_samples[OBSTACLE_MAX_SAMPLES];  /* 32 B, was 64 */

static uint16_t g_scans         = 0U;
static uint16_t g_build_us_max  = 0U;
static uint8_t  g_sensor_faults = 0U;

static uint32_t g_last_close_ms = 0U;   /* blind-zone window        */

/* v3.0 WATCH and REVERSE working data (task-private). */
static s5_watch_ctx_t g_wctx;
static int32_t  g_w_start_mm   = 0;     /* odometry at watch start     */
static int32_t  g_w_clear_mm   = 0;     /* odometry at first clear look*/
static uint16_t g_w_lat_min    = 0xFFFFU; /* closest face seen, mm     */
static uint16_t g_w_remaining  = 0U;    /* travel after clearing       */
static uint16_t g_w_last_range = OBSTACLE_RANGE_UNKNOWN;
static uint16_t g_reverse_mm   = 0U;    /* for the last REVERSE profile*/
static s5_edge_probes_t g_probes;       /* v3.0 fine edge intervals    */
static uint8_t  g_kick_seen    = 0U;
static uint8_t  g_w_last_phase = 0U;    /* for the phase log           */
static uint32_t g_phase_ms      = 0U;   /* WAIT_STOP / ASSIST entry */
static uint32_t g_stopped_ms    = 0U;   /* speed first read as 0    */

/** Coarse sweep bearings, right to left. v2.2: inside +/-30 deg, the
 *  widest the sensor bracket can turn without hitting the Pico board. */
static const int8_t g_coarse_deg[S5_COARSE_POINTS] = { -30, -20, 0, 20, 30 };

/*=====================================================================*/
/* Small helpers                                                       */
/*=====================================================================*/

/** Milliseconds since boot (low 32 bits; all uses are wrap-safe). */
static uint32_t obs_now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);

    return (uint32_t)tim.lo;
}

static bool obs_is_range(uint16_t r)
{
    return ((r != OBSTACLE_RANGE_UNKNOWN) && (r != OBSTACLE_RANGE_CLEAR));
}

static bool obs_is_close(uint16_t r)
{
    return (obs_is_range(r) && (r <= (uint16_t)OBSTACLE_TRIGGER_MM));
}

static bool obs_is_hard(uint16_t r)
{
    return (obs_is_range(r) && (r < (uint16_t)OBSTACLE_HARD_STOP_MM));
}

static void obs_raise(UINT bits)
{
    (void)tk_set_flg(flg_obstacle, bits);
}

/** Take one IN request bit if it is set. Never blocks. */
static bool obs_take_request(UINT bit)
{
    UINT ptn = 0U;

    return (tk_wai_flg(flg_obstacle, bit, TWF_ORW | TWF_BITCLR,
                       &ptn, TMO_POL) == E_OK);
}

/** Drop a stale request, so an old resume cannot end a new bypass. */
static void obs_drop_request(UINT bit)
{
    (void)tk_clr_flg(flg_obstacle, ~bit);
}

/*=====================================================================*/
/* Publication                                                         */
/*=====================================================================*/

static void obs_publish_front(uint16_t range_mm, ultrasonic_status_t st)
{
    /* Build in a register, store once. */
    g_front = s5_front_pack(range_mm, (uint32_t)st, obs_now_ms());
}

static void obs_publish_health(void)
{
    obstacle_health_t        h;
    obstacle_health_packed_t pk;

    h.state             = (obstacle_state_t)g_ctx.state;
    h.fault             = (obstacle_fault_t)g_ctx.fault;
    h.estop_requested   = (g_ctx.estop_req != 0U);
    h.has_profile       = (g_ctx.has_profile != 0U);
    h.odometry_ok       = (g_ctx.odo_ok != 0U);
    h.calibrated        = (S5_CAL_DONE != 0);
    h.reverses          = (uint8_t)g_ctx.reverses;
    h.last_status       = (uint8_t)g_ctx.last_status;
    h.deadline_overruns = g_overruns;
    h.sensor_faults     = g_sensor_faults;
    h.scans_completed   = g_scans;
    h.build_us_max      = g_build_us_max;

    obstacle_health_pack(&h, &pk);

    if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
    {
        g_health_pk = pk;
        (void)tk_unl_mtx(g_mtxid);
    }
}

static void obs_publish_profile(const obstacle_profile_packed_t *p_pk)
{
    if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
    {
        g_profile_pk = *p_pk;
        (void)tk_unl_mtx(g_mtxid);
    }

    g_ctx.has_profile = 1U;
}

/** Set S5's E-stop cause bit. Only VehicleController clears it.
 *  Setting a bit that is already set is harmless, so no "already
 *  done" check that could miss a re-latch after a reset. */
static void obs_request_estop(void)
{
    if ((g_estop_flg > 0) && (g_estop_bit != 0U))
    {
        (void)tk_set_flg(g_estop_flg, g_estop_bit);
    }

    g_ctx.estop_req = 1U;
}

/*=====================================================================*/
/* Guard tick                                                          */
/*=====================================================================*/

/** Cyclic handler, interrupt context: signal only. E_QOVR means the
 *  task had not consumed the previous tick, a missed deadline. */
static void obs_tick_handler(void *exinf)
{
    (void)exinf;

    if (tk_sig_sem(g_tick_sem, 1) == E_QOVR)
    {
        if (g_overruns < 0xFFU)
        {
            g_overruns++;
        }
    }
}

static void obs_tick_enable(bool on)
{
    if (on)
    {
        (void)tk_sta_cyc(g_cycid);
    }
    else
    {
        (void)tk_stp_cyc(g_cycid);

        while (tk_wai_sem(g_tick_sem, 1, TMO_POL) == E_OK)
        {
            /* Drain, so a stale tick cannot shorten the next period. */
        }
    }
}

static void obs_wait_tick(void)
{
    (void)tk_wai_sem(g_tick_sem, 1, TMO_FEVR);
}

/*=====================================================================*/
/* Measurement                                                         */
/*=====================================================================*/

/** Map a driver status to a range code for the geometry. */
static uint16_t obs_status_to_range(ultrasonic_status_t st,
                                    ultrasonic_result_t res)
{
    uint16_t r;

    if (st == ULTRASONIC_OK)
    {
        r = drv_us_result_mm(res);
    }
    else if (st == ULTRASONIC_TOO_CLOSE)
    {
        r = (uint16_t)DRV_US_MIN_VALID_MM;
    }
    else if ((st == ULTRASONIC_NO_ECHO) || (st == ULTRASONIC_TOO_FAR))
    {
        r = OBSTACLE_RANGE_CLEAR;
    }
    else
    {
        r = OBSTACLE_RANGE_UNKNOWN;
    }

    return r;
}

/**
 * @brief   One ping straight ahead: publish it, track sensor health and
 *          the blind zone.
 * @return  Range code (mm, CLEAR or UNKNOWN).
 */
static uint16_t obs_ping_ahead(void)
{
    ultrasonic_result_t       res;
    const ultrasonic_status_t st  = drv_ultrasonic_ping(&res);
    const uint32_t            now = obs_now_ms();
    uint16_t                  r   = obs_status_to_range(st, res);

    g_ctx.last_status = S5_FIT(st, DRV_US_STATUS_BITS);

    if (obs_is_range(r))
    {
        g_ctx.fault_run = 0U;

        if (r <= (uint16_t)S5_BLIND_RANGE_MM)
        {
            g_last_close_ms  = now;
            g_ctx.close_seen = 1U;
        }
    }
    else if (r == OBSTACLE_RANGE_CLEAR)
    {
        g_ctx.fault_run = 0U;

        /* About to touch something and the echo vanished: that is the
         * blind-zone signature, not an empty track. */
        if ((g_ctx.close_seen != 0U) &&
            ((now - g_last_close_ms) < S5_BLIND_WINDOW_MS))
        {
            r = OBSTACLE_RANGE_UNKNOWN;
        }
    }
    else
    {
        /* NO_RESPONSE, ECHO_STUCK, NOT_READY: the sensor is not telling
         * us anything. */
        if (g_ctx.fault_run < S5_FAULT_LIMIT)
        {
            g_ctx.fault_run++;
        }
    }

    obs_publish_front(r, st);

    return r;
}

/** v3.0: swept ranges are measured from the servo axis, which is what
 *  the polar geometry assumes. The sensor face sits S5_CAL_SENSOR_ARM_MM
 *  in front of it. The forward guard keeps the face distance. */
static uint16_t obs_add_arm(uint16_t r)
{
    uint16_t out = r;

    if (obs_is_range(r))
    {
        const uint32_t a = (uint32_t)r + (uint32_t)S5_CAL_SENSOR_ARM_MM;

        out = (uint16_t)((a > 0xFFFDU) ? 0xFFFDU : a);
    }

    return out;
}

/**
 * @brief   Point the sensor at a bearing and range once (or median of 3).
 *
 * The servo settling delay and the 60 ms quiet period overlap: the
 * driver only sleeps whatever is left of the quiet time.
 */
static uint16_t obs_measure_at(int16_t bearing_deg, bool use_median)
{
    const int16_t from_deg = drv_servo_get_angle();
    const int16_t to_deg   =
        (int16_t)((int16_t)DRV_SERVO_CENTRE_DEG + bearing_deg);
    ultrasonic_result_t res;
    ultrasonic_status_t st;

    (void)drv_servo_set_angle(to_deg);
    (void)tk_dly_tsk((RELTIM)drv_servo_settle_ms(from_deg, to_deg));

    st = use_median ? drv_ultrasonic_ping_median(&res)
                    : drv_ultrasonic_ping(&res);

    g_ctx.last_status = S5_FIT(st, DRV_US_STATUS_BITS);

    return obs_add_arm(obs_status_to_range(st, res));
}

/*=====================================================================*/
/* Sample buffer: packed, sorted by bearing                            */
/*=====================================================================*/

static void obs_samples_reset(void)
{
    g_ctx.count = 0U;
}

/** Insert or replace (a fine point on a coarse bearing replaces it). */
static void obs_samples_insert(int16_t bearing_deg, uint16_t range_mm)
{
    const uint8_t n = (uint8_t)g_ctx.count;
    uint8_t       i = 0U;

    while ((i < n) && (s5_sample_bearing(g_samples[i]) < bearing_deg))
    {
        i++;
    }

    if ((i < n) && (s5_sample_bearing(g_samples[i]) == bearing_deg))
    {
        g_samples[i] = s5_sample_pack(bearing_deg, range_mm);
    }
    else if (n < (uint8_t)OBSTACLE_MAX_SAMPLES)
    {
        uint8_t j = n;

        while (j > i)
        {
            g_samples[j] = g_samples[j - 1U];
            j--;
        }

        g_samples[i] = s5_sample_pack(bearing_deg, range_mm);
        g_ctx.count  = S5_FIT(n + 1U, 5U);   /* n < 16 here */
    }
    else
    {
        /* Full: dropping costs resolution, never correctness. */
    }
}

static bool obs_samples_have(int16_t bearing_deg)
{
    const uint8_t n = (uint8_t)g_ctx.count;
    uint8_t       i;
    bool          found = false;

    for (i = 0U; (i < n) && !found; i++)
    {
        found = (s5_sample_bearing(g_samples[i]) == bearing_deg);
    }

    return found;
}

/*=====================================================================*/
/* v3.0 edge probes                                                    */
/*=====================================================================*/

/** One ping at a logical angle in 0.1 deg; true if it is the object. */
static bool obs_probe_hit(int16_t ddeg, uint16_t nearest_mm)
{
    const int16_t from_deg = drv_servo_get_angle();
    ultrasonic_result_t res;
    ultrasonic_status_t st;
    uint16_t            r;
    int16_t             to_deg;

    (void)drv_servo_set_angle_ddeg(
        (int16_t)(((int16_t)DRV_SERVO_CENTRE_DEG * 10) + ddeg));
    to_deg = drv_servo_get_angle();
    (void)tk_dly_tsk((RELTIM)drv_servo_settle_ms(from_deg, to_deg));

    st = drv_ultrasonic_ping(&res);
    g_ctx.last_status = S5_FIT(st, DRV_US_STATUS_BITS);
    r  = obs_add_arm(obs_status_to_range(st, res));

    return obs_is_range(r) &&
           ((uint32_t)r <= ((uint32_t)nearest_mm + S5_PROBE_DEPTH_MM));
}

/**
 * Bisect the interval between the outermost bearing that heard the
 * object (hit) and the next sample out (miss). towards = +1 left edge,
 * -1 right edge.
 */
static void obs_probe_edge(s5_edge_probe_t *p_probe, int16_t edge_deg,
                           int16_t towards, uint16_t nearest_mm)
{
    int32_t hit  = (int32_t)edge_deg * 10;
    int32_t miss = hit + ((int32_t)towards * S5_FINE_STEP_DEG * 10);
    uint8_t k;

    for (k = 0U; k < (uint8_t)S5_EDGE_PROBES; k++)
    {
        const int32_t mid = (hit + miss) / 2;

        if (obs_probe_hit((int16_t)mid, nearest_mm))
        {
            hit = mid;
        }
        else
        {
            miss = mid;
        }
    }

    p_probe->valid     = true;
    p_probe->hit_ddeg  = (int16_t)hit;
    p_probe->miss_ddeg = (int16_t)miss;
}

/** Probe every edge the sweep saw (an edge at the sweep limit was not
 *  seen: nothing to narrow). */
static void obs_probe_edges(const obstacle_profile_t *p)
{
    g_probes.left.valid  = false;
    g_probes.right.valid = false;

    if (p->valid && obs_is_range(p->nearest_mm))
    {
        if (p->left_edge_deg < (int16_t)S5_SWEEP_LIMIT_DEG)
        {
            obs_probe_edge(&g_probes.left, p->left_edge_deg, 1,
                           p->nearest_mm);
        }

        if (p->right_edge_deg > (int16_t)(-S5_SWEEP_LIMIT_DEG))
        {
            obs_probe_edge(&g_probes.right, p->right_edge_deg, -1,
                           p->nearest_mm);
        }
    }
}

/**
 * @brief   Bearing of the nearest return INSIDE the driving path, or 0.
 *
 * v1 centred the fine scan on the nearest return anywhere, so a side
 * wall could pull the fine scan off the real obstacle.
 */
static int16_t obs_fine_centre(void)
{
    const uint8_t n       = (uint8_t)g_ctx.count;
    uint16_t      best    = OBSTACLE_RANGE_UNKNOWN;
    int16_t       bearing = 0;
    uint8_t       i;

    for (i = 0U; i < n; i++)
    {
        const int16_t  b = s5_sample_bearing(g_samples[i]);
        const uint16_t r = s5_sample_range(g_samples[i]);

        if (obstacle_in_path(b, r) &&
            ((best == OBSTACLE_RANGE_UNKNOWN) || (r < best)))
        {
            best    = r;
            bearing = b;
        }
    }

    return bearing;
}

/*=====================================================================*/
/* State transitions                                                   */
/*=====================================================================*/

static obstacle_state_t obs_goto_fault(obstacle_fault_t cause)
{
    g_ctx.fault    = S5_FIT(cause, S5_FAULT_BITS);
    g_ctx.good_run = 0U;
    obs_drop_request(S5_REQ_RESET);
    obs_raise(S5_EVT_FAULT);

    if (cause == OBSTACLE_FAULT_SENSOR)
    {
        if (g_sensor_faults < 0xFFU)
        {
            g_sensor_faults++;
        }

        /* Blind robot: the one case S5 uses the E-stop. */
        obs_request_estop();
        obs_tick_enable(false);
        obs_publish_front(OBSTACLE_RANGE_UNKNOWN, ULTRASONIC_NO_RESPONSE);
    }
    else
    {
        /* Mission fault: sensor fine, keep the forward range alive. */
        (void)drv_servo_set_angle((int16_t)DRV_SERVO_CENTRE_DEG);
        obs_tick_enable(true);
    }

    tm_printf((UB *)"S5: FAULT cause %d\n", (INT)cause);

    return OBSTACLE_ST_FAULT;
}

static obstacle_state_t obs_goto_wait_stop(uint16_t r)
{
    obs_raise(obs_is_hard(r) ? (S5_EVT_DETECTED | S5_EVT_HARD_STOP)
                             : S5_EVT_DETECTED);
    g_phase_ms    = obs_now_ms();
    g_ctx.stopped = 0U;

    return OBSTACLE_ST_WAIT_STOP;
}

/** ASSIST for a NEW profile: a request left over from the previous
 *  obstacle must not steer this one. */
static obstacle_state_t obs_goto_assist(void)
{
    obs_drop_request(S5_REQ_RESUME | S5_REQ_WATCH_LEFT | S5_REQ_WATCH_RIGHT |
                     S5_REQ_WATCH_END | S5_REQ_RESCAN);
    g_phase_ms  = obs_now_ms();
    g_kick_seen = g_kick;

    return OBSTACLE_ST_ASSIST;
}

/** Back to ASSIST in the middle of the same bypass (from WATCH): keep
 *  whatever VC has already asked for next. */
static obstacle_state_t obs_return_assist(void)
{
    g_phase_ms  = obs_now_ms();
    g_kick_seen = g_kick;

    return OBSTACLE_ST_ASSIST;
}

/** VehicleController said something (kick or command): restart the
 *  ASSIST / WATCH watchdog. */
static void obs_note_kick(void)
{
    const uint8_t k = g_kick;

    if (k != g_kick_seen)
    {
        g_kick_seen = k;
        g_phase_ms  = obs_now_ms();
    }
}

static bool obs_vc_silent(void)
{
    return ((obs_now_ms() - g_phase_ms) >= S5_ASSIST_TIMEOUT_MS);
}

/** v3.0 rescan after a REVERSE: the car was stopped by VC, so no
 *  DETECTED event, just WaitStopped and a fresh sweep. */
static obstacle_state_t obs_goto_rescan(void)
{
    g_phase_ms    = obs_now_ms();
    g_ctx.stopped = 0U;
    obs_drop_request(S5_REQ_WATCH_END);     /* superseded by the rescan */
    obs_tick_enable(true);  /* WAIT_STOP runs on the tick; WATCH had it off */

    return OBSTACLE_ST_WAIT_STOP;
}

static void obs_publish_watch(void)
{
    g_watch = s5_watch_pack((g_wctx.left != 0U),
                            (s5_watch_phase_t)g_wctx.phase, g_reverse_mm,
                            g_w_lat_min, g_w_last_range);
}

static bool obs_odo_distance(int32_t *p_mm)
{
    bool ok = false;

    if (g_odometry != NULL)
    {
        int32_t speed = 0;

        ok = g_odometry(&speed, p_mm);
    }

    return ok;
}

static obstacle_state_t obs_goto_watch(bool left)
{
    int32_t d = 0;

    obs_tick_enable(false);

    g_wctx.phase       = S5_FIT(S5_WATCH_LOOKING, S5_WATCH_PHASE_BITS);
    g_wctx.left        = left ? 1U : 0U;
    g_wctx.clear_looks = 0U;
    g_wctx.look_side   = 1U;
    g_wctx.odo         = obs_odo_distance(&d) ? 1U : 0U;
    g_w_start_mm       = d;
    g_w_clear_mm       = d;
    g_w_lat_min        = 0xFFFFU;
    g_w_remaining      = 0U;
    g_w_last_range     = OBSTACLE_RANGE_UNKNOWN;
    g_w_last_phase     = (uint8_t)S5_WATCH_LOOKING;
    g_phase_ms         = obs_now_ms();
    g_kick_seen        = g_kick;
    obs_publish_watch();

    return OBSTACLE_ST_WATCH;
}

/** Servo back to straight ahead, waiting for it to settle. */
static void obs_servo_centre(void)
{
    const int16_t from_deg = drv_servo_get_angle();

    if (from_deg != (int16_t)DRV_SERVO_CENTRE_DEG)
    {
        (void)drv_servo_set_angle((int16_t)DRV_SERVO_CENTRE_DEG);
        (void)tk_dly_tsk((RELTIM)drv_servo_settle_ms(
            from_deg, (int16_t)DRV_SERVO_CENTRE_DEG));
    }
}

/** Leave WATCH for ASSIST (front guard on the tick, servo ahead). */
static obstacle_state_t obs_watch_to_assist(void)
{
    obs_servo_centre();
    (void)obs_ping_ahead();
    obs_tick_enable(true);

    return obs_return_assist();
}

static obstacle_state_t obs_goto_guard(void)
{
    g_ctx.fault    = S5_FIT(OBSTACLE_FAULT_NONE, S5_FAULT_BITS);
    g_ctx.good_run = 0U;
    obs_tick_enable(true);

    return OBSTACLE_ST_GUARD;
}

/*=====================================================================*/
/* State handlers. Each returns the next state.                        */
/*=====================================================================*/

static obstacle_state_t obs_state_guard(void)
{
    obstacle_state_t next = OBSTACLE_ST_GUARD;
    uint16_t         r;

    obs_wait_tick();
    r = obs_ping_ahead();

    if (g_ctx.fault_run >= S5_FAULT_LIMIT)
    {
        next = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
    }
    else if (obs_is_hard(r))
    {
        /* Inside 120 mm there is no time for a second ping. */
        next = obs_goto_wait_stop(r);
    }
    else if (obs_is_close(r))
    {
        next = OBSTACLE_ST_CONFIRM;
    }
    else
    {
        /* Stay in guard. */
    }

    return next;
}

static obstacle_state_t obs_state_confirm(void)
{
    obstacle_state_t next = OBSTACLE_ST_GUARD;
    uint16_t         r;

    obs_wait_tick();
    r = obs_ping_ahead();

    if (g_ctx.fault_run >= S5_FAULT_LIMIT)
    {
        next = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
    }
    else if (obs_is_close(r))
    {
        next = obs_goto_wait_stop(r);
    }
    else
    {
        /* One-off reflection: back to guard. */
    }

    return next;
}

/**
 * @brief   Wait for the car to stop before sweeping (tutor feedback:
 *          the servo must not sweep while the robot is moving).
 */
static obstacle_state_t obs_state_wait_stop(void)
{
    obstacle_state_t next = OBSTACLE_ST_WAIT_STOP;
    uint32_t         now;

    obs_wait_tick();
    (void)obs_ping_ahead();     /* forward range stays fresh meanwhile */
    now = obs_now_ms();

    if (g_ctx.fault_run >= S5_FAULT_LIMIT)
    {
        next = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
    }
    else if (g_odometry != NULL)
    {
        int32_t speed = 0;
        int32_t dist  = 0;

        if (g_odometry(&speed, &dist) &&
            (speed <= S5_STOPPED_SPEED_MM_S) &&
            (speed >= -S5_STOPPED_SPEED_MM_S))
        {
            if (g_ctx.stopped == 0U)
            {
                g_ctx.stopped = 1U;
                g_stopped_ms  = now;
            }
            else if ((now - g_stopped_ms) >= S5_STOPPED_HOLD_MS)
            {
                g_ctx.odo_ok = 1U;
                next         = OBSTACLE_ST_COARSE;
            }
            else
            {
                /* Holding still, not long enough yet. */
            }
        }
        else
        {
            g_ctx.stopped = 0U;
        }

        if ((next == OBSTACLE_ST_WAIT_STOP) &&
            ((now - g_phase_ms) >= S5_STOP_TIMEOUT_MS))
        {
            next = obs_goto_fault(OBSTACLE_FAULT_STOP_TIMEOUT);
        }
    }
    else if ((now - g_phase_ms) >= S5_STOP_FALLBACK_MS)
    {
        /* No odometry registered yet (bench, or before Buddy 2's API
         * exists). Fixed wait, flagged in health so it is visible. */
        g_ctx.odo_ok = 0U;
        next         = OBSTACLE_ST_COARSE;
    }
    else
    {
        /* Keep waiting. */
    }

    return next;
}

static obstacle_state_t obs_state_coarse(void)
{
    uint8_t i;

    obs_tick_enable(false);
    obs_samples_reset();

    for (i = 0U; i < (uint8_t)S5_COARSE_POINTS; i++)
    {
        const int16_t b = (int16_t)g_coarse_deg[i];

        obs_samples_insert(b, obs_measure_at(b, false));
    }

    return OBSTACLE_ST_FINE;
}

/**
 * @brief   Fine sweep round the in-path target. Only the centre point
 *          uses the median (all seven would add ~0.7 s). The sweep
 *          starts from the end nearest the servo, which cuts up to
 *          ~0.25 s of travel after the coarse pass.
 */
static obstacle_state_t obs_state_fine(void)
{
    const int16_t centre = obs_fine_centre();
    const int16_t span   =
        (int16_t)(((int16_t)S5_FINE_POINTS / 2) * S5_FINE_STEP_DEG);
    const int16_t now_b  =
        (int16_t)(drv_servo_get_angle() - (int16_t)DRV_SERVO_CENTRE_DEG);
    int16_t lo = (int16_t)(centre - span);
    int16_t hi = (int16_t)(centre + span);
    int16_t step;
    int16_t b;
    uint8_t i;

    if (lo < -S5_SWEEP_LIMIT_DEG) { lo = (int16_t)(-S5_SWEEP_LIMIT_DEG); }
    if (hi > S5_SWEEP_LIMIT_DEG)  { hi = (int16_t)S5_SWEEP_LIMIT_DEG; }

    if ((now_b - lo) <= (hi - now_b))
    {
        b    = lo;
        step = (int16_t)S5_FINE_STEP_DEG;
    }
    else
    {
        b    = hi;
        step = (int16_t)(-S5_FINE_STEP_DEG);
    }

    for (i = 0U; i < (uint8_t)S5_FINE_POINTS; i++)
    {
        /* v3.0: a coarse bearing is not measured twice, except the
         * centre (median of 3). The time saved pays for the edge
         * probes in PUBLISH. */
        if ((b >= lo) && (b <= hi) &&
            ((b == centre) || !obs_samples_have(b)))
        {
            obs_samples_insert(b, obs_measure_at(b, (b == centre)));
        }

        b = (int16_t)(b + step);
    }

    return OBSTACLE_ST_PUBLISH;
}

static obstacle_state_t obs_state_publish(void)
{
    obstacle_state_t          next = OBSTACLE_ST_GUARD;
    obstacle_profile_t        p;
    obstacle_profile_packed_t pk;
    uint32_t                  t0;
    uint32_t                  dt;
    bool                      third_reverse = false;

    /* v3.0: narrow each seen edge with two extra pings first (servo
     * time, so outside the timed part). */
    obstacle_profile_build(g_samples, (uint8_t)g_ctx.count, &p);
    obs_probe_edges(&p);

    /* Timed: CR1 evidence that packed decoding costs no real time. */
    t0 = drv_ultrasonic_time_us();
    obstacle_profile_build(g_samples, (uint8_t)g_ctx.count, &p);
    /* v3.0: beam-width edges, true centre, unseen edges -> REVERSE */
    obstacle_profile_refine(&p, (g_ctx.reverses != 0U) ? &g_geom_cal_rescan
                                                       : &g_geom_cal,
                            &g_probes);
    dt = drv_ultrasonic_time_us() - t0;

    if (dt > (uint32_t)g_build_us_max)
    {
        g_build_us_max = (uint16_t)s5_sat_u(dt, 16U);
    }

    p.timestamp_ms = obs_now_ms();

    if (p.action == OBSTACLE_ACT_REVERSE)
    {
        if (g_ctx.reverses >= S5_MAX_REVERSES)
        {
            /* Third REVERSE in a row: boxed in. Publish STOP so the
             * controller does not back off a third time. */
            p.action      = OBSTACLE_ACT_STOP;
            third_reverse = true;
        }
        else
        {
            g_ctx.reverses++;
        }
    }
    else if (p.action == OBSTACLE_ACT_CONTINUE)
    {
        g_ctx.reverses = 0U;
    }
    else
    {
        /* Bypass resets the count on resume; STOP/SLOW keep it. */
    }

    /* v3.0: how far VC should back up, published BEFORE the event so
     * VC reads the matching value when it sees PROFILED. */
    g_reverse_mm = obstacle_reverse_mm(&p);
    g_wctx.phase = S5_FIT(S5_WATCH_IDLE, S5_WATCH_PHASE_BITS);
    obs_publish_watch();

    obstacle_profile_pack(&p, &pk);
    obs_publish_profile(&pk);
    g_ctx.last_action = S5_FIT(p.action, S5_ACTION_BITS);

    if (g_scans < 0xFFFFU)
    {
        g_scans++;
    }

    /* Sensor straight ahead again, and a fresh forward range, before
     * the controller is told to move. */
    {
        const int16_t from_deg = drv_servo_get_angle();

        (void)drv_servo_set_angle((int16_t)DRV_SERVO_CENTRE_DEG);
        (void)tk_dly_tsk((RELTIM)drv_servo_settle_ms(
            from_deg, (int16_t)DRV_SERVO_CENTRE_DEG));
    }
    (void)obs_ping_ahead();
    obs_tick_enable(true);

    if (third_reverse)
    {
        next = obs_goto_fault(OBSTACLE_FAULT_REVERSE_LIMIT);
    }
    else
    {
        obs_raise((p.action == OBSTACLE_ACT_CONTINUE) ? S5_EVT_CLEARED
                                                      : S5_EVT_PROFILED);

        if ((p.action == OBSTACLE_ACT_BYPASS_LEFT) ||
            (p.action == OBSTACLE_ACT_BYPASS_RIGHT) ||
            (p.action == OBSTACLE_ACT_REVERSE))
        {
            next = obs_goto_assist();
        }
        else
        {
            /* CONTINUE / SLOW / STOP: nothing to wait for. If it is
             * still there, guard detects it again and rescans. */
            next = OBSTACLE_ST_GUARD;
        }
    }

    return next;
}

static obstacle_state_t obs_state_assist(void)
{
    obstacle_state_t next = OBSTACLE_ST_ASSIST;
    uint16_t         r;

    obs_wait_tick();
    r = obs_ping_ahead();
    obs_note_kick();

    if (g_ctx.fault_run >= S5_FAULT_LIMIT)
    {
        next = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
    }
    else if (obs_take_request(S5_REQ_RESUME))
    {
        if (g_ctx.last_action != (unsigned int)OBSTACLE_ACT_REVERSE)
        {
            g_ctx.reverses = 0U;    /* got past it */
        }

        next = OBSTACLE_ST_GUARD;
    }
    else if (obs_take_request(S5_REQ_RESCAN))
    {
        /* v3.0: VC backed off; look again from here. */
        next = obs_goto_rescan();
    }
    else if (obs_take_request(S5_REQ_WATCH_LEFT))
    {
        next = obs_goto_watch(true);
    }
    else if (obs_take_request(S5_REQ_WATCH_RIGHT))
    {
        next = obs_goto_watch(false);
    }
    else if (obs_vc_silent())
    {
        next = obs_goto_fault(OBSTACLE_FAULT_ASSIST_TIMEOUT);
    }
    else
    {
        if (obs_is_hard(r))
        {
            /* Something under 120 mm while manoeuvring. */
            obs_raise(S5_EVT_HARD_STOP);
        }
    }

    return next;
}

/**
 * @brief   v3.0 WATCH: one look per call, alternating side and front.
 *
 * Side look (servo at S5_WATCH_DEG towards the obstacle):
 *   LOOKING  -> SEEN      the face is within S5_WATCH_SEEN_LAT_MM beside us
 *   SEEN     -> CLEARING  S5_WATCH_CLEAR_LOOKS looks in a row with nothing
 *                         beside us; the remaining travel is worked out
 *                         from the closest face seen
 *   CLEARING -> SEEN      the face came back (a longer or uneven object)
 *   LOOKING  -> BLIND     S5_WATCH_BLIND_MM driven and never seen it
 *   any: face closer than the chassis half-width + S5_SIDE_CLOSE_MM
 *        -> S5_EVT_SIDE_CLOSE (VC steers away)
 * Front look: forward guard, S5_EVT_HARD_STOP under 120 mm.
 * CLEARING and the odometry say the rear is past -> S5_EVT_PASSED,
 * back to ASSIST.
 */
static obstacle_state_t obs_state_watch(void)
{
    obstacle_state_t next = OBSTACLE_ST_WATCH;
    int32_t          dist = 0;
    const bool       have = obs_odo_distance(&dist);

    if (have && (g_wctx.odo == 0U))
    {
        g_wctx.odo   = 1U;
        g_w_start_mm = dist;
    }

    if (g_wctx.look_side != 0U)
    {
        const int16_t b = (g_wctx.left != 0U) ? (int16_t)S5_WATCH_DEG
                                              : (int16_t)(-S5_WATCH_DEG);
        const uint16_t r   = obs_measure_at(b, false);
        const uint16_t lat = obstacle_watch_lateral_mm(
            r, (int16_t)S5_WATCH_DEG, (int16_t)S5_CAL_BEAM_HALF_DDEG);
        const bool beside  = obs_is_range(r) &&
                             (lat <= (uint16_t)S5_WATCH_SEEN_LAT_MM);

        g_w_last_range = r;

        if (beside)
        {
            if (lat < g_w_lat_min)
            {
                g_w_lat_min = lat;
            }

            if (lat < (uint16_t)(OBSTACLE_HALF_WIDTH_MM + S5_SIDE_CLOSE_MM))
            {
                obs_raise(S5_EVT_SIDE_CLOSE);
            }

            g_wctx.phase       = S5_FIT(S5_WATCH_SEEN, S5_WATCH_PHASE_BITS);
            g_wctx.clear_looks = 0U;
        }
        else if ((g_wctx.phase == (unsigned int)S5_WATCH_SEEN) ||
                 (g_wctx.phase == (unsigned int)S5_WATCH_CLEARING))
        {
            if (g_wctx.clear_looks < S5_WATCH_CLEAR_LOOKS)
            {
                if (g_wctx.clear_looks == 0U)
                {
                    g_w_clear_mm = dist;    /* first look past the face */
                }
                g_wctx.clear_looks++;
            }

            if ((g_wctx.phase == (unsigned int)S5_WATCH_SEEN) &&
                (g_wctx.clear_looks >= S5_WATCH_CLEAR_LOOKS))
            {
                g_w_remaining = obstacle_watch_remaining_mm(
                    g_w_lat_min, (int16_t)S5_WATCH_DEG,
                    (int16_t)S5_CAL_BEAM_HALF_DDEG);
                g_wctx.phase  = S5_FIT(S5_WATCH_CLEARING,
                                       S5_WATCH_PHASE_BITS);
            }
        }
        else if ((g_wctx.phase == (unsigned int)S5_WATCH_LOOKING) &&
                 have && (g_wctx.odo != 0U) &&
                 ((dist - g_w_start_mm) >= (int32_t)S5_WATCH_BLIND_MM))
        {
            g_wctx.phase = S5_FIT(S5_WATCH_BLIND, S5_WATCH_PHASE_BITS);
        }
        else
        {
            /* Still looking, or BLIND: nothing new. */
        }
    }
    else
    {
        uint16_t r;

        obs_servo_centre();
        r = obs_ping_ahead();

        if (obs_is_hard(r))
        {
            obs_raise(S5_EVT_HARD_STOP);
        }
    }

    g_wctx.look_side = S5_FIT(g_wctx.look_side ^ 1U, 1U);
    obs_note_kick();

    if (g_wctx.phase != g_w_last_phase)
    {
        /* A few lines per bypass; the evidence for "passed". */
        tm_printf((UB *)"S5: watch %d -> %d, face %d mm beside, at %d mm\n",
                  (INT)g_w_last_phase, (INT)g_wctx.phase, (INT)g_w_lat_min,
                  (INT)(dist - g_w_start_mm));
        g_w_last_phase = (uint8_t)g_wctx.phase;
    }

    if ((g_wctx.phase == (unsigned int)S5_WATCH_CLEARING) && have &&
        ((dist - g_w_clear_mm) >= (int32_t)g_w_remaining))
    {
        g_wctx.phase = S5_FIT(S5_WATCH_PASSED, S5_WATCH_PHASE_BITS);
        obs_publish_watch();
        obs_raise(S5_EVT_PASSED);
        next = obs_watch_to_assist();
    }
    else if (g_ctx.fault_run >= S5_FAULT_LIMIT)
    {
        next = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
    }
    else if (obs_take_request(S5_REQ_RESCAN))
    {
        /* Checked before WATCH_END: VC sends both when it gives up on
         * a leg, and the rescan is the one that matters. */
        obs_servo_centre();
        next = obs_goto_rescan();
    }
    else if (obs_take_request(S5_REQ_WATCH_END))
    {
        next = obs_watch_to_assist();
    }
    else if (obs_take_request(S5_REQ_RESUME))
    {
        obs_servo_centre();
        g_ctx.reverses = 0U;
        obs_tick_enable(true);
        next = OBSTACLE_ST_GUARD;
    }
    else if (obs_vc_silent())
    {
        obs_servo_centre();
        next = obs_goto_fault(OBSTACLE_FAULT_ASSIST_TIMEOUT);
    }
    else
    {
        /* Keep watching. */
    }

    if (next == OBSTACLE_ST_WATCH)
    {
        obs_publish_watch();
    }

    return next;
}

static obstacle_state_t obs_state_fault(void)
{
    obstacle_state_t next = OBSTACLE_ST_FAULT;

    if (g_ctx.fault == (unsigned int)OBSTACLE_FAULT_SENSOR)
    {
        ultrasonic_result_t res;
        ultrasonic_status_t st;

        (void)tk_dly_tsk((RELTIM)S5_FAULT_RETRY_MS);

        st = drv_ultrasonic_ping(&res);
        g_ctx.last_status = S5_FIT(st, DRV_US_STATUS_BITS);

        /* Not trusted yet: publish UNKNOWN whatever the ping said. */
        obs_publish_front(OBSTACLE_RANGE_UNKNOWN, st);

        /* Recovery needs proof that ECHO moves. Silence is not proof. */
        if ((st == ULTRASONIC_OK) || (st == ULTRASONIC_NO_ECHO) ||
            (st == ULTRASONIC_TOO_CLOSE) || (st == ULTRASONIC_TOO_FAR))
        {
            if (g_ctx.good_run < S5_RECOVER_SAMPLES)
            {
                g_ctx.good_run++;
            }
        }
        else
        {
            g_ctx.good_run = 0U;
        }

        if ((g_ctx.good_run >= S5_RECOVER_SAMPLES) ||
            obs_take_request(S5_REQ_RESET))
        {
            g_ctx.fault_run = 0U;
            next = obs_goto_guard();    /* E-stop stays latched */
        }
    }
    else
    {
        obs_wait_tick();
        (void)obs_ping_ahead();

        if (g_ctx.fault_run >= S5_FAULT_LIMIT)
        {
            next = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
        }
        else if (obs_take_request(S5_REQ_RESET))
        {
            g_ctx.reverses  = 0U;
            g_ctx.estop_req = 0U;
            next = obs_goto_guard();
        }
        else
        {
            /* Latched until VehicleController resets it. */
        }
    }

    return next;
}

/*=====================================================================*/
/* Task                                                                */
/*=====================================================================*/

EXPORT void obstacle_task(INT stacd, void *exinf)
{
    obstacle_state_t state;

    (void)stacd;
    (void)exinf;

    /* The owning task initialises its own hardware (TEAM.md sec. 3). */
    if (drv_servo_init() && drv_ultrasonic_init())
    {
        tm_printf((UB *)"S5: ObstacleScan up (servo GP15, sonar GP16/17)\n");
        state = obs_goto_guard();
    }
    else
    {
        tm_printf((UB *)"S5: driver init failed\n");
        state = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
    }

    for (;;)
    {
        g_ctx.state = S5_FIT(state, S5_STATE_BITS);
        obs_publish_health();

        switch (state)
        {
            case OBSTACLE_ST_GUARD:     state = obs_state_guard();     break;
            case OBSTACLE_ST_CONFIRM:   state = obs_state_confirm();   break;
            case OBSTACLE_ST_WAIT_STOP: state = obs_state_wait_stop(); break;
            case OBSTACLE_ST_COARSE:    state = obs_state_coarse();    break;
            case OBSTACLE_ST_FINE:      state = obs_state_fine();      break;
            case OBSTACLE_ST_PUBLISH:   state = obs_state_publish();   break;
            case OBSTACLE_ST_ASSIST:    state = obs_state_assist();    break;
            case OBSTACLE_ST_WATCH:     state = obs_state_watch();     break;
            case OBSTACLE_ST_FAULT:     state = obs_state_fault();     break;
            case OBSTACLE_ST_INIT:
            default:
                state = obs_goto_fault(OBSTACLE_FAULT_SENSOR);
                break;
        }
    }
}

/*=====================================================================*/
/* Creation (subsystem5_init)                                          */
/*=====================================================================*/

EXPORT ER obstacle_create_objects(void)
{
    T_CSEM csem = { 0 };
    T_CMTX cmtx = { 0 };
    T_CFLG cflg = { 0 };
    T_CCYC ccyc = { 0 };
    obstacle_health_t        h0 = { 0 };   /* state INIT, all clear */
    obstacle_profile_t       p0;
    ER     er = E_OK;

    /* Safe defaults before anyone can read them. */
    g_front = s5_front_pack(OBSTACLE_RANGE_UNKNOWN,
                            (uint32_t)ULTRASONIC_NOT_READY, 0U);
    obstacle_health_pack(&h0, &g_health_pk);
    obstacle_profile_build(NULL, 0U, &p0);      /* invalid, STOP */
    obstacle_profile_pack(&p0, &g_profile_pk);

    csem.sematr  = TA_TFIFO | TA_FIRST;
    csem.isemcnt = 0;
    csem.maxsem  = 1;
    g_tick_sem   = tk_cre_sem(&csem);

    cmtx.mtxatr  = TA_TFIFO | TA_INHERIT;
    g_mtxid      = tk_cre_mtx(&cmtx);

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    flg_obstacle = tk_cre_flg(&cflg);

    ccyc.cycatr  = TA_HLNG;                     /* created stopped */
    ccyc.cychdr  = (FP)obs_tick_handler;
    ccyc.cyctim  = (RELTIM)S5_GUARD_PERIOD_MS;
    ccyc.cycphs  = (RELTIM)S5_GUARD_PERIOD_MS;
    g_cycid      = tk_cre_cyc(&ccyc);

    if (g_tick_sem < E_OK)   { er = g_tick_sem; }
    if (g_mtxid < E_OK)      { er = g_mtxid; }
    if (flg_obstacle < E_OK) { er = flg_obstacle; }
    if (g_cycid < E_OK)      { er = g_cycid; }

    return er;
}

/*=====================================================================*/
/* Public interface                                                    */
/*=====================================================================*/

EXPORT uint16_t obstacle_get_front_mm(void)
{
    const s5_front_t w     = g_front;   /* one 32-bit load */
    uint16_t         range = s5_front_range(w);

    if (s5_age_ms(s5_front_stamp(w), obs_now_ms()) > S5_STALE_MS)
    {
        range = OBSTACLE_RANGE_UNKNOWN;
    }

    return range;
}

EXPORT bool obstacle_get_profile_packed(obstacle_profile_packed_t *p_out)
{
    bool ok = false;

    if ((p_out != NULL) && (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK))
    {
        obstacle_health_t h;

        *p_out = g_profile_pk;
        obstacle_health_unpack(&g_health_pk, &h);
        (void)tk_unl_mtx(g_mtxid);

        ok = h.has_profile;
    }

    return ok;
}

EXPORT bool obstacle_get_profile(obstacle_profile_t *p_out)
{
    obstacle_profile_packed_t pk;
    bool                      ok = false;

    if (p_out != NULL)
    {
        ok = obstacle_get_profile_packed(&pk);
        obstacle_profile_unpack(&pk, p_out);
    }

    return ok;
}

EXPORT void obstacle_get_health(obstacle_health_t *p_out)
{
    if ((p_out != NULL) && (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK))
    {
        const obstacle_health_packed_t pk = g_health_pk;

        (void)tk_unl_mtx(g_mtxid);
        obstacle_health_unpack(&pk, p_out);
    }
}

EXPORT uint32_t obstacle_get_telemetry(uint8_t *p_buf, uint32_t len)
{
    uint32_t n = 0U;

    if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
    {
        const obstacle_health_packed_t  h = g_health_pk;
        const obstacle_profile_packed_t p = g_profile_pk;

        (void)tk_unl_mtx(g_mtxid);
        n = s5_tlm_encode(p_buf, len, g_front, &h, &p, g_watch);
    }

    return n;
}

EXPORT void obstacle_resume(void)
{
    (void)tk_set_flg(flg_obstacle, S5_REQ_RESUME);
}

EXPORT void obstacle_reset_fault(void)
{
    (void)tk_set_flg(flg_obstacle, S5_REQ_RESET);
}

EXPORT void obstacle_rescan(void)
{
    (void)tk_set_flg(flg_obstacle, S5_REQ_RESCAN);
}

EXPORT void obstacle_watch_start(bool obstacle_on_left)
{
    (void)tk_set_flg(flg_obstacle, obstacle_on_left ? S5_REQ_WATCH_LEFT
                                                    : S5_REQ_WATCH_RIGHT);
}

EXPORT void obstacle_watch_end(void)
{
    (void)tk_set_flg(flg_obstacle, S5_REQ_WATCH_END);
}

EXPORT void obstacle_assist_kick(void)
{
    /* One writer (VehicleController), so the ++ cannot race. */
    g_kick = (uint8_t)(g_kick + 1U);
}

EXPORT void obstacle_get_watch(obstacle_watch_t *p_out)
{
    if (p_out != NULL)
    {
        const s5_watch_t w = g_watch;   /* one 32-bit load */

        p_out->obstacle_on_left = s5_watch_left(w);
        p_out->phase            = s5_watch_phase(w);
        p_out->reverse_mm       = s5_watch_reverse_mm(w);
        p_out->lateral_mm       = s5_watch_lateral_mm(w);
        p_out->side_range_mm    = s5_watch_range(w);
    }
}

EXPORT void obstacle_register_odometry(s5_odometry_fn fn)
{
    g_odometry = fn;
}

EXPORT void obstacle_register_estop(ID flgid, UINT cause_bit)
{
    g_estop_flg = flgid;
    g_estop_bit = cause_bit;
}
