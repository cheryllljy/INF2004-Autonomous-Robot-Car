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
 * States (9, stored in 4 bits)
 * ----------------------------
 *   GUARD      sensor ahead, one ping per 60 ms tick
 *   CONFIRM    one close return seen; a second one starts the scan
 *   WAIT_STOP  DETECTED raised; VehicleController stops the car. Sweep
 *              only once odometry speed = 0 for 100 ms. 1 s -> FAULT.
 *   COARSE     5-point sweep, one ping each
 *   FINE       7 points, 10 deg apart, round the nearest IN-PATH return
 *   PUBLISH    build, plan, publish, sensor back to straight ahead
 *   ASSIST     controller manoeuvres; S5 keeps pinging ahead. No
 *              resume within 5 s -> FAULT
 *   FAULT      SENSOR: range UNKNOWN, E-stop cause bit set, retry every
 *              500 ms, 3 good pings -> GUARD (the E-stop stays latched
 *              until VehicleController clears it).
 *              STOP_TIMEOUT / REVERSE_LIMIT / ASSIST_TIMEOUT: latched,
 *              still pinging ahead, leave only on obstacle_reset_fault().
 */

#include "subsystem5.h"

#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "s5_config.h"

/*=====================================================================*/
/* Task-private control state: C bit-fields, 31 bits in one word       */
/*=====================================================================*/

typedef struct
{
    unsigned int state       : 4;   /* obstacle_state_t (9 values)      */
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
static uint32_t g_phase_ms      = 0U;   /* WAIT_STOP / ASSIST entry */
static uint32_t g_stopped_ms    = 0U;   /* speed first read as 0    */

/** Coarse sweep bearings, right to left. */
static const int8_t g_coarse_deg[S5_COARSE_POINTS] = { -60, -30, 0, 30, 60 };

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

    return obs_status_to_range(st, res);
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

static obstacle_state_t obs_goto_assist(void)
{
    obs_drop_request(S5_REQ_RESUME);
    g_phase_ms = obs_now_ms();

    return OBSTACLE_ST_ASSIST;
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
        if ((b >= lo) && (b <= hi))
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

    /* Timed: CR1 evidence that packed decoding costs no real time. */
    t0 = drv_ultrasonic_time_us();
    obstacle_profile_build(g_samples, (uint8_t)g_ctx.count, &p);
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
    else if ((obs_now_ms() - g_phase_ms) >= S5_ASSIST_TIMEOUT_MS)
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
        n = s5_tlm_encode(p_buf, len, g_front, &h, &p);
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

EXPORT void obstacle_register_odometry(s5_odometry_fn fn)
{
    g_odometry = fn;
}

EXPORT void obstacle_register_estop(ID flgid, UINT cause_bit)
{
    g_estop_flg = flgid;
    g_estop_bit = cause_bit;
}
