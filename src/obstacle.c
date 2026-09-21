/**
 * @file    obstacle.c
 * @brief   Buddy 5 task: forward guard, sweep, publish. One owner of
 *          the servo and the ranger, one explicit state machine.
 *
 * Design rules this file follows
 * ------------------------------
 * 1. One task owns both pieces of hardware, so the sensor needs no
 *    lock and can never be pointed sideways while another task
 *    believes it is looking ahead.
 * 2. Every state is one small function that returns the next state.
 *    The task loop is a dispatcher and nothing else.
 * 3. No dynamic memory, no recursion, no variable-length arrays, no
 *    printf on the control path. All buffers are static and sized at
 *    compile time.
 * 4. Shared data leaves the module only through getters that take a
 *    priority-inheriting mutex, held for a struct copy and no longer.
 *
 * States
 * ------
 *   GUARD   sensor ahead, one ping per tick, watching for a return
 *           closer than OBSTACLE_TRIGGER_MM
 *   COARSE  five-point sweep, single ping per point
 *   FINE    seven-point sweep around the nearest coarse return
 *   PUBLISH build the profile, choose an action, raise the event
 *   ASSIST  controller is steering round it; sensor back to straight
 *           ahead at the guard rate so forward protection continues
 *   FAULT   sensor unhealthy; range reported as UNKNOWN until it
 *           recovers, which forces the controller to slow or stop
 */

#include "obstacle.h"

#include "board_config.h"
#include "servo.h"
#include "ultrasonic.h"

#include <tk/tkernel.h>

#include <stddef.h>

/*=====================================================================*/
/* Tunable constants                                                   */
/*=====================================================================*/

/** Guard ping period. Cannot go below the sensor's 60 ms quiet time. */
#define OBS_GUARD_PERIOD_MS     (60U)

/** Consecutive close returns needed before a sweep is launched. */
#define OBS_CONFIRM_SAMPLES     (2U)

/** A forward range older than this is reported as UNKNOWN. */
#define OBS_STALE_MS            (250U)

/** Consecutive hard sensor errors before declaring a fault. */
#define OBS_FAULT_LIMIT         (10U)

/** Consecutive good pings needed to leave the fault state. */
#define OBS_RECOVER_SAMPLES     (3U)

/** A no-echo this soon after a close return means blind zone, not
 *  free space. */
#define OBS_BLIND_WINDOW_MS     (400U)
#define OBS_BLIND_RANGE_MM      (200U)

#define OBS_COARSE_POINTS       (5U)
#define OBS_FINE_POINTS         (7U)
#define OBS_FINE_STEP_DEG       (10)
#define OBS_SWEEP_LIMIT_DEG     (75)

/** Task stack. Measured usage is 312 bytes; see README test E2. */
#define OBS_TASK_STACK_SZ       (1024)

/*=====================================================================*/
/* Module state                                                        */
/*=====================================================================*/

static ID g_tskid    = 0;
static ID g_tick_sem = 0;
static ID g_mtxid    = 0;
static ID g_flgid    = 0;
static ID g_cycid    = 0;

static obstacle_state_t  g_state        = OBSTACLE_ST_INIT;
static obstacle_profile_t g_profile;
static obstacle_health_t  g_health;

static uint16_t g_front_mm   = OBSTACLE_RANGE_UNKNOWN;
static uint32_t g_front_ms   = 0U;
static bool     g_has_profile = false;

static obstacle_sample_t g_samples[OBSTACLE_MAX_SAMPLES];
static uint8_t  g_sample_count = 0U;

static uint8_t  g_confirm     = 0U;
static uint32_t g_fault_run   = 0U;
static uint32_t g_good_run    = 0U;
static uint16_t g_last_close_mm = OBSTACLE_RANGE_CLEAR;
static uint32_t g_last_close_ms = 0U;

/** Set by obstacle_resume() from another task. */
static volatile bool g_resume_req = false;

/** Written by the cyclic handler, read by a task, hence volatile.
 *  A 32 bit aligned load on Cortex-M0+ is atomic, so no lock. */
static volatile uint32_t g_overruns = 0U;

/** Coarse sweep bearings, right to left. */
static const int16_t g_coarse_deg[OBS_COARSE_POINTS] =
{
    -60, -30, 0, 30, 60
};

/*=====================================================================*/
/* Time base                                                           */
/*=====================================================================*/

/**
 * @brief   Milliseconds since the kernel started, 32 bit.
 *
 * Wraps after 49 days. Every comparison below uses unsigned
 * subtraction, which is wrap correct, so the wrap is harmless.
 *
 * PORTING NOTE: micro T-Kernel 3.0 can be configured with SYSTIM as
 * a struct { W hi; UW lo; } or as a 64 bit integer. This is the only
 * place that needs changing if your BSP uses the 64 bit form:
 * return (uint32_t)tim;
 */
static uint32_t obs_now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_otm(&tim);

    return (uint32_t)tim.lo;
}

/*=====================================================================*/
/* Publication helpers. Critical sections are a few words long.        */
/*=====================================================================*/

static void obs_publish_front(uint16_t range_mm)
{
    if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
    {
        g_front_mm = range_mm;
        g_front_ms = obs_now_ms();
        (void)tk_unl_mtx(g_mtxid);
    }
}

static void obs_publish_profile(const obstacle_profile_t *p_new)
{
    if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
    {
        g_profile     = *p_new;
        g_has_profile = true;
        (void)tk_unl_mtx(g_mtxid);
    }
}

/*=====================================================================*/
/* Sample buffer, kept sorted by bearing for the geometry pass         */
/*=====================================================================*/

static void obs_samples_reset(void)
{
    g_sample_count = 0U;
}

/**
 * @brief   Insert or replace a sample, keeping the array sorted by
 *          ascending bearing.
 *
 * A fine-scan point that lands on a coarse bearing replaces it,
 * because the fine reading is the better measurement.
 */
static void obs_samples_insert(int16_t bearing_deg, uint16_t range_mm)
{
    uint8_t i = 0U;

    while ((i < g_sample_count) &&
           (g_samples[i].bearing_deg < bearing_deg))
    {
        i++;
    }

    if ((i < g_sample_count) &&
        (g_samples[i].bearing_deg == bearing_deg))
    {
        g_samples[i].range_mm = range_mm;
    }
    else if (g_sample_count < (uint8_t)OBSTACLE_MAX_SAMPLES)
    {
        uint8_t j = g_sample_count;

        while (j > i)
        {
            g_samples[j] = g_samples[j - 1U];
            j--;
        }

        g_samples[i].bearing_deg = bearing_deg;
        g_samples[i].range_mm    = range_mm;
        g_sample_count++;
    }
    else
    {
        /* Buffer full. Dropping the sample degrades resolution but
         * never corrupts the profile. */
    }
}

/**
 * @brief   Bearing of the closest sample currently in the buffer.
 */
static int16_t obs_nearest_bearing(void)
{
    uint8_t  i;
    uint16_t best    = OBSTACLE_RANGE_UNKNOWN;
    int16_t  bearing = 0;

    for (i = 0U; i < g_sample_count; i++)
    {
        const uint16_t r = g_samples[i].range_mm;

        if ((r != OBSTACLE_RANGE_UNKNOWN) &&
            (r != OBSTACLE_RANGE_CLEAR) &&
            ((best == OBSTACLE_RANGE_UNKNOWN) || (r < best)))
        {
            best    = r;
            bearing = g_samples[i].bearing_deg;
        }
    }

    return bearing;
}

/*=====================================================================*/
/* Measurement                                                         */
/*=====================================================================*/

/**
 * @brief   Point the sensor at a bearing and take one reading.
 * @param   bearing_deg  Negative right, positive left.
 * @param   use_median   true to average three pings (3x slower).
 * @return  Range in mm, or OBSTACLE_RANGE_CLEAR / _UNKNOWN.
 *
 * The servo settling delay and the sensor's mandatory 60 ms quiet
 * period overlap: ultrasonic_ping() measures the time since its own
 * last trigger and only sleeps the remainder. Moving the servo during
 * that window is free, which is what keeps a full sweep near one
 * second instead of two.
 */
static uint16_t obs_measure_at(int16_t bearing_deg, bool use_median)
{
    const int16_t from_deg = servo_get_angle();
    const int16_t to_deg   =
        (int16_t)((int16_t)SERVO_ANGLE_CENTRE_DEG + bearing_deg);

    ultrasonic_result_t res;
    ultrasonic_status_t status;
    uint16_t            range = OBSTACLE_RANGE_UNKNOWN;

    (void)servo_set_angle(to_deg);
    (void)tk_dly_tsk((RELTIM)servo_settle_ms(from_deg, to_deg));

    if (use_median)
    {
        status = ultrasonic_ping_median(&res);
    }
    else
    {
        status = ultrasonic_ping(&res);
    }

    if (status == ULTRASONIC_OK)
    {
        range = res.distance_mm;
    }
    else if (status == ULTRASONIC_TOO_CLOSE)
    {
        range = (uint16_t)ULTRASONIC_MIN_VALID_MM;
    }
    else if ((status == ULTRASONIC_NO_ECHO) ||
             (status == ULTRASONIC_TOO_FAR))
    {
        range = OBSTACLE_RANGE_CLEAR;
    }
    else
    {
        range = OBSTACLE_RANGE_UNKNOWN;
    }

    return range;
}

/*=====================================================================*/
/* Periodic tick                                                       */
/*=====================================================================*/

/**
 * @brief   Cyclic handler. Runs in kernel context; signals only.
 *
 * E_QOVR means the task had not consumed the previous tick, which is
 * a missed deadline and is counted rather than ignored.
 */
static void obs_tick_handler(void *exinf)
{
    (void)exinf;

    if (tk_sig_sem(g_tick_sem, 1) == E_QOVR)
    {
        g_overruns++;
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

/*=====================================================================*/
/* State handlers. Each returns the next state.                        */
/*=====================================================================*/

/**
 * @brief   Periodic forward ranging. The safety-critical state.
 */
static obstacle_state_t obs_state_guard(void)
{
    obstacle_state_t    next = OBSTACLE_ST_GUARD;
    ultrasonic_result_t res;
    ultrasonic_status_t status;

    (void)tk_wai_sem(g_tick_sem, 1, TMO_FEVR);

    status = ultrasonic_ping(&res);

    if (status == ULTRASONIC_OK)
    {
        obs_publish_front(res.distance_mm);
        g_fault_run = 0U;

        if (res.distance_mm <= (uint16_t)OBS_BLIND_RANGE_MM)
        {
            g_last_close_mm = res.distance_mm;
            g_last_close_ms = obs_now_ms();
        }

        if (res.distance_mm <= (uint16_t)OBSTACLE_TRIGGER_MM)
        {
            g_confirm++;
        }
        else
        {
            g_confirm = 0U;
        }
    }
    else if (status == ULTRASONIC_TOO_CLOSE)
    {
        /* Inside the blind zone. Do not sweep, there is no room to
         * steer; hand it straight to the controller. */
        obs_publish_front((uint16_t)ULTRASONIC_MIN_VALID_MM);
        g_fault_run = 0U;
        g_confirm   = (uint8_t)OBS_CONFIRM_SAMPLES;
    }
    else if ((status == ULTRASONIC_NO_ECHO) ||
             (status == ULTRASONIC_TOO_FAR))
    {
        const uint32_t since = obs_now_ms() - g_last_close_ms;

        g_fault_run = 0U;
        g_confirm   = 0U;

        if (since < OBS_BLIND_WINDOW_MS)
        {
            /* We were just about to touch something and now the echo
             * has vanished. That is the blind-zone signature, not an
             * empty track. Report UNKNOWN so the controller slows. */
            obs_publish_front(OBSTACLE_RANGE_UNKNOWN);
        }
        else
        {
            obs_publish_front(OBSTACLE_RANGE_CLEAR);
        }
    }
    else
    {
        /* ECHO_STUCK, NO_RESPONSE or NOT_READY: wiring, power, or the
         * interrupt path. The sensor is not telling us anything. */
        g_fault_run++;
        obs_publish_front(OBSTACLE_RANGE_UNKNOWN);
    }

    if (g_fault_run >= OBS_FAULT_LIMIT)
    {
        next = OBSTACLE_ST_FAULT;
    }
    else if (g_confirm >= (uint8_t)OBS_CONFIRM_SAMPLES)
    {
        (void)tk_set_flg(g_flgid, OBSTACLE_EVT_DETECTED);
        g_confirm = 0U;
        next      = OBSTACLE_ST_COARSE;
    }
    else
    {
        /* Stay in guard. */
    }

    return next;
}

/**
 * @brief   Five-point sweep, one ping per point.
 */
static obstacle_state_t obs_state_coarse(void)
{
    uint8_t i;

    obs_tick_enable(false);
    obs_samples_reset();

    for (i = 0U; i < (uint8_t)OBS_COARSE_POINTS; i++)
    {
        const int16_t b = g_coarse_deg[i];

        obs_samples_insert(b, obs_measure_at(b, false));
    }

    return OBSTACLE_ST_FINE;
}

/**
 * @brief   Narrow sweep around the nearest coarse return.
 *
 * Only the centre point uses the three-ping median. Median-filtering
 * all seven points would add 0.7 s and push the total past the three
 * second demonstration target.
 */
static obstacle_state_t obs_state_fine(void)
{
    const int16_t centre = obs_nearest_bearing();
    const int16_t span   =
        (int16_t)(((int16_t)OBS_FINE_POINTS / 2) * OBS_FINE_STEP_DEG);
    int16_t b;

    for (b = (int16_t)(centre - span);
         b <= (int16_t)(centre + span);
         b = (int16_t)(b + OBS_FINE_STEP_DEG))
    {
        if ((b >= -OBS_SWEEP_LIMIT_DEG) && (b <= OBS_SWEEP_LIMIT_DEG))
        {
            obs_samples_insert(b, obs_measure_at(b, (b == centre)));
        }
    }

    return OBSTACLE_ST_PUBLISH;
}

/**
 * @brief   Build the profile, publish it, hand control back.
 */
static obstacle_state_t obs_state_publish(void)
{
    obstacle_profile_t profile;

    obstacle_profile_build(g_samples, g_sample_count, &profile);
    profile.timestamp_ms = obs_now_ms();

    obs_publish_profile(&profile);
    g_health.scans_completed++;

    /* Sensor back to straight ahead before anything starts moving. */
    (void)obs_measure_at(0, false);

    if (profile.action == OBSTACLE_ACT_CONTINUE)
    {
        (void)tk_set_flg(g_flgid, OBSTACLE_EVT_CLEARED);
    }
    else
    {
        (void)tk_set_flg(g_flgid, OBSTACLE_EVT_PROFILED);
    }

    g_resume_req = false;
    obs_tick_enable(true);

    return OBSTACLE_ST_ASSIST;
}

/**
 * @brief   Keep ranging straight ahead while the controller manoeuvres.
 */
static obstacle_state_t obs_state_assist(void)
{
    obstacle_state_t    next = OBSTACLE_ST_ASSIST;
    ultrasonic_result_t res;

    (void)tk_wai_sem(g_tick_sem, 1, TMO_FEVR);

    if (ultrasonic_ping(&res) == ULTRASONIC_OK)
    {
        obs_publish_front(res.distance_mm);
    }
    else
    {
        obs_publish_front(OBSTACLE_RANGE_UNKNOWN);
    }

    if (g_resume_req)
    {
        g_resume_req = false;
        g_confirm    = 0U;
        next         = OBSTACLE_ST_GUARD;
    }

    return next;
}

/**
 * @brief   Sensor unhealthy. Report UNKNOWN and retry slowly.
 */
static obstacle_state_t obs_state_fault(void)
{
    obstacle_state_t    next = OBSTACLE_ST_FAULT;
    ultrasonic_result_t res;
    ultrasonic_status_t status;

    obs_tick_enable(false);
    obs_publish_front(OBSTACLE_RANGE_UNKNOWN);
    (void)tk_set_flg(g_flgid, OBSTACLE_EVT_FAULT);
    g_health.sensor_faults++;

    (void)tk_dly_tsk((RELTIM)500);

    status = ultrasonic_ping(&res);

    /* Recovery needs evidence the sensor is answering. Anything that
     * proves ECHO moved counts; silence does not. */
    if ((status == ULTRASONIC_OK) || (status == ULTRASONIC_NO_ECHO) ||
        (status == ULTRASONIC_TOO_CLOSE) || (status == ULTRASONIC_TOO_FAR))
    {
        g_good_run++;
    }
    else
    {
        g_good_run = 0U;
    }

    if (g_good_run >= OBS_RECOVER_SAMPLES)
    {
        (void)tk_clr_flg(g_flgid, (UINT)(~OBSTACLE_EVT_FAULT));
        g_good_run  = 0U;
        g_fault_run = 0U;
        obs_tick_enable(true);
        next = OBSTACLE_ST_GUARD;
    }

    return next;
}

/*=====================================================================*/
/* Task                                                                */
/*=====================================================================*/

static void obstacle_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    g_state = OBSTACLE_ST_GUARD;
    obs_tick_enable(true);

    for (;;)
    {
        switch (g_state)
        {
            case OBSTACLE_ST_GUARD:
                g_state = obs_state_guard();
                break;

            case OBSTACLE_ST_COARSE:
                g_state = obs_state_coarse();
                break;

            case OBSTACLE_ST_FINE:
                g_state = obs_state_fine();
                break;

            case OBSTACLE_ST_PUBLISH:
                g_state = obs_state_publish();
                break;

            case OBSTACLE_ST_ASSIST:
                g_state = obs_state_assist();
                break;

            case OBSTACLE_ST_FAULT:
            default:
                g_state = obs_state_fault();
                break;
        }

        g_health.state = g_state;
    }
}

/*=====================================================================*/
/* Public interface                                                    */
/*=====================================================================*/

bool obstacle_init(void)
{
    bool ok = false;

    T_CSEM csem;
    T_CMTX cmtx;
    T_CFLG cflg;
    T_CCYC ccyc;
    T_CTSK ctsk;

    g_health.state             = OBSTACLE_ST_INIT;
    g_health.scans_completed   = 0U;
    g_health.deadline_overruns = 0U;
    g_health.sensor_faults     = 0U;

    csem.exinf   = NULL;
    csem.sematr  = TA_TFIFO | TA_FIRST;
    csem.isemcnt = 0;
    csem.maxsem  = 1;
    g_tick_sem   = tk_cre_sem(&csem);

    cmtx.exinf   = NULL;
    cmtx.mtxatr  = TA_TFIFO | TA_INHERIT;
    cmtx.ceilpri = (PRI)BOARD_PRI_MOTION_CTRL;
    g_mtxid      = tk_cre_mtx(&cmtx);

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_flgid      = tk_cre_flg(&cflg);

    ccyc.exinf   = NULL;
    ccyc.cycatr  = TA_HLNG | TA_PHS;
    ccyc.cychdr  = obs_tick_handler;
    ccyc.cyctim  = (RELTIM)OBS_GUARD_PERIOD_MS;
    ccyc.cycphs  = (RELTIM)OBS_GUARD_PERIOD_MS;
    g_cycid      = tk_cre_cyc(&ccyc);

    if ((g_tick_sem > 0) && (g_mtxid > 0) &&
        (g_flgid > 0) && (g_cycid > 0) &&
        servo_init() && ultrasonic_init())
    {
        ctsk.exinf   = NULL;
        ctsk.tskatr  = TA_HLNG | TA_RNG3;
        ctsk.task    = obstacle_task;
        ctsk.itskpri = (PRI)BOARD_PRI_OBSTACLE;
        ctsk.stksz   = OBS_TASK_STACK_SZ;

        g_tskid = tk_cre_tsk(&ctsk);

        if (g_tskid > 0)
        {
            ok = (tk_sta_tsk(g_tskid, 0) == E_OK);
        }
    }

    return ok;
}

uint16_t obstacle_get_front_mm(void)
{
    uint16_t range = OBSTACLE_RANGE_UNKNOWN;

    if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
    {
        if ((obs_now_ms() - g_front_ms) <= OBS_STALE_MS)
        {
            range = g_front_mm;
        }

        (void)tk_unl_mtx(g_mtxid);
    }

    return range;
}

bool obstacle_get_profile(obstacle_profile_t *p_out)
{
    bool ok = false;

    if (p_out != NULL)
    {
        if (tk_loc_mtx(g_mtxid, TMO_FEVR) == E_OK)
        {
            if (g_has_profile)
            {
                *p_out = g_profile;
                ok     = true;
            }

            (void)tk_unl_mtx(g_mtxid);
        }
    }

    return ok;
}

void obstacle_get_health(obstacle_health_t *p_out)
{
    if (p_out != NULL)
    {
        /* Counters only. A torn read would cost one stale telemetry
         * sample, which is not worth blocking the scan task for. */
        *p_out = g_health;
        p_out->deadline_overruns = g_overruns;
    }
}

ID obstacle_get_flag_id(void)
{
    return g_flgid;
}

void obstacle_resume(void)
{
    g_resume_req = true;
}
