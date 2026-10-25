/*
 * motion_demo.c - mock VehicleController for bench-testing MotionControl (v2).
 *
 * Compiled to NOTHING unless MOTION_DEMO is 1 (see motion_control.h).
 * After 8 s it runs a slow, short scripted sequence and prints named states:
 *   forward 300 mm @100, turn left 90, forward 200, turn right 90,
 *   backward 200, then an E-stop test (E-stop 1 s into a 500 mm move).
 * It STOPS at the first step that does not finish with DONE.
 *
 * SAFETY: first run with the wheels lifted. Press GP20 at any time = E-stop.
 * Needs about 1 m of clear floor.
 */
#include <stdint.h>
#include <stdbool.h>
#include "motion_config.h"
#include "motion_control.h"

#if MOTION_DEMO

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#define DEMO_TASK_PRIORITY  (30)
#define DEMO_TASK_STACK     (4096U)
#define DEMO_GPIO_IN        (*(volatile uint32_t *)(0xD0000004U))
#define DEMO_GPIO20_CTRL    (*(volatile uint32_t *)(0x400140A4U))   /* IO_BANK0 GPIO20_CTRL */
#define DEMO_BUTTON_PIN     (20U)

static ID s_demo_task_id;

static const char *demo_state_name(motion_state_t s)
{
    switch (s)
    {
    case MOTION_STATE_INIT:           return "INIT";
    case MOTION_STATE_IDLE:           return "IDLE";
    case MOTION_STATE_DRIVING:        return "DRIVING";
    case MOTION_STATE_LINE_FOLLOWING: return "LINE_FOLLOWING";
    case MOTION_STATE_TURNING:        return "TURNING";
    case MOTION_STATE_STOPPING:       return "STOPPING";
    case MOTION_STATE_ESTOP:          return "ESTOP";
    case MOTION_STATE_FAULT:          return "FAULT";
    default:                          return "?";
    }
}

static const char *demo_result_name(motion_result_t r)
{
    switch (r)
    {
    case MOTION_RESULT_NONE:     return "NONE";
    case MOTION_RESULT_DONE:     return "DONE";
    case MOTION_RESULT_STOPPED:  return "STOPPED";
    case MOTION_RESULT_FAULT:    return "FAULT";
    case MOTION_RESULT_ESTOP:    return "ESTOP";
    case MOTION_RESULT_REJECTED: return "REJECTED";
    default:                     return "?";
    }
}

static const char *demo_fault_name(motion_fault_t f)
{
    switch (f)
    {
    case MOTION_FAULT_NONE:             return "none";
    case MOTION_FAULT_STALL_L:          return "STALL_L";
    case MOTION_FAULT_STALL_R:          return "STALL_R";
    case MOTION_FAULT_ENCODER_TIMEOUT:  return "ENCODER_TIMEOUT";
    case MOTION_FAULT_LINE_STALE:       return "LINE_STALE";
    default:                            return "?";
    }
}

static bool demo_button_pressed(void)
{
    return (((DEMO_GPIO_IN >> DEMO_BUTTON_PIN) & 1U) == 0U);       /* active low */
}

/* Wait (real time, from the status time stamp) until the command has finished.
 * Returns the final result; on timeout it asks for a stop. GP20 = E-stop. */
static motion_result_t demo_wait_done(uint32_t timeout_ms)
{
    motion_status_t st;
    uint32_t t0;
    uint32_t last_print;
    bool estop_sent = false;

    (void)tk_dly_tsk(60);                       /* let MotionControl pick up the command */
    motion_get_status(&st);
    t0 = st.timestamp_ms;
    last_print = t0;

    for (;;)
    {
        motion_get_status(&st);
        if (demo_button_pressed() && !estop_sent)
        {
            motion_estop_request(MOTION_ESTOP_VEHICLE);
            estop_sent = true;
            tm_printf((UB *)"  GP20 pressed: E-STOP\n");
        }
        if ((st.timestamp_ms - last_print) >= 200U)
        {
            last_print = st.timestamp_ms;
            tm_printf((UB *)"  t=%u ms %s vL %d vR %d mm/s dist %d mm heading %d deg\n",
                      (unsigned int)(st.timestamp_ms - t0), demo_state_name(st.state),
                      (int)st.speed_l_mmps, (int)st.speed_r_mmps,
                      (int)st.distance_total_mm, (int)st.heading_deg);
        }
        if ((st.state == MOTION_STATE_IDLE) && (st.last_result != MOTION_RESULT_NONE))
        {
            break;
        }
        if ((st.state == MOTION_STATE_ESTOP) || (st.state == MOTION_STATE_FAULT))
        {
            break;
        }
        if ((st.timestamp_ms - t0) > timeout_ms)
        {
            tm_printf((UB *)"  TIMEOUT: asking for a stop\n");
            (void)motion_stop();
            (void)tk_dly_tsk(1500);
            motion_get_status(&st);
            break;
        }
        (void)tk_dly_tsk(20);
    }

    tm_printf((UB *)"  -> %s, result %s, fault %s, dist %d mm (L %d, R %d), heading %d deg, overruns %u\n",
              demo_state_name(st.state), demo_result_name(st.last_result), demo_fault_name(st.fault),
              (int)st.distance_total_mm, (int)st.dist_l_mm, (int)st.dist_r_mm,
              (int)st.heading_deg, (unsigned int)st.overrun_count);
    return st.last_result;
}

static bool demo_step(const char *name, motion_err_t err, uint32_t timeout_ms)
{
    tm_printf((UB *)"demo: %s\n", name);
    if (err != MOTION_OK)
    {
        tm_printf((UB *)"  command not accepted (%d)\n", (int)err);
        return false;
    }
    return (demo_wait_done(timeout_ms) == MOTION_RESULT_DONE);
}

static void demo_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    DEMO_GPIO20_CTRL = 5U;                      /* SIO function: GP20 readable as a plain input */
    (void)tk_dly_tsk(8000);

    if (demo_step("1) forward 300 mm at 100 mm/s", motion_move(300, 100), 8000U) &&
        demo_step("2) turn left 90 deg", motion_turn(90, 90), 6000U) &&
        demo_step("3) forward 200 mm", motion_move(200, 100), 6000U) &&
        demo_step("4) turn right 90 deg", motion_turn(-90, 90), 6000U) &&
        demo_step("5) backward 200 mm", motion_move(-200, 100), 6000U))
    {
        motion_status_t st;

        tm_printf((UB *)"demo: 6) E-stop test: 500 mm move, E-stop after 1 s\n");
        (void)motion_move(500, 100);
        (void)tk_dly_tsk(1000);
        motion_estop_request(MOTION_ESTOP_VEHICLE);
        (void)demo_wait_done(2000U);
        tm_printf((UB *)"demo: clearing the E-stop and resetting\n");
        motion_estop_clear(MOTION_ESTOP_VEHICLE);
        (void)motion_reset();
        (void)tk_dly_tsk(100);
        motion_get_status(&st);
        tm_printf((UB *)"  -> %s, result %s\n", demo_state_name(st.state), demo_result_name(st.last_result));
        tm_printf((UB *)"demo: finished OK\n");
    }
    else
    {
        tm_printf((UB *)"demo: STOPPED at the first step that did not finish with DONE\n");
    }

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

void motion_demo_init(void)
{
    T_CTSK ctsk = {0};

    ctsk.tskatr  = TA_HLNG | TA_RNG0;
    ctsk.task    = demo_task;
    ctsk.itskpri = DEMO_TASK_PRIORITY;
    ctsk.stksz   = DEMO_TASK_STACK;
    s_demo_task_id = tk_cre_tsk(&ctsk);
}

void motion_demo_start(void)
{
    if (s_demo_task_id > 0)
    {
        (void)tk_sta_tsk(s_demo_task_id, 0);
    }
}

#endif /* MOTION_DEMO */
