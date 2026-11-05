/* Buddy 3 / Codex, 2026-11-05: exercise the real controller with lost,
 * healthy and intermittent encoder feedback. All motor writes are mocked. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motion_control.c"

static uint32_t clock_ms = 1000U;
static uint32_t ticks[2];
static int32_t outputs[2];
static uint32_t estop_bits;
static bool hardware_ok = true;

bool motion_hw_init(void) { return hardware_ok; }
uint32_t motion_hw_ticks(uint32_t side) { return ticks[side]; }
uint32_t motion_hw_time_us(void) { return clock_ms * 1000U; }
void motion_hw_set_encoder_min_edge_ticks(uint32_t value) { (void)value; }
void motion_hw_motor_set(uint32_t side, int32_t duty) { outputs[side] = duty; }
ER tk_get_otm(SYSTIM *time) { time->hi = 0U; time->lo = clock_ms; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *flag) { (void)id; flag->flgptn = estop_bits; return E_OK; }
ER tk_set_flg(ID id, UINT value) { (void)id; (void)value; return E_OK; }
ER tk_clr_flg(ID id, UINT mask) { (void)id; estop_bits &= mask; return E_OK; }
INT tk_rcv_mbf(ID id, void *message, INT timeout) { (void)id; (void)message; (void)timeout; return E_TMOUT; }
ER tk_snd_mbf(ID id, const void *message, INT size, INT timeout) { (void)id; (void)message; (void)size; (void)timeout; return E_OK; }
ER tk_loc_mtx(ID id, INT timeout) { (void)id; (void)timeout; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ER tk_slp_tsk(INT timeout) { (void)timeout; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
ID tk_cre_mtx(const T_CMTX *p) { (void)p; return 1; }
ID tk_cre_mbf(const T_CMBF *p) { (void)p; return 2; }
ID tk_cre_flg(const T_CFLG *p) { (void)p; return 3; }
ID tk_cre_tsk(const T_CTSK *p) { (void)p; return 4; }
ID tk_cre_cyc(const T_CCYC *p) { (void)p; return 5; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
ER tk_sta_cyc(ID id) { (void)id; return E_OK; }

static void reset_control(void)
{
    mc_cut_outputs();
    s_state = MOTION_STATE_IDLE;
    s_fault = MOTION_FAULT_NONE;
    s_result = MOTION_RESULT_NONE;
    s_pending_evt = 0U;
    s_estop_src = 0U;
    s_estop_flg = 3;
    s_status_mtx = 1;
    s_cmd_mbf = 2;
    s_t_last_ms = clock_ms;
    s_edge_clock_valid = false;
    s_hist_idx = 0U;
    estop_bits = 0U;
    memset(ticks, 0, sizeof(ticks));
    memset(s_raw_last, 0, sizeof(s_raw_last));
    memset(s_delta, 0, sizeof(s_delta));
    memset(s_pos_ticks, 0, sizeof(s_pos_ticks));
    memset(s_hist, 0, sizeof(s_hist));
    memset(s_speed, 0, sizeof(s_speed));
    s_dir[0] = s_dir[1] = 1;
    for (unsigned i = 0U; i < MC_HIST_LEN; ++i) s_hist_t_ms[i] = clock_ms;
}

static void cycle(uint32_t left, uint32_t right)
{
    clock_ms += MOTION_CONTROL_PERIOD_MS;
    ticks[0] += left;
    ticks[1] += right;
    mc_control_cycle();
}

static void test_missing_feedback(unsigned bad_side, int velocity)
{
    reset_control();
    mc_start_velocity(velocity, velocity);
    unsigned elapsed = 0U;
    while (!mc_latched() && elapsed <= MOTION_STALL_TIMEOUT_MS + 2U * MOTION_CONTROL_PERIOD_MS)
    {
        /* The MQTT VC refreshes commands; this must not restart the timer. */
        if ((elapsed % 100U) == 0U) mc_start_velocity(velocity, velocity);
        cycle((bad_side == 0U) ? 0U : 5U, (bad_side == 1U) ? 0U : 5U);
        elapsed += MOTION_CONTROL_PERIOD_MS;
        assert(outputs[bad_side] <= MOTION_NO_FEEDBACK_MAX_PWM_PM);
        assert(outputs[bad_side] >= -MOTION_NO_FEEDBACK_MAX_PWM_PM);
        assert(s_integ[bad_side] == 0.0f);
    }
    assert(s_fault == ((bad_side == 0U) ? MOTION_FAULT_STALL_L : MOTION_FAULT_STALL_R));
    assert(elapsed <= MOTION_STALL_TIMEOUT_MS + MOTION_CONTROL_PERIOD_MS);
    assert(outputs[0] == 0 && outputs[1] == 0);
    motion_cmd_msg_t refresh = {MOTION_CMD_SET_SPEED, velocity, velocity};
    mc_handle_cmd(&refresh);
    assert(s_state == MOTION_STATE_FAULT);
    cycle(0U, 0U);
    assert(outputs[0] == 0 && outputs[1] == 0);
    motion_cmd_msg_t reset = {MOTION_CMD_RESET, 0, 0};
    mc_handle_cmd(&reset);
    assert(s_state == MOTION_STATE_IDLE && s_fault == MOTION_FAULT_NONE);
    assert(outputs[0] == 0 && outputs[1] == 0);
}

int main(void)
{
    hardware_ok = false;
    assert(!motion_init());
    hardware_ok = true;
    assert(motion_init());
    test_missing_feedback(1U, 40);
    test_missing_feedback(0U, 40);
    test_missing_feedback(1U, -40);
    test_missing_feedback(1U, 300);
    puts("PASS S2-HOST-02/03: lost feedback cuts both wheels within 320 ms, caps power and freezes integral");

    reset_control();
    mc_start_velocity(40, 40);
    for (unsigned i = 0U; i < 100U; ++i) cycle(5U, 5U);
    assert(!mc_latched());
    /* An intermittent quiet cycle must not latch a fault. */
    for (unsigned i = 0U; i < 100U; ++i) cycle((i & 1U) ? 2U : 0U, (i & 1U) ? 2U : 0U);
    assert(!mc_latched());
    reset_control();
    mc_start_velocity(40, 0);
    for (unsigned i = 0U; i < 100U; ++i) cycle(5U, 0U);
    assert(!mc_latched() && outputs[1] == 0);
    puts("PASS S2-HOST-04: healthy/intermittent feedback and a stationary pivot wheel remain usable");

    reset_control();
    mc_start_velocity(40, 40);
    for (unsigned i = 0U; i < 8U; ++i) cycle(5U, 0U);
    assert(s_stall_ms[1] > 0U);
    cycle(5U, 5U);
    assert(s_stall_ms[1] == 0U);
    for (unsigned i = 0U; i < 100U; ++i) cycle(5U, 5U);
    assert(!mc_latched());
    /* A dropout after healthy running must cap existing integral power too. */
    s_integ[1] = 250.0f;
    cycle(5U, 0U);
    assert(outputs[1] <= MOTION_NO_FEEDBACK_MAX_PWM_PM && s_integ[1] == 250.0f);
    estop_bits = MOTION_ESTOP_VEHICLE;
    cycle(5U, 0U);
    assert(s_state == MOTION_STATE_ESTOP && outputs[0] == 0 && outputs[1] == 0);
    puts("PASS S2-HOST-05: recovery clears the timer; faults latch; E-stop still cuts immediately");
    return 0;
}
