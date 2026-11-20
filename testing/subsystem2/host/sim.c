/* Host test for Buddy 2's motion_control.c (Buddy 5, 2026-11-20).
 * Compiles the real file and drives it with a simple wheel model fitted to
 * Buddy 2's own calibration log (testing/subsystem2/logs/v6-Tests.txt):
 *   forward (A-driven) starts at 80-120 pm, stops below 60-80 pm,
 *   coast (both low) stops in about 100 ms (tau ~40 ms).
 * Build and run: make (in this folder)                                  */
#include <stdio.h>
#include <math.h>
#include "motion_control.c"

/* ---------- time + kernel stubs ---------- */
static uint32_t g_ms = 1000U;
ER tk_get_otm(SYSTIM *t) { t->hi = 0; t->lo = g_ms; return E_OK; }
ER tk_cre_tsk(const T_CTSK *p) { (void)p; return 1; }
ER tk_sta_tsk(ID id, INT s) { (void)id; (void)s; return E_OK; }
ER tk_cre_cyc(const T_CCYC *p) { (void)p; return 1; }
ER tk_sta_cyc(ID id) { (void)id; return E_OK; }
static UINT g_flg;
ER tk_cre_flg(const T_CFLG *p) { g_flg = p->iflgptn; return 1; }
ER tk_set_flg(ID id, UINT p) { (void)id; g_flg |= p; return E_OK; }
ER tk_clr_flg(ID id, UINT p) { (void)id; g_flg &= p; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *r) { (void)id; r->flgptn = g_flg; return E_OK; }
#define QN 4
static unsigned char g_q[QN][64];
static int g_qsz[QN], g_qh, g_qn;
ER tk_cre_mbf(const T_CMBF *p) { (void)p; return 1; }
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO t)
{
    (void)id; (void)t;
    if (g_qn >= QN) { return E_TMOUT; }
    memcpy(g_q[(g_qh + g_qn) % QN], m, (size_t)sz);
    g_qsz[(g_qh + g_qn) % QN] = sz;
    g_qn++;
    return E_OK;
}
INT tk_rcv_mbf(ID id, void *m, TMO t)
{
    int sz;
    (void)id; (void)t;
    if (0 == g_qn) { return E_TMOUT; }
    sz = g_qsz[g_qh];
    memcpy(m, g_q[g_qh], (size_t)sz);
    g_qh = (g_qh + 1) % QN;
    g_qn--;
    return sz;
}
ER tk_cre_mtx(const T_CMTX *p) { (void)p; return 1; }
ER tk_loc_mtx(ID id, TMO t) { (void)id; (void)t; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ER tk_slp_tsk(TMO t) { (void)t; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }

/* ---------- wheel model ---------- */
static double w_v[2];      /* mm/s, signed */
static double w_ticks[2];  /* edges counted, both directions */
static int32_t w_duty[2];
static double w_start[2] = { 100.0, 115.0 }; /* pm to get moving */
static double w_stop[2] = { 65.0, 75.0 };    /* pm below which it stops */
static double w_load[2] = { 0.93, 1.00 };    /* left a bit weaker */

bool motion_hw_init(void) { return true; }
void motion_hw_motor_set(uint32_t side, int32_t duty_pm) { w_duty[side] = duty_pm; }
uint32_t motion_hw_ticks(uint32_t side) { return (uint32_t)w_ticks[side]; }
void motion_hw_set_encoder_min_edge_ticks(uint32_t ticks) { (void)ticks; }
void motion_hw_irq_status(int32_t *a, int32_t *b, uint32_t *c, uint32_t *d)
{ (void)a; (void)b; (void)c; (void)d; }
uint32_t motion_hw_time_us(void) { return g_ms * 1000U; }

static void wheel_step_1ms(void)
{
    int s;
    for (s = 0; s < 2; s++)
    {
        const double d = fabs((double)w_duty[s]);
        const double sg = (w_duty[s] >= 0) ? 1.0 : -1.0;
        double target;
        double tau;
        if (0.0 == d)
        {
            target = 0.0;
            tau = 0.040;
        }
        else if ((fabs(w_v[s]) < 3.0) && (d < w_start[s]))
        {
            target = 0.0; /* static friction holds it */
            tau = 0.010;
        }
        else if (d < w_stop[s])
        {
            target = 0.0;
            tau = 0.040;
        }
        else
        {
            target = sg * (d - 50.0) * MM_PER_TICK * MOTOR_FF_TICKS_PER_PM *
                     w_load[s];
            tau = 0.060;
        }
        w_v[s] += (target - w_v[s]) * (0.001 / tau);
        if (fabs(w_v[s]) < 0.5 && 0.0 == target) { w_v[s] = 0.0; }
        w_ticks[s] += fabs(w_v[s]) * 0.001 / MM_PER_TICK;
    }
}

static void sim_reset(void)
{
    uint32_t side, i;
    memset(w_v, 0, sizeof(w_v));
    memset(w_ticks, 0, sizeof(w_ticks));
    memset(w_duty, 0, sizeof(w_duty));
    g_qn = 0; g_qh = 0; g_flg = 0;
    (void)motion_init();
    for (side = 0U; side < MC_SIDES; side++)
    {
        s_raw_last[side] = motion_hw_ticks(side);
        for (i = 0U; i < MC_HIST_LEN; i++)
        {
            s_hist[side][i] = 0;
            s_hist_t_ms[i] = g_ms;
        }
    }
    s_state = MOTION_STATE_IDLE;
}

/* one 20 ms control period */
static void sim_period(void)
{
    int k;
    for (k = 0; k < 20; k++) { g_ms++; wheel_step_1ms(); }
    mc_control_cycle();
}

/* ---------- scenarios ---------- */
static int g_kicks[2], g_neg[2];

static void run_ms(int ms, FILE *log)
{
    int t;
    for (t = 0; t < ms; t += 20)
    {
        sim_period();
        if (log) { fprintf(log, "%u %.1f %.1f %.1f %.1f %d %d %d\n", g_ms,
                           s_set[0], s_set[1], w_v[0], w_v[1],
                           (int)w_duty[0], (int)w_duty[1], (int)s_state); }
        if (w_duty[0] == MOTOR_BREAKAWAY_PM) { g_kicks[0]++; }
        if (w_duty[1] == MOTOR_BREAKAWAY_PM) { g_kicks[1]++; }
        if (w_duty[0] < 0) { g_neg[0]++; }
        if (w_duty[1] < 0) { g_neg[1]++; }
    }
}

/* time (ms) until wheel speed is within 10% of the step, -1 if never */
static int settle(int side, double from, double to, int max_ms)
{
    int t;
    for (t = 0; t < max_ms; t += 20)
    {
        sim_period();
        if (fabs(w_v[side] - to) <= 0.1 * fabs(to - from)) { return t + 20; }
    }
    return -1;
}

int main(void)
{
    int t1, t2;
    double sum, sq, n;
    int t;

    /* 1. steering step, the S-turn case: mean stays 100 */
    sim_reset();
    (void)motion_set_wheel_speeds(100, 100);
    run_ms(1500, NULL);
    (void)motion_set_wheel_speeds(40, 160);
    t1 = settle(0, 100, 40 * w_load[0] / 1.0, 1000);
    run_ms(500, NULL);
    (void)motion_set_wheel_speeds(160, 40);
    t2 = settle(1, 160, 40, 1000);
    printf("steer step 100/100 -> 40/160: inner wheel settles in %d ms\n", t1);
    printf("steer flip 40/160 -> 160/40: new inner wheel settles in %d ms\n", t2);

    /* 2. slow inner wheel: 20/60 for 2 s */
    sim_reset();
    memset(g_kicks, 0, sizeof(g_kicks));
    (void)motion_set_wheel_speeds(100, 100);
    run_ms(1000, NULL);
    (void)motion_set_wheel_speeds(20, 60);
    run_ms(600, NULL);
    memset(g_kicks, 0, sizeof(g_kicks));
    sum = sq = n = 0;
    for (t = 0; t < 2000; t += 20)
    {
        run_ms(20, NULL);
        sum += w_v[0]; sq += w_v[0] * w_v[0]; n++;
    }
    printf("slow wheel asked 20 mm/s: mean %.1f, swing (sd) %.1f mm/s, "
           "kicks %d in 2 s\n", sum / n, sqrt(sq / n - (sum / n) * (sum / n)),
           g_kicks[0]);

    /* 3. stop then go 50 ms later, sent once (like drive() between refreshes) */
    sim_reset();
    (void)motion_set_wheel_speeds(150, 150);
    run_ms(1000, NULL);
    (void)motion_set_wheel_speeds(0, 0);
    run_ms(60, NULL);
    (void)motion_set_wheel_speeds(150, 150);
    run_ms(20, NULL);
    printf("stop then go after 60 ms: go %s (state %d, result %d)\n",
           (MOTION_STATE_DRIVING == s_state) ? "accepted" : "REFUSED",
           (int)s_state, (int)s_result);

    /* 4. start from rest straight into a curve: no wheel may drive backwards */
    sim_reset();
    memset(g_neg, 0, sizeof(g_neg));
    (void)motion_set_wheel_speeds(31, 169);
    run_ms(600, NULL);
    printf("start 0 -> 31/169: backwards duty cycles L %d R %d, speeds now "
           "L %.0f R %.0f\n", g_neg[0], g_neg[1], w_v[0], w_v[1]);

    /* 5. regression: a 300 mm move and a 90 deg turn */
    sim_reset();
    (void)motion_move(300, 200);
    run_ms(4000, NULL);
    printf("move 300 mm: L %.1f R %.1f mm (model), state %d result %d\n",
           w_ticks[0] * MM_PER_TICK, w_ticks[1] * MM_PER_TICK, (int)s_state,
           (int)s_result);
    sim_reset();
    (void)motion_turn(90, 90);
    run_ms(4000, NULL);
    printf("turn 90 deg: L %.1f R %.1f mm arc (want %.1f), state %d result %d\n",
           w_ticks[0] * MM_PER_TICK, w_ticks[1] * MM_PER_TICK,
           M_PI / 2.0 * TRACK_WIDTH_MM / 2.0, (int)s_state, (int)s_result);

    /* 6. soft start still soft: 0 -> 200/200 */
    sim_reset();
    (void)motion_set_wheel_speeds(200, 200);
    run_ms(100, NULL);
    printf("soft start 0 -> 200: setpoint after 100 ms L %.0f R %.0f\n",
           s_set[0], s_set[1]);
    /* 7. vc_turn-style spin: must match the original exactly */
    sim_reset();
    (void)motion_set_wheel_speeds(-120, 120);
    run_ms(600, NULL);
    printf("spin -120/120 for 600 ms: L %.2f R %.2f mm, set L %.1f R %.1f\n",
           w_ticks[0] * MM_PER_TICK, w_ticks[1] * MM_PER_TICK, s_set[0], s_set[1]);

    /* 8. hard slow-down into a curve: no wheel may speed up */
    sim_reset();
    (void)motion_set_wheel_speeds(200, 200);
    run_ms(1500, NULL);
    {
        float maxr = 0.0f, prevr = s_set[1]; int up = 0, k;
        (void)motion_set_wheel_speeds(0, 100);
        for (k = 0; k < 40; k++)
        {
            run_ms(20, NULL);
            if (s_set[1] > prevr + 0.01f) { up++; }
            if (s_set[1] > maxr) { maxr = s_set[1]; }
            prevr = s_set[1];
        }
        printf("200/200 -> 0/100: right setpoint went up %d times (max %.0f), "
               "now L %.0f R %.0f\n", up, maxr, s_set[0], s_set[1]);
    }

    /* 9. (0,0) sent while a MOVE is finishing: result must stay DONE */
    sim_reset();
    (void)motion_move(100, 150);
    {
        int k;
        for (k = 0; k < 200 && MOTION_STATE_STOPPING != s_state; k++) { run_ms(20, NULL); }
    }
    (void)motion_set_wheel_speeds(0, 0);
    run_ms(1000, NULL);
    printf("(0,0) during the end of a move: state %d result %d (1 1 = idle, done)\n",
           (int)s_state, (int)s_result);

    /* 10. steering setpoint: cycles until the inner wheel setpoint is there */
    sim_reset();
    (void)motion_set_wheel_speeds(100, 100);
    run_ms(1500, NULL);
    (void)motion_set_wheel_speeds(40, 160);
    {
        int k;
        for (k = 1; k < 50; k++) { run_ms(20, NULL); if (fabsf(s_set[0] - 40.0f) < 1.0f) break; }
        printf("setpoint 100 -> 40 reached after %d cycles (%d ms)\n", k, k * 20);
    }
    return 0;
}
