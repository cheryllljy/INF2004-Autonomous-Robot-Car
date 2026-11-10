/*
 *      task_monitor.c
 *      Subsystem 4 -- the monitor task and its alarm handler.
 *
 *      APIs shown: tk_sta_alm, tk_stp_alm, alarm handler, tk_wai_flg,
 *                  tk_rcv_mbf, tk_loc_mtx/tk_unl_mtx, tk_ref_sem, tk_ref_mpf,
 *                  tk_ref_tsk, tk_sus_tsk, tk_rsm_tsk, tk_chg_pri,
 *                  tk_get_tid, tk_get_tim
 */

#include "subsystem4.h"
#include "../subsystem2/subsystem2.h" /* sem_slots, mpf_samples (read only) */
#include "../subsystem3/subsystem3.h" /* IF-04: app_stats, mtx_stats */

UW s4_lines_received;

typedef struct
{
    int64_t sum_lateral;
    int64_t sum_vertical;
    uint64_t sum_total;
    uint8_t cal_samples;
    int32_t base_lateral;
    int32_t base_vertical;
    uint32_t base_total;
    int32_t filtered_sin_q15;
    uint8_t level_samples;
    uint8_t hump_sequence;
    uint8_t peak_height_mm;
    uint16_t hump_sample_count;
    uint32_t hump_started_ms;
    int32_t peak_roll_sign;
    bool calibrated;
    bool filter_ready;
    bool hump_active;
    bool hump_start_time_valid;
} s4_hump_state_t;

static s4_hump_state_t s4_hump_state;

static uint32_t s4_isqrt(uint64_t value)
{
    uint64_t result = 0U;
    uint64_t bit = 1ULL << 62;

    while (bit > value)
    {
        bit >>= 2;
    }
    while (0U != bit)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)result;
}

static int32_t s4_axis_value(const imu_vector_t *p_sample, uint32_t axis)
{
    switch (axis)
    {
        case 0U:
            return p_sample->x;
        case 1U:
            return p_sample->y;
        default:
            return p_sample->z;
    }
}

static uint32_t s4_abs_i32(int32_t value)
{
    return (uint32_t)((value < 0) ? -value : value);
}

static void s4_hump_step(const imu_vector_t *p_raw)
{
    int32_t lateral = s4_axis_value(p_raw, S4_HUMP_LATERAL_AXIS);
    int32_t vertical = s4_axis_value(p_raw, S4_HUMP_VERTICAL_AXIS);
    uint64_t total_sq = (uint64_t)((int64_t)p_raw->x * p_raw->x) +
                        (uint64_t)((int64_t)p_raw->y * p_raw->y) +
                        (uint64_t)((int64_t)p_raw->z * p_raw->z);
    uint32_t total = s4_isqrt(total_sq);
    uint32_t plane = s4_isqrt((uint64_t)((int64_t)lateral * lateral) +
                              (uint64_t)((int64_t)vertical * vertical));
    uint32_t base_plane;
    uint64_t denominator;
    int64_t cross;
    int32_t sin_q15;
    uint32_t height_mm;
    SYSTIM now;
    ER time_er;

    if (!s4_hump_state.calibrated)
    {
        s4_hump_state.sum_lateral += lateral;
        s4_hump_state.sum_vertical += vertical;
        s4_hump_state.sum_total += total;
        if (++s4_hump_state.cal_samples >= S4_HUMP_CAL_SAMPLES)
        {
            s4_hump_state.base_lateral =
                (int32_t)(s4_hump_state.sum_lateral / S4_HUMP_CAL_SAMPLES);
            s4_hump_state.base_vertical =
                (int32_t)(s4_hump_state.sum_vertical / S4_HUMP_CAL_SAMPLES);
            s4_hump_state.base_total =
                (uint32_t)(s4_hump_state.sum_total / S4_HUMP_CAL_SAMPLES);
            s4_hump_state.calibrated = (0U != s4_hump_state.base_total);
            tm_printf((UB *)"S4: roll baseline %s; axes L=%u V=%u\n",
                      s4_hump_state.calibrated ? "ready" : "invalid",
                      (UINT)S4_HUMP_LATERAL_AXIS, (UINT)S4_HUMP_VERTICAL_AXIS);
            if (!s4_hump_state.calibrated)
            {
                s4_hump_state.sum_lateral = 0;
                s4_hump_state.sum_vertical = 0;
                s4_hump_state.sum_total = 0U;
                s4_hump_state.cal_samples = 0U;
            }
        }
        return;
    }

    if ((0U == plane) || (0U == s4_hump_state.base_total) ||
        ((uint64_t)s4_abs_i32((int32_t)total -
                              (int32_t)s4_hump_state.base_total) *
             100U >
         (uint64_t)s4_hump_state.base_total * S4_HUMP_ACCEL_TOLERANCE_PCT))
    {
        return;
    }

    base_plane = s4_isqrt((uint64_t)((int64_t)s4_hump_state.base_lateral *
                                     s4_hump_state.base_lateral) +
                          (uint64_t)((int64_t)s4_hump_state.base_vertical *
                                     s4_hump_state.base_vertical));
    if (0U == base_plane)
    {
        return;
    }

    /* sin(delta roll) = cross(baseline, sample) / (|baseline|*|sample|). */
    cross = (int64_t)s4_hump_state.base_vertical * lateral -
            (int64_t)s4_hump_state.base_lateral * vertical;
    cross *= S4_HUMP_ROLL_SIGN;
    denominator = (uint64_t)base_plane * plane;
    sin_q15 = (int32_t)((cross * 32768) / (int64_t)denominator);
    if (sin_q15 > 32767)
    {
        sin_q15 = 32767;
    }
    else if (sin_q15 < -32768)
    {
        sin_q15 = -32768;
    }

    if (!s4_hump_state.filter_ready)
    {
        s4_hump_state.filtered_sin_q15 = sin_q15;
        s4_hump_state.filter_ready = true;
    }
    else
    {
        s4_hump_state.filtered_sin_q15 +=
            (sin_q15 - s4_hump_state.filtered_sin_q15) /
            (1 << S4_HUMP_FILTER_SHIFT);
    }

    /* Opposite wheel-side height = track width * |sin(roll)|. */
    height_mm =
        (S4_HUMP_TRACK_WIDTH_MM * s4_abs_i32(s4_hump_state.filtered_sin_q15) +
         16384U) /
        32768U;
    if (!s4_hump_state.hump_active)
    {
        if (height_mm < S4_HUMP_START_HEIGHT_MM)
        {
            return;
        }
        s4_hump_state.hump_active = true;
        s4_hump_state.peak_height_mm =
            (uint8_t)((height_mm > 255U) ? 255U : height_mm);
        s4_hump_state.peak_roll_sign = (sin_q15 < 0) ? -1 : 1;
        s4_hump_state.hump_sample_count = 0U;
        s4_hump_state.level_samples = 0U;
        time_er = tk_get_tim(&now);
        s4_hump_state.hump_start_time_valid = (time_er >= E_OK);
        if (s4_hump_state.hump_start_time_valid)
        {
            s4_hump_state.hump_started_ms = now.lo;
        }
        else
        {
            tm_printf((UB *)"[VC] terrain report: failed to timestamp "
                            "hump start, error=%d\n",
                      (INT)time_er);
        }
    }
    else if (height_mm > s4_hump_state.peak_height_mm)
    {
        s4_hump_state.peak_height_mm =
            (uint8_t)((height_mm > 255U) ? 255U : height_mm);
        s4_hump_state.peak_roll_sign = (sin_q15 < 0) ? -1 : 1;
    }

    if (s4_hump_state.hump_sample_count < UINT16_MAX)
    {
        s4_hump_state.hump_sample_count++;
    }

    if (height_mm <= S4_HUMP_LEVEL_HEIGHT_MM)
    {
        if (++s4_hump_state.level_samples >= S4_HUMP_LEVEL_SAMPLES)
        {
            if (s4_hump_state.peak_height_mm >= S4_HUMP_MIN_PEAK_HEIGHT_MM)
            {
                s4_hump_state.hump_sequence++;
                time_er = tk_get_tim(&now);
                if ((time_er >= E_OK) &&
                    s4_hump_state.hump_start_time_valid)
                {
                    tm_printf((UB *)"[VC] TERRAIN_REPORT seq=%u "
                            "classification=ROLL_HUMP "
                            "peak_side_height_est=%u mm "
                            "duration=%u ms imu_samples=%u\n",
                    (UINT)s4_hump_state.hump_sequence,
                    (UINT)s4_hump_state.peak_height_mm,
                    (UINT)(now.lo - s4_hump_state.hump_started_ms),
                    (UINT)s4_hump_state.hump_sample_count);
                }
                else
                {
                    if (time_er < E_OK)
                    {
                        tm_printf((UB *)"[VC] terrain report: failed to "
                                        "timestamp hump completion, error=%d\n",
                                  (INT)time_er);
                    }
                    tm_printf((UB *)"[VC] TERRAIN_REPORT seq=%u "
                                    "classification=ROLL_HUMP "
                                    "peak_side_height_est=%u mm "
                                    "duration=unavailable imu_samples=%u\n",
                              (UINT)s4_hump_state.hump_sequence,
                              (UINT)s4_hump_state.peak_height_mm,
                              (UINT)s4_hump_state.hump_sample_count);
                }
            }
            s4_hump_state.peak_height_mm = 0U;
            s4_hump_state.hump_sample_count = 0U;
            s4_hump_state.hump_start_time_valid = false;
            s4_hump_state.level_samples = 0U;
            s4_hump_state.hump_active = false;
        }
    }
    else
    {
        s4_hump_state.level_samples = 0U;
    }
}

bool subsystem4_on_hump(void)
{
    /* one aligned bool, written only by the monitor task */
    return s4_hump_state.hump_active;
}

/*
 * Alarm handler: one-shot, interrupt context (same rules as a cyclic
 * handler: keep it short, never wait).
 */
void monitor_alarm_handler(void *exinf)
{
    (void)exinf;
    tk_set_flg(flg_events, FLG_ALARM);
}

static ER publish_imu_sample(const imu_vector_t *p_accel,
                             imu_status_t accel_status,
                             const imu_vector_t *p_mag, imu_status_t mag_status)
{
    s4_imu_snapshot_t snapshot = { 0 };
    SYSTIM now;
    ER er;

    er = tk_get_tim(&now);
    if (er < E_OK)
    {
        tm_printf((UB *)"monitor: failed to timestamp IMU sample, error=%d\n",
                  (INT)er);
        return er;
    }
    if (NULL != p_accel)
    {
        snapshot.accel = *p_accel;
    }
    if (NULL != p_mag)
    {
        snapshot.mag = *p_mag;
    }
    snapshot.accel_status = accel_status;
    snapshot.mag_status = mag_status;
    snapshot.timestamp_ms = now.lo;
    snapshot.has_sample = true;

    er = subsystem4_publish_imu_snapshot(&snapshot);
    if (er < E_OK)
    {
        tm_printf((UB *)"monitor: failed to publish IMU snapshot, error=%d\n",
                  (INT)er);
    }
    return er;
}

void imu_task(INT stacd, void *exinf)
{
    imu_vector_t accel = { 0 };
    imu_vector_t accel_raw = { 0 };
    imu_vector_t mag = { 0 };
    imu_status_t init_status;
    imu_status_t accel_status;
    imu_status_t mag_status;
    INT consecutive_fails = 0;

    (void)stacd;
    (void)exinf;

    init_status = imu_init();
    if (IMU_OK == init_status)
    {
        tm_printf((UB *)"monitor: IMU ready at 0x%02X\n",
                  (UINT)imu_accel_address());
        imu_calibrate();
        s4_hump_state = (s4_hump_state_t){ 0 };
        tm_printf((UB *)"S4: keep car level and still for roll calibration\n");
    }
    else
    {
        tm_printf((UB *)"monitor: IMU init failed, status=%d\n",
                  (INT)init_status);
        (void)publish_imu_sample(NULL, init_status, NULL, init_status);
    }

    for (;;)
    {
        if (IMU_OK != init_status)
        {
            tk_dly_tsk(500);
            init_status = imu_reset();
            if (IMU_OK == init_status)
            {
                tm_printf((UB *)"monitor: IMU recovered at 0x%02X\n",
                          (UINT)imu_accel_address());
                imu_calibrate();
                s4_hump_state = (s4_hump_state_t){ 0 };
                tm_printf((
                    UB *)"S4: keep car level and still for roll calibration\n");
                consecutive_fails = 0;
            }
            else
            {
                (void)publish_imu_sample(NULL, init_status, NULL, init_status);
                continue;
            }
        }

        accel_status = imu_read_accel_with_raw(&accel, &accel_raw);
        mag_status = imu_read_mag(&mag);
        if (IMU_OK == accel_status)
        {
            s4_hump_step(&accel_raw);
        }
        (void)publish_imu_sample(&accel, accel_status, &mag, mag_status);

        if ((IMU_OK == accel_status) && (IMU_OK == mag_status))
        {
            consecutive_fails = 0;
        }
        else if (++consecutive_fails >= 10)
        {
            tm_printf((UB *)"monitor: IMU read failures, resetting sensor\n");
            init_status = imu_reset();
            consecutive_fails = 0;
            if (IMU_OK == init_status)
            {
                imu_calibrate();
                s4_hump_state = (s4_hump_state_t){ 0 };
                tm_printf((
                    UB *)"S4: keep car level and still for roll calibration\n");
            }
        }

        tk_dly_tsk(S4_IMU_SAMPLE_MS);
    }
}

void monitor_task(INT stacd, void *exinf)
{
    UINT flgptn;
    UB text[S4_SUMMARY_MAX + 1];
    INT len;
    APP_STATS snap;
    T_RSEM rsem;
    T_RMPF rmpf;
    SYSTIM now;
    s4_imu_snapshot_t imu_snapshot;
    ER er;

    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"monitor: I am task %d\n", tk_get_tid());

    while (1)
    {
        /* 1. Arm the one-shot alarm, then wait for EITHER bit.
                      TWF_ORW = wake on any of the bits, TWF_CLR = clear them
                      on wake-up.  The extra timeout is just a safety net. */
        tk_sta_alm(alm_monitor, S4_ALARM_MS);
        er = tk_wai_flg(flg_events, FLG_ALARM | FLG_BATCH, TWF_ORW | TWF_CLR,
                        &flgptn, S4_ALARM_MS * 2);
        /* cancel it if a batch came first */
        tk_stp_alm(alm_monitor);

        if (E_TMOUT == er)
        {
            tm_printf((UB *)"monitor: nothing at all happened?\n");
            continue;
        }

        if (flgptn & FLG_ALARM)
        {
            tm_printf((UB *)"monitor: alarm -- no batch for %d ms\n",
                      S4_ALARM_MS);
        }

        /* 2. Drain the message buffer without waiting (TMO_POL).
                      tk_rcv_mbf returns the message length, or an error. */
        while ((len = tk_rcv_mbf(mbf_summary, text, TMO_POL)) > 0)
        {
            text[len] = '\0';
            s4_lines_received++;
            tm_printf((UB *)"monitor: batch  %s\n", text);
        }

        /* 3. Copy the stats under subsystem 3's mutex, print outside
                      it: hold a mutex for as short a time as possible. */
        tk_loc_mtx(mtx_stats, TMO_FEVR);
        snap = app_stats;
        tk_unl_mtx(mtx_stats);

        tk_get_tim(&now);
        tk_ref_sem(sem_slots, &rsem);
        tk_ref_mpf(mpf_samples, &rmpf);
        tm_printf((UB *)"monitor: t=%u ms  samples=%u  min=%d max=%d avg=%d"
                        "  free slots=%d  free blocks=%d\n",
                  now.lo, snap.received, snap.min, snap.max,
                  snap.received ? (INT)(snap.sum / (W)snap.received) : 0,
                  rsem.semcnt, (INT)rmpf.frbcnt);

        er = subsystem4_get_imu_snapshot(&imu_snapshot);
        if (er < E_OK)
        {
            tm_printf((UB *)"monitor: failed to copy IMU snapshot, error=%d\n",
                      (INT)er);
        }
        else if (imu_snapshot.has_sample)
        {
            if ((IMU_OK == imu_snapshot.accel_status) &&
                (IMU_OK == imu_snapshot.mag_status))
            {
                tm_printf((UB *)"monitor: IMU accel=(%d,%d,%d) "
                                "mag=(%d,%d,%d)\n",
                          (INT)imu_snapshot.accel.x, (INT)imu_snapshot.accel.y,
                          (INT)imu_snapshot.accel.z, (INT)imu_snapshot.mag.x,
                          (INT)imu_snapshot.mag.y, (INT)imu_snapshot.mag.z);
            }
            else
            {
                tm_printf((UB *)"monitor: IMU read failed, "
                                "accel=%d mag=%d\n",
                          (INT)imu_snapshot.accel_status,
                          (INT)imu_snapshot.mag_status);
            }
        }
    }
}
