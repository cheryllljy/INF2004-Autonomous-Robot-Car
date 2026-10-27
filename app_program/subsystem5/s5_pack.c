/**
 * @file    s5_pack.c
 * @brief   CR1 packing of the obstacle profile, health record and
 *          telemetry packet. Pure C, host tested.
 *
 * Every pack function builds the whole word in a local and returns it
 * by value or writes it once, so a packed word is never visible half
 * written.
 */

#include "obstacle_model.h"

#include <stddef.h>     /* NULL; this file never includes the kernel */

/*=====================================================================*/
/* Field positions: profile                                            */
/*=====================================================================*/

#define PF_NEAREST_SHIFT    (0U)
#define PF_WIDTH_SHIFT      (12U)
#define PF_BEARING_SHIFT    (24U)
#define PF_ANGLE_BITS       (8U)

#define PF_LEDGE_SHIFT      (0U)
#define PF_REDGE_SHIFT      (8U)
#define PF_GAPL_SHIFT       (16U)
#define PF_GAP_BITS         (11U)
#define PF_COUNT_SHIFT      (27U)
#define PF_COUNT_BITS       (5U)

#define PF_GAPR_SHIFT       (0U)
#define PF_SWINGL_SHIFT     (11U)
#define PF_CONF_SHIFT       (23U)
#define PF_CONF_BITS        (7U)
#define PF_VALID_SHIFT      (30U)

#define PF_SWINGR_SHIFT     (0U)
#define PF_ACTION_SHIFT     (12U)
#define PF_TIME_SHIFT       (15U)

/** Bearings -75..+75 are stored +75 so the field is unsigned. */
#define PF_ANGLE_OFFSET     (75)

S5_STATIC_ASSERT(PF_BEARING_SHIFT + PF_ANGLE_BITS == 32U, pf_w0_full);
S5_STATIC_ASSERT(PF_COUNT_SHIFT + PF_COUNT_BITS == 32U, pf_w1_full);
S5_STATIC_ASSERT(PF_VALID_SHIFT + 1U <= 32U, pf_w2_fits);
S5_STATIC_ASSERT(PF_TIME_SHIFT + S5_TIME_BITS == 32U, pf_w3_full);
S5_STATIC_ASSERT(OBSTACLE_GAP_OPEN_MM <= 2047U, gap_fits_11_bits);
S5_STATIC_ASSERT(OBSTACLE_MAX_SAMPLES <= 31U, count_fits_5_bits);

/*=====================================================================*/
/* Field positions: health                                             */
/*=====================================================================*/

#define HF_STATE_SHIFT      (0U)
#define HF_FAULT_SHIFT      (4U)
#define HF_ESTOP_SHIFT      (8U)
#define HF_HASPROF_SHIFT    (9U)
#define HF_ODO_SHIFT        (10U)
#define HF_REV_SHIFT        (11U)
#define HF_REV_BITS         (2U)
#define HF_STATUS_SHIFT     (13U)
#define HF_STATUS_BITS      (3U)
#define HF_OVR_SHIFT        (16U)
#define HF_SFLT_SHIFT       (24U)
#define HF_BYTE_BITS        (8U)

#define HF_SCANS_SHIFT      (0U)
#define HF_BUILD_SHIFT      (16U)
#define HF_HALF_BITS        (16U)

/*=====================================================================*/
/* Helpers                                                             */
/*=====================================================================*/

/** Signed angle -> unsigned 8-bit offset code, clamped to +/-75. */
static uint32_t pk_angle(int16_t deg)
{
    int32_t a = (int32_t)deg;

    if (a < -PF_ANGLE_OFFSET) { a = -PF_ANGLE_OFFSET; }
    if (a > PF_ANGLE_OFFSET)  { a = PF_ANGLE_OFFSET; }

    return (uint32_t)(a + PF_ANGLE_OFFSET);
}

static int16_t unpk_angle(uint32_t code)
{
    return (int16_t)((int32_t)code - PF_ANGLE_OFFSET);
}

static uint32_t pk_bool(bool b)
{
    return b ? 1U : 0U;
}

/*=====================================================================*/
/* Profile                                                             */
/*=====================================================================*/

void obstacle_profile_pack(const obstacle_profile_t *p_in,
                           obstacle_profile_packed_t *p_out)
{
    if ((p_in != NULL) && (p_out != NULL))
    {
        const uint32_t w0 =
            S5_PUT(s5_range_encode(p_in->nearest_mm),
                   PF_NEAREST_SHIFT, S5_RANGE_BITS) |
            S5_PUT(s5_sat_u(p_in->width_mm, S5_RANGE_BITS),
                   PF_WIDTH_SHIFT, S5_RANGE_BITS) |
            S5_PUT(pk_angle(p_in->bearing_deg),
                   PF_BEARING_SHIFT, PF_ANGLE_BITS);

        const uint32_t w1 =
            S5_PUT(pk_angle(p_in->left_edge_deg),
                   PF_LEDGE_SHIFT, PF_ANGLE_BITS) |
            S5_PUT(pk_angle(p_in->right_edge_deg),
                   PF_REDGE_SHIFT, PF_ANGLE_BITS) |
            S5_PUT(s5_sat_u(p_in->gap_left_mm, PF_GAP_BITS),
                   PF_GAPL_SHIFT, PF_GAP_BITS) |
            S5_PUT(s5_sat_u(p_in->sample_count, PF_COUNT_BITS),
                   PF_COUNT_SHIFT, PF_COUNT_BITS);

        const uint32_t w2 =
            S5_PUT(s5_sat_u(p_in->gap_right_mm, PF_GAP_BITS),
                   PF_GAPR_SHIFT, PF_GAP_BITS) |
            S5_PUT(s5_sat_u(p_in->swing_left_mm, S5_RANGE_BITS),
                   PF_SWINGL_SHIFT, S5_RANGE_BITS) |
            S5_PUT(s5_sat_u(p_in->confidence_pct, PF_CONF_BITS),
                   PF_CONF_SHIFT, PF_CONF_BITS) |
            S5_PUT(pk_bool(p_in->valid), PF_VALID_SHIFT, 1U);

        const uint32_t w3 =
            S5_PUT(s5_sat_u(p_in->swing_right_mm, S5_RANGE_BITS),
                   PF_SWINGR_SHIFT, S5_RANGE_BITS) |
            S5_PUT((uint32_t)p_in->action, PF_ACTION_SHIFT, S5_ACTION_BITS) |
            S5_PUT(s5_time_stamp(p_in->timestamp_ms),
                   PF_TIME_SHIFT, S5_TIME_BITS);

        p_out->w[0] = w0;
        p_out->w[1] = w1;
        p_out->w[2] = w2;
        p_out->w[3] = w3;
    }
}

void obstacle_profile_unpack(const obstacle_profile_packed_t *p_in,
                             obstacle_profile_t *p_out)
{
    if ((p_in != NULL) && (p_out != NULL))
    {
        const uint32_t w0 = p_in->w[0];
        const uint32_t w1 = p_in->w[1];
        const uint32_t w2 = p_in->w[2];
        const uint32_t w3 = p_in->w[3];
        uint32_t       act;

        p_out->nearest_mm     = s5_range_decode(
            S5_GET(w0, PF_NEAREST_SHIFT, S5_RANGE_BITS));
        p_out->width_mm       = (uint16_t)S5_GET(w0, PF_WIDTH_SHIFT,
                                                 S5_RANGE_BITS);
        p_out->bearing_deg    = unpk_angle(S5_GET(w0, PF_BEARING_SHIFT,
                                                  PF_ANGLE_BITS));
        p_out->left_edge_deg  = unpk_angle(S5_GET(w1, PF_LEDGE_SHIFT,
                                                  PF_ANGLE_BITS));
        p_out->right_edge_deg = unpk_angle(S5_GET(w1, PF_REDGE_SHIFT,
                                                  PF_ANGLE_BITS));
        p_out->gap_left_mm    = (uint16_t)S5_GET(w1, PF_GAPL_SHIFT,
                                                 PF_GAP_BITS);
        p_out->sample_count   = (uint8_t)S5_GET(w1, PF_COUNT_SHIFT,
                                                PF_COUNT_BITS);
        p_out->gap_right_mm   = (uint16_t)S5_GET(w2, PF_GAPR_SHIFT,
                                                 PF_GAP_BITS);
        p_out->swing_left_mm  = (uint16_t)S5_GET(w2, PF_SWINGL_SHIFT,
                                                 S5_RANGE_BITS);
        p_out->confidence_pct = (uint8_t)S5_GET(w2, PF_CONF_SHIFT,
                                                PF_CONF_BITS);
        p_out->valid          = (S5_GET(w2, PF_VALID_SHIFT, 1U) != 0U);
        p_out->swing_right_mm = (uint16_t)S5_GET(w3, PF_SWINGR_SHIFT,
                                                 S5_RANGE_BITS);
        p_out->timestamp_ms   = S5_GET(w3, PF_TIME_SHIFT, S5_TIME_BITS);

        /* A corrupted action code must never decode as a manoeuvre. */
        act = S5_GET(w3, PF_ACTION_SHIFT, S5_ACTION_BITS);
        p_out->action = (act < (uint32_t)OBSTACLE_ACT_COUNT) ?
                        (obstacle_action_t)act : OBSTACLE_ACT_STOP;
    }
}

/*=====================================================================*/
/* Health                                                              */
/*=====================================================================*/

void obstacle_health_pack(const obstacle_health_t *p_in,
                          obstacle_health_packed_t *p_out)
{
    if ((p_in != NULL) && (p_out != NULL))
    {
        const uint32_t w0 =
            S5_PUT((uint32_t)p_in->state, HF_STATE_SHIFT, S5_STATE_BITS) |
            S5_PUT((uint32_t)p_in->fault, HF_FAULT_SHIFT, S5_FAULT_BITS) |
            S5_PUT(pk_bool(p_in->estop_requested), HF_ESTOP_SHIFT, 1U) |
            S5_PUT(pk_bool(p_in->has_profile), HF_HASPROF_SHIFT, 1U) |
            S5_PUT(pk_bool(p_in->odometry_ok), HF_ODO_SHIFT, 1U) |
            S5_PUT(s5_sat_u(p_in->reverses, HF_REV_BITS),
                   HF_REV_SHIFT, HF_REV_BITS) |
            S5_PUT(p_in->last_status, HF_STATUS_SHIFT, HF_STATUS_BITS) |
            S5_PUT(p_in->deadline_overruns, HF_OVR_SHIFT, HF_BYTE_BITS) |
            S5_PUT(p_in->sensor_faults, HF_SFLT_SHIFT, HF_BYTE_BITS);

        const uint32_t w1 =
            S5_PUT(p_in->scans_completed, HF_SCANS_SHIFT, HF_HALF_BITS) |
            S5_PUT(p_in->build_us_max, HF_BUILD_SHIFT, HF_HALF_BITS);

        p_out->w[0] = w0;
        p_out->w[1] = w1;
    }
}

void obstacle_health_unpack(const obstacle_health_packed_t *p_in,
                            obstacle_health_t *p_out)
{
    if ((p_in != NULL) && (p_out != NULL))
    {
        const uint32_t w0 = p_in->w[0];
        const uint32_t w1 = p_in->w[1];
        const uint32_t st = S5_GET(w0, HF_STATE_SHIFT, S5_STATE_BITS);
        const uint32_t ft = S5_GET(w0, HF_FAULT_SHIFT, S5_FAULT_BITS);

        p_out->state = (st < (uint32_t)OBSTACLE_ST_COUNT) ?
                       (obstacle_state_t)st : OBSTACLE_ST_FAULT;
        p_out->fault = (ft < (uint32_t)OBSTACLE_FAULT_COUNT) ?
                       (obstacle_fault_t)ft : OBSTACLE_FAULT_SENSOR;
        p_out->estop_requested   = (S5_GET(w0, HF_ESTOP_SHIFT, 1U) != 0U);
        p_out->has_profile       = (S5_GET(w0, HF_HASPROF_SHIFT, 1U) != 0U);
        p_out->odometry_ok       = (S5_GET(w0, HF_ODO_SHIFT, 1U) != 0U);
        p_out->reverses          = (uint8_t)S5_GET(w0, HF_REV_SHIFT,
                                                   HF_REV_BITS);
        p_out->last_status       = (uint8_t)S5_GET(w0, HF_STATUS_SHIFT,
                                                   HF_STATUS_BITS);
        p_out->deadline_overruns = (uint8_t)S5_GET(w0, HF_OVR_SHIFT,
                                                   HF_BYTE_BITS);
        p_out->sensor_faults     = (uint8_t)S5_GET(w0, HF_SFLT_SHIFT,
                                                   HF_BYTE_BITS);
        p_out->scans_completed   = (uint16_t)S5_GET(w1, HF_SCANS_SHIFT,
                                                    HF_HALF_BITS);
        p_out->build_us_max      = (uint16_t)S5_GET(w1, HF_BUILD_SHIFT,
                                                    HF_HALF_BITS);
    }
}

/*=====================================================================*/
/* Telemetry                                                           */
/*=====================================================================*/

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

uint32_t s5_tlm_encode(uint8_t *p_buf, uint32_t len,
                     s5_front_t front,
                     const obstacle_health_packed_t *p_health,
                     const obstacle_profile_packed_t *p_profile)
{
    uint32_t written = 0U;

    if ((p_buf != NULL) && (len >= S5_TLM_LEN) &&
        (p_health != NULL) && (p_profile != NULL))
    {
        uint8_t i;

        /* Byte order is written out explicitly, so the packet is the
         * same whatever the compiler or the dashboard's CPU. */
        p_buf[0] = (uint8_t)((S5_TLM_VERSION << 4) | S5_TLM_TYPE_OBSTACLE);
        put_le32(&p_buf[1], front);
        put_le32(&p_buf[5], p_health->w[0]);
        put_le32(&p_buf[9], p_health->w[1]);

        for (i = 0U; i < 4U; i++)
        {
            put_le32(&p_buf[13U + (4U * i)], p_profile->w[i]);
        }

        written = S5_TLM_LEN;
    }

    return written;
}

bool s5_tlm_decode(const uint8_t *p_buf, uint32_t len,
                   s5_front_t *p_front,
                   obstacle_health_packed_t *p_health,
                   obstacle_profile_packed_t *p_profile)
{
    bool ok = false;

    if ((p_buf != NULL) && (len >= S5_TLM_LEN) &&
        (p_front != NULL) && (p_health != NULL) && (p_profile != NULL) &&
        (p_buf[0] == (uint8_t)((S5_TLM_VERSION << 4) |
                               S5_TLM_TYPE_OBSTACLE)))
    {
        uint8_t i;

        *p_front       = get_le32(&p_buf[1]);
        p_health->w[0] = get_le32(&p_buf[5]);
        p_health->w[1] = get_le32(&p_buf[9]);

        for (i = 0U; i < 4U; i++)
        {
            p_profile->w[i] = get_le32(&p_buf[13U + (4U * i)]);
        }

        ok = true;
    }

    return ok;
}
