/* HOST TEST ONLY. Compiled with -DOBSTACLE_HOST_TEST -Iv1_reference. */
#include "obstacle.h"
#include "ultrasonic.h"
#include "v1_adapter.h"

void v1_build(const int16_t *p_bearing, const uint16_t *p_range,
              uint8_t count, v1_out_t *p_out)
{
    obstacle_sample_t  s[OBSTACLE_MAX_SAMPLES];
    obstacle_profile_t p;
    uint8_t            i;

    for (i = 0U; i < count; i++)
    {
        s[i].bearing_deg = p_bearing[i];
        s[i].range_mm    = p_range[i];
    }

    obstacle_profile_build(s, count, &p);

    p_out->valid          = p.valid;
    p_out->nearest_mm     = p.nearest_mm;
    p_out->bearing_deg    = p.bearing_deg;
    p_out->width_mm       = p.width_mm;
    p_out->left_edge_deg  = p.left_edge_deg;
    p_out->right_edge_deg = p.right_edge_deg;
    p_out->gap_left_mm    = p.gap_left_mm;
    p_out->gap_right_mm   = p.gap_right_mm;
    p_out->swing_left_mm  = p.swing_left_mm;
    p_out->swing_right_mm = p.swing_right_mm;
    p_out->confidence_pct = p.confidence_pct;
    p_out->sample_count   = p.sample_count;
    p_out->action         = (int)p.action;
}

unsigned v1_sizeof_sample(void)    { return (unsigned)sizeof(obstacle_sample_t); }
unsigned v1_sizeof_profile(void)   { return (unsigned)sizeof(obstacle_profile_t); }
unsigned v1_sizeof_health(void)    { return (unsigned)sizeof(obstacle_health_t); }
unsigned v1_sizeof_us_result(void) { return (unsigned)sizeof(ultrasonic_result_t); }
unsigned v1_sizeof_us_stats(void)  { return (unsigned)sizeof(ultrasonic_stats_t); }
