#ifndef BUDDY3_H
#define BUDDY3_H

#include "b3_ir.h"
#include "b3_line.h"
#include <tk/tkernel.h>

typedef enum
{
    B3_NAV_NONE = 0,
    B3_NAV_LEFT,
    B3_NAV_RIGHT,
    B3_NAV_STRAIGHT,
    B3_NAV_UTURN
} b3_nav_command_t;

/* Controller supplies a fresh snapshot, ideally every 10-20 ms. */
typedef struct
{
    uint32_t time_us;
    bool follow_enabled;
    bool scan_enabled;
    bool stop_requested;
    bool motion_valid;
    int16_t left_measured_mm_s;
    int16_t right_measured_mm_s;
    bool imu_valid;
    int32_t yaw_rate_mdeg_s;
    bool terrain_unstable;
} b3_context_t;

typedef struct
{
    uint32_t sequence;
    b3_barcode_result_t barcode;
    b3_nav_command_t command;
    bool navigation_allowed;
} b3_nav_event_t;

typedef struct
{
    uint32_t time_us;
    b3_line_result_t line;
    b3_nav_event_t last_barcode;
    uint32_t edge_faults;
    uint32_t rejected_scans;
    uint32_t dropped_events;
    uint32_t max_service_gap_us;
    int32_t task_error;
    bool scan_active;
    bool junction_candidate;
    bool control_active;
    bool stop_required;
} b3_snapshot_t;

/* Existing micro T-Kernel must already be running. In standalone mode the
 * port installs IO_IRQ_BANK0. In an integrated robot, use the team's shared
 * GPIO dispatcher as described in b3_port.h. */
ER buddy3_start(b3_position_reader_t position_reader);
void buddy3_set_context(const b3_context_t *p_context);
void buddy3_get_snapshot(b3_snapshot_t *p_snapshot);
/* One consumer only: the vehicle controller. Telemetry reads the snapshot. */
bool buddy3_pop_navigation(b3_nav_event_t *p_event);
b3_nav_command_t buddy3_map_symbol(char symbol);

#if defined(APP_TEST) && APP_TEST
/* Test build only: restart the longest-service-gap measurement. */
void buddy3_test_reset_max_gap(void);
#endif

#endif
