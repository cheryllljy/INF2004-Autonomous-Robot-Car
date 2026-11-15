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

/* Serious Buddy 3 faults. Any of these latches
 * stop_required in the snapshot; the VehicleController then E-stops the
 * car (VC_FAULT_LINE). The first cause is kept for the log. */
typedef enum
{
    B3_FAULT_NONE = 0,
    B3_FAULT_EVENT_QUEUE, /* navigation event queue overflowed       */
    B3_FAULT_TASK,        /* sensor task: RTOS call failed, task ended */
    B3_FAULT_MBF_SEND,    /* consumer task: message buffer send failed */
    B3_FAULT_MBX_RECEIVE, /* consumer task: mailbox receive failed     */
    B3_FAULT_COUNT
} b3_fault_t;

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
    bool accel_valid;
    int16_t accel_x_lsb;
    int16_t accel_y_lsb;
    int16_t accel_z_lsb;
    bool imu_valid;
    int32_t yaw_rate_mdeg_s;
    bool terrain_unstable;
    /* line speed set by the VehicleController (it backs
     * up and reads barcodes slowly, vc_barcode.c). 0 = B3_BASE_SPEED_MM_S
     * with Buddy 3's own barcode slowdown. */
    int16_t base_speed_mm_s;
    /* the VehicleController drives the wheels itself
     * (junction turn, obstacle bypass, barcode back-up): line following
     * starts again from straight when it hands back (b3_line_hold()). */
    bool steer_hold;
} b3_context_t;

typedef struct
{
    uint32_t sequence;
    b3_barcode_result_t barcode;
    b3_nav_command_t command;
    bool navigation_allowed;
} b3_nav_event_t;

/* why a black patch under both line sensors was not a
 * junction (2026-11-08: B3_PATCH_SHORT = a sensor's run was short). */
typedef enum
{
    B3_PATCH_JUNCTION = 0, /* it was one */
    B3_PATCH_SHORT,        /* a run under B3_JUNCTION_MIN_PATCH_UM */
    B3_PATCH_SLOW,         /* mean speed under B3_JUNCTION_MIN_SPEED_MM_S */
    B3_PATCH_NOT_DRIVING,  /* stop requested, no motion data, stale */
    B3_PATCH_SCANNING,     /* a barcode scan was in progress under IR_B */
    B3_PATCH_WHY_COUNT
} b3_patch_why_t;

typedef struct
{
    uint32_t time_us;
    b3_line_result_t line;
    b3_nav_event_t last_barcode;
    uint32_t edge_faults;
    uint32_t rejected_scans;
    /* element duration that caused the last reject */
    uint32_t reject_element_us;
    uint8_t reject_reason; /* b3_reject_reason_t of the last reject */
    /* elements measured (of B3_BARCODE_RUNS) at the last reject */
    uint8_t reject_edges;
    /* widths (us) of the last "not Code 39" reject */
    uint32_t reject_widths[B3_BARCODE_RUNS];
    /* last "white before barcode too short" reject: the
     * white measured and the white needed, in encoder ticks (the mean of
     * both wheels) if reject_encoder, else in us. */
    uint32_t reject_lead;
    uint32_t reject_need;
    bool reject_encoder;
    uint32_t dropped_events;
    uint32_t max_service_gap_us;
    int32_t task_error;
    bool scan_active;
    /* elements measured so far in the scan under way
     * (0 when none); a crossing line gives at most 1 */
    uint8_t scan_elements;
    bool junction_candidate;
    /* +1 per junction reached; compare to detect a new one */
    uint32_t junction_sequence;
    /* last decoded letter, held until a junction ('\0' = none) */
    char held_symbol;
    /* letter handed over at junction_sequence ('\0' = none) */
    char junction_symbol;
    uint8_t fault_source; /* b3_fault_t of the first serious fault */
    /* the last time both line sensors were black and it
     * was NOT counted as a junction: how long the patch was and why. The
     * count goes up once per such patch, so the VC can print each one. */
    uint32_t patch_misses;
    uint16_t patch_miss_mm; /* both black together (2026-11-08) */
    /* each sensor's own run over the patch */
    uint16_t patch_miss_l_mm;
    uint16_t patch_miss_r_mm;
    uint8_t patch_miss_why; /* b3_patch_why_t */
    bool control_active;
    bool stop_required;
    /* last time IR_L or IR_R was black (the VC's line-loss check) */
    uint32_t line_seen_us;
    /* IR_B reads black now (the VC's lift check) */
    bool ir_b_black;
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

/* Report a serious fault from another Buddy 3 task (the
 * consumer task's message buffer / mailbox). Never blocks; safe from any
 * task. The car stays stopped until the board is reset. */
void buddy3_report_fault(b3_fault_t source, ER error);

/* Forget the held barcode letter (VehicleController calls
 * it on STOP and RESET, so an old letter is not used at the next junction).
 * Single caller (the VehicleController); never blocks. */
void buddy3_clear_held_symbol(void);

#if defined(APP_TEST) && APP_TEST
/* Test build only: restart the longest-service-gap measurement. */
void buddy3_test_reset_max_gap(void);
#endif

#endif
