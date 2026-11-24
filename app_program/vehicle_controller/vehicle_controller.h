/*
 *      vehicle_controller.h
 *      VehicleController -- the one task that decides what the car does.
 *
 *      Shared by the whole team: each buddy fills in the hook marked with
 *      their name in vehicle_controller.c.  The other subsystems produce data
 *      and requests; only this task commands the motors (through Buddy 2)
 *      and only this task clears a latched fault.
 *
 *      Interfaces
 *      ----------
 *      IF-VC-1  any task / ISR -> VC  vc_raise_fault()     flg_vc 0..7
 *      IF-VC-2  Buddy 1 -> VC         vc_post_command()    flg_vc 8..15
 *               (START/STOP/RESET, and TURN_* after MQTT maps a letter)
 *      IF-VC-3  VC -> any task        vc_get_status_word() one 32-bit read
 *      IF-VC-4  VC -> Buddy 1         vc_fill_telemetry()  B1_Telemetry
 *      Plus the existing ones it consumes: Buddy 2 motion_*(), Buddy 3
 *      buddy3_set_context / get_snapshot / pop_navigation, Buddy 5
 *      s5_avoid_*() (subsystem5/s5_avoid.h: the bypass runs in vc_task).
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"
#include "vc_types.h"

/* 1 = usermain creates and starts the VehicleController (START/STOP over
   MQTT). 0 for the B3-H08/H09 tests, which drive the motors themselves. */
#ifndef VC_ENABLE
#define VC_ENABLE 1
#endif

/* 1 = bench test without Buddy 1: START by itself VC_BENCH_START_S after
   boot, and on_barcode() maps the letter with the brief's example table
   (Z left, B right, C straight, D U-turn) in place of MQTT.  0 for the
   real robot. */
#ifndef VC_BENCH
#define VC_BENCH 0
#endif
#define VC_BENCH_START_S 10

/* below ObstacleScan (16), LineSense (17) and the IMU (18), above Wi-Fi
 * and telemetry (24) */
#define VC_PRI       22
#define VC_PERIOD_MS 10 /* matches Buddy 3's service period */
/* resend wheel speeds at least this often */
#define VC_SPEED_REFRESH_MS 100
/* car lifted: IR_L, IR_R and IR_B all see no floor (they read black)
 * for this far of wheel travel while the car drives. Longer than any
 * black patch on the track (a junction is under 25 mm; both line IRs
 * have read black for up to 67 mm in tight bends). */
#ifndef VC_LIFT_MM
#define VC_LIFT_MM 100
#endif

/* flg_vc: bits 0..7 latched fault causes, bits 8..15 command requests */
#define VC_FLG_CMD_SHIFT 8U
#define VC_FLG_CMD(cmd)  (1U << (VC_FLG_CMD_SHIFT + (UINT)(cmd)))
#define VC_FLG_CMD_MASK  (((1U << (1U << VC_CMD_BITS)) - 1U) << VC_FLG_CMD_SHIFT)

extern ID flg_vc;

extern ER vc_init(void);  /* phase 1: kernel objects */
extern ER vc_start(void); /* phase 2: the task */

extern void vc_raise_fault(UINT causes);   /* any task or ISR */
extern void vc_post_command(vc_cmd_t cmd); /* any task */
extern uint32_t vc_get_status_word(void);  /* lock-free */
extern void vc_fill_telemetry(B1_Telemetry *p_out);

#endif /* VEHICLE_CONTROLLER_H */
