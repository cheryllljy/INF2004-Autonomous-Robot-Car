/*
 *	vehicle_controller.h
 *	VehicleController -- the one task that decides what the car does.
 *
 *	Shared by the whole team: each buddy fills in the hook marked with
 *	their name in vehicle_controller.c.  The other subsystems produce data
 *	and requests; only this task commands the motors (through Buddy 2)
 *	and only this task clears a latched fault.
 *
 *	Interfaces
 *	----------
 *	IF-VC-1  any task / ISR -> VC   vc_raise_fault(cause bits)   flg_vc 0..7
 *	IF-VC-2  Buddy 1 -> VC          vc_post_command(VC_CMD_*)    flg_vc 8..15
 *	IF-VC-3  VC -> any task (read)  vc_get_status_word()         one 32-bit read
 *	IF-VC-4  VC -> Buddy 1          vc_fill_telemetry()          B1_Telemetry
 *	Plus the existing ones it consumes: Buddy 2 motion_*(), Buddy 3
 *	buddy3_set_context / get_snapshot / pop_navigation, Buddy 5
 *	flg_obstacle + obstacle_resume / reset_fault.
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"
#include "vc_types.h"

/* 1 = usermain creates and starts the VehicleController.  Off for now:
   the B3-H08/H09 tests drive the motors themselves, and two tasks
   commanding Buddy 2 at once would fight. */
#ifndef VC_ENABLE
#define VC_ENABLE		0
#endif

#define VC_PRI			22	/* band 22-23: below LineSense (17) and
					   ObstacleScan (20), per Week 6 */
#define VC_PERIOD_MS		10	/* matches Buddy 3's service period */
#define VC_SPEED_REFRESH_MS	100	/* resend wheel speeds at least this often */

/* flg_vc: bits 0..7 latched fault causes, bits 8..15 command requests */
#define VC_FLG_CMD_SHIFT	8U
#define VC_FLG_CMD(cmd)		(1U << (VC_FLG_CMD_SHIFT + (UINT)(cmd)))
#define VC_FLG_CMD_MASK		(((1U << (1U << VC_CMD_BITS)) - 1U) << VC_FLG_CMD_SHIFT)

IMPORT ID	flg_vc;

IMPORT ER	vc_init(void);			/* phase 1: kernel objects */
IMPORT ER	vc_start(void);			/* phase 2: the task */

IMPORT void	vc_raise_fault(UINT causes);	/* any task or ISR */
IMPORT void	vc_post_command(vc_cmd_t cmd);	/* any task */
IMPORT uint32_t	vc_get_status_word(void);	/* lock-free */
IMPORT void	vc_fill_telemetry(B1_Telemetry *p_out);

#endif /* VEHICLE_CONTROLLER_H */
