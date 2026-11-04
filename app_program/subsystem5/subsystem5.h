/*
 *	subsystem5.h
 *	Subsystem 5 -- ObstacleScan (ultrasonic + servo, obstacle profiling).
 *	Buddy 5, student ID 2502410.
 *
 *	Public interface of subsystem 5. Other subsystems include it as
 *	"../subsystem5/subsystem5.h". Everything else stays private.
 *
 *	What S5 owns:   GP15 (servo, PWM slice 7), GP16 (TRIG), GP17 (ECHO,
 *	                PWM slice 0 input), the decision of which side to pass.
 *	What S5 never:  drives a motor, reads the line sensors, calls MQTT,
 *	                or clears the E-stop. It produces data and a
 *	                recommendation; VehicleController acts.
 *
 *	Interfaces (TEAM.md section 5)
 *	------------------------------
 *	IF-S5-1  S5 -> VehicleController   event flag flg_obstacle (owned by S5)
 *	         Wait with tk_wai_flg(flg_obstacle, S5_EVT_ALL,
 *	                              TWF_ORW | TWF_BITCLR, &ptn, tmo)
 *	         so only the bits you handled are cleared.
 *	IF-S5-2  VehicleController -> S5   obstacle_resume(), obstacle_reset_fault()
 *	IF-S5-3  any task -> S5 (read)     obstacle_get_front_mm()   (lock-free)
 *	                                   obstacle_get_profile()    (mutex copy)
 *	IF-S5-4  S2 / VehicleController -> S5   obstacle_register_odometry()
 *	IF-S5-5  S5 -> E-stop owner        obstacle_register_estop(): S5 SETS its
 *	                                   cause bit on a sensor fault only;
 *	                                   only VehicleController clears it.
 *	IF-S5-6  S5 -> Telemetry (Buddy 1) obstacle_get_telemetry(): 33-byte
 *	                                   binary packet, version 2 (layout
 *	                                   in obstacle_model.h)
 *	IF-S5-7  v3.0 VehicleController <-> S5, the bypass:
 *	         obstacle_watch_start(side) / obstacle_watch_end()   pass leg
 *	         obstacle_rescan()        after backing off a REVERSE
 *	         obstacle_assist_kick()   every VC loop while OBSTACLE
 *	         obstacle_get_watch()     side clearance, phase, reverse mm
 *	         events S5_EVT_SIDE_CLOSE, S5_EVT_PASSED
 *
 *	Normal obstacle stops do NOT use the E-stop. S5 raises S5_EVT_DETECTED
 *	(and S5_EVT_HARD_STOP under 120 mm); VehicleController sends STOP to
 *	MotionControl. A latched E-stop would block the bypass.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#include "obstacle_model.h"

/* ---------------------------------------------------------------------
 *  Priority. Template band for S5 is 20-21 (TEAM.md section 4).
 *
 *  The Week 6 design ranks ObstacleScan second, right below
 *  MotionControl (1 = highest). The template bands put S5 last. On this
 *  dual-core build the task blocks for ~95 % of each 60 ms period, so
 *  the band is workable for bring-up, but the team must agree one
 *  mapping before integration (tutor feedback: fix the priority
 *  convention first). Change it HERE only, after that agreement.
 * --------------------------------------------------------------------- */
#define S5_PRI_OBSTACLE		20

/* ---------------------------------------------------------------------
 *  flg_obstacle bits.  OUT = set by S5, cleared by the waiter.
 *                      IN  = set through the functions below, consumed
 *                            by S5.  Do not set IN bits directly.
 * --------------------------------------------------------------------- */
#define S5_EVT_DETECTED		0x0001U	/* OUT  close object confirmed:
					   STOP the car, S5 waits for
					   speed = 0, then scans */
#define S5_EVT_HARD_STOP	0x0002U	/* OUT  object under 120 mm */
#define S5_EVT_PROFILED		0x0004U	/* OUT  profile ready, act on it */
#define S5_EVT_CLEARED		0x0008U	/* OUT  scan found the path clear */
#define S5_EVT_FAULT		0x0010U	/* OUT  S5 entered FAULT; see
					   obstacle_get_health().fault */
#define S5_EVT_SIDE_CLOSE	0x0020U	/* OUT  v3.0 WATCH: flank closer
					   than S5_SIDE_CLOSE_MM to the
					   obstacle: steer away */
#define S5_EVT_PASSED		0x0040U	/* OUT  v3.0 WATCH: the rear of the
					   car is past the obstacle */
#define S5_EVT_ALL		0x007FU

#define S5_REQ_RESUME		0x0100U	/* IN   bypass done, line found */
#define S5_REQ_RESET		0x0200U	/* IN   leave a latched fault */
#define S5_REQ_WATCH_LEFT	0x0400U	/* IN   v3.0 pass leg, obstacle on
					   the car's left */
#define S5_REQ_WATCH_RIGHT	0x0800U	/* IN   ...on the right */
#define S5_REQ_WATCH_END	0x1000U	/* IN   stop watching, back to ASSIST */
#define S5_REQ_RESCAN		0x2000U	/* IN   car backed off: scan again */

IMPORT ID	flg_obstacle;

/* ---------------------------------------------------------------------
 *  Fast path
 * --------------------------------------------------------------------- */

/* Latest forward range in mm, OBSTACLE_RANGE_CLEAR, or
   OBSTACLE_RANGE_UNKNOWN (stale > 250 ms, faulted, or blind zone).
   UNKNOWN never means clear.  Lock-free: one 32-bit read. */
IMPORT uint16_t	obstacle_get_front_mm(void);

/* Copy of the latest profile (unpacked for the caller).
   Returns FALSE if none has been published yet. */
IMPORT bool	obstacle_get_profile(obstacle_profile_t *p_out);
IMPORT bool	obstacle_get_profile_packed(obstacle_profile_packed_t *p_out);

IMPORT void	obstacle_get_health(obstacle_health_t *p_out);

/* 29-byte binary status packet for MQTT.  Returns bytes written or 0. */
IMPORT uint32_t	obstacle_get_telemetry(uint8_t *p_buf, uint32_t len);

/* ---------------------------------------------------------------------
 *  Commands from VehicleController
 * --------------------------------------------------------------------- */
IMPORT void	obstacle_resume(void);		/* ASSIST -> GUARD */
IMPORT void	obstacle_reset_fault(void);	/* latched FAULT -> GUARD */

/* v3.0 bypass support (IF-S5-7) */
IMPORT void	obstacle_rescan(void);		/* ASSIST -> WAIT_STOP -> scan */
IMPORT void	obstacle_watch_start(bool obstacle_on_left); /* -> WATCH */
IMPORT void	obstacle_watch_end(void);	/* WATCH -> ASSIST */
IMPORT void	obstacle_assist_kick(void);	/* "VC alive", every loop */

typedef struct {
	bool		 obstacle_on_left;
	s5_watch_phase_t phase;
	uint16_t	 reverse_mm;	/* back off this far on REVERSE */
	uint16_t	 lateral_mm;	/* closest face beside us (sensor axis),
					   0xFFFF = none seen yet */
	uint16_t	 side_range_mm;	/* last side ping (range code) */
} obstacle_watch_t;

/* Lock-free: one 32-bit read of the watch word. */
IMPORT void	obstacle_get_watch(obstacle_watch_t *p_out);

/* ---------------------------------------------------------------------
 *  Registration, call during subsystemN_init() (before any task runs)
 * --------------------------------------------------------------------- */

/* Non-blocking odometry read: speed in mm/s (signed), distance in mm.
   Return FALSE if no fresh data.  Must not call anything that blocks. */
typedef bool (*s5_odometry_fn)(int32_t *p_speed_mm_s, int32_t *p_distance_mm);
IMPORT void	obstacle_register_odometry(s5_odometry_fn fn);

/* The E-stop flag and the ONE bit S5 may set in it. */
IMPORT void	obstacle_register_estop(ID flgid, UINT cause_bit);

/* ---------------------------------------------------------------------
 *  Private to subsystem 5 (used by subsystem5.c and the tests)
 * --------------------------------------------------------------------- */
IMPORT ER	obstacle_create_objects(void);
IMPORT void	obstacle_task(INT stacd, void *exinf);

/* v3.0 calibration build (s5_calibrate.c): the task, and its result for
   the host test that runs it against a simulated wrong servo. */
typedef struct {
	bool	finished;		/* all three steps ran */
	bool	step1_ok;		/* servo trim, backlash, beam */
	bool	step2_ok;		/* range fit within NFR15 */
	bool	step3_ok;		/* verify: post centred at 0 deg */
	int16_t	trim_ddeg;
	int16_t	backlash_ddeg;
	int16_t	beam_half_ddeg;
	int32_t	scale_q12;
	int32_t	offset_mm;
	int16_t	verify_bearing_deg;	/* S5 sweep centre of the post */
} s5_cal_result_t;

IMPORT void	s5_calibrate_task(INT stacd, void *exinf);
IMPORT void	s5_cal_get_result(s5_cal_result_t *p_out);

#endif /* SUBSYSTEM5_H */
