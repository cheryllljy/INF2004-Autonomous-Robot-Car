/*
 *	subsystem5.c
 *	Subsystem 5 -- ObstacleScan.  Buddy 5, student ID 2502410.
 *
 *	subsystem5_init():  create the kernel objects (tick semaphore,
 *	                    profile mutex, flg_obstacle, 60 ms cyclic).
 *	                    Other subsystems may call
 *	                    obstacle_register_odometry() /
 *	                    obstacle_register_estop() in their own _init().
 *	subsystem5_start(): start the obstacle task.  The task initialises
 *	                    its own hardware (servo, sonar) because it is
 *	                    the owning task of GP15-17 and PWM slices 0, 7.
 *
 *	Files in this folder
 *	  subsystem5.h        public interface (events, getters, commands)
 *	  obstacle_task.c     the task and its state machine
 *	  obstacle_model.h    data model, packed layouts, pure functions
 *	  obstacle_geometry.c profile building and bypass policy (pure)
 *	  s5_pack.h/.c        CR1 minimum-bit encoders (pure)
 *	  drv_servo.c/.h      SG90 on PWM slice 7 B (GP15)
 *	  drv_ultrasonic.c/.h HC-SR04, echo timed by PWM slice 0 B (GP17)
 *	  drv_pwm_s5.h        register helpers for the two drivers
 *	  s5_config.h         pins, timing, limits
 *	  subsystem5_test.c   on-target tests (make TEST=1)
 *
 *	Host tests (no hardware): testing/subsystem5/host, run "make".
 */

#include "subsystem5.h"

EXPORT ER subsystem5_init(void)
{
	return obstacle_create_objects();
}

EXPORT ER subsystem5_start(void)
{
	/* Core 0 or 1: the task uses no interrupt, so it may run on
	   either core (unlike interrupt owners, which stay on core 0). */
	ID tid = app_start_task((FP)obstacle_task, S5_PRI_OBSTACLE, 0,
				"obstacle");

	return (tid < E_OK) ? tid : E_OK;
}
