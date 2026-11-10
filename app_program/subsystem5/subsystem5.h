/*
 *      subsystem5.h
 *      Subsystem 5 -- obstacle detection and bypass: HC-SR04P ultrasonic
 *      sensor on an SG90 servo (rewritten from scratch on
 *      2026-11-07; the old version is in git history before this change).
 *
 *      Files
 *        s5_config.h   every pin, size and tuning number (CALIBRATE marks
 *                      the ones the calibration build measures)
 *        s5_hw.c       TRIG / ECHO / servo, register level
 *        s5_sensor.c   the S5 task: servo, pings every 60 ms, the sweep
 *        s5_scan.c     the object's width and the room beside it
 *        s5_path.c     the smooth path round it and how it is followed
 *        s5_odom.c     dead reckoning from Buddy 2's encoder ticks
 *        s5_avoid.c    the episode, run by the VehicleController's task
 *        s5_cal.c      guided calibration (make S5CAL=1)
 *        s5_math.c     sin, cos, atan2, sqrt without libm
 *
 *      Who does what
 *        S5 task (priority 20)  sensor work only, never the motors
 *        VehicleController      s5_avoid_*(): decides, drives Buddy 2
 *
 *      Interfaces
 *        IF-S5-1  VC -> S5   s5_sensor_guard / look / scan (one word)
 *        IF-S5-2  S5 -> VC   s5_sensor_ping, s5_sensor_scan_get
 *        IF-S5-3  VC         s5_avoid_check / begin / step / abort,
 *                            s5_avoid_blocked (fault bit 0x40, no E-stop),
 *                            s5_avoid_sensor_fault (FR25, VC_FAULT_NO_ECHO)
 *      Console lines start with "[VC-S5]" (mirrored to MQTT).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "s5_avoid.h"

#endif /* SUBSYSTEM5_H */
