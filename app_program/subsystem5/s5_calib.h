/**
 * @file    s5_calib.h
 * @brief   Calibration values for THIS robot's servo, sensor and bracket.
 *
 * v3.0 (prof feedback, Nov 2026: "however you adjust it you can't get the
 * centre point"). Three errors stack up, and none can be fixed by hand:
 *
 *   1. Servo trim. The SG90 horn sits on a spline with teeth ~15 deg
 *      apart, so re-seating it can never remove an offset smaller than
 *      that. "90 deg" is wherever the horn landed, not straight ahead.
 *   2. Backlash. The gears stop a few tenths of a degree to a few degrees
 *      short, on the side the horn came from. Moving left and moving
 *      right land on different angles for the same command.
 *   3. Beam width. The HC-SR04 hears anything inside a cone of about
 *      +/-7.5..15 deg, so an object looks wider than it is and its edges
 *      (and so its centre) move.
 *
 * Plus the range error of the sensor itself (a scale and an offset).
 *
 * HOW TO FILL THIS IN
 *   1. Set S5_CAL_BUILD to 1 in s5_config.h, build, flash.
 *   2. Follow the serial monitor (3 guided steps, ~4 minutes). Guide:
 *      testing/subsystem5/CALIBRATION.md
 *   3. It prints a block of #defines. Paste it over the values below,
 *      set S5_CAL_BUILD back to 0, build, flash.
 *
 * Each value is #ifndef-guarded only so the host tests can inject a
 * deliberately wrong servo or sensor; on the robot, edit them here.
 *
 * The defaults below are the datasheet / "no correction" values, so an
 * uncalibrated robot behaves exactly like v2.2. S5_CAL_DONE = 0 is
 * reported in health (obstacle_health_t.calibrated) and telemetry, so
 * the dashboard shows when a robot runs uncalibrated.
 */
#ifndef S5_CALIB_H
#define S5_CALIB_H

/** 1 once real measured values are pasted in. */
#ifndef S5_CAL_DONE
#define S5_CAL_DONE                 (0)
#endif

/*---------------------------------------------------------------------*/
/* Servo                                                               */
/*---------------------------------------------------------------------*/

/** Command (in 0.1 deg) that points the sensor straight ahead, minus
 *  90.0 deg. Example: -35 means "command 86.5 deg to look straight". */
#ifndef S5_CAL_SERVO_TRIM_DDEG
#define S5_CAL_SERVO_TRIM_DDEG      (0)
#endif

/** Half the backlash dead band, 0.1 deg. Added when the horn moves
 *  towards higher angles (left), subtracted when it moves right. */
#ifndef S5_CAL_SERVO_BACKLASH_DDEG
#define S5_CAL_SERVO_BACKLASH_DDEG  (0)
#endif

/*---------------------------------------------------------------------*/
/* Range: true_mm = raw_mm * SCALE / 4096 + OFFSET                     */
/*---------------------------------------------------------------------*/

/** Q12 gain: 4096 = 1.000. */
#ifndef S5_CAL_RANGE_SCALE_Q12
#define S5_CAL_RANGE_SCALE_Q12      (4096)
#endif

/** mm, added after the gain. */
#ifndef S5_CAL_RANGE_OFFSET_MM
#define S5_CAL_RANGE_OFFSET_MM      (0)
#endif

/*---------------------------------------------------------------------*/
/* Beam and mount                                                      */
/*---------------------------------------------------------------------*/

/** Beam half-angle, 0.1 deg: the angle off the sensor axis at which a
 *  thin post still answers. Datasheet "15 deg measuring angle" = 7.5. */
#ifndef S5_CAL_BEAM_HALF_DDEG
#define S5_CAL_BEAM_HALF_DDEG       (75)
#endif

/** Sensor face ahead of the servo axis, mm (ruler). The geometry works
 *  in polar coordinates round the servo axis, so this is added to every
 *  swept range. TODO(measure). */
#ifndef S5_CAL_SENSOR_ARM_MM
#define S5_CAL_SENSOR_ARM_MM        (0)
#endif

#endif /* S5_CALIB_H */
