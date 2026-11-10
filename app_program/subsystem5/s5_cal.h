/*
 *      s5_cal.h
 *      Subsystem 5 -- guided calibration.
 *
 *      Built with make S5CAL=1 (the image gets a _s5cal suffix). The S5
 *      task then runs four steps instead of the obstacle logic, printing
 *      what to set up before each one and the numbers to copy into
 *      s5_config.h after it:
 *
 *        1  servo direction and trim   S5_SERVO_INVERT, S5_SERVO_TRIM_DDEG
 *        2  range offset               S5_RANGE_OFFSET_MM
 *        3  beam half-angle            S5_BEAM_NEAR_DDEG, S5_BEAM_FAR_DDEG
 *        4  how far the car turns      S5_TURN_PCT
 *
 *      The sweeps in steps 1 and 3 are the obstacle logic's own (same
 *      code, same timing), so what is measured is what the robot uses.
 *      Step by step: testing/subsystem5/CALIBRATION.md.
 */

#ifndef S5_CAL_H
#define S5_CAL_H

#include "s5_config.h"

#if S5_CAL_BUILD
/* The S5 task, calibration build: never returns. */
void s5_cal_run(void);
#endif

/* Sleep until the S5 task's next poll (subsystem5.c, every S5_POLL_MS);
 * the calibration's loops use it too, so they poll at the same rate. */
void s5_poll_wait(void);

#endif /* S5_CAL_H */
