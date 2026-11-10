/*
 *      s5_avoid.h
 *      Subsystem 5 -- the obstacle bypass, run by the VehicleController's
 *      task.
 *
 *      The VehicleController calls these from its own loop (about every
 *      20 ms), so the bypass and the line following never fight over
 *      Buddy 2: only vc_task commands the motors. The S5 task does the
 *      sensor work (s5_sensor.c) and never touches the motors.
 *
 *        line following  s5_avoid_track()  records where the line runs
 *                        s5_avoid_check()  true = stop, something ahead
 *        OBSTACLE state  s5_avoid_begin(), then s5_avoid_step() each pass
 *                        until it says DONE (back on the line: Buddy 3
 *                        follows again) or FAULT
 *        leaving early   s5_avoid_abort() (STOP, RESET, E-stop)
 *
 *      An episode:
 *        1. stop; if the object is closer than S5_SCAN_MIN_MM, back off
 *        2. sweep: width, which side is shorter (the same: either, at
 *           random), room on each side
 *        3. no room on either side: wait, stopped, with the obstacle fault
 *           bit showing (s5_avoid_blocked(); no E-stop); go on when the way
 *           ahead clears, look again every S5_BLOCKED_SCAN_MS
 *        4. reverse first if the ramp out needs more room
 *        5. drive the planned path (s5_path.h). Passing, the sensor looks
 *           across at the far corner: how deep the object is (the ramp back
 *           starts right after it; S5_DEPTH_DEFAULT_MM if it cannot be
 *           fixed) and where its side really is (further out if closer
 *           than planned). Every S5_AHEAD_EVERY_MM it looks once along the
 *           pass lane instead.
 *        6. another object: one close behind this one and
 *           clear of the pass lane is passed too; one in the pass lane, or
 *           ahead on the way back, stops the car and the episode starts
 *           again at 1 from where it is (turned square to the line first,
 *           on the spot if a sweep finds room), and ends on the same line.
 *           The new path, out, along and back in, keeps clear of the
 *           object just passed, and echoes that may be from it are left
 *           out of the new sweep (an edge next to them is not used). The
 *           episode's time and reverse limits run on across objects;
 *           more than S5_NEXT_MAX of them in one bypass is a fault
 *        7. the IRs find the line on the way back (another lane's line,
 *           further than S5_LINE_WINDOW_MM from ours, is ignored); the car
 *           settles on it and hands back to Buddy 3
 */

#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem2/motion_control.h"

typedef enum
{
    S5_AVOID_RUNNING = 0, /* send *p_l, *p_r */
    S5_AVOID_DONE,        /* on the line again: Buddy 3 drives */
    S5_AVOID_FAULT        /* raise VC_FAULT_OBSTACLE */
} s5_avoid_result_t;

void s5_avoid_init(void);

/* Every pass outside the OBSTACLE state. b_following: Buddy 3 is
 * following the line (its direction is learnt from these passes). */
void s5_avoid_track(const motion_status_t *p_m, bool b_following);

/* True: stop now, something is within S5_DETECT_MM ahead. Only while
 * b_armed (line following, nothing else driving). */
bool s5_avoid_check(bool b_armed);

/* FR25: the sensor gives no echo at all, or no ping came for
 * S5_SILENT_MS. */
bool s5_avoid_sensor_fault(void);

void s5_avoid_begin(const motion_status_t *p_m);

/* line search (robot.ps1 search): the car is off the
 * line; straight on until the IRs meet one, onto it, along it. Driven by
 * s5_avoid_step() like a bypass (the VehicleController's OBSTACLE state):
 * DONE = on the line, Buddy 3 follows; FAULT = no line found. */
void s5_search_begin(const motion_status_t *p_m);

s5_avoid_result_t s5_avoid_step(const motion_status_t *p_m, bool b_ir_l,
                                bool b_ir_r, int32_t dt_ms, int16_t *p_l,
                                int16_t *p_r);

void s5_avoid_abort(void);

/* No way round: stopped, waiting (the obstacle fault bit, no E-stop). */
bool s5_avoid_blocked(void);

#endif /* S5_AVOID_H */
