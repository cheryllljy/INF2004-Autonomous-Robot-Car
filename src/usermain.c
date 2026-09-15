/**
 * @file    usermain.c
 * @brief   Entry point for the micro T-Kernel 3.0 build.
 *
 * This replaces src/main.c when the project is built against the real
 * kernel (USE_MTK3=ON). main.c is the bench bring-up application and
 * depends on the shim; this file depends only on the kernel API.
 *
 * In the integrated robot this function belongs to the team, not to
 * Buddy 5. The obstacle_init() call and the priority constant are the
 * only two lines this subsystem contributes; everything else here is
 * a placeholder showing where the other buddies hook in.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "board_config.h"
#include "obstacle.h"

EXPORT INT usermain(void)
{
    tm_printf((UB *)"robot starting\n");

    /* --- Buddy 5 ---------------------------------------------------
     * Creates the servo and ranger drivers, the guard cyclic handler
     * and the scan task. Must succeed before anything is allowed to
     * drive: without it there is no forward ranging at all.
     * -------------------------------------------------------------- */
    if (!obstacle_init())
    {
        tm_printf((UB *)"obstacle_init FAILED - not safe to drive\n");
    }
    else
    {
        tm_printf((UB *)"obstacle subsystem up\n");
    }

    /* --- Other buddies hook in here --------------------------------
     * motion_init();      Buddy 2, priority BOARD_PRI_MOTION_CTRL
     * line_init();        Buddy 3, priority BOARD_PRI_LINE_FOLLOW
     * imu_init();         Buddy 4, priority BOARD_PRI_IMU
     * telemetry_init();   Buddy 1, priority BOARD_PRI_TELEMETRY
     * vehicle_init();     controller, BOARD_PRI_VEHICLE_CTRL
     * -------------------------------------------------------------- */

    /* usermain must not return while tasks are running. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}
