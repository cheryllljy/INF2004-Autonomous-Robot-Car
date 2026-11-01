/* HOST TEST ONLY. Subsystem 5's kernel-facing code reaches hardware
 * only through drv_servo / drv_ultrasonic, which sim_hal.c replaces,
 * so nothing from the BSP is needed here. */
#ifndef MOCK_LIBBSP_H
#define MOCK_LIBBSP_H
#endif
