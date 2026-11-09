/* HOST TEST ONLY. vc_turn.c includes the BSP header through the
 * VehicleController's headers but uses nothing from it here; subsystem 5
 * reaches its hardware only through s5_hw.c, which s5_sim.c replaces.
 * (nyx, 2026-11-07) */
#ifndef MOCK_LIBBSP_H
#define MOCK_LIBBSP_H
#endif
