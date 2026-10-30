/* Buddy 3 team-integration switch. New file; existing subsystem code remains intact.
 * Keep disabled until the team has approved the GPIO map and barcode IRQ owner.
 * Enable (1) only after reading BUDDY3_PORTING_NOTES.md at repository root.
 */
#ifndef BUDDY3_TEAM_CONFIG_H
#define BUDDY3_TEAM_CONFIG_H
#define B3_TEAM_AUTOSTART 1 // set this for testing 

/* 1 = pass Buddy 2's encoder ticks to buddy3_start() as the position reader.
 * Needs subsystem2 in the build and subsystem2_init() called before
 * subsystem3_init(). Keep 0 for the Buddy 3 standalone test. */
#define B3_USE_B2_ODOMETRY 1 // set 1 for testing with Buddy 2, 0 for standalone bench test
#endif
