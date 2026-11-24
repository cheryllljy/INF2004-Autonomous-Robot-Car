/* Buddy 3 team-integration switches. See BUDDY3_PORTING_NOTES.md at the
 * repository root before changing them. */
#ifndef BUDDY3_TEAM_CONFIG_H
#define BUDDY3_TEAM_CONFIG_H
#define B3_TEAM_AUTOSTART 1

/* 1 = pass Buddy 2's encoder ticks to buddy3_start() as the position
 * reader. Needs subsystem2 in the build and subsystem2_init() called before
 * subsystem3_init(). 0 for the Buddy 3 standalone bench test. */
#define B3_USE_B2_ODOMETRY 1
#endif
