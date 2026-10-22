/*
 * cyc_check.h - interface of the cyclic-handler bench check (cyc_check.c).
 * Include THIS header from subsystem2.c. Never #include a .c file.
 */
#ifndef CYC_CHECK_H
#define CYC_CHECK_H

void cyc_check_init(void);      /* call from subsystem2_init()  */
void cyc_check_start(void);     /* call from subsystem2_start() */

#endif /* CYC_CHECK_H */
