/*
 *      s5_port.h
 *      Subsystem 5 -- console output (nyx, 2026-11-07).
 *
 *      Every S5 line starts with "[VC-S5]" so the console mirror sends it
 *      to inf2004/robot/test too (lines starting "[VC" are mirrored; keep
 *      each line under 160 characters). tm_printf has no %f: print whole
 *      millimetres and tenths of a degree with %d.
 *
 *      The host tests build with S5_HOST defined: S5_LOG goes to
 *      s5_host_log() there (testing/subsystem5/host/s5_sim.c), which
 *      prints the line and lets the test read it.
 */

#ifndef S5_PORT_H
#define S5_PORT_H

#if defined(S5_HOST)
void s5_host_log(const char *p_fmt, ...);
#define S5_LOG(...) s5_host_log(__VA_ARGS__)
#else
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#define S5_LOG(...) ((void)tm_printf((UB *)__VA_ARGS__))
#endif

#endif /* S5_PORT_H */
