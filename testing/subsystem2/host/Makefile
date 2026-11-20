# Host tests for Buddy 2's motion control (Buddy 5, 2026-11-20).
# Compiles the real app_program/subsystem2/motion_control.c against stub
# kernel and hardware calls and a wheel model fitted to logs/v6-Tests.txt.
#   make         build and run all
#   make CFLAGS_EXTRA=-DSPEED_WINDOW_PERIODS=4U   try another setting
S2 := ../../../app_program/subsystem2
CFLAGS := -std=gnu99 -O1 -w -I$(S2) -I. $(CFLAGS_EXTRA)
TESTS := sim step sbend slow

all: $(TESTS)
	./sim
	./step
	./sbend
	./slow 75 20
	./slow 90 20
	./slow 100 20

%: %.c sim.c tk/tkernel.h $(S2)/motion_control.c $(S2)/motion_config.h
	gcc $(CFLAGS) $< -lm -o $@

clean:
	rm -f $(TESTS)
.PHONY: all clean
