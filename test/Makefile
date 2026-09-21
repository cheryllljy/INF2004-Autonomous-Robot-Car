# Host-side tests for the Buddy 5 obstacle subsystem.
#
# These build with your PC's normal compiler. No Pico SDK, no ARM
# toolchain, no hardware. Run them before every flash.
#
#   make            build and run everything
#   make geometry   just the pure geometry and policy tests
#   make sim        just the five simulation scenarios
#   make clean
#
# Run from the Ultrasonic/test directory.

CC      ?= cc
CFLAGS  := -std=c99 -Wall -Wextra -Wpedantic -O1 -g
INC     := -I../include -I../shim -I.
OUT     := ../build/host

# OBSTACLE_HOST_TEST hides the kernel-facing half of obstacle.h.
# TKSHIM_HOST selects the shim's virtual-clock backend.
GEOM_DEFS := -DOBSTACLE_HOST_TEST
SIM_DEFS  := -DTKSHIM_HOST

GEOM_SRC := ../src/obstacle_geometry.c test_geometry.c

SIM_SRC  := ../src/obstacle.c \
            ../src/obstacle_geometry.c \
            ../src/tk_shim.c \
            sim_hal.c \
            test_sim.c

.PHONY: all geometry sim clean

all: geometry sim
	@echo ""
	@echo "All host tests passed."

$(OUT):
	@mkdir -p $(OUT)

geometry: $(OUT)
	@echo "=== geometry and policy ==="
	$(CC) $(CFLAGS) $(GEOM_DEFS) $(INC) $(GEOM_SRC) -o $(OUT)/test_geometry
	@$(OUT)/test_geometry

sim: $(OUT)
	@echo ""
	@echo "=== simulation scenarios ==="
	$(CC) $(CFLAGS) $(SIM_DEFS) $(INC) $(SIM_SRC) -lm -o $(OUT)/test_sim
	@for s in 1 2 3 4 5 7; do $(OUT)/test_sim $$s || exit 1; done

# Width calibration sweep (bench test G3, done in simulation first).
.PHONY: calibrate
calibrate: $(OUT)
	$(CC) $(CFLAGS) $(SIM_DEFS) $(INC) $(SIM_SRC) -lm -o $(OUT)/test_sim
	@$(OUT)/test_sim 6 100 180 | grep -E 'S6|nearest|ideal|no scan'
	@$(OUT)/test_sim 6 150 220 | grep -E 'S6|nearest|ideal|no scan'
	@$(OUT)/test_sim 6 200 260 | grep -E 'S6|nearest|ideal|no scan'
	@$(OUT)/test_sim 6 250 290 | grep -E 'S6|nearest|ideal|no scan'
	@echo ""
	@echo "Rows must sit inside OBSTACLE_TRIGGER_MM or no sweep is run."

clean:
	rm -rf $(OUT)
