# Subsystem 5 host tests. PC compiler only: no Pico SDK, no ARM
# toolchain, no hardware. Run before every flash and before every
# commit that touches app_program/subsystem5/.
#
#   make            build and run everything
#   make geometry   geometry and policy (G1..G16)
#   make pack       CR1 encoders, sizes, message size (P1..P8)
#   make equiv      CR1: v1 vs v2 geometry on 20 000 random sweeps
#   make sim        state machine on a virtual clock (S1..S12, no S6)
#   make calibrate  S6 width calibration rows
#   make v3         v3.0 maths: centre point, reverse, side watch (V1..V6)
#   make cal        v3.0: the real calibration vs a faulty simulated robot,
#                   then the obstacle task on it without / with the result
#   make car        v3.0: 2-D bypass simulation, S5 + VehicleController's
#                   obstacle part driving a simulated car (C1..C9)
#   make turn       nyx, 2026-11-06: the junction turn (arc / U-turn) of
#                   vc_turn.c with Buddy 3's line following (J1..J22)
#   make clean
#
# Run from testing/subsystem5/host.

CC      ?= cc
CFLAGS  := -std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion \
           -Wno-sign-conversion -O1 -g
S5      := ../../../app_program/subsystem5
OUT     := build

# mock/ provides tk/tkernel.h, tm/tmonitor.h, bsp/libbsp.h; it must
# come before anything else so the real kernel headers are never seen.
APP     := ../../../app_program
INC     := -Imock -I$(S5) -I.
CAR_INC := $(INC) -I$(APP)/subsystem2 -I$(APP)/vehicle_controller \
           -I$(APP)/subsystem3
# nyx, 2026-11-06: v1 gets v2's chassis numbers (obstacle_model.h:
# OBSTACLE_HALF_WIDTH_MM, OBSTACLE_SAFETY_MARGIN_MM), so the CR1
# equivalence test still compares the two algorithms on the same car.
V1_CAR  := -DBOARD_HALF_WIDTH_MM=65U -DOBSTACLE_SAFETY_MARGIN_MM=40U
V1_INC  := -Iv1_reference -Imock -DOBSTACLE_HOST_TEST $(V1_CAR) \
           -Dobstacle_profile_build=v1_obstacle_profile_build \
           -Dobstacle_plan=v1_obstacle_plan

PURE    := $(S5)/obstacle_geometry.c $(S5)/s5_pack.c
TASK    := $(S5)/obstacle_task.c tk_shim.c sim_hal.c

.PHONY: all geometry pack equiv sim calibrate v3 cal car turn clean

# The robot's calibration as the faulty simulated robot needs it (the
# values "make cal" finds); compiled into the *_fixed binaries.
CAL_FIX := -DS5_CAL_DONE=1 -DS5_CAL_SERVO_TRIM_DDEG=-35 \
           -DS5_CAL_SERVO_BACKLASH_DDEG=15 -DS5_CAL_BEAM_HALF_DDEG=110 \
           -DS5_CAL_RANGE_SCALE_Q12=3983 -DS5_CAL_RANGE_OFFSET_MM=7
# nyx, 2026-11-06: the same robot with the beam step 4 finds on the 21 cm
# box (test_cal box), compiled into test_cal_box.
CAL_BOX := $(subst -DS5_CAL_BEAM_HALF_DDEG=110,-DS5_CAL_BEAM_HALF_DDEG=221,$(CAL_FIX))

all: geometry pack equiv sim v3 cal car turn
	@echo ""
	@echo "All subsystem 5 host tests passed."

$(OUT):
	@mkdir -p $(OUT)

$(OUT)/v1_geometry.o: v1_reference/obstacle_geometry_v1.c | $(OUT)
	$(CC) -std=c99 -O1 $(V1_INC) -c $< -o $@

$(OUT)/v1_adapter.o: v1_reference/v1_adapter.c | $(OUT)
	$(CC) -std=c99 -O1 $(V1_INC) -c $< -o $@

geometry: | $(OUT)
	@echo "=== geometry and policy ==="
	$(CC) $(CFLAGS) $(INC) $(PURE) test_geometry.c -o $(OUT)/test_geometry
	@$(OUT)/test_geometry

pack: $(OUT)/v1_geometry.o $(OUT)/v1_adapter.o
	@echo ""
	@echo "=== CR1 packing ==="
	$(CC) $(CFLAGS) $(INC) -Iv1_reference $(PURE) test_pack.c \
	    $(OUT)/v1_geometry.o $(OUT)/v1_adapter.o -o $(OUT)/test_pack
	@$(OUT)/test_pack

equiv: $(OUT)/v1_geometry.o $(OUT)/v1_adapter.o
	@echo ""
	@echo "=== CR1 equivalence: v1 vs v2 geometry ==="
	$(CC) $(CFLAGS) $(INC) -Iv1_reference $(PURE) test_equiv.c \
	    $(OUT)/v1_geometry.o $(OUT)/v1_adapter.o -o $(OUT)/test_equiv
	@$(OUT)/test_equiv

sim: | $(OUT)
	@echo ""
	@echo "=== simulation scenarios ==="
	$(CC) $(CFLAGS) $(INC) $(PURE) $(TASK) test_sim.c -lm -o $(OUT)/test_sim
	@for s in 1 2 3 4 5 7 8 9 10 11 12 13; do $(OUT)/test_sim $$s || exit 1; done

calibrate: | $(OUT)
	$(CC) $(CFLAGS) $(INC) $(PURE) $(TASK) test_sim.c -lm -o $(OUT)/test_sim
	@$(OUT)/test_sim 6 100 180 | grep -E 'S6|nearest|no scan'
	@$(OUT)/test_sim 6 150 220 | grep -E 'S6|nearest|no scan'
	@$(OUT)/test_sim 6 200 260 | grep -E 'S6|nearest|no scan'
	@$(OUT)/test_sim 6 250 290 | grep -E 'S6|nearest|no scan'

v3: | $(OUT)
	@echo ""
	@echo "=== v3.0 geometry: centre point, reverse, side watch ==="
	$(CC) $(CFLAGS) $(INC) $(PURE) test_v3.c -lm -o $(OUT)/test_v3
	@$(OUT)/test_v3

cal: | $(OUT)
	@echo ""
	@echo "=== v3.0 calibration against a faulty robot ==="
	$(CC) $(CFLAGS) $(INC) $(PURE) $(TASK) $(S5)/s5_calibrate.c test_cal.c \
	    -lm -o $(OUT)/test_cal
	$(CC) $(CFLAGS) $(INC) $(CAL_FIX) $(PURE) $(TASK) $(S5)/s5_calibrate.c \
	    test_cal.c -lm -o $(OUT)/test_cal_fixed
	@$(OUT)/test_cal cal
	@$(OUT)/test_cal centre
	@$(OUT)/test_cal_fixed centre
	$(CC) $(CFLAGS) $(INC) $(CAL_BOX) $(PURE) \
	    $(TASK) $(S5)/s5_calibrate.c test_cal.c -lm -o $(OUT)/test_cal_box
	@$(OUT)/test_cal box
	@$(OUT)/test_cal_box box

car: | $(OUT)
	@echo ""
	@echo "=== v3.0 2-D bypass simulation ==="
	$(CC) $(CFLAGS) $(CAR_INC) $(PURE) $(TASK) motion_mock.c \
	    $(APP)/vehicle_controller/vc_obstacle.c $(APP)/subsystem3/b3_line.c \
	    test_car.c -lm -o $(OUT)/test_car
	@$(OUT)/test_car
	$(CC) $(CFLAGS) $(CAR_INC) -DS5_CAL_BEAM_HALF_DDEG=221 $(PURE) $(TASK) \
	    motion_mock.c $(APP)/vehicle_controller/vc_obstacle.c \
	    $(APP)/subsystem3/b3_line.c test_car.c -lm -o $(OUT)/test_car_cal
	@$(OUT)/test_car_cal

# nyx, 2026-11-06: the VehicleController's junction turn (vc_turn.c) with
# Buddy 3's line following, at 25, 50 (the car's) and 80 mm/s.
TURN_SRC := motion_mock.c $(APP)/vehicle_controller/vc_turn.c \
            $(APP)/subsystem3/b3_line.c test_turn.c
turn: | $(OUT)
	@echo ""
	@echo "=== junction turns (VC-H01) ==="
	$(CC) $(CFLAGS) $(CAR_INC) $(TURN_SRC) -lm -o $(OUT)/test_turn
	@$(OUT)/test_turn
	$(CC) $(CFLAGS) $(CAR_INC) -DB3_BASE_SPEED_MM_S=25 $(TURN_SRC) -lm \
	    -o $(OUT)/test_turn_25
	@$(OUT)/test_turn_25
	$(CC) $(CFLAGS) $(CAR_INC) -DB3_BASE_SPEED_MM_S=80 $(TURN_SRC) -lm \
	    -o $(OUT)/test_turn_80
	@$(OUT)/test_turn_80

clean:
	rm -rf $(OUT)
