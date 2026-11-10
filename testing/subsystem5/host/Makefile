# Subsystem 5 host tests (rewritten with subsystem 5).
# PC compiler only: no Pico SDK, no ARM toolchain, no hardware. Run
# before every flash and before every commit that touches
# app_program/subsystem5/ or the VehicleController.
#
#   make            build and run everything
#   make unit       S5-H11..H20: maths, the real sweep on a simulated
#                   sensor, width, room beside the object, ramps,
#                   footprint check, steering, odometry, ping timing
#   make car        S5-H21: the whole episode on a simulated car (S5 task,
#                   bypass, Buddy 3's line following, Buddy 2 with lag),
#                   43 scenarios x 3 seeds; LIMIT rows are shown only
#   make turn       the junction turn (arc / U-turn) of vc_turn.c with
#                   Buddy 3's line following (J1..J22)
#   make follow     Buddy 3's line following on bends (B3-H10)
#   make clean
#
# Run from testing/subsystem5/host. One car scenario with its log:
#   build/test_s5_car C1 v      (vv: also the car's pose every 100 ms)
#   build/test_s5_car -s 10 -k 7 C22 v   only the 7th of 10 seeds

CC      ?= cc
CFLAGS  := -std=c99 -Wall -Wextra -Wpedantic -Wshadow -Wconversion \
           -Wno-sign-conversion -O1 -g
APP     := ../../../app_program
S5      := $(APP)/subsystem5
OUT     := build

# mock/ provides tk/tkernel.h, tm/tmonitor.h, bsp/libbsp.h; it must
# come before anything else so the real kernel headers are never seen.
INC     := -Imock -I$(S5) -I. -I$(APP)/subsystem2 \
           -I$(APP)/vehicle_controller -I$(APP)/subsystem3

# the real subsystem 5, on the simulated sensor (s5_sim.c replaces s5_hw.c)
S5_SRC  := $(S5)/s5_sensor.c $(S5)/s5_scan.c $(S5)/s5_path.c \
           $(S5)/s5_odom.c $(S5)/s5_math.c
SIM     := s5_sim.c motion_mock.c
# the simulated sensor and car have their own errors
# (beam 22 deg, no servo or range error unless a test sets one), so the
# real car's calibration in s5_config.h stays out of the host tests.
# The compass is the simulation's (s5_sim.c: field 2500 counts, centred,
# turning the other way to the car), and it steers (S5_MAG_USE 1) whatever
# the car's s5_config.h says.
S5FLAGS := $(CFLAGS) -DS5_HOST -Wno-missing-field-initializers \
           -DS5_SERVO_TRIM_DDEG=0 -DS5_RANGE_OFFSET_MM=0 \
           -DS5_BEAM_NEAR_DDEG=220 -DS5_BEAM_FAR_DDEG=220 -DS5_TURN_PCT=100 \
           -DS5_WIDTH_TRIM_MM=0 \
           -DS5_MAG_SIGN=-1 -DS5_MAG_X0=0 -DS5_MAG_Y0=0 -DS5_MAG_R=2500 \
           -DS5_MAG_USE=1

.PHONY: all unit car search turn follow clean

all: unit car search turn follow
	@echo ""
	@echo "All subsystem 5 host tests passed."

$(OUT):
	@mkdir -p $(OUT)

unit: | $(OUT)
	@echo "=== subsystem 5 parts (S5-H11..H20) ==="
	$(CC) $(S5FLAGS) -Wno-float-conversion $(INC) $(SIM) $(S5_SRC) \
	    test_s5_unit.c -lm -o $(OUT)/test_s5_unit
	@$(OUT)/test_s5_unit

car: | $(OUT)
	@echo ""
	@echo "=== obstacle episodes on a simulated car (S5-H21) ==="
	$(CC) $(S5FLAGS) $(INC) $(SIM) $(S5_SRC) $(S5)/s5_avoid.c \
	    $(APP)/subsystem3/b3_line.c test_s5_car.c -lm -o $(OUT)/test_s5_car
	@$(OUT)/test_s5_car

# the line search (robot.ps1 search) on the simulated car
search: | $(OUT)
	@echo ""
	@echo "=== line search on a simulated car (S5-H22) ==="
	$(CC) $(S5FLAGS) $(INC) $(SIM) $(S5_SRC) $(S5)/s5_avoid.c \
	    $(APP)/subsystem3/b3_line.c test_search.c -lm -o $(OUT)/test_search
	@$(OUT)/test_search

# the VehicleController's junction turn (vc_turn.c) with
# Buddy 3's line following, at 25, 50 (the car's) and 80 mm/s.
TURN_SRC := motion_mock.c $(APP)/vehicle_controller/vc_turn.c \
            $(APP)/subsystem3/b3_line.c test_turn.c
turn: | $(OUT)
	@echo ""
	@echo "=== junction turns (VC-H01) ==="
	$(CC) $(CFLAGS) $(INC) $(TURN_SRC) -lm -o $(OUT)/test_turn
	@$(OUT)/test_turn
	$(CC) $(CFLAGS) $(INC) -DB3_BASE_SPEED_MM_S=25 $(TURN_SRC) -lm \
	    -o $(OUT)/test_turn_25
	@$(OUT)/test_turn_25
	$(CC) $(CFLAGS) $(INC) -DB3_BASE_SPEED_MM_S=80 $(TURN_SRC) -lm \
	    -o $(OUT)/test_turn_80
	@$(OUT)/test_turn_80

# Buddy 3's smooth line following (B3-H10) on straights,
# bends, S-curves, a circle and a hairpin, with Buddy 2's 20 ms goal, ramp
# and a 60 ms wheel lag. IRs 15 mm (the default) and 20 mm off the centre
# must pass; 10 mm (1 mm of white each side of 18 mm tape) is shown only.
# Bends tighter than R140 are shown, LIMIT when the line is lost.
FOLLOW_SRC := motion_mock.c $(APP)/subsystem3/b3_line.c test_follow.c
follow: | $(OUT)
	@echo ""
	@echo "=== line following (B3-H10) ==="
	$(CC) $(CFLAGS) $(INC) $(FOLLOW_SRC) -lm -o $(OUT)/test_follow
	@$(OUT)/test_follow -n
	@$(OUT)/test_follow -s 20 -n
	@echo "smoothness (reversals per metre), shown only:"
	@$(OUT)/test_follow || true
	@echo "wheels turn the car 1/1.6 as tightly as asked (slip, caster), shown only:"
	@$(OUT)/test_follow -t 0.6 -n || true
	@echo "IRs 10 mm off the centre (1 mm of white each side), shown only:"
	@$(OUT)/test_follow -s 10 -q || true
	@# B3-H11, Buddy 2's PI wheel loop on a motor fitted
	@# to the car's curve log (15 % more drive per mm/s than its
	@# feed-forward, turning load 20 pm): must pass
	@echo "=== line following, wheels fitted to the car's curve log (B3-H11) ==="
	@$(OUT)/test_follow -m 50,1.15 -y 20,10 -n
	@$(OUT)/test_follow -m 50,1.15 -y 20,10 -c -q | tail -3
	@echo "=== both IRs black for 60 mm inside a bend (B3-H12), shown only ==="
	@$(OUT)/test_follow -x 200,60 -n -q || true
	@$(OUT)/test_follow -m 50,1 -x 200,60 -n -q || true

clean:
	rm -rf $(OUT)
