# Subsystem 5 calibration (v3.0)

About 5 minutes, once per robot, and again if the servo horn or the
sensor bracket is moved. The robot stays still the whole time.

## Why

The servo's "90 degrees" is not straight ahead, and you can't fix that by
hand. The horn sits on teeth about 15 degrees apart, so re-seating it
always leaves an error. On top of that:

- **Backlash.** The gears stop a little short, on the side they came
  from. Turning left and turning right land on different angles.
- **Beam width.** The HC-SR04 hears anything in a cone of roughly
  ±7.5–15 degrees. An object looks wider than it is, and its edges and
  centre move.
- **Range error.** Every sensor reads a little long or short.

The calibration measures all four and prints the numbers. The code then
corrects every servo move and every distance.

## What you need

- A thin post: a pen or a straw, standing up. Write down its diameter.
  The default is 10 mm (`S5_CAL_POST_DIAMETER_MM` in `s5_config.h`).
- A flat board, at least 30 cm wide: a book or a piece of cardboard.
- A tape measure.
- Tape to mark the car's centre line on the floor.

## Steps

1. In `app_program/subsystem5/s5_config.h`, set
   `#define S5_CAL_BUILD (1)`. Build and flash.
   (Or flash `IS21_S5_calibration_v3.0.uf2` from the release zip.)
2. Open the serial monitor at 115200.
3. Follow the prompts. Each one gives you 20 seconds.

| Step | You do | It does |
|---|---|---|
| 1/3 | Put the post on the centre line, **300 mm** from the sensor face. Keep everything else at least 30 cm away. | Creeps the servo 60→120° and back, twice, 1° at a time. |
| 2/3 | Put the board square to the sensor at **100**, then **200**, **300** and **500 mm**. Measure from the sensor face. | Takes 20 pings at each distance and fits a straight line. Shows PASS/FAIL against NFR15 (±10 mm). |
| 3/3 | Put the post back at 300 mm on the centre line. | Switches the new values on and checks the post now reads as straight ahead. |

4. It prints a block like this, every 15 seconds:

```
=== S5 CALIBRATION RESULT: step1 PASS, step2 PASS, step3 PASS ===
---- paste into app_program/subsystem5/s5_calib.h ----
#define S5_CAL_DONE                 (1)
#define S5_CAL_SERVO_TRIM_DDEG      (-35)
#define S5_CAL_SERVO_BACKLASH_DDEG  (15)
#define S5_CAL_RANGE_SCALE_Q12      (3983)
#define S5_CAL_RANGE_OFFSET_MM      (7)
#define S5_CAL_BEAM_HALF_DDEG       (110)
```

5. Copy those values into `s5_calib.h`, replacing the defaults. Keep
   the `#ifndef` / `#endif` lines around each one.
6. Measure with a ruler how far the sensor face sits in front of the
   servo shaft. Put that in `S5_CAL_SENSOR_ARM_MM`.
7. Set `S5_CAL_BUILD` back to `(0)`. Build and flash the normal image.
8. Run the TEST=1 image. `S5-U14` should say `calibration: DONE`.
9. Add a row to `TEST_LOG.md` (test S5-M05) with the printed values.

## If a step fails

| Message | Usually means |
|---|---|
| `post not heard on every sweep` | Post too far off the centre line, too thin, or not 300 mm away. |
| `trim / backlash / beam out of range` | Something else is inside the beam (a wall, your hand), or the post is too wide. |
| `no stable echo at N mm` | Board not square to the sensor, or too small. |
| `spread over 10 mm` | Board moving, or at an angle. |
| `FAIL NFR15` after the fit | The distances were measured from the wrong point. Measure from the sensor face. |

## Units

- `DDEG` = 0.1 degree. So `-35` is −3.5°.
- `SCALE_Q12`: 4096 = ×1.000, so 3983 = ×0.972.
