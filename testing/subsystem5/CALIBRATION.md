# Subsystem 5 — calibration (nyx, 2026-11-07)

The obstacle logic measures an object's width from where the ultrasonic
echoes start and stop as the sensor turns. Four numbers about **this**
sensor, servo and car make that accurate. Measure them once; they hold for
any object afterwards (they describe the sensor and the car, not the box).
Measure again only if the sensor or its bracket is moved or swapped.

| Step | What it measures | `s5_config.h` value | Default |
|---|---|---|---|
| 1 | which way the servo turns, and where straight ahead really is | `S5_SERVO_INVERT`, `S5_SERVO_TRIM_DDEG` | 0, 0 |
| 2 | how far the range reads long or short | `S5_RANGE_OFFSET_MM` | 0 |
| 3 | the beam half-angle: how far past each corner of a flat face the echoes go on | `S5_BEAM_NEAR_DDEG` (box at 200 mm), `S5_BEAM_FAR_DDEG` (box at 300 mm) | 220, 220 (22.0 deg) |
| 4 | how far the car really turns for what the encoders say | `S5_TURN_PCT` | 100 |
| 4 | the compass (Buddy 4's IMU), read on the same circle: the field's centre, its strength and which way it turns | `S5_MAG_SIGN`, `S5_MAG_X0`, `S5_MAG_Y0`, `S5_MAG_R` | 0 (no compass) |

Angles are in tenths of a degree (`ddeg`): 220 means 22.0 deg.

Step 3 matters most. If the beam is set too wide, objects read too narrow
and the car passes closer than planned; too narrow, and they read too wide
(safe, but the car swerves further than it needs to). Each degree of beam
error moves each edge about 5 mm at 250 mm.

## What you need

- nyx's box (21 x 16 cm), its **21 cm side** facing the car.
- A tape measure. Measure distances from the **front of the sensor** (the
  two metal cans), not from the car's bumper.
- For step 4: a clear floor patch about 50 cm across, and a strip of tape.

## Run it

1. Build the calibration image (it replaces the obstacle logic; nothing
   else changes):
   ```sh
   cd build_make
   make S5CAL=1 -j8                              # USB serial only
   # or, to read the results over MQTT when the car is unplugged:
   make WIFI=cyw43 NET=lwip MQTT=1 S5CAL=1 -j8
   ```
   The image has `_s5cal` at the end of its name, e.g.
   `mtk3pico_smp1_usb_cdc_wifi_s5cal.uf2`.
   nyx, 2026-11-08: `S5CAL=2` instead of `S5CAL=1` runs step 4 only (the
   circle, with the compass); its image ends in `_s5cal4`, e.g.
   `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s5cal4.uf2`.
2. Flash it and open the serial console (115200), or watch the test topic:
   `mosquitto_sub -h <broker> -t inf2004/robot/test` (every `[VC-S5] CAL`
   line is mirrored there).
3. Follow the prompts. Before each step the console prints what to set up
   and counts down 15 s (`measuring in 15 s`, `10 s`, `5 s`). Do not touch
   the car or the box while it measures.

### Step 1: servo direction and trim

- The sensor turns **45 deg to the car's LEFT**, then **45 deg to the
  RIGHT**, then straight ahead. If it went right first, left and right are
  swapped: set `S5_SERVO_INVERT` to the value it prints and run the
  calibration again from the start.
- Then: put the box straight ahead of the car, face square to the car,
  300 mm from the sensor. It sweeps and prints
  `step 1 result: the box's middle reads N (0.1 deg) ... set
  S5_SERVO_TRIM_DDEG to T`. Copy T only if the box really was centred
  (sight along the car's middle; a box 10 mm off centre at 300 mm is
  about 2 deg).

### Step 2: range

- Box face square to the sensor, **200 mm** from the sensor's front.
- It takes 20 pings and prints the median and
  `set S5_RANGE_OFFSET_MM to N`. Step 3 already uses this correction.

### Step 3: beam half-angle (twice)

- `NEAR`: the 21 cm side facing the car, centred, face **200 mm** from
  the sensor. It sweeps and prints
  `set S5_BEAM_NEAR_DDEG to N (now M; with it the box read W mm wide)`.
- `FAR`: the same at **300 mm**: `set S5_BEAM_FAR_DDEG to N`.
- If it says an edge is outside the sweep, the box was not centred:
  centre it and run the calibration again.
- Expect roughly 150 to 300 (15 to 30 deg) on a cardboard box. With the
  new values in place, the same box reads 210 mm wide within about
  10 mm.

### Step 4: how far the car turns

- Put the car on the floor with room for a 40 cm circle. Lay a strip of
  tape along its side so you can see where it pointed at the start.
- It drives **one circle to the left** at 60 mm/s, by its encoders, then
  stops and prints `the encoders say N deg` and how to set `S5_TURN_PCT`
  from its current value P (100 until you change it):
  - it stopped **short** of where it started pointing by A deg:
    `S5_TURN_PCT = P x (360 - A) / 360`
  - it went **past** by A deg: `S5_TURN_PCT = P x (360 + A) / 360`
  - Example: P = 100 and it stopped 20 deg short: 100 x 340 / 360 = 94.
- Run it twice and use the average if the two differ by more than 3.
- nyx, 2026-11-08: on the same circle it reads Buddy 4's compass (the
  IMU must be plugged in) and prints
  `CAL compass: set S5_MAG_SIGN S, S5_MAG_X0 X, S5_MAG_Y0 Y, S5_MAG_R R`,
  then `it turned C deg by the compass, E by the encoders; at most W deg
  apart on the way round` and one verdict:
  - `good; by it S5_TURN_PCT would be N (now P)`: copy the four numbers.
    N is a second opinion on `S5_TURN_PCT` (the compass does not drift
    with the wheels); if it and the tape mark disagree by more than 3,
    run the circle again.
  - `more than 10 deg apart: ...`: iron or wires near the IMU bend the
    field. Leave `S5_MAG_SIGN` 0 (no compass), or copy the numbers and set
    `S5_MAG_USE` 0 to only log what it reads.
  - `the field hardly changed as the car turned: no compass`, or `only N
    samples from Buddy 4's IMU`: the IMU is not answering; leave
    `S5_MAG_SIGN` 0.
  Do the circle where the car runs (the floor, not the desk), away from
  laptops and steel table legs: the numbers are for the car, but a big
  piece of iron nearby bends the field the car sees.

## Put the numbers in

1. Edit `app_program/subsystem5/s5_config.h`, the lines marked
   `CALIBRATE step N`, e.g.
   ```c
   #define S5_SERVO_TRIM_DDEG (-15)
   #define S5_RANGE_OFFSET_MM (6)
   #define S5_BEAM_NEAR_DDEG (205)
   #define S5_BEAM_FAR_DDEG (230)
   #define S5_TURN_PCT (94)
   #define S5_MAG_SIGN (-1)
   #define S5_MAG_X0 (-310)
   #define S5_MAG_Y0 (125)
   #define S5_MAG_R (2480)
   ```
2. Build and flash the normal image again (**without** `S5CAL`).
3. Check: put the box on the line 30 cm ahead and start the car. The
   console prints `[VC-S5] object W mm wide: L mm left and R mm right of
   the line`. W should be 210 within about 10 mm. Another object can read
   wider or narrower than it is (the echoes go on past a flat face by much
   the same whatever its width): if the car swings far wider than needed
   or comes close to it, run step 3 with that object.
   nyx, 2026-11-08: on the track the 21 cm face read 355 mm with FAR 155
   (a value picked below what step 3 printed, to be safe); 235 reads it
   about 210 and the 16 cm face about 185. If W is far over the real
   width, raise `S5_BEAM_FAR_DDEG` towards what step 3 printed.
4. Record the run in `TEST_LOG.md` (S5-M09) with the values you used.

## Reading the bypass log

| Line | Meaning |
|---|---|
| `object N mm ahead: stopping (line from the last L mm, within W mm; car D deg to it)` | detected; the line's direction comes from the last L mm the IRs followed |
| `sweep: P pings, face F mm, echoes from A to B deg, beam C deg` | the sweep; C is the calibrated beam at that distance |
| `object W mm wide: L mm left and R mm right of the line` | the width and where the object sits on the line |
| `going LEFT/RIGHT: axle Y mm off the line, ramps ...` | the plan: the shorter side (equal sides: random) |
| `only N mm away: backing off to 250 mm first` | too close to sweep; reverses first |
| `backing up N mm first, for room to turn out` | too close to swerve round; reverses, then goes round at once |
| `far corner: object D mm deep, its side S mm from the sweep's` | measured while passing; S far from 0 on a square box means step 3 is off |
| `side (far corner) N mm closer than planned: N mm further out` | the far side was nearer than the sweep said (e.g. a box turned towards the car); the car moved out |
| `far corner not fixed: back in after N mm of object (S5_DEPTH_DEFAULT_MM 200)` | the look across could not fix the far corner, so the car took the object as `S5_DEPTH_DEFAULT_MM` deep (or deeper, if it heard something further on). If your objects are deeper than 20 cm, raise it in `s5_config.h` |
| `another object just behind this one (N mm past its far corner): going on past both` | a second object close behind and clear of the pass lane: the car stays out and passes both |
| `another object ahead in the pass lane (N mm away): stopping` / `... ahead on the way back ...` | something the first object cannot explain: the car stops, sweeps again from where it is and goes round that too |
| `the car is N deg to the line: turning on the spot to straighten first` | stopped at an angle on the way back; a sweep found nothing within 200 mm, so it turns square to the line on the spot before the sweep that plans |
| `the car is N deg to the line, something within 200 mm: backing up to straighten first` | the same, with something close: it backs up 100 mm, steering straight, instead of turning on the spot |
| `sweep: N returns from the object just passed left out` | the new sweep still saw the object behind; those echoes are not the new object, and not clear either |
| `..., left edge hidden by the object just passed` (or right) | that edge of the new object is next to echoes left out: the car does not go round on that side |
| `...: back to the line from N mm off it` | the new object has gone, or was not in the way: straight back onto the line |
| `IRs found the line (edge), N mm from where the encoders put it` | coming back; a large N (over ~20) means step 4 is off |
| `both IRs crossed the line (N mm from where the encoders put it, D deg to it): onto it and facing along it` | nyx, 2026-11-08: both IRs black at once, or one and then the other within 150 mm of driving, near where the line should be: the car drives on until its axle is on the line, then turns on the spot until the tape is between the IRs, and hands over (`back on the line (picked up by the IRs...)`) |
| `by where the two IRs met it, the car is D deg to the line (the encoders said E)` | nyx, 2026-11-08: after `both IRs crossed`: the two points where the IRs met the tape give the line's real direction, and the car is D deg to it; the car drives on and turns by that. D far from E means the encoders' heading drifted on the way round. (Earlier builds printed `by the IRs' timing ...`, from how far the car went between the two, which is wrong while the car turns) |
| `the IRs crossed a line N mm from where the encoders put ours, at D deg to it: a junction, not our line; driving on` | nyx, 2026-11-08: the tape the IRs crossed runs D deg away from our line's direction (more than `S5_CAP_LINE_DEG` 20): a junction's branch, not our line. The IRs are ignored until both have been on white for 20 mm, and the car carries on towards where the line should be |
| `both IRs black at once N mm from where the encoders put the line, car D deg to it: a line across ours (junction); driving on` | nyx, 2026-11-08: both IRs went black together while the car was less than `S5_CAP_BOTH_DEG` 45 deg to our line; a line parallel to ours cannot do that below 53 deg, so it is a line across ours. Ignored as above |
| `back on the line (picked up by the IRs, turned to where they had it run)` | nyx, 2026-11-08: the turn on the spot after `axle on the line` went by the direction the two IRs measured: it stops 3 deg past straight. The IRs only count within 6 deg of straight, so a junction's branch touching an IR earlier in the turn does not cut it short. `... the turn took too long, Buddy 3 looks` means it did not get round in time and Buddy 3 takes over from there |
| `the line is not where the encoders put it: looking on at D deg` | also printed when the car was turning onto a line the IRs had found and it was not there after all (a junction): it looks on at D deg |
| `compass not calibrated (S5_MAG_SIGN 0): encoders only; ...` | nyx, 2026-11-08: once after power-up: step 4 has not been run (or said no compass), so the heading comes from the encoders alone |
| `compass: heading taken at the stop (N samples, field P percent of the calibration's)` | standing at the first stop, where the IRs had just measured the car's angle to the line, the compass's reading is matched to that angle; from then on it tells the car's angle to the line without the wheels. P far from 100 (over 25 off) means iron nearby or the calibration is old |
| `compass: no good sample at the stop (field P percent of the calibration's): encoders only` | the field at the stop was not like the calibration's (iron in or under the box, or the IMU not answering): this bypass goes by the encoders alone |
| `compass: A deg taken out of the encoders' heading so far, now B deg from it (N samples skipped)` | printed at the end of a bypass and before the turn onto the line: A is how far the encoders' heading had drifted and was pulled back; B is what is left (the compass leaves the encoders alone while the two are within `S5_MAG_DEAD_DEG` 3 deg, so B up to about 3 is normal). Samples are skipped where the field is over 25 % off the calibration's (iron) or the compass and encoders are more than 30 deg apart. A over ~10 on every run: run step 4 again (`S5_TURN_PCT`) |
| `axle on the line (N mm on): turning on the spot to face along it` | the pick-up drove N mm on to put the axle over the tape and now turns until the tape is between the IRs; then `back on the line (picked up by the IRs...)` |
| `the line is not where the encoders put it: looking on at 30 deg` | the way back ended with no IR on the line: it carries on at 30 deg towards it for up to 400 mm. `no black on the way back: the first line ahead is ours` after it means no IR saw black at all, so the first line it meets is taken as ours however far the encoders have drifted |
| `left side blocked by an echo at A deg, N mm` (or right) | nyx, 2026-11-08: printed after `... left side blocked`: the echo that ruled that side out. Its own direction is where the car would pass (up to the car's width plus 20 mm out) |
| `left side maybe blocked (beam edge) by an echo at A deg, N mm` | nyx, 2026-11-08: the echo's own direction is outside where the car would pass, but the sonar cannot tell an echo's direction closer than its beam (A +- the beam), and that much would reach it. Such a side is used only when the other side is blocked outright: `no side clear: going LEFT, where an echo is in the pass lane only at the edge of its beam; watching the lane`. Both sides only "maybe" (walls either side): it waits |
| `back on the line: line following again` | handed back to Buddy 3 |
| `no way round (...): waiting here with the obstacle fault bit 0x40 set, no E-stop` | blocked both sides; it looks again every 3 s and carries on when the way ahead is clear (`still no way round after N s` every look) |
| `bypass failed: ...` | gave up (e.g. `the line was not found (FR24)`, `lost the line after the bypass`, a dead wheel, `another object again: S5_NEXT_MAX in one bypass` when a fifth other object turns up in one bypass): E-stop with fault bit 0x40 |
| `sensor fault (FR25): ...` | the sensor stopped answering: E-stop |
