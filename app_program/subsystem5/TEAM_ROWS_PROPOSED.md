# Proposed TEAM.md rows for subsystem 5

`TEAM.md` is shared, so these rows are NOT applied yet. Pins and empty
rows may be claimed by anyone (TEAM.md section 1); the priority and the
interfaces need team agreement. Copy them in once agreed.

## Section 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 5 | ObstacleScan: ultrasonic + servo, obstacle profiling, bypass side | 2502410 | | `app_program/subsystem5/` | `testing/subsystem5/` |

## Section 2. Pins

| Pin | Owner | Used for |
|---|---|---|
| GP15 | S5 (Buddy 5) | Servo signal, SERVO 4 header, PWM slice 7 B |
| GP16 | S5 (Buddy 5) | HC-SR04 TRIG (GPIO output) |
| GP17 | S5 (Buddy 5) | HC-SR04 ECHO via level converter, PWM slice 0 B input (no IRQ) |

Also note: GP14 must not be used as PWM (it shares slice 7 with the servo).

## Section 3. Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| PWM slices (list) | S5 (Buddy 5): slice 0 (echo timing), slice 7 (servo) | `obstacle` | GP15, GP17 |

S5 releases the PWM block from reset at init (idempotent; `config.h` has
`USE_PTMR = 0`). Buddy 2's motors need the same release.

## Section 4. Task priorities

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 5 | 20–21 | obstacle – 20 – any |

**Needs a team decision:** the Week 6 design ranks ObstacleScan second
(below MotionControl). With the template bands it is last. Agree one
mapping and change `S5_PRI_OBSTACLE` in `subsystem5.h` only.

## Section 5. Interfaces

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-S5-1 | S5 → VehicleController | event flag (owned by S5) | `flg_obstacle` | DETECTED, HARD_STOP, PROFILED, CLEARED, FAULT; wait with `TWF_ORW \| TWF_BITCLR` | proposed |
| IF-S5-2 | VehicleController → S5 | function → flag bit | `obstacle_resume()`, `obstacle_reset_fault()` | bypass done / leave latched fault | proposed |
| IF-S5-3 | S5 → any task | lock-free word; mutex copy | `obstacle_get_front_mm()`, `obstacle_get_profile()` | mm / CLEAR 0xFFFE / UNKNOWN 0xFFFF; profile | proposed |
| IF-S5-4 | S2 → S5 | callback registered in `_init()` | `obstacle_register_odometry()` | speed mm/s, distance mm, non-blocking | proposed |
| IF-S5-5 | S5 → E-stop owner | sets one bit, never clears | `obstacle_register_estop()` | sensor fault only | proposed |
| IF-S5-6 | S5 → Telemetry (S1) | function | `obstacle_get_telemetry()` | 29 B binary, layout in `obstacle_model.h` | proposed |

## Section 6. Kernel object budget (S5 column)

| Object | S5 |
|---|---|
| Task | 1 |
| Semaphore | 1 |
| Event flag | 1 |
| Mutex | 1 |
| Cyclic handler | 1 |
