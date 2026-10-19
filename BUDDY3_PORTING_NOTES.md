# Buddy 3 additive import — read before activating

## Imported code

Six Buddy3 `.c` files and seven Buddy3 `.h` files were added inside the
existing `app_program/subsystem3/` folder:

- `b3_ir.c` / `.h`: GPIO sampling and barcode edge queue
- `b3_line.c` / `.h`: line detection and speed requests
- `b3_barcode.c` / `.h`: barcode collection / validation
- `b3_code39.c` / `.h`: Code39 decoding
- `b3_port_mtk3_rp2040.c` / `b3_port.h`: micro T-Kernel / RP2040 adapter
- `buddy3.c` / `buddy3.h`: worker task, context, output snapshots and navigation queue
- `b3_config.h`: pins, task priority, validation thresholds

`buddy3_team_config.h` is new and governs the optional start call.
The original consumer/statistics example and all other subsystems remain as-is.
Buddy3 algorithms are carried over unchanged. Two imported files have
team-integration adjustments: `b3_config.h` and `b3_port_mtk3_rp2040.c`;
`buddy3_team_config.h` is a new opt-in switch. The copied host-test mock was
adjusted to provide the kernel API `size_t` and use the configured barcode
electrical polarity, without changing the real firmware algorithm.

## What is disabled by default — intentionally

`#define B3_TEAM_AUTOSTART 0` in `app_program/subsystem3/buddy3_team_config.h`.
With that default, Buddy3 is **compiled and linked**, but it is NOT started.
This preserves the team's current runtime behaviour while pin ownership and
vehicle-controller interfaces are unresolved. This is not yet a fully running
Buddy3/vehicle-controller integration.

When the team agrees on the pins and shared interrupt ownership, switch to
`#define B3_TEAM_AUTOSTART 1`, then rebuild. The `subsystem3_start()` hook
will call `buddy3_start(NULL)` without replacing the existing consumer task.
This starter does not spoof motion/encoder feedback.

## Hardware and integration precautions

1. **GPIO pins:** IR_R = GP26, IR_L = GP27, IR_B = GP28
   (`B3_IR_R_GPIO`, `B3_IR_L_GPIO`, `B3_IR_B_GPIO` in `b3_config.h`).
   IR_L and IR_R follow the line; IR_B reads barcodes by edge interrupt
   and confirms junctions. The DO outputs are read digitally. The kernel
   ADC driver (`USE_SDEV_DRV=1`) does not reconfigure these pads
   (`DEVCONF_ADC_PIN_INIT_n` are FALSE), so the test build also reads them
   as ADC0–2 for a self-check.
2. **Debug outputs:** removed. Buddy 3 drives no output pins; the old
   diagnostic LEDs on GP2/3/5/6/16/17 are gone (GP2/GP3 are the encoders).
   Use `make TEST=1` and the `[TEST] B3-…` lines instead (see
   `testing/subsystem3/TEST_LOG.md`).
3. **Priority:** `B3_TASK_PRIORITY=17` is in the current template's S3
   priority band (16–17); the existing S3 consumer remains priority 16.
4. **GPIO interrupt:** `B3_INSTALL_IO_BANK0_HANDLER=1` assumes nobody else
   installed the RP2040 IO_IRQ_BANK0 handler. `app_program/old_src/` contains
   old GPIO interrupt code but is not part of the current auto-build.
   Before enabling other edge-based peripherals, agree on ONE shared IRQ
   dispatcher. If another owner exists, set the flag to `0` and have its
   handler call `b3_port_dispatch_barcode_irq()`.
5. **Sensor voltage:** Pico W GPIO accepts 3.3 V logic, not 5 V. Check
   the sensor modules' DO voltage before connecting. The barcode polarity,
   widths, junction assumptions and other thresholds still need calibration.
6. **Navigation commands:** `B3_ENABLE_EXAMPLE_NAV_MAP=0` remains unchanged;
   code does not invent A/B/C/D turn meanings. Buddy3 does not drive motors.

## Vehicle controller work still required

There is no confirmed vehicle-controller interface in the uploaded first ZIP
that can be wired up without modifying or guessing teammates' code. Until the
vehicle controller supplies a fresh `b3_context_t`, Buddy3 keeps outputs
failsafe (zero requested speeds / stop_required). Its required APIs are:

```c
#include "../subsystem3/buddy3.h"
#include "../subsystem3/b3_port.h"

b3_context_t context = {0};
context.time_us = b3_port_time_us();
context.follow_enabled = true;  /* only when controller allows following */
context.scan_enabled = true;    /* only when route/scan mode allows scanning */
/* Supply real measured motion and IMU fields if available; do not fabricate. */
buddy3_set_context(&context);

b3_snapshot_t snapshot;
buddy3_get_snapshot(&snapshot);
/* Feed snapshot.line.left_mm_s / right_mm_s into the VEHICLE CONTROLLER,
 * respecting obstacle/safety/turn precedence. */

b3_nav_event_t event;
while (buddy3_pop_navigation(&event)) {
    /* route event to the navigation controller; obey navigation_allowed */
}
```

Controller update is expected every ~10–20 ms. Pass a nonblocking encoder
position callback to `buddy3_start()` once one is agreed upon; until then the
installed hook uses `NULL`.

## Build and verification

Inside WSL Ubuntu with `PICO_SDK_PATH` and `arm-none-eabi-gcc` installed:

```bash
cd build_make
make -j12
# Produces mtk3pico_smp1_usb_cdc_wifi.uf2 with default flags
```

For standalone portable and adapter regression tests:

```bash
python3 testing/subsystem3/buddy3_host/run_tests.py
```

Because the original first ZIP contains `.git`, its full Git repository is
preserved in the merged ZIP. Before committing, review `git diff` and use a
feature branch. There is no automatic push to GitHub.
