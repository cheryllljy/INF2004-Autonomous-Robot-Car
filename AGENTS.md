# AGENTS.md: handoff notes for coding agents (Codex etc.)

Written 2026-11-05 by Claude for nyx (Buddy 3, team IS21). Read this first.

## Project
INF2004 autonomous line-following car. micro T-Kernel 3 (uT-Kernel) on a
Raspberry Pi Pico W (RP2040). Five subsystems ("buddies"):

| Dir (app_program/) | Owner | What |
|---|---|---|
| subsystem1/ | Buddy 1 | Wi-Fi + MQTT commands/telemetry, FR3 link-loss latch |
| subsystem2/ | Buddy 2 | Motors (PWM GP8-11), encoders (GP2 L, GP3 R), speed PI loop, stall faults |
| subsystem3/ | Buddy 3 (nyx) | Line sensors IR_R GP26 / IR_L GP27, barcode IR_B GP28, Code 39 decode, junctions |
| subsystem4/ | Buddy 4 | IMU (currently not started) |
| subsystem5/ | Buddy 5 (rewritten by nyx 2026-11-07) | HC-SR04P (TRIG GP16, ECHO GP17) on an SG90 servo (GP15): stops at 30 cm, sweeps the width, goes round the shorter side back onto the same line. Calibration: testing/subsystem5/CALIBRATION.md |
| vehicle_controller/ | shared | State machine tying them together (`VC_ENABLE 1`) |

nyx owns subsystem3. Editing other buddies' code is allowed, but mark every
change with a comment naming who/why/date.

## Build (WSL, from build_make/, NOT CMake)
Needs `PICO_SDK_PATH=$HOME/pico/pico-sdk`. (The stale root CMakeLists named `src` and include/*.h were
Buddy 5's old obstacle build; removed with the subsystem 5 rewrite, 2026-11-07.)

    make clean && make WIFI=cyw43 NET=lwip MQTT=1 -j12          # drive build -> mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
    make clean && make WIFI=cyw43 NET=lwip MQTT=1 TEST=1 -j12   # on-car tests, output over MQTT
    make clean && make TEST=1 -j12                               # on-car tests, USB serial only, no Wi-Fi
    make clean && make WIFI=cyw43 NET=lwip MQTT=1 S5CAL=1 -j12  # subsystem 5 calibration (testing/subsystem5/CALIBRATION.md)

App sources are listed in build_make/mtkernel_3/app_program/subdir.mk (old_src/ excluded).
Console: use `tm_printf` from <tm/tmonitor.h>, never printf. USB CDC 115200, mirrored on UART0 GP0/GP1.

## Host tests (no hardware, run before every change to subsystem3)
    cd testing/subsystem3/buddy3_host && python3 run_tests.py
Last result: 2057 core + 404 adapter checks pass. Real car captures live in
test_core.c (`test_trim_outer_bars`, `test_any_size`): keep them passing.

## Driving and watching the car (Windows PowerShell, repo root)
`robot.ps1` (local, nyx's test version):
    .\robot.ps1 broker   # window 1, mosquitto with broker_port.conf
    .\robot.ps1 test     # window 2, subscribes inf2004/robot/test ([TEST] and [VC...] lines)
    .\robot.ps1 go | stop
Broker IP is `MQTT_BROKER_IP` in config/wifi_secrets.h (currently 172.20.10.12, iPhone hotspot,
2.4 GHz). robot.ps1 reads only the live #define line. Console lines starting with "[TEST]" or
"[VC" are copied to MQTT by lib/libtm/sysdepend/pico_rp2040/tm_com.c + mqtt_utk.c.

## Rules that matter
- One GPIO IRQ handler for everyone: app_program/subsystem3/gpio_dispatch.c. Register pins
  with gpio_dispatch_register(); never call tk_def_int for IO_IRQ_BANK0 elsewhere.
- Line sensors straddle the line: both WHITE = on track; one black = steer toward it;
  both black = crossing/junction. Do not hardcode track shape (figure 8 is only the practice track).
- Barcodes: IR_B is DO-only (AO mode was removed; do not bring it back). Any printed size;
  decoding is ratio-based (b3_code39.c). Junction detection needs >= 80 mm/s measured speed.
- Keep changes uncommitted unless nyx asks; nyx commits and opens PRs.

## Current state (2026-11-05)
Main is at PR #29 (d0b5d52), which includes all barcode/MQTT/turning work. Uncommitted:
- b3_config.h: nyx lowered B3_BASE_SPEED_MM_S and B3_SCAN_SPEED_MM_S 60 -> 40.
  NOTE: below 80 mm/s junctions are never detected (B3_JUNCTION_MIN_SPEED_MM_S).
- robot.ps1: test command + broker-IP regex fix.

Barcode tuning in b3_config.h (all from real car scans of 'Z'):
B3_WIDE_NARROW_RATIO_MAX_PCT 550, B3_NARROW_SPREAD_MAX_PCT 400, B3_NEED_BOTH_STARS 0,
B3_TRIM_OUTER_BARS 1, B3_SCAN_SPEED_COMPENSATE 0 (recommended 1), slowdown over a barcode
held until IR_B is white for B3_SCAN_SLOW_WHITE_US. Blips < 200 us on IR_B are absorbed.

## Open issues
1. Right encoder (GP3) sometimes gives 0 ticks while the right wheel spins (car lifted) ->
   Buddy 2 STALL_R E-stop (fault bits 0x01). Pull-up enabled on GP2/GP3 in
   subsystem2/motion_hw.c; not yet confirmed on the car. Next: swap encoder plugs to
   split module vs wire/pin.
2. Barcode reliability target: 19/20 passes. On a reason-6 reject the VC prints all 29
   widths (`[VC]   widths ms ...`, B = bar, w = gap); turn each failing capture into a host
   test before changing decoder limits.
3. Car not appearing as an MQTT client on the broker (only the laptop's mosquitto_pub shows).
   Check same 2.4 GHz hotspot, MQTT build flashed, laptop IP, firewall on 1883.

## Diagnostics printed by the VC
- `[VC] barcode rejected (total N): E/29 elements, last element X us, reason R: ...`
  reasons 1 same colour twice, 2 too short, 3 too long, 4 much wider than narrowest,
  5 scan too long, 6 not Code 39, 7 white before too short, 8 white after too short.
- `[VC] Buddy 2 fault F: STALL_L/STALL_R/... speed L R, ticks L R`
- `[VC-S5] ...` obstacle lines (detected, sweep, width, side, bypass, back on the line; table in
  testing/subsystem5/CALIBRATION.md), `[VC] link lost while driving: ...`
Fault bits: 0x01 Buddy 2 stall, 0x08 S5 sensor not answering (FR25, E-stop), 0x10 Wi-Fi/MQTT link loss
(Buddy 1 FR3), 0x40 obstacle: set WITHOUT E-stop while no way round (car waits, looks again every 3 s),
latched E-stop when a bypass fails (e.g. encoder dead).
S5 host tests: `cd testing/subsystem5/host && make` (unit S5-H11..H20, car S5-H21, junction turns, B3-H10).
