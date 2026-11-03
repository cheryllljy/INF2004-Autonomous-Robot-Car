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


## How to work this:
install submodules in own local folder:
wsl -d ubuntu
cd /home/jace/pico/pico-sdk
git submodule update --init --recursive

''
build and make the file: make clean && make WIFI=cyw43 NET=lwip MQTT=1 TEST=0 -j12
''
run this command on POWERSHELL (ADMIN) to download mosquitto broker: 
install: winget install -e --id EclipseFoundation.Mosquitto
check if installed: & "C:\Program Files\mosquitto\mosquitto.exe" --help
to start the broker: & "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\<'path to your project folder'>\INF2004-Autonomous-Robot-Car\broker_port.conf" -v
''
On another POWERSHELL (ADMIN), run:
Reset
[System.IO.File]::WriteAllBytes("$env:TEMP\reset.bin", [byte[]](1,3,84))
& "C:\Program Files\mosquitto\mosquitto_pub.exe" -h <Your broker ip> -p 1883 -t "inf2004/robot/command" -f "$env:TEMP\reset.bin"
NOTE: the 84 need to keep increasing, cant use the same number all the time
Then start
[System.IO.File]::WriteAllBytes("$env:TEMP\start.bin", [byte[]](1,1,100))
& "C:\Program Files\mosquitto\mosquitto_pub.exe" -h  172.20.10.12 -p 1883 -t "inf2004/robot/command" -f "$env:TEMP\start.bin"

For the second parameter,
1 = start
2 = stop
3 = reset