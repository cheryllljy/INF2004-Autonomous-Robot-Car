<#
  robot.ps1 - super simple MQTT control for the IS21 robot car.

  Put this file in the repo folder (next to broker_port.conf), then in
  PowerShell:

      powershell -ExecutionPolicy Bypass -File .\robot.ps1 check
      powershell -ExecutionPolicy Bypass -File .\robot.ps1 broker   (window 1, leave open)
      powershell -ExecutionPolicy Bypass -File .\robot.ps1 test     (window 2: [TEST]/[VC] lines from the car)
      powershell -ExecutionPolicy Bypass -File .\robot.ps1 watch    (window 3, optional)
      powershell -ExecutionPolicy Bypass -File .\robot.ps1 go       (reset + start)
      powershell -ExecutionPolicy Bypass -File .\robot.ps1 stop

  Commands
      check   is mosquitto installed, which IP is this laptop, does it
              match MQTT_BROKER_IP in config\wifi_secrets.h
      broker  start the broker with broker_port.conf
      test    print the car's test and VehicleController log lines
              (topic inf2004/robot/test: barcode scans, rejects, junctions,
              E-stops). Start it before powering the car (QoS 0, not kept).
      watch   print everything the robot publishes (inf2004/#)
      start | stop | reset    send one command
      go      reset, wait 2 s, start (use this after every power-up)

  The robot ignores a command whose sequence number is not newer than
  the last one, so this script remembers it in %TEMP%\inf2004_seq.txt
  and adds 1 every time. You never type the number.

  Payload (Buddy 1, FR2): 3 bytes = version 1, command (1 start, 2 stop,
  3 reset), sequence 0..255.
#>
param(
    [Parameter(Position = 0)]
    [ValidateSet('check', 'broker', 'test', 'watch', 'start', 'stop', 'reset', 'go')]
    [string]$Cmd = 'check',
    [string]$Broker = '',
    [int]$Port = 1883,
    [string]$Mosquitto = 'C:\Program Files\mosquitto'
)

$ErrorActionPreference = 'Stop'
$Topic   = 'inf2004/robot/command'
$TestTopic = 'inf2004/robot/test'
$SeqFile = Join-Path $env:TEMP 'inf2004_seq.txt'
$Repo    = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
$Secrets = Join-Path $Repo 'config\wifi_secrets.h'
$Pub     = Join-Path $Mosquitto 'mosquitto_pub.exe'
$Sub     = Join-Path $Mosquitto 'mosquitto_sub.exe'
$Exe     = Join-Path $Mosquitto 'mosquitto.exe'

function Get-FirmwareBroker {
    if (Test-Path $Secrets) {
        # Only the live #define line: a commented-out old IP above it was being picked up.
        $m = Select-String -Path $Secrets -Pattern '^\s*#define\s+MQTT_BROKER_IP\s+"([0-9.]+)"' | Select-Object -First 1
        if ($m) { return $m.Matches[0].Groups[1].Value }
    }
    return ''
}

function Get-BrokerIp {
    if ($Broker -ne '') { return $Broker }
    $fw = Get-FirmwareBroker
    if ($fw -ne '') { return $fw }
    return '172.20.10.5'
}

function Next-Seq {
    $s = 0
    if (Test-Path $SeqFile) {
        $t = (Get-Content $SeqFile -Raw).Trim()
        if ($t -match '^\d+$') { $s = [int]$t }
    }
    $s = ($s + 1) % 256
    Set-Content -Path $SeqFile -Value $s
    return $s
}

function Send-Robot([int]$code, [string]$name) {
    if (-not (Test-Path $Pub)) { throw "mosquitto_pub not found in $Mosquitto (winget install -e --id EclipseFoundation.Mosquitto)" }
    $ip  = Get-BrokerIp
    $seq = Next-Seq
    $f   = Join-Path $env:TEMP "inf2004_$name.bin"
    [System.IO.File]::WriteAllBytes($f, [byte[]](1, $code, $seq))
    & $Pub -h $ip -p $Port -t $Topic -f $f
    if ($LASTEXITCODE -ne 0) { throw "could not reach the broker at ${ip}:$Port - is 'robot.ps1 broker' running?" }
    Write-Host ("sent {0,-5} (seq {1}) to {2}:{3}" -f $name.ToUpper(), $seq, $ip, $Port)
}

switch ($Cmd) {
    'check' {
        Write-Host "mosquitto      : $(if (Test-Path $Exe) { 'installed' } else { 'NOT FOUND - winget install -e --id EclipseFoundation.Mosquitto' })"
        $fw = Get-FirmwareBroker
        Write-Host "firmware broker: $(if ($fw) { $fw } else { 'not found (run this from the repo folder)' })  (config\wifi_secrets.h)"
        $ips = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
               Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } |
               Select-Object -ExpandProperty IPAddress
        Write-Host "this laptop    : $($ips -join ', ')"
        if ($fw -and ($ips -notcontains $fw)) {
            Write-Host ""
            Write-Host "MISMATCH: the robot will look for the broker at $fw, but this laptop is not $fw." -ForegroundColor Yellow
            Write-Host "Connect the laptop to the same 2.4 GHz hotspot as the robot, or change MQTT_BROKER_IP" -ForegroundColor Yellow
            Write-Host "in config\wifi_secrets.h to one of the addresses above, then rebuild and flash." -ForegroundColor Yellow
        } elseif ($fw) {
            Write-Host "OK: the robot's broker address is this laptop." -ForegroundColor Green
        }
    }
    'broker' {
        $conf = Join-Path $Repo 'broker_port.conf'
        if (-not (Test-Path $conf)) { throw "broker_port.conf not found in $Repo" }
        Write-Host "broker on port $Port - leave this window open (Ctrl+C to stop)"
        & $Exe -c $conf -v
    }
    'test' {
        if (-not (Test-Path $Sub)) { throw "mosquitto_sub not found in $Mosquitto (winget install -e --id EclipseFoundation.Mosquitto)" }
        $ip = Get-BrokerIp
        Write-Host "car test log on ${ip}:$Port topic $TestTopic (Ctrl+C to stop)"
        Write-Host "start this before powering the car; lines are not stored by the broker"
        & $Sub -h $ip -p $Port -t $TestTopic
    }
    'watch' {
        $ip = Get-BrokerIp
        Write-Host "watching inf2004/# on ${ip}:$Port (Ctrl+C to stop); telemetry is binary, so some lines look like noise"
        & $Sub -h $ip -p $Port -t 'inf2004/#' -v
    }
    'start' { Send-Robot 1 'start' }
    'stop'  { Send-Robot 2 'stop' }
    'reset' { Send-Robot 3 'reset' }
    'go' {
        Send-Robot 3 'reset'
        Start-Sleep -Seconds 2
        Send-Robot 1 'start'
    }
}
