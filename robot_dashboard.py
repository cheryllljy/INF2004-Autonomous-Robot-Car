
"""Live MQTT dashboard for the INF2004 robot car (with test buttons).

Every run is recorded to runs/ next to this script (one .jsonl file per
run, named by its start time) and can be opened again from the
"Runs" section, with charts, a state timeline and the event table.
Set ROBOT_RUNS_DIR to record somewhere else.
"""

import ipaddress
import json
import os
import re
import time
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import altair as alt
import pandas as pd
import paho.mqtt.client as mqtt
import streamlit as st


ROOT = Path(__file__).resolve().parent
RUNS_DIR = Path(os.environ.get("ROBOT_RUNS_DIR") or (ROOT / "runs"))

TOPIC_MAPPING = "inf2004/robot/mapping"
TOPIC_CONNECTION = "inf2004/robot/connection"
TOPIC_TELEMETRY = "inf2004/robot/telemetry"
TOPIC_HEARTBEAT = "inf2004/robot/heartbeat"
TOPIC_TEST = "inf2004/robot/test"
TOPIC_RUN = "inf2004/robot/run"

# start/stop buttons for testing. The same FR2 command
# robot.ps1 sends (Buddy 1): 3 bytes = version 1, command, sequence
# 0..255. The car ignores a sequence that is not newer than the last one,
# so the counter is shared with robot.ps1 (%TEMP%\inf2004_seq.txt) and
# both can be used side by side.
TOPIC_COMMAND = "inf2004/robot/command"
FR2_VERSION = 1
CMD_START = 1
CMD_STOP = 2
CMD_RESET = 3
CMD_SEARCH = 4
SEQ_FILE = Path(
    os.environ.get("TEMP") or os.environ.get("TMP") or "."
) / "inf2004_seq.txt"
SEQ_LOCK = threading.Lock()

MAP_LINE = re.compile(
    r"^([A-Z])\s*->\s*(FORWARD|LEFT|RIGHT|U-TURN)$",
    re.MULTILINE
)

# run telemetry (15 bytes every 200 ms, bit layout in buddy1_cr1.c)
RUN_BYTES = 15
STATE_NAMES = {
    0: "IDLE",
    1: "LINE_FOLLOW",
    2: "TURNING",
    3: "OBSTACLE",
    4: "STOPPED",
    5: "ESTOP"
}
STATE_COLOURS = {
    "IDLE": "#9e9e9e",
    "LINE_FOLLOW": "#2e7d32",
    "TURNING": "#1565c0",
    "OBSTACLE": "#ef6c00",
    "STOPPED": "#6a1b9a",
    "ESTOP": "#c62828"
}
DRIVING_STATES = (1, 2, 3)
END_STATES = (0, 4, 5)
FAULT_NAMES = (
    (0x01, "motor stall"),
    (0x02, "encoder"),
    (0x04, "IMU / tilt"),
    (0x08, "no echo"),
    (0x10, "Wi-Fi/MQTT link lost"),
    (0x20, "line (lost / lifted / Buddy 3)"),
    (0x40, "obstacle (no way round)"),
    (0x80, "task fault")
)
DISTANCE_WRAP = 1 << 20
JUNCTION_WRAP = 64

# a run with no packet for this long is closed
RUN_GAP_S = 30.0
# event lines this close before a run starts or after it ends are also
# copied into the run file (the state change line and the packet that
# shows it can arrive in either order)
EVENT_MARGIN_S = 2.0

# event lines worth spotting in the run table, first match wins
EVENT_TAGS = (
    ("E-STOP", re.compile(r"e-?stop", re.IGNORECASE), True),
    ("FAULT", re.compile(r"fault", re.IGNORECASE), True),
    ("LINE LOST", re.compile(r"line lost", re.IGNORECASE), True),
    ("TILTED", re.compile(r"tilted", re.IGNORECASE), True),
    ("LIFTED", re.compile(r"car lifted", re.IGNORECASE), True),
    ("OBSTACLE", re.compile(
        r"^\[VC-S5\]|obstacle|no way round", re.IGNORECASE
    ), True),
    ("LINK LOST", re.compile(
        r"link lost|offline|disconnected", re.IGNORECASE
    ), True),
    ("HUMP", re.compile(r"terrain_report|hump", re.IGNORECASE), False),
    ("BARCODE", re.compile(r"barcode", re.IGNORECASE), False),
    ("JUNCTION", re.compile(r"junction", re.IGNORECASE), False),
    ("STATE", re.compile(r"state \d+ -> \d+"), False),
    ("COMMAND", re.compile(r"\bMQTT (START|STOP|RESET|SEARCH)\b"), False)
)
ALERT_TAGS = tuple(tag for tag, _, alert in EVENT_TAGS if alert)
TERRAIN_REPORT = re.compile(r"TERRAIN_REPORT")


def next_sequence():
    """Next command sequence number, shared with robot.ps1."""
    with SEQ_LOCK:
        value = 0

        try:
            raw = SEQ_FILE.read_bytes()
            text = raw.decode(
                "utf-16" if b"\x00" in raw else "ascii",
                errors="ignore"
            )
            match = re.search(r"\d+", text)

            if match is not None:
                value = int(match.group(0))
        except OSError:
            pass

        value = (value + 1) % 256

        try:
            SEQ_FILE.write_text(f"{value}\r\n", encoding="ascii")
        except OSError:
            pass

        return value


def load_broker_config():
    """Read the Pico MQTT IP and Mosquitto port from project files."""
    wifi_file = ROOT / "config" / "wifi_secrets.h"
    broker_file = ROOT / "broker_port.conf"

    if not wifi_file.is_file():
        raise FileNotFoundError(
            "Missing config/wifi_secrets.h"
        )

    if not broker_file.is_file():
        raise FileNotFoundError(
            "Missing broker_port.conf"
        )

    wifi_text = wifi_file.read_text(encoding="utf-8")
    broker_text = broker_file.read_text(encoding="utf-8")

    ip_match = re.search(
        r'^\s*#\s*define\s+MQTT_BROKER_IP\s+"([^"]+)"',
        wifi_text,
        re.MULTILINE
    )

    port_match = re.search(
        r"^\s*listener\s+(\d+)\b",
        broker_text,
        re.MULTILINE
    )

    if ip_match is None:
        raise ValueError(
            "MQTT_BROKER_IP not found in wifi_secrets.h"
        )

    if port_match is None:
        raise ValueError(
            "No listener port found in broker_port.conf"
        )

    broker_ip = str(ipaddress.ip_address(ip_match.group(1)))
    broker_port = int(port_match.group(1))

    if not (1 <= broker_port <= 65535):
        raise ValueError("Invalid MQTT broker port")

    return broker_ip, broker_port


def state_name(code):
    return STATE_NAMES.get(code, f"STATE_{code}")


def fault_names(bits):
    return [name for mask, name in FAULT_NAMES if bits & mask]


def decode_run_packet(payload):
    """Decode one 15-byte run packet into a dict of fields."""
    if len(payload) != RUN_BYTES:
        raise ValueError(
            f"run packet is {len(payload)} bytes, expected {RUN_BYTES}"
        )

    value = int.from_bytes(payload, byteorder="little")

    def unsigned(low, width):
        return (value >> low) & ((1 << width) - 1)

    def signed(low, width):
        raw = unsigned(low, width)

        if raw & (1 << (width - 1)):
            return raw - (1 << width)

        return raw

    state = unsigned(8, 3)
    faults = unsigned(16, 8)
    letter_code = unsigned(75, 5)

    if letter_code == 0:
        letter = ""
    elif letter_code <= 26:
        letter = chr(ord("A") + letter_code - 1)
    else:
        letter = "?"

    return {
        "seq": unsigned(0, 8),
        "state": state,
        "state_name": state_name(state),
        "wifi": bool(unsigned(11, 1)),
        "mqtt": bool(unsigned(12, 1)),
        "ir_l": bool(unsigned(13, 1)),
        "ir_r": bool(unsigned(14, 1)),
        "ir_b": bool(unsigned(15, 1)),
        "faults": faults,
        "fault_names": fault_names(faults),
        "speed_l": signed(24, 10),
        "speed_r": signed(34, 10),
        "distance": unsigned(44, 20),
        "range": unsigned(64, 11),
        "letter": letter,
        "junctions": unsigned(80, 6),
        "lift": unsigned(86, 7),
        # highest hump peak of the current run, reset by the car
        "hump": unsigned(93, 7),
        "uptime": unsigned(100, 20)
    }


def event_tag(text):
    """Short tag for an event line ("" when nothing matches)."""
    for tag, pattern, _ in EVENT_TAGS:
        if pattern.search(text):
            return tag

    return ""


class RunRecorder:
    """Write run packets and event lines to runs/ as they arrive.

    Each run goes to its own file, one JSON object per line, flushed
    straight away. Anything outside a run goes to idle_<date>.jsonl.
    """

    def __init__(self, folder):
        self.folder = Path(folder)
        self.lock = threading.Lock()
        self.clock = time.monotonic
        self.wall_clock = datetime.now
        self.run_file = None
        self.run_path = None
        self.run_start = None
        self.last_state = None
        self.last_packet = None
        self.ended_path = None
        self.ended_start = None
        self.ended_at = None
        self.idle_file = None
        self.idle_date = None
        self.recent = deque(maxlen=50)
        self.error = None

    def _open(self, path):
        self.folder.mkdir(parents=True, exist_ok=True)
        return open(path, "a", encoding="utf-8", newline="\n")

    def _write(self, handle, record):
        handle.write(json.dumps(record, separators=(",", ":")) + "\n")
        handle.flush()

    def _write_idle(self, wall, record):
        date = wall.strftime("%Y-%m-%d")

        if self.idle_file is None or self.idle_date != date:
            if self.idle_file is not None:
                self.idle_file.close()

            self.idle_file = self._open(
                self.folder / f"idle_{date}.jsonl"
            )
            self.idle_date = date

        self._write(self.idle_file, record)

    def _safely(self, action):
        try:
            action()
            self.error = None
        except OSError as error:
            self.error = f"Cannot write to {self.folder}: {error}"

    def _start_run(self, wall, now, trigger):
        stamp = wall.strftime("%Y-%m-%d_%H%M%S")
        path = self.folder / f"{stamp}.jsonl"
        number = 2

        while path.exists():
            path = self.folder / f"{stamp}_{number}.jsonl"
            number += 1

        self.run_file = self._open(path)
        self.run_path = path
        self.run_start = now
        self.ended_path = None

        self._write(self.run_file, {
            "kind": "run_start",
            "wall": wall.isoformat(timespec="milliseconds"),
            "t_ms": 0,
            "trigger": trigger
        })

        # event lines from just before the start belong to this run too
        for when, record in self.recent:
            if now - when <= EVENT_MARGIN_S:
                copy = dict(record)
                copy["t_ms"] = int((when - now) * 1000)
                copy["before_start"] = True
                self._write(self.run_file, copy)

    def _end_run(self, wall, now, state, reason):
        self._write(self.run_file, {
            "kind": "run_end",
            "wall": wall.isoformat(timespec="milliseconds"),
            "t_ms": int((now - self.run_start) * 1000),
            "state": state,
            "state_name": state_name(state) if state is not None else "",
            "reason": reason
        })
        self.run_file.close()
        self.ended_path = self.run_path
        self.ended_start = self.run_start
        self.ended_at = now
        self.run_file = None
        self.run_path = None
        self.run_start = None

    def sample(self, fields, lost):
        """Record one decoded run packet; start or end runs on state."""
        with self.lock:
            now = self.clock()
            wall = self.wall_clock()
            state = fields["state"]

            def act():
                if (
                    self.run_file is not None
                    and self.last_packet is not None
                    and now - self.last_packet > RUN_GAP_S
                ):
                    self._end_run(
                        wall, now, None,
                        f"no run packets for {RUN_GAP_S:.0f} s"
                    )

                if self.run_file is None and state in DRIVING_STATES:
                    if self.last_state is None:
                        trigger = "first packet seen while driving"
                    else:
                        trigger = (
                            f"state {state_name(self.last_state)}"
                            f" -> {state_name(state)}"
                        )

                    self._start_run(wall, now, trigger)

                record = {
                    "kind": "sample",
                    "wall": wall.isoformat(timespec="milliseconds")
                }

                if self.run_file is not None:
                    record["t_ms"] = int((now - self.run_start) * 1000)

                record.update(fields)
                record["lost"] = lost

                if self.run_file is not None:
                    self._write(self.run_file, record)

                    if state in END_STATES:
                        self._end_run(
                            wall, now, state,
                            f"state {state_name(state)}"
                        )
                else:
                    self._write_idle(wall, record)

            self._safely(act)
            self.last_packet = now
            self.last_state = state

    def event(self, text, kind="event"):
        """Record one event line (robot log or dashboard note)."""
        with self.lock:
            now = self.clock()
            wall = self.wall_clock()
            record = {
                "kind": kind,
                "wall": wall.isoformat(timespec="milliseconds"),
                "text": text
            }

            def act():
                if self.run_file is not None:
                    record["t_ms"] = int((now - self.run_start) * 1000)
                    self._write(self.run_file, record)
                    return

                self._write_idle(wall, record)
                self.recent.append((now, dict(record)))

                if (
                    self.ended_path is not None
                    and now - self.ended_at <= EVENT_MARGIN_S
                ):
                    copy = dict(record)
                    copy["t_ms"] = int((now - self.ended_start) * 1000)
                    copy["after_end"] = True

                    with self._open(self.ended_path) as handle:
                        self._write(handle, copy)

            self._safely(act)

    def check_timeout(self):
        """Close a run whose packets stopped arriving."""
        with self.lock:
            now = self.clock()

            if (
                self.run_file is not None
                and self.last_packet is not None
                and now - self.last_packet > RUN_GAP_S
            ):
                wall = self.wall_clock()
                self._safely(lambda: self._end_run(
                    wall, now, None,
                    f"no run packets for {RUN_GAP_S:.0f} s"
                ))
                self.last_state = None

    def status(self):
        with self.lock:
            return {
                "run_path": self.run_path,
                "idle_date": self.idle_date,
                "error": self.error
            }


class RobotFeed:
    """Receive real MQTT messages; send start/stop commands on request."""

    def __init__(self, host, port, runs_dir=None):
        self.lock = threading.Lock()
        self.connected = False
        self.robot_state = "UNKNOWN"
        self.mapping = {}
        self.mapping_time = None
        self.telemetry = None
        self.telemetry_time = None
        self.heartbeat_time = None
        self.heartbeat_monotonic = None
        self.heartbeat_count = 0
        self.events = deque(maxlen=40)
        self.run = None
        self.run_time = None
        self.run_monotonic = None
        self.run_received = 0
        self.run_lost = 0
        self.run_last_seq = None
        self.recorder = RunRecorder(runs_dir or RUNS_DIR)

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2
        )

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        # no host: record and display only (used by the tests)
        if host is None:
            return

        try:
            self.client.connect_async(
                host,
                port,
                keepalive=30
            )
            self.client.loop_start()
        except Exception as error:
            self.add_event(f"MQTT startup error: {error}")

    def add_event(self, message, record=True):
        timestamp = datetime.now().strftime("%H:%M:%S")

        with self.lock:
            self.events.appendleft(
                f"{timestamp}  {message}"
            )

        if record:
            self.recorder.event(message, kind="note")

    def on_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties
    ):
        connected = not reason_code.is_failure

        with self.lock:
            self.connected = connected
            self.robot_state = "UNKNOWN"
            self.run_last_seq = None

        if connected:
            for topic in (
                TOPIC_MAPPING,
                TOPIC_CONNECTION,
                TOPIC_TELEMETRY,
                TOPIC_HEARTBEAT,
                TOPIC_TEST
            ):
                client.subscribe(topic, qos=1)

            # sent at QoS 0 by the car every 200 ms
            client.subscribe(TOPIC_RUN, qos=0)

            self.add_event(
                "Connected to MQTT broker"
            )
        else:
            self.add_event(
                f"Broker rejected connection: {reason_code}"
            )

    def on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties
    ):
        with self.lock:
            self.connected = False
            self.robot_state = "UNKNOWN"

        self.add_event(
            "Dashboard disconnected from MQTT broker"
        )

    def on_message(self, client, userdata, message):
        topic = message.topic
        now = datetime.now().strftime("%H:%M:%S")

        if topic == TOPIC_RUN:
            if len(message.payload) != RUN_BYTES:
                self.add_event(
                    "Unexpected run telemetry payload length "
                    f"({len(message.payload)} bytes)"
                )
                return

            fields = decode_run_packet(message.payload)

            with self.lock:
                lost = 0

                # after a reboot of the car the seq starts again
                if (
                    self.run is not None
                    and fields["uptime"] < self.run["uptime"]
                ):
                    self.run_last_seq = None

                if self.run_last_seq is not None:
                    gap = (fields["seq"] - self.run_last_seq - 1) % 256

                    # 255 means the same seq again, not 255 lost
                    if gap != 255:
                        lost = gap

                self.run_last_seq = fields["seq"]
                self.run_received += 1
                self.run_lost += lost
                self.run = fields
                self.run_time = now
                self.run_monotonic = time.monotonic()

            self.recorder.sample(fields, lost)
            return

        if topic == TOPIC_CONNECTION:
            state = message.payload.decode(
                "ascii",
                errors="replace"
            ).strip().lower()

            if state not in ("online", "offline"):
                self.add_event(
                    "Invalid robot connection status ignored"
                )
                return

            with self.lock:
                changed = self.robot_state != state.upper()
                self.robot_state = state.upper()

            if changed:
                self.add_event(
                    f"Pico W is {state.upper()}"
                )

            return

        if topic == TOPIC_MAPPING:
            payload = message.payload.decode(
                "utf-8",
                errors="replace"
            )

            entries = MAP_LINE.findall(payload)
            mapping = dict(entries)

            valid = (
                len(entries) == 4
                and len(mapping) == 4
                and len(set(mapping.values())) == 4
            )

            if valid:
                with self.lock:
                    self.mapping = mapping
                    self.mapping_time = now

                suffix = (
                    " (retained)"
                    if message.retain
                    else ""
                )

                self.add_event(
                    "Barcode mapping received" + suffix
                    + ": " + ", ".join(
                        f"{key}->{mapping[key]}" for key in sorted(mapping)
                    )
                )
            else:
                self.add_event(
                    "Invalid barcode mapping ignored"
                )

            return

        if topic == TOPIC_HEARTBEAT:
            if message.payload != b"alive":
                self.add_event("Invalid heartbeat ignored")
                return

            with self.lock:
                self.heartbeat_time = now
                self.heartbeat_monotonic = time.monotonic()
                self.heartbeat_count += 1

            return

        if topic == TOPIC_TELEMETRY:
            if len(message.payload) != 3:
                self.add_event(
                    "Unexpected telemetry payload length"
                )
                return

            packed = int.from_bytes(
                message.payload,
                byteorder="little"
            )

            metric = (packed >> 13) & 0x7F

            if (packed & 0xF00000) or metric > 100:
                self.add_event(
                    "Invalid telemetry ignored"
                )
                return

            fields = {
                "controller_state": packed & 7,
                "wifi": bool((packed >> 3) & 1),
                "mqtt": bool((packed >> 4) & 1),
                "faults": (packed >> 5) & 0xFF,
                "metric": metric
            }

            with self.lock:
                self.telemetry = fields
                self.telemetry_time = now

            return

        if topic == TOPIC_TEST:
            payload = message.payload.decode(
                "utf-8",
                errors="replace"
            ).rstrip("\r\n")

            self.recorder.event(payload)
            self.add_event(
                "Robot log: "
                + payload[:120].replace("\n", " "),
                record=False
            )

    def send_command(self, code, name):
        """Publish one FR2 command, as robot.ps1 does."""
        seq = next_sequence()
        info = self.client.publish(
            TOPIC_COMMAND,
            bytes([FR2_VERSION, code, seq]),
            qos=1
        )

        if info.rc == mqtt.MQTT_ERR_SUCCESS:
            self.add_event(f"Sent {name.upper()} (seq {seq})")
        else:
            self.add_event(
                f"Could not send {name.upper()}: "
                f"{mqtt.error_string(info.rc)}"
            )

    def send_after_reset(self, code, name):
        """RESET, wait 2 s, then the command (robot.ps1 go / search)."""
        def run():
            self.send_command(CMD_RESET, "reset")
            time.sleep(2.0)
            self.send_command(code, name)

        threading.Thread(target=run, daemon=True).start()

    def snapshot(self):
        with self.lock:
            return {
                "connected": self.connected,
                "robot_state": self.robot_state,
                "mapping": dict(self.mapping),
                "mapping_time": self.mapping_time,
                "telemetry": (
                    dict(self.telemetry)
                    if self.telemetry is not None
                    else None
                ),
                "telemetry_time": self.telemetry_time,
                "heartbeat_time": self.heartbeat_time,
                "heartbeat_monotonic": self.heartbeat_monotonic,
                "heartbeat_count": self.heartbeat_count,
                "events": list(self.events),
                "run": dict(self.run) if self.run is not None else None,
                "run_time": self.run_time,
                "run_monotonic": self.run_monotonic,
                "run_received": self.run_received,
                "run_lost": self.run_lost
            }


class RunFileReader:
    """Read .jsonl run files, only reading what was added since last time."""

    def __init__(self):
        self.lock = threading.Lock()
        self.files = {}

    def read(self, path):
        path = Path(path)

        with self.lock:
            entry = self.files.get(path)

            try:
                size = path.stat().st_size
            except OSError:
                return []

            if entry is None or size < entry["offset"]:
                entry = {"offset": 0, "records": [], "bad": 0}
                self.files[path] = entry

            if size > entry["offset"]:
                with open(path, "rb") as handle:
                    handle.seek(entry["offset"])
                    chunk = handle.read(size - entry["offset"])

                # a line still being written has no newline yet
                end = chunk.rfind(b"\n")

                if end >= 0:
                    for line in chunk[:end].split(b"\n"):
                        line = line.strip()

                        if not line:
                            continue

                        try:
                            entry["records"].append(json.loads(line))
                        except ValueError:
                            entry["bad"] += 1

                    entry["offset"] += end + 1

            return entry["records"]


def list_run_files(folder):
    """Run files newest first, then the idle logs newest first."""
    folder = Path(folder)

    if not folder.is_dir():
        return [], []

    names = sorted(
        (item.name for item in folder.glob("*.jsonl")),
        reverse=True
    )
    runs = [name for name in names if not name.startswith("idle_")]
    idle = [name for name in names if name.startswith("idle_")]

    return runs, idle


def run_label(name):
    stem = name[:-len(".jsonl")]

    if stem.startswith("idle_"):
        return f"Idle log {stem[len('idle_'):]}"

    match = re.match(
        r"^(\d{4}-\d{2}-\d{2})_(\d{2})(\d{2})(\d{2})(_\d+)?$",
        stem
    )

    if match is None:
        return stem

    label = (
        f"Run {match.group(1)} "
        f"{match.group(2)}:{match.group(3)}:{match.group(4)}"
    )

    if match.group(5):
        label += f" ({match.group(5)[1:]})"

    return label


def record_seconds(records):
    """Seconds since run start for each record (wall time for idle logs)."""
    first_wall = None
    times = []

    for record in records:
        if "t_ms" in record:
            times.append(record["t_ms"] / 1000.0)
            continue

        try:
            wall = datetime.fromisoformat(record["wall"])
        except (KeyError, TypeError, ValueError):
            times.append(None)
            continue

        if first_wall is None:
            first_wall = wall

        times.append((wall - first_wall).total_seconds())

    return times


def build_run_view(records, max_samples=None):
    """Samples, state segments, events and summary for one file."""
    times = record_seconds(records)
    samples = []
    events = []
    start = None
    end = None

    for record, seconds in zip(records, times):
        kind = record.get("kind")

        if kind == "sample" and seconds is not None:
            row = dict(record)
            row["t_s"] = seconds
            samples.append(row)
        elif kind in ("event", "note"):
            text = str(record.get("text", ""))
            tag = event_tag(text)
            events.append({
                "t (s)": round(seconds, 2) if seconds is not None else None,
                "tag": tag,
                "alert": tag in ALERT_TAGS,
                "source": "robot" if kind == "event" else "dashboard",
                "text": text,
                "wall": str(record.get("wall", ""))[11:23]
            })
        elif kind == "run_start":
            start = record
        elif kind == "run_end":
            end = dict(record)
            end["t_s"] = seconds

    if max_samples is not None and len(samples) > max_samples:
        samples = samples[-max_samples:]

    frame = pd.DataFrame(samples)

    if not frame.empty:
        # distance since the first packet; the 20-bit counter wraps
        steps = frame["distance"].diff().fillna(0) % DISTANCE_WRAP
        frame["run_mm"] = steps.cumsum()
        frame["range_mm"] = frame["range"].where(frame["range"] > 0)

    segments = []

    for row in samples:
        if segments and segments[-1]["state"] == row["state_name"]:
            segments[-1]["end"] = row["t_s"]
        else:
            if segments:
                segments[-1]["end"] = row["t_s"]

            segments.append({
                "state": row["state_name"],
                "start": row["t_s"],
                "end": row["t_s"]
            })

    if segments:
        # the last state lasts until the next packet would have come
        segments[-1]["end"] += 0.2

    return {
        "samples": frame,
        "segments": pd.DataFrame(segments),
        "events": pd.DataFrame(
            events,
            columns=["t (s)", "tag", "alert", "source", "text", "wall"]
        ),
        "start": start,
        "end": end,
        "summary": summarise(samples, events, end)
    }


def summarise(samples, events, end):
    summary = {
        "samples": len(samples),
        "duration_s": 0.0,
        "distance_mm": 0,
        "max_speed": 0,
        "junctions": 0,
        "letters": [],
        "humps": 0,
        "hump_peak": 0,
        "faults_seen": 0,
        "faults_new": 0,
        "final_state": "",
        "lost": 0,
        "alerts": sum(1 for event in events if event["alert"])
    }

    if not samples:
        return summary

    first = samples[0]
    last = samples[-1]
    summary["duration_s"] = last["t_s"] - first["t_s"]

    if end is not None and end.get("t_s") is not None:
        summary["duration_s"] = max(
            summary["duration_s"],
            end["t_s"] - min(0.0, first["t_s"])
        )

    previous = first
    faults = 0

    for row in samples:
        summary["max_speed"] = max(
            summary["max_speed"],
            abs(row["speed_l"]),
            abs(row["speed_r"])
        )
        faults |= row["faults"]

        if row is not first:
            summary["lost"] += row.get("lost", 0)
            summary["distance_mm"] += (
                (row["distance"] - previous["distance"]) % DISTANCE_WRAP
            )
            summary["junctions"] += (
                (row["junctions"] - previous["junctions"]) % JUNCTION_WRAP
            )

        if row["letter"] and (
            not summary["letters"]
            or summary["letters"][-1] != row["letter"]
        ):
            summary["letters"].append(row["letter"])

        summary["hump_peak"] = max(summary["hump_peak"], row["hump"])
        previous = row

    reports = sum(
        1 for event in events
        if event["source"] == "robot" and TERRAIN_REPORT.search(event["text"])
    )
    # the hump field is only the highest peak so far, so humps are
    # counted from the TERRAIN_REPORT lines
    summary["humps"] = reports

    if not reports and summary["hump_peak"]:
        summary["humps"] = 1
    summary["faults_seen"] = faults
    summary["faults_new"] = faults & ~first["faults"] & 0xFF

    if end is not None:
        summary["final_state"] = end.get("state_name") or (
            f"{last['state_name']} ({end.get('reason', '')})"
        )
    else:
        summary["final_state"] = last["state_name"]

    return summary


@st.cache_resource
def create_feed(host, port):
    return RobotFeed(host, port)


@st.cache_resource
def create_reader():
    return RunFileReader()


def on_off(value):
    return "black" if value else "white"


@st.fragment(run_every="1s")
def live_dashboard(feed):
    data = feed.snapshot()
    feed.recorder.check_timeout()

    broker_up = data["connected"]
    robot = (
        data["robot_state"]
        if broker_up
        else "UNKNOWN"
    )

    mapping = data["mapping"]
    telemetry = data["telemetry"]
    run = data["run"]
    last_heartbeat = data["heartbeat_monotonic"]
    heartbeat_live = (
        broker_up
        and last_heartbeat is not None
        and (time.monotonic() - last_heartbeat) < 15.0
    )

    col1, col2, col3, col4 = st.columns(4)

    col1.metric(
        "MQTT broker",
        "Connected" if broker_up else "Disconnected"
    )

    col2.metric(
        "Pico W",
        robot.title()
    )

    col3.metric(
        "Mapped barcodes",
        f"{len(mapping)}/4"
    )

    col4.metric(
        "Robot telemetry",
        "Received" if telemetry or run else "Unavailable"
    )

    if not broker_up:
        st.error(
            "MQTT broker disconnected. "
            "Robot connection state is unknown."
        )
    elif robot == "OFFLINE":
        st.error(
            "Pico W is offline according to "
            "the last MQTT status message."
        )
    elif robot == "UNKNOWN":
        st.warning(
            "Waiting for Pico W connection status."
        )
    else:
        st.success(
            "Pico W is online according to "
            "the last MQTT status message."
        )

    st.subheader("Run telemetry (live)")

    recording = feed.recorder.status()

    if recording["error"]:
        st.error(f"Run recording failed: {recording['error']}")

    if run:
        age = time.monotonic() - data["run_monotonic"]
        received = data["run_received"]
        lost = data["run_lost"]
        loss = 100.0 * lost / (received + lost) if received + lost else 0.0

        first, second, third, fourth, fifth = st.columns(5)
        first.metric("State", run["state_name"])
        second.metric("Left speed (mm/s)", run["speed_l"])
        third.metric("Right speed (mm/s)", run["speed_r"])
        fourth.metric(
            "Distance since boot (m)",
            f"{run['distance'] / 1000:.2f}"
        )
        fifth.metric(
            "Range (mm)",
            run["range"] if run["range"] else "nothing"
        )

        first, second, third, fourth, fifth = st.columns(5)
        first.metric(
            "Line IRs L / R",
            f"{on_off(run['ir_l'])} / {on_off(run['ir_r'])}"
        )
        second.metric("Barcode IR", on_off(run["ir_b"]))
        third.metric("Last letter", run["letter"] or "none")
        fourth.metric("Junctions", run["junctions"])
        fifth.metric("Side lift (mm)", run["lift"])

        first, second, third, fourth, fifth = st.columns(5)
        first.metric("Run hump peak (mm)", run["hump"] or "none")
        second.metric("Packets received", received)
        third.metric("Packets lost", lost)
        fourth.metric("Packet loss", f"{loss:.1f} %")
        uptime = run["uptime"]
        fifth.metric(
            "Car uptime",
            f"{uptime // 3600}:{uptime // 60 % 60:02d}:{uptime % 60:02d}"
        )

        if run["faults"]:
            st.error(
                f"Faults latched (0x{run['faults']:02X}): "
                + ", ".join(run["fault_names"])
            )
        else:
            st.caption("No faults latched.")

        if age > 2.0:
            st.warning(
                f"No run packet for {age:.0f} s; "
                "the values above are the last ones received."
            )

        st.caption(
            f"Last run packet: {data['run_time']} | seq {run['seq']} | "
            f"Wi-Fi {'up' if run['wifi'] else 'down'} | "
            f"MQTT {'up' if run['mqtt'] else 'down'}"
        )
    else:
        st.info(
            f"Waiting for run telemetry on {TOPIC_RUN} "
            "(sent every 200 ms)."
        )

    if recording["run_path"] is not None:
        st.success(
            f"Recording run to {recording['run_path'].name} "
            f"in {feed.recorder.folder}"
        )
    elif recording["idle_date"] is not None:
        st.caption(
            f"No run in progress; packets and events go to "
            f"idle_{recording['idle_date']}.jsonl"
        )

    st.subheader("Latest barcode mapping")

    if mapping:
        left, right = st.columns(2)

        for index, letter in enumerate(sorted(mapping)):
            column = (
                left if index % 2 == 0 else right
            )

            column.metric(
                f"Barcode {letter}",
                mapping[letter]
            )

        st.caption(
            f"Last mapping received: {data['mapping_time']}. "
            "Stored mappings can become stale "
            "when the robot disconnects."
        )
    else:
        st.info(
            "Waiting for an accepted barcode mapping "
            "published by the Pico W."
        )

    st.subheader("MQTT heartbeat")

    heartbeat_left, heartbeat_right = st.columns(2)
    heartbeat_left.metric(
        "Pico W heartbeat",
        "Receiving" if heartbeat_live else "Not receiving"
    )
    heartbeat_right.metric("Heartbeats received", data["heartbeat_count"])

    if data["heartbeat_time"] is not None:
        st.caption(
            f"Last heartbeat received: {data['heartbeat_time']} | "
            "Expected interval: 5 seconds | "
            "No heartbeat for 15 seconds = not receiving"
        )
    else:
        st.info("Waiting for the first heartbeat from Pico W.")

    st.subheader("Robot status (5 s status message)")

    if telemetry:
        first, second, third = st.columns(3)

        first.metric(
            "Controller state (raw code)",
            telemetry["controller_state"]
        )

        second.metric(
            "Fault flags",
            f"0x{telemetry['faults']:02X}"
        )

        third.metric(
            "Metric 0-100",
            telemetry["metric"]
        )

        st.caption(
            f"Last telemetry: {data['telemetry_time']} | "
            f"Wi-Fi flag: {telemetry['wifi']} | "
            f"MQTT flag: {telemetry['mqtt']}"
        )

        st.warning(
            "Telemetry is the last received value; "
            "it may not represent the current state."
        )
    else:
        st.info(
            "No controller status received yet. "
            "The run telemetry above is sent more often."
        )

    st.subheader("Recent MQTT events")

    if data["events"]:
        st.code(
            "\n".join(data["events"]),
            language="text"
        )
    else:
        st.caption(
            "Waiting for MQTT messages..."
        )


def time_axis():
    return alt.X("t_s:Q", title="time since run start (s)")


def line_chart(frame, column, title, unit):
    return alt.Chart(frame).mark_line().encode(
        x=time_axis(),
        y=alt.Y(f"{column}:Q", title=unit),
        tooltip=[
            alt.Tooltip("t_s:Q", title="t (s)", format=".1f"),
            alt.Tooltip(f"{column}:Q", title=unit)
        ]
    ).properties(title=title, height=220)


def alert_rules(events):
    """Red vertical lines where alert events happened."""
    if events.empty:
        return None

    alerts = events[events["alert"] & events["t (s)"].notna()]

    if alerts.empty:
        return None

    alerts = alerts.rename(columns={"t (s)": "t_s"})

    return alt.Chart(alerts).mark_rule(
        color="#c62828",
        strokeDash=[4, 3]
    ).encode(
        x="t_s:Q",
        tooltip=["t_s:Q", "tag:N", "text:N"]
    )


def show_charts(view):
    frame = view["samples"]
    rules = alert_rules(view["events"])

    def with_rules(chart):
        return chart + rules if rules is not None else chart

    speeds = frame.melt(
        id_vars=["t_s"],
        value_vars=["speed_l", "speed_r"],
        var_name="wheel",
        value_name="speed"
    )
    speeds["wheel"] = speeds["wheel"].map(
        {"speed_l": "left", "speed_r": "right"}
    )

    speed_chart = alt.Chart(speeds).mark_line().encode(
        x=time_axis(),
        y=alt.Y("speed:Q", title="mm/s"),
        color=alt.Color("wheel:N", title="wheel"),
        tooltip=[
            alt.Tooltip("t_s:Q", title="t (s)", format=".1f"),
            "wheel:N",
            "speed:Q"
        ]
    ).properties(
        title="Wheel speeds (dashed red = alert event)",
        height=240
    )

    st.altair_chart(with_rules(speed_chart))

    segments = view["segments"]

    if not segments.empty:
        domain = [name for name in STATE_COLOURS]
        domain += [
            name for name in segments["state"].unique()
            if name not in STATE_COLOURS
        ]
        colours = [STATE_COLOURS.get(name, "#000000") for name in domain]

        timeline = alt.Chart(segments).mark_rect().encode(
            x=alt.X("start:Q", title="time since run start (s)"),
            x2="end:Q",
            color=alt.Color(
                "state:N",
                title="state",
                scale=alt.Scale(domain=domain, range=colours)
            ),
            tooltip=[
                "state:N",
                alt.Tooltip("start:Q", title="from (s)", format=".1f"),
                alt.Tooltip("end:Q", title="to (s)", format=".1f")
            ]
        ).properties(title="State timeline", height=70)

        st.altair_chart(timeline)

    left, right = st.columns(2)

    with left:
        st.altair_chart(line_chart(
            frame, "run_mm", "Distance since run start", "mm"
        ))

        lift = line_chart(frame, "lift", "Side lift (IMU roll)", "mm")
        hump = alt.Chart(frame).mark_line(
            strokeDash=[2, 2],
            color="#ef6c00"
        ).encode(x=time_axis(), y="hump:Q")
        st.altair_chart(lift + hump)
        st.caption("Dashed orange: highest hump peak so far in the run.")

    with right:
        ranges = frame[frame["range_mm"].notna()]
        range_chart = alt.Chart(ranges).mark_circle(size=14).encode(
            x=time_axis(),
            y=alt.Y("range_mm:Q", title="mm"),
            tooltip=[
                alt.Tooltip("t_s:Q", title="t (s)", format=".1f"),
                alt.Tooltip("range_mm:Q", title="range (mm)")
            ]
        ).properties(title="Ultrasonic range", height=220)
        st.altair_chart(range_chart)
        st.caption("Range 0 (nothing within 2 m / no echo) is not drawn.")

        irs = frame[["t_s"]].copy()
        irs["left IR"] = frame["ir_l"].astype(int) * 1.0 + 2.0
        irs["right IR"] = frame["ir_r"].astype(int) * 1.0
        irs = irs.melt(
            id_vars=["t_s"],
            var_name="sensor",
            value_name="level"
        )
        ir_chart = alt.Chart(irs).mark_line(
            interpolate="step-after"
        ).encode(
            x=time_axis(),
            y=alt.Y("level:Q", title="", axis=None),
            color=alt.Color("sensor:N", title="")
        ).properties(title="Line IRs (high = black)", height=220)
        st.altair_chart(ir_chart)


def highlight_alerts(row):
    colour = "background-color: rgba(198, 40, 40, 0.25)"
    return [colour if row["alert"] else ""] * len(row)


def show_events(view, key):
    events = view["events"]

    st.markdown("**Events**")

    if events.empty:
        st.caption("No event lines in this file.")
        return

    left, right = st.columns([1, 2])
    only_alerts = left.checkbox(
        "Only E-stop / fault / line lost / tilted / lifted / "
        "obstacle / link lost",
        key=f"{key}_alerts"
    )
    search = right.text_input("Filter text", key=f"{key}_search")

    shown = events

    if only_alerts:
        shown = shown[shown["alert"]]

    if search:
        shown = shown[shown["text"].str.contains(
            search, case=False, regex=False
        )]

    tags = shown["tag"].replace("", "-").value_counts()

    if not tags.empty:
        st.caption(
            "Tags: " + ", ".join(f"{tag} {count}" for tag, count in
                                 tags.items())
        )

    st.dataframe(
        shown.style.apply(highlight_alerts, axis=1),
        hide_index=True,
        column_order=["t (s)", "tag", "source", "text", "wall"]
    )


def show_summary(view, live):
    summary = view["summary"]
    faults = summary["faults_seen"]
    new = summary["faults_new"]

    first, second, third, fourth = st.columns(4)
    first.metric("Duration (s)", f"{summary['duration_s']:.1f}")
    second.metric("Distance (m)", f"{summary['distance_mm'] / 1000:.2f}")
    third.metric("Max wheel speed (mm/s)", summary["max_speed"])
    fourth.metric("Junctions", summary["junctions"])

    first, second, third, fourth = st.columns(4)
    first.metric(
        "Barcode letters",
        " ".join(summary["letters"]) or "none"
    )
    second.metric(
        "Humps",
        f"{summary['humps']} (peak {summary['hump_peak']} mm)"
        if summary["humps"] else "0"
    )
    third.metric(
        "Final state",
        (summary["final_state"] or "-") + (" (running)" if live else "")
    )
    fourth.metric("Packets lost", summary["lost"])

    if faults:
        text = "Faults raised: " + ", ".join(fault_names(faults))

        if faults != new:
            text += (
                " (already latched at the start: "
                + ", ".join(fault_names(faults & ~new)) + ")"
            )

        st.error(text)
    else:
        st.caption("No faults raised in this run.")

    if summary["alerts"]:
        st.warning(f"{summary['alerts']} alert event(s); see the table.")


def show_run(path, live):
    records = create_reader().read(path)
    idle = path.name.startswith("idle_")
    view = build_run_view(records, max_samples=6000 if idle else None)

    start = view["start"]
    end = view["end"]
    heading = run_label(path.name)

    if live:
        heading += " (recording)"

    st.markdown(f"#### {heading}")

    details = [f"File: {path}"]

    if start is not None:
        details.append(f"started by {start.get('trigger', '?')}")

    if end is not None:
        details.append(f"ended: {end.get('reason', '?')}")
    elif not live and not idle:
        details.append("no end record (dashboard closed during the run?)")

    st.caption(" | ".join(details))

    if idle:
        st.caption(
            "Idle log: packets and events outside runs. Time is "
            "seconds since the first line of the day"
            + (" (last 6000 packets)." if len(view["samples"]) >= 6000
               else ".")
        )
    else:
        show_summary(view, live)

    if view["samples"].empty:
        st.info("No telemetry packets in this file yet.")
    else:
        show_charts(view)

    show_events(view, "live" if live else "past")

    try:
        st.download_button(
            "Download this file (.jsonl)",
            data=path.read_bytes(),
            file_name=path.name,
            mime="application/json",
            key=f"download_{'live' if live else 'past'}"
        )
    except OSError:
        pass


LATEST = "__latest__"


@st.fragment(run_every="2s")
def latest_run_view(feed, folder):
    """Follow the run being recorded, or the newest one on disk."""
    live_path = None

    if feed is not None:
        live_path = feed.recorder.status()["run_path"]

    if live_path is not None:
        show_run(live_path, live=True)
        return

    runs, _ = list_run_files(folder)

    if runs:
        show_run(Path(folder) / runs[0], live=False)
    else:
        st.info(
            f"No runs recorded yet in {folder}. A run starts when the "
            "car goes from IDLE/STOPPED to LINE_FOLLOW."
        )


def runs_section(feed):
    folder = feed.recorder.folder if feed is not None else RUNS_DIR

    st.header("Runs")
    st.caption(
        f"Every run is saved as it happens to {folder} "
        "(one .jsonl file per run, named by start time). "
        "Pick one to see its charts and events."
    )

    runs, idle = list_run_files(folder)
    live_path = (
        feed.recorder.status()["run_path"] if feed is not None else None
    )
    options = [LATEST] + runs + idle

    def label(name):
        if name == LATEST:
            return "Latest run (follows the live run)"

        text = run_label(name)

        if live_path is not None and name == live_path.name:
            text += " (recording)"

        return text

    pick, refresh = st.columns([4, 1])
    choice = pick.selectbox(
        "Run",
        options,
        format_func=label,
        key="run_choice"
    )
    refresh.button("Refresh list")

    if choice == LATEST:
        latest_run_view(feed, folder)
    else:
        path = Path(folder) / choice
        show_run(
            path,
            live=live_path is not None and path.name == live_path.name
        )


def main():
    st.set_page_config(
        page_title="INF2004 Car Telemetry",
        layout="wide"
    )

    st.title("INF2004 | Robotic Car Telemetry")

    st.caption(
        "Buddy 1 | MQTT dashboard | "
        "Live robot messages, start/stop buttons for testing, "
        "run recording"
    )

    feed = None

    try:
        broker, port = load_broker_config()
    except (OSError, ValueError) as error:
        st.error(f"MQTT configuration error: {error}")
        st.caption("Recorded runs can still be viewed below.")
    else:
        feed = create_feed(broker, port)
        st.write(f"Broker from project config: `{broker}:{port}`")

    if feed is not None:
        # drive the car from here for testing (same commands as
        # robot.ps1 go / search / stop).
        st.subheader("Drive the car (testing)")

        go_col, search_col, stop_col = st.columns(3)

        if go_col.button("Start (reset + go)"):
            feed.send_after_reset(CMD_START, "start")
            go_col.caption("Reset sent; start follows in 2 s.")

        if search_col.button("Line search (reset + search)"):
            feed.send_after_reset(CMD_SEARCH, "search")
            search_col.caption("Reset sent; search follows in 2 s.")

        if stop_col.button("STOP", type="primary"):
            feed.send_command(CMD_STOP, "stop")
            stop_col.caption("Stop sent.")

        st.caption(
            "Start = robot.ps1 go (also clears a latched E-stop). "
            "Line search: start the car off the line. "
            "Sent commands show under Recent MQTT events."
        )

        live_dashboard(feed)

    runs_section(feed)


if __name__ == "__main__":
    main()
