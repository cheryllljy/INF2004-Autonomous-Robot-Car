

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





# Original historical plotting helpers were removed: the dashboard now
# displays meaningful charts only in the live monitoring panel.


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
        st.caption("Telemetry remains available in the downloadable JSONL file; "
                   "charts appear only in Live Robot Monitoring.")

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





@st.fragment(run_every="6s")

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



    st.header("Run recordings & event history")

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

        selected_run_view(path, feed)


@st.fragment(run_every="6s")
def selected_run_view(path, feed):
    """Refresh the chosen recorded run without rerunning the entire page."""
    current = (feed.recorder.status()["run_path"]
               if feed is not None else None)
    show_run(path, live=current is not None and path.name == current.name)





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

        st.header("Live robot monitoring")
        st.caption("All five live monitoring panels are visible below. "
                   "Only this section refreshes every four seconds; "
                   "sensor values and events are from actual MQTT messages.")
        advanced_live_dashboard(feed)
        live_classic_charts(feed)



    runs_section(feed)





# ================================================================
# ADDITIVE ADVANCED MONITORING
# The existing dashboard, controls, recorder, and original charts
# remain unchanged. These helpers use only robot-reported values.
# ================================================================

import math

TOPIC_SENSORS_ADVANCED = "inf2004/robot/sensors"
TOPIC_CONTROLLER_ADVANCED = "inf2004/robot/controller"
TERRAIN_ADVANCED_RE = re.compile(
    r"TERRAIN_REPORT\s+seq=(\d+)\s+classification=([A-Z_]+)\s+"
    r"peak_side_height_est=(-?\d+)\s*mm(?:\s+duration=(\d+)\s*ms)?",
    re.IGNORECASE,
)
OLD_TERRAIN_ADVANCED_RE = re.compile(
    r"HUMP_DONE.*?peak\s+side-height=(-?\d+)\s*mm", re.IGNORECASE
)
ADVANCED_EVENT_RE = re.compile(
    r"hump|terrain_report|barcode|junction|obstacle|turn|"
    r"line lost|link lost|fault|e-?stop|tilted|picked up|"
    r"reroute|re-route|shoc?k", re.IGNORECASE,
)
ADVANCED_SAMPLES_MAX = 1200   # memory-bound live history (~4 min at 5 Hz)


def advanced_decode_sensor(payload):
    """Decode *actual* Buddy 1 sensor protocol v1, 42 bytes."""
    if len(payload) != 42 or payload[0] != 1:
        return None

    def signed(offset, count):
        return int.from_bytes(payload[offset:offset + count],
                              "little", signed=True)

    valid = payload[1]
    flags = payload[4]
    return {
        "line_valid": bool(valid & 1),
        "motion_valid": bool(valid & 2),
        "imu_valid": bool(valid & 4),
        "hump_valid": bool(valid & 8),
        "line_state": payload[2],
        "line_error": signed(3, 1),
        "line_left_black": bool(flags & 1),
        "line_right_black": bool(flags & 2),
        "line_control": bool(flags & 4),
        "scanning": bool(flags & 8),
        "stop_required": bool(flags & 16),
        "on_hump": bool(flags & 32),
        "motion_state": payload[5],
        "motion_fault": payload[6],
        "hump_counter_mod256": payload[7],
        "requested_left_mm_s": signed(8, 2),
        "requested_right_mm_s": signed(10, 2),
        "measured_left_mm_s": signed(12, 2),
        "measured_right_mm_s": signed(14, 2),
        "total_distance_mm": signed(16, 4),
        "trip_distance_mm": signed(20, 4),
        "ticks_left": signed(24, 4),
        "ticks_right": signed(28, 4),
        "accel_x": signed(32, 2),
        "accel_y": signed(34, 2),
        "accel_z": signed(36, 2),
        "heading_deg": signed(38, 4),
    }


def advanced_record_sensor(recorder, fields):
    """Append sensor samples without changing original run records."""
    with recorder.lock:
        now = recorder.clock()
        wall = recorder.wall_clock()
        record = {"kind": "sensor_sample",
                  "wall": wall.isoformat(timespec="milliseconds")}
        record.update(fields)

        def action():
            if recorder.run_file is not None and recorder.run_start is not None:
                record["t_ms"] = int((now - recorder.run_start) * 1000)
                recorder._write(recorder.run_file, record)
            else:
                recorder._write_idle(wall, record)

        recorder._safely(action)


def advanced_record_terrain(recorder, fields):
    with recorder.lock:
        now = recorder.clock()
        wall = recorder.wall_clock()
        record = {"kind": "terrain_sample",
                  "wall": wall.isoformat(timespec="milliseconds")}
        record.update(fields)

        def action():
            if recorder.run_file is not None and recorder.run_start is not None:
                record["t_ms"] = int((now - recorder.run_start) * 1000)
                recorder._write(recorder.run_file, record)
            else:
                recorder._write_idle(wall, record)

        recorder._safely(action)


def advanced_initialize_feed(self, *args, **kwargs):
    # Set up before the original constructor starts its MQTT thread.
    self.advanced_start = time.monotonic()
    self.advanced_sensor = None
    self.advanced_sensor_monotonic = None
    self.advanced_controller = None
    self.advanced_controller_monotonic = None
    self.advanced_samples = deque(maxlen=ADVANCED_SAMPLES_MAX)
    self.advanced_terrain = deque(maxlen=200)
    self.advanced_events = deque(maxlen=150)
    self.advanced_seen_terrain = set()
    self.advanced_sensor_received = 0
    self.advanced_sensor_invalid = 0
    self.advanced_barcode_trace = deque(maxlen=150)
    self.advanced_barcode_last = None
    self.advanced_run_history = deque(maxlen=ADVANCED_SAMPLES_MAX)
    return _advanced_original_init(self, *args, **kwargs)


def advanced_on_connect(self, client, userdata, flags, reason_code, properties):
    result = _advanced_original_connect(
        self, client, userdata, flags, reason_code, properties)
    if not reason_code.is_failure:
        client.subscribe(TOPIC_SENSORS_ADVANCED, qos=0)
        client.subscribe(TOPIC_CONTROLLER_ADVANCED, qos=0)
    return result


def advanced_on_message(self, client, userdata, message):
    topic = message.topic
    if topic == TOPIC_SENSORS_ADVANCED:
        fields = advanced_decode_sensor(message.payload)
        now = time.monotonic()
        if fields is None:
            with self.lock:
                self.advanced_sensor_invalid += 1
            self.add_event("Ignored invalid 42-byte sensor packet")
            return
        with self.lock:
            self.advanced_sensor_received += 1
            fields["t_s"] = round(now - self.advanced_start, 3)
            self.advanced_sensor = dict(fields)
            self.advanced_sensor_monotonic = now
            self.advanced_samples.append(dict(fields))
        advanced_record_sensor(self.recorder, fields)
        return

    if topic == TOPIC_CONTROLLER_ADVANCED:
        if len(message.payload) == 4:
            word = int.from_bytes(message.payload, "little")
            with self.lock:
                self.advanced_controller = {
                    "state": word & 0x7,
                    "fault_bits": (word >> 3) & 0xff,
                    "pending_turn": (word >> 11) & 0x7,
                    "symbol_code": (word >> 14) & 0x7f,
                    "sequence": (word >> 21) & 0xff,
                    "junctions_low3": (word >> 29) & 0x7,
                }
                self.advanced_controller_monotonic = time.monotonic()
        else:
            self.add_event("Ignored invalid 4-byte controller packet")
        return

    if topic == TOPIC_TEST:
        message_text = message.payload.decode("utf-8", errors="replace").strip()
        now = time.monotonic()
        terrain_match = TERRAIN_ADVANCED_RE.search(message_text)
        old_match = OLD_TERRAIN_ADVANCED_RE.search(message_text)
        if terrain_match or old_match:
            with self.lock:
                sensor = self.advanced_sensor
                distance = (sensor.get("trip_distance_mm")
                            if sensor and sensor.get("motion_valid") else None)
                t_s = round(now - self.advanced_start, 3)
            if terrain_match:
                seq = int(terrain_match.group(1))
                classification = terrain_match.group(2).upper()
                peak = int(terrain_match.group(3))
                duration = (int(terrain_match.group(4))
                            if terrain_match.group(4) else None)
                unique_id = (seq, classification)
            else:
                seq = None
                classification = "HUMP_DONE"
                peak = int(old_match.group(1))
                duration = None
                unique_id = (message_text,)
            entry = {"t_s": t_s, "sequence": seq,
                     "classification": classification, "peak_mm": peak,
                     "duration_ms": duration, "trip_distance_mm": distance}
            with self.lock:
                if unique_id not in self.advanced_seen_terrain:
                    self.advanced_seen_terrain.add(unique_id)
                    self.advanced_terrain.append(entry)
                    advanced_record = True
                else:
                    advanced_record = False
            if advanced_record:
                advanced_record_terrain(self.recorder, entry)
        if ADVANCED_EVENT_RE.search(message_text):
            with self.lock:
                self.advanced_events.append({
                    "t_s": round(now - self.advanced_start, 3),
                    "tag": event_tag(message_text) or "EVENT",
                    "text": message_text[:250],
                })
    result = _advanced_original_message(self, client, userdata, message)
    if topic == TOPIC_RUN and len(message.payload) == RUN_BYTES:
        # This is only an observed change of the last letter, not a claim
        # that the vehicle necessarily executed or completed that turn.
        with self.lock:
            run = self.run
            if run is not None:
                self.advanced_run_history.append({
                    **run, "t_s": round(time.monotonic() - self.advanced_start, 3)
                })
                letter = run.get("letter")
                if letter and letter != self.advanced_barcode_last:
                    self.advanced_barcode_trace.append({
                        "t_s": round(time.monotonic() - self.advanced_start, 3),
                        "letter": letter,
                        "mapped_action": self.mapping.get(letter),
                    })
                self.advanced_barcode_last = letter
    return result


def advanced_snapshot(feed):
    with feed.lock:
        return {
            "connected": feed.connected,
            "sensor": (dict(feed.advanced_sensor)
                       if feed.advanced_sensor is not None else None),
            "sensor_monotonic": feed.advanced_sensor_monotonic,
            "controller": (dict(feed.advanced_controller)
                           if feed.advanced_controller is not None else None),
            "controller_monotonic": feed.advanced_controller_monotonic,
            "sensors_received": feed.advanced_sensor_received,
            "invalid_sensors": feed.advanced_sensor_invalid,
            "samples": list(feed.advanced_samples),
            "terrain": list(feed.advanced_terrain),
            "events": list(feed.advanced_events),
            "run": dict(feed.run) if feed.run else None,
            "run_monotonic": feed.run_monotonic,
            "run_history": list(feed.advanced_run_history),
            "barcode_trace": list(feed.advanced_barcode_trace),
            "mapping": dict(feed.mapping),
            "run_received": feed.run_received,
            "run_lost": feed.run_lost,
        }


def advanced_route(samples):
    """Local x/y from *encoder-derived* heading + trip-distance deltas.

    This is not GPS, not localization, and will drift on slip/turns.
    Missing motion validity produces a break; no values are fabricated.
    """
    positions = []
    x = 0.0
    y = 0.0
    previous = None
    segment = 0
    for item in samples:
        if not item.get("motion_valid"):
            previous = None
            continue
        distance = item.get("trip_distance_mm")
        heading = item.get("heading_deg")
        timestamp = item.get("t_s")
        if not all(isinstance(v, (float, int)) for v in
                   (distance, heading, timestamp)):
            previous = None
            continue
        if previous is not None:
            delta = distance - previous["trip_distance_mm"]
            # Reset/overflow and teleport spikes are NOT drawn as travel.
            if 0 <= delta <= 1500:
                previous_heading = previous["heading_deg"]
                turn = ((heading - previous_heading + 180.0) % 360.0) - 180.0
                angle = math.radians(previous_heading + 0.5 * turn)
                x += delta * math.cos(angle)
                y += delta * math.sin(angle)
            else:
                previous = None
                segment += 1
        positions.append({"t_s": timestamp, "x_m": x / 1000.0,
                          "y_m": y / 1000.0,
                          "distance_m": distance / 1000.0,
                          "heading_deg": heading, "segment": segment})
        previous = item
    return pd.DataFrame(positions)


ADVANCED_CHART_LABELS = (
    "Motion · actual vs requested wheel speed",
    "IMU · X / Y / Z acceleration",
    "Terrain · side lift and hump reports",
    "Route · estimated local odometry",
    "Navigation · barcode and safety events",
)


def advanced_charts(sensor_rows, terrain_rows, events_rows, live=False,
                    barcode_rows=None, mapping=None, run_rows=None,
                    selected_chart=ADVANCED_CHART_LABELS[0]):
    """Only one high-value LIVE panel is rendered at a time.

    No invented readings; missing values stay missing. Historical visualization
    was intentionally removed at the user's request.
    """
    sensors = pd.DataFrame(sensor_rows)
    terrain = pd.DataFrame(terrain_rows)
    events = pd.DataFrame(events_rows)
    run_frame = pd.DataFrame(run_rows or [])
    selected_index = ADVANCED_CHART_LABELS.index(selected_chart)

    if selected_index == 0:
        st.markdown("**Motor tracking & obstacle distance**")
        st.caption("Requested versus measured speeds are real Buddy 1 sensor "
                   "snapshots. A missing motion-valid flag excludes that sample.")
        if not sensors.empty and "motion_valid" in sensors:
            good = sensors[sensors["motion_valid"]].copy()
            if not good.empty:
                speed_fields = {
                    "requested_left_mm_s": "Left requested",
                    "measured_left_mm_s": "Left measured",
                    "requested_right_mm_s": "Right requested",
                    "measured_right_mm_s": "Right measured",
                }
                # Requested speeds originate from the line snapshot. Its
                # values are unspecified when line_valid is false, even
                # though the payload bytes are zero-initialized.
                good.loc[~good["line_valid"],
                         ["requested_left_mm_s", "requested_right_mm_s"]] = float("nan")
                speeds = good.melt(id_vars=["t_s"],
                                   value_vars=list(speed_fields),
                                   var_name="Signal", value_name="Speed (mm/s)")
                speeds["Signal"] = speeds["Signal"].map(speed_fields)
                speeds = speeds.dropna(subset=["Speed (mm/s)"])
                line = alt.Chart(speeds).mark_line(strokeWidth=2).encode(
                    x=alt.X("t_s:Q", title="Monitoring session time (s)"),
                    y=alt.Y("Speed (mm/s):Q", title="Wheel speed (mm/s)"),
                    color=alt.Color("Signal:N", title="Wheel / source"),
                    strokeDash=alt.StrokeDash("Signal:N", legend=None),
                    tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                             "Signal:N", "Speed (mm/s):Q"],
                ).properties(height=310)
                st.altair_chart(line, use_container_width=True)
                latest = good.iloc[-1]
                a, b, c, d = st.columns(4)
                a.metric("Left measured", f"{int(latest['measured_left_mm_s'])} mm/s")
                b.metric("Left requested", f"{int(latest['requested_left_mm_s'])} mm/s"
                         if pd.notna(latest["requested_left_mm_s"]) else "Unavailable")
                c.metric("Right measured", f"{int(latest['measured_right_mm_s'])} mm/s")
                d.metric("Right requested", f"{int(latest['requested_right_mm_s'])} mm/s"
                         if pd.notna(latest["requested_right_mm_s"]) else "Unavailable")
            else:
                st.info("No motion-valid sensor packets received yet.")
        else:
            st.info("Waiting for actual sensor packets on inf2004/robot/sensors.")
        if not run_frame.empty and {"t_s", "range"}.issubset(run_frame.columns):
            ranges = run_frame[run_frame["range"] > 0].copy()
            if not ranges.empty:
                st.markdown("**Ultrasonic range over time**")
                st.altair_chart(alt.Chart(ranges).mark_line(point=True).encode(
                    x=alt.X("t_s:Q", title="Monitoring session time (s)"),
                    y=alt.Y("range:Q", title="Reported range (mm)"),
                    tooltip=[alt.Tooltip("t_s:Q", format=".1f"), "range:Q"],
                ).properties(height=190), use_container_width=True)
                st.caption("Zero/no-echo readings are excluded; range is not a GPS measurement.")

    elif selected_index == 1:
        st.markdown("**3-axis acceleration from Buddy 4**")
        st.caption("Raw IMU LSB values, not calibrated m/s² or g. Only valid "
                   "IMU snapshots are plotted.")
        if not sensors.empty and "imu_valid" in sensors:
            good = sensors[sensors["imu_valid"]].copy()
            if not good.empty:
                plotted = good.melt(id_vars=["t_s"],
                                    value_vars=["accel_x", "accel_y", "accel_z"],
                                    var_name="Axis", value_name="Acceleration (raw LSB)")
                graph = alt.Chart(plotted).mark_line(strokeWidth=2).encode(
                    x=alt.X("t_s:Q", title="Monitoring session time (s)"),
                    y=alt.Y("Acceleration (raw LSB):Q"),
                    color=alt.Color("Axis:N"),
                    tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                             "Axis:N", "Acceleration (raw LSB):Q"],
                ).properties(height=320)
                st.altair_chart(graph, use_container_width=True)
                latest = good.iloc[-1]
                a, b, c = st.columns(3)
                a.metric("X (LSB)", int(latest["accel_x"]))
                b.metric("Y (LSB)", int(latest["accel_y"]))
                c.metric("Z (LSB)", int(latest["accel_z"]))
            else:
                st.info("Sensor feed is present but has no valid IMU readings.")
        else:
            st.info("Waiting for valid IMU data from the 42-byte MQTT sensor feed.")

    elif selected_index == 2:
        st.markdown("**Terrain side lift and confirmed robot reports**")
        st.caption("Side lift is an orientation-derived estimate; it is not "
                   "a surveyed ground elevation. Hump peaks are reported by "
                   "Buddy 4/VehicleController, not generated by the dashboard.")
        if not run_frame.empty and {"distance", "lift", "t_s"}.issubset(run_frame.columns):
            profile = run_frame.sort_values("t_s").copy()
            travelled, total, previous = [], 0, None
            for distance in profile["distance"]:
                if previous is not None:
                    raw_delta = int(distance) - int(previous)
                    # Reject resets or implausible jumps; counter wrap is
                    # handled only for a plausible change close to the boundary.
                    delta = raw_delta if raw_delta >= 0 else raw_delta + DISTANCE_WRAP
                    if 0 <= delta <= 1500:
                        total += delta
                travelled.append(total / 1000.0)
                previous = distance
            profile["travel_m"] = travelled
            st.altair_chart(alt.Chart(profile).mark_line(strokeWidth=2).encode(
                x=alt.X("travel_m:Q", title="Distance travelled (m)"),
                y=alt.Y("lift:Q", title="Estimated side lift (mm)"),
                tooltip=[alt.Tooltip("t_s:Q", title="Time (s)", format=".1f"),
                         alt.Tooltip("travel_m:Q", format=".2f"), "lift:Q"],
            ).properties(height=260), use_container_width=True)
        else:
            st.info("Waiting for run telemetry to plot side lift along distance.")
        if not terrain.empty and "peak_mm" in terrain:
            report = terrain[terrain["classification"].isin(["ROLL_HUMP", "HUMP_DONE"])].copy()
            if not report.empty:
                st.markdown("**Reported hump peaks (individual events)**")
                report["report_order"] = range(1, len(report) + 1)
                st.altair_chart(alt.Chart(report).mark_point(
                    filled=True, size=95).encode(
                        x=alt.X("report_order:Q", title="Hump report number"),
                        y=alt.Y("peak_mm:Q", title="Estimated peak (mm)"),
                        tooltip=["report_order:Q", "peak_mm:Q", "classification:N"],
                    ).properties(height=220), use_container_width=True)
                a, b = st.columns(2)
                a.metric("Reported hump events", len(report))
                b.metric("Maximum estimated height", f"{report['peak_mm'].max():.0f} mm")
            else:
                st.info("No ROLL_HUMP terrain reports received yet.")
        else:
            st.info("Waiting for actual TERRAIN_REPORT messages from VehicleController.")

    elif selected_index == 3:
        st.markdown("**Estimated travelled route (relative X/Y)**")
        st.caption("Reconstructed from reported encoder trip distance and "
                   "heading. Origin is (0, 0). Not GPS: wheel slip and "
                   "turns can cause substantial drift.")
        path = advanced_route(sensor_rows)
        if len(path) >= 2:
            trace = alt.Chart(path).mark_line(strokeWidth=3).encode(
                x=alt.X("x_m:Q", title="Relative X (m)", scale=alt.Scale(zero=False)),
                y=alt.Y("y_m:Q", title="Relative Y (m)", scale=alt.Scale(zero=False)),
                order="t_s:Q", detail="segment:N",
                tooltip=[alt.Tooltip("t_s:Q", title="Time (s)", format=".1f"),
                         alt.Tooltip("x_m:Q", format=".2f"),
                         alt.Tooltip("y_m:Q", format=".2f"), "heading_deg:Q"],
            )
            starts = alt.Chart(path.head(1)).mark_point(
                size=150, filled=True, color="#2eaf73").encode(x="x_m:Q", y="y_m:Q")
            ends = alt.Chart(path.tail(1)).mark_point(
                size=150, filled=True, color="#ed5c5c").encode(x="x_m:Q", y="y_m:Q")
            chart = trace + starts + ends
            markers = []
            if not events.empty and "t_s" in events:
                for _, event in events.iterrows():
                    if event.get("tag") not in ("HUMP", "BARCODE", "JUNCTION",
                                                 "OBSTACLE", "LINK LOST", "E-STOP", "FAULT"):
                        continue
                    near = path.loc[(path["t_s"] - event["t_s"]).abs().idxmin()]
                    if abs(float(near["t_s"]) - float(event["t_s"])) <= 1.5:
                        markers.append({"x_m": near["x_m"], "y_m": near["y_m"],
                                        "tag": event["tag"], "text": event["text"]})
            if markers:
                chart += alt.Chart(pd.DataFrame(markers)).mark_point(
                    filled=True, size=95).encode(x="x_m:Q", y="y_m:Q",
                                                   color="tag:N", tooltip=["tag:N", "text:N"])
            st.altair_chart(chart.properties(height=380), use_container_width=True)
            end = path.iloc[-1]
            a, b, c = st.columns(3)
            a.metric("Relative X", f"{end['x_m']:.2f} m")
            b.metric("Relative Y", f"{end['y_m']:.2f} m")
            c.metric("Last heading", f"{end['heading_deg']:.0f}°")
            st.caption("Green: first sample · Red: most recent sample · event markers use nearest available sensor sample.")
        else:
            st.info("Need at least two valid encoder distance + heading samples. "
                    "Check inf2004/robot/sensors and the motion-valid flag.")

    elif selected_index == 4:
        st.markdown("**Observed barcode instructions and safety events**")
        st.caption("This is real reported navigation history, not a claim "
                   "that every requested turn was successfully executed.")
        left, right = st.columns(2)
        with left:
            st.markdown("**Accepted MQTT mapping**")
            if mapping:
                st.dataframe(pd.DataFrame([
                    {"Barcode": letter, "Assigned action": action}
                    for letter, action in sorted(mapping.items())
                ]), hide_index=True, use_container_width=True)
            else:
                st.info("No accepted mapping received this session.")
        with right:
            st.markdown("**Observed barcode sequence**")
            if barcode_rows:
                table = pd.DataFrame(barcode_rows)
                st.dataframe(table[[c for c in ("t_s", "letter", "mapped_action")
                                    if c in table]], hide_index=True,
                             use_container_width=True)
            else:
                st.info("No barcode observations received.")
        st.markdown("**Safety and navigation events**")
        if not events.empty:
            st.dataframe(events[[c for c in ("t_s", "tag", "text")
                                 if c in events]], hide_index=True,
                         use_container_width=True, height=270)
        else:
            st.info("No relevant robot events received.")


@st.fragment(run_every="4s")
def advanced_live_dashboard(feed):
    # The title and chart selector are outside the fragment, so changing data
    # updates only the chart panel instead of blinking its navigation controls.
    data = advanced_snapshot(feed)
    now = time.monotonic()
    sensor = data["sensor"]
    sensor_age = (now - data["sensor_monotonic"]
                  if data["sensor_monotonic"] is not None else None)
    sensor_fresh = bool(data["connected"] and sensor is not None
                        and sensor_age is not None and sensor_age <= 2.0)
    run = data["run"]
    run_age = (now - data["run_monotonic"]
               if data["run_monotonic"] is not None else None)
    latest_run_fresh = bool(data["connected"] and run is not None
                            and run_age is not None and run_age <= 2.0)
    a, b, c, d = st.columns(4)
    a.metric("Sensor feed", "Live" if sensor_fresh else "Stale / unavailable")
    b.metric("Sensor packet age",
             f"{sensor_age:.1f} s" if sensor_age is not None else "Unavailable")
    c.metric("Sensor packets", data["sensors_received"])
    d.metric("Invalid sensor packets", data["invalid_sensors"])
    if latest_run_fresh:
        if run["faults"]:
            st.error("VehicleController latched fault(s): " +
                     ", ".join(run["fault_names"]))
        elif run["state_name"] == "STOPPED":
            st.info("VehicleController reports STOPPED.")
    if not sensor_fresh:
        st.warning("Current sensor values cannot be confirmed. Historical "
                   "samples below are retained for analysis only.")
    else:
        left, right = st.columns(2)
        with left:
            st.markdown("**Motion and safety · live**")
            st.dataframe(pd.DataFrame([
                {"Reading": "Measured left wheel", "Value": sensor["measured_left_mm_s"], "Unit": "mm/s"},
                {"Reading": "Measured right wheel", "Value": sensor["measured_right_mm_s"], "Unit": "mm/s"},
                {"Reading": "Trip distance", "Value": sensor["trip_distance_mm"], "Unit": "mm"},
                {"Reading": "Encoder heading", "Value": sensor["heading_deg"], "Unit": "degrees"},
                {"Reading": "Emergency stop requested", "Value": str(sensor["stop_required"]), "Unit": "flag"},
            ]), hide_index=True, use_container_width=True)
        with right:
            st.markdown("**Perception and IMU · live**")
            st.dataframe(pd.DataFrame([
                {"Reading": "Line error", "Value": sensor["line_error"] if sensor["line_valid"] else "Unavailable", "Unit": "raw"},
                {"Reading": "On hump", "Value": str(sensor["on_hump"]) if sensor["hump_valid"] else "Unavailable", "Unit": "flag"},
                {"Reading": "IMU X", "Value": sensor["accel_x"] if sensor["imu_valid"] else "Unavailable", "Unit": "LSB"},
                {"Reading": "IMU Y", "Value": sensor["accel_y"] if sensor["imu_valid"] else "Unavailable", "Unit": "LSB"},
                {"Reading": "IMU Z", "Value": sensor["accel_z"] if sensor["imu_valid"] else "Unavailable", "Unit": "LSB"},
            ]), hide_index=True, use_container_width=True)
    # Two balanced columns of always-visible live charts; no dropdown.
    # Every chart uses the same MQTT snapshot for a consistent refresh.
    panels = ADVANCED_CHART_LABELS
    chart_data = (data["samples"], data["terrain"], data["events"])
    chart_options = {
        "live": True,
        "barcode_rows": data["barcode_trace"],
        "mapping": data["mapping"],
        "run_rows": data["run_history"],
    }

    left, right = st.columns(2, gap="large")
    with left:
        with st.container(border=True):
            advanced_charts(*chart_data, selected_chart=panels[0],
                            **chart_options)
        with st.container(border=True):
            advanced_charts(*chart_data, selected_chart=panels[2],
                            **chart_options)
    with right:
        with st.container(border=True):
            advanced_charts(*chart_data, selected_chart=panels[1],
                            **chart_options)
        with st.container(border=True):
            advanced_charts(*chart_data, selected_chart=panels[3],
                            **chart_options)

    with st.container(border=True):
        advanced_charts(*chart_data, selected_chart=panels[4],
                        **chart_options)




# ================================================================
# Live classic charts — preserved from the original run viewer,
# now shown together as responsive live plots (not duplicated in history).
# This section only reads genuine 15-byte /run telemetry already decoded
# by RobotFeed. No zero-filled or simulated measurements are generated.
# ================================================================

def _classic_live_data(run_rows):
    """Prepare received run packets; avoid distance rollover/reset spikes."""
    if not run_rows:
        return pd.DataFrame()
    frame = pd.DataFrame(run_rows).sort_values("t_s").copy()
    frame = frame.drop_duplicates(subset=["t_s"], keep="last")
    if frame.empty:
        return frame

    distance_m = []
    total_mm = 0.0
    previous = None
    prev_uptime = None
    for row in frame.itertuples(index=False):
        counter = int(row.distance)
        uptime = int(row.uptime)
        if previous is not None:
            # The 20-bit counter can wrap. A reset/reboot instead creates
            # a jump that must never be interpreted as metres of travel.
            step = (counter - previous) % DISTANCE_WRAP
            if (prev_uptime is not None and uptime >= prev_uptime
                    and 0 <= step <= 1500):
                total_mm += step
        distance_m.append(total_mm / 1000.0)
        previous = counter
        prev_uptime = uptime
    frame["travel_m"] = distance_m
    frame["range_mm"] = frame["range"].where(frame["range"] > 0)
    return frame


def _classic_line(frame, field, y_title, height=220):
    return alt.Chart(frame).mark_line(strokeWidth=2).encode(
        x=alt.X("t_s:Q", title="Session time (s)"),
        y=alt.Y(f"{field}:Q", title=y_title),
        tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                 alt.Tooltip(f"{field}:Q")],
    ).properties(height=height)


@st.fragment(run_every="5s")
def live_classic_charts(feed):
    """Original six useful charts, live and without a historical duplicate."""
    feed_data = advanced_snapshot(feed)
    frame = _classic_live_data(feed_data["run_history"])
    st.subheader("Live driving & track sensor charts")
    st.caption("These six charts use real 15-byte VehicleController run packets. "
               "They update every five seconds and retain received session "
               "history; no readings are invented while offline.")
    if frame.empty:
        st.info("Waiting for robot run telemetry on inf2004/robot/run.")
        return

    # 1 — the original live left/right wheel-speed chart, including negative
    # speeds when reversing. This complements the requested/actual comparison.
    with st.container(border=True):
        st.markdown("**Wheel speeds — left and right**")
        wheels = frame.melt(id_vars=["t_s"],
                            value_vars=["speed_l", "speed_r"],
                            var_name="Wheel", value_name="Speed (mm/s)")
        wheels["Wheel"] = wheels["Wheel"].map({"speed_l": "Left", "speed_r": "Right"})
        plot = alt.Chart(wheels).mark_line(strokeWidth=2).encode(
            x=alt.X("t_s:Q", title="Session time (s)"),
            y=alt.Y("Speed (mm/s):Q", title="Speed (mm/s)"),
            color=alt.Color("Wheel:N"),
            tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                     "Wheel:N", "Speed (mm/s):Q"],
        ).properties(height=240)
        st.altair_chart(plot, use_container_width=True)

    # 2 — categorical state timeline; duplicate samples of the same state
    # are merged to give correctly sized segments.
    with st.container(border=True):
        st.markdown("**Robot state timeline**")
        segments = []
        for row in frame.itertuples(index=False):
            label = str(row.state_name)
            t = float(row.t_s)
            if segments and segments[-1]["State"] == label:
                segments[-1]["End"] = t
            else:
                if segments:
                    segments[-1]["End"] = t
                segments.append({"State": label, "Start": t, "End": t})
        if segments:
            segments[-1]["End"] = max(segments[-1]["End"],
                                      float(frame["t_s"].iloc[-1]) + 0.2)
            timeline = alt.Chart(pd.DataFrame(segments)).mark_rect().encode(
                x=alt.X("Start:Q", title="Session time (s)"),
                x2="End:Q",
                y=alt.Y("State:N", title=None, sort=list(STATE_COLOURS)),
                color=alt.Color("State:N", scale=alt.Scale(
                    domain=list(STATE_COLOURS),
                    range=list(STATE_COLOURS.values()))),
                tooltip=["State:N", alt.Tooltip("Start:Q", format=".1f"),
                         alt.Tooltip("End:Q", format=".1f")],
            ).properties(height=155)
            st.altair_chart(timeline, use_container_width=True)

    # Remaining four original plots, balanced to avoid horizontal crowding.
    left, right = st.columns(2, gap="large")
    with left:
        with st.container(border=True):
            st.markdown("**Distance travelled since monitoring began**")
            st.altair_chart(_classic_line(frame, "travel_m", "Travelled (m)"),
                            use_container_width=True)
            st.caption("Encoder-derived, reset-aware distance; not GPS.")
        with st.container(border=True):
            st.markdown("**Side lift and maximum reported hump peak**")
            shaped = frame.melt(id_vars=["t_s"],
                                value_vars=["lift", "hump"],
                                var_name="Measure", value_name="Height (mm)")
            shaped["Measure"] = shaped["Measure"].map({
                "lift": "Side lift", "hump": "Run max hump peak"})
            lift = alt.Chart(shaped).mark_line(strokeWidth=2).encode(
                x=alt.X("t_s:Q", title="Session time (s)"),
                y=alt.Y("Height (mm):Q", title="Estimated height (mm)"),
                color="Measure:N", strokeDash="Measure:N",
                tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                         "Measure:N", "Height (mm):Q"],
            ).properties(height=220)
            st.altair_chart(lift, use_container_width=True)
            st.caption("Estimated IMU side lift and reported run peak, not ground-truth elevation.")
    with right:
        with st.container(border=True):
            st.markdown("**Ultrasonic obstacle distance**")
            good_range = frame[frame["range_mm"].notna()]
            if good_range.empty:
                st.info("No nonzero ultrasonic ranges received.")
            else:
                ranges = alt.Chart(good_range).mark_line(point=True).encode(
                    x=alt.X("t_s:Q", title="Session time (s)"),
                    y=alt.Y("range_mm:Q", title="Range (mm)"),
                    tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                             "range_mm:Q"],
                ).properties(height=220)
                st.altair_chart(ranges, use_container_width=True)
            st.caption("Zero/no-echo values are excluded rather than plotted as obstacles.")
        with st.container(border=True):
            st.markdown("**Left and right line sensors**")
            irs = frame[["t_s", "ir_l", "ir_r"]].melt(
                id_vars=["t_s"], value_vars=["ir_l", "ir_r"],
                var_name="Sensor", value_name="Black detected")
            irs["Sensor"] = irs["Sensor"].map({"ir_l": "Left", "ir_r": "Right"})
            irs["Black detected"] = irs["Black detected"].astype(int)
            ir_plot = alt.Chart(irs).mark_line(interpolate="step-after").encode(
                x=alt.X("t_s:Q", title="Session time (s)"),
                y=alt.Y("Black detected:Q", title="Black (1) / white (0)",
                        scale=alt.Scale(domain=[0, 1])),
                color="Sensor:N",
                tooltip=[alt.Tooltip("t_s:Q", format=".1f"),
                         "Sensor:N", "Black detected:Q"],
            ).properties(height=220)
            st.altair_chart(ir_plot, use_container_width=True)


# Extend rather than replace the original MQTT handling and run logic.
_advanced_original_init = RobotFeed.__init__
_advanced_original_connect = RobotFeed.on_connect
_advanced_original_message = RobotFeed.on_message
RobotFeed.__init__ = advanced_initialize_feed
RobotFeed.on_connect = advanced_on_connect
RobotFeed.on_message = advanced_on_message


if __name__ == "__main__":

    main()
