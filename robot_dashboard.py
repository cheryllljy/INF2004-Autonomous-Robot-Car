
"""Live MQTT dashboard for the INF2004 robot car (with test buttons)."""

import ipaddress
import os
import re
import time
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import paho.mqtt.client as mqtt
import streamlit as st


ROOT = Path(__file__).resolve().parent

TOPIC_MAPPING = "inf2004/robot/mapping"
TOPIC_CONNECTION = "inf2004/robot/connection"
TOPIC_TELEMETRY = "inf2004/robot/telemetry"
TOPIC_HEARTBEAT = "inf2004/robot/heartbeat"
TOPIC_TEST = "inf2004/robot/test"

# nyx, 2026-11-09: start/stop buttons for testing. The same FR2 command
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


st.set_page_config(
    page_title="INF2004 Car Telemetry",
    layout="wide"
)

try:
    BROKER, PORT = load_broker_config()
except (OSError, ValueError) as error:
    st.error(f"MQTT configuration error: {error}")
    st.stop()


class RobotFeed:
    """Receive real MQTT messages; send start/stop commands on request."""

    def __init__(self, host, port):
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

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2
        )

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        try:
            self.client.connect_async(
                host,
                port,
                keepalive=30
            )
            self.client.loop_start()
        except Exception as error:
            self.add_event(f"MQTT startup error: {error}")

    def add_event(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")

        with self.lock:
            self.events.appendleft(
                f"{timestamp}  {message}"
            )

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

        if connected:
            for topic in (
                TOPIC_MAPPING,
                TOPIC_CONNECTION,
                TOPIC_TELEMETRY,
                TOPIC_HEARTBEAT,
                TOPIC_TEST
            ):
                client.subscribe(topic, qos=1)

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
            )

            self.add_event(
                "Robot log: "
                + payload[:120].replace("\n", " ")
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
                "events": list(self.events)
            }


@st.cache_resource
def create_feed(host, port):
    return RobotFeed(host, port)


feed = create_feed(BROKER, PORT)

st.title("INF2004 | Robotic Car Telemetry")

st.caption(
    "Buddy 1 | MQTT dashboard | "
    "Live robot messages, start/stop buttons for testing"
)

st.write(f"Broker from project config: `{BROKER}:{PORT}`")

# nyx, 2026-11-09: drive the car from here for testing (same commands as
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


@st.fragment(run_every="1s")
def live_dashboard():
    data = feed.snapshot()

    broker_up = data["connected"]
    robot = (
        data["robot_state"]
        if broker_up
        else "UNKNOWN"
    )

    mapping = data["mapping"]
    telemetry = data["telemetry"]
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
        "Received" if telemetry else "Unavailable"
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

    st.subheader("Robot status and sensors")

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
            "No controller telemetry received yet. "
            "The MQTT heartbeat is shown separately above."
        )

    st.write(
        "**Latest scanned barcode:** "
        "Unavailable - scan feed not integrated"
    )

    st.write(
        "**Line following:** "
        "Unavailable - motion feed not integrated"
    )

    st.write(
        "**Hump height:** "
        "Unavailable - IMU feed not integrated"
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


live_dashboard()
