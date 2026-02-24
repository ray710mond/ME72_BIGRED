#!/usr/bin/env python3
"""
rf_driver_ibus.py

ROS2 node that reads FlySky iBUS from a UART (/dev/ttyAMA*, /dev/ttyUSB*, etc.)
and publishes:
  - Twist on `cmd_vel_des`
  - Bool on `intake_running`
  - Bool on `outtake_open`

Replaces 6x PWM GPIO edge-capture with a single serial iBUS connection.

Wiring (typical FS-iA6B / FS-iA6 / FS-iA10B class receivers):
  Receiver iBUS (signal) -> Raspberry Pi UART RX (3.3V TTL)
  Receiver GND           -> Pi GND
  Receiver VCC (5V)      -> 5V supply for receiver (NOT Pi 3.3V)

Pi 5 note:
  Use a free UART and ensure it's enabled (e.g., /dev/ttyAMA10 in your setup).

iBUS frame:
  32 bytes total: [0]=0x20 (len), [1]=0x40 (cmd), [2..29]=14 channels * 2 bytes LE,
  [30..31]=checksum (0xFFFF - sum(bytes[0..29])) little-endian.
"""

import time
import threading
import serial  # pip install pyserial (or apt install python3-serial)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# ----------------- USER SETTINGS -----------------

# Serial device for iBUS
IBUS_PORT = "/dev/ttyAMA10"   # <-- CHANGE to your actual UART device
IBUS_BAUD = 115200            # iBUS is typically 115200 8N1

# Channel mapping (iBUS channels are 0-based in the code below)
# Your old code used: THROTTLE_CHNL=2, STEER_CHNL=4, INTAKE_CHNL=5, OUTTAKE_CHNL=6
THROTTLE_CHNL = 2
STEER_CHNL    = 4
INTAKE_CHNL   = 5
OUTTAKE_CHNL  = 6

THROTTLE_IDX = THROTTLE_CHNL - 1
STEER_IDX    = STEER_CHNL - 1
INTAKE_IDX   = INTAKE_CHNL - 1
OUTTAKE_IDX  = OUTTAKE_CHNL - 1

# iBUS typical value range is ~1000..2000, center ~1500 (sometimes 988..2012)
PULSE_MIN_US    = 1000.0
PULSE_CENTER_US = 1500.0
PULSE_MAX_US    = 2000.0
DEADBAND_US     = 40.0

# Control loop
LOOP_DT = 0.02  # 50 Hz

# Gains (keep same as you had)
THROTTLE_GAIN = 1.0
STEER_GAIN    = 0.25


# ----------------- MATH HELPERS -----------------

def pulse_to_norm(
    pulse_us,
    center=PULSE_CENTER_US,
    min_us=PULSE_MIN_US,
    max_us=PULSE_MAX_US,
    deadband_us=DEADBAND_US
):
    """Map RC pulse (μs) to −1..+1 with deadband."""
    if pulse_us is None:
        return 0.0

    if pulse_us < min_us:
        pulse_us = min_us
    elif pulse_us > max_us:
        pulse_us = max_us

    offset = pulse_us - center

    if abs(offset) < deadband_us:
        return 0.0

    if offset > 0:
        return min(1.0, offset / (max_us - center))
    else:
        return max(-1.0, offset / (center - min_us))


def mix_steer_throttle(throttle, steer):
    """Differential mix (debug only)."""
    left  = throttle + steer
    right = throttle - steer
    left  = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))
    return left, right


def _fmt_pw(pw):
    if pw is None:
        return "  ---- "
    return f"{pw:6.1f}"


# ----------------- iBUS READER -----------------

class IBusReader:
    """
    Background thread that reads iBUS frames from serial and exposes latest channels.

    - channels_us: list of 14 ints (microseconds-like values), or None until first valid frame
    - last_frame_time: monotonic timestamp of last valid frame
    """
    FRAME_LEN = 32
    FRAME_START_LEN = 0x20
    FRAME_CMD = 0x40
    N_CHANNELS = 14

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.02):
        self.port = port
        self.baud = baud
        self.timeout = timeout

        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self._lock = threading.Lock()
        self.channels_us = None
        self.last_frame_time = None

        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        print(f"iBUS: listening on {self.port} @ {self.baud} baud")

    @staticmethod
    def _checksum_ok(frame: bytes) -> bool:
        if len(frame) != IBusReader.FRAME_LEN:
            return False
        # checksum is little-endian in last 2 bytes
        rx_ck = frame[30] | (frame[31] << 8)
        s = sum(frame[0:30]) & 0xFFFF
        calc = (0xFFFF - s) & 0xFFFF
        return rx_ck == calc

    @staticmethod
    def _parse_channels(frame: bytes):
        # channels are 2 bytes little-endian, 14 channels from bytes 2..29
        ch = []
        for i in range(IBusReader.N_CHANNELS):
            lo = frame[2 + 2*i]
            hi = frame[2 + 2*i + 1]
            ch.append(lo | (hi << 8))
        return ch

    def _find_frame(self, buf: bytearray):
        """
        Try to find and pop one full 32-byte frame from buf.
        Returns frame bytes or None.
        """
        # Search for potential frame start
        for start in range(len(buf)):
            if buf[start] != self.FRAME_START_LEN:
                continue
            if start + self.FRAME_LEN > len(buf):
                return None  # need more data
            candidate = bytes(buf[start:start + self.FRAME_LEN])
            # quick header check
            if candidate[1] != self.FRAME_CMD:
                continue
            if not self._checksum_ok(candidate):
                continue
            # Remove everything up to end of frame
            del buf[:start + self.FRAME_LEN]
            return candidate
        # If no start found, keep buffer from growing unbounded
        if len(buf) > 4 * self.FRAME_LEN:
            del buf[:-self.FRAME_LEN]
        return None

    def _run(self):
        buf = bytearray()
        while not self._stop:
            try:
                data = self._ser.read(64)  # small chunk
                if data:
                    buf.extend(data)

                frame = self._find_frame(buf)
                if frame is None:
                    continue

                chans = self._parse_channels(frame)
                now = time.monotonic()
                with self._lock:
                    self.channels_us = chans
                    self.last_frame_time = now

            except Exception:
                # If serial hiccups, don't kill the node; just pause briefly.
                time.sleep(0.05)

    def read_channels(self):
        """Return a copy of latest channels list, or None if no valid frame yet."""
        with self._lock:
            if self.channels_us is None:
                return None
            return list(self.channels_us)

    def age_s(self):
        """Seconds since last valid frame, or None if never."""
        with self._lock:
            if self.last_frame_time is None:
                return None
            return time.monotonic() - self.last_frame_time

    def close(self):
        self._stop = True
        try:
            self._thread.join(timeout=0.2)
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass


# ----------------- MAIN NODE -----------------

class RfDriverIbusNode(Node):
    def __init__(self):
        super().__init__("rf_driver_ibus")

        self.intake_pub = self.create_publisher(Bool, "intake_running", 10)
        self.outtake_pub = self.create_publisher(Bool, "outtake_open", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_des", 10)

        self.ibus = IBusReader(IBUS_PORT, IBUS_BAUD, timeout=0.02)

        self._timer = self.create_timer(LOOP_DT, self._tick)

        self.get_logger().info("Started iBUS → cmd_vel_des / intake_running / outtake_open")

    def _tick(self):
        chans = self.ibus.read_channels()

        # Fail-safe: if no data yet or stale, publish zeros
        age = self.ibus.age_s()
        if chans is None or (age is not None and age > 0.25):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            # You can choose to force intake/outtake false on failsafe:
            msg = Bool()
            msg.data = False
            self.intake_pub.publish(msg)
            self.outtake_pub.publish(msg)

            if chans is None:
                self.get_logger().debug("Waiting for first valid iBUS frame...")
            else:
                self.get_logger().warn(f"iBUS stale ({age:.2f}s) -> zero commands")
            return

        # Convert channels to norms using your old pulse mapping
        # chans are int values in ~1000..2000 range
        pulses = [float(v) for v in chans]  # mimic "us" values

        # Ensure we have enough channels (iBUS usually 14)
        def safe_get(idx):
            if 0 <= idx < len(pulses):
                return pulses[idx]
            return None

        throttle_pw = safe_get(THROTTLE_IDX)
        steer_pw = safe_get(STEER_IDX)
        intake_pw = safe_get(INTAKE_IDX)
        outtake_pw = safe_get(OUTTAKE_IDX)

        throttle = pulse_to_norm(throttle_pw) * THROTTLE_GAIN
        steer = pulse_to_norm(steer_pw) * STEER_GAIN

        left_cmd, right_cmd = mix_steer_throttle(throttle, steer)

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(throttle)
        twist.linear.y = 0.0
        twist.angular.z = float(steer)
        self.cmd_pub.publish(twist)

        # Intake/outtake switches
        if intake_pw is not None:
            msg = Bool()
            msg.data = bool(intake_pw > PULSE_CENTER_US)
            self.intake_pub.publish(msg)

        if outtake_pw is not None:
            msg = Bool()
            msg.data = bool(outtake_pw > PULSE_CENTER_US)
            self.outtake_pub.publish(msg)

        # Debug print similar to your old output (first 6 channels)
        ch_parts = []
        for i in range(6):
            pw = pulses[i] if i < len(pulses) else None
            norm = pulse_to_norm(pw) if pw is not None else 0.0
            ch_parts.append(f"CH{i+1}={_fmt_pw(pw)}us({norm:+5.2f})")
        ch_str = "  ".join(ch_parts)
        print(f"{ch_str}   L={left_cmd:+.2f} R={right_cmd:+.2f}")

    def destroy_node(self):
        try:
            self.ibus.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RfDriverIbusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()