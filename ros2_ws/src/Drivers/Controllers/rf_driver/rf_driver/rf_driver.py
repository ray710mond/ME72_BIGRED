#!/usr/bin/env python3
"""
rf_driver_ibus.py

ROS2 node that reads FlySky iBUS from a UART (/dev/ttyAMA0, /dev/ttyS0, etc.)
and publishes:
  - Twist on `cmd_vel_teleop`
  - Bool on `intake_running`
  - Bool on `outtake_open`

Key robustness improvements vs the earlier version:
  - Serial opened with flow control disabled explicitly.
  - Reader thread parses ALL frames available in the buffer (keeps freshest data).
  - Frame counter + age diagnostics so you can see if frames are actually updating.
  - Periodic full 14-channel dump (2 Hz) via ROS logger (no stdout buffering issues).
  - Loud exception logging in reader thread (no silent failures).

NOTES (Pi 4):
  - If you use the GPIO header UART and want it to be /dev/ttyAMA0, set:
      enable_uart=1
      dtoverlay=disable-bt
    in /boot/firmware/config.txt and remove console=serial0,115200 from cmdline.
  - If ttyAMA0 is still used by serial-getty, disable/mask serial-getty@ttyAMA0.
"""

import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# ----------------- USER SETTINGS -----------------

# Pick the port you are actually wired to:
#   - /dev/ttyAMA0 (preferred hardware UART when BT disabled)
#   - /dev/ttyS0   (miniUART)
IBUS_PORT = "/dev/ttyAMA0"
IBUS_BAUD = 115200

# Channel mapping (1-based like RC conventions)
THROTTLE_CHNL = 2
STEER_CHNL    = 4
INTAKE_CHNL   = 5
OUTTAKE_CHNL  = 6
AUTO_CHNL     = 7
DOCK_CHNL     =    8

THROTTLE_IDX = THROTTLE_CHNL - 1
STEER_IDX    = STEER_CHNL - 1
INTAKE_IDX   = INTAKE_CHNL - 1
OUTTAKE_IDX  = OUTTAKE_CHNL - 1
AUTO_IDX     = AUTO_CHNL - 1
DOCK_IDX     = DOCK_CHNL - 1

# iBUS typical value range is ~1000..2000, center ~1500
PULSE_MIN_US    = 1000.0
PULSE_CENTER_US = 1500.0
PULSE_MAX_US    = 2000.0
DEADBAND_US     = 40.0

# Control loop
LOOP_DT = 0.02  # 50 Hz

# Gains (tune for your robot)
THROTTLE_GAIN = 1.0
# STEER_GAIN    = 0.25
STEER_GAIN    = 1.0

# Failsafe if no fresh frames within this time
STALE_S = 0.25


# ----------------- MATH HELPERS -----------------

def pulse_to_norm(
    pulse_us: float | None,
    center: float = PULSE_CENTER_US,
    min_us: float = PULSE_MIN_US,
    max_us: float = PULSE_MAX_US,
    deadband_us: float = DEADBAND_US
) -> float:
    """Map RC pulse (μs-like) to −1..+1 with deadband."""
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
    return max(-1.0, offset / (center - min_us))


def mix_steer_throttle(throttle: float, steer: float) -> tuple[float, float]:
    """Differential mix (debug only)."""
    left  = max(-1.0, min(1.0, throttle + steer))
    right = max(-1.0, min(1.0, throttle - steer))
    return left, right


# ----------------- iBUS READER -----------------

class IBusReader:
    """
    Background thread that reads iBUS frames from serial and exposes latest channels.

    - channels_us: list of 10 ints, or None until first valid frame
    - last_frame_time: monotonic timestamp of last valid frame
    - frame_count: increments every valid frame parsed
    """
    FRAME_LEN = 32
    FRAME_START_LEN = 0x20
    FRAME_CMD = 0x40
    N_CHANNELS = 10

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.02):
        self.port = port
        self.baud = baud
        self.timeout = timeout

        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout,
            # iBUS is 1-wire TX from receiver; no flow control
            rtscts=False,
            dsrdtr=False,
            xonxoff=False,
        )

        # Extra safety: force DTR/RTS low-ish if supported
        try:
            self._ser.setDTR(False)
            self._ser.setRTS(False)
        except Exception:
            pass

        self._lock = threading.Lock()
        self.channels_us: list[int] | None = None
        self.last_frame_time: float | None = None
        self.frame_count: int = 0

        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        print(f"iBUS: listening on {self.port} @ {self.baud} baud", flush=True)

    @staticmethod
    def _checksum_ok(frame: bytes) -> bool:
        if len(frame) != IBusReader.FRAME_LEN:
            return False
        rx_ck = frame[30] | (frame[31] << 8)
        s = sum(frame[0:30]) & 0xFFFF
        calc = (0xFFFF - s) & 0xFFFF
        return rx_ck == calc

    @staticmethod
    def _parse_channels(frame: bytes) -> list[int]:
        ch: list[int] = []
        for i in range(IBusReader.N_CHANNELS):
            lo = frame[2 + 2*i]
            hi = frame[2 + 2*i + 1]
            ch.append(lo | (hi << 8))
        return ch

    def _find_frame(self, buf: bytearray) -> bytes | None:
        """
        Try to find and pop one full 32-byte frame from buf.
        Returns frame bytes or None.
        """
        for start in range(len(buf)):
            if buf[start] != self.FRAME_START_LEN:
                continue
            if start + self.FRAME_LEN > len(buf):
                return None  # need more data
            candidate = bytes(buf[start:start + self.FRAME_LEN])

            if candidate[1] != self.FRAME_CMD:
                continue
            if not self._checksum_ok(candidate):
                continue

            del buf[:start + self.FRAME_LEN]
            return candidate

        # prevent unbounded growth if misaligned
        if len(buf) > 4 * self.FRAME_LEN:
            del buf[:-self.FRAME_LEN]
        return None

    def _run(self):
        buf = bytearray()
        while not self._stop:
            try:
                data = self._ser.read(256)
                if data:
                    buf.extend(data)

                # Parse ALL complete frames currently available
                parsed_any = False
                while True:
                    frame = self._find_frame(buf)
                    if frame is None:
                        break
                    chans = self._parse_channels(frame)
                    now = time.monotonic()
                    with self._lock:
                        self.channels_us = chans
                        self.last_frame_time = now
                        self.frame_count += 1
                    parsed_any = True

                # If we read nothing and parsed nothing, avoid a hot spin
                if not data and not parsed_any:
                    time.sleep(0.002)

            except Exception as e:
                print(f"iBUS reader exception: {repr(e)}", flush=True)
                time.sleep(0.2)

    def read_channels(self) -> list[int] | None:
        with self._lock:
            return None if self.channels_us is None else list(self.channels_us)

    def age_s(self) -> float | None:
        with self._lock:
            if self.last_frame_time is None:
                return None
            return time.monotonic() - self.last_frame_time

    def get_frame_count(self) -> int:
        with self._lock:
            return self.frame_count

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
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_teleop", 10)
        self.autonomous_active = self.create_publisher(Bool, "autonomous_active", 10)
        self.dock_active = self.create_publisher(Bool, "dock_active", 10)
        self.ibus = IBusReader(IBUS_PORT, IBUS_BAUD, timeout=0.02)

        self._last_dump = 0.0
        self._last_fc_logged = -1

        self._timer = self.create_timer(LOOP_DT, self._tick)
        self.get_logger().info(f"Started iBUS on {IBUS_PORT} @ {IBUS_BAUD}")

    def _publish_zero(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        msg = Bool()
        msg.data = False
        self.intake_pub.publish(msg)
        self.outtake_pub.publish(msg)
        self.autonomous_active.publish(msg)
        self.dock_active.publish(msg)

    def _tick(self):
        chans = self.ibus.read_channels()
        age = self.ibus.age_s()
        fc = self.ibus.get_frame_count()

        # Always log when frame_count changes (proves live updates)
        # if fc != self._last_fc_logged:
        #     self._last_fc_logged = fc
        #     self.get_logger().info(f"frames={fc} age={(age if age is not None else -1.0):.3f}s")

        # Failsafe
        if chans is None or (age is not None and age > STALE_S):
            self._publish_zero()
            if chans is None:
                self.get_logger().info("Waiting for first valid iBUS frame...")
            else:
                self.get_logger().warn(f"iBUS stale ({age:.2f}s) -> zero commands")
            return

        pulses = [float(v) for v in chans]
        pulses = [1500.0 if p > 2000 else p for p in pulses]

        def safe_get(idx: int) -> float | None:
            return pulses[idx] if 0 <= idx < len(pulses) else None

        throttle_pw = safe_get(THROTTLE_IDX)
        steer_pw    = safe_get(STEER_IDX)
        intake_pw   = safe_get(INTAKE_IDX)
        outtake_pw  = safe_get(OUTTAKE_IDX)
        autonomous_pw = safe_get(AUTO_IDX)
        dock_pw = safe_get(DOCK_IDX)

        throttle = pulse_to_norm(throttle_pw) * THROTTLE_GAIN
        steer    = pulse_to_norm(steer_pw) * STEER_GAIN

        left_cmd, right_cmd = mix_steer_throttle(throttle, steer)

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(throttle)
        twist.linear.y = 0.0
        twist.angular.z = float(steer)
        self.cmd_pub.publish(twist)

        # Switches
        msg = Bool()
        msg.data = bool((intake_pw or PULSE_CENTER_US) > PULSE_CENTER_US)
        self.intake_pub.publish(msg)

        msg = Bool()
        msg.data = bool((outtake_pw or PULSE_CENTER_US) > PULSE_CENTER_US)
        self.outtake_pub.publish(msg)

        msg = Bool()
        msg.data = bool((autonomous_pw or PULSE_CENTER_US) > PULSE_CENTER_US)
        self.autonomous_active.publish(msg)

        msg = Bool()
        msg.data = bool((dock_pw or PULSE_CENTER_US) > PULSE_CENTER_US)
        self.dock_active.publish(msg)

        # Debug print ALL channels (14), 10 Hz is too spammy — do 2 Hz
        if not hasattr(self, "_last_dump"):
            self._last_dump = 0.0
        now = time.monotonic()
        if now - self._last_dump > 0.5:
            self._last_dump = now
            parts = [f"CH{i+1}={int(pulses[i])}" for i in range(min(14, len(pulses)))]
            self.get_logger().info(" ".join(parts))

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