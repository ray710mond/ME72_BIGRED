#!/usr/bin/env python3
"""
rf_driver_ibus.py

ROS2 node that reads FlySky iBUS from a UART (/dev/ttyAMA0, /dev/ttyS0, etc.)
and publishes:
	- Twist on `cmd_vel_teleop`
	- Bool on `intake_running`
	- Bool on `outtake_open`

Behavior:
	- Does NOT publish on `cmd_vel_teleop` until the first valid iBUS frame is received.
	- If iBUS goes stale, it does NOT publish on `cmd_vel_teleop`.
	- Auxiliary bool topics are also not updated from stale data.

Key robustness improvements:
	- Serial opened with flow control disabled explicitly.
	- Reader thread parses ALL frames available in the buffer (keeps freshest data).
	- Loud exception logging in reader thread (no silent failures).
"""

import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# ----------------- USER SETTINGS -----------------

IBUS_PORT = "/dev/ttyAMA0"
IBUS_BAUD = 115200

# Channel mapping (1-based like RC conventions)
THROTTLE_CHNL = 2
STEER_CHNL = 4
INTAKE_CHNL = 5
OUTTAKE_CHNL = 6
AUTO_CHNL = 7
DOCK_CHNL = 8

THROTTLE_IDX = THROTTLE_CHNL - 1
STEER_IDX = STEER_CHNL - 1
INTAKE_IDX = INTAKE_CHNL - 1
OUTTAKE_IDX = OUTTAKE_CHNL - 1
AUTO_IDX = AUTO_CHNL - 1
DOCK_IDX = DOCK_CHNL - 1

# iBUS typical value range is ~1000..2000, center ~1500
PULSE_MIN_US = 1000.0
PULSE_CENTER_US = 1500.0
PULSE_MAX_US = 2000.0
DEADBAND_US = 40.0

# Control loop
LOOP_DT = 0.02  # 50 Hz

# Gains
THROTTLE_GAIN = 1.0
STEER_GAIN = 1.0

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
	"""Map RC pulse to -1..+1 with deadband."""
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
			rtscts=False,
			dsrdtr=False,
			xonxoff=False,
		)

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
		ch = []
		for i in range(IBusReader.N_CHANNELS):
			lo = frame[2 + 2 * i]
			hi = frame[2 + 2 * i + 1]
			ch.append(lo | (hi << 8))
		return ch

	def _find_frame(self, buf: bytearray) -> bytes | None:
		for start in range(len(buf)):
			if buf[start] != self.FRAME_START_LEN:
				continue
			if start + self.FRAME_LEN > len(buf):
				return None

			candidate = bytes(buf[start:start + self.FRAME_LEN])

			if candidate[1] != self.FRAME_CMD:
				continue
			if not self._checksum_ok(candidate):
				continue

			del buf[:start + self.FRAME_LEN]
			return candidate

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
		self._last_stale_warn = 0.0

		self._seen_first_frame = False
		self._waiting_logged = False

		self._timer = self.create_timer(LOOP_DT, self._tick)
		self.get_logger().info(f"Started iBUS on {IBUS_PORT} @ {IBUS_BAUD}")

	def _publish_aux(self, intake: bool, outtake: bool, autonomous: bool, dock: bool):
		msg = Bool()
		msg.data = intake
		self.intake_pub.publish(msg)

		msg = Bool()
		msg.data = outtake
		self.outtake_pub.publish(msg)

		msg = Bool()
		msg.data = autonomous
		self.autonomous_active.publish(msg)

		msg = Bool()
		msg.data = dock
		self.dock_active.publish(msg)

	def _tick(self):
		chans = self.ibus.read_channels()
		age = self.ibus.age_s()

		if chans is not None:
			self._seen_first_frame = True
			self._waiting_logged = False

		# Do not publish anything until the first valid frame arrives
		if not self._seen_first_frame:
			if not self._waiting_logged:
				self.get_logger().info("Waiting for first valid iBUS frame...")
				self._waiting_logged = True
			return

		# If frames are stale, do NOT publish cmd_vel_teleop
		if age is None or age > STALE_S:
			now = time.monotonic()
			if now - self._last_stale_warn > 1.0:
				if age is None:
					self.get_logger().warn("iBUS missing after initialization -> not publishing cmd_vel_teleop")
				else:
					self.get_logger().warn(f"iBUS stale ({age:.2f}s) -> not publishing cmd_vel_teleop")
				self._last_stale_warn = now
			return

		# Extra safety
		if chans is None:
			return

		pulses = [float(v) for v in chans]
		pulses = [1500.0 if p > 2000 else p for p in pulses]

		def safe_get(idx: int) -> float | None:
			return pulses[idx] if 0 <= idx < len(pulses) else None

		throttle_pw = safe_get(THROTTLE_IDX)
		steer_pw = safe_get(STEER_IDX)
		intake_pw = safe_get(INTAKE_IDX)
		outtake_pw = safe_get(OUTTAKE_IDX)
		autonomous_pw = safe_get(AUTO_IDX)
		dock_pw = safe_get(DOCK_IDX)

		throttle = pulse_to_norm(throttle_pw) * THROTTLE_GAIN
		steer = pulse_to_norm(steer_pw) * STEER_GAIN

		twist = Twist()
		twist.linear.x = float(throttle)
		twist.linear.y = 0.0
		twist.angular.z = float(steer)
		self.cmd_pub.publish(twist)

		self._publish_aux(
			intake=bool((intake_pw or PULSE_CENTER_US) > PULSE_CENTER_US),
			outtake=bool((outtake_pw or PULSE_CENTER_US) > PULSE_CENTER_US),
			autonomous=bool((autonomous_pw or PULSE_CENTER_US) > PULSE_CENTER_US),
			dock=bool((dock_pw or PULSE_CENTER_US) > PULSE_CENTER_US),
		)

		now = time.monotonic()
		if now - self._last_dump > 0.5:
			self._last_dump = now
			parts = [f"CH{i+1}={int(pulses[i])}" for i in range(min(10, len(pulses)))]
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