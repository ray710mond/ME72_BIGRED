import time
import gpiod

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# RC input settings
CHIP_NAME = "gpiochip4"   # RP1 40-pin header on Pi 5

# Channels 1–6 wired to these GPIOs (in order):
CHANNEL_PINS = [25, 17, 27, 22, 23, 24]

THROTTLE_CHNL = 2
STEER_CHNL = 4
INTAKE_CHNL = 5


STEER_IDX    = STEER_CHNL - 1 
THROTTLE_IDX = THROTTLE_CHNL - 1
INTAKE_IDX = INTAKE_CHNL - 1

# Pulse range (tweak after you measure real values)
PULSE_MIN_US    = 1000.0
PULSE_CENTER_US = 1500.0
PULSE_MAX_US    = 2000.0
DEADBAND_US     = 40.0

LOOP_DT = 0.02            # 50 Hz control loop


# ----------------- PWM READER -----------------

class RF_Driver:
	def __init__(self, chip_name, line_offset, name="pwm"):
		self.chip = gpiod.Chip(chip_name)
		self.line = self.chip.get_line(line_offset)
		self.line.request(
			consumer=name,
			type=gpiod.LINE_REQ_EV_BOTH_EDGES
		)

		# Detect idle level once: if it's high, pulses are likely active-low.
		try:
			idle_level = self.line.get_value()
		except Exception:
			# Fallback: assume active-high if we can't read the value
			idle_level = 0

		# If line is idle high, pulses are low (active-low).
		self.active_low = bool(idle_level)

		self.last_edge_ts = None     # timestamp of start of the *pulse* (low or high)
		self.last_pw_us   = None
		self.name = name

		mode_str = "active-low" if self.active_low else "active-high"
		print(f"{name}: listening on {chip_name}, line {line_offset} ({mode_str})")

	def _process_event(self, event):
		ts = event.sec + event.nsec / 1e9

		if not self.active_low:
			# ----- ACTIVE-HIGH: measure high time (rising -> falling) -----
			if event.type == gpiod.LineEvent.RISING_EDGE:
				self.last_edge_ts = ts
			elif event.type == gpiod.LineEvent.FALLING_EDGE and self.last_edge_ts is not None:
				self.last_pw_us = (ts - self.last_edge_ts) * 1e6
		else:
			# ----- ACTIVE-LOW: measure low time (falling -> rising) -----
			if event.type == gpiod.LineEvent.FALLING_EDGE:
				self.last_edge_ts = ts
			elif event.type == gpiod.LineEvent.RISING_EDGE and self.last_edge_ts is not None:
				self.last_pw_us = (ts - self.last_edge_ts) * 1e6

	def read(self):
		"""
		Non-blocking.
		Returns latest pulse width in microseconds, or None if none yet.
		"""
		# If there are no events pending, just return the last measured pulse.
		if not self.line.event_wait(0):
			return self.last_pw_us

		# Drain all pending events so we end up with the newest pulse.
		while True:
			event = self.line.event_read()
			self._process_event(event)

			# Check if more events are queued; don't block.
			if not self.line.event_wait(0):
				break

		return self.last_pw_us

	def close(self):
		self.line.release()
		self.chip.close()


# ----------------- MATH HELPERS -----------------

def pulse_to_norm(
	pulse_us,
	center=PULSE_CENTER_US,
	min_us=PULSE_MIN_US,
	max_us=PULSE_MAX_US,
	deadband_us=DEADBAND_US
):
	"""
	Map RC pulse (μs) to −1..+1 with deadband.
	"""
	if pulse_us is None:
		return 0.0

	# Clamp absurd values so bad pulses don't blow up the math.
	if pulse_us < min_us:
		pulse_us = min_us
	elif pulse_us > max_us:
		pulse_us = max_us

	offset = pulse_us - center

	# Deadband around center to avoid creeping
	if abs(offset) < deadband_us:
		return 0.0

	if offset > 0:
		# Center..max -> 0..+1
		return min(1.0, offset / (max_us - center))
	else:
		# Min..center -> -1..0
		return max(-1.0, offset / (center - min_us))


def mix_steer_throttle(steer, throttle):
	"""
	Standard differential mix:
	- steer: forward/back
	- throttle: left/right (positive = turn right)
	Returns (left, right) commands in −1..+1
	"""
	left  = steer + throttle
	right = steer - throttle

	# Clip to [-1, 1]
	left  = max(-1.0, min(1.0, left))
	right = max(-1.0, min(1.0, right))

	return left, right


# ----------------- ROBOCLAW OUTPUT -----------------

def send_motor_norm_duty(rc, address, left_norm, right_norm):
	"""
	Map −1..+1 to signed duty commands (BasicMicro API).
	"""
	# RoboClaw-specific sending logic has been moved to the roboclaw_driver node.
	# This helper is intentionally left as a no-op to avoid accidental hardware calls.
	return


def _fmt_pw(pw):
	"""Nice fixed-width formatting for pulses (μs)."""
	if pw is None:
		return "  ---- "
	return f"{pw:6.1f}"


# ----------------- MAIN LOOP -----------------

# RC inputs: create one reader per channel
channel_names = [f"ch{i+1}" for i in range(len(CHANNEL_PINS))]
readers = [
	RF_Driver(CHIP_NAME, pin, name=name)
	for pin, name in zip(CHANNEL_PINS, channel_names)
]

print("Starting 6-channel RC → cmd_vel_des publisher loop...")
time.sleep(1.0)

try:
	# initialize ROS2 node + publishers
	rclpy.init()
	node = Node('rf_driver')
	intake_pub = node.create_publisher(Bool, 'intake_running', 10)
	cmd_pub = node.create_publisher(Twist, 'cmd_vel_des', 10)

	while True:
		# 1) Read all RC pulses (μs)
		pulses = [r.read() for r in readers]

		# 2) Convert all to −1..+1
		norms = [pulse_to_norm(pw) for pw in pulses]

		# Use CH1 and CH2 for steer/throttle
		steer_pw = pulses[STEER_IDX]
		throttle_pw = pulses[THROTTLE_IDX]

		THROTTLE_GAIN = 1.0
		STEER_GAIN = 0.25

		steer = norms[STEER_IDX] * STEER_GAIN
		throttle = norms[THROTTLE_IDX] * THROTTLE_GAIN

		# Optional: invert axes if they feel backwards
		# steer = -steer
		# throttle = -throttle

		# 3) Mix into left/right motor commands (kept for debug print)
		left_cmd, right_cmd = mix_steer_throttle(steer, throttle)

		# 4) Publish desired velocity as Twist on `cmd_vel_des`
		twist = Twist()
		# Map existing signals: `steer` -> forward/back (linear.x),
		# `throttle` -> rotation (angular.z). Leave linear.y zero.
		twist.linear.x = float(steer)
		twist.linear.y = 0.0
		twist.angular.z = float(throttle)
		cmd_pub.publish(twist)

		# Publish intake_running based on channel 5 (switch toggles ~1000/2000 us)
		intake_pw = pulses[INTAKE_IDX]
		if intake_pw is not None:
			intake_on = intake_pw > PULSE_CENTER_US
			msg = Bool()
			msg.data = bool(intake_on)
			intake_pub.publish(msg)

		# Debug print for all 6 channels
		ch_parts = []
		for i in range(6):
			ch_parts.append(
				f"CH{i+1}={_fmt_pw(pulses[i])}us({norms[i]:+5.2f})"
			)
		ch_str = "  ".join(ch_parts)

		print(f"{ch_str}   L={left_cmd:+.2f} R={right_cmd:+.2f}")

		time.sleep(LOOP_DT)

except KeyboardInterrupt:
	print("\nStopping and cleaning up...")
	for r in readers:
		r.close()
	try:
		node.destroy_node()
		rclpy.shutdown()
	except Exception:
		pass