#!/usr/bin/env python3
import gpiod
import time
from roboclaw_3 import Roboclaw   # BasicMicro's Python 3 library

# ----------------- CONFIG -----------------

# RoboClaw serial settings
ROBOCLAW_PORT = "/dev/ttyACM0"
ROBOCLAW_BAUD = 115200
ADDRESS       = 0x80      # Packet serial address set in Motion Studio

# RC input settings
CHIP_NAME = "gpiochip4"

# Fixed channel order: CH1..CH6
CHANNEL_PINS = [27, 17, 22, 25, 23, 24]

# Use CH1 and CH2 for robot motion
THROTTLE_CH_INDEX = 0   # CH1 -> throttle (GPIO27)
STEER_CH_INDEX    = 1   # CH2 -> steer    (GPIO17)

# Pulse range (adjust after measuring)
PULSE_MIN_US    = 1000.0
PULSE_CENTER_US = 1500.0
PULSE_MAX_US    = 2000.0
DEADBAND_US     = 40.0

LOOP_DT = 0.02  # 50 Hz


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

	if pulse_us < min_us:
		pulse_us = min_us
	elif pulse_us > max_us:
		pulse_us = max_us

	offset = pulse_us - center

	# Deadband around center
	if abs(offset) < deadband_us:
		return 0.0

	if offset > 0:
		return min(1.0, offset / (max_us - center))
	else:
		return max(-1.0, offset / (center - min_us))


def mix_throttle_steer(throttle, steer):
	"""
	Differential drive mix.
	"""
	left  = throttle + steer
	right = throttle - steer

	left  = max(-1.0, min(1.0, left))
	right = max(-1.0, min(1.0, right))
	return left, right


def send_motor_norm_duty(rc, address, left_norm, right_norm):
	"""
	Map −1..+1 to signed duty commands (BasicMicro API).
	"""
	left_norm  = max(-1.0, min(1.0, left_norm))
	right_norm = max(-1.0, min(1.0, right_norm))

	MAX_DUTY = 32767

	left_duty  = int(left_norm * MAX_DUTY)
	right_duty = int(right_norm * MAX_DUTY)

	rc.DutyM1(address, left_duty)
	rc.DutyM2(address, right_duty)


def _fmt_pw(pw):
	if pw is None:
		return "  ---- "
	return f"{pw:6.1f}"


# ----------------- MAIN -----------------

def main():
	# Set up GPIO lines
	chip = gpiod.Chip(CHIP_NAME)
	lines = [chip.get_line(pin) for pin in CHANNEL_PINS]

	for line in lines:
		line.request(
			consumer="pwm-reader",
			type=gpiod.LINE_REQ_EV_BOTH_EDGES
		)

	print("Reading PWM on pins (CH1..CH6):", CHANNEL_PINS)

	# Time of last rising edge and last measured pulse per channel
	last_rise  = [None] * len(CHANNEL_PINS)
	last_value = [None] * len(CHANNEL_PINS)

	# RoboClaw
	rc = Roboclaw(ROBOCLAW_PORT, ROBOCLAW_BAUD)
	rc.Open()
	print("RoboClaw opened on", ROBOCLAW_PORT)
	print("Starting 6-channel RC → RoboClaw control loop...\n")

	try:
		while True:
			# New pulse widths measured this iteration
			pw_list = [None] * len(CHANNEL_PINS)

			# Check all lines for pending events, drain them
			for i, line in enumerate(lines):
				while line.event_wait(0):
					event = line.event_read()
					ts = event.sec + event.nsec / 1e9

					if event.type == gpiod.LineEvent.RISING_EDGE:
						last_rise[i] = ts
					elif event.type == gpiod.LineEvent.FALLING_EDGE:
						if last_rise[i] is not None:
							pw_list[i] = (ts - last_rise[i]) * 1e6  # μs

			# Update stored values with any new pulse widths
			for i, v in enumerate(pw_list):
				if v is not None:
					last_value[i] = v

			# Convert all channels to −1..+1
			norms = [pulse_to_norm(v) for v in last_value]

			# Use CH1, CH2 for motion
			throttle_pw = last_value[THROTTLE_CH_INDEX]
			steer_pw    = last_value[STEER_CH_INDEX]

			throttle = norms[THROTTLE_CH_INDEX]
			steer    = norms[STEER_CH_INDEX]

			# Optional: invert if needed
			# throttle = -throttle
			# steer    = -steer

			left_cmd, right_cmd = mix_throttle_steer(throttle, steer)
			send_motor_norm_duty(rc, ADDRESS, left_cmd, right_cmd)

			# Print debug for all 6 channels
			ch_parts = []
			for i in range(len(CHANNEL_PINS)):
				ch_parts.append(
					f"CH{i+1}={_fmt_pw(last_value[i])}us({norms[i]:+5.2f})"
				)

			print("  ".join(ch_parts) + f"   L={left_cmd:+.2f} R={right_cmd:+.2f}")

			time.sleep(LOOP_DT)

	except KeyboardInterrupt:
		print("\nStopping motors and cleaning up...")
		send_motor_norm_duty(rc, ADDRESS, 0.0, 0.0)

	finally:
		for line in lines:
			line.release()
		chip.close()


if __name__ == "__main__":
	main()
