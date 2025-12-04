#!/usr/bin/env python3
import gpiod
import time

# ----------------- CONFIG -----------------

# RC input settings
CHIP_NAME = "gpiochip4"

# Fixed channel order: CH1..CH6
CHANNEL_PINS = [27, 17, 22, 25, 23, 24]

LOOP_DT = 0.02  # 50 Hz


def _fmt_pw(pw):
	if pw is None:
		return "  ---- "
	return f"{pw:6.1f}"  # fixed width, 1 decimal place


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
	print("Ctrl+C to stop.\n")

	# Time of last rising edge and last measured pulse per channel
	last_rise  = [None] * len(CHANNEL_PINS)
	last_value = [None] * len(CHANNEL_PINS)

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

			# Print raw PWM values for all 6 channels
			ch_parts = []
			for i in range(len(CHANNEL_PINS)):
				ch_parts.append(f"CH{i+1}={_fmt_pw(last_value[i])}us")

			print("  ".join(ch_parts))

			time.sleep(LOOP_DT)

	except KeyboardInterrupt:
		print("\nStopping and cleaning up...")

	finally:
		for line in lines:
			line.release()
		chip.close()


if __name__ == "__main__":
	main()
