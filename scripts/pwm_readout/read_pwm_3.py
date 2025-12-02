#!/usr/bin/env python3
import gpiod
import time

# Use the RP1 pin controller – this is the 40-pin header
CHIP_NAME = "gpiochip4"
LINE_OFFSET = 22        

chip = gpiod.Chip(CHIP_NAME)

line = chip.get_line(LINE_OFFSET)

line.request(consumer="pwm-reader",
             type=gpiod.LINE_REQ_EV_BOTH_EDGES)

print(f"Reading PWM on {CHIP_NAME}, line {LINE_OFFSET} (GPIO17)...")
print("Move the stick; Ctrl+C to stop.\n")

last_rise = None

try:
    while True:
        if not line.event_wait(1):
            continue

        event = line.event_read()
        ts = event.sec + event.nsec / 1e9

        if event.type == gpiod.LineEvent.RISING_EDGE:
            last_rise = ts
        elif event.type == gpiod.LineEvent.FALLING_EDGE and last_rise is not None:
            pw_us = (ts - last_rise) * 1e6
            print(f"Pulse width: {pw_us:.1f} us")

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    line.release()
    chip.close()

