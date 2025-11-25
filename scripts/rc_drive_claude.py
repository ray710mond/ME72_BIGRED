#!/usr/bin/env python3
import time
import gpiod
from roboclaw_3 import Roboclaw   # 👈 use BasicMicro's Python 3 library

# ----------------- CONFIG -----------------

# RoboClaw serial settings
ROBOCLAW_PORT = "/dev/ttyACM0"
ROBOCLAW_BAUD = 115200
ADDRESS       = 0x80      # Packet serial address set in Motion Studio

# RC input settings
CHIP_NAME = "gpiochip4"   # RP1 40-pin header on Pi 5

# CHANGE THESE to the lines you wired:
THROTTLE_LINE_OFFSET = 27  # e.g. GPIO27 (pin 13) = left stick up/down
STEER_LINE_OFFSET    = 17  # e.g. GPIO17 (pin 11) = right stick left/right

# Pulse range (tweak after you measure real values)
PULSE_MIN_US    = 1000.0
PULSE_CENTER_US = 1500.0
PULSE_MAX_US    = 2000.0
DEADBAND_US     = 40.0

LOOP_DT = 0.02            # 50 Hz control loop

# Timeout for stale signals (if no new pulse in 500ms, return None)
SIGNAL_TIMEOUT_S = 0.5


# ----------------- PWM READER -----------------

class PWMReader:
    def __init__(self, chip_name, line_offset, name="pwm"):
        self.chip = gpiod.Chip(chip_name)
        self.line = self.chip.get_line(line_offset)
        self.line.request(
            consumer=name,
            type=gpiod.LINE_REQ_EV_BOTH_EDGES
        )

        self.last_rise = None
        self.last_fall = None
        self.last_pw_us = None
        self.last_update_time = None
        self.name = name
        print(f"{name}: listening on {chip_name}, line {line_offset}")

    def read(self):
        """
        Non-blocking.
        Returns latest pulse width in microseconds, or None if none yet or signal is stale.
        """
        # Process all available events
        while self.line.event_wait(0):  # 0-sec timeout: just poll
            event = self.line.event_read()
            ts = event.sec + event.nsec / 1e9

            if event.type == gpiod.LineEvent.RISING_EDGE:
                self.last_rise = ts
                # Reset fall time when we see a new rising edge
                self.last_fall = None

            elif event.type == gpiod.LineEvent.FALLING_EDGE:
                self.last_fall = ts
                
                # Only calculate pulse width if we have a valid rising edge
                if self.last_rise is not None and self.last_fall > self.last_rise:
                    pw_us = (self.last_fall - self.last_rise) * 1e6
                    
                    # Sanity check: RC PWM should be between 500-2500μs
                    # Reject readings outside this range (likely noise or full period)
                    if 500 <= pw_us <= 2500:
                        self.last_pw_us = pw_us
                        self.last_update_time = time.time()
                    else:
                        # Debug: uncomment to see rejected values
                        # print(f"{self.name}: Rejected pw_us={pw_us:.0f} (out of range)")
                        pass

        # Check if signal is stale (no updates recently)
        if self.last_update_time is not None:
            if time.time() - self.last_update_time > SIGNAL_TIMEOUT_S:
                # Signal lost, return None instead of stale value
                return None

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


def mix_throttle_steer(throttle, steer):
    """
    Standard differential mix:
    - throttle: forward/back
    - steer: left/right (positive = turn right)
    Returns (left, right) commands in −1..+1
    """
    left  = throttle + steer
    right = throttle - steer

    # Clip to [-1, 1]
    left  = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))

    return left, right


# ----------------- ROBOCLAW OUTPUT -----------------

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


# ----------------- MAIN LOOP -----------------

def main():
    # RC inputs
    throttle_reader = PWMReader(CHIP_NAME, THROTTLE_LINE_OFFSET, name="throttle")
    steer_reader    = PWMReader(CHIP_NAME, STEER_LINE_OFFSET,   name="steer")

    # RoboClaw (BasicMicro library: port + baud, then Open())
    rc = Roboclaw(ROBOCLAW_PORT, ROBOCLAW_BAUD)
    rc.Open()
    print("RoboClaw opened on", ROBOCLAW_PORT)

    print("Starting dual-stick RC → RoboClaw control loop...")
    print("Move your RC sticks to verify signal detection...")
    time.sleep(1.0)

    try:
        while True:
            # 1) Read RC pulses (μs)
            throttle_pw = throttle_reader.read()
            steer_pw    = steer_reader.read()

            # 2) Convert to −1..+1
            throttle = pulse_to_norm(throttle_pw)
            steer    = pulse_to_norm(steer_pw)

            # Optional: invert axes if they feel backwards
            # throttle = -throttle
            # steer    = -steer

            # 3) Mix into left/right motor commands
            left_cmd, right_cmd = mix_throttle_steer(throttle, steer)

            # 4) Send to RoboClaw
            send_motor_norm_duty(rc, ADDRESS, left_cmd, right_cmd)

            # Debug print
            print(
                f"thr_pw={throttle_pw if throttle_pw else 'None':>6}, "
                f"steer_pw={steer_pw if steer_pw else 'None':>6}, "
                f"thr={throttle:+.2f}, str={steer:+.2f}, "
                f"L={left_cmd:+.2f}, R={right_cmd:+.2f}"
            )

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\nStopping motors and cleaning up...")
        send_motor_norm_duty(rc, ADDRESS, 0.0, 0.0)
        throttle_reader.close()
        steer_reader.close()


if __name__ == "__main__":
    main()
