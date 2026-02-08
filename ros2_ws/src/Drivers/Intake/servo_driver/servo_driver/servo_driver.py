#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import gpiod

# ---------------- GPIO CONFIG ----------------
CHIP_NAME = "gpiochip4"   # Pi 5 header chip
GPIO_LINE = 13            # BCM 13 (PWM1-capable pin)

# ---------------- SERVO CONFIG ----------------
PWM_HZ = 50.0
PWM_PERIOD_S = 1.0 / PWM_HZ

# Tune these if needed
PULSE_CLOSED_S = 0.0010   # 1.0 ms
PULSE_OPEN_S   = 0.0020   # 2.0 ms

# How long to pulse the servo to move (seconds)
MOVE_TIME_S = 0.4

# Ignore duplicate commands close together
DEBOUNCE_S = 0.15


class OuttakeServoNode(Node):
    def __init__(self):
        super().__init__("outtake_servo_node")

        self.sub = self.create_subscription(
            Bool,
            "outtake_open",
            self.on_cmd,
            10
        )

        self._last_cmd = None
        self._last_cmd_time = 0.0
        self._busy = False

        # GPIO setup
        self.chip = gpiod.Chip(CHIP_NAME)
        self.line = self.chip.get_line(GPIO_LINE)
        self.line.request(
            consumer="outtake_servo",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0],
        )

        self.get_logger().info(
            "Outtake servo ready (gpiochip4, GPIO13) — pulse-to-position, self-holding"
        )

    # ---------------- ROS CALLBACK ----------------

    def on_cmd(self, msg: Bool):
        now = time.time()

        if self._busy:
            return

        if self._last_cmd is not None and msg.data == self._last_cmd:
            if (now - self._last_cmd_time) < DEBOUNCE_S:
                return

        self._last_cmd = msg.data
        self._last_cmd_time = now

        pulse_width = PULSE_OPEN_S if msg.data else PULSE_CLOSED_S
        label = "OPEN" if msg.data else "CLOSED"

        self.get_logger().info(f"Outtake servo -> {label}")
        threading.Thread(
            target=self.pulse_servo,
            args=(pulse_width,),
            daemon=True
        ).start()

    # ---------------- SERVO PULSE ----------------

    def pulse_servo(self, pulse_width_s: float):
        self._busy = True
        end_time = time.time() + MOVE_TIME_S

        while time.time() < end_time:
            # HIGH pulse
            self.line.set_value(1)
            time.sleep(pulse_width_s)

            # LOW for remainder of 20 ms period
            self.line.set_value(0)
            time.sleep(PWM_PERIOD_S - pulse_width_s)

        # Ensure line is low forever after
        self.line.set_value(0)
        self._busy = False

    # ---------------- CLEANUP ----------------

    def destroy_node(self):
        try:
            self.line.set_value(0)
            self.line.release()
            self.chip.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OuttakeServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
