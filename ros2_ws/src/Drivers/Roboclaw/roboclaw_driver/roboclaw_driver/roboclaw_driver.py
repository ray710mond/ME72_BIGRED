#!/usr/bin/env python3
"""ROS2 node: subscribes to `cmd_vel_des` (geometry_msgs/Twist) and sends
left/right motor duties to a RoboClaw controller using the BasicMicro API.

This node uses the `libraries.roboclaw_python.roboclaw.Roboclaw` class.

Added safety behavior:
- If no new `cmd_vel_des` message is received within `cmd_timeout_sec`,
  the motors are commanded to stop.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .roboclaw_python.roboclaw_3 import Roboclaw


class RoboclawDriver(Node):
    def __init__(self):
        super().__init__('roboclaw_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('address', 0x80)
        self.declare_parameter('cmd_timeout_sec', 0.25)   # stop if no fresh cmd for 250 ms
        self.declare_parameter('watchdog_period_sec', 0.05)  # check at 20 Hz

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.address = int(self.get_parameter('address').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        watchdog_period_sec = float(self.get_parameter('watchdog_period_sec').value)

        # Create Roboclaw instance and open
        self.rc = Roboclaw(comport=port, rate=baud)
        opened = self.rc.Open()
        if not opened:
            self.get_logger().error(
                f'Failed to open Roboclaw on {port}: Open() returned {opened}'
            )
            raise RuntimeError(f'Failed to open Roboclaw on {port}')

        # Track when the last command was received
        self.last_cmd_time = self.get_clock().now()

        # Track whether we are already stopped so we don't spam stop commands
        self.timed_out = True

        # Subscriber to Twist
        self.sub = self.create_subscription(Twist, 'cmd_vel_des', self._cmd_cb, 10)

        # Watchdog timer
        self.watchdog_timer = self.create_timer(watchdog_period_sec, self._watchdog_cb)

        self.get_logger().info(
            f'RoboclawDriver ready — port={port} baud={baud} '
            f'address={hex(self.address)} cmd_timeout_sec={self.cmd_timeout_sec}'
        )

    def _send_duties(self, left_duty: int, right_duty: int):
        try:
            self.rc.DutyM1(self.address, left_duty)
            self.rc.DutyM2(self.address, right_duty)
        except Exception as e:
            self.get_logger().error(f'Failed to send duties to RoboClaw: {e}')

    def _stop_motors(self):
        self._send_duties(0, 0)

    def _cmd_cb(self, msg: Twist):
        # Mark this command as fresh
        self.last_cmd_time = self.get_clock().now()
        self.timed_out = False

        # Expect msg.linear.x (forward) and msg.angular.z (rotation)
        throttle = float(msg.linear.x)
        steer = float(msg.angular.z)

        # Mix into left/right
        left = throttle + steer
        right = -(throttle - steer)  # FLIP RIGHT MOTOR DIRECTION

        # Clip to [-1, 1]
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Map -1..1 to duty range
        max_duty = 32767
        left_duty = int(left * max_duty)
        right_duty = int(right * max_duty)

        self._send_duties(left_duty, right_duty)

    def _watchdog_cb(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9

        if age > self.cmd_timeout_sec:
            if not self.timed_out:
                self.get_logger().warn(
                    f'cmd_vel_des timeout ({age:.3f}s > {self.cmd_timeout_sec:.3f}s). Stopping motors.'
                )
                self._stop_motors()
                self.timed_out = True

    def destroy_node(self):
        try:
            try:
                self._stop_motors()
            except Exception:
                pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_motors()
    finally:
        node.get_logger().info('roboclaw_driver shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()