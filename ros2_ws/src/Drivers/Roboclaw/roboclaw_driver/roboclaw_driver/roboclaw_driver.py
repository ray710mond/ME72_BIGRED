#!/usr/bin/env python3
"""ROS2 node: subscribes to `cmd_vel_des` (geometry_msgs/Twist) and sends
left/right motor duties to a RoboClaw controller using the BasicMicro API.

This node uses the `libraries.roboclaw_python.roboclaw.Roboclaw` class.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool

# from roboclaw_3 import Roboclaw
from .roboclaw_python.roboclaw_3 import Roboclaw



class RoboclawDriver(Node):
    def __init__(self):
        super().__init__('roboclaw_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('address', 0x80)

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.address = int(self.get_parameter('address').value)

        # Create Roboclaw instance and open
        self.rc = Roboclaw(self, comport=port, rate=baud)
        try:
            self.rc.Open()
        except Exception as e:
            self.get_logger().error(f'Failed to open Roboclaw on {port}: {e}')
            raise

        # Subscriber to Twist
        self.sub = self.create_subscription(Twist, 'cmd_vel_des', self._cmd_cb, 10)

        self.get_logger().info(f'RoboclawDriver ready — port={port} baud={baud} address={hex(self.address)}')

    def _cmd_cb(self, msg: Twist):
        # Expect msg.linear.x (forward) and msg.angular.z (rotation)
        throttle = float(msg.linear.x)
        steer = float(msg.angular.z)

        # Mix into left/right (same mixing as RF driver)
        left = throttle + steer
        right = throttle - steer

        # Clip
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Map -1..1 to duty range
        MAX_DUTY = 32767
        left_duty = int(left * MAX_DUTY)
        right_duty = int(right * MAX_DUTY)

        try:
            self.rc.DutyM1(self.address, left_duty)
            self.rc.DutyM2(self.address, right_duty)
        except Exception as e:
            self.get_logger().error(f'Failed to send duties to RoboClaw: {e}')

    def destroy_node(self):
        try:
            # stop motors
            try:
                self.rc.DutyM1(self.address, 0)
                self.rc.DutyM2(self.address, 0)
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
        node.rc.DutyM1(node.address, 0)
        node.rc.DutyM2(node.address, 0)
    finally:
        node.get_logger().info('roboclaw_driver shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
