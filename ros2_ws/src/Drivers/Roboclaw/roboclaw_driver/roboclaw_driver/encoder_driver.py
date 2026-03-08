#!/usr/bin/env python3
"""
ROS2 node: reads encoder counts and speeds from a RoboClaw and publishes them.

This is a standalone encoder-only node so you do not need to modify your
existing roboclaw_driver node.

WARNING:
- This node opens the RoboClaw serial port itself.
- Do NOT run this at the same time as another node that also opens the same
  RoboClaw port, unless you intentionally build a shared-access layer.

Published topics:
- encoder/left/count        (std_msgs/msg/Int64)
- encoder/right/count       (std_msgs/msg/Int64)
- encoder/left/speed_cps    (std_msgs/msg/Int32)
- encoder/right/speed_cps   (std_msgs/msg/Int32)
- wheel_states              (sensor_msgs/msg/JointState)

Optional services/features can be added later for encoder reset or odometry.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64, Int32
from sensor_msgs.msg import JointState

from .roboclaw_python.roboclaw_3 import Roboclaw


class RoboclawEncoderNode(Node):
	def __init__(self):
		super().__init__('roboclaw_encoder')

		# ---------------- Parameters ----------------
		self.declare_parameter('port', '/dev/ttyACM0')
		self.declare_parameter('baud', 115200)
		self.declare_parameter('address', 0x80)

		self.declare_parameter('poll_period_sec', 0.05)  # 20 Hz

		# Encoder conversion params
		self.declare_parameter('encoder_counts_per_motor_rev', 8192)
		self.declare_parameter('gear_ratio', 1.0)  # motor rev / wheel rev

		# Sign corrections for published values only
		self.declare_parameter('left_encoder_sign', 1)
		self.declare_parameter('right_encoder_sign', 1)

		# Joint names for JointState
		self.declare_parameter('left_joint_name', 'left_wheel_joint')
		self.declare_parameter('right_joint_name', 'right_wheel_joint')

		port = str(self.get_parameter('port').value)
		baud = int(self.get_parameter('baud').value)
		self.address = int(self.get_parameter('address').value)

		poll_period_sec = float(self.get_parameter('poll_period_sec').value)
		self.encoder_counts_per_motor_rev = float(
			self.get_parameter('encoder_counts_per_motor_rev').value
		)
		self.gear_ratio = float(self.get_parameter('gear_ratio').value)

		self.left_encoder_sign = int(self.get_parameter('left_encoder_sign').value)
		self.right_encoder_sign = int(self.get_parameter('right_encoder_sign').value)

		self.left_joint_name = str(self.get_parameter('left_joint_name').value)
		self.right_joint_name = str(self.get_parameter('right_joint_name').value)

		# ---------------- RoboClaw ----------------
		self.rc = Roboclaw(comport=port, rate=baud)
		opened = self.rc.Open()
		if not opened:
			self.get_logger().error(
				f'Failed to open RoboClaw on {port}: Open() returned {opened}'
			)
			raise RuntimeError(f'Failed to open RoboClaw on {port}')

		# ---------------- Publishers ----------------
		self.pub_left_count = self.create_publisher(Int64, 'encoder/left/count', 10)
		self.pub_right_count = self.create_publisher(Int64, 'encoder/right/count', 10)
		self.pub_left_speed = self.create_publisher(Int32, 'encoder/left/speed_cps', 10)
		self.pub_right_speed = self.create_publisher(Int32, 'encoder/right/speed_cps', 10)
		self.pub_joint_states = self.create_publisher(JointState, 'wheel_states', 10)

		# ---------------- Timer ----------------
		self.poll_timer = self.create_timer(poll_period_sec, self._poll_cb)

		self.get_logger().info(
			f'RoboclawEncoderNode ready — port={port} baud={baud} '
			f'address={hex(self.address)} poll_period_sec={poll_period_sec}'
		)

	def _extract_value(self, result) -> Optional[int]:
		"""
		Normalize possible RoboClaw library return formats.

		Common library return formats include:
		- (status, value, crc)
		- (status, value)
		- value
		"""
		if result is None:
			return None

		if isinstance(result, int):
			return result

		if isinstance(result, (tuple, list)):
			if len(result) >= 2 and isinstance(result[1], int):
				return int(result[1])
			if len(result) >= 1 and isinstance(result[0], int):
				return int(result[0])

		return None

	def _read_enc_m1(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadEncM1(self.address))
		except Exception as e:
			self.get_logger().error(f'Failed reading M1 encoder count: {e}')
			return None

	def _read_enc_m2(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadEncM2(self.address))
		except Exception as e:
			self.get_logger().error(f'Failed reading M2 encoder count: {e}')
			return None

	def _read_speed_m1(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadSpeedM1(self.address))
		except Exception as e:
			self.get_logger().error(f'Failed reading M1 encoder speed: {e}')
			return None

	def _read_speed_m2(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadSpeedM2(self.address))
		except Exception as e:
			self.get_logger().error(f'Failed reading M2 encoder speed: {e}')
			return None

	def _counts_to_wheel_rad(self, counts: int) -> float:
		# wheel angle = motor counts / counts-per-motor-rev / gear_ratio * 2pi
		return (2.0 * math.pi * float(counts)) / (
			self.encoder_counts_per_motor_rev * self.gear_ratio
		)

	def _cps_to_wheel_rad_s(self, counts_per_sec: int) -> float:
		return (2.0 * math.pi * float(counts_per_sec)) / (
			self.encoder_counts_per_motor_rev * self.gear_ratio
		)

	def _poll_cb(self):
		left_count = self._read_enc_m1()
		right_count = self._read_enc_m2()
		left_speed = self._read_speed_m1()
		right_speed = self._read_speed_m2()

		if left_count is None or right_count is None or left_speed is None or right_speed is None:
			return

		left_count *= self.left_encoder_sign
		right_count *= self.right_encoder_sign
		left_speed *= self.left_encoder_sign
		right_speed *= self.right_encoder_sign

		# Publish raw counts
		msg_left_count = Int64()
		msg_left_count.data = left_count
		self.pub_left_count.publish(msg_left_count)

		msg_right_count = Int64()
		msg_right_count.data = right_count
		self.pub_right_count.publish(msg_right_count)

		# Publish raw speeds in counts/sec
		msg_left_speed = Int32()
		msg_left_speed.data = left_speed
		self.pub_left_speed.publish(msg_left_speed)

		msg_right_speed = Int32()
		msg_right_speed.data = right_speed
		self.pub_right_speed.publish(msg_right_speed)

		# Publish JointState too
		js = JointState()
		js.header.stamp = self.get_clock().now().to_msg()
		js.name = [self.left_joint_name, self.right_joint_name]
		js.position = [
			self._counts_to_wheel_rad(left_count),
			self._counts_to_wheel_rad(right_count),
		]
		js.velocity = [
			self._cps_to_wheel_rad_s(left_speed),
			self._cps_to_wheel_rad_s(right_speed),
		]
		self.pub_joint_states.publish(js)


def main(args=None):
	rclpy.init(args=args)
	node = RoboclawEncoderNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.get_logger().info('roboclaw_encoder shutting down')
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()