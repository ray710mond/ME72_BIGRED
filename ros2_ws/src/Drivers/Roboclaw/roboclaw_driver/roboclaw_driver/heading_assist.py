#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def clamp(x: float, lo: float, hi: float) -> float:
	return max(lo, min(hi, x))


def wrap_to_pi(angle: float) -> float:
	while angle > math.pi:
		angle -= 2.0 * math.pi
	while angle < -math.pi:
		angle += 2.0 * math.pi
	return angle


class ImuHeadingAssist(Node):

	def __init__(self):
		super().__init__('imu_heading_assist')

		self.declare_parameter('control_rate_hz', 50.0)
		self.declare_parameter('cmd_timeout_sec', 0.25)

		self.declare_parameter('imu_gyro_z_sign', 1.0)

		self.declare_parameter('straight_linear_threshold', 0.08)
		self.declare_parameter('straight_angular_deadband', 0.01)

		self.declare_parameter('kp_heading', 1.0)
		self.declare_parameter('ki_heading', 0.05)

		self.declare_parameter('kp_yaw_rate', 2.0)
		self.declare_parameter('ki_yaw_rate', 0.10)

		self.declare_parameter('max_heading_correction', 0.35)
		self.declare_parameter('max_yaw_rate_correction', 1.0)

		self.declare_parameter('max_linear_cmd', 1.0)
		self.declare_parameter('max_angular_cmd', 1.0)

		self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
		self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)

		self.imu_gyro_z_sign = float(self.get_parameter('imu_gyro_z_sign').value)

		self.straight_linear_threshold = float(self.get_parameter('straight_linear_threshold').value)
		self.straight_angular_deadband = float(self.get_parameter('straight_angular_deadband').value)

		self.kp_heading = float(self.get_parameter('kp_heading').value)
		self.ki_heading = float(self.get_parameter('ki_heading').value)

		self.kp_yaw_rate = float(self.get_parameter('kp_yaw_rate').value)
		self.ki_yaw_rate = float(self.get_parameter('ki_yaw_rate').value)

		self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)
		self.max_yaw_rate_correction = float(self.get_parameter('max_yaw_rate_correction').value)

		self.max_linear_cmd = float(self.get_parameter('max_linear_cmd').value)
		self.max_angular_cmd = float(self.get_parameter('max_angular_cmd').value)

		self.des_v = 0.0
		self.des_w = 0.0

		self.yaw = 0.0
		self.gyro_z = 0.0

		self.heading_hold_active = False
		self.heading_target = 0.0

		self.int_heading = 0.0
		self.int_yaw_rate = 0.0

		self.last_cmd_time = self.get_clock().now()
		self.last_update_time = self.get_clock().now()

		self.have_imu = False
		self.last_imu_time = None

		self.sub_cmd = self.create_subscription(
			Twist,
			'cmd_vel_auto',
			self._cmd_cb,
			20
		)

		self.sub_imu = self.create_subscription(
			Imu,
			'imu/data',
			self._imu_cb,
			50
		)

		self.pub_cmd = self.create_publisher(
			Twist,
			'cmd_vel_auto_filtered',
			20
		)

		self.timer = self.create_timer(
			1.0 / self.control_rate_hz,
			self._update_cb
		)

		self.get_logger().info('imu_heading_assist ready: cmd_vel_auto -> cmd_vel_auto_filtered')

	def _cmd_cb(self, msg: Twist):
		self.des_v = float(msg.linear.x)
		self.des_w = float(msg.angular.z)
		self.last_cmd_time = self.get_clock().now()

	def _imu_cb(self, msg: Imu):
		now = self.get_clock().now()

		if self.last_imu_time is not None:
			dt = (now - self.last_imu_time).nanoseconds / 1e9
			if 0.0 < dt < 0.2:
				self.yaw = wrap_to_pi(self.yaw + self.gyro_z * dt)

		self.gyro_z = float(msg.angular_velocity.z) * self.imu_gyro_z_sign
		self.last_imu_time = now
		self.have_imu = True

	def _publish_cmd(self, v: float, w: float):
		msg = Twist()
		msg.linear.x = float(v)
		msg.angular.z = float(w)
		self.pub_cmd.publish(msg)

	def _update_cb(self):
		now = self.get_clock().now()

		dt = (now - self.last_update_time).nanoseconds / 1e9
		if dt <= 0.0 or dt > 0.2:
			dt = 1.0 / self.control_rate_hz
		self.last_update_time = now

		cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9

		if cmd_age > self.cmd_timeout_sec:
			self.heading_hold_active = False
			self.int_heading = 0.0
			self.int_yaw_rate = 0.0
			self._publish_cmd(0.0, 0.0)
			return

		v_cmd = clamp(self.des_v, -self.max_linear_cmd, self.max_linear_cmd)
		w_cmd = clamp(self.des_w, -self.max_angular_cmd, self.max_angular_cmd)

		if not self.have_imu:
			self._publish_cmd(v_cmd, w_cmd)
			return

		is_straight = (
			abs(v_cmd) >= self.straight_linear_threshold and
			abs(w_cmd) <= self.straight_angular_deadband
		)

		if is_straight:
			if not self.heading_hold_active:
				self.heading_target = self.yaw
				self.heading_hold_active = True
				self.int_heading = 0.0

			err = wrap_to_pi(self.heading_target - self.yaw)

			self.int_heading += err * dt
			max_int = self.max_heading_correction / max(self.ki_heading, 1e-6)
			self.int_heading = clamp(self.int_heading, -max_int, max_int)

			correction = self.kp_heading * err + self.ki_heading * self.int_heading
			correction = clamp(
				correction,
				-self.max_heading_correction,
				self.max_heading_correction
			)

			w_out = correction
			self.int_yaw_rate = 0.0

		else:
			self.heading_hold_active = False
			self.int_heading = 0.0

			err = w_cmd - self.gyro_z

			self.int_yaw_rate += err * dt
			max_int = self.max_yaw_rate_correction / max(self.ki_yaw_rate, 1e-6)
			self.int_yaw_rate = clamp(self.int_yaw_rate, -max_int, max_int)

			correction = self.kp_yaw_rate * err + self.ki_yaw_rate * self.int_yaw_rate
			correction = clamp(
				correction,
				-self.max_yaw_rate_correction,
				self.max_yaw_rate_correction
			)

			w_out = w_cmd + correction

		w_out = clamp(w_out, -self.max_angular_cmd, self.max_angular_cmd)
		self._publish_cmd(v_cmd, w_out)


def main(args=None):
	rclpy.init(args=args)
	node = ImuHeadingAssist()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()