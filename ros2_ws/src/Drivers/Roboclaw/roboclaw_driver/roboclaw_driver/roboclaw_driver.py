#!/usr/bin/env python3
"""
roboclaw_hybrid_driver.py

Hybrid RoboClaw ROS 2 driver.

Subscribes:
  - cmd_vel                     (geometry_msgs/msg/Twist)

Publishes:
  - cmd_vel_applied            (geometry_msgs/msg/Twist)      filtered/applied command
  - wheel_states               (sensor_msgs/msg/JointState)   [if read_encoders=True]
  - encoder/left/count         (std_msgs/msg/Int64)           [if read_encoders=True]
  - encoder/right/count        (std_msgs/msg/Int64)           [if read_encoders=True]
  - encoder/left/speed_cps     (std_msgs/msg/Int32)           [if read_encoders=True]
  - encoder/right/speed_cps    (std_msgs/msg/Int32)           [if read_encoders=True]

Modes:
  - use_encoders = True:
      uses RoboClaw signed speed commands in counts/sec
  - use_encoders = False:
      uses duty-cycle open loop, with calibration trims

Independent encoder publishing:
  - read_encoders = True:
      poll and publish encoder data regardless of control mode

Important:
  - Do NOT run a separate encoder node on the same RoboClaw port at the same time.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Int64

from .roboclaw_python.roboclaw_3 import Roboclaw


def clamp(x: float, lo: float, hi: float) -> float:
	return max(lo, min(hi, x))


class RoboclawHybridDriver(Node):
	def __init__(self):
		super().__init__('roboclaw_hybrid_driver')

		# ---------------- Parameters ----------------
		self.declare_parameter('port', '/dev/ttyACM0')
		self.declare_parameter('baud', 115200)
		self.declare_parameter('address', 0x80)

		# Split control-vs-read behavior
		self.declare_parameter('use_encoders', False)
		self.declare_parameter('read_encoders', True)

		self.declare_parameter('cmd_timeout_sec', 0.25)
		self.declare_parameter('watchdog_period_sec', 0.05)

		# Robot geometry
		self.declare_parameter('wheel_radius_m', 0.05)
		self.declare_parameter('track_width_m', 0.30)

		# Encoder parameters
		self.declare_parameter('encoder_counts_per_motor_rev', 8192.0)
		self.declare_parameter('gear_ratio', 1.0)

		self.declare_parameter('left_encoder_sign', 1)
		self.declare_parameter('right_encoder_sign', 1)

		# Motor command signs
		self.declare_parameter('left_motor_command_sign', 1)
		self.declare_parameter('right_motor_command_sign', -1)

		# Wheel names
		self.declare_parameter('left_joint_name', 'left_wheel_joint')
		self.declare_parameter('right_joint_name', 'right_wheel_joint')

		# Speed mode params
		self.declare_parameter('max_wheel_speed_rad_s', 25.0)
		self.declare_parameter('max_accel_cps2', 120000)

		# Duty mode params
		self.declare_parameter('left_trim', 1.0)
		self.declare_parameter('right_trim', 1.0)
		self.declare_parameter('max_duty_scale', 1.0)

		# Mapping for open-loop robot
		self.declare_parameter('linear_gain_to_wheel_cmd', 1.0)
		self.declare_parameter('angular_gain_to_wheel_cmd', 1.0)

		# Encoder publish
		self.declare_parameter('encoder_poll_period_sec', 0.05)

		port = str(self.get_parameter('port').value)
		baud = int(self.get_parameter('baud').value)
		self.address = int(self.get_parameter('address').value)

		self.use_encoders = bool(self.get_parameter('use_encoders').value)
		self.read_encoders = bool(self.get_parameter('read_encoders').value)

		self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
		watchdog_period_sec = float(self.get_parameter('watchdog_period_sec').value)

		self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
		self.track_width_m = float(self.get_parameter('track_width_m').value)

		self.encoder_counts_per_motor_rev = float(self.get_parameter('encoder_counts_per_motor_rev').value)
		self.gear_ratio = float(self.get_parameter('gear_ratio').value)

		self.left_encoder_sign = int(self.get_parameter('left_encoder_sign').value)
		self.right_encoder_sign = int(self.get_parameter('right_encoder_sign').value)

		self.left_motor_command_sign = int(self.get_parameter('left_motor_command_sign').value)
		self.right_motor_command_sign = int(self.get_parameter('right_motor_command_sign').value)

		self.left_joint_name = str(self.get_parameter('left_joint_name').value)
		self.right_joint_name = str(self.get_parameter('right_joint_name').value)

		self.max_wheel_speed_rad_s = float(self.get_parameter('max_wheel_speed_rad_s').value)
		self.max_accel_cps2 = int(self.get_parameter('max_accel_cps2').value)

		self.left_trim = float(self.get_parameter('left_trim').value)
		self.right_trim = float(self.get_parameter('right_trim').value)
		self.max_duty_scale = float(self.get_parameter('max_duty_scale').value)

		self.linear_gain_to_wheel_cmd = float(self.get_parameter('linear_gain_to_wheel_cmd').value)
		self.angular_gain_to_wheel_cmd = float(self.get_parameter('angular_gain_to_wheel_cmd').value)

		encoder_poll_period_sec = float(self.get_parameter('encoder_poll_period_sec').value)

		# ---------------- Roboclaw ----------------
		self.rc = Roboclaw(comport=port, rate=baud)
		opened = self.rc.Open()
		if not opened:
			raise RuntimeError(f'Failed to open RoboClaw on {port}')

		# ---------------- State ----------------
		self.last_cmd_time = self.get_clock().now()
		self.timed_out = True

		# ---------------- I/O ----------------
		self.sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 20)

		self.pub_cmd_vel_applied = self.create_publisher(Twist, 'cmd_vel_applied', 20)

		self.watchdog_timer = self.create_timer(watchdog_period_sec, self._watchdog_cb)

		# Create encoder publishers/timer whenever read_encoders is enabled,
		# regardless of whether encoder feedback is used for control.
		if self.read_encoders:
			self.pub_left_count = self.create_publisher(Int64, 'encoder/left/count', 10)
			self.pub_right_count = self.create_publisher(Int64, 'encoder/right/count', 10)
			self.pub_left_speed = self.create_publisher(Int32, 'encoder/left/speed_cps', 10)
			self.pub_right_speed = self.create_publisher(Int32, 'encoder/right/speed_cps', 10)
			self.pub_joint_states = self.create_publisher(JointState, 'wheel_states', 10)
			self.encoder_timer = self.create_timer(encoder_poll_period_sec, self._encoder_poll_cb)

		self.get_logger().info(
			f'roboclaw_hybrid_driver ready port={port} '
			f'use_encoders={self.use_encoders} '
			f'read_encoders={self.read_encoders} '
			f'left_motor_command_sign={self.left_motor_command_sign} '
			f'right_motor_command_sign={self.right_motor_command_sign}'
		)

	# ------------------------------------------------------------------
	# Helpers
	# ------------------------------------------------------------------
	def _extract_value(self, result) -> Optional[int]:
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

	def _wheel_rad_s_to_cps(self, wheel_rad_s: float) -> int:
		motor_rev_per_sec = (wheel_rad_s / (2.0 * math.pi)) * self.gear_ratio
		counts_per_sec = motor_rev_per_sec * self.encoder_counts_per_motor_rev
		return int(round(counts_per_sec))

	def _cps_to_wheel_rad_s(self, cps: int) -> float:
		return (2.0 * math.pi * float(cps)) / (
			self.encoder_counts_per_motor_rev * self.gear_ratio
		)

	def _counts_to_wheel_rad(self, counts: int) -> float:
		return (2.0 * math.pi * float(counts)) / (
			self.encoder_counts_per_motor_rev * self.gear_ratio
		)

	def _twist_to_wheel_linear(self, v: float, w: float):
		v_left = v - 0.5 * self.track_width_m * w
		v_right = v + 0.5 * self.track_width_m * w
		return v_left, v_right

	def _wheel_linear_to_rad_s(self, v_wheel: float) -> float:
		return v_wheel / self.wheel_radius_m

	def _wheel_rad_s_to_body_twist(self, w_left: float, w_right: float):
		v_left = w_left * self.wheel_radius_m
		v_right = w_right * self.wheel_radius_m
		v = 0.5 * (v_left + v_right)
		w = (v_right - v_left) / self.track_width_m
		return v, w

	def _publish_filtered_cmd(self, v: float, w: float):
		msg = Twist()
		msg.linear.x = float(v)
		msg.angular.z = float(w)
		self.pub_cmd_vel_applied.publish(msg)

	def _stop(self):
		try:
			if self.use_encoders:
				self._send_speed_commands(0, 0)
			else:
				self._send_duty_commands(0, 0)
			self._publish_filtered_cmd(0.0, 0.0)
		except Exception as e:
			self.get_logger().error(f'Failed to stop motors: {e}')

	def _send_duty_commands(self, left_duty: int, right_duty: int):
		left_duty_cmd = int(self.left_motor_command_sign * left_duty)
		right_duty_cmd = int(self.right_motor_command_sign * right_duty)

		self.rc.DutyM1(self.address, left_duty_cmd)
		self.rc.DutyM2(self.address, right_duty_cmd)

	def _send_speed_commands(self, left_cps: int, right_cps: int):
		left_cps_cmd = int(self.left_motor_command_sign * left_cps)
		right_cps_cmd = int(self.right_motor_command_sign * right_cps)

		if hasattr(self.rc, 'SpeedAccelM1M2'):
			self.rc.SpeedAccelM1M2(
				self.address,
				int(self.max_accel_cps2),
				left_cps_cmd,
				right_cps_cmd
			)
		elif hasattr(self.rc, 'SpeedM1M2'):
			self.rc.SpeedM1M2(self.address, left_cps_cmd, right_cps_cmd)
		else:
			if hasattr(self.rc, 'SpeedAccelM1') and hasattr(self.rc, 'SpeedAccelM2'):
				self.rc.SpeedAccelM1(self.address, int(self.max_accel_cps2), left_cps_cmd)
				self.rc.SpeedAccelM2(self.address, int(self.max_accel_cps2), right_cps_cmd)
			else:
				self.rc.SpeedM1(self.address, left_cps_cmd)
				self.rc.SpeedM2(self.address, right_cps_cmd)

	# ------------------------------------------------------------------
	# Command path
	# ------------------------------------------------------------------
	def _cmd_cb(self, msg: Twist):
		self.last_cmd_time = self.get_clock().now()
		self.timed_out = False

		v = float(msg.linear.x)
		w = -float(msg.angular.z)   # flip steering direction

		v_left, v_right = self._twist_to_wheel_linear(v, w)
		w_left = self._wheel_linear_to_rad_s(v_left)
		w_right = self._wheel_linear_to_rad_s(v_right)

		if self.use_encoders:
			w_left_cmd = clamp(w_left, -self.max_wheel_speed_rad_s, self.max_wheel_speed_rad_s)
			w_right_cmd = clamp(w_right, -self.max_wheel_speed_rad_s, self.max_wheel_speed_rad_s)

			left_cps = self._wheel_rad_s_to_cps(w_left_cmd) * self.left_encoder_sign
			right_cps = self._wheel_rad_s_to_cps(w_right_cmd) * self.right_encoder_sign

			try:
				self._send_speed_commands(left_cps, right_cps)
			except Exception as e:
				self.get_logger().error(f'Failed sending speed commands: {e}')
				return

			v_out, w_out = self._wheel_rad_s_to_body_twist(w_left_cmd, w_right_cmd)
			self._publish_filtered_cmd(v_out, w_out)

		else:
			left = self.linear_gain_to_wheel_cmd * v + self.angular_gain_to_wheel_cmd * w
			right = self.linear_gain_to_wheel_cmd * v - self.angular_gain_to_wheel_cmd * w

			left *= self.left_trim
			right *= self.right_trim

			left_cmd = clamp(left, -self.max_duty_scale, self.max_duty_scale)
			right_cmd = clamp(right, -self.max_duty_scale, self.max_duty_scale)

			max_duty = 32767
			left_duty = int(left_cmd * max_duty)
			right_duty = int(right_cmd * max_duty)

			try:
				self._send_duty_commands(left_duty, right_duty)
			except Exception as e:
				self.get_logger().error(f'Failed sending duty commands: {e}')
				return

			if abs(self.linear_gain_to_wheel_cmd) > 1e-9:
				v_out = 0.5 * (left_cmd + right_cmd) / self.linear_gain_to_wheel_cmd
			else:
				v_out = 0.0

			if abs(self.angular_gain_to_wheel_cmd) > 1e-9:
				w_out = 0.5 * (left_cmd - right_cmd) / self.angular_gain_to_wheel_cmd
			else:
				w_out = 0.0

			self._publish_filtered_cmd(v_out, w_out)

	def _watchdog_cb(self):
		age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
		if age > self.cmd_timeout_sec and not self.timed_out:
			self.get_logger().warn(f'cmd_vel timeout ({age:.3f}s), stopping motors')
			self._stop()
			self.timed_out = True

	# ------------------------------------------------------------------
	# Encoder publishing path
	# ------------------------------------------------------------------
	def _read_enc_m1(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadEncM1(self.address))
		except Exception:
			return None

	def _read_enc_m2(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadEncM2(self.address))
		except Exception:
			return None

	def _read_speed_m1(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadSpeedM1(self.address))
		except Exception:
			return None

	def _read_speed_m2(self) -> Optional[int]:
		try:
			return self._extract_value(self.rc.ReadSpeedM2(self.address))
		except Exception:
			return None

	def _encoder_poll_cb(self):
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

		msg = Int64()
		msg.data = left_count
		self.pub_left_count.publish(msg)

		msg = Int64()
		msg.data = right_count
		self.pub_right_count.publish(msg)

		msg = Int32()
		msg.data = left_speed
		self.pub_left_speed.publish(msg)

		msg = Int32()
		msg.data = right_speed
		self.pub_right_speed.publish(msg)

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

	def destroy_node(self):
		try:
			self._stop()
		finally:
			super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	node = RoboclawHybridDriver()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()