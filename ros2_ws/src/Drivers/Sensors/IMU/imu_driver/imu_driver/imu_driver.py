#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from smbus import SMBus


class ImuDriver(Node):
	def __init__(self):
		super().__init__('imu_driver')

		self.declare_parameter('bus', 1)
		self.declare_parameter('address', 0x68)
		self.declare_parameter('frame_id', 'imu_link')
		self.declare_parameter('publish_rate_hz', 100.0)
		self.declare_parameter('gyro_calibration_seconds', 2.0)

		self.bus_num = int(self.get_parameter('bus').value)
		self.address = int(self.get_parameter('address').value)
		self.frame_id = str(self.get_parameter('frame_id').value)
		self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
		self.gyro_calibration_seconds = float(self.get_parameter('gyro_calibration_seconds').value)

		self.bus = SMBus(self.bus_num)

		# Shared MPU6050 / MPU9250 accel+gyro registers
		self.REG_WHO_AM_I = 0x75
		self.REG_PWR_MGMT_1 = 0x6B
		self.REG_ACCEL_CONFIG = 0x1C
		self.REG_GYRO_CONFIG = 0x1B
		self.REG_ACCEL_XOUT_H = 0x3B

		# Full-scale settings used below
		self.accel_lsb_per_g = 16384.0		# +/- 2g
		self.gyro_lsb_per_dps = 131.0		# +/- 250 deg/s

		# Gyro bias in raw counts
		self.gyro_bias_x = 0.0
		self.gyro_bias_y = 0.0
		self.gyro_bias_z = 0.0

		# Integrated heading state
		self.yaw = 0.0
		self.last_time = None

		self.device_name = self.detect_device()
		self.initialize_imu()
		self.calibrate_gyro_bias()

		self.publisher = self.create_publisher(Imu, 'imu/data', 10)
		self.zero_sub = self.create_subscription(Bool, 'imu/zero_yaw', self.zero_yaw_callback, 10)

		self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_imu)

		self.get_logger().info(
			f'IMU initialized on /dev/i2c-{self.bus_num} '
			f'addr=0x{self.address:02X} type={self.device_name}'
		)

	def read_u8(self, reg: int) -> int:
		return self.bus.read_byte_data(self.address, reg)

	def write_u8(self, reg: int, value: int) -> None:
		self.bus.write_byte_data(self.address, reg, value)

	def read_i16_from_buf(self, buf, idx: int) -> int:
		value = (buf[idx] << 8) | buf[idx + 1]
		if value & 0x8000:
			value -= 0x10000
		return value

	def wrap_angle(self, angle: float) -> float:
		while angle > math.pi:
			angle -= 2.0 * math.pi
		while angle < -math.pi:
			angle += 2.0 * math.pi
		return angle

	def quaternion_from_yaw(self, yaw: float):
		half = yaw * 0.5
		return (0.0, 0.0, math.sin(half), math.cos(half))

	def detect_device(self) -> str:
		try:
			who = self.read_u8(self.REG_WHO_AM_I)
			self.get_logger().info(f'WHO_AM_I = 0x{who:02X}')

			if who == 0x68:
				return 'MPU6050_or_compatible'
			if who in [0x70, 0x71, 0x73, 0x75]:
				return 'MPU9250_or_compatible'

			self.get_logger().warn(f'Unexpected WHO_AM_I value: 0x{who:02X}, continuing anyway')
			return f'unknown_0x{who:02X}'

		except Exception as e:
			raise RuntimeError(f'Failed to detect IMU at 0x{self.address:02X}: {e}')

	def initialize_imu(self) -> None:
		# Device reset
		self.write_u8(self.REG_PWR_MGMT_1, 0x80)
		time.sleep(0.1)

		# Wake up, use gyro X-axis PLL as clock source
		self.write_u8(self.REG_PWR_MGMT_1, 0x01)
		time.sleep(0.05)

		# Accel full scale = +/- 2g
		self.write_u8(self.REG_ACCEL_CONFIG, 0x00)

		# Gyro full scale = +/- 250 deg/s
		self.write_u8(self.REG_GYRO_CONFIG, 0x00)

		time.sleep(0.05)

	def read_raw_burst(self):
		# 14-byte burst read:
		# accel xyz, temp, gyro xyz
		data = self.bus.read_i2c_block_data(self.address, self.REG_ACCEL_XOUT_H, 14)

		ax_raw = self.read_i16_from_buf(data, 0)
		ay_raw = self.read_i16_from_buf(data, 2)
		az_raw = self.read_i16_from_buf(data, 4)

		gx_raw = self.read_i16_from_buf(data, 8)
		gy_raw = self.read_i16_from_buf(data, 10)
		gz_raw = self.read_i16_from_buf(data, 12)

		return ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw

	def calibrate_gyro_bias(self) -> None:
		self.get_logger().info(
			f'Calibrating gyro bias for {self.gyro_calibration_seconds:.1f}s. Keep robot still.'
		)

		samples = 0
		sum_gx = 0.0
		sum_gy = 0.0
		sum_gz = 0.0

		start = time.time()
		while (time.time() - start) < self.gyro_calibration_seconds:
			try:
				_, _, _, gx_raw, gy_raw, gz_raw = self.read_raw_burst()
				sum_gx += gx_raw
				sum_gy += gy_raw
				sum_gz += gz_raw
				samples += 1
			except Exception as e:
				self.get_logger().warn(f'Gyro calibration sample failed: {e}')
			time.sleep(0.005)

		if samples == 0:
			self.get_logger().warn('Gyro calibration failed; using zero bias')
			self.gyro_bias_x = 0.0
			self.gyro_bias_y = 0.0
			self.gyro_bias_z = 0.0
		else:
			self.gyro_bias_x = sum_gx / samples
			self.gyro_bias_y = sum_gy / samples
			self.gyro_bias_z = sum_gz / samples

		# Force boot heading to zero
		self.yaw = 0.0
		self.last_time = None

		self.get_logger().info(
			f'Gyro bias raw counts: '
			f'x={self.gyro_bias_x:.2f}, '
			f'y={self.gyro_bias_y:.2f}, '
			f'z={self.gyro_bias_z:.2f}'
		)
		self.get_logger().info('Initial heading set to yaw = 0 rad')

	def zero_yaw_callback(self, msg: Bool) -> None:
		if msg.data:
			self.yaw = 0.0
			self.last_time = None
			self.get_logger().info('Yaw reset to 0')

	def publish_imu(self) -> None:
		try:
			ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.read_raw_burst()

			# Bias-correct gyro raw counts
			gx_raw -= self.gyro_bias_x
			gy_raw -= self.gyro_bias_y
			gz_raw -= self.gyro_bias_z

			# Convert accel to m/s^2
			ax = (ax_raw / self.accel_lsb_per_g) * 9.80665
			ay = (ay_raw / self.accel_lsb_per_g) * 9.80665
			az = (az_raw / self.accel_lsb_per_g) * 9.80665

			# Convert gyro to rad/s
			gx = math.radians(gx_raw / self.gyro_lsb_per_dps)
			gy = math.radians(gy_raw / self.gyro_lsb_per_dps)
			gz = math.radians(gz_raw / self.gyro_lsb_per_dps)

			now = self.get_clock().now()
			now_sec = now.nanoseconds * 1e-9

			if self.last_time is None:
				dt = 0.0
			else:
				dt = now_sec - self.last_time

			self.last_time = now_sec

			# Integrate yaw from gyro z
			if 0.0 < dt < 0.1:
				self.yaw += gz * dt
				self.yaw = self.wrap_angle(self.yaw)

			qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)

			msg = Imu()
			msg.header.stamp = now.to_msg()
			msg.header.frame_id = self.frame_id

			# Yaw-only orientation estimate
			msg.orientation.x = qx
			msg.orientation.y = qy
			msg.orientation.z = qz
			msg.orientation.w = qw

			msg.angular_velocity.x = gx
			msg.angular_velocity.y = gy
			msg.angular_velocity.z = gz

			msg.linear_acceleration.x = ax
			msg.linear_acceleration.y = ay
			msg.linear_acceleration.z = az

			# Placeholder covariances; tune later if needed
			msg.orientation_covariance[0] = 0.05
			msg.orientation_covariance[4] = 0.05
			msg.orientation_covariance[8] = 0.10

			msg.angular_velocity_covariance[0] = 0.02
			msg.angular_velocity_covariance[4] = 0.02
			msg.angular_velocity_covariance[8] = 0.02

			msg.linear_acceleration_covariance[0] = 0.10
			msg.linear_acceleration_covariance[4] = 0.10
			msg.linear_acceleration_covariance[8] = 0.10

			self.publisher.publish(msg)

		except Exception as e:
			self.get_logger().warn(f'IMU read failed: {e}')


def main(args=None):
	rclpy.init(args=args)
	node = ImuDriver()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()