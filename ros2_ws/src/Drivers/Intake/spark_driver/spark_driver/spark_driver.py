#!/usr/bin/env python3
"""ROS2 node to control REV Spark Mini for the intake.

Subscribes to `intake_running` (std_msgs/Bool). When true, sets a hardware
PWM output on the configured Raspberry Pi pin to the configured duty that
is expected to run the 775 motor at ~3000 RPM (open-loop). When false, PWM
is stopped.

Parameters (ROS2 params):
- `pwm_pin` (int): GPIO pin number (BCM) for hardware PWM — default 18.
- `pwm_freq` (int): PWM frequency in Hz — default 20000.
- `duty_for_3000` (float): normalized duty 0.0..1.0 to approximate 3000 RPM — default 0.50.

Requires `pigpio` daemon running on the Pi.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pigpio


class SparkDriver(Node):
	def __init__(self):
		super().__init__('spark_driver')

		# Parameters
		# We use servo-style control pulses per motor controller specs
		self.declare_parameter('pwm_pin', 18)
		self.declare_parameter('pulse_min_us', 500)
		self.declare_parameter('neutral_us', 1500)
		self.declare_parameter('pulse_max_us', 2500)
		# duty_for_3000 expresses how far between neutral and max to drive
		# to reach ~3000 RPM (0.0..1.0)
		self.declare_parameter('duty_for_3000', 0.5)

		self.pwm_pin = int(self.get_parameter('pwm_pin').value)
		self.pulse_min_us = int(self.get_parameter('pulse_min_us').value)
		self.neutral_us = int(self.get_parameter('neutral_us').value)
		self.pulse_max_us = int(self.get_parameter('pulse_max_us').value)
		self.duty_for_3000 = float(self.get_parameter('duty_for_3000').value)

		# Connect to pigpio daemon
		self.pi = pigpio.pi()
		if not self.pi.connected:
			self.get_logger().error('Failed to connect to pigpio daemon. Is pigpiod running?')
			raise RuntimeError('pigpio connection failed')

		# Ensure motor is stopped initially
		self.stop_pwm()

		# Subscribe to intake_running
		self.sub = self.create_subscription(Bool, 'intake_running', self._intake_cb, 10)
		self.get_logger().info(f'SparkDriver ready — pwm_pin={self.pwm_pin} pwm_freq={self.pwm_freq}')

	def set_pwm(self, duty_norm: float) -> None:
		"""Set servo-style PWM pulse width based on duty_norm (0..1).

		Maps duty_norm to a pulse between `neutral_us` and `pulse_max_us`.
		Uses pigpio.set_servo_pulsewidth which emits 50Hz-style pulses.
		"""
		duty_norm = max(0.0, min(1.0, float(duty_norm)))

		# Map 0..1 to neutral..pulse_max (single-direction intake)
		pulse_range = self.pulse_max_us - self.neutral_us
		pulse = int(self.neutral_us + duty_norm * pulse_range)

		# Clamp
		if pulse < self.pulse_min_us:
			pulse = self.pulse_min_us
		if pulse > self.pulse_max_us:
			pulse = self.pulse_max_us

		self.get_logger().debug(f'set servo pulse {pulse}us on pin {self.pwm_pin}')
		self.pi.set_servo_pulsewidth(self.pwm_pin, pulse)

	def stop_pwm(self) -> None:
		# For safety set to neutral pulse (stop). If needed this can be 0.
		try:
			self.pi.set_servo_pulsewidth(self.pwm_pin, int(self.neutral_us))
		except Exception:
			pass

	def _intake_cb(self, msg: Bool) -> None:
		if msg.data:
			self.get_logger().info('intake_running: ON — setting PWM')
			self.set_pwm(self.duty_for_3000)
		else:
			self.get_logger().info('intake_running: OFF — stopping PWM')
			self.stop_pwm()

	def destroy_node(self):
		try:
			self.get_logger().info('Shutting down SparkDriver — stopping motor')
			self.stop_pwm()
			try:
				self.pi.stop()
			except Exception:
				pass
		finally:
			super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	node = SparkDriver()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.get_logger().info('SparkDriver exit — cleaning up')
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

