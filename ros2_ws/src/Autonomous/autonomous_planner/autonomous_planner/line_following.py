#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import gpiod
import time


class IRLineFollower(Node):

	def __init__(self):
		super().__init__("ir_line_follower")

		# ---- GPIO Setup ----
		self.chip = gpiod.Chip("gpiochip0")

		self.sensor_pins = [4, 5, 6, 16, 20, 21, 26, 14]
		self.lines = []

		for pin in self.sensor_pins:
			line = self.chip.get_line(pin)
			line.request(consumer="ir_array", type=gpiod.LINE_REQ_DIR_IN)
			self.lines.append(line)

		# ---- Control parameters ----
		self.base_speed = 0.25
		self.kp = 0.005

		# Sensor positions (left → right)
		self.positions = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]

		# Publisher
		self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_des", 10)

		self.timer = self.create_timer(0.01, self.control_loop)

		self.last_error = 0.0

		self.get_logger().info("IR Line Follower Started")

	def read_sensors(self):
		values = []
		for line in self.lines:
			values.append(line.get_value())
		return values

	def compute_error(self, sensor_values):
		total = sum(sensor_values)

		if total == 0:
			# Lost line — return last error to keep turning
			return self.last_error

		weighted_sum = 0.0
		for i in range(8):
			weighted_sum += sensor_values[i] * self.positions[i]

		error = weighted_sum / total
		return error

	def control_loop(self):
		sensors = self.read_sensors()
		error = self.compute_error(sensors)

		self.last_error = error

		angular = -self.kp * error

		msg = Twist()
		msg.linear.x = self.base_speed
		msg.angular.z = angular

		self.cmd_pub.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = IRLineFollower()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()