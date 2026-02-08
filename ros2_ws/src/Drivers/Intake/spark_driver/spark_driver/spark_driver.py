#!/usr/bin/env python3
import time
import threading

import gpiod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class IntakeSparkGpiod(Node):
	"""
	Subscribes to `intake_running` (std_msgs/Bool).
	If True -> constant forward (2000us @ 50Hz)
	If False -> stop (1500us @ 50Hz)

	GPIO output uses SAME libgpiod request style as rf_driver:
	  chip = gpiod.Chip("gpiochip4")
	  line = chip.get_line(offset)
	  line.request(consumer=..., type=gpiod.LINE_REQ_DIR_OUT)
	"""

	CHIP_NAME = "gpiochip4"  # Pi 5 header chip (matches your rf_driver)
	GPIO_LINE = 12           # GPIO12 (BCM 12), PWM0-capable pin

	PWM_STOP_US = 1500
	PWM_FWD_US  = 2000
	PERIOD_US   = 20_000     # 50Hz

	ARM_SECONDS = 2.0

	def __init__(self):
		super().__init__("spark_driver")

		# State
		self.intake_running = False
		self._stop_evt = threading.Event()

		# GPIO setup (same style as your rf_driver)
		self.chip = gpiod.Chip(self.CHIP_NAME)
		self.line = self.chip.get_line(self.GPIO_LINE)
		self.line.request(
			consumer="spark_pwm_out",
			type=gpiod.LINE_REQ_DIR_OUT,
			default_vals=[0]
		)

		# Subscriber
		self.sub = self.create_subscription(Bool, "intake_running", self._on_intake, 10)

		# Start PWM thread (always running; chooses pulse based on intake_running)
		self.thread = threading.Thread(target=self._pwm_loop, daemon=True)
		self.thread.start()

		# Arm ESC: hold STOP for ARM_SECONDS
		self.get_logger().info(f"Arming ESC: STOP {self.PWM_STOP_US}us for {self.ARM_SECONDS:.1f}s")
		time.sleep(self.ARM_SECONDS)

		self.get_logger().info("Ready. Publish /intake_running (std_msgs/Bool).")

	def _on_intake(self, msg: Bool):
		new_state = bool(msg.data)
		if new_state == self.intake_running:
			return
		self.intake_running = new_state
		self.get_logger().info("INTAKE ON -> forward" if self.intake_running else "INTAKE OFF -> stop")

	def _pwm_loop(self):
		"""
		Software-timed 50Hz pulses.
		High time: 1500us (stop) or 2000us (forward)
		"""
		while not self._stop_evt.is_set():
			high_us = self.PWM_FWD_US if self.intake_running else self.PWM_STOP_US

			# HIGH portion
			self.line.set_value(1)
			time.sleep(high_us / 1_000_000.0)

			# LOW portion
			self.line.set_value(0)
			low_us = self.PERIOD_US - high_us
			if low_us > 0:
				time.sleep(low_us / 1_000_000.0)

	def destroy_node(self):
		# Fail-safe stop
		try:
			self.intake_running = False
			time.sleep(0.05)
			self._stop_evt.set()
			if hasattr(self, "thread"):
				self.thread.join(timeout=0.5)

			if hasattr(self, "line"):
				self.line.set_value(0)
				self.line.release()
			if hasattr(self, "chip"):
				self.chip.close()
		except Exception:
			pass

		super().destroy_node()


def main():
	rclpy.init()
	node = None
	try:
		node = IntakeSparkGpiod()
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if node is not None:
			node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
