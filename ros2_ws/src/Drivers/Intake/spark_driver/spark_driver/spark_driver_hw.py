#!/usr/bin/env python3
import os
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class IntakeSparkHwPwm(Node):
	"""
	Subscribes to `intake_running` (std_msgs/Bool).
	If True -> forward (1000us pulse)
	If False -> stop (no PWM output)

	Uses Pi 5 hardware PWM via sysfs for jitter-free output.
	GPIO12 = PWM0 channel 0
	"""

	PWM_CHIP = "/sys/class/pwm/pwmchip0"
	PWM_CHANNEL = 0  # GPIO12 = PWM0 channel 0

	PERIOD_NS = 20_000_000   # 50Hz = 20ms
	PWM_FWD_NS = 1_000_000   # 1000us = 1ms

	ARM_SECONDS = 2.0

	def __init__(self):
		super().__init__("spark_driver")

		# Thread-safe state
		self._lock = threading.Lock()
		self._intake_running = False

		# Hardware PWM setup
		self.pwm_path = f"{self.PWM_CHIP}/pwm{self.PWM_CHANNEL}"
		self._setup_hw_pwm()

		# Subscriber
		self.sub = self.create_subscription(Bool, "intake_running", self._on_intake, 10)

		# Arm ESC: hold stopped state
		self.get_logger().info(f"Arming ESC: stopped for {self.ARM_SECONDS:.1f}s")
		time.sleep(self.ARM_SECONDS)

		self.get_logger().info("Ready. Publish /intake_running (std_msgs/Bool).")

	def _setup_hw_pwm(self):
		"""Export and configure hardware PWM channel."""
		# Export the channel if not already exported
		if not os.path.exists(self.pwm_path):
			with open(f"{self.PWM_CHIP}/export", "w") as f:
				f.write(str(self.PWM_CHANNEL))
			time.sleep(0.1)  # Wait for sysfs to create files

		# Set period (must be set before duty_cycle)
		with open(f"{self.pwm_path}/period", "w") as f:
			f.write(str(self.PERIOD_NS))

		# Start with 0 duty cycle (stopped)
		with open(f"{self.pwm_path}/duty_cycle", "w") as f:
			f.write("0")

		# Enable PWM
		with open(f"{self.pwm_path}/enable", "w") as f:
			f.write("1")

		self.get_logger().info(f"Hardware PWM initialized on {self.pwm_path}")

	def _on_intake(self, msg: Bool):
		new_state = bool(msg.data)
		with self._lock:
			if new_state == self._intake_running:
				return
			self._intake_running = new_state

		# Set duty cycle based on state
		duty_ns = self.PWM_FWD_NS if new_state else 0
		try:
			with open(f"{self.pwm_path}/duty_cycle", "w") as f:
				f.write(str(duty_ns))
		except OSError as e:
			self.get_logger().error(f"Failed to set PWM duty cycle: {e}")

		self.get_logger().info("INTAKE ON -> forward" if new_state else "INTAKE OFF -> stop")

	def destroy_node(self):
		# Fail-safe stop
		try:
			# Set duty cycle to 0 (stop)
			with open(f"{self.pwm_path}/duty_cycle", "w") as f:
				f.write("0")
			# Disable PWM
			with open(f"{self.pwm_path}/enable", "w") as f:
				f.write("0")
		except Exception:
			pass

		super().destroy_node()


def main():
	rclpy.init()
	node = None
	try:
		node = IntakeSparkHwPwm()
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if node is not None:
			node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
