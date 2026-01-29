from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='spark_driver',
			executable='spark_driver',
			name='spark_driver',
			output='screen',
			parameters=[
				{'pwm_pin': 18},
				{'pulse_min_us': 500},
				{'neutral_us': 1500},
				{'pulse_max_us': 2500},
				{'duty_for_3000': 0.5},
			],
		),
	])

