from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='rf_driver',
			executable='rf_driver',
			name='rf_driver',
			output='screen',
			parameters=[
				{'chip_name': 'gpiochip4'},
			],
		),
	])

