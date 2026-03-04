from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	namespace = os.uname().nodename

	return LaunchDescription([
		Node(
			package='servo_driver',
			executable='servo_driver',
			name='servo_driver',
			namespace=namespace,
			output='screen',
		),
	])

