
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	namespace = os.uname().nodename

	return LaunchDescription([
		Node(
			package='roboclaw_driver',
			executable='roboclaw_driver',
			name='roboclaw_driver',
			namespace=namespace,
			output='screen',
		),
	])

