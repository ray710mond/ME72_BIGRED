from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	namespace_arg = DeclareLaunchArgument(
		'namespace',
		default_value='',
		description='ROS namespace',
	)

	namespace = LaunchConfiguration('namespace')

	return LaunchDescription([
		namespace_arg,
		Node(
			package='servo_driver',
			executable='servo_driver',
			name='servo_driver',
			namespace=namespace,
			output='screen',
		),
	])