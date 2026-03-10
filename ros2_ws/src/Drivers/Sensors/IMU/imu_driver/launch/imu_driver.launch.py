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
	if namespace == 'bigred1':
		return LaunchDescription([
			namespace_arg,
			Node(
				package='imu_driver',
				executable='imu_driver',
				name='imu_driver',
				namespace=namespace,
				output='screen',
			),
		])