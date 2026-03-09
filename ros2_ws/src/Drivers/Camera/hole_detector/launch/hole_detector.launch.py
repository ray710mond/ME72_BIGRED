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

	camera_source_arg = DeclareLaunchArgument(
		'camera_source',
		default_value='0',
		description='Camera index, file path, or GStreamer pipeline',
	)

	namespace = LaunchConfiguration('namespace')
	camera_source = LaunchConfiguration('camera_source')

	node = Node(
		package='hole_detector',
		executable='hole_detector',
		name='hole_detector',
		namespace=namespace,
		output='screen',
		parameters=[{
			'camera_source': camera_source,
		}],
	)

	return LaunchDescription([
		namespace_arg,
		camera_source_arg,
		node,
	])
