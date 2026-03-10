from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


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
			package='imu_driver',
			executable='imu_driver',
			name='imu_driver',
			namespace=namespace,
			output='screen',
			condition=IfCondition(PythonExpression(["'", namespace, "' == 'bigred1'"])),
		),
	])