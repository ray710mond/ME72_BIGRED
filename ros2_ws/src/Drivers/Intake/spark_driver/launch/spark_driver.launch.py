from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
	driver_type_arg = DeclareLaunchArgument(
		'driver_type',
		default_value='sw',
		description='Driver type to launch: "hw" or "sw"',
	)

	namespace_arg = DeclareLaunchArgument(
		'namespace',
		default_value='',
		description='ROS namespace',
	)

	driver_type = LaunchConfiguration('driver_type')
	namespace = LaunchConfiguration('namespace')

	hw_node = Node(
		package='spark_driver',
		executable='spark_driver_hw',
		name='spark_driver_hw',
		namespace=namespace,
		output='screen',
		condition=IfCondition(PythonExpression(["'", driver_type, "' == 'hw'"])),
	)

	sw_node = Node(
		package='spark_driver',
		executable='spark_driver_sw',
		name='spark_driver_sw',
		namespace=namespace,
		output='screen',
		condition=IfCondition(PythonExpression(["'", driver_type, "' == 'sw'"])),
	)

	return LaunchDescription([
		driver_type_arg,
		namespace_arg,
		hw_node,
		sw_node,
	])