from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
	driver_type_arg = DeclareLaunchArgument(
		'driver_type',
		default_value='ibus',
		description='Driver type to launch: "ibus" or "pwm"',
	)
	namespace = os.uname().nodename

	driver_type = LaunchConfiguration('driver_type')

	pwm_node = Node(
		package='rf_driver',
		executable='rf_driver_pwm',
		name='rf_driver_pwm',
		namespace=namespace,
		output='screen',
		parameters=[{'chip_name': 'gpiochip4'}],
		condition=IfCondition(PythonExpression(["'", driver_type, "' == 'pwm'"])),
	)

	ibus_node = Node(
		package='rf_driver',
		executable='rf_driver_ibus',
		name='rf_driver_ibus',
		namespace=namespace,
		output='screen',
		condition=IfCondition(PythonExpression(["'", driver_type, "' == 'ibus'"])),
	)

	return LaunchDescription([
		driver_type_arg,
		pwm_node,
		ibus_node,
	])

