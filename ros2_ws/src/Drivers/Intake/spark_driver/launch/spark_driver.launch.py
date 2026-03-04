from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
	driver_type_arg = DeclareLaunchArgument(
		'driver_type',
		default_value='hw',
		description='Driver type to launch: "hw" or "sw"',
	)

	namespace = os.uname().nodename
	driver_type = LaunchConfiguration('driver_type')

	hw_node = Node(
		package='spark_driver',
		executable='spark_driver_hw',
		name='spark_driver_hw',
		namespace=namespace,
		output='screen',
		parameters=[
			{'pwm_pin': 18},
			{'pulse_min_us': 500},
			{'neutral_us': 1500},
			{'pulse_max_us': 2500},
			{'duty_for_3000': 0.5},
		],
		condition=IfCondition(PythonExpression(["'", driver_type, "' == 'hw'"])),
	)

	sw_node = Node(
		package='spark_driver',
		executable='spark_driver_sw',
		name='spark_driver_sw',
		namespace=namespace,
		output='screen',
		parameters=[
			{'pwm_pin': 18},
			{'pulse_min_us': 500},
			{'neutral_us': 1500},
			{'pulse_max_us': 2500},
			{'duty_for_3000': 0.5},
		],
		condition=IfCondition(PythonExpression(["'", driver_type, "' == 'sw'"])),
	)

	return LaunchDescription([
		driver_type_arg,
		hw_node,
		sw_node,
	])

