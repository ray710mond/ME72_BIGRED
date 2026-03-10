from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

	namespace = LaunchConfiguration('namespace').perform(context)

	if namespace == 'bigred1':
		use_encoders = True
		read_encoders = True
		left_trim = 1.0
		right_trim = 1.0
		log_msg = 'roboclaw launch: bigred1 open loop, encoders still publishing'
	elif namespace == 'bigred2':
		use_encoders = False
		read_encoders = True
		left_trim = 1.0
		right_trim = 0.93
		log_msg = 'roboclaw launch: bigred2 open loop, encoders still publishing'
	else:
		use_encoders = False
		read_encoders = True
		left_trim = 1.0
		right_trim = 1.0
		log_msg = 'roboclaw launch: unknown namespace, open loop, encoders still publishing'

	return [

		LogInfo(msg=log_msg),

		Node(
			package='roboclaw_driver',
			executable='heading_assist',
			name='heading_assist',
			namespace=namespace,
			output='screen'
		),

		Node(
			package='roboclaw_driver',
			executable='roboclaw_driver',
			name='roboclaw_driver',
			namespace=namespace,
			output='screen',
			parameters=[{
				'port': '/dev/ttyACM0',
				'baud': 115200,
				'address': 0x80,

				'use_encoders': use_encoders,
				'read_encoders': read_encoders,

				'cmd_timeout_sec': 0.25,
				'watchdog_period_sec': 0.05,

				'wheel_radius_m': 0.0635,
				'track_width_m': 0.33,

				'encoder_counts_per_motor_rev': 8192.0,
				'gear_ratio': 6.875,

				'left_encoder_sign': 1,
				'right_encoder_sign': 1,

				'left_joint_name': 'left_wheel_joint',
				'right_joint_name': 'right_wheel_joint',

				'max_wheel_speed_rad_s': 350.0,
				'max_accel_cps2': 730000,

				'left_trim': left_trim,
				'right_trim': right_trim,
				'max_duty_scale': 1.0,

				'linear_gain_to_wheel_cmd': 1.0,
				'angular_gain_to_wheel_cmd': 1.0,

				'encoder_poll_period_sec': 0.05,

				'left_motor_command_sign': 1,
				'right_motor_command_sign': 1,
			}],
		)
	]


def generate_launch_description():

	namespace_arg = DeclareLaunchArgument(
		'namespace',
		default_value='',
		description='robot namespace'
	)

	return LaunchDescription([
		namespace_arg,
		OpaqueFunction(function=launch_setup)
	])