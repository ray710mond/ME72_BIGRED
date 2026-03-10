from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
	namespace = os.uname().nodename

	pkg_share = get_package_share_directory('launch_all')
	twist_mux_cfg_file = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

	spark_driver_type = LaunchConfiguration('spark_driver_type').perform(context)
	stream_ip = LaunchConfiguration('stream_ip').perform(context)
	save_recording = LaunchConfiguration('save_recording').perform(context).lower() == 'true'

	actions = []

	actions.append(LogInfo(msg=f'launch_all: namespace={namespace}'))
	actions.append(LogInfo(msg=f'launch_all: spark_driver.driver_type={spark_driver_type}'))
	actions.append(LogInfo(msg=f'launch_all: twist_mux config={twist_mux_cfg_file}'))

	rf_pkg_share = get_package_share_directory('rf_driver')
	spark_pkg_share = get_package_share_directory('spark_driver')
	servo_pkg_share = get_package_share_directory('servo_driver')
	autonomous_pkg_share = get_package_share_directory('autonomous_planner')
	imu_pkg_share = get_package_share_directory('imu_driver')
	roboclaw_pkg_share = get_package_share_directory('roboclaw_driver')
	hole_detector_pkg_share = get_package_share_directory('hole_detector')

	rf_launch_file = os.path.join(rf_pkg_share, 'launch', 'rf_driver.launch.py')
	actions.append(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(rf_launch_file),
			launch_arguments={
				'namespace': namespace,
			}.items()
		)
	)

	servo_launch_file = os.path.join(servo_pkg_share, 'launch', 'servo_driver.launch.py')
	actions.append(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(servo_launch_file),
			launch_arguments={
				'namespace': namespace,
			}.items()
		)
	)

	spark_launch_file = os.path.join(spark_pkg_share, 'launch', 'spark_driver.launch.py')
	actions.append(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(spark_launch_file),
			launch_arguments={
				'namespace': namespace,
				'driver_type': spark_driver_type,
			}.items()
		)
	)

	# imu_launch_file = os.path.join(imu_pkg_share, 'launch', 'imu_driver.launch.py')
	# actions.append(
	# 	IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource(imu_launch_file),
	# 		launch_arguments={
	# 			'namespace': namespace,
	# 		}.items()
	# 	)
	# )

	roboclaw_launch_file = os.path.join(roboclaw_pkg_share, 'launch', 'roboclaw_driver.launch.py')
	actions.append(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(roboclaw_launch_file),
			launch_arguments={
				'namespace': namespace,
			}.items()
		)
	)

	hole_detector_launch_file = os.path.join(hole_detector_pkg_share, 'launch', 'hole_detector.launch.py')
	actions.append(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(hole_detector_launch_file),
			launch_arguments={
				'namespace': namespace,
			}.items()
		)
	)

	# autonomous_launch_file = os.path.join(autonomous_pkg_share, 'launch', 'autonomous_planner.launch.py')
	# actions.append(
	# 	IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource(autonomous_launch_file),
	# 		launch_arguments={
	# 			'namespace': namespace,
	# 		}.items()
	# 	)
	# )

	actions.append(
		Node(
			package='twist_mux',
			executable='twist_mux',
			name='twist_mux',
			namespace=namespace,
			parameters=[twist_mux_cfg_file],
			remappings=[
				('cmd_vel_out', 'cmd_vel_des'),
			],
			output='screen',
		)
	)

	if stream_ip:
		stream_port = LaunchConfiguration('stream_port').perform(context)
		stream_script = os.path.join(pkg_share, 'config', 'stream_camera.sh')
		stream_cmd = ['bash', stream_script, stream_ip, stream_port, '1280', '720', '30']
		if not save_recording:
			stream_cmd.append('--no-save')
		actions.append(ExecuteProcess(
			cmd=stream_cmd,
			output='screen',
			name='camera_stream',
		))
		actions.append(LogInfo(
			msg=f'launch_all: camera stream -> {stream_ip}:{stream_port} (save={save_recording})'
		))

	return actions


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			'spark_driver_type',
			default_value='sw',
			description='Spark driver type (sw or hw)'
		),
		DeclareLaunchArgument(
			'stream_ip',
			default_value='',
			description='IP to stream camera to (empty = no stream)'
		),
		DeclareLaunchArgument(
			'stream_port',
			default_value='5000',
			description='UDP port for camera stream'
		),
		DeclareLaunchArgument(
			'save_recording',
			default_value='true',
			description='Save camera recording on Pi (true/false)'
		),
		OpaqueFunction(function=launch_setup),
	])