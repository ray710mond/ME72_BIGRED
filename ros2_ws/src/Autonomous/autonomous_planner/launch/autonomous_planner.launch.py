from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

	namespace = LaunchConfiguration('namespace').perform(context)
	start_delay = float(LaunchConfiguration('start_delay').perform(context))

	if namespace == 'bigred1':
		# exec_name = 'autonomous_planner_pellets'
	elif namespace == 'bigred2':
		exec_name = 'autonomous_planner_button'
	else:
		exec_name = 'autonomous_planner'

	return [
		Node(
			package='autonomous_planner',
			executable=exec_name,
			name='autonomous_planner',
			namespace=namespace,
			output='screen',
			parameters=[{'start_delay': start_delay}],
		)
	]


def generate_launch_description():

	return LaunchDescription([
		DeclareLaunchArgument('start_delay', default_value='0.0',
					description='seconds to wait before sending first autonomy command'),
		OpaqueFunction(function=launch_setup),
	])