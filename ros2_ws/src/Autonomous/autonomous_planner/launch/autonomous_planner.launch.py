from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import osfrom launch import LaunchDescription
from launch_ros.actions import Node


pkg_share = get_package_share_directory('launch_all')
config_file = os.path.join(
		pkg_share,
		'config',
		'params.yaml'
	)


def generate_launch_description():
	namespace = os.uname().nodename

	return LaunchDescription([
		Node(
			package='autonomous_planner',
			executable='line_following',
			name='autonomous_planner',
			namespace=namespace,
			output='screen',
		),

		Node(
		package='autonomous_planner',
		executable='autonomous_planner_node',
		name='autonomous_planner',
		namespace=namespace,
		output='screen',
		parameters=[config_file],
		),
	])

