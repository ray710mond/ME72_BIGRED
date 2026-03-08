
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
	namespace = os.uname().nodename

	return LaunchDescription([
		Node(
			package='imu_driver',
			executable='imu_driver',
			name='imu_driver',
			namespace=namespace,
			output='screen',
		),
	])

