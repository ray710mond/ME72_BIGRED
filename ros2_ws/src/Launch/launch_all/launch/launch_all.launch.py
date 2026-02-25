from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # locate config
    pkg_share = get_package_share_directory('launch_all')
    cfg_file = os.path.join(pkg_share, 'config', 'params.yaml')

    try:
        with open(cfg_file, 'r') as f:
            params = yaml.safe_load(f) or {}
    except Exception:
        params = {}

    rf_params = params.get('rf_driver', {})
    driver_type = str(rf_params.get('driver_type', 'pwm'))
    chip_name = str(rf_params.get('chip_name', 'gpiochip4'))

    ld = LaunchDescription()
    ld.add_action(LogInfo(msg=f'launch_all: using config {cfg_file}'))
    ld.add_action(LogInfo(msg=f'launch_all: rf_driver.driver_type={driver_type}, chip_name={chip_name}'))

    if driver_type == 'pwm':
        node = Node(
            package='rf_driver',
            executable='rf_driver_pwm',
            name='rf_driver_pwm',
            output='screen',
            parameters=[{'chip_name': chip_name}],
        )
        ld.add_action(node)
    elif driver_type == 'ibus':
        node = Node(
            package='rf_driver',
            executable='rf_driver_ibus',
            name='rf_driver_ibus',
            output='screen',
        )
        ld.add_action(node)
    else:
        ld.add_action(LogInfo(msg=f'launch_all: unknown rf_driver.driver_type="{driver_type}"; nothing launched'))

    return ld
