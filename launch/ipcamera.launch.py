import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Read MAV_NAME from environment (default to empty if not set)
    mav_name = EnvironmentVariable('MAV_NAME', default_value='')

    pkg_share = get_package_share_directory('ros2_ipcamera')
    config_dir = os.path.join(pkg_share, 'config')
    param_config = os.path.join(config_dir, 'ipcamera.yaml')

    with open(param_config, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    params = cfg.get('ipcamera', {}).get('ros__parameters', {})

    config_file = 'file://' + os.path.join(config_dir, 'camera_info.yaml')

    ipcamera_node = ComposableNode(
        package='ros2_ipcamera',
        plugin='ros2_ipcamera::IpCamera',
        namespace=mav_name,
        parameters=[
            params,
            {"camera_calibration_file": config_file},
        ],
    )

    container = ComposableNodeContainer(
        name='container',
        namespace='ipcamera_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[ipcamera_node],
        output='screen',
    )

    return LaunchDescription([container])

