import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory("indoor_camera")

    default_mav = EnvironmentVariable("MAV_NAME", default_value="")
    default_params = PathJoinSubstitution([pkg_share, "config", "rtsp_camera.yaml"])

    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value=default_mav,
        description="UAV namespace (default: env MAV_NAME). Example: uav60",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML config file for both GUI and driver",
    )

    mav_name = LaunchConfiguration("mav_name")
    params_file = LaunchConfiguration("params_file")

    # --- Driver ---
    driver_node = Node(
        package="indoor_gimbal_controller",
        executable="topotek_udp_driver",
        name="topotek_udp_driver",
        output="screen",
        parameters=[params_file],
        # If you need to force absolute topics (no namespace), you can remap here.
        # remappings=[("gimbal_angles_gyro_deg", "/gimbal_angles_gyro_deg")]
    )

    # --- GUI (your app that runs GStreamer + buttons + angle entry) ---
    gui_node = Node(
        package="indoor_gimbal_controller",
        executable="indoor_camera_gui",
        name="indoor_camera_gui",
        output="screen",
        parameters=[params_file],
    )

    # If mav_name is empty, we don't want a leading "/" namespace group.
    # But PushRosNamespace("") is okay; it effectively keeps topics relative.
    group = GroupAction([
        PushRosNamespace(mav_name),
        driver_node,
        gui_node,
    ])

    return LaunchDescription([
        mav_name_arg,
        params_arg,
        group,
    ])
