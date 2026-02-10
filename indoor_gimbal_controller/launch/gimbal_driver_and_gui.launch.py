import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value=TextSubstitution(text=os.environ.get("MAV_NAME", "")),
    )

    mav_name = LaunchConfiguration("mav_name")

    driver = Node(
        package="indoor_gimbal_controller",
        executable="topotek_udp_driver",
        name="topotek_udp_driver",
        namespace=mav_name,
        output="screen",
        # optionally add params yaml here later
        # parameters=[...],
    )

    gui = Node(
        package="indoor_gimbal_controller",
        executable="topotek_operator_gui_ros",
        name="topotek_operator_gui_ros",
        namespace=mav_name,
        output="screen",
    )

    return LaunchDescription([
        mav_name_arg,
        driver,
        gui,
    ])
