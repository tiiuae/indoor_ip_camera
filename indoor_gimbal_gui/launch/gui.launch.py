import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value=TextSubstitution(text=os.environ.get("MAV_NAME", "")),
        description="UAV namespace (default: env MAV_NAME). Example: uav60",
    )

    rtsp_uri_arg = DeclareLaunchArgument(
        "rtsp_uri",
        default_value=TextSubstitution(text="rtsp://192.168.1.108:554/stream=1"),
        description="RTSP URI",
    )

    rtsp_protocol_arg = DeclareLaunchArgument(
        "rtsp_protocol",
        default_value=TextSubstitution(text="udp"),
        description="RTSP protocol: udp or tcp",
    )

    use_gl_sink_arg = DeclareLaunchArgument(
        "use_gl_sink",
        default_value=TextSubstitution(text="true"),
        description="Try glimagesink first (true) else ximagesink",
    )

    mav_name = LaunchConfiguration("mav_name")

    gui = Node(
        package="indoor_gimbal_gui",
        executable="indoor_gimbal_gui_node",
        name="indoor_gimbal_gui",
        namespace=mav_name,
        output="screen",
        parameters=[{
            "rtsp_uri": LaunchConfiguration("rtsp_uri"),
            "rtsp_protocol": LaunchConfiguration("rtsp_protocol"),
            "use_gl_sink": LaunchConfiguration("use_gl_sink"),
        }],
    )

    return LaunchDescription([
        mav_name_arg,
        rtsp_uri_arg,
        rtsp_protocol_arg,
        use_gl_sink_arg,
        gui
    ])
