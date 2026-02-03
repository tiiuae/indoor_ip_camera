import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _load_rtsp_defaults():
    pkg_share = get_package_share_directory("indoor_camera")
    config_dir = os.path.join(pkg_share, "config")
    yaml_path = os.path.join(config_dir, "rtsp_camera.yaml")

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}

    # Adjust this key if your yaml uses a different root name
    params = (data.get("indoor_camera", {}) or {}).get("ros__parameters", {}) or {}

    enable_gst = params.get("enable_gst", True)
    rtsp_uri = params.get("rtsp_uri", "rtsp://192.168.1.108:554/stream=1")
    rtsp_protocol = params.get("rtsp_protocol", "udp")

    enable_gst_str = "true" if bool(enable_gst) else "false"
    return yaml_path, params, enable_gst_str, rtsp_uri, rtsp_protocol


def generate_launch_description():
    yaml_path, yaml_params, default_enable_gst, default_rtsp_uri, default_rtsp_protocol = _load_rtsp_defaults()

    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value=TextSubstitution(text=os.environ.get("MAV_NAME", "")),
        description="UAV namespace (default: env MAV_NAME). Example: uav60",
    )

    enable_gst_arg = DeclareLaunchArgument(
        "enable_gst",
        default_value=TextSubstitution(text=default_enable_gst),
        description=f"Start low-latency GStreamer operator viewer. Default from {os.path.basename(yaml_path)}",
    )

    rtsp_uri_arg = DeclareLaunchArgument(
        "rtsp_uri",
        default_value=TextSubstitution(text=default_rtsp_uri),
        description=f"RTSP URI for the camera. Default from {os.path.basename(yaml_path)}",
    )

    rtsp_protocol_arg = DeclareLaunchArgument(
        "rtsp_protocol",
        default_value=TextSubstitution(text=default_rtsp_protocol),
        description=f"RTSP transport protocol: udp or tcp. Default from {os.path.basename(yaml_path)}",
    )

    mav_name = LaunchConfiguration("mav_name")
    enable_gst = LaunchConfiguration("enable_gst")
    rtsp_uri = LaunchConfiguration("rtsp_uri")
    rtsp_protocol = LaunchConfiguration("rtsp_protocol")

    # ---- ROS params (publisher node) ----
    node_params = dict(yaml_params)
    node_params["rtsp_uri"] = rtsp_uri  # override from launch arg

    pkg_share = get_package_share_directory("indoor_camera")
    camera_info_url = "file://" + os.path.join(pkg_share, "config", "camera_info.yaml")

    indoor_camera_node = ComposableNode(
        package="indoor_camera",
        plugin="indoor_camera::IpCamera",  # <-- IMPORTANT: match your component class
        name="ipcamera",
        namespace=mav_name,
        parameters=[
            node_params,
            {"camera_calibration_file": camera_info_url},
        ],
    )

    container = ComposableNodeContainer(
        name="container",
        namespace="indoor_camera_container",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[indoor_camera_node],
        output="screen",
    )

    # ---- GStreamer viewer ----
    viewer_cmd = [
        "gst-launch-1.0", "-v",
        "rtspsrc",
        "location=",
        rtsp_uri,
        "protocols=",
        rtsp_protocol,
        "latency=0",
        "drop-on-latency=true",
        "!",
        "rtph264depay", "!", "h264parse", "!", "avdec_h264", "!", "videoconvert", "!",
        "queue", "max-size-buffers=1", "leaky=downstream", "!",
        "autovideosink", "sync=false",
    ]

    # BUT gst-launch expects "location=xxx" as one token, not ("location=", uri).
    # So we build the correct tokens like "location=<uri>" and "protocols=<proto>".
    viewer_cmd = [
        "gst-launch-1.0", "-v",
        "rtspsrc",
        ["location=", rtsp_uri],  # placeholder, will be flattened below
    ]

    # Properly build as substitutions (single tokens):
    viewer_cmd = [
        "gst-launch-1.0", "-v",
        "rtspsrc",
        # each of these becomes a single argv token:
        ["location=", rtsp_uri],
        ["protocols=", rtsp_protocol],
        "latency=0",
        "drop-on-latency=true",
        "!",
        "rtph264depay", "!", "h264parse", "!", "avdec_h264", "!", "videoconvert", "!",
        "queue", "max-size-buffers=1", "leaky=downstream", "!",
        "autovideosink", "sync=false",
    ]

    # Flatten nested lists to keep launch substitutions intact:
    flattened = []
    for t in viewer_cmd:
        if isinstance(t, list):
            flattened.append(t[0])
            flattened.append(t[1])
        else:
            flattened.append(t)

    viewer_log = LogInfo(msg=["Starting GStreamer viewer with RTSP URI: ", rtsp_uri, " (", rtsp_protocol, ")"])
    viewer = ExecuteProcess(
        cmd=flattened,
        output="screen",
        condition=IfCondition(enable_gst),
    )

    return LaunchDescription([
        mav_name_arg,
        enable_gst_arg,
        rtsp_uri_arg,
        rtsp_protocol_arg,
        viewer_log,
        viewer,
        container,
    ])
