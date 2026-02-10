#!/usr/bin/env bash
set -e  # NO -u

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

MAV_NAME="${MAV_NAME:-uav60}"
WS="${WS:-$HOME/indoor_uav/ros_ws}"

echo "[RUN] MAV_NAME=$MAV_NAME"
echo "[RUN] WS=$WS"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export MAV_NAME="$MAV_NAME"

# 1) Driver
echo "[RUN] Launching driver..."
ros2 launch indoor_gimbal_controller gimbal_driver.launch.py mav_name:="$MAV_NAME" &
PID_GIMBAL=$!

# 2) Start GStreamer pipeline
echo "[RUN] Launching Gstreamer pipeline..."
ros2 launch indoor_camera rtsp_camera.launch.py &
PID_VISION=$!

cleanup() {
  echo "[RUN] Cleanup..."
  kill -TERM "$PID_GIMBAL" 2>/dev/null || true
  kill -TERM "$PID_VISION" 2>/dev/null || true
  wait || true
}
trap cleanup INT TERM EXIT

wait
