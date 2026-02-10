#!/usr/bin/env python3
import sys
import argparse
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# GUI
from PyQt5 import QtCore, QtGui, QtWidgets

# Video
import numpy as np
import cv2

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst


def build_gst_pipeline(rtsp_uri: str, rtsp_proto: str) -> str:
    # Low-latency RTSP -> H264 depay -> decode -> appsink (drop old buffers)
    # This mirrors your known-good gst-launch pipeline but ends with appsink.
    proto = rtsp_proto.lower()
    if proto not in ("udp", "tcp"):
        proto = "udp"

    # rtspsrc protocols property is a bitmask internally, but accepts "udp"/"tcp" in gst-launch.
    # In parse_launch, we can set protocols=udp / tcp the same way.
    return (
        f'rtspsrc location="{rtsp_uri}" protocols={proto} latency=0 drop-on-latency=true ! '
        f'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
        f'queue max-size-buffers=1 leaky=downstream ! '
        f'appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false'
    )


class RosBridge(Node):
    """
    Pure ROS publisher node (no GUI here).
    """
    def __init__(self):
        super().__init__("topotek_operator_gui_ros")

        # Params (match driver defaults: relative topics inside namespace)
        self.declare_parameter("topic_ptz_cmd", "ptz_cmd")
        self.declare_parameter("topic_gimbal_angles", "gimbal_angles_gyro_deg")

        self.declare_parameter("rtsp_uri", "rtsp://192.168.1.108:554/stream=1")
        self.declare_parameter("rtsp_protocol", "udp")

        self.topic_ptz_cmd = self.get_parameter("topic_ptz_cmd").get_parameter_value().string_value
        self.topic_gimbal_angles = self.get_parameter("topic_gimbal_angles").get_parameter_value().string_value

        self.rtsp_uri = self.get_parameter("rtsp_uri").get_parameter_value().string_value
        self.rtsp_protocol = self.get_parameter("rtsp_protocol").get_parameter_value().string_value

        self.pub_ptz = self.create_publisher(String, self.topic_ptz_cmd, 10)
        self.pub_angles = self.create_publisher(Vector3, self.topic_gimbal_angles, 10)

        self.get_logger().info(
            f"GUI ROS bridge up. Publishing PTZ -> '{self.topic_ptz_cmd}', "
            f"Angles -> '{self.topic_gimbal_angles}'. RTSP={self.rtsp_uri} proto={self.rtsp_protocol}"
        )

    def send_ptz(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.pub_ptz.publish(msg)
        self.get_logger().info(f"PTZ published: {cmd}")

    def send_angles_deg(self, yaw_deg: float, pitch_deg: float, roll_deg: float = 0.0):
        # IMPORTANT: keep consistent with what YOU said works:
        # ros2 topic pub --once /gimbal_angles_gyro_deg Vector3 "{x: 10.0, y: -20.0, z: 0.0}" works.
        # So we map x=yaw, y=pitch, z=roll (or 0).
        msg = Vector3()
        msg.x = float(yaw_deg)
        msg.y = float(pitch_deg)
        msg.z = float(roll_deg)
        self.pub_angles.publish(msg)
        self.get_logger().info(f"Angles published (deg): yaw={msg.x:.2f} pitch={msg.y:.2f} roll={msg.z:.2f}")


class OperatorGui(QtWidgets.QMainWindow):
    def __init__(self, ros_node: RosBridge):
        super().__init__()
        self.ros = ros_node

        self.setWindowTitle("Topotek Operator Station (RTSP + PTZ + Angles)")
        self.resize(1100, 650)

        # --- Video state ---
        self._frame_lock = threading.Lock()
        self._latest_bgr = None
        self._last_frame_time = 0.0

        # --- Build UI ---
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        main = QtWidgets.QHBoxLayout(central)

        # Video panel
        self.video_label = QtWidgets.QLabel("Starting video…")
        self.video_label.setMinimumSize(800, 450)
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #111; color: #ddd;")
        main.addWidget(self.video_label, stretch=3)

        # Controls panel
        right = QtWidgets.QVBoxLayout()
        main.addLayout(right, stretch=1)

        # PTZ buttons
        ptz_group = QtWidgets.QGroupBox("PTZ")
        ptz_layout = QtWidgets.QGridLayout(ptz_group)

        btn_up = QtWidgets.QPushButton("Up")
        btn_down = QtWidgets.QPushButton("Down")
        btn_left = QtWidgets.QPushButton("Left")
        btn_right = QtWidgets.QPushButton("Right")
        btn_stop = QtWidgets.QPushButton("Stop")
        btn_home = QtWidgets.QPushButton("Home")

        ptz_layout.addWidget(btn_up, 0, 1)
        ptz_layout.addWidget(btn_left, 1, 0)
        ptz_layout.addWidget(btn_stop, 1, 1)
        ptz_layout.addWidget(btn_right, 1, 2)
        ptz_layout.addWidget(btn_down, 2, 1)
        ptz_layout.addWidget(btn_home, 3, 1)

        right.addWidget(ptz_group)

        # Angles
        ang_group = QtWidgets.QGroupBox("Set Gimbal Angles (deg)")
        form = QtWidgets.QFormLayout(ang_group)

        self.edit_yaw = QtWidgets.QDoubleSpinBox()
        self.edit_yaw.setRange(-180.0, 180.0)
        self.edit_yaw.setDecimals(2)
        self.edit_yaw.setSingleStep(1.0)

        self.edit_pitch = QtWidgets.QDoubleSpinBox()
        self.edit_pitch.setRange(-90.0, 90.0)
        self.edit_pitch.setDecimals(2)
        self.edit_pitch.setSingleStep(1.0)

        self.edit_roll = QtWidgets.QDoubleSpinBox()
        self.edit_roll.setRange(-180.0, 180.0)
        self.edit_roll.setDecimals(2)
        self.edit_roll.setSingleStep(1.0)
        self.edit_roll.setValue(0.0)

        self.btn_send_angles = QtWidgets.QPushButton("Send angles")

        form.addRow("Yaw (x):", self.edit_yaw)
        form.addRow("Pitch (y):", self.edit_pitch)
        form.addRow("Roll (z):", self.edit_roll)
        form.addRow(self.btn_send_angles)

        right.addWidget(ang_group)

        # Status
        self.status = QtWidgets.QLabel("Status: initializing…")
        self.status.setWordWrap(True)
        right.addWidget(self.status)
        right.addStretch(1)

        # --- Wire actions ---
        btn_up.clicked.connect(lambda: self.ros.send_ptz("down"))
        btn_down.clicked.connect(lambda: self.ros.send_ptz("up"))
        btn_left.clicked.connect(lambda: self.ros.send_ptz("right"))
        btn_right.clicked.connect(lambda: self.ros.send_ptz("left"))
        btn_stop.clicked.connect(lambda: self.ros.send_ptz("stop"))
        btn_home.clicked.connect(lambda: self.ros.send_ptz("home"))

        self.btn_send_angles.clicked.connect(self._on_send_angles)

        # --- Start GStreamer ---
        Gst.init(None)
        pipeline_str = build_gst_pipeline(self.ros.rtsp_uri, self.ros.rtsp_protocol)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        if self.appsink is None:
            self.status.setText("Status: ERROR - appsink not found in pipeline.")
            raise RuntimeError("appsink not found")

        self.appsink.connect("new-sample", self._on_new_sample)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.status.setText("Status: ERROR - GStreamer failed to start.")
            raise RuntimeError("GStreamer pipeline failed to start")

        # UI timer to repaint video at ~30Hz
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_video)
        self.timer.start(33)

        self.status.setText(f"Status: RTSP playing ({self.ros.rtsp_protocol})")

    def closeEvent(self, event):
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().closeEvent(event)

    def _on_send_angles(self):
        yaw = self.edit_yaw.value()
        pitch = self.edit_pitch.value()
        roll = self.edit_roll.value()
        self.ros.send_angles_deg(yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll)

    def _on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)

        width = s.get_value("width")
        height = s.get_value("height")
        fmt = s.get_value("format") if s.has_field("format") else None

        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK

        try:
            data = map_info.data
            # Most decoders output I420 or NV12; handle common cases.
            if fmt in ("I420", "YV12"):
                yuv = np.frombuffer(data, dtype=np.uint8).reshape((height * 3 // 2, width))
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            elif fmt == "NV12":
                yuv = np.frombuffer(data, dtype=np.uint8).reshape((height * 3 // 2, width))
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            elif fmt in ("BGR", "BGRx"):
                # BGRx is 4 channels; drop alpha if present
                arr = np.frombuffer(data, dtype=np.uint8)
                if fmt == "BGRx":
                    arr = arr.reshape((height, width, 4))[:, :, :3]
                else:
                    arr = arr.reshape((height, width, 3))
                bgr = arr
            elif fmt in ("RGB", "RGBx"):
                arr = np.frombuffer(data, dtype=np.uint8)
                if fmt == "RGBx":
                    arr = arr.reshape((height, width, 4))[:, :, :3]
                else:
                    arr = arr.reshape((height, width, 3))
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            else:
                # Fallback: try to interpret as BGR
                arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, -1))
                bgr = arr[:, :, :3]

            with self._frame_lock:
                self._latest_bgr = bgr.copy()
                self._last_frame_time = time.time()
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _update_video(self):
        with self._frame_lock:
            frame = None if self._latest_bgr is None else self._latest_bgr.copy()
            last_t = self._last_frame_time

        if frame is None:
            return

        # Convert to QImage
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)

        # Fit into label
        self.video_label.setPixmap(pix.scaled(
            self.video_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation
        ))

        age_ms = (time.time() - last_t) * 1000.0
        self.status.setText(f"Status: live (frame age ~{age_ms:.0f} ms)")



def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]

    # parse only non-ROS args here
    parser = argparse.ArgumentParser(add_help=True)
    _known, unknown = parser.parse_known_args(argv)

    # rclpy must consume ROS args
    rclpy.init(args=unknown)
    ros_node = RosBridge()

    # spin ROS in background so Qt event loop can run
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = OperatorGui(ros_node)
    win.show()

    code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
