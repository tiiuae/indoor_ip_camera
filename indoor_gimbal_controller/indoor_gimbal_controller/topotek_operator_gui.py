#!/usr/bin/env python3
import argparse
import socket
import sys
from dataclasses import dataclass

import cv2
from PyQt5 import QtCore, QtGui, QtWidgets


# -----------------------
# Topotek protocol helper
# -----------------------
def _crc8_sum_ascii(cmd_ascii: str) -> str:
    """
    CRC is: sum of bytes from frame header up to end of data,
    then append as 2 ASCII hex chars (uppercase). (Per doc appendix)
    """
    s = sum(cmd_ascii.encode("ascii")) & 0xFF
    return f"{s:02X}"


def build_command_ascii(frame_header: str, src: str, dst: str, length_char: str, control: str, ident3: str, data: str) -> str:
    """
    Build an ASCII command like: #TPUG2wPTZ03RR or #tpUG6wGIYEF0732RR
    - frame_header: '#TP' (fixed len=2) or '#tp' (variable)
    - length_char: single ASCII char. For '#TP' it MUST be '2'. For '#tp' it's hex digit of data length (bytes),
      BUT here our "data" is ASCII characters, so data length == len(data).
    """
    if len(frame_header) != 3 or not frame_header.startswith("#"):
        raise ValueError("frame_header must be 3 chars like '#TP' or '#tp'")
    if len(src) != 1 or len(dst) != 1:
        raise ValueError("src/dst must be single char")
    if len(ident3) != 3:
        raise ValueError("ident3 must be 3 chars")
    if control not in ("w", "r"):
        raise ValueError("control must be 'w' or 'r'")
    if len(length_char) != 1:
        raise ValueError("length_char must be 1 char")

    base = f"{frame_header}{src}{dst}{length_char}{control}{ident3}{data}"
    return base + _crc8_sum_ascii(base)


def build_ptz_command(ptz_code_hex2: str, src: str = "P", dst: str = "G") -> bytes:
    """
    PTZ is fixed header '#TP' with length '2' and data '00'..'0A' as ASCII.
    Example known-good: #TPUG2wPTZ036D
    """
    if len(ptz_code_hex2) != 2:
        raise ValueError("ptz_code_hex2 must be 2 chars like '03'")
    cmd = build_command_ascii("#TP", src, dst, "2", "w", "PTZ", ptz_code_hex2.upper())
    return cmd.encode("ascii")


def _int16_to_hex4_twos_complement(angle_i16: int) -> str:
    # Convert signed int16 to 0..65535 and format as 4 hex digits (uppercase)
    return f"{angle_i16 & 0xFFFF:04X}"


def build_gyro_angle_axis_command(axis_ident: str, angle_deg: float, speed_deg_s: float, src: str = "P", dst: str = "G") -> bytes:
    """
    Gyro angle control (GIY/GIP/GIR):
      #tpUG 6 w GIY X0X1X2X3 X4X5 RR
    In the doc the angle+speed are sent as "character type hex number" and the length is 6,
    meaning 6 ASCII characters: 4 for angle (hex) + 2 for speed (hex).【GIY/GIP/GIR section】
    """
    if axis_ident not in ("GIY", "GIP", "GIR"):
        raise ValueError("axis_ident must be GIY/GIP/GIR")

    # angle is 0.01 deg units in int16
    angle_i16 = int(round(angle_deg * 100.0))
    angle_hex4 = _int16_to_hex4_twos_complement(angle_i16)

    # speed is 0.1 deg/s units in [0,99], then represented as 1 byte shown as 2 hex chars
    speed_01 = int(round(speed_deg_s * 10.0))
    speed_01 = max(0, min(99, speed_01))
    speed_hex2 = f"{speed_01:02X}"

    data = angle_hex4 + speed_hex2  # 6 ASCII chars
    cmd = build_command_ascii("#tp", src, dst, "6", "w", axis_ident, data)
    return cmd.encode("ascii")


@dataclass
class CameraNet:
    ip: str
    tx_port: int = 9003  # Pod Port (device side):contentReference[oaicite:4]{index=4}
    src_addr: str = "P"  # network side
    dst_addr: str = "G"  # gimbal


# -------------
# Video widget
# -------------
class VideoThread(QtCore.QThread):
    frame = QtCore.pyqtSignal(QtGui.QImage)
    status = QtCore.pyqtSignal(str)

    def __init__(self, rtsp_uri: str, protocol: str, parent=None):
        super().__init__(parent)
        self.rtsp_uri = rtsp_uri
        self.protocol = protocol.lower()
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        proto = "udp" if self.protocol not in ("tcp", "udp") else self.protocol

        # OpenCV will use GStreamer if CAP_GSTREAMER is selected.
        # We terminate with appsink for frames.
        pipeline = (
            f'rtspsrc location="{self.rtsp_uri}" protocols={proto} latency=0 drop-on-latency=true ! '
            f'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
            f'appsink drop=true max-buffers=1 sync=false'
        )

        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.status.emit("ERROR: Failed to open RTSP stream (check URI / GStreamer plugins).")
            return

        self.status.emit("Video: streaming")
        while not self._stop:
            ok, bgr = cap.read()
            if not ok or bgr is None:
                self.status.emit("Video: frame read failed (RTSP hiccup?)")
                self.msleep(200)
                continue

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888).copy()
            self.frame.emit(qimg)

            self.msleep(10)  # ~100 fps max; GUI will drop naturally

        cap.release()
        self.status.emit("Video: stopped")


# -------------
# Main GUI
# -------------
class OperatorGUI(QtWidgets.QMainWindow):
    def __init__(self, cam: CameraNet, rtsp_uri: str, rtsp_proto: str):
        super().__init__()
        self.setWindowTitle("Topotek Operator GUI (RTSP + PTZ + Gyro Angle)")

        self.cam = cam
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # --- Layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # Left: video
        self.video_label = QtWidgets.QLabel("Video not started")
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(960, 540)
        self.video_label.setStyleSheet("background: #111; color: #ddd;")
        root.addWidget(self.video_label, 3)

        # Right: controls
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        self.status = QtWidgets.QLabel("Status: idle")
        right.addWidget(self.status)

        # PTZ buttons
        ptz_group = QtWidgets.QGroupBox("PTZ")
        ptz = QtWidgets.QGridLayout(ptz_group)

        btn_up = QtWidgets.QPushButton("Up")
        btn_down = QtWidgets.QPushButton("Down")
        btn_left = QtWidgets.QPushButton("Left")
        btn_right = QtWidgets.QPushButton("Right")
        btn_stop = QtWidgets.QPushButton("Stop")
        btn_home = QtWidgets.QPushButton("Home")

        ptz.addWidget(btn_up, 0, 1)
        ptz.addWidget(btn_left, 1, 0)
        ptz.addWidget(btn_stop, 1, 1)
        ptz.addWidget(btn_right, 1, 2)
        ptz.addWidget(btn_down, 2, 1)
        ptz.addWidget(btn_home, 3, 1)

        right.addWidget(ptz_group)

        btn_stop.clicked.connect(lambda: self.send_ptz("00"))
        btn_down.clicked.connect(lambda: self.send_ptz("01"))
        btn_up.clicked.connect(lambda: self.send_ptz("02"))
        btn_right.clicked.connect(lambda: self.send_ptz("03"))
        btn_left.clicked.connect(lambda: self.send_ptz("04"))
        btn_home.clicked.connect(lambda: self.send_ptz("05"))

        # Angle setpoints
        angle_group = QtWidgets.QGroupBox("Gyro Angle Setpoint (deg)")
        form = QtWidgets.QFormLayout(angle_group)

        self.yaw_in = QtWidgets.QDoubleSpinBox()
        self.yaw_in.setRange(-150.0, 150.0)
        self.yaw_in.setDecimals(2)

        self.pitch_in = QtWidgets.QDoubleSpinBox()
        self.pitch_in.setRange(-90.0, 90.0)
        self.pitch_in.setDecimals(2)

        self.roll_in = QtWidgets.QDoubleSpinBox()
        self.roll_in.setRange(-90.0, 90.0)
        self.roll_in.setDecimals(2)

        self.speed_in = QtWidgets.QDoubleSpinBox()
        self.speed_in.setRange(0.0, 9.9)
        self.speed_in.setDecimals(1)
        self.speed_in.setValue(3.2)  # deg/s default

        form.addRow("Yaw", self.yaw_in)
        form.addRow("Pitch", self.pitch_in)
        form.addRow("Roll", self.roll_in)
        form.addRow("Speed (deg/s)", self.speed_in)

        send_angles = QtWidgets.QPushButton("Send angles")
        form.addRow(send_angles)
        right.addWidget(angle_group)

        send_angles.clicked.connect(self.send_angles)

        # Quit
        quit_btn = QtWidgets.QPushButton("Quit")
        quit_btn.clicked.connect(self.close)
        right.addWidget(quit_btn)
        right.addStretch(1)

        # Video thread
        self.vt = VideoThread(rtsp_uri=rtsp_uri, protocol=rtsp_proto)
        self.vt.frame.connect(self.on_frame)
        self.vt.status.connect(self.on_status)
        self.vt.start()

    def on_status(self, s: str):
        self.status.setText(f"Status: {s}")

    def on_frame(self, qimg: QtGui.QImage):
        pix = QtGui.QPixmap.fromImage(qimg)
        self.video_label.setPixmap(pix.scaled(self.video_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def send_ptz(self, code_hex2: str):
        try:
            payload = build_ptz_command(code_hex2, src=self.cam.src_addr, dst=self.cam.dst_addr)
            self.sock.sendto(payload, (self.cam.ip, self.cam.tx_port))
            self.on_status(f"PTZ sent: {code_hex2}  ({payload.decode('ascii')})")
        except Exception as e:
            self.on_status(f"PTZ send failed: {e}")

    def send_angles(self):
        yaw = float(self.yaw_in.value())
        pitch = float(self.pitch_in.value())
        roll = float(self.roll_in.value())
        spd = float(self.speed_in.value())

        try:
            # Send per-axis gyro angle commands
            for ident, ang in (("GIY", yaw), ("GIP", pitch), ("GIR", roll)):
                payload = build_gyro_angle_axis_command(
                    axis_ident=ident,
                    angle_deg=ang,
                    speed_deg_s=spd,
                    src=self.cam.src_addr,
                    dst=self.cam.dst_addr,
                )
                self.sock.sendto(payload, (self.cam.ip, self.cam.tx_port))

            self.on_status("Angles sent (GIY/GIP/GIR)")
        except Exception as e:
            self.on_status(f"Angle send failed: {e}")

    def closeEvent(self, event):
        try:
            self.vt.stop()
            self.vt.wait(1000)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
        event.accept()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-ip", default="192.168.1.108")
    ap.add_argument("--tx-port", type=int, default=9003)
    ap.add_argument("--rtsp-uri", default="rtsp://192.168.1.108:554/stream=1")
    ap.add_argument("--rtsp-proto", default="udp", choices=["udp", "tcp"])
    ap.add_argument("--src", default="P", help="Source address (network side usually 'P')")
    ap.add_argument("--dst", default="G", help="Destination address (gimbal usually 'G')")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    cam = CameraNet(ip=args.camera_ip, tx_port=args.tx_port, src_addr=args.src, dst_addr=args.dst)
    w = OperatorGUI(cam=cam, rtsp_uri=args.rtsp_uri, rtsp_proto=args.rtsp_proto)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
