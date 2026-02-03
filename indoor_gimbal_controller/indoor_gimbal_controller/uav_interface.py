#!/usr/bin/env python3
import struct
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from .topotekcmdparse import build_command  # (better than import *)


def build_uav_command_bytes(
    azimuth_deg: float,
    pitch_deg: float,
    roll_deg: float,
    speed_ms: float,
    path_angle_deg: float,
    address1: str = 'P',   # network
    address2: str = 'D'
) -> bytes:
    # Scale
    azi_val  = int(round(azimuth_deg    * 100))
    pit_val  = int(round(pitch_deg      * 100))
    rol_val  = int(round(roll_deg       * 100))
    spd_val  = int(round(speed_ms       * 100))
    path_val = int(round(path_angle_deg * 100))

    data_bytes = struct.pack('<hhhHH', azi_val, pit_val, rol_val, spd_val, path_val)
    data_hex = ' '.join(f'{b:02X}' for b in data_bytes)

    cmd_hex = build_command(
        frame_header='#tp',
        address_bit1=address1,
        address_bit2=address2,
        control_bit='w',
        identifier_bit='UAV',
        data=data_hex,
        data_mode='Hex',
        input_space_separate=True,
        output_format='Hex',
        output_space_separate=True
    )

    # cmd_hex is "AA BB CC ..." -> bytes
    return bytes.fromhex(cmd_hex.replace(" ", ""))


class TopotekUavLink(Node):
    def __init__(self):
        super().__init__("topotek_uav_link")

        self.declare_parameter("camera_ip", "192.168.1.108")
        self.declare_parameter("tx_port", 9003)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("address1", "P")
        self.declare_parameter("address2", "D")
        self.declare_parameter("speed_ms", 0.0)
        self.declare_parameter("path_angle_deg", 0.0)

        self.camera_ip = self.get_parameter("camera_ip").value
        self.tx_port = int(self.get_parameter("tx_port").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.addr1 = self.get_parameter("address1").value
        self.addr2 = self.get_parameter("address2").value

        # If you don't have speed/path yet, keep constants for now
        self.speed_ms = float(self.get_parameter("speed_ms").value)
        self.path_angle_deg = float(self.get_parameter("path_angle_deg").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # latest state (degrees)
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._lock = threading.Lock()

        self.create_subscription(Vector3, "vehicle_rpy_deg", self._on_rpy, 10)

        period = 1.0 / max(self.rate_hz, 1e-3)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Sending UAV packets to {self.camera_ip}:{self.tx_port} @ {self.rate_hz} Hz (addr {self.addr1}->{self.addr2})"
        )

    def _on_rpy(self, msg: Vector3):
        with self._lock:
            self._roll = float(msg.x)
            self._pitch = float(msg.y)
            self._yaw = float(msg.z)
        print("UAV Command received")

    def _tick(self):
        with self._lock:
            roll = self._roll
            pitch = self._pitch
            yaw = self._yaw

        try:
            payload = build_uav_command_bytes(
                azimuth_deg=yaw,
                pitch_deg=pitch,
                roll_deg=roll,
                speed_ms=self.speed_ms,
                path_angle_deg=self.path_angle_deg,
                address1=self.addr1,
                address2=self.addr2,
            )
            self.sock.sendto(payload, (self.camera_ip, self.tx_port))
        except Exception as e:
            self.get_logger().warn(f"UAV send failed: {e}")


def main():
    rclpy.init()
    node = TopotekUavLink()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
