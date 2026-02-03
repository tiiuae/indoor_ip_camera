#!/usr/bin/env python3
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


def crc8_sum_ascii(cmd_bytes: bytes) -> bytes:
    crc = sum(cmd_bytes) & 0xFF
    return f"{crc:02X}".encode("ascii")


def build_command_bytes_tp(
    frame_header: str,      # "#TP" or "#tp"
    src: str,               # 'P','U',...
    dst: str,               # 'G','D',...
    control: str,           # 'w' or 'r'
    ident3: str,            # 'PTZ','UAV',...
    data_bytes: bytes,
    # For "#TP" fixed-length, length is always "2" (ASCII '2') in this protocol doc.
    # For "#tp", length is 1 nibble ASCII hex (0..F) = len(data_bytes).
) -> bytes:
    assert frame_header in ("#TP", "#tp")
    assert len(src) == 1 and len(dst) == 1
    assert control in ("w", "r")
    assert len(ident3) == 3

    if frame_header == "#TP":
        # fixed length command: data length is 2 (ASCII '2')
        if len(data_bytes) != 2:
            raise ValueError("For #TP, data must be exactly 2 bytes (x1x2).")
        length_field = b"2"
    else:
        # variable length: 0..F encoded as ASCII hex
        if len(data_bytes) > 0x0F:
            raise ValueError("For #tp, max payload length is 15 bytes.")
        length_field = f"{len(data_bytes):X}".encode("ascii")

    base = bytearray()
    base += frame_header.encode("ascii")
    base += src.encode("ascii")
    base += dst.encode("ascii")
    base += length_field
    base += control.encode("ascii")
    base += ident3.encode("ascii")
    base += data_bytes

    base += crc8_sum_ascii(bytes(base))
    return bytes(base)


class TopotekUdpDriver(Node):
    """
    Lowest layer UDP driver for Topotek SIP-series:
    - PTZ
    - gyro angle control (yaw+pitch)
    - UAV attitude send
    """

    def __init__(self):
        super().__init__("topotek_udp_driver")

        # ---- Parameters ----
        self.declare_parameter("camera_ip", "192.168.1.108")
        self.declare_parameter("tx_port", 9003)
        self.declare_parameter("rx_port", 9004)

        # address bits: source 'P' (network side), destinations: 'G' gimbal, 'D' system/image
        self.declare_parameter("src_addr", "P")

        # UAV periodic send
        self.declare_parameter("uav_send_hz", 30.0)
        self.declare_parameter("uav_speed_ms", 0.0)
        self.declare_parameter("uav_path_angle_deg", 0.0)

        # gyro angle control defaults
        self.declare_parameter("angle_speed_dps", 30.0)  # deg/s

        self.camera_ip = self.get_parameter("camera_ip").get_parameter_value().string_value
        self.tx_port = self.get_parameter("tx_port").get_parameter_value().integer_value
        self.rx_port = self.get_parameter("rx_port").get_parameter_value().integer_value
        self.src_addr = self.get_parameter("src_addr").get_parameter_value().string_value

        self.uav_send_hz = self.get_parameter("uav_send_hz").get_parameter_value().double_value
        self.uav_speed_ms = self.get_parameter("uav_speed_ms").get_parameter_value().double_value
        self.uav_path_angle_deg = self.get_parameter("uav_path_angle_deg").get_parameter_value().double_value
        self.angle_speed_dps = self.get_parameter("angle_speed_dps").get_parameter_value().double_value

        # ---- UDP socket ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Optional RX listener (useful later for feedback)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(("0.0.0.0", int(self.rx_port)))
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ---- State for UAV attitude ----
        self._lock = threading.Lock()
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # ---- ROS I/O ----
        self.sub_rpy = self.create_subscription(Vector3, "vehicle_rpy_deg", self._on_rpy, 10)

        # PTZ control topic: publish "left/right/up/down/stop/home"
        self.sub_ptz = self.create_subscription(String, "ptz_cmd", self._on_ptz, 10)

        # Angle topic: publish Vector3 {x:yaw_deg, y:pitch_deg, z:unused}
        self.sub_angles = self.create_subscription(Vector3, "gimbal_angles_gyro_deg", self._on_angles_gyro, 10)

        # Periodic UAV send
        period = 1.0 / max(1.0, float(self.uav_send_hz))
        self.timer = self.create_timer(period, self._tick_uav)

        self.get_logger().info(
            f"Driver up. TX->{self.camera_ip}:{self.tx_port}, RX->0.0.0.0:{self.rx_port}, src_addr={self.src_addr}"
        )
        self.get_logger().info("Topics: /vehicle_rpy_deg (Vector3), /ptz_cmd (String), /gimbal_angles_gyro_deg (Vector3)")

    # ---------- RX ----------
    def _rx_loop(self):
        while self._rx_running:
            try:
                data, addr = self.rx_sock.recvfrom(4096)
                # keep it minimal for now
                self.get_logger().debug(f"RX {addr}: {data!r}")
            except Exception:
                break

    # ---------- Callbacks ----------
    def _on_rpy(self, msg: Vector3):
        with self._lock:
            self._roll = float(msg.x)
            self._pitch = float(msg.y)
            self._yaw = float(msg.z)
        self.get_logger().info(f"UAV RPY updated (deg): roll={self._roll:.2f}, pitch={self._pitch:.2f}, yaw={self._yaw:.2f}")

    def _on_ptz(self, msg: String):
        cmd = msg.data.strip().lower()
        mapping = {
            "stop": "00",
            "up": "01",
            "down": "02",
            "left": "03",
            "right": "04",
            "home": "05",
        }
        if cmd not in mapping:
            self.get_logger().warn(f"Unknown PTZ '{cmd}'. Use one of: {list(mapping.keys())}")
            return

        # PTZ is Class G, destination 'G'
        payload = mapping[cmd].encode("ascii")  # x1x2
        pkt = build_command_bytes_tp(
            frame_header="#TP",
            src=self.src_addr,   # use 'P' when sending over network
            dst="G",
            control="w",
            ident3="PTZ",
            data_bytes=payload,
        )
        self.sock.sendto(pkt, (self.camera_ip, int(self.tx_port)))
        self.get_logger().info(f"Sent PTZ {cmd} -> {pkt.decode('ascii', errors='replace')}")

    def _on_angles_gyro(self, msg: Vector3):
        # Gyro angle control: GIM (yaw+pitch), destination 'G'
        yaw = float(msg.x)
        pitch = float(msg.y)

        # angle: int16, unit 0.01 deg
        y_i16 = int(round(yaw * 100.0))
        p_i16 = int(round(pitch * 100.0))

        # speed: uint8, unit 0.1 deg/s (doc shows 0..99) => clamp to [0..99]
        sp = int(round(self.angle_speed_dps * 10.0))
        sp = max(0, min(99, sp))

        # Data is: Y0Y1Y2Y3 Y4Y5  P0P1P2P3 P4P5
        # Where angles are signed 16-bit shown as HEX chars, and speed is hex as chars.
        # We’ll format exactly as ASCII hex characters as in examples.
        y_hex = (y_i16 & 0xFFFF)
        p_hex = (p_i16 & 0xFFFF)
        y_str = f"{y_hex:04X}{sp:02X}"
        p_str = f"{p_hex:04X}{sp:02X}"
        data_ascii = (y_str + p_str).encode("ascii")  # total 12 bytes => use #tp length 'C'

        pkt = build_command_bytes_tp(
            frame_header="#tp",
            src=self.src_addr,
            dst="G",
            control="w",
            ident3="GIM",
            data_bytes=data_ascii,
        )
        self.sock.sendto(pkt, (self.camera_ip, int(self.tx_port)))
        self.get_logger().info(f"Sent GIM yaw={yaw:.2f} pitch={pitch:.2f} sp={sp/10.0:.1f}dps -> {pkt.decode('ascii', errors='replace')}")

    # ---------- Periodic UAV send ----------
    def _tick_uav(self):
        with self._lock:
            roll = self._roll
            pitch = self._pitch
            yaw = self._yaw

        # Doc struct says: int16 yaw, pitch, roll in 0.01 deg; speed 0.01 m/s; path angle 0.01 deg :contentReference[oaicite:11]{index=11} (page 20)
        yaw_i16 = int(round(yaw * 100.0))
        pit_i16 = int(round(pitch * 100.0))
        rol_i16 = int(round(roll * 100.0))
        spd_u16 = int(round(float(self.uav_speed_ms) * 100.0)) & 0xFFFF
        path_u16 = int(round(float(self.uav_path_angle_deg) * 100.0)) & 0xFFFF

        data_bytes = struct.pack("<hhhHH", yaw_i16, pit_i16, rol_i16, spd_u16, path_u16)

        pkt = build_command_bytes_tp(
            frame_header="#tp",
            src=self.src_addr,
            dst="D",          # UAV attitude is sent to D per struct in doc
            control="w",
            ident3="UAV",
            data_bytes=data_bytes,
        )
        self.sock.sendto(pkt, (self.camera_ip, int(self.tx_port)))
        self.get_logger().debug(f"Sent UAV -> {pkt!r}")

    def destroy_node(self):
        self._rx_running = False
        try:
            self.rx_sock.close()
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TopotekUdpDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
