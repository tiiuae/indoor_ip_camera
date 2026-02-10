#!/usr/bin/env python3
import socket
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import String


# ----------------------------
# Topotek helpers (CONSISTENT)
# ----------------------------
def calculate_crc(cmd_bytes: bytes) -> int:
    return sum(cmd_bytes) & 0xFF


def build_command_ascii_TP(address_bit1: str, address_bit2: str,
                           control_bit: str, identifier_bit: str,
                           data_ascii: str) -> bytes:
    """
    Builds ASCII command like your known-good:
      #TPUG2wPTZ036D
    Notes:
      - For #TP length field is ASCII '2' (as in your working example).
      - data is ASCII bytes (e.g. "03")
    """
    frame_header = "#TP"
    data_bytes = data_ascii.encode("ascii")

    if len(identifier_bit) != 3:
        raise ValueError("identifier_bit must be 3 chars (e.g. PTZ)")

    length_bytes = b"2"  # per your working implementation/spec for #TP
    cmd = bytearray()
    cmd.extend(frame_header.encode("ascii"))
    cmd.extend(address_bit1.encode("ascii"))
    cmd.extend(address_bit2.encode("ascii"))
    cmd.extend(length_bytes)
    cmd.extend(control_bit.encode("ascii"))
    cmd.extend(identifier_bit.encode("ascii"))
    cmd.extend(data_bytes)
    crc = calculate_crc(cmd)
    cmd.extend(f"{crc:02X}".encode("ascii"))
    return bytes(cmd)


def build_command_ascii_tp(address_bit1: str, address_bit2: str,
                           control_bit: str, identifier_bit: str,
                           data_ascii: str) -> bytes:
    """
    Builds #tp (lowercase) commands used by GIM/GIY/etc from your PDF.

    Format (from doc):
      #tp <A1><A2> <len_nibble_ascii> <control> <ID3> <DATA_ASCII> <CRC2>
    len is 1 hex digit (0..F) representing *byte length of DATA*.
    DATA here is ASCII characters representing hex nibbles (as in examples).
    Example from your log:
      #tpUGCwGIM0000630000638C
        - len nibble 'C' => 12 bytes of DATA ("000063000063")
    """
    frame_header = "#tp"
    data_bytes = data_ascii.encode("ascii")
    if len(data_bytes) > 0x0F:
        raise ValueError("For #tp, DATA length must be <= 15 bytes")

    if len(identifier_bit) != 3:
        raise ValueError("identifier_bit must be 3 chars (e.g. GIM)")

    length_nibble = f"{len(data_bytes):X}".encode("ascii")  # one ASCII hex digit
    cmd = bytearray()
    cmd.extend(frame_header.encode("ascii"))
    cmd.extend(address_bit1.encode("ascii"))
    cmd.extend(address_bit2.encode("ascii"))
    cmd.extend(length_nibble)
    cmd.extend(control_bit.encode("ascii"))
    cmd.extend(identifier_bit.encode("ascii"))
    cmd.extend(data_bytes)
    crc = calculate_crc(cmd)
    cmd.extend(f"{crc:02X}".encode("ascii"))
    return bytes(cmd)
 
def _int16_to_hex4(value_int16: int) -> str:
    """Convert signed int16 to 4 hex chars (two's complement)."""
    v = value_int16 & 0xFFFF
    return f"{v:04X}"

def _uint8_to_hex2(value_u8: int) -> str:
    if value_u8 < 0 or value_u8 > 255:
        raise ValueError("u8 out of range")
    return f"{value_u8:02X}"

def build_gim_gyro_yaw_pitch(yaw_deg: float, pitch_deg: float,
                             speed_01dps: int) -> bytes:
    """
    Gyro angle control yaw+pitch (GIM) from your PDF:
      #tpUG C w GIM  Y0Y1Y2Y3 Y4Y5  P0P1P2P3 P4P5 RR

    - angles are signed int16 in 0.01 deg units
    - speed is 0..99 in 0.1 deg/s units (doc says 0..99)
      -> encoded as 1 byte, but represented as 2 ASCII hex chars (e.g. "63" == 99)
    """
    yaw_i16 = int(round(yaw_deg * 100.0))
    pit_i16 = int(round(pitch_deg * 100.0))

    # clamp to spec ranges (doc)
    # yaw:  [-150,150], pitch: [-90,90]
    yaw_i16 = max(int(-15000), min(int(15000), yaw_i16))
    pit_i16 = max(int(-9000),  min(int(9000),  pit_i16))

    sp = max(0, min(99, int(speed_01dps)))

    data_ascii = (
        _int16_to_hex4(pit_i16) +
        _uint8_to_hex2(sp) +
        _int16_to_hex4(yaw_i16) +
        _uint8_to_hex2(sp)
    )  # total 12 ASCII bytes

    return build_command_ascii_tp(
        address_bit1="U",   # your current working setup
        address_bit2="G",
        control_bit="w",
        identifier_bit="GIM",
        data_ascii=data_ascii
    )


def build_gir_gyro_roll(roll_deg: float, speed_01dps: int) -> bytes:
    """
    Gyro roll control (GIR) from doc:
      #tpUG 6 w GIR X0X1X2X3 X4X5 RR
    DATA = 6 ASCII bytes: angle(4) + speed(2)
    """
    rol_i16 = int(round(roll_deg * 100.0))
    rol_i16 = max(int(-9000), min(int(9000), rol_i16))
    sp = max(0, min(99, int(speed_01dps)))

    data_ascii = _int16_to_hex4(rol_i16) + _uint8_to_hex2(sp)  # 6 bytes ASCII
    return build_command_ascii_tp(
        address_bit1="U",
        address_bit2="G",
        control_bit="w",
        identifier_bit="GIR",
        data_ascii=data_ascii
    )


def build_ptz(cmd_ascii_two_chars: str) -> bytes:
    """
    Your PTZ is the known-good #TP...PTZ..
    Data is ASCII "00","01","02","03","04","05"
    """
    return build_command_ascii_TP(
        address_bit1="U",
        address_bit2="G",
        control_bit="w",
        identifier_bit="PTZ",
        data_ascii=cmd_ascii_two_chars
    )

# ----------------------------
# ROS2 Driver
# ----------------------------
@dataclass
class Params:
    camera_ip: str = "192.168.1.108"
    tx_port: int = 9003
    rx_port: int = 9004

    # topics are RELATIVE by default (so namespace works nicely)
    topic_gimbal_angles: str = "gimbal_angles_gyro_deg"   # Vector3: x=yaw, y=pitch, z=roll
    topic_ptz_cmd: str = "ptz_cmd"                        # String: left/right/up/down/stop/home

    # control settings
    speed_01dps: int = 99          # 99 => 9.9 deg/s
    send_hz: float = 30.0          # send at most this rate
    roll_enable: bool = False      # if True, also send GIR for roll when z!=0
    roll_deadband_deg: float = 0.5 # ignore tiny roll


class TopotekUdpDriver(Node):
    def __init__(self):
        super().__init__("topotek_udp_driver")

        self.declare_parameter("camera_ip", Params.camera_ip)
        self.declare_parameter("tx_port", Params.tx_port)
        self.declare_parameter("rx_port", Params.rx_port)
        self.declare_parameter("topic_gimbal_angles", Params.topic_gimbal_angles)
        self.declare_parameter("topic_ptz_cmd", Params.topic_ptz_cmd)
        self.declare_parameter("speed_01dps", Params.speed_01dps)
        self.declare_parameter("send_hz", Params.send_hz)
        self.declare_parameter("roll_enable", Params.roll_enable)
        self.declare_parameter("roll_deadband_deg", Params.roll_deadband_deg)

        self.p = Params(
            camera_ip=str(self.get_parameter("camera_ip").value),
            tx_port=int(self.get_parameter("tx_port").value),
            rx_port=int(self.get_parameter("rx_port").value),
            topic_gimbal_angles=str(self.get_parameter("topic_gimbal_angles").value),
            topic_ptz_cmd=str(self.get_parameter("topic_ptz_cmd").value),
            speed_01dps=int(self.get_parameter("speed_01dps").value),
            send_hz=float(self.get_parameter("send_hz").value),
            roll_enable=bool(self.get_parameter("roll_enable").value),
            roll_deadband_deg=float(self.get_parameter("roll_deadband_deg").value),
        )

        self._lock = threading.Lock()
        self._last_angles: Vector3 | None = None
        self._last_angles_time = 0.0
        self._last_angles_sent = False
        self._speed_deg_s = 70.0

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.p.rx_port))
        self.sock.setblocking(False)

        self.get_logger().info(
            f"Driver up. TX->{self.p.camera_ip}:{self.p.tx_port}, RX->0.0.0.0:{self.p.rx_port}"
        )
        self.get_logger().info(
            f"Topics (relative): {self.p.topic_gimbal_angles} (Vector3), {self.p.topic_ptz_cmd} (String)"
        )

        # subs
        self.sub_angles = self.create_subscription(Vector3, self.p.topic_gimbal_angles, self._on_angles, 10)
        self.sub_ptz = self.create_subscription(String, self.p.topic_ptz_cmd, self._on_ptz, 10)

        # timer tick
        period = 1.0 / max(1.0, self.p.send_hz)
        self.timer = self.create_timer(period, self._tick)

        self._last_warn = 0.0

    def _on_angles(self, msg: Vector3):
        with self._lock:
            self._last_angles = msg
            self._last_angles_time = time.time()
            self._last_angles_sent = False
        self.get_logger().info(f"Angles received (deg) yaw={msg.x:.2f} pitch={msg.y:.2f} roll={msg.z:.2f}")
        yaw = float(msg.x)
        pitch = float(msg.y)
        roll = float(msg.z)

        try:

            payload_yaw_pitch = build_gim_gyro_yaw_pitch(yaw_deg=yaw, pitch_deg=pitch, speed_01dps=self._speed_deg_s)
            self.get_logger().info(f"Sending out GIM angle message.")
            self.sock.sendto(payload_yaw_pitch, (self.p.camera_ip, self.p.tx_port))

            payload_roll = build_gir_gyro_roll(roll_deg=roll, speed_01dps=self._speed_deg_s)
            self.get_logger().info(f"Sending out GIR angle message.")
            self.sock.sendto(payload_roll, (self.p.camera_ip, self.p.tx_port))

        except Exception as e:
            self.get_logger().info(f"Angle send failed: {e}")


    def _on_ptz(self, msg: String):
        s = (msg.data or "").strip().lower()
        mapping = {
            "stop": "00",
            "up": "01",
            "down": "02",
            "left": "03",
            "right": "04",
            "home": "05",
        }
        if s not in mapping:
            self.get_logger().warn(f"Unknown PTZ cmd '{msg.data}'. Use one of: {list(mapping.keys())}")
            return

        payload = build_ptz(mapping[s])
        self.sock.sendto(payload, (self.p.camera_ip, self.p.tx_port))
        try:
            self.get_logger().info(f"Sent PTZ '{s}' -> {payload.decode('ascii', errors='replace')}")
        except Exception:
            self.get_logger().info(f"Sent PTZ '{s}' (bytes={payload!r})")

    def _tick(self):
        # read latest
        with self._lock:
            angles = self._last_angles

        if angles is None:
            now = time.time()
            if now - self._last_warn > 2.0:
                self._last_warn = now
                self.get_logger().warn("No gimbal angles received yet -> not sending GIM.")
            return

def main():
    rclpy.init()
    node = TopotekUdpDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
