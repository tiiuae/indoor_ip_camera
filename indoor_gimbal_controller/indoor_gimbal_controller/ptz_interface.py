#!/usr/bin/env python3
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool

from .topotekcmdparse import build_command


PTZ_MAP = {
    "stop": "00",
    "up": "01",
    "down": "02",
    "left": "03",
    "right": "04",
    "home": "05",
    "00": "00",
    "01": "01",
    "02": "02",
    "03": "03",
    "04": "04",
    "05": "05",
}


class TopotekGimbalUdp(Node):
    """
    Lowest-layer Topotek gimbal control over UDP.
    Sends ASCII commands like: #TPUG2wPTZ036D
    """

    def __init__(self):
        super().__init__("topotek_gimbal_udp")

        # ---- Params ----
        self.declare_parameter("camera_ip", "192.168.1.108")
        self.declare_parameter("tx_port", 9003)      # control port
        self.declare_parameter("rx_port", 9004)      # optional feedback port
        self.declare_parameter("addr1", "U")         # matches your known-good command
        self.declare_parameter("addr2", "G")         # gimbal
        self.declare_parameter("enable_rx", True)
        self.declare_parameter("repeat_hz", 0.0)     # 0 = send once; >0 = keep sending last cmd
        self.declare_parameter("auto_stop_ms", 0)    # 0 = never auto-stop; else send stop after X ms

        self.camera_ip = str(self.get_parameter("camera_ip").value)
        self.tx_port = int(self.get_parameter("tx_port").value)
        self.rx_port = int(self.get_parameter("rx_port").value)
        self.addr1 = str(self.get_parameter("addr1").value)
        self.addr2 = str(self.get_parameter("addr2").value)
        self.enable_rx = bool(self.get_parameter("enable_rx").value)
        self.repeat_hz = float(self.get_parameter("repeat_hz").value)
        self.auto_stop_ms = int(self.get_parameter("auto_stop_ms").value)

        # ---- UDP sockets ----
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.rx_sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_running = False

        if self.enable_rx:
            try:
                self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.rx_sock.bind(("0.0.0.0", self.rx_port))
                self._rx_running = True
                self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self._rx_thread.start()
                self.get_logger().info(f"Listening for camera replies on UDP :{self.rx_port}")
            except Exception as e:
                self.get_logger().warn(f"RX disabled (bind failed): {e}")
                self.enable_rx = False

        # ---- State ----
        self._lock = threading.Lock()
        self._last_cmd_ascii: Optional[str] = None
        self._last_cmd_time = 0.0

        # ---- ROS interfaces ----
        # Topic: send PTZ via string (up/down/left/right/stop/home or 00..05)
        self.sub = self.create_subscription(String, "ptz_cmd", self._on_ptz_cmd, 10)

        # Service: tiny “demo” service (true -> left, false -> stop)
        self.srv = self.create_service(SetBool, "ptz_demo", self._on_ptz_demo)

        # Timer for repeating last command (if repeat_hz > 0)
        self._repeat_timer = None
        if self.repeat_hz > 0.0:
            period = 1.0 / self.repeat_hz
            self._repeat_timer = self.create_timer(period, self._repeat_tick)
            self.get_logger().info(f"Repeat enabled: {self.repeat_hz} Hz")

        # Timer for auto-stop (if auto_stop_ms > 0)
        self._auto_stop_timer = None
        if self.auto_stop_ms > 0:
            self._auto_stop_timer = self.create_timer(0.02, self._auto_stop_tick)  # 50 Hz check
            self.get_logger().info(f"Auto-stop enabled: {self.auto_stop_ms} ms after any non-stop command")

        self.get_logger().info(
            f"Ready. TX -> {self.camera_ip}:{self.tx_port} addr={self.addr1}{self.addr2}"
        )
        self.get_logger().info("Publish on ~ptz_cmd: stop|up|down|left|right|home (or 00..05)")

    # ---------------- ROS callbacks ----------------

    def _on_ptz_cmd(self, msg: String):
        key = (msg.data or "").strip().lower()
        if key not in PTZ_MAP:
            self.get_logger().warn(f"Unknown PTZ '{key}'. Valid: {sorted(set(PTZ_MAP.keys()))}")
            return
        code = PTZ_MAP[key]
        self.send_ptz(code)

    def _on_ptz_demo(self, request: SetBool.Request, response: SetBool.Response):
        # Just a convenience: True => left, False => stop
        try:
            if request.data:
                self.send_ptz("03")  # left
                response.message = "Sent LEFT"
            else:
                self.send_ptz("00")  # stop
                response.message = "Sent STOP"
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    # ---------------- Core send logic ----------------

    def build_ptz_cmd_ascii(self, code_two_chars: str) -> str:
        # code is ASCII "00".."05" (NOT binary)
        cmd = build_command(
            frame_header="#TP",
            address_bit1=self.addr1,
            address_bit2=self.addr2,
            control_bit="w",
            identifier_bit="PTZ",
            data=code_two_chars,
            data_mode="ASCII",
            output_format="ASCII",
        )
        return cmd

    def send_raw_ascii(self, cmd_ascii: str):
        payload = cmd_ascii.encode("ascii")
        self.tx_sock.sendto(payload, (self.camera_ip, self.tx_port))
        self.get_logger().info(f"Sent: {cmd_ascii}")

        with self._lock:
            self._last_cmd_ascii = cmd_ascii
            self._last_cmd_time = time.time()

    def send_ptz(self, code_two_chars: str):
        cmd_ascii = self.build_ptz_cmd_ascii(code_two_chars)
        self.send_raw_ascii(cmd_ascii)

    # ---------------- Timers ----------------

    def _repeat_tick(self):
        # Re-send last command to keep motion alive (some devices need this)
        with self._lock:
            cmd = self._last_cmd_ascii
        if cmd:
            try:
                self.tx_sock.sendto(cmd.encode("ascii"), (self.camera_ip, self.tx_port))
            except Exception as e:
                self.get_logger().warn(f"Repeat send failed: {e}")

    def _auto_stop_tick(self):
        # If last command was not STOP and time elapsed > auto_stop_ms, send STOP once.
        now = time.time()
        with self._lock:
            last_cmd = self._last_cmd_ascii
            last_t = self._last_cmd_time

        if not last_cmd:
            return

        # detect stop command by payload content "...PTZ00..."
        is_stop = ("PTZ00" in last_cmd)
        if is_stop:
            return

        if (now - last_t) * 1000.0 >= float(self.auto_stop_ms):
            try:
                self.send_ptz("00")
            except Exception as e:
                self.get_logger().warn(f"Auto-stop failed: {e}")

    # ---------------- UDP RX (optional) ----------------

    def _rx_loop(self):
        assert self.rx_sock is not None
        while self._rx_running:
            try:
                data, addr = self.rx_sock.recvfrom(4096)
                # some devices reply binary-ish; show both safe decode + hex
                try:
                    txt = data.decode("utf-8", errors="replace")
                except Exception:
                    txt = "<decode failed>"
                hx = data.hex().upper()
                self.get_logger().info(f"RX from {addr}: '{txt}' | HEX={hx}")
            except Exception as e:
                if self._rx_running:
                    self.get_logger().warn(f"RX error: {e}")
                break

    # ---------------- Shutdown ----------------

    def destroy_node(self):
        self._rx_running = False
        try:
            if self.rx_sock:
                self.rx_sock.close()
        except Exception:
            pass
        try:
            self.tx_sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TopotekGimbalUdp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
