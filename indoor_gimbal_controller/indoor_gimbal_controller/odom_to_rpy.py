#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    """
    Convert quaternion to roll, pitch, yaw (radians), ROS convention.
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


class OdomToRpy(Node):
    def __init__(self):
        super().__init__("odom_to_rpy")

        # ---- Params ----
        self.declare_parameter("odom_topic", "/uav60/filter/odom")
        self.declare_parameter("out_topic", "/vehicle_rpy_deg")  # <-- set to your current UAV input topic
        self.declare_parameter("publish_hz", 30.0)               # rate-limit output; set 0.0 to publish on every odom msg
        self.declare_parameter("use_child_frame", False)         # usually False; some odom setups flip frames
        self.declare_parameter("invert_roll", False)
        self.declare_parameter("invert_pitch", False)
        self.declare_parameter("invert_yaw", False)
        self.declare_parameter("roll_offset_deg", 0.0)
        self.declare_parameter("pitch_offset_deg", 0.0)
        self.declare_parameter("yaw_offset_deg", 0.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.use_child_frame = bool(self.get_parameter("use_child_frame").value)
        self.invert_roll = bool(self.get_parameter("invert_roll").value)
        self.invert_pitch = bool(self.get_parameter("invert_pitch").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)

        self.roll_offset_deg = float(self.get_parameter("roll_offset_deg").value)
        self.pitch_offset_deg = float(self.get_parameter("pitch_offset_deg").value)
        self.yaw_offset_deg = float(self.get_parameter("yaw_offset_deg").value)

        # ---- State ----
        self._latest_rpy_deg = Vector3()
        self._have = False

        # ---- ROS IO ----
        self.pub = self.create_publisher(Vector3, self.out_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        if self.publish_hz > 0.0:
            period = 1.0 / self.publish_hz
            self.timer = self.create_timer(period, self._tick)
            self.get_logger().info(f"Publishing at {self.publish_hz} Hz to {self.out_topic}")
        else:
            self.timer = None
            self.get_logger().info(f"Publishing on every odom msg to {self.out_topic}")

        self.get_logger().info(f"Listening odom: {self.odom_topic}")

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation

        # Optional: some systems consider the "robot" orientation in child_frame;
        # usually you don't want this, so default False.
        # Keeping param for flexibility, but in practice you typically use pose.pose.orientation.
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)

        r = rad2deg(roll)
        p = rad2deg(pitch)
        y = rad2deg(yaw)

        # Apply invert
        if self.invert_roll:
            r = -r
        if self.invert_pitch:
            p = -p
        if self.invert_yaw:
            y = -y

        # Offsets (useful to align camera/gimbal frame)
        r += self.roll_offset_deg
        p += self.pitch_offset_deg
        y += self.yaw_offset_deg

        self._latest_rpy_deg.x = float(r)
        self._latest_rpy_deg.y = float(p)
        self._latest_rpy_deg.z = float(y)
        self._have = True

        if self.publish_hz <= 0.0:
            self.pub.publish(self._latest_rpy_deg)

    def _tick(self):
        if not self._have:
            return
        self.pub.publish(self._latest_rpy_deg)


def main():
    rclpy.init()
    node = OdomToRpy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
