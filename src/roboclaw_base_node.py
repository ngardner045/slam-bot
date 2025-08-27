#!/usr/bin/env python3
"""
RoboClaw base node for a differential drive robot.

Features
- Subscribes: /cmd_vel (geometry_msgs/Twist)
- Publishes:  /odom (nav_msgs/Odometry)
- Optional TF: odom -> base_link
- Reads wheel encoder speeds from RoboClaw (cmd 108)
- Computes wheel linear speeds, robot v,w, and integrates odometry
- Sends motor commands as signed duty (cmd 34) with safety timeout
- Parameters to tune geometry, encoder CPR, gear ratio, and scaling

Run directly (no package needed):
  python3 roboclaw_base_node.py

Or make executable and ros2-run it from your workspace package.

Notes
- Default port is /dev/ttyACM0. If you added the udev rule, set port to /dev/roboclaw.
- enc_cpr is counts per motor shaft revolution **after quadrature** (CPR),
  and gear_ratio is motor_shaft_rev : wheel_rev. Adjust both to get correct odometry.
- If encoders tick the opposite sign, flip invert_left / invert_right.
"""
import math
import struct
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import serial

CRC_POLY = 0x1021


def crc16(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= (b << 8)
        for _ in range(8):
            c = ((c << 1) ^ CRC_POLY) & 0xFFFF if (c & 0x8000) else ((c << 1) & 0xFFFF)
    return c


class RoboClawIF:
    """Minimal RoboClaw packet-serial interface for the commands we need."""

    def __init__(self, port: str, baud: int, address: int, timeout_s: float = 0.2):
        self.addr = address
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout_s)
        # Avoid DTR/RTS resets on some boards
        self.ser.setDTR(False)
        self.ser.setRTS(False)

    def _write(self, cmd: int, payload: bytes = b"", expect_ack: bool = False) -> bool:
        pkt = bytes([self.addr, cmd]) + payload
        c = crc16(pkt)
        pkt += bytes([(c >> 8) & 0xFF, c & 0xFF])
        self.ser.write(pkt)
        self.ser.flush()
        if not expect_ack:
            return True
        a = self.ser.read(1)
        return len(a) == 1 and a[0] == 0xFF

    def _read_exact(self, n: int, to: float = 0.4) -> bytes:
        self.ser.timeout = to
        out = b""
        while len(out) < n:
            chunk = self.ser.read(n - len(out))
            if not chunk:
                break
            out += chunk
        return out

    # --- Commands ---
    def read_speed12(self) -> Optional[Tuple[int, int]]:
        """Return (m1_qpps, m2_qpps) in encoder counts/sec, or None on failure."""
        # Cmd 108: ReadSpeedM1M2 -> 4 + 4 + CRC
        self._write(108)
        r = self._read_exact(10)
        if len(r) != 10:
            return None
        m1 = struct.unpack(">i", r[:4])[0]
        m2 = struct.unpack(">i", r[4:8])[0]
        return m1, m2

    def read_enc1(self) -> Optional[int]:
        # Cmd 16: ReadEncM1 -> 4 bytes count + 1 status + 2 CRC
        self._write(16)
        r = self._read_exact(7)
        if len(r) != 7:
            return None
        return struct.unpack(">i", r[:4])[0]

    def read_enc2(self) -> Optional[int]:
        # Cmd 17: ReadEncM2
        self._write(17)
        r = self._read_exact(7)
        if len(r) != 7:
            return None
        return struct.unpack(">i", r[:4])[0]

    def drive_signed_duty(self, m1: int, m2: int) -> None:
        """Cmd 34: Drive both motors with signed duty (-32768..32767)."""
        m1 = max(-32768, min(32767, int(m1)))
        m2 = max(-32768, min(32767, int(m2)))
        payload = struct.pack(">hh", m1, m2)
        self._write(34, payload, expect_ack=True)

    def close(self):
        try:
            self.drive_signed_duty(0, 0)
        except Exception:
            pass
        self.ser.close()


class RoboClawBase(Node):
    def __init__(self):
        super().__init__("roboclaw_base")

        # --- Parameters ---
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("address", 0x80)
        self.declare_parameter("wheel_radius", 0.06)   # m
        self.declare_parameter("track_width", 0.34)    # m
        self.declare_parameter("enc_cpr", 2048)        # counts per motor rev (after quadrature)
        self.declare_parameter("gear_ratio", 30.0)     # motor revs per 1 wheel rev
        self.declare_parameter("max_lin", 0.8)         # m/s clamp
        self.declare_parameter("max_ang", 2.0)         # rad/s clamp
        self.declare_parameter("max_duty_frac", 0.5)   # of 32767
        self.declare_parameter("v_full", 1.5)          # m/s that maps to max duty
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("swap_motors", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("cmd_timeout", 0.6)     # s (stop if stale)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        addr = int(self.get_parameter("address").value)

        self.R = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("track_width").value)
        self.enc_cpr = float(self.get_parameter("enc_cpr").value)
        self.gear = float(self.get_parameter("gear_ratio").value)
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)
        self.max_duty = int(32767 * float(self.get_parameter("max_duty_frac").value))
        self.v_full = float(self.get_parameter("v_full").value)
        self.inv_L = bool(self.get_parameter("invert_left").value)
        self.inv_R = bool(self.get_parameter("invert_right").value)
        self.swap = bool(self.get_parameter("swap_motors").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.iface = RoboClawIF(port, baud, addr)

        # State
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0

        # Odometry state (2D)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ROS I/O
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tfb = TransformBroadcaster(self)

        # Control/read loop
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.loop)

        # Ensure stop on shutdown
        # Register shutdown hook so motors stop if process is killed
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

        self.get_logger().info("roboclaw_base node started.")

    # --- Helpers ---
    def cb_cmd(self, msg: Twist):
        # Clamp
        v = max(-self.max_lin, min(self.max_lin, float(msg.linear.x)))
        w = max(-self.max_ang, min(self.max_ang, float(msg.angular.z)))
        self.last_cmd_v = v
        self.last_cmd_w = w
        self.last_cmd_time = self.get_clock().now()

    def wheel_linear_to_duty(self, v_l: float, v_r: float) -> Tuple[int, int]:
        # Simple proportional map: v_full -> max_duty
        duty_l = int(self.max_duty * (v_l / self.v_full))
        duty_r = int(self.max_duty * (v_r / self.v_full))
        return duty_l, duty_r

    def qpps_to_wheel_linear(self, qpps: float) -> float:
        # counts/sec -> wheel rad/s -> m/s
        # counts per motor rev -> counts per wheel rev = enc_cpr * gear
        counts_per_wheel_rev = self.enc_cpr * self.gear
        wheel_rev_per_s = qpps / counts_per_wheel_rev
        omega = wheel_rev_per_s * 2.0 * math.pi
        return omega * self.R

    def send_drive(self, duty_left: int, duty_right: int):
        # Motor mapping / inversion handling
        L = -duty_left if self.inv_L else duty_left
        R = -duty_right if self.inv_R else duty_right
        if self.swap:
            m1, m2 = R, L
        else:
            m1, m2 = L, R
        self.iface.drive_signed_duty(m1, m2)

    def read_wheel_speeds(self) -> Optional[Tuple[float, float]]:
        s = self.iface.read_speed12()
        if s is None:
            return None
        m1_qpps, m2_qpps = s
        # Map back to left/right based on swap/invert
        if self.swap:
            L_qpps, R_qpps = m2_qpps, m1_qpps
        else:
            L_qpps, R_qpps = m1_qpps, m2_qpps
        if self.inv_L:
            L_qpps = -L_qpps
        if self.inv_R:
            R_qpps = -R_qpps
        v_l = self.qpps_to_wheel_linear(float(L_qpps))
        v_r = self.qpps_to_wheel_linear(float(R_qpps))
        return v_l, v_r

    def integrate_odom(self, v: float, w: float, dt: float):
        if abs(w) < 1e-6:
            # Straight
            dx = v * dt * math.cos(self.yaw)
            dy = v * dt * math.sin(self.yaw)
            self.x += dx
            self.y += dy
        else:
            # Arc
            Rturn = v / w
            dtheta = w * dt
            self.x += Rturn * (math.sin(self.yaw + dtheta) - math.sin(self.yaw))
            self.y += -Rturn * (math.cos(self.yaw + dtheta) - math.cos(self.yaw))
            self.yaw += dtheta
        # Normalize yaw
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

    def publish_odom(self, now: RosTime, v: float, w: float):
        msg = Odometry()
        msg.header = Header(stamp=now, frame_id=self.odom_frame)
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        # Yaw -> quaternion (Z-only)
        half = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.pub_odom.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z = msg.pose.pose.orientation.z
            tf.transform.rotation.w = msg.pose.pose.orientation.w
            self.tfb.sendTransform(tf)

    def loop(self):
        now_time = self.get_clock().now()
        # Safety timeout: stop if no fresh cmd_vel
        if (now_time - self.last_cmd_time) > Duration(seconds=self.cmd_timeout):
            cmd_v, cmd_w = 0.0, 0.0
        else:
            cmd_v, cmd_w = self.last_cmd_v, self.last_cmd_w

        # Convert desired body velocities to wheel linear
        v_l_des = cmd_v - (cmd_w * self.L / 2.0)
        v_r_des = cmd_v + (cmd_w * self.L / 2.0)

        # Map to duty and send
        duty_l, duty_r = self.wheel_linear_to_duty(v_l_des, v_r_des)
        self.send_drive(duty_l, duty_r)

        # Read actual wheel speeds
        measured = self.read_wheel_speeds()
        if measured is not None:
            v_l, v_r = measured
            v_body = 0.5 * (v_l + v_r)
            w_body = (v_r - v_l) / self.L
        else:
            # Fall back to commanded (not ideal, but keeps odom moving)
            v_body, w_body = cmd_v, cmd_w

        # Integrate odom
        self.integrate_odom(v_body, w_body, self.dt)

        # Publish odom (use node clock time)
        self.publish_odom(now_time.to_msg(), v_body, w_body)

    def on_shutdown(self):
        try:
            self.iface.drive_signed_duty(0, 0)
        except Exception:
            pass
        self.iface.close()
        self.get_logger().info("Stopped motors and closed serial.")


def main():
    rclpy.init()
    node = RoboClawBase()
    try:
        rclpy.spin(node)
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
