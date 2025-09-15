#!/usr/bin/env python3
import numpy as np
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import PointCloud2, Joy
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

BEST_EFFORT_10 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
RELIABLE_10 = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class SafeManualGuard(Node):
    def __init__(self):
        super().__init__('safe_manual_guard')

        # Params
        self.declare_parameter('pointcloud_topic', '/camera/d455/depth/color/points')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_in_topic', '/cmd_vel')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_safe')
        self.declare_parameter('target_frame', 'base_link')

        # Robot geometry & safety
        self.declare_parameter('robot_width', 0.35)
        self.declare_parameter('robot_length', 0.40)
        self.declare_parameter('side_margin', 0.06)
        self.declare_parameter('front_margin', 0.10)
        self.declare_parameter('rear_margin', 0.06)

        # Dynamics
        self.declare_parameter('accel_limit', 0.8)
        self.declare_parameter('ctrl_latency', 0.10)
        self.declare_parameter('min_buffer', 0.15)

        # Cloud filtering (CHANGED: z_max up, reverse window larger)
        self.declare_parameter('z_min', 0.10)
        self.declare_parameter('z_max', 1.20)          # was 0.50 → let wall hits through
        self.declare_parameter('x_forward_max', 2.0)    # 2 m is plenty for manual safety
        self.declare_parameter('x_forward_min', 1.0)
        self.declare_parameter('voxel_stride', 12)
        self.declare_parameter('max_points', 60000)

        # Controls
        self.declare_parameter('override_button_idx', 7)
        self.declare_parameter('hard_stop_clearance', 0.20)

        # Sensor limits / guard (CHANGED: realistic near range + curtain)
        self.declare_parameter('sensor_min_range', 0.65)     # D455 near limit in practice
        self.declare_parameter('guard_distance', 0.75)       # stop before blind zone
        self.declare_parameter('cloud_stale_timeout', 0.20)  # require fresh cloud before moving

        # Read params
        gp = self.get_parameter
        self.pc_topic      = gp('pointcloud_topic').value
        self.joy_topic     = gp('joy_topic').value
        self.cmd_in_topic  = gp('cmd_in_topic').value
        self.cmd_out_topic = gp('cmd_out_topic').value
        self.target_frame  = gp('target_frame').value

        self.robot_width   = gp('robot_width').value
        self.robot_length  = gp('robot_length').value
        self.side_margin   = gp('side_margin').value
        self.front_margin  = gp('front_margin').value
        self.rear_margin   = gp('rear_margin').value

        self.accel_limit   = gp('accel_limit').value
        self.ctrl_latency  = gp('ctrl_latency').value
        self.min_buffer    = gp('min_buffer').value

        self.z_min         = gp('z_min').value
        self.z_max         = gp('z_max').value
        self.x_forward_max = gp('x_forward_max').value
        self.x_forward_min = gp('x_forward_min').value
        self.voxel_stride  = max(1, int(gp('voxel_stride').value))
        self.max_points    = int(gp('max_points').value)

        self.override_button_idx = int(gp('override_button_idx').value)
        self.hard_stop_clearance = gp('hard_stop_clearance').value
        self.sensor_min_range    = gp('sensor_min_range').value
        self.guard_distance      = gp('guard_distance').value
        self.cloud_stale_timeout = gp('cloud_stale_timeout').value

        self._last_cloud_time = None  # updated each cloud

        # State
        self._latest_cmd = Twist()
        self._override_held = False
        self._front_min = float('inf')
        self._rear_min = float('inf')
        self._front_has = False        # NEW
        self._rear_has  = False        # NEW
        self._safety_blocking = False
        self._debug_reason = ""

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subs/Pubs
        self.create_subscription(Twist, self.cmd_in_topic, self._on_cmd, qos_profile=RELIABLE_10)
        self.create_subscription(PointCloud2, self.pc_topic, self._on_cloud, qos_profile=BEST_EFFORT_10)
        self.create_subscription(Joy, self.joy_topic, self._on_joy, qos_profile=RELIABLE_10)

        self.pub_cmd    = self.create_publisher(Twist, self.cmd_out_topic, qos_profile=RELIABLE_10)
        self.pub_active = self.create_publisher(Bool, '/safety/active', qos_profile=RELIABLE_10)
        self.pub_debug  = self.create_publisher(String, '/safety/debug', qos_profile=RELIABLE_10)

        # Fast timer (50 Hz)
        self.timer = self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f"SafeManualGuard TF→{self.target_frame}; stride={self.voxel_stride}, cap={self.max_points}."
        )

    # --- Callbacks ---
    def _on_cmd(self, msg: Twist):
        self._latest_cmd = msg

    def _on_joy(self, msg: Joy):
        if 0 <= self.override_button_idx < len(msg.buttons):
            self._override_held = (msg.buttons[self.override_button_idx] == 1)

    def _on_cloud(self, cloud_in: PointCloud2):
        # Transform to base frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, cloud_in.header.frame_id, Time.from_msg(cloud_in.header.stamp)
            )
            cloud = do_transform_cloud(cloud_in, tf)
        except Exception as e:
            self.get_logger().debug(f"TF miss: {e}")
            return

        try:
            # Vectorized read of xyz
            arr = pc2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
            if isinstance(arr, np.ndarray) and arr.dtype.names:
                xyz = np.vstack((arr['x'], arr['y'], arr['z'])).T
            else:
                a = np.asarray(arr)
                xyz = a.reshape(-1, 3) if a.ndim == 1 else a[:, :3]
            xyz = xyz.astype(np.float32, copy=False)

            # Decimate
            if self.voxel_stride > 1 and xyz.shape[0] > self.voxel_stride:
                xyz = xyz[::self.voxel_stride]

            # Cap
            if self.max_points > 0 and xyz.shape[0] > self.max_points:
                idx = np.random.choice(xyz.shape[0], self.max_points, replace=False)
                xyz = xyz[idx]

            # Crop Z
            z = xyz[:, 2]
            zmask = (z >= self.z_min) & (z <= self.z_max)
            if not zmask.any():
                self._front_min = float('inf')
                self._rear_min = float('inf')
                self._front_has = False
                self._rear_has = False
                return
            xyz = xyz[zmask]

            # ROI masks
            x = xyz[:, 0]
            y = xyz[:, 1]
            half_w = 0.5 * self.robot_width + self.side_margin

            front_band = (x >= self.x_forward_min) & (x <= self.x_forward_max) & (np.abs(y) <= half_w)

            # front_mask = (x >= 0.0) & (x <= self.x_forward_max) & (np.abs(y) <= half_w)
            # rear_mask  = (x <= 0.0) & (x >= -self.x_reverse_max) & (np.abs(y) <= half_w)

            front_has = bool(np.any(front_band))

            self._front_min = float(np.min(x[front_band])) if front_has else float('inf')
            self._front_has = front_has                    # NEW
            self._last_cloud_time = self.get_clock().now()

            # Optional debug — uncomment if needed:
            n_f = int(np.count_nonzero(front_band))
            self.get_logger().info(f"front_has={front_has} n_f={n_f} front_min={self._front_min:.2f}")

        except Exception as e:
            self.get_logger().warn(f"Cloud parse error: {e}")

    # --- Safety ---
    def _stopping_distance(self, v_abs: float) -> float:
        return (v_abs**2) / (2.0 * max(1e-3, self.accel_limit)) + self.ctrl_latency * v_abs + self.min_buffer

    def _max_speed_from_clearance(self, clearance: float, margin: float) -> float:
        a = max(1e-6, self.accel_limit)
        b = self.ctrl_latency
        c = self.min_buffer + margin
        C = clearance - c
        if C <= 0.0:
            return 0.0
        # Solve v^2/(2a) + b v - C = 0 → v = -ab + sqrt((ab)^2 + 2aC)
        disc = (a*b)**2 + 2*a*C
        return max(0.0, -a*b + math.sqrt(disc))

    def _tick(self):  # CHANGED: clamp + stale gate + no-points handling
        now = self.get_clock().now()
        cmd_in = self._latest_cmd

        # 1) Override → passthrough
        if self._override_held:
            self._safety_blocking = False
            self._debug_reason = "override"
            self._publish(cmd_in)
            return

        # 2) Require fresh cloud to avoid the initial jolt
        if (self._last_cloud_time is None or
            (now - self._last_cloud_time).nanoseconds * 1e-9 > self.cloud_stale_timeout):
            out = Twist()
            out.angular.z = 0.0 if cmd_in.linear.x != 0.0 else cmd_in.angular.z
            self._safety_blocking = True
            self._debug_reason = "stale_cloud"
            self._publish(out)
            return

        v_in = cmd_in.linear.x
        w_in = cmd_in.angular.z

        if v_in >= 0.0:
            clearance = self._front_min
            margin = self.front_margin
            roi_has_points = self._front_has
            dir_tag = "forward"
        else:
            clearance = self._rear_min
            margin = self.rear_margin
            roi_has_points = self._rear_has
            dir_tag = "reverse"

        # 3) If ROI has no points, treat as unknown/unsafe → stop linear
        if not roi_has_points:
            out = Twist()
            out.angular.z = 0.0 if v_in != 0.0 else w_in
            self._safety_blocking = True
            self._debug_reason = f"no_points_{dir_tag}"
            self._publish(out)
            return

        # 4) Guard curtain (near-range blind zone)
        curtain = max(self.guard_distance, self.sensor_min_range)
        if clearance <= curtain:
            v_allowed = 0.0
            reason = f"guard_{dir_tag} (min={clearance:.2f}≤{curtain:.2f})"
        else:
            # 5) Speed clamp from stopping model
            v_allowed = self._max_speed_from_clearance(clearance, margin)
            reason = f"clamp_{dir_tag} (min={clearance:.2f}, vmax={v_allowed:.2f})"

        # 6) Apply clamp smoothly
        out = Twist()
        out.linear.x = math.copysign(min(abs(v_in), v_allowed), v_in)
        out.angular.z = 0.0 if (out.linear.x == 0.0 and clearance <= curtain) else w_in

        self._safety_blocking = (abs(out.linear.x) < abs(v_in) - 1e-6)
        self._debug_reason = "clear" if not self._safety_blocking else reason
        self._publish(out)

    def _publish(self, cmd: Twist):
        self.pub_cmd.publish(cmd)
        self.pub_active.publish(Bool(data=self._safety_blocking))
        self.pub_debug.publish(String(data=self._debug_reason))

def main():
    rclpy.init()
    node = SafeManualGuard()
    try:
        exec = MultiThreadedExecutor(num_threads=2)
        exec.add_node(node)
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
