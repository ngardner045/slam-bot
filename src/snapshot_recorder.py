#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Joy, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

CAPTURE_DIR = "/captures"

IMG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,     # clouds should be RELIABLE
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

def _cloud_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    """
    Return (N,3) float32 array of xyz from PointCloud2.
    Uses the NumPy reader which often returns a structured array,
    then stacks fields explicitly.
    """
    # This returns a structured array with fields 'x','y','z' on many installs
    arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    # `arr` might be shape (N,) with dtype names, or already (N,3)
    if isinstance(arr, np.ndarray) and arr.dtype.names:
        xyz = np.vstack((arr['x'], arr['y'], arr['z'])).T
    else:
        # Fallback: ensure at least 2D and take first 3 columns
        a = np.asarray(arr)
        if a.ndim == 1:
            a = a.reshape(-1, 3)
        xyz = a[:, :3]
    return xyz.astype(np.float32, copy=False)

def _save_pcd_ascii(path: str, xyz: np.ndarray) -> None:
    """
    Minimal ASCII PCD writer for (N,3) float32 points.
    """
    n = xyz.shape[0]
    header = (
        "VERSION .7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    with open(path, "w") as f:
        f.write(header)
        # Write rows as "x y z"
        np.savetxt(f, xyz, fmt="%.6f %.6f %.6f")

def _save_bin(path: str, xyz: np.ndarray) -> None:
    """
    Save as raw binary (float32 little-endian), layout [x,y,z] per point.
    """
    xyz.astype(np.float32, copy=False).tofile(path)

class SnapshotTrigger(Node):
    def __init__(self):
        super().__init__('snapshot_trigger')
        # Params
        self.declare_parameter('y_button_index', 4)         # XBox: Y=4 in your current mapping
        self.declare_parameter('fn_trigger_index', 4)
        self.declare_parameter('cloud_format', 'pcd')       # 'pcd' | 'bin' | 'npy'

        self.declare_parameter('target_frame', 'base_link')  # TF target frame

        self.declare_parameter('robot_width', 0.35)
        self.declare_parameter('side_margin', 0.06)
        self.declare_parameter('z_min', 0.10)
        self.declare_parameter('z_max', 1.20)
        self.declare_parameter('x_forward_min', 0.70)
        self.declare_parameter('x_forward_max', 2.0)

        self.y_idx = int(self.get_parameter('y_button_index').value)
        self.fn_trigger_index = int(self.get_parameter('fn_trigger_index').value)
        self.cloud_format = str(self.get_parameter('cloud_format').value).lower()
        
        self.target_frame = str(self.get_parameter('target_frame').value)
        
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.side_margin = float(self.get_parameter('side_margin').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.x_forward_min = float(self.get_parameter('x_forward_min').value)
        self.x_forward_max = float(self.get_parameter('x_forward_max').value)

        # State
        self.prev_button = 0
        self.want_image = False
        self.want_cloud = False
        self.fn_trigger_pressed = False

        os.makedirs(CAPTURE_DIR, exist_ok=True)
        self.bridge = CvBridge()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subs
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(Image, '/camera/d455/color/image_raw', self.on_img, IMG_QOS)
        self.create_subscription(PointCloud2, '/camera/d455/depth/color/points', self.on_cloud, CLOUD_QOS)

        self.get_logger().info(
            "Snapshot trigger ready.\n"
            f"  Saving to:     {CAPTURE_DIR}\n"
            f"  Image topic:   /camera/d455/color/image_raw\n"
            f"  Cloud topic:   /camera/d455/depth/color/points\n"
            f"  Cloud format:  {self.cloud_format}\n"
            f"  Y button idx:  {self.y_idx}\n"
            f"  Target frame:  {self.target_frame}"
        )

    def on_joy(self, msg: Joy):
        if self.y_idx >= len(msg.buttons):
            return
        cur = msg.buttons[self.y_idx]
        if self.prev_button == 0 and cur == 1:
            # Rising edge: arm both captures
            self.want_image = True
            self.want_cloud = True
            self.get_logger().info('Y pressed → will save next color image (PNG) and next point cloud')
        self.prev_button = cur

        if len(msg.axes) > self.fn_trigger_index:
            fn_trigger_value = msg.axes[self.fn_trigger_index]
            self.fn_trigger_pressed = (fn_trigger_value < 0)

    def on_img(self, msg: Image):
        if not self.want_image:
            return
        self.want_image = False
        fname = os.path.join(CAPTURE_DIR, f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}.png")
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok = cv2.imwrite(fname, cv_img)
            if not ok:
                raise RuntimeError("cv2.imwrite returned False")
            self.get_logger().info(f"Saved image → {fname}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def on_cloud(self, msg: PointCloud2):
        if not self.want_cloud:
            return
        self.want_cloud = False
        try:            
            # 1) Look up the transform at the cloud timestamp
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,                 # target
                msg.header.frame_id,               # source (camera frame)
                Time.from_msg(msg.header.stamp)    # same time as the cloud
            )
            # 2) Transform cloud to target frame
            cloud_robot = do_transform_cloud(msg, tf)
            cloud_robot.header.frame_id = self.target_frame

            # 3) Convert to xyz for saving
            xyz = _cloud_to_xyz_array(cloud_robot)

            z = xyz[:, 2]
            zmask = (z >= -0.05) & (z <= 10)
            xyz = xyz[zmask]
            
            if xyz.size == 0:
                self.get_logger().warn("Point cloud empty after filtering; nothing saved.")
                return
            
            xyz_safe = None
            
            if self.fn_trigger_pressed:
                x = xyz[:, 0]
                y = xyz[:, 1]
                z = xyz[:, 2]
                half_w = 0.5 * self.robot_width + self.side_margin

                front_band = (x >= self.x_forward_min) & (x <= self.x_forward_max) & (np.abs(y) <= half_w)
                safety_zmask = (z >= self.z_min) & (z <= self.z_max)

                roi_mask = front_band & safety_zmask

                xyz_safe = xyz[roi_mask]

                self.get_logger().info(f"ROI points: {xyz_safe.shape[0]} / {xyz.shape[0]}")

            stamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
            fmt = self.cloud_format

            if fmt == 'pcd':
                out = os.path.join(CAPTURE_DIR, f"{stamp}.pcd")
                _save_pcd_ascii(out, xyz)
                if xyz_safe is not None:
                    out_safe = os.path.join(CAPTURE_DIR, f"{stamp}_safe.pcd")
                    _save_pcd_ascii(out_safe, xyz_safe)
            elif fmt == 'bin':
                out = os.path.join(CAPTURE_DIR, f"{stamp}.bin")
                _save_bin(out, xyz)
                if xyz_safe is not None:
                    out_safe = os.path.join(CAPTURE_DIR, f"{stamp}_safe.bin")
                    _save_bin(out_safe, xyz_safe)
            elif fmt == 'npy':
                out = os.path.join(CAPTURE_DIR, f"{stamp}.npy")
                np.save(out, xyz, allow_pickle=False)
                if xyz_safe is not None:
                    out_safe = os.path.join(CAPTURE_DIR, f"{stamp}_safe.npy")
                    np.save(out_safe, xyz_safe, allow_pickle=False)
            else:
                raise ValueError(f"Unsupported cloud_format '{fmt}' (use 'pcd', 'bin', or 'npy').")

            self.get_logger().info(f"Saved point cloud → {out}  (points: {xyz.shape[0]})")

        except Exception as e:
            self.get_logger().error(f"Failed to save point cloud: {e}")

def main():
    rclpy.init()
    node = SnapshotTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()