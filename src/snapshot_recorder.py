#!/usr/bin/env python3
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2

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

def _ensure_xyz32(arr) -> np.ndarray:
    """
    Accept any of: structured array with fields x,y,z; object array of tuples;
    plain array with ≥3 columns; list of tuples. Return (N,3) float32.
    """
    a = arr

    # Structured dtype?
    if isinstance(a, np.ndarray) and a.dtype.names:
        # e.g. dtype with names ['x','y','z'] (possibly with padding)
        a = np.stack([a['x'], a['y'], a['z']], axis=1)
        a = a.astype(np.float32, copy=False)
        return a

    # Object array (each element is a 3-tuple)?
    if isinstance(a, np.ndarray) and a.dtype == object:
        a = np.array(list(a), dtype=np.float32)  # expands tuples to 2D
        if a.ndim == 1:
            a = a.reshape(-1, 3)
        return a

    # Anything list-like: coerce
    a = np.asarray(a)
    # If still 1D (flat), reshape to (N,3)
    if a.ndim == 1:
        a = a.reshape(-1, 3)

    # If it has more than 3 columns (e.g., xyzrgba), take the first 3
    if a.shape[1] > 3:
        a = a[:, :3]

    # Final dtype
    return a.astype(np.float32, copy=False)

def _rpy_deg_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Build 3x3 rotation matrix from roll, pitch, yaw in DEGREES (ROS RPY order).
    Rotation order: Rz(yaw) * Ry(pitch) * Rx(roll). Axes follow right-hand rule.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr,  cr]], dtype=np.float32)
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]], dtype=np.float32)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]], dtype=np.float32)
    R = (Rz @ Ry @ Rx).astype(np.float32)
    return R

def _apply_rigid_transform(xyz: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    xyz: (N,3), R: (3,3), t: (3,)
    Returns xyz' = R * xyz + t
    """
    return (xyz @ R.T) + t  # row vectors; faster than per-point loops

class SnapshotTrigger(Node):
    def __init__(self):
        super().__init__('snapshot_trigger')
        # Params
        self.declare_parameter('y_button_index', 4)         # XBox: Y=4 in your current mapping
        self.declare_parameter('cloud_format', 'pcd')       # 'pcd' | 'bin' | 'npy'

        self.declare_parameter('apply_transform', True)    # if True, apply T_cam->robot before save
        self.declare_parameter('tx', 0.2281)                   # meters (forward +X of robot)
        self.declare_parameter('ty', 0.0475)                   # meters (left +Y of robot)
        self.declare_parameter('tz', 0.1532)                   # meters (up +Z of robot)
        self.declare_parameter('roll_deg', 0.0)             # degrees
        self.declare_parameter('pitch_deg', 15.0)            # degrees (downward tilt is positive)
        self.declare_parameter('yaw_deg', 0.0)              # degrees

        R_OPT_TO_BASE = np.array([
            [ 0,  0, 1],
            [-1,  0, 0],
            [ 0, -1, 0],
        ], dtype=np.float32)

        self.y_idx = int(self.get_parameter('y_button_index').value)
        self.cloud_format = str(self.get_parameter('cloud_format').value).lower()
        
        self.apply_transform = bool(self.get_parameter('apply_transform').value)
        tx = float(self.get_parameter('tx').value)
        ty = float(self.get_parameter('ty').value)
        tz = float(self.get_parameter('tz').value)
        roll = float(self.get_parameter('roll_deg').value)
        pitch = float(self.get_parameter('pitch_deg').value)
        yaw = float(self.get_parameter('yaw_deg').value)

        self.t_vec = np.array([tx, ty, tz], dtype=np.float32)
        R_mount = _rpy_deg_to_R(roll, pitch, yaw).astype(np.float32)
        self.R_mat = (R_mount @ R_OPT_TO_BASE).astype(np.float32)

        # State
        self.prev_button = 0
        self.want_image = False
        self.want_cloud = False

        os.makedirs(CAPTURE_DIR, exist_ok=True)
        self.bridge = CvBridge()

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
            f"  Transform on:  {self.apply_transform}\n"
            f"  T (m):         [{tx:.3f}, {ty:.3f}, {tz:.3f}]\n"
            f"  R (deg RPY):   [{roll:.2f}, {pitch:.2f}, {yaw:.2f}]"
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
            raw = _cloud_to_xyz_array(msg)
            # DEBUG: see what came in
            self.get_logger().info(f"cloud raw dtype={getattr(raw, 'dtype', None)} shape={getattr(raw, 'shape', None)}")
            xyz = _ensure_xyz32(raw)
            self.get_logger().info(f"cloud norm dtype={xyz.dtype} shape={xyz.shape}")
            if xyz.size == 0:
                self.get_logger().warn("Point cloud empty after filtering; nothing saved.")
                return
            
            if self.apply_transform:
                xyz = _apply_rigid_transform(xyz, self.R_mat, self.t_vec)

            stamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
            fmt = self.cloud_format

            if fmt == 'pcd':
                out = os.path.join(CAPTURE_DIR, f"{stamp}.pcd")
                _save_pcd_ascii(out, xyz)
            elif fmt == 'bin':
                out = os.path.join(CAPTURE_DIR, f"{stamp}.bin")
                _save_bin(out, xyz)
            elif fmt == 'npy':
                out = os.path.join(CAPTURE_DIR, f"{stamp}.npy")
                np.save(out, xyz, allow_pickle=False)
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