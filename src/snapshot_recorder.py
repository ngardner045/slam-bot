#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
import cv2

CAPTURE_DIR = "/captures"
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class SnapshotTrigger(Node):
    def __init__(self):
        super().__init__('snapshot_trigger')
        self.declare_parameter('y_button_index', 4)
        self.y_idx = int(self.get_parameter('y_button_index').value)
        self.prev = 0
        self.want_image = True  # arm once at startup for sanity

        os.makedirs(CAPTURE_DIR, exist_ok=True)
        self.bridge = CvBridge()

        self.img_count = 0
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(Image, '/camera/d455/color/image_raw', self.on_img, SENSOR_QOS)

        # self.create_timer(2.0, self.tick)
        self.get_logger().info(f"Snapshot trigger ready. Saving to {CAPTURE_DIR}")

    def tick(self):
        self.get_logger().info(f"heartbeat: img_count={self.img_count}, want_image={self.want_image}")

    def on_joy(self, msg: Joy):
        if self.y_idx >= len(msg.buttons):
            return
        cur = msg.buttons[self.y_idx]
        if self.prev == 0 and cur == 1:
            self.want_image = True
            self.get_logger().info('Y pressed → will save next color frame as PNG')
        self.prev = cur

    def on_img(self, msg: Image):
        self.img_count += 1
        # self.get_logger().info(f"on_img: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} want_image={self.want_image}")
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

def main():
    rclpy.init()
    n = SnapshotTrigger()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
