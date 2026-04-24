import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2


class DetectionOverlay(Node):
    def __init__(self):
        super().__init__("detection_overlay")
        self.bridge = CvBridge()
        self.latest_detections = None

        self.declare_parameter("corner_tag_ids", [0, 1, 2, 3])
        # Throttle so the compressed debug stream stays light on DDS —
        # RViz doesn't need 30 Hz overlay. Set to 0 to disable throttling.
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("jpeg_quality", 70)

        self.corner_ids = set(self.get_parameter("corner_tag_ids").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.min_period = (1.0 / rate) if rate > 0.0 else 0.0
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.last_publish_t = 0.0

        self.create_subscription(
            AprilTagDetectionArray, "/detections", self._on_detections, 10
        )
        self.create_subscription(
            Image, "image_raw", self._on_image, 10
        )
        self.pub_raw = self.create_publisher(Image, "/detections/image", 1)
        self.pub_compressed = self.create_publisher(
            CompressedImage, "/detections/image/compressed", 1
        )

    def _on_detections(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    def _on_image(self, msg: Image):
        if self.latest_detections is None:
            return

        now = time.monotonic()
        if self.min_period > 0.0 and (now - self.last_publish_t) < self.min_period:
            return
        self.last_publish_t = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        for det in self.latest_detections.detections:
            corners = np.array(
                [(int(c.x), int(c.y)) for c in det.corners], dtype=np.int32
            )
            is_corner = det.id in self.corner_ids
            color = (255, 0, 0) if is_corner else (0, 255, 0)
            prefix = "field" if is_corner else "robot"

            cv2.polylines(cv_image, [corners], isClosed=True, color=color, thickness=2)
            centre = corners.mean(axis=0).astype(int)
            cv2.putText(
                cv_image, f"{prefix}:{det.id}",
                (centre[0] - 20, centre[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )

        # Raw Image for local subscribers that want uncompressed; the
        # CompressedImage variant is what remote tools (RViz) should use.
        raw_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        raw_msg.header = msg.header
        self.pub_raw.publish(raw_msg)

        ok, buf = cv2.imencode(
            ".jpg", cv_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if ok:
            comp = CompressedImage()
            comp.header = msg.header
            comp.format = "jpeg"
            comp.data = buf.tobytes()
            self.pub_compressed.publish(comp)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
