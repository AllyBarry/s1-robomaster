import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2


class DetectionOverlay(Node):
    def __init__(self):
        super().__init__("detection_overlay")
        self.bridge = CvBridge()
        self.latest_detections = None

        self.sub_detections = self.create_subscription(
            AprilTagDetectionArray, "/detections", self._on_detections, 10
        )
        self.sub_image = self.create_subscription(
            Image, "image_raw", self._on_image, 10
        )
        self.pub = self.create_publisher(Image, "/detections/image", 10)

    def _on_detections(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    def _on_image(self, msg: Image):
        if self.latest_detections is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        for det in self.latest_detections.detections:
            corners = np.array(
                [(int(c.x), int(c.y)) for c in det.corners], dtype=np.int32
            )

            cv2.polylines(cv_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

            centre = corners.mean(axis=0).astype(int)
            label = f"id:{det.id}"
            cv2.putText(
                cv_image, label, (centre[0] - 20, centre[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
