import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camera_info_publisher")

        self.declare_parameter("calibration_file", "")
        calibration_file = self.get_parameter("calibration_file").value

        self.camera_info_msg = CameraInfo()
        if calibration_file:
            self._load_calibration(calibration_file)
        else:
            self.get_logger().warn("No calibration_file parameter set")

        self.pub = self.create_publisher(CameraInfo, "/ir/camera_info", 10)
        self.sub = self.create_subscription(Image, "/ir/image", self._on_image, 10)

    def _load_calibration(self, path):
        with open(path, "r") as f:
            cal = yaml.safe_load(f)

        msg = self.camera_info_msg
        msg.width = cal["image_width"]
        msg.height = cal["image_height"]
        msg.distortion_model = cal.get("distortion_model", "plumb_bob")
        msg.d = [float(v) for v in cal["distortion_coefficients"]["data"]]
        msg.k = [float(v) for v in cal["camera_matrix"]["data"]]
        msg.r = [float(v) for v in cal["rectification_matrix"]["data"]]
        msg.p = [float(v) for v in cal["projection_matrix"]["data"]]

        self.get_logger().info(f"Loaded calibration from {path}")

    def _on_image(self, msg: Image):
        self.camera_info_msg.header = msg.header
        self.pub.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
