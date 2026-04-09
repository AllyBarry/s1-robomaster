import numpy as np
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
import cv2


class FieldLocalizer(Node):
    def __init__(self):
        super().__init__("field_localizer")

        self.declare_parameter("corner_tag_ids", [0, 1, 2, 3])
        self.declare_parameter(
            "corner_tag_positions",
            [0.0, 0.0, 3.0, 0.0, 3.0, 2.0, 0.0, 2.0],
        )

        corner_ids = self.get_parameter("corner_tag_ids").value
        corner_pos_flat = self.get_parameter("corner_tag_positions").value

        self.corner_map = {}
        for i, tag_id in enumerate(corner_ids):
            self.corner_map[tag_id] = np.array(
                [corner_pos_flat[i * 2], corner_pos_flat[i * 2 + 1]],
                dtype=np.float32,
            )

        self.homography = None
        self.corner_image_points = {}

        self.sub_corners = self.create_subscription(
            AprilTagDetectionArray,
            "/detections/field",
            self._on_corner_detections,
            10,
        )
        self.sub_robots = self.create_subscription(
            AprilTagDetectionArray,
            "/detections/robots",
            self._on_robot_detections,
            10,
        )

        self.pub_poses = self.create_publisher(PoseArray, "/field/robot_poses", 10)
        self.robot_pose_pubs = {}

        self.get_logger().info(
            f"Field localizer started — corner tags: {corner_ids}"
        )

    # ------------------------------------------------------------------
    # Corner (16h5) detections → homography
    # ------------------------------------------------------------------
    def _on_corner_detections(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            if det.id in self.corner_map:
                corners = np.array([(c.x, c.y) for c in det.corners])
                self.corner_image_points[det.id] = corners.mean(axis=0)

        if len(self.corner_image_points) >= 4:
            self._compute_homography()

    def _compute_homography(self):
        src_pts = []
        dst_pts = []
        for tag_id in sorted(self.corner_image_points.keys()):
            if tag_id in self.corner_map:
                src_pts.append(self.corner_image_points[tag_id])
                dst_pts.append(self.corner_map[tag_id])

        if len(src_pts) >= 4:
            src = np.array(src_pts, dtype=np.float32)
            dst = np.array(dst_pts, dtype=np.float32)
            self.homography, _ = cv2.findHomography(src, dst)
            self.get_logger().debug("Homography updated")

    # ------------------------------------------------------------------
    # Robot (36h11) detections → field coordinates
    # ------------------------------------------------------------------
    def _on_robot_detections(self, msg: AprilTagDetectionArray):
        if self.homography is None:
            return

        pose_array = PoseArray()
        pose_array.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="field",
        )

        for det in msg.detections:
            corners = np.array([(c.x, c.y) for c in det.corners])
            centre = corners.mean(axis=0).reshape(1, 1, 2).astype(np.float32)
            field_pt = cv2.perspectiveTransform(centre, self.homography)
            fx, fy = field_pt[0, 0]

            pose = Pose(
                position=Point(x=float(fx), y=float(fy), z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            pose_array.poses.append(pose)

            # Per-robot topic
            if det.id not in self.robot_pose_pubs:
                self.robot_pose_pubs[det.id] = self.create_publisher(
                    PoseStamped, f"/field/robot_{det.id}/pose", 10
                )
            ps = PoseStamped(header=pose_array.header, pose=pose)
            self.robot_pose_pubs[det.id].publish(ps)

        self.pub_poses.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = FieldLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
