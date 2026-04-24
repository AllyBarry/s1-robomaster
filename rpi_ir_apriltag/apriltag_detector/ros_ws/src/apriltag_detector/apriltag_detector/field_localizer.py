import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header, Float32MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import cv2


class FieldLocalizer(Node):
    def __init__(self):
        super().__init__("field_localizer")

        self.declare_parameter("corner_tag_ids", [0, 1, 2, 3])
        self.declare_parameter(
            "corner_tag_positions",
            [0.0, 0.0, 3.0, 0.0, 3.0, 3.0, 0.0, 3.0],
        )

        corner_ids = self.get_parameter("corner_tag_ids").value
        corner_pos_flat = self.get_parameter("corner_tag_positions").value

        self.corner_map = {}
        for i, tag_id in enumerate(corner_ids):
            self.corner_map[tag_id] = np.array(
                [corner_pos_flat[i * 2], corner_pos_flat[i * 2 + 1]],
                dtype=np.float32,
            )

        self.corner_ids = set(corner_ids)
        self.homography = None
        self.corner_image_points = {}

        # Single detection stream — all 36h11 tags arrive here
        self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self._on_detections,
            10,
        )

        self.pub_poses = self.create_publisher(PoseArray, "/field/robot_poses", 10)
        self.robot_pose_pubs = {}

        # Latched so late-joining subscribers (e.g. video_recorder spawned
        # after the corner tags have already been located) still receive
        # the current inverse homography. Keeps the pixel-frame overlay
        # logic out of this node — consumers just map field -> pixel.
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub_homography_inv = self.create_publisher(
            Float32MultiArray, "/field/homography_inv", latched_qos
        )

        # Dynamic TF broadcaster for robot poses
        self.tf_broadcaster = TransformBroadcaster(self)

        # Static TF: publish map -> field and keep re-publishing so
        # late-joining subscribers (rviz, other containers) receive it.
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self.create_timer(1.0, self._publish_static_transforms)

        self.get_logger().info(
            f"Field localizer started — corner tag IDs: {corner_ids}"
        )

    def _publish_static_transforms(self):
        now = self.get_clock().now().to_msg()

        # map -> field (identity — field origin is the map origin)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "field"
        t.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Split detections into corner vs robot by tag ID
    # ------------------------------------------------------------------
    def _on_detections(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            if det.id in self.corner_ids:
                corners = np.array([(c.x, c.y) for c in det.corners])
                self.corner_image_points[det.id] = corners.mean(axis=0)

        if len(self.corner_image_points) >= 4:
            self._compute_homography()
            if self.homography is not None:
                for tag_id, px in sorted(self.corner_image_points.items()):
                    mapped = self.corner_map.get(tag_id)
                    self.get_logger().debug(
                        f"tag {tag_id}: pixel ({px[0]:.0f}, {px[1]:.0f}) "
                        f"-> field ({mapped[0]:.2f}, {mapped[1]:.2f})"
                    )

        if self.homography is None:
            return

        # Process robot tags (anything not a corner)
        pose_array = PoseArray()
        pose_array.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="field",
        )

        for det in msg.detections:
            if det.id in self.corner_ids:
                continue

            corners = np.array([(c.x, c.y) for c in det.corners])
            centre = corners.mean(axis=0).reshape(1, 1, 2).astype(np.float32)
            field_pt = cv2.perspectiveTransform(centre, self.homography)
            fx, fy = field_pt[0, 0]

            # Compute heading from tag edge: corners 0→1 is the top edge
            # of the tag, giving the robot's forward direction in field space
            edge_pts = corners[:2].reshape(1, 2, 2).astype(np.float32)
            field_edge = cv2.perspectiveTransform(edge_pts, self.homography)
            dx_tag = field_edge[0, 1, 0] - field_edge[0, 0, 0]
            dy_tag = field_edge[0, 1, 1] - field_edge[0, 0, 1]
            yaw = math.atan2(dy_tag, dx_tag)

            # Yaw to quaternion (rotation about Z)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)

            pose = Pose(
                position=Point(x=float(fx), y=float(fy), z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=float(qz), w=float(qw)),
            )
            pose_array.poses.append(pose)

            if det.id not in self.robot_pose_pubs:
                self.robot_pose_pubs[det.id] = self.create_publisher(
                    PoseStamped, f"/field/robot_{det.id}/pose", 10
                )
            ps = PoseStamped(header=pose_array.header, pose=pose)
            self.robot_pose_pubs[det.id].publish(ps)

            # Broadcast robot pose as TF: field -> robot_{id}
            tf_msg = TransformStamped()
            tf_msg.header = pose_array.header
            tf_msg.child_frame_id = f"robot_{det.id}"
            tf_msg.transform.translation.x = float(fx)
            tf_msg.transform.translation.y = float(fy)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        self.pub_poses.publish(pose_array)

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

            if self.homography is not None:
                try:
                    h_inv = np.linalg.inv(self.homography).astype(np.float32)
                    msg = Float32MultiArray()
                    msg.data = h_inv.flatten().tolist()
                    self.pub_homography_inv.publish(msg)
                except np.linalg.LinAlgError:
                    self.get_logger().warning(
                        "Homography non-invertible — skipping H^-1 publish"
                    )


def main(args=None):
    rclpy.init(args=args)
    node = FieldLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
