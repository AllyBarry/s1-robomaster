"""
Records the overhead camera stream with per-robot trajectory overlays.

Subscribes to the annotated apriltag image stream, the inverse homography
published by field_localizer, each robot's `/field/robot_{id}/pose`, and
`/target_markers` (for formation target crosses). On every incoming frame
it projects field-frame trails and target positions into pixel space via
the inverse homography and writes the composited frame to
`{log_dir}/{scenario}.mp4`.

Drops into the same run folder as trajectory_logger so each experiment
carries its own video artifact alongside the CSV, JSON sidecar, and
`plots/` directory.
"""

import collections
import pathlib
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


_PALETTE_BGR = [
    (66, 135, 245),   # blue
    (80, 200, 120),   # green
    (60, 76, 231),    # red
    (232, 189, 56),   # cyan
    (180, 100, 220),  # purple
]


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__("video_recorder")

        self.declare_parameter("robot_ids", [0, 1, 2])
        self.declare_parameter("log_dir", "/ros_ws/experiment_logs")
        self.declare_parameter("scenario", "run")
        self.declare_parameter("image_topic", "/detections/image")
        self.declare_parameter("homography_topic", "/field/homography_inv")
        self.declare_parameter("markers_topic", "/target_markers")
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("trail_length", 0)

        self.robot_ids = [int(v) for v in self.get_parameter("robot_ids").value]
        self.log_dir = pathlib.Path(self.get_parameter("log_dir").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.fps = float(self.get_parameter("fps").value)
        trail_len = int(self.get_parameter("trail_length").value)
        image_topic = str(self.get_parameter("image_topic").value)
        hom_topic = str(self.get_parameter("homography_topic").value)
        markers_topic = str(self.get_parameter("markers_topic").value)

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.video_path = self.log_dir / f"{self.scenario}.mp4"

        self.bridge = CvBridge()
        self.writer: cv2.VideoWriter | None = None
        self.h_inv: np.ndarray | None = None
        self.targets: list[tuple[float, float]] | None = None

        maxlen = trail_len if trail_len > 0 else None
        self.trails: dict[int, collections.deque] = {
            rid: collections.deque(maxlen=maxlen) for rid in self.robot_ids
        }
        self._colors = {
            rid: _PALETTE_BGR[i % len(_PALETTE_BGR)]
            for i, rid in enumerate(self.robot_ids)
        }
        self._lock = threading.Lock()

        # Latched so we still receive the last-published H^-1 even if
        # field_localizer was already running before this node came up.
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(
            Float32MultiArray, hom_topic, self._on_homography, latched_qos,
        )
        self.create_subscription(
            MarkerArray, markers_topic, self._on_markers, 10,
        )
        for rid in self.robot_ids:
            self.create_subscription(
                PoseStamped,
                f"/field/robot_{rid}/pose",
                lambda msg, r=rid: self._on_pose(msg, r),
                10,
            )

        self.get_logger().info(
            f"video_recorder: image='{image_topic}', output={self.video_path}, "
            f"fps={self.fps}, robots={self.robot_ids}"
        )

    def _on_homography(self, msg: Float32MultiArray):
        if len(msg.data) != 9:
            return
        with self._lock:
            self.h_inv = np.array(msg.data, dtype=np.float32).reshape(3, 3)

    def _on_markers(self, msg: MarkerArray):
        if self.targets is not None:
            return
        # Mirror trajectory_logger: formation_targets CYLINDERs only, to
        # avoid double-drawing the text-label twin markers.
        targets = [
            (float(m.pose.position.x), float(m.pose.position.y))
            for m in msg.markers
            if m.ns == "formation_targets" and m.type == Marker.CYLINDER
        ]
        if targets:
            with self._lock:
                self.targets = targets

    def _on_pose(self, msg: PoseStamped, rid: int):
        with self._lock:
            self.trails[rid].append(
                (float(msg.pose.position.x), float(msg.pose.position.y))
            )

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge convert failed: {e}")
            return

        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(
                str(self.video_path), fourcc, self.fps, (w, h)
            )
            if not self.writer.isOpened():
                self.get_logger().error(
                    f"VideoWriter failed to open {self.video_path} — disabling"
                )
                self.writer = None
                return
            self.get_logger().info(
                f"video_recorder: writing {w}x{h} @ {self.fps} fps "
                f"-> {self.video_path}"
            )

        with self._lock:
            h_inv = None if self.h_inv is None else self.h_inv.copy()
            trails = {rid: list(pts) for rid, pts in self.trails.items()}
            targets = list(self.targets) if self.targets else []

        if h_inv is not None:
            self._draw_overlay(frame, h_inv, trails, targets)

        self.writer.write(frame)

    def _field_to_pixel(self, pts_xy: np.ndarray, h_inv: np.ndarray) -> np.ndarray:
        if pts_xy.size == 0:
            return np.empty((0, 2), dtype=np.int32)
        src = pts_xy.reshape(-1, 1, 2).astype(np.float32)
        mapped = cv2.perspectiveTransform(src, h_inv)
        return mapped.reshape(-1, 2).astype(np.int32)

    def _draw_overlay(self, frame, h_inv, trails, targets):
        if targets:
            tgt_px = self._field_to_pixel(
                np.array(targets, dtype=np.float32), h_inv,
            )
            for (x, y) in tgt_px:
                # Black outline + white fill so targets read against both
                # bright and dark patches of the field.
                cv2.drawMarker(frame, (int(x), int(y)), (0, 0, 0),
                               markerType=cv2.MARKER_TILTED_CROSS,
                               markerSize=24, thickness=4)
                cv2.drawMarker(frame, (int(x), int(y)), (255, 255, 255),
                               markerType=cv2.MARKER_TILTED_CROSS,
                               markerSize=24, thickness=2)

        for rid, pts in trails.items():
            if not pts:
                continue
            pix = self._field_to_pixel(
                np.array(pts, dtype=np.float32), h_inv,
            )
            color = self._colors.get(rid, (200, 200, 200))
            if len(pix) >= 2:
                cv2.polylines(frame, [pix], isClosed=False, color=color,
                              thickness=2, lineType=cv2.LINE_AA)
            cx, cy = int(pix[-1, 0]), int(pix[-1, 1])
            cv2.circle(frame, (cx, cy), 5, color, -1, lineType=cv2.LINE_AA)
            cv2.putText(frame, f"r{rid}", (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                        lineType=cv2.LINE_AA)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"video_recorder: closed {self.video_path}")
            self.writer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
