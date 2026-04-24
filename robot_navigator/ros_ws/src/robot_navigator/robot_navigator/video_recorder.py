"""
Records a bird's-eye, field-rectified camera stream with trajectory overlays.

Each incoming source frame is warped so that the four field corner tags
land exactly at the frame corners — the output is cropped to the field
boundary and the perspective distortion is removed. Trails and target
crosses are drawn in the rectified image directly in field metres, so
they line up with the video regardless of camera mounting angle.

Subscribes to the apriltag debug image, the inverse homography published
by field_localizer (used to derive the source→rectified warp), each
robot's `/field/robot_{id}/pose`, and `/target_markers`. Output drops
into the run folder as `{scenario}.mp4` alongside the CSV, JSON
sidecar, and `plots/` directory.
"""

import atexit
import collections
import pathlib
import signal
import sys
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

# Ordered codec fallback: MJPG/.avi first because it's the one combo that
# reliably works across Jetson apt opencv builds — avc1/mp4v both pass
# cv2.VideoWriter.isOpened() even when the underlying h264/mpeg4 encoder
# is missing or broken, producing "opened but garbage" files. Trading the
# .mp4 extension for a file that actually plays is the right default.
_CODEC_CHAIN = [
    ("MJPG", ".avi"),
    ("mp4v", ".mp4"),
    ("avc1", ".mp4"),
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
        self.declare_parameter("field_width", 3.0)
        self.declare_parameter("field_height", 3.0)
        self.declare_parameter("output_width", 600)

        self.robot_ids = [int(v) for v in self.get_parameter("robot_ids").value]
        self.log_dir = pathlib.Path(self.get_parameter("log_dir").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.fps = float(self.get_parameter("fps").value)
        trail_len = int(self.get_parameter("trail_length").value)
        image_topic = str(self.get_parameter("image_topic").value)
        hom_topic = str(self.get_parameter("homography_topic").value)
        markers_topic = str(self.get_parameter("markers_topic").value)
        self.field_w = float(self.get_parameter("field_width").value)
        self.field_h = float(self.get_parameter("field_height").value)
        self.out_w = int(self.get_parameter("output_width").value)
        # Preserve field aspect ratio so metres are isotropic on screen.
        self.out_h = int(round(self.out_w * self.field_h / self.field_w))

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.video_stem = self.log_dir / self.scenario
        self.video_path: pathlib.Path | None = None

        self.bridge = CvBridge()
        self.writer: cv2.VideoWriter | None = None
        self.frame_size: tuple[int, int] | None = None
        # M maps source pixel -> rectified output pixel (field-aligned).
        # Updated whenever field_localizer republishes the homography.
        self.warp_M: np.ndarray | None = None
        self.targets: list[tuple[float, float]] | None = None

        # Field metres -> rectified pixel (linear, no homography needed
        # once the frame is warped). y is flipped so field +Y is at the
        # top of the frame, matching how the operator sees the arena.
        self._sx = self.out_w / self.field_w
        self._sy = self.out_h / self.field_h
        self._T_field_to_pixel = np.array([
            [self._sx, 0.0,        0.0],
            [0.0,     -self._sy,   float(self.out_h)],
            [0.0,      0.0,        1.0],
        ], dtype=np.float32)

        maxlen = trail_len if trail_len > 0 else None
        self.trails: dict[int, collections.deque] = {
            rid: collections.deque(maxlen=maxlen) for rid in self.robot_ids
        }
        self._colors = {
            rid: _PALETTE_BGR[i % len(_PALETTE_BGR)]
            for i, rid in enumerate(self.robot_ids)
        }
        self._lock = threading.Lock()

        # Diagnostic counters — surface what's actually arriving so a
        # silent "no output file" failure is debuggable from the logs
        # instead of from guessing which upstream topic is dead.
        self._frames_received = 0
        self._frames_written = 0
        self._homography_updates = 0

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

        self.create_timer(5.0, self._log_status)

        self.get_logger().info(
            f"video_recorder: image='{image_topic}', output_stem={self.video_stem}, "
            f"fps={self.fps}, robots={self.robot_ids}, "
            f"rectified_size={self.out_w}x{self.out_h} "
            f"(field {self.field_w}x{self.field_h} m)"
        )

    def _on_homography(self, msg: Float32MultiArray):
        if len(msg.data) != 9:
            return
        h_inv = np.array(msg.data, dtype=np.float32).reshape(3, 3)
        try:
            # field_localizer publishes the *inverse* (field -> pixel);
            # for the warp we need pixel -> field, then field -> output.
            h = np.linalg.inv(h_inv).astype(np.float32)
        except np.linalg.LinAlgError:
            self.get_logger().warning("homography non-invertible — skipping update")
            return
        M = (self._T_field_to_pixel @ h).astype(np.float32)
        with self._lock:
            first = self.warp_M is None
            self.warp_M = M
            self._homography_updates += 1
        if first:
            self.get_logger().info("homography received — rectified recording enabled")

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

        with self._lock:
            self._frames_received += 1
            M = None if self.warp_M is None else self.warp_M.copy()
            trails = {rid: list(pts) for rid, pts in self.trails.items()}
            targets = list(self.targets) if self.targets else []

        # Without a homography we can't rectify — drop the frame rather
        # than record a raw-perspective segment that won't line up with
        # the rest of the recording.
        if M is None:
            return

        warped = cv2.warpPerspective(
            frame, M, (self.out_w, self.out_h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        if self.writer is None:
            if not self._open_writer():
                return

        self._draw_field_overlay(warped, trails, targets)
        self.writer.write(warped)
        self._frames_written += 1

    def _open_writer(self) -> bool:
        for fourcc_str, ext in _CODEC_CHAIN:
            path = self.video_stem.with_suffix(ext)
            fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
            writer = cv2.VideoWriter(
                str(path), fourcc, self.fps, (self.out_w, self.out_h)
            )
            if writer.isOpened():
                self.writer = writer
                self.frame_size = (self.out_w, self.out_h)
                self.video_path = path
                self.get_logger().info(
                    f"video_recorder: writing {self.out_w}x{self.out_h} "
                    f"@ {self.fps} fps [{fourcc_str}] -> {path}"
                )
                return True
            writer.release()
            self.get_logger().warning(
                f"codec {fourcc_str} ({ext}) unavailable — trying next"
            )
        self.get_logger().error(
            "no working VideoWriter codec — disabling recording"
        )
        return False

    def _field_to_pixel(self, pts_xy: np.ndarray) -> np.ndarray:
        if pts_xy.size == 0:
            return np.empty((0, 2), dtype=np.int32)
        arr = np.asarray(pts_xy, dtype=np.float32).reshape(-1, 2)
        px = arr[:, 0] * self._sx
        py = float(self.out_h) - arr[:, 1] * self._sy
        return np.stack([px, py], axis=1).astype(np.int32)

    def _draw_field_overlay(self, frame, trails, targets):
        if targets:
            tgt_px = self._field_to_pixel(np.array(targets, dtype=np.float32))
            for (x, y) in tgt_px:
                # Black outline + white fill so targets read against both
                # bright and dark patches of the rectified field.
                cv2.drawMarker(frame, (int(x), int(y)), (0, 0, 0),
                               markerType=cv2.MARKER_TILTED_CROSS,
                               markerSize=24, thickness=4)
                cv2.drawMarker(frame, (int(x), int(y)), (255, 255, 255),
                               markerType=cv2.MARKER_TILTED_CROSS,
                               markerSize=24, thickness=2)

        for rid, pts in trails.items():
            if not pts:
                continue
            pix = self._field_to_pixel(np.array(pts, dtype=np.float32))
            color = self._colors.get(rid, (200, 200, 200))
            if len(pix) >= 2:
                cv2.polylines(frame, [pix], isClosed=False, color=color,
                              thickness=2, lineType=cv2.LINE_AA)
            cx, cy = int(pix[-1, 0]), int(pix[-1, 1])
            cv2.circle(frame, (cx, cy), 5, color, -1, lineType=cv2.LINE_AA)
            cv2.putText(frame, f"r{rid}", (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                        lineType=cv2.LINE_AA)

    def _log_status(self):
        # Warn loudly if we've been up but nothing is flowing — most
        # common cause is field_localizer never seeing all 4 corner tags
        # (or the apriltag_detector container not having the
        # /field/homography_inv publisher from a stale build).
        if self._frames_received == 0:
            self.get_logger().warning(
                "no frames on image topic yet — check that detection_overlay "
                "is running and publishing /detections/image"
            )
            return
        if self._homography_updates == 0:
            self.get_logger().warning(
                f"received {self._frames_received} image frames but no "
                "/field/homography_inv yet — check that all 4 corner tags "
                "are visible and the apriltag_detector container has been rebuilt"
            )
            return
        self.get_logger().info(
            f"frames_in={self._frames_received} frames_written={self._frames_written} "
            f"H_updates={self._homography_updates} -> {self.video_path}"
        )

    def close_writer(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"video_recorder: closed {self.video_path}")
            self.writer = None

    def destroy_node(self):
        self.close_writer()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()

    # Two-layer safety net so the mp4 moov atom is always finalized:
    # (1) atexit runs on any Python-level exit (SystemExit, normal return,
    #     uncaught exception); (2) the SIGTERM handler converts docker's
    #     graceful-stop signal into SystemExit so atexit actually fires
    #     — rclpy's default signal handling covers SIGINT but not SIGTERM.
    atexit.register(node.close_writer)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_writer()
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
