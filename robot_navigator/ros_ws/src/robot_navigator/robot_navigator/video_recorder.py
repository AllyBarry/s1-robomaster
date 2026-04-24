"""
Records a bird's-eye, field-rectified camera stream with trajectory overlays.

Each incoming source frame is warped so that the four field corner tags
land exactly at the frame corners — the output is cropped to the field
boundary and the perspective distortion is removed. Trails and target
crosses are drawn *after* warping, directly in rectified pixel space, so
they stay crisp instead of being skewed with the underlying frame.

Subscribes to the *rectified* raw camera image (so detection_overlay's
burnt-in AprilTag labels don't get warped into smears), the inverse
homography published by field_localizer, each robot's
`/field/robot_{id}/pose`, and `/target_markers`. Output drops into the
run folder as `{scenario}.mp4` alongside the CSV, JSON sidecar, and
`plots/` directory.
"""

import atexit
import collections
import pathlib
import signal
import subprocess
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

# Jetson's apt opencv often routes cv2.VideoWriter through GStreamer with
# missing encoder plugins — it "opens" successfully but writes malformed
# files. Piping raw BGR frames to an ffmpeg subprocess bypasses opencv's
# encoder lookup entirely; as long as the binary is in PATH (installed
# via the Dockerfile) the output is guaranteed to be a valid H.264 mp4.
_FFMPEG_BIN = "ffmpeg"


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__("video_recorder")

        self.declare_parameter("robot_ids", [0, 1, 2])
        self.declare_parameter("log_dir", "/ros_ws/experiment_logs")
        self.declare_parameter("scenario", "run")
        self.declare_parameter("image_topic", "/webcam/image_rect")
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
        self.writer: subprocess.Popen | None = None
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

        # /webcam/image_rect streams at the full camera rate (~30 Hz);
        # output mp4 is 5 fps. Drop anything arriving within one frame
        # period of the last write so ffmpeg sees a real-time stream
        # instead of a 6x speedup.
        self._min_write_dt = 1.0 / self.fps if self.fps > 0 else 0.0
        self._last_write_ts: float | None = None

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
        with self._lock:
            self._frames_received += 1

        # Rate-limit before doing anything expensive (cv_bridge, warp).
        # Use msg stamp when available so the recorded video matches
        # wall-clock pacing even if a source burst catches up.
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if now <= 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
        if (
            self._last_write_ts is not None
            and (now - self._last_write_ts) < self._min_write_dt
        ):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge convert failed: {e}")
            return

        with self._lock:
            M = None if self.warp_M is None else self.warp_M.copy()
            trails = {rid: list(pts) for rid, pts in self.trails.items()}
            targets = list(self.targets) if self.targets else []

        # Without a homography we can't rectify — drop the frame rather
        # than record a raw-perspective segment that won't line up with
        # the rest of the recording.
        if M is None:
            return

        # Rectify first, THEN overlay: keeps trails/targets/text crisp
        # instead of being stretched by the perspective warp.
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
        self._last_write_ts = now
        try:
            self.writer.stdin.write(warped.tobytes())
        except (BrokenPipeError, ValueError):
            # ffmpeg died — surface stderr once so the failure isn't silent.
            stderr = b""
            try:
                stderr = self.writer.stderr.read() or b""
            except Exception:
                pass
            self.get_logger().error(
                f"ffmpeg pipe closed unexpectedly — disabling recording. "
                f"stderr: {stderr.decode(errors='replace')[:500]}"
            )
            self.writer = None
            return
        self._frames_written += 1

    def _open_writer(self) -> bool:
        path = self.video_stem.with_suffix(".mp4")
        cmd = [
            _FFMPEG_BIN, "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{self.out_w}x{self.out_h}",
            "-r", f"{self.fps}",
            "-i", "-",
            "-an",
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            str(path),
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError:
            self.get_logger().error(
                f"'{_FFMPEG_BIN}' not found on PATH — rebuild the image "
                "(Dockerfile installs ffmpeg)"
            )
            return False

        self.writer = proc
        self.frame_size = (self.out_w, self.out_h)
        self.video_path = path
        self.get_logger().info(
            f"video_recorder: piping {self.out_w}x{self.out_h} @ {self.fps} fps "
            f"to ffmpeg libx264 -> {path}"
        )
        return True

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
                               markerSize=28, thickness=5,
                               line_type=cv2.LINE_AA)
                cv2.drawMarker(frame, (int(x), int(y)), (255, 255, 255),
                               markerType=cv2.MARKER_TILTED_CROSS,
                               markerSize=28, thickness=2,
                               line_type=cv2.LINE_AA)

        for rid, pts in trails.items():
            if not pts:
                continue
            pix = self._field_to_pixel(np.array(pts, dtype=np.float32))
            color = self._colors.get(rid, (200, 200, 200))
            # Dark halo under the trail so coloured lines stay readable
            # over the field's lighter patches.
            if len(pix) >= 2:
                cv2.polylines(frame, [pix], isClosed=False, color=(0, 0, 0),
                              thickness=5, lineType=cv2.LINE_AA)
                cv2.polylines(frame, [pix], isClosed=False, color=color,
                              thickness=3, lineType=cv2.LINE_AA)
            cx, cy = int(pix[-1, 0]), int(pix[-1, 1])
            cv2.circle(frame, (cx, cy), 9, (0, 0, 0), -1, lineType=cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 7, color, -1, lineType=cv2.LINE_AA)
            label = f"r{rid}"
            lx, ly = cx + 10, cy - 10
            cv2.putText(frame, label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 4, lineType=cv2.LINE_AA)
            cv2.putText(frame, label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2, lineType=cv2.LINE_AA)

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
        # Closing stdin tells ffmpeg EOF; wait() lets it finalize the mp4
        # moov atom before we return. Idempotent — safe to call from both
        # the atexit hook and destroy_node().
        if self.writer is None:
            return
        proc = self.writer
        self.writer = None
        try:
            if proc.stdin and not proc.stdin.closed:
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                "ffmpeg did not exit within 10s — killing (file may be truncated)"
            )
            proc.kill()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
        rc = proc.returncode
        if rc == 0:
            self.get_logger().info(
                f"video_recorder: closed {self.video_path} "
                f"({self._frames_written} frames)"
            )
        else:
            stderr = b""
            try:
                stderr = proc.stderr.read() or b""
            except Exception:
                pass
            self.get_logger().error(
                f"ffmpeg exited rc={rc} — {self.video_path} may be incomplete. "
                f"stderr: {stderr.decode(errors='replace')[:500]}"
            )

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
