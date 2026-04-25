"""
Per-episode CSV logger for belief-system hardware experiments.

Subscribes to every robot's `/field/robot_{id}/pose`, the scalar
`/global_reward`, `/target_markers` (once, to cache target positions),
and per-robot belief telemetry (uncertainty mean, last-sample weight),
then writes one CSV row per sample tick:

    t,
    {robot_{id}_x, robot_{id}_y, robot_{id}_pose_age_s,
     robot_{id}_belief_unc, robot_{id}_sample_weight}*,
    reward

`pose_age_s` = (logger receive time − message header.stamp) cached at
the latest pose receipt — captures the apriltag → field_localizer →
pose-publish pipeline latency. Spikes here are the most reliable
real-system robustness signal we have without instrumenting upstream.

`belief_unc` and `sample_weight` are the most recent scalars from the
per-robot belief node; held until a fresher value arrives.

A JSON sidecar `{scenario}.json` records the robot IDs, target
positions, and field frame so the offline plotter can overlay targets
without re-deriving the formation.

Shuts the node down after `duration_sec` if set (>0); set it to 0 to
run until the process is killed. Setting `duration_sec > 0` lets launch
files use `on_exit=Shutdown()` to tear the whole bundle down when an
episode ends.
"""

import csv
import json
import os
import pathlib

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryLoggerNode(Node):
    def __init__(self):
        super().__init__("trajectory_logger")

        self.declare_parameter("robot_ids", [0, 1, 2])
        self.declare_parameter("log_dir", "/ros_ws/experiment_logs")
        self.declare_parameter("scenario", "run")
        self.declare_parameter("duration_sec", 0.0)
        self.declare_parameter("sample_hz", 10.0)
        self.declare_parameter("reward_topic", "/global_reward")
        self.declare_parameter("markers_topic", "/target_markers")
        self.declare_parameter("field_frame", "field")
        # Field extent in metres — stamped into the JSON sidecar so the
        # offline plotter can draw full-field axes regardless of where
        # the robots actually explored.
        self.declare_parameter("field_x_min", 0.0)
        self.declare_parameter("field_x_max", 3.0)
        self.declare_parameter("field_y_min", 0.0)
        self.declare_parameter("field_y_max", 3.0)

        self.robot_ids = [int(v) for v in self.get_parameter("robot_ids").value]
        self.log_dir = pathlib.Path(self.get_parameter("log_dir").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.duration = float(self.get_parameter("duration_sec").value)
        self.sample_period = 1.0 / float(self.get_parameter("sample_hz").value)
        self.field_frame = str(self.get_parameter("field_frame").value)
        self.field_bounds = {
            "x_min": float(self.get_parameter("field_x_min").value),
            "x_max": float(self.get_parameter("field_x_max").value),
            "y_min": float(self.get_parameter("field_y_min").value),
            "y_max": float(self.get_parameter("field_y_max").value),
        }

        self.log_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.log_dir / f"{self.scenario}.csv"
        self.json_path = self.log_dir / f"{self.scenario}.json"

        self.positions: dict[int, tuple[float, float] | None] = {
            rid: None for rid in self.robot_ids
        }
        # Most recent pose-pipeline latency per robot (s). Computed at
        # receipt as (logger_recv - msg.header.stamp). NaN until first
        # pose arrives. Captures the apriltag→pose end-to-end delay.
        self.pose_age_s: dict[int, float] = {rid: float("nan") for rid in self.robot_ids}
        # Most recent per-robot belief telemetry from belief_node. Cached
        # so the sample tick can dump a value even when these arrive at
        # a slower cadence than sample_hz.
        self.belief_unc: dict[int, float] = {rid: float("nan") for rid in self.robot_ids}
        self.sample_weight: dict[int, float] = {rid: float("nan") for rid in self.robot_ids}
        self.reward: float | None = None
        self.targets: list[tuple[float, float]] | None = None
        self._sidecar_written = False

        self.csv_file = open(csv_path, "w", newline="")
        cols = ["t"]
        for rid in self.robot_ids:
            cols += [
                f"robot_{rid}_x",
                f"robot_{rid}_y",
                f"robot_{rid}_pose_age_s",
                f"robot_{rid}_belief_unc",
                f"robot_{rid}_sample_weight",
            ]
        cols += ["reward"]
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(cols)

        for rid in self.robot_ids:
            self.create_subscription(
                PoseStamped,
                f"/field/robot_{rid}/pose",
                lambda msg, r=rid: self._on_pose(msg, r),
                10,
            )
            # Belief telemetry — best-effort: if a belief node isn't up
            # for this robot id, the columns just stay NaN.
            self.create_subscription(
                Float32,
                f"/robot_{rid}/belief/uncertainty_mean",
                lambda msg, r=rid: self._on_belief_unc(msg, r),
                10,
            )
            self.create_subscription(
                Float32,
                f"/robot_{rid}/belief/sample_weight",
                lambda msg, r=rid: self._on_sample_weight(msg, r),
                10,
            )
        self.create_subscription(
            Float32, str(self.get_parameter("reward_topic").value),
            self._on_reward, 10,
        )
        self.create_subscription(
            MarkerArray, str(self.get_parameter("markers_topic").value),
            self._on_markers, 10,
        )

        self.t0 = self._now_sec()
        self.create_timer(self.sample_period, self._sample_tick)

        self.get_logger().info(
            f"trajectory_logger: scenario='{self.scenario}', "
            f"duration={self.duration}s "
            f"({'bounded' if self.duration > 0 else 'unbounded'}), "
            f"writing {csv_path}"
        )

    def _now_sec(self) -> float:
        msg = self.get_clock().now().to_msg()
        return msg.sec + msg.nanosec * 1e-9

    def _on_pose(self, msg: PoseStamped, rid: int):
        self.positions[rid] = (msg.pose.position.x, msg.pose.position.y)
        # Pose-pipeline latency: gap between when the upstream node
        # stamped the pose and when this logger received it. Negative
        # values (clock skew across nodes on the same host shouldn't
        # happen, but guard) become NaN — we'd rather flag a bad sample
        # than report fake near-zero latency.
        stamp = msg.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1e-9
        if stamp_s <= 0.0:
            # Upstream forgot to set the stamp — record NaN so the
            # plot makes the missing-instrumentation case obvious.
            self.pose_age_s[rid] = float("nan")
            return
        age = self._now_sec() - stamp_s
        self.pose_age_s[rid] = age if age >= 0.0 else float("nan")

    def _on_belief_unc(self, msg: Float32, rid: int):
        self.belief_unc[rid] = float(msg.data)

    def _on_sample_weight(self, msg: Float32, rid: int):
        self.sample_weight[rid] = float(msg.data)

    def _on_reward(self, msg: Float32):
        self.reward = float(msg.data)

    def _on_markers(self, msg: MarkerArray):
        if self.targets is not None:
            return
        # global_feedback publishes a CYLINDER per target with
        # ns='formation_targets'. Text labels share the ns but are
        # TEXT_VIEW_FACING — filter to the cylinders to avoid duplicates.
        targets = [
            (float(m.pose.position.x), float(m.pose.position.y))
            for m in msg.markers
            if m.ns == "formation_targets" and m.type == Marker.CYLINDER
        ]
        if not targets:
            return
        self.targets = targets
        self._write_sidecar()

    def _write_sidecar(self):
        if self._sidecar_written or self.targets is None:
            return
        payload = {
            "scenario": self.scenario,
            "robot_ids": self.robot_ids,
            "targets": [[x, y] for (x, y) in self.targets],
            "field_frame": self.field_frame,
            "field_bounds": self.field_bounds,
        }
        self.json_path.write_text(json.dumps(payload, indent=2))
        self._sidecar_written = True
        self.get_logger().info(
            f"trajectory_logger: cached {len(self.targets)} target(s) -> {self.json_path}"
        )

    def _sample_tick(self):
        t = self._now_sec() - self.t0
        if self.duration > 0.0 and t > self.duration:
            self.get_logger().info(
                f"trajectory_logger: duration reached ({self.duration:.1f}s), exiting."
            )
            self.csv_file.flush()
            self.csv_file.close()
            os._exit(0)

        # Empty cells (vs "nan") preserve the existing convention for
        # uninitialised pose data; NaN-bearing scalars (pose_age, etc.)
        # write as "nan" so pandas reads them back as float NaN.
        def _fmt(v: float) -> str:
            import math
            return "" if v is None else ("nan" if math.isnan(v) else f"{v:.4f}")

        row: list[str] = [f"{t:.3f}"]
        for rid in self.robot_ids:
            p = self.positions[rid]
            if p is None:
                row += ["", ""]
            else:
                row += [f"{p[0]:.4f}", f"{p[1]:.4f}"]
            row += [
                _fmt(self.pose_age_s[rid]),
                _fmt(self.belief_unc[rid]),
                _fmt(self.sample_weight[rid]),
            ]
        row.append("" if self.reward is None else f"{self.reward:.4f}")
        self.writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.csv_file.flush()
        node.csv_file.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
