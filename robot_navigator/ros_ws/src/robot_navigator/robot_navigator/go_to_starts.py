"""
go_to_starts — drive every currently-visible RoboMaster to a preset start pose.

Snapshots `/field/robot_*/pose` at startup to see which cars are on the mat,
assigns each to the closest free target from a CLI-selected shape, and
publishes `/robot_{id}/goal_pose` until all arrive (or timeout). Collision
avoidance between robots is handled by the already-running RobotNavigator
instances — this script does NOT launch them. Run `./run_navigator.sh <ids>`
first so peer repulsion is wired up.
"""

import argparse
import math
import re
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


FIELD_SIZE_M = 3.0
# Stay off the corner AprilTags — parking on them blocks the homography.
CORNER_INSET_M = 0.4
FIELD_TOPIC_RE = re.compile(r"^/field/robot_(\d+)/pose$")

STATE_DISCOVER = "DISCOVER"
STATE_ASSIGN = "ASSIGN"
STATE_DRIVE = "DRIVE"
STATE_COMPLETE = "COMPLETE"

STATUS_ARRIVED = "ARRIVED"
STATUS_TIMEOUT = "TIMEOUT"
STATUS_LOST = "LOST"
STATUS_UNASSIGNED = "UNASSIGNED"


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _rotate(px: float, py: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return c * px - s * py, s * px + c * py


def _shape_targets(shape: str, center: tuple[float, float],
                   spacing: float, yaw: float) -> list[tuple[float, float]]:
    """Generate target (x, y) points in field frame for the requested shape."""
    if shape == "corners":
        i = CORNER_INSET_M
        return [(i, i), (FIELD_SIZE_M - i, i),
                (FIELD_SIZE_M - i, FIELD_SIZE_M - i), (i, FIELD_SIZE_M - i)]

    if shape == "line":
        local = [(-1.5 * spacing, 0.0), (-0.5 * spacing, 0.0),
                 (0.5 * spacing, 0.0), (1.5 * spacing, 0.0)]
    elif shape == "triangle":
        r = spacing / math.sqrt(3.0)
        local = [(0.0, r),
                 (-spacing / 2.0, -spacing / (2.0 * math.sqrt(3.0))),
                 (spacing / 2.0, -spacing / (2.0 * math.sqrt(3.0)))]
    elif shape == "circle":
        local = [(spacing * math.cos(2.0 * math.pi * k / 4.0),
                  spacing * math.sin(2.0 * math.pi * k / 4.0))
                 for k in range(4)]
    else:
        raise ValueError(f"unknown shape: {shape}")

    cx, cy = center
    out = []
    for px, py in local:
        rx, ry = _rotate(px, py, yaw)
        out.append((cx + rx, cy + ry))
    return out


def _clamp_targets(targets: list[tuple[float, float]],
                   margin: float) -> tuple[list[tuple[float, float]], bool]:
    lo, hi = margin, FIELD_SIZE_M - margin
    clamped = False
    out = []
    for x, y in targets:
        cx = min(hi, max(lo, x))
        cy = min(hi, max(lo, y))
        if cx != x or cy != y:
            clamped = True
        out.append((cx, cy))
    return out, clamped


def _any_too_close(targets: list[tuple[float, float]], min_sep: float) -> bool:
    for i in range(len(targets)):
        for j in range(i + 1, len(targets)):
            dx = targets[i][0] - targets[j][0]
            dy = targets[i][1] - targets[j][1]
            if math.hypot(dx, dy) < min_sep:
                return True
    return False


def _assign(cost: list[list[float]]) -> list[tuple[int, int]]:
    """Return (row, col) pairs minimizing total cost. Hungarian w/ greedy fallback."""
    if not cost or not cost[0]:
        return []
    try:
        import numpy as np
        from scipy.optimize import linear_sum_assignment
        row_ind, col_ind = linear_sum_assignment(np.array(cost))
        return list(zip(row_ind.tolist(), col_ind.tolist()))
    except ImportError:
        return _greedy_nearest(cost)


def _greedy_nearest(cost: list[list[float]]) -> list[tuple[int, int]]:
    n_rows, n_cols = len(cost), len(cost[0])
    used_rows: set[int] = set()
    used_cols: set[int] = set()
    pairs: list[tuple[int, int]] = []
    k = min(n_rows, n_cols)
    for _ in range(k):
        best = None
        best_c = math.inf
        for i in range(n_rows):
            if i in used_rows:
                continue
            for j in range(n_cols):
                if j in used_cols:
                    continue
                if cost[i][j] < best_c:
                    best_c = cost[i][j]
                    best = (i, j)
        if best is None:
            break
        pairs.append(best)
        used_rows.add(best[0])
        used_cols.add(best[1])
    return pairs


def _parse_center(s: str) -> tuple[float, float]:
    parts = s.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("center must be 'X,Y'")
    return float(parts[0]), float(parts[1])


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="go_to_starts",
        description=(
            "Drive every currently-visible RoboMaster to a preset start pose. "
            "Run ./run_navigator.sh <ids> first so collision avoidance is wired up. "
            "--align-heading only has an effect if navigators were launched with "
            "align_heading:=true (default is false)."
        ),
    )
    p.add_argument("--shape", required=True,
                   choices=["corners", "line", "triangle", "circle"])
    p.add_argument("--center", type=_parse_center, default=(1.5, 1.5),
                   help="X,Y in meters; ignored by 'corners' (default 1.5,1.5)")
    p.add_argument("--spacing", type=float, default=1.0,
                   help="meters; step for line, side for triangle, radius for circle")
    p.add_argument("--yaw", type=float, default=0.0,
                   help="shape rotation in degrees (default 0)")
    p.add_argument("--align-heading", action="store_true")
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--tolerance", type=float, default=0.05,
                   help="meters; matches navigator's goal_tolerance default")
    p.add_argument("--discover-duration", type=float, default=1.5)
    p.add_argument("--arrival-dwell", type=float, default=0.5)
    p.add_argument("--goal-republish-period", type=float, default=1.0)
    p.add_argument("--field-margin", type=float, default=0.1,
                   help="keep non-corner targets at least this far from field edge")
    p.add_argument("--lost-timeout", type=float, default=3.0)
    p.add_argument("--dry-run", action="store_true",
                   help="compute + log plan, publish nothing")
    p.add_argument("--no-avoidance", action="store_true",
                   help="publish Bool(false) once to /collision_avoidance_enabled")
    return p


class GoToStarts(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("go_to_starts")
        self.args = args
        self.state = STATE_DISCOVER

        self._discover_start = time.monotonic()
        self._discover_duration = float(args.discover_duration)
        self._discover_retried = False
        self._discovery_subs: dict[int, object] = {}
        self._seen: dict[int, tuple[float, float, float]] = {}

        self._targets: list[tuple[float, float]] = []
        self._assignments: dict[int, tuple[float, float]] = {}
        self._unassigned_ids: list[int] = []
        self._starts: dict[int, tuple[float, float]] = {}

        self._goal_pubs: dict[int, object] = {}
        self._estimate_subs: dict[int, object] = {}
        self._last_estimate: dict[int, tuple[float, float, float]] = {}
        self._in_tol_since: dict[int, float | None] = {}
        self._arrived: dict[int, bool] = {}
        self._last_goal_pub: float = 0.0
        self._drive_start: float | None = None

        self._avoid_pub = self.create_publisher(Bool, "/collision_avoidance_enabled", 10)
        self._published_avoidance_off = False

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"go_to_starts — shape={args.shape} center={args.center} "
            f"spacing={args.spacing} yaw={args.yaw}° timeout={args.timeout}s "
            f"dry_run={args.dry_run}"
        )

    # ------------------------------------------------------------------ #
    # State machine
    # ------------------------------------------------------------------ #
    def _tick(self):
        if self.state == STATE_DISCOVER:
            self._tick_discover()
        elif self.state == STATE_ASSIGN:
            self._tick_assign()
        elif self.state == STATE_DRIVE:
            self._tick_drive()
        elif self.state == STATE_COMPLETE:
            self._finish()

    # ------------------------------------------------------------------ #
    # DISCOVER
    # ------------------------------------------------------------------ #
    def _tick_discover(self):
        self._scan_graph_for_robots()

        elapsed = time.monotonic() - self._discover_start
        if elapsed < self._discover_duration:
            return

        if not self._seen:
            if not self._discover_retried:
                self._discover_retried = True
                self._discover_start = time.monotonic()
                self._discover_duration *= 2.0
                self.get_logger().warn(
                    "No robots seen yet — retrying discovery with "
                    f"{self._discover_duration:.1f}s window."
                )
                return
            self.get_logger().error(
                "No robots publishing on /field/robot_*/pose — "
                "is the localizer running? exiting."
            )
            sys.exit(2)

        self.state = STATE_ASSIGN

    def _scan_graph_for_robots(self):
        for name, types in self.get_topic_names_and_types():
            m = FIELD_TOPIC_RE.match(name)
            if not m:
                continue
            if "geometry_msgs/msg/PoseStamped" not in types:
                continue
            rid = int(m.group(1))
            if rid in self._discovery_subs:
                continue
            sub = self.create_subscription(
                PoseStamped, name,
                lambda msg, i=rid: self._on_discovery_pose(i, msg), 10,
            )
            self._discovery_subs[rid] = sub

    def _on_discovery_pose(self, rid: int, msg: PoseStamped):
        self._seen[rid] = (msg.pose.position.x, msg.pose.position.y, time.monotonic())

    # ------------------------------------------------------------------ #
    # ASSIGN
    # ------------------------------------------------------------------ #
    def _tick_assign(self):
        self._starts = {rid: (x, y) for rid, (x, y, _) in self._seen.items()}

        yaw_rad = math.radians(self.args.yaw)
        raw = _shape_targets(self.args.shape, self.args.center,
                             self.args.spacing, yaw_rad)
        if self.args.shape == "corners":
            targets = raw
            clamped = False
        else:
            targets, clamped = _clamp_targets(raw, self.args.field_margin)
        if clamped:
            self.get_logger().warn(
                "One or more targets were clamped to stay inside the field."
            )
        if _any_too_close(targets, 2.0 * self.args.tolerance):
            self.get_logger().error(
                "Two targets are closer than 2x tolerance after clamping — "
                "check --spacing / --center / --field-margin. Aborting."
            )
            sys.exit(3)
        self._targets = targets

        ids_sorted = sorted(self._starts.keys())
        cost = [
            [math.hypot(self._starts[rid][0] - tx, self._starts[rid][1] - ty)
             for (tx, ty) in targets]
            for rid in ids_sorted
        ]
        pairs = _assign(cost)

        assigned_rows: set[int] = set()
        for i, j in pairs:
            rid = ids_sorted[i]
            self._assignments[rid] = targets[j]
            assigned_rows.add(i)
        self._unassigned_ids = [ids_sorted[i] for i in range(len(ids_sorted))
                                if i not in assigned_rows]

        self._log_assignment()

        if self.args.dry_run:
            self.get_logger().info("--dry-run set; exiting without publishing goals.")
            self.state = STATE_COMPLETE
            return

        if self.args.no_avoidance and not self._published_avoidance_off:
            self._avoid_pub.publish(Bool(data=False))
            self._published_avoidance_off = True
            self.get_logger().warn(
                "Published Bool(false) on /collision_avoidance_enabled."
            )

        for rid in self._assignments:
            self._in_tol_since[rid] = None
            self._arrived[rid] = False
            est_sub = self.create_subscription(
                PoseStamped, f"/robot_{rid}/estimated_pose",
                lambda msg, i=rid: self._on_estimate(i, msg), 10,
            )
            self._estimate_subs[rid] = est_sub
            self._goal_pubs[rid] = self.create_publisher(
                PoseStamped, f"/robot_{rid}/goal_pose", 10,
            )

        self._drive_start = time.monotonic()
        self._publish_all_goals()
        self._last_goal_pub = time.monotonic()
        self.state = STATE_DRIVE

    def _log_assignment(self):
        self.get_logger().info(
            f"Detected {len(self._starts)} robot(s); "
            f"{len(self._targets)} target(s) available; "
            f"assigned {len(self._assignments)}."
        )
        for rid, (tx, ty) in sorted(self._assignments.items()):
            sx, sy = self._starts[rid]
            d = math.hypot(sx - tx, sy - ty)
            self.get_logger().info(
                f"  robot_{rid}: ({sx:.2f},{sy:.2f}) -> "
                f"({tx:.2f},{ty:.2f}) dist={d:.2f}m"
            )
        if self._unassigned_ids:
            self.get_logger().warn(
                "Unassigned (not enough targets): " +
                ", ".join(f"robot_{r}" for r in self._unassigned_ids)
            )

    # ------------------------------------------------------------------ #
    # DRIVE
    # ------------------------------------------------------------------ #
    def _tick_drive(self):
        now = time.monotonic()

        if now - self._last_goal_pub >= self.args.goal_republish_period:
            self._publish_all_goals()
            self._last_goal_pub = now

        if not self.args.no_avoidance:
            self._avoid_pub.publish(Bool(data=True))

        for rid, target in self._assignments.items():
            if self._arrived[rid]:
                continue
            est = self._last_estimate.get(rid)
            if est is None:
                continue
            ex, ey, _ = est
            tx, ty = target
            d = math.hypot(ex - tx, ey - ty)
            if d < self.args.tolerance:
                if self._in_tol_since[rid] is None:
                    self._in_tol_since[rid] = now
                elif now - self._in_tol_since[rid] >= self.args.arrival_dwell:
                    self._arrived[rid] = True
                    self.get_logger().info(f"robot_{rid} ARRIVED (err={d:.3f}m)")
            else:
                self._in_tol_since[rid] = None

        if all(self._arrived[rid] for rid in self._assignments):
            self.state = STATE_COMPLETE
            return

        if now - self._drive_start > self.args.timeout:
            self.get_logger().warn(
                f"Timeout {self.args.timeout:.1f}s reached — aborting drive."
            )
            self.state = STATE_COMPLETE

    def _publish_all_goals(self):
        stamp = self.get_clock().now().to_msg()
        yaw_rad = math.radians(self.args.yaw)
        qx, qy, qz, qw = _quat_from_yaw(yaw_rad)
        for rid, (tx, ty) in self._assignments.items():
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "field"
            msg.pose.position.x = tx
            msg.pose.position.y = ty
            if self.args.align_heading:
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw
            else:
                msg.pose.orientation.w = 1.0
            self._goal_pubs[rid].publish(msg)

    def _on_estimate(self, rid: int, msg: PoseStamped):
        self._last_estimate[rid] = (
            msg.pose.position.x, msg.pose.position.y, time.monotonic(),
        )

    # ------------------------------------------------------------------ #
    # COMPLETE
    # ------------------------------------------------------------------ #
    def _finish(self):
        now = time.monotonic()
        rows: list[tuple[int, tuple[float, float], tuple[float, float] | None,
                         tuple[float, float] | None, float | None, str]] = []

        for rid, target in sorted(self._assignments.items()):
            start = self._starts[rid]
            est = self._last_estimate.get(rid)
            if est is None:
                final = None
                err = None
            else:
                final = (est[0], est[1])
                err = math.hypot(est[0] - target[0], est[1] - target[1])

            if self._arrived.get(rid):
                status = STATUS_ARRIVED
            elif est is None or now - est[2] > self.args.lost_timeout:
                status = STATUS_LOST
            else:
                status = STATUS_TIMEOUT
            rows.append((rid, start, target, final, err, status))

        for rid in self._unassigned_ids:
            start = self._starts[rid]
            rows.append((rid, start, None, None, None, STATUS_UNASSIGNED))

        self.get_logger().info("---- final status ----")
        self.get_logger().info(
            f"{'id':<6} {'start':<14} {'target':<14} "
            f"{'final':<14} {'err':<8} status"
        )
        for rid, start, target, final, err, status in rows:
            s = f"({start[0]:.2f},{start[1]:.2f})"
            t = f"({target[0]:.2f},{target[1]:.2f})" if target else "-"
            f = f"({final[0]:.2f},{final[1]:.2f})" if final else "-"
            e = f"{err:.3f}" if err is not None else "-"
            self.get_logger().info(
                f"{rid:<6} {s:<14} {t:<14} {f:<14} {e:<8} {status}"
            )

        rclpy.shutdown()


def main(args=None):
    parser = _build_parser()
    cli_args, ros_args = parser.parse_known_args(args if args is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    node = GoToStarts(cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
