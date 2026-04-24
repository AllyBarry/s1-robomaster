"""Minimal go_to_starts — discover visible cars, send to preset formation.

No frills: greedy nearest-target assignment, no dwell/LOST/clamp/dry-run.
Relies on already-running RobotNavigator instances for collision avoidance.
"""

import argparse
import math
import re
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


TOPIC_RE = re.compile(r"^/field/robot_(\d+)/pose$")

# Each shape is a fixed set of (x, y) targets in the 3x3m field. Hardcoded
# so a repeated test always places robots in the same spots. Corners are
# inset 0.4m to keep robots off the corner AprilTags (which the homography
# needs to see).
SHAPES = {
    "corners": [
        (0.4, 0.4),
        (2.6, 0.4),
        (2.6, 2.6),
        (0.4, 2.6),
    ],
    "line": [
        (0.50, 1.5),
        (1.17, 1.5),
        (1.83, 1.5),
        (2.50, 1.5),
    ],
    "triangle": [
        (1.5, 2.19),
        (0.9, 1.15),
        (2.1, 1.15),
    ],
    "circle": [
        (2.5, 1.5),
        (1.5, 2.5),
        (0.5, 1.5),
        (1.5, 0.5),
    ],
}


class GoLite(Node):
    def __init__(self, shape, timeout, tol):
        super().__init__("go_to_starts_lite")
        self.targets = SHAPES[shape]
        self.timeout = timeout
        self.tol = tol
        self.pose: dict[int, tuple[float, float]] = {}
        self.assigned: dict[int, tuple[float, float]] = {}
        self.pubs: dict[int, object] = {}
        self.arrived: set[int] = set()
        self.phase = "discover"
        self.t0 = time.monotonic()
        self.create_timer(0.2, self._tick)

    def _scan(self):
        for name, types in self.get_topic_names_and_types():
            m = TOPIC_RE.match(name)
            if m and "geometry_msgs/msg/PoseStamped" in types:
                rid = int(m.group(1))
                if rid not in self.pose:
                    self.pose[rid] = (0.0, 0.0)
                    self.create_subscription(
                        PoseStamped, name,
                        lambda msg, i=rid: self._on_pose(i, msg), 10,
                    )

    def _on_pose(self, rid, msg):
        self.pose[rid] = (msg.pose.position.x, msg.pose.position.y)

    def _greedy_assign(self):
        ids = list(self.pose.keys())
        taken = set()
        out = {}
        for _ in range(min(len(ids), len(self.targets))):
            best, best_d = None, math.inf
            for rid in ids:
                if rid in out:
                    continue
                for j, (tx, ty) in enumerate(self.targets):
                    if j in taken:
                        continue
                    d = math.hypot(self.pose[rid][0] - tx, self.pose[rid][1] - ty)
                    if d < best_d:
                        best_d, best = d, (rid, j)
            if best is None:
                break
            out[best[0]] = self.targets[best[1]]
            taken.add(best[1])
        return out

    def _tick(self):
        now = time.monotonic()
        self._scan()

        if self.phase == "discover":
            if now - self.t0 < 1.5:
                return
            self.assigned = self._greedy_assign()
            if not self.assigned:
                self.get_logger().error("no robots on /field/robot_*/pose")
                rclpy.shutdown()
                return
            for rid, (tx, ty) in self.assigned.items():
                self.pubs[rid] = self.create_publisher(
                    PoseStamped, f"/robot_{rid}/goal_pose", 10,
                )
                self.get_logger().info(f"robot_{rid} -> ({tx:.2f},{ty:.2f})")
            self.t0 = now
            self.phase = "drive"
            return

        for rid, (tx, ty) in self.assigned.items():
            msg = PoseStamped()
            msg.header.frame_id = "field"
            msg.pose.position.x = tx
            msg.pose.position.y = ty
            msg.pose.orientation.w = 1.0
            self.pubs[rid].publish(msg)
            px, py = self.pose[rid]
            if math.hypot(px - tx, py - ty) < self.tol and rid not in self.arrived:
                self.arrived.add(rid)
                self.get_logger().info(f"robot_{rid} arrived")

        if len(self.arrived) == len(self.assigned) or now - self.t0 > self.timeout:
            self.get_logger().info(
                f"done: {len(self.arrived)}/{len(self.assigned)} arrived"
            )
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--shape", required=True, choices=list(SHAPES.keys()))
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--tolerance", type=float, default=0.1)
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = GoLite(args.shape, args.timeout, args.tolerance)
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
