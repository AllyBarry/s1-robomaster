import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray


def _line(n: int, spacing: float):
    half = (n - 1) / 2.0
    return [((i - half) * spacing, 0.0) for i in range(n)]


def _ring(n: int, radius: float, start_angle: float = math.pi / 2):
    return [
        (radius * math.cos(start_angle + 2 * math.pi * i / n),
         radius * math.sin(start_angle + 2 * math.pi * i / n))
        for i in range(n)
    ]


def _rotate_translate(points, yaw: float, cx: float, cy: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * x - s * y, cy + s * x + c * y) for (x, y) in points]


class GlobalFeedback(Node):
    """
    Publishes a scalar feedback signal based on how well a group of robots
    matches a target formation in the field frame.

    The signal is `reward = -sum(distances)` — zero when every target has a
    robot on it, more negative as robots drift away. Higher is better.

    Formations:
      * "line"     — N targets equally spaced along a line
      * "triangle" — 3 targets at the vertices of an equilateral triangle
      * "circle"   — N targets on a circle (triangle is the N=3 special case)
      * "custom"   — explicit list of (x, y) pairs via `targets_flat`

    Assignment (how each target's distance is computed):
      * "ordered" — target i is matched to `robot_ids[i]` (fixed assignment)
      * "nearest" — for each target, take the closest robot (lets robots
                    shuffle into whichever position they want; matches the
                    simulator's global_reward_node)
    """

    def __init__(self):
        super().__init__("global_feedback")

        self.declare_parameter("robot_ids", [0, 1, 2])
        self.declare_parameter("formation", "triangle")
        self.declare_parameter("formation_center_x", 0.0)
        self.declare_parameter("formation_center_y", 0.0)
        # Line: spacing between neighbours. Triangle/circle: radius from centre.
        self.declare_parameter("formation_spacing", 0.5)
        self.declare_parameter("formation_yaw", 0.0)
        # Only used when formation == "custom". Flat [x0, y0, x1, y1, ...].
        self.declare_parameter("targets_flat", [0.0])
        self.declare_parameter("assignment", "ordered")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("marker_rate_hz", 1.0)
        self.declare_parameter("stale_pose_timeout", 2.0)
        self.declare_parameter("field_frame", "field")

        self.robot_ids = [int(x) for x in self.get_parameter("robot_ids").value]
        formation = str(self.get_parameter("formation").value)
        cx = float(self.get_parameter("formation_center_x").value)
        cy = float(self.get_parameter("formation_center_y").value)
        spacing = float(self.get_parameter("formation_spacing").value)
        yaw = float(self.get_parameter("formation_yaw").value)
        targets_flat = list(self.get_parameter("targets_flat").value)
        self.assignment = str(self.get_parameter("assignment").value)
        self.field_frame = str(self.get_parameter("field_frame").value)
        self.stale_timeout = float(self.get_parameter("stale_pose_timeout").value)

        if self.assignment not in ("ordered", "nearest"):
            raise ValueError(f"unknown assignment '{self.assignment}'")

        n = len(self.robot_ids)
        if n == 0:
            raise ValueError("robot_ids must be non-empty")

        if formation == "line":
            local = _line(n, spacing)
        elif formation == "triangle":
            if n != 3:
                raise ValueError(f"triangle formation requires 3 robots, got {n}")
            local = _ring(3, spacing)
        elif formation == "circle":
            local = _ring(n, spacing)
        elif formation == "custom":
            if len(targets_flat) != 2 * n:
                raise ValueError(
                    f"custom formation requires {2 * n} values in targets_flat "
                    f"({n} robots × 2 coords), got {len(targets_flat)}"
                )
            local = [
                (float(targets_flat[i]), float(targets_flat[i + 1]))
                for i in range(0, len(targets_flat), 2)
            ]
        else:
            raise ValueError(f"unknown formation '{formation}'")

        self.targets = _rotate_translate(local, yaw, cx, cy)

        self.positions: dict[int, tuple[float, float] | None] = {
            rid: None for rid in self.robot_ids
        }
        self.position_times: dict[int, float] = {rid: 0.0 for rid in self.robot_ids}

        for rid in self.robot_ids:
            self.create_subscription(
                PoseStamped,
                f"/field/robot_{rid}/pose",
                lambda msg, r=rid: self._on_pose(msg, r),
                10,
            )

        self.reward_pub = self.create_publisher(Float32, "/global_reward", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/target_markers", 10)

        pub_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        marker_period = 1.0 / float(self.get_parameter("marker_rate_hz").value)
        self.create_timer(pub_period, self._tick)
        self.create_timer(marker_period, self._publish_markers)

        target_str = ", ".join(f"({t[0]:.2f},{t[1]:.2f})" for t in self.targets)
        self.get_logger().info(
            f"global_feedback — formation={formation} "
            f"assignment={self.assignment} robots={self.robot_ids} "
            f"targets=[{target_str}]"
        )

    def _on_pose(self, msg: PoseStamped, robot_id: int):
        self.positions[robot_id] = (msg.pose.position.x, msg.pose.position.y)
        self.position_times[robot_id] = self._now_sec()

    def _now_sec(self) -> float:
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec * 1e-9

    def _tick(self):
        if any(p is None for p in self.positions.values()):
            return

        now = self._now_sec()
        stale = [
            rid for rid, t in self.position_times.items()
            if now - t > self.stale_timeout
        ]
        if stale:
            self.get_logger().warn(
                f"Pose stale for robot(s) {stale}; skipping reward publish.",
                throttle_duration_sec=2.0,
            )
            return

        pos_ordered = [self.positions[rid] for rid in self.robot_ids]

        total = 0.0
        if self.assignment == "ordered":
            for p, t in zip(pos_ordered, self.targets):
                total += math.hypot(p[0] - t[0], p[1] - t[1])
        else:  # nearest
            for t in self.targets:
                total += min(math.hypot(p[0] - t[0], p[1] - t[1]) for p in pos_ordered)

        msg = Float32()
        msg.data = float(-total)
        self.reward_pub.publish(msg)

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for i, (tx, ty) in enumerate(self.targets):
            label = (f"robot_{self.robot_ids[i]}"
                     if self.assignment == "ordered" else f"T{i}")

            cyl = Marker()
            cyl.header.frame_id = self.field_frame
            cyl.header.stamp = stamp
            cyl.ns = "formation_targets"
            cyl.id = i * 2
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = float(tx)
            cyl.pose.position.y = float(ty)
            cyl.pose.position.z = 0.05
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.2
            cyl.scale.y = 0.2
            cyl.scale.z = 0.10
            cyl.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.8)
            arr.markers.append(cyl)

            txt = Marker()
            txt.header.frame_id = self.field_frame
            txt.header.stamp = stamp
            txt.ns = "formation_targets"
            txt.id = i * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(tx)
            txt.pose.position.y = float(ty)
            txt.pose.position.z = 0.25
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.12
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = label
            arr.markers.append(txt)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
