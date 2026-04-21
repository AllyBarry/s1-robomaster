import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped


class Follower(Node):
    """
    Publishes a waypoint for `robot_id` that sits `follow_distance` metres
    from `target_robot_id` along the line between them. When the follower
    is at the correct distance the waypoint coincides with its own position
    and the navigator stops; if the follower is too far the waypoint moves
    closer to the target; if too close it moves past the follower away from
    the target.
    """

    def __init__(self):
        super().__init__("follower")

        self.declare_parameter("robot_id", 0)
        self.declare_parameter("target_robot_id", 1)
        self.declare_parameter("follow_distance", 0.5)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("stale_pose_timeout", 1.0)
        self.declare_parameter("field_frame", "field")

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.target_id = int(self.get_parameter("target_robot_id").value)
        self.follow_distance = float(self.get_parameter("follow_distance").value)
        self.stale_timeout = float(self.get_parameter("stale_pose_timeout").value)
        self.field_frame = str(self.get_parameter("field_frame").value)
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)

        if self.robot_id == self.target_id:
            raise RuntimeError(
                f"follower: robot_id and target_robot_id are both {self.robot_id}; "
                "a robot can't follow itself."
            )

        own_pose_topic = f"/field/robot_{self.robot_id}/pose"
        target_pose_topic = f"/field/robot_{self.target_id}/pose"
        waypoint_topic = f"/robot_{self.robot_id}/waypoint"

        self.own_x: float | None = None
        self.own_y: float | None = None
        self.own_time: float | None = None

        self.target_x: float | None = None
        self.target_y: float | None = None
        self.target_time: float | None = None

        self.create_subscription(PoseStamped, own_pose_topic, self._on_own_pose, 10)
        self.create_subscription(PoseStamped, target_pose_topic, self._on_target_pose, 10)
        self.waypoint_pub = self.create_publisher(PointStamped, waypoint_topic, 10)

        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"follower[robot_{self.robot_id} → robot_{self.target_id}] — "
            f"distance={self.follow_distance:.2f}m, "
            f"publishing to {waypoint_topic}"
        )

    def _on_own_pose(self, msg: PoseStamped):
        self.own_x = msg.pose.position.x
        self.own_y = msg.pose.position.y
        self.own_time = time.monotonic()

    def _on_target_pose(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_time = time.monotonic()

    def _tick(self):
        if self.target_x is None or self.own_x is None:
            return

        now = time.monotonic()
        if now - self.target_time > self.stale_timeout:
            self.get_logger().warn(
                f"Target robot_{self.target_id} pose stale; pausing waypoint updates.",
                throttle_duration_sec=2.0,
            )
            return
        if now - self.own_time > self.stale_timeout:
            self.get_logger().warn(
                f"Own (robot_{self.robot_id}) pose stale; pausing waypoint updates.",
                throttle_duration_sec=2.0,
            )
            return

        dx = self.own_x - self.target_x
        dy = self.own_y - self.target_y
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            # Overlapping — arbitrary direction so we don't divide by zero.
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dx / dist, dy / dist

        wp = PointStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = self.field_frame
        wp.point.x = self.target_x + self.follow_distance * ux
        wp.point.y = self.target_y + self.follow_distance * uy
        self.waypoint_pub.publish(wp)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
