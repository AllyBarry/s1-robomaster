import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Twist


class GoToPoint(Node):
    def __init__(self):
        super().__init__("go_to_point")

        # Parameters
        self.declare_parameter("robot_id", 4)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("goal_tolerance", 0.05)

        self.robot_id = self.get_parameter("robot_id").value
        self.linear_gain = self.get_parameter("linear_gain").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value

        # State
        self.current_pose = None
        self.goal = None

        # Subscribe to robot pose from field_localizer
        pose_topic = f"/field/robot_{self.robot_id}/pose"
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

        # Subscribe to clicked point from rviz "Publish Point" tool
        self.create_subscription(PointStamped, "/clicked_point", self._on_goal, 10)

        # Publish velocity commands per robot
        cmd_topic = f"/robot_{self.robot_id}/cmd_vel"
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"go_to_point started — pose: {pose_topic}, "
            f"cmd: {cmd_topic} — click a point in rviz to set a goal"
        )

    def _on_pose(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def _on_goal(self, msg: PointStamped):
        self.goal = msg.point
        self.get_logger().info(
            f"New goal: ({self.goal.x:.2f}, {self.goal.y:.2f})"
        )

    def _control_loop(self):
        if self.current_pose is None or self.goal is None:
            return

        dx = self.goal.x - self.current_pose.position.x
        dy = self.goal.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        cmd = Twist()

        if distance < self.goal_tolerance:
            # Goal reached — stop and clear
            self.cmd_pub.publish(cmd)
            if self.goal is not None:
                self.get_logger().info("Goal reached!")
                self.goal = None
            return

        # Proportional control, clamped to max speed
        speed = min(self.linear_gain * distance, self.max_linear_speed)
        cmd.linear.x = speed * (dx / distance)
        cmd.linear.y = speed * (dy / distance)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
