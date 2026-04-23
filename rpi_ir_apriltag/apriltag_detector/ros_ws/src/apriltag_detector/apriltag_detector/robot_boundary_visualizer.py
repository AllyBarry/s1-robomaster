import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class RobotBoundaryVisualizer(Node):
    def __init__(self):
        super().__init__("robot_boundary_visualizer")

        self.declare_parameter("diameter_m", 0.32)
        self.declare_parameter("height_m", 0.02)
        self.declare_parameter("color_rgba", [1.0, 0.5, 0.0, 0.4])
        self.declare_parameter("frame_id", "field")

        self.diameter = float(self.get_parameter("diameter_m").value)
        self.height = float(self.get_parameter("height_m").value)
        rgba = list(self.get_parameter("color_rgba").value)
        self.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        self.frame_id = self.get_parameter("frame_id").value

        self.marker_pub = self.create_publisher(
            MarkerArray, "/field/robot_boundaries", 10
        )

        self.create_subscription(
            PoseArray, "/field/robot_poses", self._on_poses, 10
        )

        self.get_logger().info(
            f"Robot boundary visualizer started — diameter={self.diameter}m"
        )

    def _on_poses(self, msg: PoseArray):
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        stamp = self.get_clock().now().to_msg()
        for i, pose in enumerate(msg.poses):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.frame_id
            m.ns = "robot_boundaries"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = self.height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = self.diameter
            m.scale.y = self.diameter
            m.scale.z = self.height
            m.color = self.color
            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RobotBoundaryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
