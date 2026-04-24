import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class FieldVisualizer(Node):
    def __init__(self):
        super().__init__("field_visualizer")

        self.declare_parameter("corner_tag_ids", [0, 1, 2, 3])
        self.declare_parameter(
            "corner_tag_positions",
            [0.0, 0.0, 2.5, 0.0, 2.5, 2.2, 0.0, 2.2],
        )

        corner_ids = self.get_parameter("corner_tag_ids").value
        pos_flat = self.get_parameter("corner_tag_positions").value

        self.corners = []
        for i, tag_id in enumerate(corner_ids):
            x = pos_flat[i * 2]
            y = pos_flat[i * 2 + 1]
            self.corners.append((tag_id, x, y))

        self.marker_pub = self.create_publisher(
            MarkerArray, "/field/markers", 10
        )

        # Publish at 1 Hz so rviz picks it up on connect
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            f"Field visualizer started — {len(self.corners)} corners"
        )

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Corner point markers
        for i, (tag_id, x, y) in enumerate(self.corners):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "field"
            m.ns = "field_corners"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.02
            m.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
            markers.markers.append(m)

            # Label
            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = "field"
            label.ns = "field_corner_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"tag {tag_id}"
            markers.markers.append(label)

        # Field boundary outline
        boundary = Marker()
        boundary.header.stamp = stamp
        boundary.header.frame_id = "field"
        boundary.ns = "field_boundary"
        boundary.id = 0
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = 0.02
        boundary.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.8)

        for _, x, y in self.corners:
            boundary.points.append(Point(x=x, y=y, z=0.0))
        # Close the loop
        _, x0, y0 = self.corners[0]
        boundary.points.append(Point(x=x0, y=y0, z=0.0))
        markers.markers.append(boundary)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FieldVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
