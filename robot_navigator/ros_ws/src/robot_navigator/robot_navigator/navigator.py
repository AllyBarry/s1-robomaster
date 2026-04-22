import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    QuaternionStamped,
    Twist,
    TwistStamped,
)
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class RobotNavigator(Node):
    """
    Per-robot navigator that drives a RoboMaster S1 to a goal pose using
    vision-based localization plus onboard odometry to bridge camera latency.

    Pose sources, in priority order:
      1. /robot_{id}/attitude (QuaternionStamped, from IMU) — yaw
      2. /robot_{id}/vel (TwistStamped, body frame) — linear velocity for dead
         reckoning between camera pose updates
      3. Last commanded cmd_vel — fallback when the onboard topics are silent
         or stale (e.g. not bridged from domain 10)

    Field-frame position comes from /field/robot_{id}/pose (tag camera). Between
    camera updates, position is integrated forward using the best velocity
    available. Fresh camera poses are fused with a complementary filter so
    stale position fixes do not cause snap-back.
    """

    def __init__(self):
        super().__init__("robot_navigator")

        self.declare_parameter("robot_id", 0)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("angular_gain", 1.2)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("angular_tolerance", 0.15)
        self.declare_parameter("align_heading", False)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("camera_pose_timeout", 1.0)
        self.declare_parameter("camera_trust", 0.35)
        # How long an onboard measurement is considered "fresh" before we
        # drop back to integrating the last commanded velocity.
        self.declare_parameter("onboard_sensor_timeout", 0.3)
        # /vel's body frame puts y pointing right (matches cmd_vel wire convention).
        # Our internal integration uses y-left, so flip by default.
        self.declare_parameter("vel_y_flip", True)
        # Offset added to IMU yaw to align it with the field frame. Leave at 0
        # if the robot starts aligned with the field +x axis; otherwise set it
        # once via `ros2 topic echo /field/robot_{id}/pose` vs the IMU reading.
        self.declare_parameter("attitude_yaw_offset", 0.0)
        # Slow learning rate that lets the camera correct residual IMU yaw
        # bias over time. Set to 0.0 to freeze the offset.
        self.declare_parameter("attitude_yaw_bias_trust", 0.02)
        self.declare_parameter("field_frame", "field")
        # Peer-aware reactive repulsion: list of other robot IDs to subscribe
        # to for /field/robot_{pid}/pose. Each peer within avoidance_radius
        # adds a velocity push away from itself, falling off linearly to zero
        # at the radius.
        self.declare_parameter(
            "peer_robot_ids", [],
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("avoidance_radius", 0.4)
        self.declare_parameter("avoidance_gain", 0.5)
        self.declare_parameter("peer_pose_timeout", 1.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.angular_tolerance = float(self.get_parameter("angular_tolerance").value)
        self.align_heading = bool(self.get_parameter("align_heading").value)
        self.control_period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.camera_pose_timeout = float(self.get_parameter("camera_pose_timeout").value)
        self.camera_trust = float(self.get_parameter("camera_trust").value)
        self.onboard_timeout = float(self.get_parameter("onboard_sensor_timeout").value)
        self.vel_y_flip = bool(self.get_parameter("vel_y_flip").value)
        self.attitude_yaw_offset = float(self.get_parameter("attitude_yaw_offset").value)
        self.attitude_bias_trust = float(self.get_parameter("attitude_yaw_bias_trust").value)
        self.field_frame = str(self.get_parameter("field_frame").value)
        peer_ids_raw = self.get_parameter("peer_robot_ids").value or []
        self.peer_ids = [int(v) for v in peer_ids_raw if int(v) != self.robot_id]
        self.avoid_radius = float(self.get_parameter("avoidance_radius").value)
        self.avoid_gain = float(self.get_parameter("avoidance_gain").value)
        self.peer_pose_timeout = float(self.get_parameter("peer_pose_timeout").value)

        pose_topic = f"/field/robot_{self.robot_id}/pose"
        goal_pose_topic = f"/robot_{self.robot_id}/goal_pose"
        waypoint_topic = f"/robot_{self.robot_id}/waypoint"
        vel_topic = f"/robot_{self.robot_id}/vel"
        attitude_topic = f"/robot_{self.robot_id}/attitude"
        cmd_topic = f"/robot_{self.robot_id}/cmd_vel"
        estimate_topic = f"/robot_{self.robot_id}/estimated_pose"
        goal_echo_topic = f"/robot_{self.robot_id}/current_goal"
        marker_topic = f"/robot_{self.robot_id}/goal_marker"

        self.create_subscription(PoseStamped, pose_topic, self._on_camera_pose, 10)
        self.create_subscription(PoseStamped, goal_pose_topic, self._on_goal_pose, 10)
        self.create_subscription(PointStamped, waypoint_topic, self._on_waypoint, 10)
        self.create_subscription(TwistStamped, vel_topic, self._on_vel, 20)
        self.create_subscription(QuaternionStamped, attitude_topic, self._on_attitude, 20)

        # Peer pose cache: {pid: (x, y, monotonic_stamp)}.
        self.peer_positions: dict[int, tuple[float, float, float]] = {}
        for pid in self.peer_ids:
            self.create_subscription(
                PoseStamped,
                f"/field/robot_{pid}/pose",
                lambda msg, i=pid: self._on_peer_pose(i, msg),
                10,
            )

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.estimate_pub = self.create_publisher(PoseStamped, estimate_topic, 10)
        self.goal_echo_pub = self.create_publisher(PoseStamped, goal_echo_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        # Estimated state in field frame.
        self.est_x: float | None = None
        self.est_y: float | None = None
        self.est_yaw: float | None = None
        self.last_integration_time: float | None = None
        self.last_camera_time: float | None = None

        # Onboard-sensor cache.
        self.meas_vx_body: float | None = None
        self.meas_vy_body: float | None = None  # stored in internal (y-left) frame
        self.meas_wz: float | None = None
        self.last_vel_time: float | None = None

        self.meas_imu_yaw: float | None = None
        self.last_att_time: float | None = None

        # Commanded body-frame velocity, used as the fallback when onboard
        # measurements are stale. Stored in internal (y-left) frame.
        self.last_cmd_vx_body = 0.0
        self.last_cmd_vy_body = 0.0
        self.last_cmd_wz = 0.0

        # Goal in field frame: (x, y, yaw or None).
        self.goal: tuple[float, float, float | None] | None = None

        self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info(
            f"robot_navigator[id={self.robot_id}] — "
            f"pose: {pose_topic}, goal: {goal_pose_topic} / {waypoint_topic}, "
            f"onboard: {vel_topic} + {attitude_topic}, cmd: {cmd_topic}"
        )

    # ------------------------------------------------------------------ #
    # Subscribers
    # ------------------------------------------------------------------ #
    def _on_camera_pose(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.field_frame:
            self.get_logger().warn(
                f"Camera pose frame '{msg.header.frame_id}' != "
                f"'{self.field_frame}'; ignoring.",
                throttle_duration_sec=5.0,
            )
            return

        cam_x = msg.pose.position.x
        cam_y = msg.pose.position.y
        cam_yaw = _yaw_from_quat(msg.pose.orientation)
        now = time.monotonic()

        if self.est_x is None:
            self.est_x, self.est_y, self.est_yaw = cam_x, cam_y, cam_yaw
            self.last_integration_time = now
            # If we already have an IMU reading, seed the yaw offset so the
            # IMU-based yaw matches the camera's first fix.
            if self.meas_imu_yaw is not None:
                self.attitude_yaw_offset = _wrap_angle(cam_yaw - self.meas_imu_yaw)
        else:
            self._integrate_to(now)
            a = self.camera_trust
            self.est_x = a * cam_x + (1.0 - a) * self.est_x
            self.est_y = a * cam_y + (1.0 - a) * self.est_y

            # Slow correction of IMU-to-field yaw offset; only meaningful when
            # we have a recent attitude reading to compare against.
            if (self.meas_imu_yaw is not None
                    and self.attitude_bias_trust > 0.0
                    and self._attitude_fresh(now)):
                imu_yaw_in_field = self.meas_imu_yaw + self.attitude_yaw_offset
                err = _wrap_angle(cam_yaw - imu_yaw_in_field)
                self.attitude_yaw_offset = _wrap_angle(
                    self.attitude_yaw_offset + self.attitude_bias_trust * err
                )
            else:
                # Direct complementary fusion on integrated yaw.
                dyaw = _wrap_angle(cam_yaw - self.est_yaw)
                self.est_yaw = _wrap_angle(self.est_yaw + a * dyaw)

        self.last_camera_time = now

    def _on_goal_pose(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.field_frame:
            self.get_logger().warn(
                f"Goal frame '{msg.header.frame_id}' != '{self.field_frame}'; "
                f"set rviz Fixed Frame to '{self.field_frame}'.",
            )
            return
        yaw = _yaw_from_quat(msg.pose.orientation) if self.align_heading else None
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.get_logger().info(
            f"Goal pose set: ({self.goal[0]:.2f}, {self.goal[1]:.2f})"
            + (f" yaw={math.degrees(yaw):.0f}°" if yaw is not None else "")
        )

    def _on_waypoint(self, msg: PointStamped):
        if msg.header.frame_id and msg.header.frame_id != self.field_frame:
            self.get_logger().warn(
                f"Waypoint frame '{msg.header.frame_id}' != '{self.field_frame}'.",
            )
            return
        self.goal = (msg.point.x, msg.point.y, None)
        self.get_logger().info(
            f"Waypoint set: ({self.goal[0]:.2f}, {self.goal[1]:.2f})"
        )

    def _on_vel(self, msg: TwistStamped):
        self.meas_vx_body = msg.twist.linear.x
        # Convert wire-frame (y-right) → internal (y-left) if configured.
        self.meas_vy_body = -msg.twist.linear.y if self.vel_y_flip else msg.twist.linear.y
        self.meas_wz = msg.twist.angular.z
        self.last_vel_time = time.monotonic()

    def _on_attitude(self, msg: QuaternionStamped):
        self.meas_imu_yaw = _yaw_from_quat(msg.quaternion)
        self.last_att_time = time.monotonic()

    def _on_peer_pose(self, pid: int, msg: PoseStamped):
        self.peer_positions[pid] = (
            msg.pose.position.x, msg.pose.position.y, time.monotonic()
        )

    # ------------------------------------------------------------------ #
    # Reactive repulsion
    # ------------------------------------------------------------------ #
    def _repulsion_field_frame(self, now: float) -> tuple[float, float]:
        """Sum of linear-falloff repulsion vectors from fresh peers, field frame."""
        if not self.peer_positions or self.avoid_gain <= 0.0 or self.avoid_radius <= 0.0:
            return 0.0, 0.0
        ax, ay = 0.0, 0.0
        for px, py, stamp in self.peer_positions.values():
            if now - stamp > self.peer_pose_timeout:
                continue
            dx = self.est_x - px
            dy = self.est_y - py
            d = math.hypot(dx, dy)
            if d < 1e-4 or d >= self.avoid_radius:
                continue
            mag = self.avoid_gain * (1.0 - d / self.avoid_radius)
            ax += mag * dx / d
            ay += mag * dy / d
        return ax, ay

    # ------------------------------------------------------------------ #
    # Freshness checks
    # ------------------------------------------------------------------ #
    def _vel_fresh(self, now: float) -> bool:
        return (self.last_vel_time is not None
                and now - self.last_vel_time <= self.onboard_timeout)

    def _attitude_fresh(self, now: float) -> bool:
        return (self.last_att_time is not None
                and now - self.last_att_time <= self.onboard_timeout)

    # ------------------------------------------------------------------ #
    # Dead reckoning
    # ------------------------------------------------------------------ #
    def _integrate_to(self, now: float):
        """Forward-integrate the best available velocity estimate to `now`."""
        if self.est_x is None or self.last_integration_time is None:
            self.last_integration_time = now
            return
        dt = now - self.last_integration_time
        if dt <= 0.0:
            return

        # Select best source for linear body velocity.
        if self._vel_fresh(now):
            vx_body = self.meas_vx_body
            vy_body = self.meas_vy_body
        else:
            vx_body = self.last_cmd_vx_body
            vy_body = self.last_cmd_vy_body

        # Yaw: prefer IMU directly (no integration drift), else integrate.
        if self._attitude_fresh(now):
            self.est_yaw = _wrap_angle(self.meas_imu_yaw + self.attitude_yaw_offset)
            yaw_mid = self.est_yaw
        else:
            wz = self.meas_wz if self._vel_fresh(now) else self.last_cmd_wz
            yaw_mid = self.est_yaw + 0.5 * wz * dt
            self.est_yaw = _wrap_angle(self.est_yaw + wz * dt)

        cos_y = math.cos(yaw_mid)
        sin_y = math.sin(yaw_mid)
        vx_field = cos_y * vx_body - sin_y * vy_body
        vy_field = sin_y * vx_body + cos_y * vy_body
        self.est_x += vx_field * dt
        self.est_y += vy_field * dt

        self.last_integration_time = now

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        now = time.monotonic()

        if self.est_x is None:
            self.cmd_pub.publish(Twist())
            return

        self._integrate_to(now)
        self._publish_estimate()
        self._publish_goal_viz()

        if self.last_camera_time is not None:
            age = now - self.last_camera_time
            if age > self.camera_pose_timeout:
                self._publish_cmd(0.0, 0.0, 0.0)
                self.get_logger().warn(
                    f"Camera pose stale ({age:.2f}s > "
                    f"{self.camera_pose_timeout:.2f}s) — stopping.",
                    throttle_duration_sec=2.0,
                )
                return

        if self.goal is None:
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        gx, gy, gyaw = self.goal
        dx = gx - self.est_x
        dy = gy - self.est_y
        distance = math.hypot(dx, dy)

        at_position = distance < self.goal_tolerance
        heading_err = 0.0
        at_heading = True
        if gyaw is not None:
            heading_err = _wrap_angle(gyaw - self.est_yaw)
            at_heading = abs(heading_err) < self.angular_tolerance

        if at_position and at_heading:
            self._publish_cmd(0.0, 0.0, 0.0)
            self.get_logger().info("Goal reached.")
            self.goal = None
            return

        cos_y = math.cos(self.est_yaw)
        sin_y = math.sin(self.est_yaw)
        dx_body = cos_y * dx + sin_y * dy
        dy_body = -sin_y * dx + cos_y * dy

        if at_position or distance < 1e-6:
            vx_body = 0.0
            vy_body = 0.0
        else:
            speed = min(self.linear_gain * distance, self.max_linear_speed)
            vx_body = speed * (dx_body / distance)
            vy_body = speed * (dy_body / distance)

        # Reactive peer repulsion: push away from any neighbor within radius.
        ax_field, ay_field = self._repulsion_field_frame(now)
        if ax_field != 0.0 or ay_field != 0.0:
            ax_body = cos_y * ax_field + sin_y * ay_field
            ay_body = -sin_y * ax_field + cos_y * ay_field
            vx_body += ax_body
            vy_body += ay_body
            mag = math.hypot(vx_body, vy_body)
            if mag > self.max_linear_speed:
                scale = self.max_linear_speed / mag
                vx_body *= scale
                vy_body *= scale

        wz = 0.0
        if gyaw is not None and not at_heading:
            wz = max(-self.max_angular_speed,
                     min(self.max_angular_speed, self.angular_gain * heading_err))

        self._publish_cmd(vx_body, vy_body, wz)

    # ------------------------------------------------------------------ #
    # Publishers
    # ------------------------------------------------------------------ #
    def _publish_cmd(self, vx_body: float, vy_body: float, wz: float):
        # Cache for the fallback integrator — stored in internal (y-left) frame.
        self.last_cmd_vx_body = vx_body
        self.last_cmd_vy_body = vy_body
        self.last_cmd_wz = wz

        cmd = Twist()
        cmd.linear.x = vx_body
        cmd.linear.y = -vy_body  # robot expects y-right
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _publish_estimate(self):
        if self.est_x is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.field_frame
        msg.pose.position.x = self.est_x
        msg.pose.position.y = self.est_y
        qx, qy, qz, qw = _quat_from_yaw(self.est_yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.estimate_pub.publish(msg)

    def _publish_goal_viz(self):
        now = self.get_clock().now().to_msg()

        if self.goal is None:
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.field_frame
            marker.ns = f"robot_{self.robot_id}_goal"
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return

        gx, gy, gyaw = self.goal

        echo = PoseStamped()
        echo.header.stamp = now
        echo.header.frame_id = self.field_frame
        echo.pose.position.x = gx
        echo.pose.position.y = gy
        yaw_for_pose = gyaw if gyaw is not None else 0.0
        qx, qy, qz, qw = _quat_from_yaw(yaw_for_pose)
        echo.pose.orientation.x = qx
        echo.pose.orientation.y = qy
        echo.pose.orientation.z = qz
        echo.pose.orientation.w = qw
        self.goal_echo_pub.publish(echo)

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.field_frame
        marker.ns = f"robot_{self.robot_id}_goal"
        marker.id = 0
        marker.type = Marker.ARROW if gyaw is not None else Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = echo.pose
        if gyaw is not None:
            marker.scale.x = 0.25
            marker.scale.y = 0.05
            marker.scale.z = 0.05
        else:
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
        marker.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.9)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
